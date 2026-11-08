// SPDX-License-Identifier: MIT
#include "touch_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>

namespace touch {

namespace {
constexpr int kDropFrames = 12;
constexpr double kRadToCdeg = 180.0 / 3.14159265358979323846 * 100.0;
inline double sq(double v) { return v * v; }

// Kalman noise in cell pitches, so it scales with the grid.
constexpr double kMeasStdCells = 0.3;     // centroid jitter (measurement noise)
constexpr double kAccelStdCells = 0.5;    // per-frame acceleration (process noise)
constexpr double kInitVelStdCells = 2.0;  // velocity uncertainty at touch-down
// A pair is accepted inside the 99% Mahalanobis bound for 2 DOF, or within
// 400 himetric of the prediction.
constexpr double kGateChi2 = 9.21;
constexpr double kGateFloorD2 = 160000.0;
// Cost of a gated-out pair. Larger than any sum of real costs, so the solver
// always prefers more valid pairs over fewer cheap ones.
constexpr double kForbid = 1e12;

// Min-cost assignment (Hungarian / Kuhn-Munkres, O(n^2 m)), n rows <= m cols.
// cost is row-major n x m. Returns the column assigned to each row.
std::vector<int> hungarian_rows_le_cols(const std::vector<double>& cost, int n, int m) {
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> u(std::size_t(n + 1), 0), v(std::size_t(m + 1), 0);
  std::vector<int> p(std::size_t(m + 1), 0), way(std::size_t(m + 1), 0);
  std::vector<double> minv(std::size_t(m + 1));
  std::vector<char> used(std::size_t(m + 1));
  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::fill(minv.begin(), minv.end(), inf);
    std::fill(used.begin(), used.end(), 0);
    do {
      used[j0] = 1;
      const int i0 = p[j0];
      double delta = inf;
      int j1 = 0;
      for (int j = 1; j <= m; ++j) {
        if (used[j]) continue;
        const double cur = cost[std::size_t(i0 - 1) * m + (j - 1)] - u[i0] - v[j];
        if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
        if (minv[j] < delta) { delta = minv[j]; j1 = j; }
      }
      for (int j = 0; j <= m; ++j) {
        if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
        else         { minv[j] -= delta; }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do { const int j1 = way[j0]; p[j0] = p[j1]; j0 = j1; } while (j0);
  }
  std::vector<int> col(std::size_t(n), -1);
  for (int j = 1; j <= m; ++j)
    if (p[j]) col[std::size_t(p[j] - 1)] = j - 1;
  return col;
}

// Any shape: returns, for each of the n rows, its column or -1.
std::vector<int> assign(const std::vector<double>& cost, int n, int m) {
  if (n == 0 || m == 0) return std::vector<int>(std::size_t(n), -1);
  if (n <= m) return hungarian_rows_le_cols(cost, n, m);
  std::vector<double> t(cost.size());
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < m; ++j)
      t[std::size_t(j) * n + i] = cost[std::size_t(i) * m + j];
  const std::vector<int> colToRow = hungarian_rows_le_cols(t, m, n);
  std::vector<int> rowToCol(std::size_t(n), -1);
  for (int j = 0; j < m; ++j) rowToCol[std::size_t(colToRow[std::size_t(j)])] = j;
  return rowToCol;
}

// Kalman filter, one axis (see KalmanAxis). qa = acceleration variance
// (Q = qa * [1/4 1/2; 1/2 1]), r = measurement variance, pv0 = initial
// velocity variance.
void kf_init(KalmanAxis& k, double z, double r, double pv0) {
  k.p = z;
  k.v = 0;
  k.pp = r;
  k.pv = 0;
  k.vv = pv0;
}

// x = A x, P = A P A^T + Q, with A = [1 1; 0 1].
void kf_predict(KalmanAxis& k, double qa) {
  k.p += k.v;
  k.pp += 2 * k.pv + k.vv + qa / 4;
  k.pv += k.vv + qa / 2;
  k.vv += qa;
}

// H = [1 0]: S = pp + r, K = [pp pv] / S, x += K (z - p), P -= K H P.
void kf_correct(KalmanAxis& k, double z, double r) {
  const double s = k.pp + r;
  const double k0 = k.pp / s, k1 = k.pv / s;
  const double y = z - k.p;
  k.p += k0 * y;
  k.v += k1 * y;
  k.vv -= k1 * k.pv;  // before pv/pp change: both read the prior
  k.pv -= k0 * k.pv;
  k.pp -= k0 * k.pp;
}
}  // namespace

ContactTracker::ContactTracker(const SensorConfig& cfg, int wire_ids)
    : cfg_(cfg),
      wire_ids_(wire_ids),
      wire_free_((1u << wire_ids) - 1),
      on_(std::size_t(cfg.rows * cfg.cols), 0) {}

void ContactTracker::process(const std::uint8_t* cells) {
  threshold(cells);
  segment(cells);
  track(cells);
}

// Per-cell Schmitt trigger: enter below gate_lo, release at gate.
void ContactTracker::threshold(const std::uint8_t* cells) {
  const int n = cfg_.rows * cfg_.cols;
  for (int i = 0; i < n; ++i) {
    const std::uint8_t v = cells[i];
    on_[i] = (v < cfg_.gate_lo) | (on_[i] & (v < cfg_.gate));
  }
}

// Seeded region growing. Seeds are 4-connected components of ON cells at or
// below core_join that contain a core cell (<= spawn_gate), so cores joined
// by a shallow saddle are one seed. Seeds grow through ON cells by
// 4-connected multi-source BFS, so touches sharing one ON region stay
// separate. ON regions with no core are dropped.
void ContactTracker::segment(const std::uint8_t* cells) {
  blobs_.clear();
  const int R = cfg_.rows, C = cfg_.cols, N = R * C;
  const cv::Mat onMap(R, C, CV_8UC1, on_.data());
  const cv::Mat cellsMap(R, C, CV_8UC1, const_cast<std::uint8_t*>(cells));

  // ON regions, labelled 1..n, with their areas and bounding boxes.
  cv::Mat regionLab, regionStats, regionCentroids;
  const int nRegion = cv::connectedComponentsWithStats(onMap, regionLab, regionStats,
                                                      regionCentroids, 4, CV_32S) - 1;
  const int* regionp = regionLab.ptr<int>(0);

  // Seeds, relabelled 1..nCore in scan order.
  cv::Mat lab;
  const cv::Mat joinMap = onMap & (cellsMap <= cfg_.core_join);
  const int nJoin = cv::connectedComponents(joinMap, lab, 4, CV_32S) - 1;
  int* labp = lab.ptr<int>(0);
  std::vector<int> remap(std::size_t(nJoin + 1), 0);
  for (int i = 0; i < N; ++i)
    if (labp[i] != 0 && cells[i] <= cfg_.spawn_gate) remap[std::size_t(labp[i])] = 1;
  int nCore = 0;
  for (int l = 1; l <= nJoin; ++l)
    if (remap[std::size_t(l)]) remap[std::size_t(l)] = ++nCore;
  for (int i = 0; i < N; ++i) labp[i] = remap[std::size_t(labp[i])];

  // Grow seeds through ON cells. Each cell is enqueued at most once.
  {
    std::vector<int> q;
    q.reserve(std::size_t(N));
    for (int i = 0; i < N; ++i)
      if (labp[i] != 0) q.push_back(i);
    for (std::size_t head = 0; head < q.size(); ++head) {
      const int i = q[head];
      const int lbl = labp[i];
      const int cr = i / C, cc = i % C;
      auto visit = [&](int j) {
        if (on_[j] && labp[j] == 0) { labp[j] = lbl; q.push_back(j); }
      };
      if (cc + 1 < C) visit(i + 1);
      if (cc > 0)     visit(i - 1);
      if (cr + 1 < R) visit(i + C);
      if (cr > 0)     visit(i - C);
    }
  }

  // One pass: weighted raw moments, cell count and min value per blob.
  // Weight = depth below baseline, floored at 1.
  struct Acc {
    double m00 = 0, m10 = 0, m01 = 0, m20 = 0, m02 = 0, m11 = 0;
    int n = 0;
    std::uint8_t lo = 255;
    int region = -1;
  };
  std::vector<Acc> acc{std::size_t(nCore)};
  for (int i = 0; i < N; ++i) {
    const int l = labp[i];
    if (l == 0) continue;
    const std::uint8_t v = cells[i];
    const double w = cfg_.baseline > v ? double(float(cfg_.baseline) - float(v)) : 1.0;
    const double x = i % C, y = i / C;
    Acc& a = acc[std::size_t(l - 1)];
    a.m00 += w;
    a.m10 += w * x;
    a.m01 += w * y;
    a.m20 += w * x * x;
    a.m02 += w * y * y;
    a.m11 += w * x * y;
    a.n++;
    a.lo = std::min(a.lo, v);
    a.region = regionp[i] - 1;
  }

  // Centroid and equivalent ellipse from the second central moments.
  blobs_.reserve(acc.size());
  for (const Acc& a : acc) {
    if (a.m00 == 0) continue;
    Blob b{};
    b.m00 = a.m00;
    b.m10 = a.m10;
    b.m01 = a.m01;
    b.m20 = a.m20;
    b.m02 = a.m02;
    b.m11 = a.m11;
    b.cells_on = a.n;
    b.min_cell = a.lo;
    b.region = a.region;
    const double inv = 1.0 / a.m00;
    b.cx = a.m10 * inv;
    b.cy = a.m01 * inv;
    const double sxx = a.m20 * inv - b.cx * b.cx;
    const double syy = a.m02 * inv - b.cy * b.cy;
    const double sxy = a.m11 * inv - b.cx * b.cy;
    const double half = 0.5 * (sxx + syy);
    const double d = std::sqrt(0.25 * sq(sxx - syy) + sxy * sxy);
    const double l1 = half + d, l2 = half - d;
    b.ax_major = l1 > 0 ? std::sqrt(l1) : 0;
    b.ax_minor = l2 > 0 ? std::sqrt(l2) : 0;
    b.orient_cdeg = int(std::atan2(2.0 * sxy, sxx - syy) * kRadToCdeg);
    b.ext_a = int(std::lround(2.0 * b.ax_major));
    b.ext_b = int(std::lround(2.0 * b.ax_minor));
    blobs_.push_back(b);
  }

  classify_regions(regionStats, nRegion);
}

// Region sizes, core counts and the palm decision; each blob inherits its
// region's decision.
void ContactTracker::classify_regions(const cv::Mat& stats, int count) {
  const double cell_mm2 = (cfg_.him_w / 100.0 / cfg_.cols) * (cfg_.him_h / 100.0 / cfg_.rows);
  regions_.assign(std::size_t(count), Region{});
  for (int k = 0; k < count; ++k) {
    Region& g = regions_[std::size_t(k)];
    g.cells = stats.at<int>(k + 1, cv::CC_STAT_AREA);  // label 0 is the background
    g.width = stats.at<int>(k + 1, cv::CC_STAT_WIDTH);
    g.height = stats.at<int>(k + 1, cv::CC_STAT_HEIGHT);
    g.area_mm2 = g.cells * cell_mm2;
  }
  for (const Blob& b : blobs_) ++regions_[std::size_t(b.region)].cores;
  for (Region& g : regions_)
    g.palm = cfg_.palm_mm2_per_core > 0 && g.cores > 0 &&
             g.area_mm2 / g.cores >= cfg_.palm_mm2_per_core;
  for (Blob& b : blobs_) b.palm = regions_[std::size_t(b.region)].palm;
}

// A coasting track has lifted when the strongest cell in the 3x3 under both
// its last matched position and its predicted position is weaker than
// lift_edge, halfway between the core edge (spawn_gate) and the ON-entry edge
// (gate_lo).
bool ContactTracker::lifted(const Track& tr, const std::uint8_t* cells) const {
  const double sx = double(cfg_.him_w) / cfg_.cols;
  const double sy = double(cfg_.him_h) / cfg_.rows;
  const int lift_edge = (int(cfg_.spawn_gate) + int(cfg_.gate_lo)) / 2;
  auto strongest = [&](double x, double y) {
    const int c = int(std::lround(x / sx - 0.5));
    const int r = int(std::lround(y / sy - 0.5));
    int lo = 255;
    for (int rr = std::max(r - 1, 0); rr <= std::min(r + 1, cfg_.rows - 1); ++rr)
      for (int cc = std::max(c - 1, 0); cc <= std::min(c + 1, cfg_.cols - 1); ++cc)
        lo = std::min<int>(lo, cells[rr * cfg_.cols + cc]);
    return lo;
  };
  return std::min(strongest(tr.x, tr.y), strongest(tr.kx.p, tr.ky.p)) > lift_edge;
}

// Optimal (Hungarian) association of Kalman-predicted tracks to blobs.
// Unmatched tracks coast and retire after kDropFrames misses, or at once
// when lifted. Unmatched blobs spawn new tracks. A retired track's wire id is
// freed only after spawning, so it is never reused in the same frame.
void ContactTracker::track(const std::uint8_t* cells) {
  retired_.clear();

  const double sx = double(cfg_.him_w) / cfg_.cols;
  const double sy = double(cfg_.him_h) / cfg_.rows;
  const double pitch = 0.5 * (sx + sy);
  const double qa = sq(kAccelStdCells * pitch);
  const double r = sq(kMeasStdCells * pitch);
  const double pv0 = sq(kInitVelStdCells * pitch);

  // Blob positions in himetric, cell centres at index + 0.5.
  struct Pos { double x, y; };
  std::vector<Pos> pos;
  pos.reserve(blobs_.size());
  for (const Blob& b : blobs_)
    pos.push_back({(b.cx + 0.5) * sx, (b.cy + 0.5) * sy});

  for (auto& tr : tracks_) {
    tr.age++;
    kf_predict(tr.kx, qa);
    kf_predict(tr.ky, qa);
  }

  // Cost = squared distance to the prediction; gated-out pairs get kForbid.
  // The gate uses the innovation covariance S = P_xy + R, diagonal because
  // the axes are independent.
  const int nT = int(tracks_.size()), nB = int(pos.size());
  std::vector<double> cost(std::size_t(nT) * nB, kForbid);
  for (int t = 0; t < nT; ++t) {
    const Track& tr = tracks_[std::size_t(t)];
    const double s00 = tr.kx.pp + r;
    const double s11 = tr.ky.pp + r;
    for (int i = 0; i < nB; ++i) {
      const double dx = pos[std::size_t(i)].x - tr.kx.p;
      const double dy = pos[std::size_t(i)].y - tr.ky.p;
      const double d2 = dx * dx + dy * dy;
      const double maha = dx * dx / s00 + dy * dy / s11;
      if (maha <= kGateChi2 || d2 < kGateFloorD2)
        cost[std::size_t(t) * nB + i] = d2;
    }
  }

  const std::vector<int> match = assign(cost, nT, nB);
  std::vector<char> blob_matched(std::size_t(nB), 0);
  for (int t = 0; t < nT; ++t) {
    Track& tr = tracks_[std::size_t(t)];
    const int i = match[std::size_t(t)];
    if (i >= 0 && cost[std::size_t(t) * nB + i] < kForbid) {
      kf_correct(tr.kx, pos[std::size_t(i)].x, r);
      kf_correct(tr.ky, pos[std::size_t(i)].y, r);
      // Position updates only on a match; a coasting track holds still.
      tr.x = tr.kx.p;
      tr.y = tr.ky.p;
      tr.miss = 0;
      tr.palm = tr.palm || blobs_[std::size_t(i)].palm;
      blob_matched[std::size_t(i)] = 1;
    } else if (++tr.miss >= kDropFrames || lifted(tr, cells)) {
      tr.alive = false;
    }
    // Corrected velocity on a match, the predicted prior while coasting.
    tr.vx = tr.kx.v;
    tr.vy = tr.ky.v;
  }

  for (const auto& tr : tracks_)
    if (!tr.alive) retired_.push_back(Retired{tr.x, tr.y, tr.wire, tr.palm});
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [](const Track& t) { return !t.alive; }),
                tracks_.end());

  for (int i = 0; i < nB; ++i) {
    if (blob_matched[std::size_t(i)]) continue;
    const int w = alloc_wire();
    if (w < 0) continue;
    Track tr;
    tr.id = next_id_++;
    tr.wire = static_cast<std::uint8_t>(w);
    tr.x = pos[std::size_t(i)].x;
    tr.y = pos[std::size_t(i)].y;
    tr.alive = true;
    tr.palm = blobs_[std::size_t(i)].palm;
    kf_init(tr.kx, tr.x, r, pv0);
    kf_init(tr.ky, tr.y, r, pv0);
    tracks_.push_back(tr);
  }

  for (const Retired& d : retired_) release_wire(d.wire);
}

void ContactTracker::mark_all_palm() {
  for (Track& tr : tracks_) tr.palm = true;
}

// Smallest free id at spawn, released at retirement.
int ContactTracker::alloc_wire() {
  for (int w = 0; w < wire_ids_; ++w)
    if (wire_free_ & (1u << w)) {
      wire_free_ &= ~(1u << w);
      return w;
    }
  return -1;
}

void ContactTracker::release_wire(std::uint8_t w) {
  if (w < wire_ids_) wire_free_ |= (1u << w);
}

}  // namespace touch
