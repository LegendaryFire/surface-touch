// SPDX-License-Identifier: MIT
// surface-heatview: live view of the touchpad's heat map, analysed by heatd's
// own engine. It only reads the node, so it runs alongside heatd, which has
// to be running to put the pad into the heat stream.
#include <SDL3/SDL.h>
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "frame.hpp"
#include "heat_engine.hpp"

namespace {

constexpr int kCols = 26, kRows = 18, kCells = kCols * kRows;
static_assert(kCells == int(heat::kMatrixCells));

struct Options {
  std::string node = "/dev/surface-thc-pad";
  std::string record;  // raw 1000-byte frames
  heat::DeviceConfig cfg;
};

void usage() {
  std::fprintf(stderr,
               "usage: surface-heatview [--node PATH] [--baseline N] [--gate-lo N]\n"
               "                     [--gate N] [--spawn N] [--palm-mm2 N] [--record FILE]\n"
               "  Defaults match heatd's built-in calibration; heatd logs the device's\n"
               "  own values at start (journalctl -u surface-heatd | grep calibration).\n"
               "keys: n cell values  r regions  g gamma  x/y flip  space pause  q quit\n");
}

// What the panel shows for one of the engine's regions.
struct RegionView {
  const touch::Region* g = nullptr;
  long signal = 0;  // sum of baseline - cell
  int peak = 0;
  double cx = 0, cy = 0;
};

std::vector<RegionView> describe(const touch::ContactTracker& tr, const std::uint8_t* cells,
                                 int baseline) {
  const auto& regions = tr.regions();
  std::vector<RegionView> out(regions.size());
  std::vector<double> wx(regions.size()), wy(regions.size());
  std::vector<int> n(regions.size());
  std::vector<double> sx(regions.size()), sy(regions.size());
  for (int i = 0; i < kCells; ++i) {
    const int k = tr.region_of()[std::size_t(i)];
    if (k < 0) continue;
    const int s = std::max(0, baseline - int(cells[i]));
    const int row = i / kCols, col = i % kCols;
    RegionView& v = out[std::size_t(k)];
    v.signal += s;
    v.peak = std::max(v.peak, s);
    wx[std::size_t(k)] += double(s) * col;
    wy[std::size_t(k)] += double(s) * row;
    ++n[std::size_t(k)];
    sx[std::size_t(k)] += col;
    sy[std::size_t(k)] += row;
  }
  for (std::size_t k = 0; k < out.size(); ++k) {
    RegionView& v = out[k];
    v.g = &regions[k];
    v.cx = v.signal > 0 ? wx[k] / double(v.signal) : sx[k] / n[k];
    v.cy = v.signal > 0 ? wy[k] / double(v.signal) : sy[k] / n[k];
  }
  std::sort(out.begin(), out.end(),
            [](const RegionView& a, const RegionView& b) { return a.g->cells > b.g->cells; });
  return out;
}

struct Rgb { std::uint8_t r, g, b; };

// Black, purple, red, orange, pale yellow.
std::array<Rgb, 256> make_ramp() {
  const Rgb stops[] = {{0, 0, 4}, {87, 16, 110}, {188, 55, 84}, {249, 142, 9}, {252, 255, 164}};
  std::array<Rgb, 256> lut{};
  for (int i = 0; i < 256; ++i) {
    const double t = i / 255.0 * 4.0;
    const int k = std::min(3, int(t));
    const double f = t - k;
    auto mix = [&](std::uint8_t a, std::uint8_t b) { return std::uint8_t(a + f * (b - a) + 0.5); };
    lut[i] = {mix(stops[k].r, stops[k + 1].r), mix(stops[k].g, stops[k + 1].g),
              mix(stops[k].b, stops[k + 1].b)};
  }
  return lut;
}

void text(SDL_Renderer* r, float x, float y, float scale, const char* s) {
  SDL_SetRenderScale(r, scale, scale);
  SDL_RenderDebugText(r, x / scale, y / scale, s);
  SDL_SetRenderScale(r, 1.0f, 1.0f);
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto value = [&]() -> const char* {
      if (i + 1 >= argc) { usage(); std::exit(2); }
      return argv[++i];
    };
    if (a == "--node") opt.node = value();
    else if (a == "--baseline") opt.cfg.baseline = std::uint8_t(std::atoi(value()));
    else if (a == "--gate-lo") opt.cfg.gate_lo = std::uint8_t(std::atoi(value()));
    else if (a == "--gate") opt.cfg.gate = std::uint8_t(std::atoi(value()));
    else if (a == "--spawn") opt.cfg.spawn_gate = std::uint8_t(std::atoi(value()));
    else if (a == "--palm-mm2") opt.cfg.palm_mm2_per_core = std::atof(value());
    else if (a == "--record") opt.record = value();
    else { usage(); return a == "-h" || a == "--help" ? 0 : 2; }
  }
  const heat::DeviceConfig& cfg = opt.cfg;
  int fd = open(opt.node.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    std::fprintf(stderr, "surface-heatview: cannot read %s: %s\n", opt.node.c_str(), std::strerror(errno));
    if (errno == EACCES)
      std::fprintf(stderr, "  grant yourself read access until the node is recreated:\n"
                           "    sudo setfacl -m u:$USER:r %s\n", opt.node.c_str());
    return 1;
  }

  std::FILE* rec = nullptr;
  if (!opt.record.empty()) {
    rec = std::fopen(opt.record.c_str(), "wb");
    if (!rec) {
      std::fprintf(stderr, "surface-heatview: cannot write %s: %s\n", opt.record.c_str(), std::strerror(errno));
      return 1;
    }
    std::fprintf(stderr, "surface-heatview: recording every frame to %s\n", opt.record.c_str());
  }

  if (!SDL_Init(SDL_INIT_VIDEO)) {
    std::fprintf(stderr, "surface-heatview: SDL_Init: %s\n", SDL_GetError());
    return 1;
  }
  constexpr int kPanel = 340, kStatus = 28;
  SDL_Window* win = nullptr;
  SDL_Renderer* ren = nullptr;
  if (!SDL_CreateWindowAndRenderer("Surface heat map", kCols * 36 + kPanel, kRows * 36 + kStatus,
                                   SDL_WINDOW_RESIZABLE, &win, &ren)) {
    std::fprintf(stderr, "surface-heatview: window: %s\n", SDL_GetError());
    return 1;
  }
  SDL_SetRenderVSync(ren, 1);

  const auto ramp = make_ramp();
  heat::HeatEngine engine(cfg);
  std::array<std::uint8_t, kCells> cells;
  cells.fill(cfg.baseline);
  std::array<bool, kCells> on{}, palm_cell{};
  std::vector<RegionView> regions;
  std::uint8_t phys = 0;
  bool have = false, paused = false, numbers = false, show_regions = true, gamma = true;
  bool flip_x = false, flip_y = false;
  long frames = 0, invalid = 0, rate_frames = 0;
  double rate = 0;
  using Clock = std::chrono::steady_clock;
  auto rate_t0 = Clock::now(), last_frame = Clock::now() - std::chrono::seconds(10);
  std::uint8_t buf[4096];

  for (bool run = true; run;) {
    for (SDL_Event e; SDL_PollEvent(&e);) {
      if (e.type == SDL_EVENT_QUIT) run = false;
      if (e.type == SDL_EVENT_KEY_DOWN) {
        switch (e.key.key) {
          case SDLK_ESCAPE: case SDLK_Q: run = false; break;
          case SDLK_N: numbers = !numbers; break;
          case SDLK_R: show_regions = !show_regions; break;
          case SDLK_G: gamma = !gamma; break;
          case SDLK_X: flip_x = !flip_x; break;
          case SDLK_Y: flip_y = !flip_y; break;
          case SDLK_SPACE: paused = !paused; break;
          default: break;
        }
      }
    }

    // Read everything pending; the newest frame is shown.
    for (;;) {
      const ssize_t n = read(fd, buf, sizeof buf);
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) break;
      if (n <= 0) {  // the node went away: try again once a second
        close(fd);
        SDL_Delay(1000);
        fd = open(opt.node.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        break;
      }
      const auto f = heat::parse_frame(buf, std::size_t(n));
      if (!f) { ++invalid; continue; }
      if (rec) std::fwrite(buf, 1, heat::kFrameSize, rec);
      ++frames;
      ++rate_frames;
      last_frame = Clock::now();
      if (!paused) {
        engine.process(*f);
        std::memcpy(cells.data(), f->cells, kCells);
        phys = f->phys;
        const touch::ContactTracker& tr = engine.tracker();
        for (int i = 0; i < kCells; ++i) {
          const int k = tr.region_of()[std::size_t(i)];
          on[i] = tr.on()[std::size_t(i)] != 0;
          palm_cell[i] = k >= 0 && tr.regions()[std::size_t(k)].palm;
        }
        regions = describe(tr, cells.data(), cfg.baseline);
        have = true;
      }
    }
    const auto now = Clock::now();
    if (const double dt = std::chrono::duration<double>(now - rate_t0).count(); dt >= 1.0) {
      rate = rate_frames / dt;
      rate_frames = 0;
      rate_t0 = now;
    }

    int W = 0, H = 0;
    SDL_GetRenderOutputSize(ren, &W, &H);
    const float cell = std::max(4.0f, std::min(float(W - kPanel) / kCols, float(H - kStatus) / kRows));
    auto px = [&](int col) { return (flip_x ? kCols - 1 - col : col) * cell; };
    auto py = [&](int row) { return (flip_y ? kRows - 1 - row : row) * cell; };

    SDL_SetRenderDrawColor(ren, 18, 18, 22, 255);
    SDL_RenderClear(ren);
    for (int row = 0; row < kRows; ++row)
      for (int col = 0; col < kCols; ++col) {
        const int i = row * kCols + col;
        double t = std::clamp(double(cfg.baseline - int(cells[i])) / cfg.baseline, 0.0, 1.0);
        if (gamma) t = std::pow(t, 0.5);
        const Rgb c = ramp[std::size_t(t * 255.0 + 0.5)];
        SDL_SetRenderDrawColor(ren, c.r, c.g, c.b, 255);
        const SDL_FRect rc{px(col), py(row), cell - 1, cell - 1};
        SDL_RenderFillRect(ren, &rc);
        if (numbers && cell >= 26) {
          char s[8];
          std::snprintf(s, sizeof s, "%u", cells[i]);
          const bool bright = (int(c.r) + c.g + c.b) > 380;
          SDL_SetRenderDrawColor(ren, bright ? 0 : 230, bright ? 0 : 230, bright ? 0 : 230, 255);
          SDL_RenderDebugText(ren, px(col) + (cell - 8.0f * std::strlen(s)) / 2, py(row) + cell / 2 - 4, s);
        }
      }

    if (show_regions) {
      // Outline the touched cells: cyan for fingers, red for palms.
      for (int row = 0; row < kRows; ++row)
        for (int col = 0; col < kCols; ++col) {
          const int i = row * kCols + col;
          if (!on[i]) continue;
          if (palm_cell[i]) SDL_SetRenderDrawColor(ren, 255, 70, 70, 255);
          else SDL_SetRenderDrawColor(ren, 80, 220, 255, 255);
          const float x0 = px(col), y0 = py(row), x1 = x0 + cell - 1, y1 = y0 + cell - 1;
          const bool up = row > 0 && on[i - kCols], down = row < kRows - 1 && on[i + kCols];
          const bool left = col > 0 && on[i - 1], right = col < kCols - 1 && on[i + 1];
          const bool top = flip_y ? !down : !up, bottom = flip_y ? !up : !down;
          const bool lft = flip_x ? !right : !left, rgt = flip_x ? !left : !right;
          if (top) SDL_RenderLine(ren, x0, y0, x1, y0);
          if (bottom) SDL_RenderLine(ren, x0, y1, x1, y1);
          if (lft) SDL_RenderLine(ren, x0, y0, x0, y1);
          if (rgt) SDL_RenderLine(ren, x1, y0, x1, y1);
          if (cells[i] <= cfg.spawn_gate) {  // core
            const SDL_FRect d{x0 + cell / 2 - 2, y0 + cell / 2 - 2, 4, 4};
            SDL_RenderFillRect(ren, &d);
          }
        }
      for (std::size_t k = 0; k < regions.size(); ++k) {
        const RegionView& r = regions[k];
        char s[24];
        std::snprintf(s, sizeof s, "#%zu", k + 1);
        const float lx = float((flip_x ? kCols - 1 - r.cx : r.cx) * cell + cell / 2 - 8);
        const float ly = float((flip_y ? kRows - 1 - r.cy : r.cy) * cell + cell / 2 - 8);
        SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
        text(ren, lx + 1, ly + 1, 2.0f, s);
        SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
        text(ren, lx, ly, 2.0f, s);
      }
    }

    const float panel_x = kCols * cell + 16;
    float y = 12;
    auto line = [&](float scale, const char* s) { text(ren, panel_x, y, scale, s); y += 10 * scale + 4; };
    char s[160];
    SDL_SetRenderDrawColor(ren, 235, 235, 235, 255);
    line(2.0f, "Surface heat map");
    std::snprintf(s, sizeof s, "%s  %.0f Hz", opt.node.c_str(), rate);
    line(1.5f, s);
    std::snprintf(s, sizeof s, "frames %ld  invalid %ld", frames, invalid);
    line(1.5f, s);
    std::snprintf(s, sizeof s, "state 0x%02X%s%s", phys,
                  (phys & heat::kPhysStateTracked) ? "  tracked" : "",
                  (phys & heat::kPhysStateButton) == heat::kPhysStateButton ? "  PRESSED" : "");
    line(1.5f, s);
    std::snprintf(s, sizeof s, "on <%d  off >=%d  core <=%d", cfg.gate_lo, cfg.gate, cfg.spawn_gate);
    line(1.5f, s);
    std::snprintf(s, sizeof s, "palm >= %.0f mm2 per core", cfg.palm_mm2_per_core);
    line(1.5f, s);
    y += 8;
    std::snprintf(s, sizeof s, "regions: %zu", regions.size());
    line(2.0f, s);
    for (std::size_t k = 0; k < regions.size() && k < 8; ++k) {
      const RegionView& r = regions[k];
      std::snprintf(s, sizeof s, "#%zu cells %d  cores %d%s", k + 1, r.g->cells, r.g->cores,
                    r.g->palm ? "  PALM" : "");
      line(1.5f, s);
      std::snprintf(s, sizeof s, "   %.0f mm2, %.0f per core", r.g->area_mm2,
                    r.g->cores ? r.g->area_mm2 / r.g->cores : 0.0);
      line(1.5f, s);
      std::snprintf(s, sizeof s, "   %dx%d  sum %ld  peak %d", r.g->width, r.g->height, r.signal, r.peak);
      line(1.5f, s);
    }

    const float sy = float(H - kStatus + 6);
    SDL_SetRenderDrawColor(ren, 160, 160, 170, 255);
    const bool stale = std::chrono::duration<double>(now - last_frame).count() > 1.0;
    if (!have || stale)
      text(ren, 8, sy, 1.5f, "no heat frames - heatd selects the heat stream; is it running?");
    else
      text(ren, 8, sy, 1.5f, paused ? "PAUSED (space)  n values  r regions  g gamma  x/y flip  q quit"
                                     : "n values  r regions  g gamma  x/y flip  space pause  q quit");
    SDL_RenderPresent(ren);
  }

  close(fd);
  if (rec) std::fclose(rec);
  SDL_DestroyRenderer(ren);
  SDL_DestroyWindow(win);
  SDL_Quit();
  return 0;
}
