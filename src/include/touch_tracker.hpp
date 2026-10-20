// SPDX-License-Identifier: MIT
// Contact pipeline shared by the pad (heatd) and the digitizer (digid):
// per-cell hysteresis, core-seeded blob segmentation, and Kalman/Hungarian
// tracking with a fixed pool of wire ids.
#pragma once

#include <cstdint>
#include <vector>

namespace touch {

// Grid geometry and the thresholds the pipeline runs on. Raw cells sit at
// baseline when idle and drop below it under a finger.
struct SensorConfig {
  int rows = 0;
  int cols = 0;
  std::uint32_t him_w = 0;  // sensor extent, himetric 0.01 mm
  std::uint32_t him_h = 0;
  std::uint8_t gate_lo = 0;     // a cell turns ON below this
  std::uint8_t gate = 0;        // an ON cell turns OFF at or above this
  std::uint8_t baseline = 0;    // blob weight = baseline - cell
  std::uint8_t spawn_gate = 0;  // ON cells at or below this are cores
  std::uint8_t core_join = 0;   // cores connected through cells at or below
                                // this are one seed (>= spawn_gate)
};

// One segmented blob: weighted raw moments and the equivalent ellipse.
struct Blob {
  double m00 = 0;  // sum(w)
  double m10 = 0;  // sum(w*col)
  double m01 = 0;  // sum(w*row)
  double m20 = 0;  // sum(w*col*col)
  double m02 = 0;  // sum(w*row*row)
  double m11 = 0;  // sum(w*col*row)
  double cx = 0, cy = 0;              // centroid, cell units
  std::uint8_t min_cell = 255;        // strongest (lowest) raw cell
  double ax_major = 0, ax_minor = 0;  // ellipse semi-axes, cell units
  int orient_cdeg = 0;                // orientation, hundredths of a degree
  int ext_a = 0, ext_b = 0;           // full axis lengths, cells (rounded)
  int cells_on = 0;                   // cells in the blob
};

// One axis of a constant-velocity Kalman filter: state [p v], measurement p,
// dt = 1 frame, white-acceleration process noise. The x/y filter has no
// cross-axis terms, so two of these are exactly the 4-state filter.
struct KalmanAxis {
  double p = 0, v = 0;            // position (himetric), velocity (per frame)
  double pp = 0, pv = 0, vv = 0;  // state covariance, symmetric 2x2
};

struct Track {
  std::uint16_t id = 0;     // rolling id, starts at 1
  double x = 0, y = 0;      // position at the last match, himetric
  double vx = 0, vy = 0;    // filter velocity, himetric per frame
  int age = 0;              // frames since spawn
  int miss = 0;             // frames since the last match
  bool alive = false;
  std::uint8_t wire = 0xFF;  // contact id on the wire, fixed for life
  KalmanAxis kx, ky;
};

// A track retired by the last process() call, at its last position.
struct Retired {
  double x = 0, y = 0;
  std::uint8_t wire = 0;
};

class ContactTracker {
 public:
  ContactTracker(const SensorConfig& cfg, int wire_ids);

  // One frame of rows*cols raw cells, row-major.
  void process(const std::uint8_t* cells);

  const std::vector<Blob>& blobs() const { return blobs_; }
  int region_count() const { return regions_; }  // ON-map components
  const std::vector<Track>& tracks() const { return tracks_; }
  const std::vector<Retired>& retired() const { return retired_; }

 private:
  void threshold(const std::uint8_t* cells);
  void segment(const std::uint8_t* cells);
  void track(const std::uint8_t* cells);
  bool lifted(const Track& tr, const std::uint8_t* cells) const;
  int alloc_wire();
  void release_wire(std::uint8_t w);

  SensorConfig cfg_;
  int wire_ids_;
  std::uint32_t wire_free_;
  std::vector<std::uint8_t> on_;  // per-cell hysteresis state
  std::vector<Blob> blobs_;
  int regions_ = 0;
  std::vector<Track> tracks_;
  std::vector<Retired> retired_;
  std::uint16_t next_id_ = 1;
};

}  // namespace touch
