// SPDX-License-Identifier: MIT
// Finds and tracks contacts in a frame of sensor cells. Shared by the
// touchpad and the touchscreen.
#pragma once

#include <cstdint>
#include <vector>

namespace cv { class Mat; }

namespace touch {

// Idle cells sit at baseline and drop below it under a finger.
struct SensorConfig {
  int rows = 0;
  int cols = 0;
  std::uint32_t him_w = 0;  // 0.01 mm
  std::uint32_t him_h = 0;
  std::uint8_t gate_lo = 0;     // a cell turns on below this
  std::uint8_t gate = 0;        // and off again at or above this
  std::uint8_t baseline = 0;
  std::uint8_t spawn_gate = 0;  // on cells at or below this start a contact
  std::uint8_t core_join = 0;   // cores joined through cells this strong are one
  // A region with at least this much area per contact is a palm. Fingers held
  // together still bring one core each. 0 turns it off.
  double palm_mm2_per_core = 0;
};

// A connected area of on cells.
struct Region {
  int cells = 0;
  int width = 0, height = 0;
  int cores = 0;
  double area_mm2 = 0;
  bool palm = false;
};

struct Blob {
  double m00 = 0;  // sum(w)
  double m10 = 0;  // sum(w*col)
  double m01 = 0;  // sum(w*row)
  double m20 = 0;  // sum(w*col*col)
  double m02 = 0;  // sum(w*row*row)
  double m11 = 0;  // sum(w*col*row)
  double cx = 0, cy = 0;              // cells
  std::uint8_t min_cell = 255;
  double ax_major = 0, ax_minor = 0;  // cells
  int orient_cdeg = 0;                // 0.01 degree
  int ext_a = 0, ext_b = 0;
  int cells_on = 0;
  int region = -1;
  bool palm = false;
};

// One axis of a constant-velocity Kalman filter.
struct KalmanAxis {
  double p = 0, v = 0;
  double pp = 0, pv = 0, vv = 0;
};

struct Track {
  std::uint16_t id = 0;
  double x = 0, y = 0;    // 0.01 mm
  double vx = 0, vy = 0;  // 0.01 mm per frame
  int age = 0;   // frames since spawn
  int miss = 0;  // frames since the last match
  bool alive = false;
  std::uint8_t wire = 0xFF;  // the contact id reported to the host
  bool palm = false;         // stays set until the contact lifts
  KalmanAxis kx, ky;
};

struct Retired {
  double x = 0, y = 0;
  std::uint8_t wire = 0;
  bool palm = false;
};

class ContactTracker {
 public:
  ContactTracker(const SensorConfig& cfg, int wire_ids);

  // rows * cols cells, row by row.
  void process(const std::uint8_t* cells);

  const std::vector<Blob>& blobs() const { return blobs_; }
  const std::vector<Region>& regions() const { return regions_; }
  int region_count() const { return int(regions_.size()); }
  const std::vector<Track>& tracks() const { return tracks_; }
  const std::vector<Retired>& retired() const { return retired_; }

  void mark_all_palm();

 private:
  void threshold(const std::uint8_t* cells);
  void segment(const std::uint8_t* cells);
  void classify_regions(const cv::Mat& stats, int count);
  void track(const std::uint8_t* cells);
  bool lifted(const Track& tr, const std::uint8_t* cells) const;
  int alloc_wire();
  void release_wire(std::uint8_t w);

  SensorConfig cfg_;
  int wire_ids_;
  std::uint32_t wire_free_;
  std::vector<std::uint8_t> on_;
  std::vector<Blob> blobs_;
  std::vector<Region> regions_;
  std::vector<Track> tracks_;
  std::vector<Retired> retired_;
  std::uint16_t next_id_ = 1;
};

}  // namespace touch
