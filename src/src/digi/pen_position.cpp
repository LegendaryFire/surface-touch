// SPDX-License-Identifier: MIT
#include "pen_position.hpp"

#include <cmath>
#include <cstdlib>
#include <opencv2/core.hpp>
#include <opencv2/geometry.hpp>
#include <opencv2/imgproc.hpp>

namespace pen {

AxisPosition locate_on_axis(const std::vector<std::uint32_t>& profile) {
  AxisPosition out;
  if (profile.empty()) return out;
  const int n = static_cast<int>(profile.size());

  cv::Mat power(1, n, CV_32F);
  for (int i = 0; i < n; ++i) power.at<float>(0, i) = static_cast<float>(profile[i]);
  cv::Mat amplitude;
  cv::sqrt(power, amplitude);

  double peak_amp = 0.0;
  cv::Point peak;
  cv::minMaxLoc(amplitude, nullptr, &peak_amp, nullptr, &peak);
  if (peak_amp <= 0.0) return out;

  const double threshold = kRunThreshold * peak_amp;
  cv::Mat labels;
  cv::connectedComponents(amplitude >= threshold, labels, 4, CV_32S);
  const cv::Mat run = labels == labels.at<int>(peak);

  cv::Mat weight = amplitude - threshold;
  weight.setTo(0, ~run);
  const cv::Moments m = cv::moments(weight, false);
  if (m.m00 <= 0.0) return out;

  cv::Mat run_cells;
  cv::findNonZero(run, run_cells);
  out.valid = true;
  out.cell = m.m10 / m.m00;
  out.peak = peak.x;
  out.first = run_cells.at<cv::Point>(0).x;
  out.last = run_cells.at<cv::Point>(int(run_cells.total()) - 1).x;
  return out;
}

PenPosition locate_pen(const Profile& p) {
  PenPosition pos;
  if (!p.full()) return pos;
  pos.col = locate_on_axis(p.col);
  pos.row = locate_on_axis(p.row);
  return pos;
}

bool agrees_with_refined_cell(const PenPosition& pos, const Profile& p) {
  return pos.valid() &&
         std::abs(std::lround(pos.col.cell) - long(p.refined_col)) <= 1 &&
         std::abs(std::lround(pos.row.cell) - long(p.refined_row)) <= 1;
}

}  // namespace pen
