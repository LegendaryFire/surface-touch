// SPDX-License-Identifier: MIT
// Pen tip pressure. Each mode-11 sub-frame carries six data bits in signal
// slots 7..15 and a tone in slots 0..5 whose position moves with pressure.
// The tone gives a rough value, the bits refine it, and a One Euro filter
// smooths the result.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <vector>

#include "pen_frame.hpp"

namespace pen {
namespace pressure {

// Bits 0..2: which slot of a pair has the signal. Bits 3..5: whether a slot
// has it at all. A soft value's sign is the bit, its size the confidence.
inline constexpr std::size_t kSlotsNeeded = 16;
inline constexpr int kSymbols = 6;
struct SlotPair { std::uint8_t zero, one; };
inline constexpr std::array<SlotPair, 3> kPairSlots = {{{7, 8}, {9, 10}, {11, 12}}};
inline constexpr std::array<std::uint8_t, 3> kOnOffSlots = {13, 14, 15};

// In log2 signal level.
inline constexpr float kSignalGate = 3.72f;  // below this, no data is being sent
inline constexpr float kOnOffBias = 0.47f;   // on/off threshold above the midpoint
inline constexpr float kErasure = 1.0f;      // at or below this, the bit is unknown

struct Symbols {
  std::array<float, kSymbols> soft{};
  bool bit(int k) const { return soft[k] > 0.0f; }
  bool known(int k) const { return std::fabs(soft[k]) > kErasure; }
};

inline float log_level(const Slot& s) { return std::log2(float(s.level) + 1.0f); }

inline std::optional<Symbols> demodulate(const SubFrame& sf) {
  if (sf.mode != kModePressure || sf.first.size() < kSlotsNeeded ||
      sf.second.size() < kSlotsNeeded)
    return std::nullopt;
  Symbols s;
  float separation = 0.0f;
  for (const std::vector<Slot>* slots : {&sf.first, &sf.second}) {
    float on = 0.0f, off = 0.0f;
    for (int p = 0; p < 3; ++p) {
      const float zero = log_level((*slots)[kPairSlots[p].zero]);
      const float one = log_level((*slots)[kPairSlots[p].one]);
      s.soft[p] += one - zero;
      on += std::max(zero, one);
      off += std::min(zero, one);
    }
    on /= 3.0f;
    off /= 3.0f;
    separation += on - off;
    const float threshold = 0.5f * (on + off);
    for (int k = 0; k < 3; ++k)
      s.soft[3 + k] += log_level((*slots)[kOnOffSlots[k]]) - threshold;
  }
  if (separation / 2.0f < kSignalGate) return Symbols{};
  for (int k = 3; k < kSymbols; ++k) s.soft[k] -= kOnOffBias;
  return s;
}

// The harder the press, the closer the tone moves to slot 0. kToneCurve maps
// its position to a rough pressure, 0..255.
inline constexpr int kFirstToneSlot = 0;
inline constexpr int kLastToneSlot = 5;
inline constexpr float kIndexMax = 255.0f;
inline constexpr float kToneFullScale = 0.31f;

struct ToneKnot { float centroid, index; };
inline constexpr std::array<ToneKnot, 28> kToneCurve = {{
    {0.310f, 255.0f}, {0.333f, 233.4f}, {0.913f, 205.8f}, {0.940f, 198.5f},
    {0.960f, 195.6f}, {0.987f, 191.1f}, {1.000f, 188.3f}, {1.020f, 185.7f},
    {1.040f, 182.8f}, {1.077f, 178.2f}, {1.179f, 171.8f}, {1.291f, 166.3f},
    {1.387f, 162.1f}, {1.638f, 152.7f}, {2.000f, 133.2f}, {2.059f, 133.1f},
    {2.250f, 123.9f}, {2.500f, 113.5f}, {2.750f, 105.4f}, {3.000f, 86.0f},
    {3.250f, 68.6f},  {3.500f, 60.5f},  {3.750f, 51.6f},  {4.000f, 35.0f},
    {4.250f, 17.6f},  {4.500f, 11.4f},  {4.750f, 1.6f},   {4.856f, 0.0f},
}};

inline std::optional<float> tone_centroid(const SubFrame& sf) {
  if (sf.first.size() <= std::size_t(kLastToneSlot) ||
      sf.second.size() <= std::size_t(kLastToneSlot))
    return std::nullopt;
  float sum = 0.0f;
  for (const std::vector<Slot>* slots : {&sf.first, &sf.second}) {
    double power = 0.0, moment = 0.0;
    for (int k = kFirstToneSlot; k <= kLastToneSlot; ++k) {
      power += double((*slots)[k].level);
      moment += double(k) * double((*slots)[k].level);
    }
    if (power <= 0.0) return std::nullopt;
    sum += float(moment / power);
  }
  return sum / 2.0f;
}

inline float reference_index(float centroid) {
  if (centroid <= kToneFullScale) return kIndexMax;
  if (centroid >= kToneCurve.back().centroid) return kToneCurve.back().index;
  for (std::size_t i = 1; i < kToneCurve.size(); ++i) {
    const ToneKnot& a = kToneCurve[i - 1];
    const ToneKnot& b = kToneCurve[i];
    if (centroid <= b.centroid) {
      const float t = (centroid - a.centroid) / (b.centroid - a.centroid);
      return std::clamp(a.index + t * (b.index - a.index), 0.0f, kIndexMax);
    }
  }
  return kToneCurve.back().index;
}

// The pressure is a 10-bit code. Absolute sub-frames send its top six bits,
// incremental ones its low six. The decoded code is the one nearest the rough
// value that matches every bit received.
inline constexpr int kCodeBits = 10;
inline constexpr std::uint16_t kCodeMax = (1u << kCodeBits) - 1;

struct KnownBits { std::uint16_t mask = 0, value = 0; };

inline KnownBits received_bits(const Symbols& s, bool absolute) {
  KnownBits b;
  for (int k = 0; k < kSymbols; ++k) {
    if (!s.known(k)) continue;
    const int position = absolute ? 9 - k : (k < 4 ? 5 - k : 1 - (k - 4));
    b.mask |= std::uint16_t(1u << position);
    if (s.bit(k)) b.value |= std::uint16_t(1u << position);
  }
  return b;
}

inline std::uint16_t nearest_code(KnownBits b, float reference) {
  std::uint16_t best = 0;
  float best_distance = 1e9f;
  for (std::uint16_t c = 0; c <= kCodeMax; ++c) {
    if ((c & b.mask) != b.value) continue;
    const float distance = std::fabs(float(c) - reference);
    if (distance < best_distance) { best = c; best_distance = distance; }
  }
  return best;
}

inline constexpr std::uint16_t kPressureMax = 4096;

inline std::uint16_t code_to_pressure(std::uint16_t code) {
  return std::uint16_t((std::uint32_t(code) * kPressureMax + kCodeMax / 2) / kCodeMax);
}

// One Euro filter (Casiez et al., CHI 2012): steady while the pressure is
// held, quick when it changes fast.
inline constexpr float kStepSeconds = 0.01466f;  // one sub-frame
inline constexpr float kMinCutoffHz = 2.0f;
inline constexpr float kBeta = 1.0f;             // cutoff Hz per (full scale / s)
inline constexpr float kRateCutoffHz = 1.0f;

struct OneEuro {
  bool primed = false;
  float value = 0.0f, rate = 0.0f;

  static float alpha(float cutoff_hz, float dt) {
    const float tau = 1.0f / (2.0f * 3.14159265f * cutoff_hz);
    return 1.0f / (1.0f + tau / dt);
  }
  float filter(float x, float dt) {
    if (!primed) {
      primed = true;
      value = x;
      rate = 0.0f;
      return value;
    }
    rate += alpha(kRateCutoffHz, dt) * ((x - value) / dt - rate);
    value += alpha(kMinCutoffHz + kBeta * std::fabs(rate), dt) * (x - value);
    return value;
  }
};

struct Decoder {
  float reference = 0.0f;
  std::uint16_t raw = 0;
  OneEuro smoother;
  std::uint32_t last_step = 0;
  bool have_step = false;

  // True when the sub-frame carried a pressure.
  bool feed(const SubFrame& sf) {
    const auto symbols = demodulate(sf);
    if (!symbols) return false;
    const auto centroid = tone_centroid(sf);
    if (centroid) reference = reference_index(*centroid);
    const KnownBits bits = received_bits(*symbols, sf.absolute_pressure);
    if (bits.mask == 0 && sf.absolute_pressure) return false;
    raw = code_to_pressure(nearest_code(bits, reference * 4.0f + 1.5f));
    const std::uint32_t steps =
        have_step ? std::clamp<std::uint32_t>(sf.timestamp - last_step, 1, 8) : 1;
    last_step = sf.timestamp;
    have_step = true;
    smoother.filter(float(raw) / float(kPressureMax), float(steps) * kStepSeconds);
    return true;
  }
  void lift() {
    smoother = OneEuro{};
    have_step = false;
  }
  std::uint16_t report() const {
    const float p = std::round(smoother.value * float(kPressureMax));
    return std::uint16_t(std::clamp(p, 0.0f, float(kPressureMax)));
  }
};

}  // namespace pressure
}  // namespace pen
