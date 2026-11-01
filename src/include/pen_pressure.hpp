// SPDX-License-Identifier: MIT
// Pen tip pressure from the mode-11 sub-frames.
//
// The pen sends its pressure as six data symbols in signal slots 7..15 of
// each mode-11 sub-frame, and as an analog tone in slots 0..5 whose position
// moves with pressure. Decoding runs in five steps:
//
//   1. symbols    log-ratio detection on the slots' signal levels, summed
//                 over the sub-frame's two slot arrays
//   2. reference  a coarse index from the tone's power-weighted centroid
//   3. code       the 10-bit code that agrees with every received bit and
//                 lies nearest the reference
//   4. pressure   the code scaled onto the descriptor's TipPressure range
//   5. smoothing  One Euro filter (Casiez, Roussel, Vogel; CHI 2012)
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

// --- 1. data symbols ---------------------------------------------------------
//
// Symbols 0..2 say which slot of a pair carries the signal; symbols 3..5 say
// whether a single slot carries it. Every quantity is a log2 signal level,
// so a symbol's soft value is a log-likelihood-style ratio: its sign is the
// bit and its magnitude the confidence. The on/off threshold comes from the
// same sub-frame: the midpoint between the pairs' "on" and "off" levels.

inline constexpr std::size_t kSlotsNeeded = 16;
inline constexpr int kSymbols = 6;
struct SlotPair { std::uint8_t zero, one; };
inline constexpr std::array<SlotPair, 3> kPairSlots = {{{7, 8}, {9, 10}, {11, 12}}};
inline constexpr std::array<std::uint8_t, 3> kOnOffSlots = {13, 14, 15};

// log2 units, over both arrays.
inline constexpr float kSignalGate = 3.72f;  // pair separation below this: no data
inline constexpr float kOnOffBias = 0.47f;   // on/off threshold above the midpoint
inline constexpr float kErasure = 1.0f;      // |soft| at or below this: bit unknown

struct Symbols {
  std::array<float, kSymbols> soft{};
  bool bit(int k) const { return soft[k] > 0.0f; }
  bool known(int k) const { return std::fabs(soft[k]) > kErasure; }
};

inline float log_level(const Slot& s) { return std::log2(float(s.level) + 1.0f); }

// Nothing when the sub-frame is not a full pressure sub-frame. When its pairs
// do not separate there is no data on the wire, and every symbol is unknown.
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

// --- 2. coarse reference -----------------------------------------------------
//
// The tone's position: the power-weighted mean slot index over tone slots
// 0..5, averaged over the two arrays. Harder pressure moves the tone towards
// slot 0. kToneCurve maps that position to the 8-bit pressure index; below
// kToneFullScale the pen reports full scale.

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

// Nothing when either array carries no tone.
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

// Piecewise-linear lookup, held at the ends.
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

// --- 3. pressure code --------------------------------------------------------
//
// Absolute sub-frames carry the code's top six bits (symbols 0..5 = bits
// 9..4). Incremental ones carry its low six bits (symbols 0..3 = bits 5..2,
// the low nibble of the 8-bit index; symbols 4..5 = bits 1..0). Either way
// the decoded code is the one that matches every known bit and lies nearest
// the reference; for an incremental sub-frame that is the usual modular
// unwrap of the low bits against the reference, and with no bits known it is
// the reference itself.

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

// reference: in code units. Ties go to the lower code.
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

// --- 4. pressure -------------------------------------------------------------

// The pen collection's TipPressure logical maximum.
inline constexpr std::uint16_t kPressureMax = 4096;

inline std::uint16_t code_to_pressure(std::uint16_t code) {
  return std::uint16_t((std::uint32_t(code) * kPressureMax + kCodeMax / 2) / kCodeMax);
}

// --- 5. smoothing ------------------------------------------------------------
//
// One Euro filter on pressure normalised to 0..1: a low-pass whose cutoff
// rises with the rate of change, so a held pressure is steady and a fast
// press is not lagged. Time comes from the sub-frame counter.

inline constexpr float kStepSeconds = 0.01466f;  // one sub-frame counter step
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

// Per-pen decoder state. lift() restarts the smoothing.
struct Decoder {
  float reference = 0.0f;  // last coarse index; held while the tone is absent
  std::uint16_t raw = 0;   // last decoded pressure
  OneEuro smoother;
  std::uint32_t last_step = 0;
  bool have_step = false;

  // Feeds one released sub-frame; returns true when it carried a pressure.
  bool feed(const SubFrame& sf) {
    const auto symbols = demodulate(sf);
    if (!symbols) return false;
    const auto centroid = tone_centroid(sf);
    if (centroid) reference = reference_index(*centroid);
    const KnownBits bits = received_bits(*symbols, sf.absolute_pressure);
    // An absolute sub-frame is self-contained: without bits it says nothing.
    // An incremental one without bits falls back on the reference.
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
