#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>
#include <utility>

namespace audio_core::sequencer {

// ============================================================================
// TrackerTiming: Renoise-Style Granular Musical Timing Engine
// Governs Lines-per-Beat (LPB), Ticks-per-Line (TPL), and Pattern Line-Lengths.
// ============================================================================
struct TrackerTiming {
    double bpm{120.0};                  // Musical Tempo [20.0 .. 400.0]
    uint16_t lpb{4};                    // Lines Per Beat [1 .. 32] (default 4 = 16th notes; 8 = 32nds; 12 = triplets)
    uint16_t tpl{12};                   // Ticks Per Line [1 .. 64] (default 12 ticks per line for micro-timing)
    uint32_t lines_per_pattern{64};     // Pattern Length in Lines (default 64 lines = 4 bars @ 4 LPB)
    uint16_t beats_per_bar{4};          // Time Signature Numerator (default 4/4)

    // ------------------------------------------------------------------------
    // Duration Calculations (Seconds)
    // ------------------------------------------------------------------------
    [[nodiscard]] constexpr double beat_duration_seconds() const noexcept {
        return 60.0 / std::max(1.0, bpm);
    }

    [[nodiscard]] constexpr double line_duration_seconds() const noexcept {
        const double safe_lpb = std::max<double>(1.0, static_cast<double>(lpb));
        return beat_duration_seconds() / safe_lpb;
    }

    [[nodiscard]] constexpr double tick_duration_seconds() const noexcept {
        const double safe_tpl = std::max<double>(1.0, static_cast<double>(tpl));
        return line_duration_seconds() / safe_tpl;
    }

    [[nodiscard]] constexpr double pattern_duration_seconds() const noexcept {
        return line_duration_seconds() * static_cast<double>(lines_per_pattern);
    }

    // ------------------------------------------------------------------------
    // Sample-Accurate Calculations
    // ------------------------------------------------------------------------
    [[nodiscard]] inline double samples_per_beat(double sample_rate) const noexcept {
        return beat_duration_seconds() * sample_rate;
    }

    [[nodiscard]] inline double samples_per_line(double sample_rate) const noexcept {
        return line_duration_seconds() * sample_rate;
    }

    [[nodiscard]] inline double samples_per_tick(double sample_rate) const noexcept {
        return tick_duration_seconds() * sample_rate;
    }

    [[nodiscard]] inline double samples_per_bar(double sample_rate) const noexcept {
        return samples_per_beat(sample_rate) * static_cast<double>(beats_per_bar);
    }

    [[nodiscard]] inline uint64_t samples_for_lines(uint32_t num_lines, double sample_rate) const noexcept {
        return static_cast<uint64_t>(std::round(samples_per_line(sample_rate) * static_cast<double>(num_lines)));
    }

    [[nodiscard]] inline uint64_t samples_for_pattern(double sample_rate) const noexcept {
        return samples_for_lines(lines_per_pattern, sample_rate);
    }

    // ------------------------------------------------------------------------
    // Sample Position to Tracker Coordinates
    // ------------------------------------------------------------------------
    struct TrackerPosition {
        uint32_t pattern_index{0};      // Zero-indexed sequence pattern
        uint32_t line_in_pattern{0};    // Line index within the active pattern [0 .. lines_per_pattern-1]
        uint32_t tick_in_line{0};       // Sub-line micro-timing tick [0 .. tpl-1]
        double tick_fraction{0.0};      // Sub-tick fraction [0.0 .. 1.0)
        double total_lines{0.0};        // Continuous total lines elapsed since timeline origin
        double total_beats{0.0};        // Continuous total beats elapsed since timeline origin
    };

    [[nodiscard]] inline TrackerPosition position_at_sample(uint64_t sample_pos, double sample_rate) const noexcept {
        TrackerPosition pos{};
        const double spl = samples_per_line(sample_rate);
        const double spt = samples_per_tick(sample_rate);
        if (spl <= 1e-4 || spt <= 1e-4) return pos;

        const double total_l = static_cast<double>(sample_pos) / spl;
        pos.total_lines = total_l;
        pos.total_beats = total_l / std::max<double>(1.0, static_cast<double>(lpb));

        const uint32_t pat_len = std::max(1u, lines_per_pattern);
        pos.pattern_index = static_cast<uint32_t>(total_l / static_cast<double>(pat_len));
        
        const double line_within_pat = std::fmod(total_l, static_cast<double>(pat_len));
        pos.line_in_pattern = static_cast<uint32_t>(std::clamp(std::floor(line_within_pat), 0.0, static_cast<double>(pat_len - 1)));

        const double line_frac = line_within_pat - static_cast<double>(pos.line_in_pattern);
        const double total_ticks = line_frac * static_cast<double>(tpl);
        pos.tick_in_line = static_cast<uint32_t>(std::clamp(std::floor(total_ticks), 0.0, static_cast<double>(tpl - 1)));
        pos.tick_fraction = total_ticks - static_cast<double>(pos.tick_in_line);

        return pos;
    }

    // ------------------------------------------------------------------------
    // Conversion Between Musical Bars and Tracker Lines
    // ------------------------------------------------------------------------
    [[nodiscard]] constexpr uint32_t lines_per_bar() const noexcept {
        return static_cast<uint32_t>(lpb) * static_cast<uint32_t>(beats_per_bar);
    }

    [[nodiscard]] constexpr double line_to_bar(double line) const noexcept {
        const double lpb_bar = static_cast<double>(lines_per_bar());
        return (lpb_bar > 0.0) ? (line / lpb_bar) : 0.0;
    }

    [[nodiscard]] constexpr double bar_to_line(double bar) const noexcept {
        return bar * static_cast<double>(lines_per_bar());
    }

    [[nodiscard]] constexpr double line_to_beat(double line) const noexcept {
        const double safe_lpb = std::max<double>(1.0, static_cast<double>(lpb));
        return line / safe_lpb;
    }

    [[nodiscard]] constexpr double beat_to_line(double beat) const noexcept {
        return beat * static_cast<double>(lpb);
    }

    // Delay command in ticks to sample offset
    [[nodiscard]] inline uint32_t delay_ticks_to_samples(uint8_t delay_ticks, double sample_rate) const noexcept {
        const uint8_t clamped_ticks = std::min<uint8_t>(delay_ticks, static_cast<uint8_t>(tpl - 1));
        return static_cast<uint32_t>(std::round(static_cast<double>(clamped_ticks) * samples_per_tick(sample_rate)));
    }
};

} // namespace audio_core::sequencer
