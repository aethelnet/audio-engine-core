#pragma once

#include <cstdint>
#include <atomic>
#include <cmath>
#include <algorithm>

namespace audio_core::clock {

enum class ClockAuthority : uint8_t {
    Master = 0,     // Sovereign internal clock. Dictates BPM and beat phase to Link. Rejects external overrides.
    Follower,       // Follows external Link clock (via smooth PLL / adaptive adjustment).
    Isolated,       // Internal clock only, Ableton Link completely detached.
    MidiClockSlave, // Slaved to incoming MIDI 24 PPQN Beat Clock + SPP
    MtcSlave        // Slaved to incoming MIDI Time Code (SMPTE linear frames)
};

struct TimeSignature {
    uint16_t numerator{4};
    uint16_t denominator{4};
};

struct MusicalPosition {
    uint64_t sample_position{0};
    double bpm{120.0};
    double total_beats{0.0};
    uint32_t bar_index{0};
    uint32_t beat_within_bar{0};
    double beat_progress{0.0}; // [0.0, 1.0) phase within current beat
    double bar_progress{0.0};  // [0.0, 1.0) phase within current bar
    bool is_playing{false};
};

struct BlockBoundaryEvents {
    bool has_beat_boundary{false};
    uint32_t beat_sample_offset{0}; // Sample offset within audio block [0..frames-1]
    bool has_bar_boundary{false};
    uint32_t bar_sample_offset{0};  // Sample offset within audio block [0..frames-1]
};

// ============================================================================
// TimelineClock: Sample-Accurate Master Clock & Musical Transport Authority
// Governs sample rate, BPM, beat grid, bar boundaries, and quantum alignment
// ============================================================================
class TimelineClock {
public:
    explicit TimelineClock(uint32_t sample_rate = 48000, double initial_bpm = 120.0)
        : m_sample_rate(sample_rate), m_bpm(initial_bpm) {}

    [[nodiscard]] ClockAuthority authority() const noexcept {
        return m_authority.load(std::memory_order_relaxed);
    }
    void set_authority(ClockAuthority authority) noexcept {
        m_authority.store(authority, std::memory_order_relaxed);
    }

    [[nodiscard]] uint32_t sample_rate() const noexcept {
        return m_sample_rate.load(std::memory_order_relaxed);
    }
    void set_sample_rate(uint32_t sample_rate) noexcept {
        if (sample_rate > 0) {
            m_sample_rate.store(sample_rate, std::memory_order_relaxed);
        }
    }

    [[nodiscard]] double bpm() const noexcept {
        return m_bpm.load(std::memory_order_relaxed);
    }
    void set_bpm(double bpm) noexcept {
        if (bpm >= 20.0 && bpm <= 400.0) {
            m_bpm.store(bpm, std::memory_order_relaxed);
        }
    }

    [[nodiscard]] TimeSignature time_signature() const noexcept {
        return TimeSignature{
            m_time_sig_num.load(std::memory_order_relaxed),
            m_time_sig_den.load(std::memory_order_relaxed)
        };
    }
    void set_time_signature(uint16_t num, uint16_t den) noexcept {
        if (num > 0 && den > 0) {
            m_time_sig_num.store(num, std::memory_order_relaxed);
            m_time_sig_den.store(den, std::memory_order_relaxed);
        }
    }

    [[nodiscard]] bool is_playing() const noexcept {
        return m_is_playing.load(std::memory_order_relaxed);
    }
    void set_playing(bool playing) noexcept {
        m_is_playing.store(playing, std::memory_order_relaxed);
    }

    [[nodiscard]] uint64_t sample_position() const noexcept {
        return m_sample_position.load(std::memory_order_relaxed);
    }
    void set_sample_position(uint64_t pos) noexcept {
        m_sample_position.store(pos, std::memory_order_relaxed);
    }

    // Timeline Scrubbing & Discontinuous Seek Architecture
    [[nodiscard]] bool is_scrubbing() const noexcept {
        return m_is_scrubbing.load(std::memory_order_relaxed);
    }
    [[nodiscard]] uint64_t seek_generation() const noexcept {
        return m_seek_generation.load(std::memory_order_relaxed);
    }
    [[nodiscard]] uint64_t last_seek_sample() const noexcept {
        return m_last_seek_sample.load(std::memory_order_relaxed);
    }
    [[nodiscard]] double scrub_velocity() const noexcept {
        return m_scrub_velocity.load(std::memory_order_relaxed);
    }
    void set_scrub_velocity(double vel) noexcept {
        m_scrub_velocity.store(vel, std::memory_order_relaxed);
    }

    void seek(uint64_t target_sample) noexcept {
        m_sample_position.store(target_sample, std::memory_order_release);
        m_last_seek_sample.store(target_sample, std::memory_order_release);
        m_seek_generation.fetch_add(1, std::memory_order_release);
    }

    void start_scrub(uint64_t target_sample, double velocity = 1.0) noexcept {
        m_is_scrubbing.store(true, std::memory_order_release);
        m_scrub_velocity.store(velocity, std::memory_order_release);
        seek(target_sample);
    }

    void update_scrub(uint64_t target_sample, double velocity = 1.0) noexcept {
        m_scrub_velocity.store(velocity, std::memory_order_relaxed);
        seek(target_sample);
    }

    void end_scrub(uint64_t target_sample) noexcept {
        m_is_scrubbing.store(false, std::memory_order_release);
        m_scrub_velocity.store(0.0, std::memory_order_relaxed);
        seek(target_sample);
    }


    // External Sync Ingestion Hooks
    void sync_from_midi_clock(double external_bpm, uint64_t sample_pos, bool is_playing) noexcept {
        if (m_authority.load(std::memory_order_relaxed) != ClockAuthority::MidiClockSlave) {
            return;
        }
        if (external_bpm >= 20.0 && external_bpm <= 400.0) {
            m_bpm.store(external_bpm, std::memory_order_relaxed);
        }
        m_sample_position.store(sample_pos, std::memory_order_relaxed);
        m_is_playing.store(is_playing, std::memory_order_relaxed);
    }

    void sync_from_mtc(double total_seconds, bool is_playing) noexcept {
        if (m_authority.load(std::memory_order_relaxed) != ClockAuthority::MtcSlave) {
            return;
        }
        const double sr = static_cast<double>(m_sample_rate.load(std::memory_order_relaxed));
        const uint64_t pos = static_cast<uint64_t>(std::max(0.0, std::round(total_seconds * sr)));
        m_sample_position.store(pos, std::memory_order_relaxed);
        m_is_playing.store(is_playing, std::memory_order_relaxed);
    }

    // Mathematical Grid Calculations
    [[nodiscard]] double samples_per_beat() const noexcept {
        const double rate = static_cast<double>(m_sample_rate.load(std::memory_order_relaxed));
        const double tempo = m_bpm.load(std::memory_order_relaxed);
        return (rate * 60.0) / std::max(1.0, tempo);
    }

    [[nodiscard]] double samples_per_bar() const noexcept {
        const double spb = samples_per_beat();
        const double num = static_cast<double>(m_time_sig_num.load(std::memory_order_relaxed));
        const double den = static_cast<double>(m_time_sig_den.load(std::memory_order_relaxed));
        return spb * num * (4.0 / den);
    }

    [[nodiscard]] uint64_t samples_for_bars(uint32_t num_bars) const noexcept {
        return static_cast<uint64_t>(std::round(samples_per_bar() * static_cast<double>(num_bars)));
    }

    [[nodiscard]] uint64_t samples_to_next_beat() const noexcept {
        const double spb = samples_per_beat();
        const uint64_t pos = m_sample_position.load(std::memory_order_relaxed);
        const double current_beat = static_cast<double>(pos) / spb;
        const double next_beat = std::floor(current_beat) + 1.0;
        const uint64_t next_sample = static_cast<uint64_t>(std::round(next_beat * spb));
        return (next_sample > pos) ? (next_sample - pos) : static_cast<uint64_t>(std::round(spb));
    }

    [[nodiscard]] uint64_t samples_to_next_bar() const noexcept {
        const double spbar = samples_per_bar();
        const uint64_t pos = m_sample_position.load(std::memory_order_relaxed);
        const double current_bar = static_cast<double>(pos) / spbar;
        const double next_bar = std::floor(current_bar) + 1.0;
        const uint64_t next_sample = static_cast<uint64_t>(std::round(next_bar * spbar));
        return (next_sample > pos) ? (next_sample - pos) : static_cast<uint64_t>(std::round(spbar));
    }

    // Get snapshot of current musical position
    [[nodiscard]] MusicalPosition position_snapshot() const noexcept {
        const uint64_t pos = m_sample_position.load(std::memory_order_relaxed);
        const double spb = samples_per_beat();
        const double spbar = samples_per_bar();
        const double total_beats = static_cast<double>(pos) / spb;
        const double total_bars = static_cast<double>(pos) / spbar;

        const uint32_t bar_index = static_cast<uint32_t>(std::floor(total_bars));
        const double bar_progress = total_bars - std::floor(total_bars);

        const double num = static_cast<double>(m_time_sig_num.load(std::memory_order_relaxed));
        const uint32_t beat_within_bar = static_cast<uint32_t>(std::floor(bar_progress * num));
        const double beat_progress = total_beats - std::floor(total_beats);

        return MusicalPosition{
            .sample_position = pos,
            .bpm = m_bpm.load(std::memory_order_relaxed),
            .total_beats = total_beats,
            .bar_index = bar_index,
            .beat_within_bar = beat_within_bar,
            .beat_progress = beat_progress,
            .bar_progress = bar_progress,
            .is_playing = m_is_playing.load(std::memory_order_relaxed)
        };
    }

    // RT Render Loop Hook: Advances the timeline clock by frames and detects sample-exact musical boundaries
    BlockBoundaryEvents advance_block(uint32_t frames) noexcept {
        BlockBoundaryEvents events{};
        if (!m_is_playing.load(std::memory_order_relaxed) || frames == 0) {
            return events;
        }

        const uint64_t start_pos = m_sample_position.load(std::memory_order_relaxed);
        const uint64_t end_pos = start_pos + frames;

        const double spb = samples_per_beat();
        const double spbar = samples_per_bar();

        const uint64_t start_beat_idx = static_cast<uint64_t>(std::floor(static_cast<double>(start_pos) / spb));
        const uint64_t end_beat_idx = static_cast<uint64_t>(std::floor(static_cast<double>(end_pos) / spb));

        if (end_beat_idx > start_beat_idx) {
            events.has_beat_boundary = true;
            uint64_t boundary_sample = static_cast<uint64_t>(std::round(static_cast<double>(end_beat_idx) * spb));
            if (boundary_sample >= start_pos && boundary_sample < end_pos) {
                events.beat_sample_offset = static_cast<uint32_t>(boundary_sample - start_pos);
            } else {
                events.beat_sample_offset = 0;
            }
        }

        const uint64_t start_bar_idx = static_cast<uint64_t>(std::floor(static_cast<double>(start_pos) / spbar));
        const uint64_t end_bar_idx = static_cast<uint64_t>(std::floor(static_cast<double>(end_pos) / spbar));

        if (end_bar_idx > start_bar_idx) {
            events.has_bar_boundary = true;
            uint64_t boundary_sample = static_cast<uint64_t>(std::round(static_cast<double>(end_bar_idx) * spbar));
            if (boundary_sample >= start_pos && boundary_sample < end_pos) {
                events.bar_sample_offset = static_cast<uint32_t>(boundary_sample - start_pos);
            } else {
                events.bar_sample_offset = 0;
            }
        }

        m_sample_position.store(end_pos, std::memory_order_relaxed);
        return events;
    }

private:
    std::atomic<ClockAuthority> m_authority{ClockAuthority::Master};
    std::atomic<uint32_t> m_sample_rate{48000};
    std::atomic<double> m_bpm{120.0};
    std::atomic<uint16_t> m_time_sig_num{4};
    std::atomic<uint16_t> m_time_sig_den{4};

    std::atomic<bool> m_is_playing{false};
    std::atomic<uint64_t> m_sample_position{0};

    // Scrubbing and Discontinuous Seek State
    std::atomic<bool> m_is_scrubbing{false};
    std::atomic<uint64_t> m_seek_generation{0};
    std::atomic<uint64_t> m_last_seek_sample{0};
    std::atomic<double> m_scrub_velocity{0.0};
};

} // namespace audio_core::clock
