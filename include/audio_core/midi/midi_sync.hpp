#pragma once

#include "audio_core/types.hpp"
#include "audio_core/clock/timeline_clock.hpp"

#include <cstdint>
#include <atomic>
#include <chrono>
#include <cmath>
#include <string>
#include <array>
#include <vector>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace audio_core::midi {

// ============================================================================
// MidiSyncMode: Defines the synchronization protocol
// Strict isolation: Beat Clock (24 PPQN) vs Linear Time Code (MTC)
// ============================================================================
enum class MidiSyncMode : uint8_t {
    Disabled = 0,
    MidiClock, // 24 PPQN Beat Clock + Song Position Pointer + Start/Continue/Stop
    MTC        // MIDI Time Code (SMPTE Frames: 24, 25, 29.97, 30 fps)
};

// ============================================================================
// MtcFrameRate: Standard SMPTE Time Code Frame Rates
// ============================================================================
enum class MtcFrameRate : uint8_t {
    Fps24 = 0,       // 24 fps (Film Standard)
    Fps25 = 1,       // 25 fps (PAL / EBU Television)
    Fps2997Drop = 2, // 29.97 fps (NTSC Color Drop-Frame)
    Fps30 = 3        // 30 fps (NTSC Non-Drop / High Speed)
};

// ============================================================================
// MtcTimecode: Absolute SMPTE Linear Wall-Clock Time (Independent of BPM!)
// ============================================================================
struct MtcTimecode {
    uint8_t hours{0};       // 0..23
    uint8_t minutes{0};     // 0..59
    uint8_t seconds{0};     // 0..59
    uint8_t frames{0};      // 0..29
    MtcFrameRate rate{MtcFrameRate::Fps25};
    bool is_valid{false};

    [[nodiscard]] constexpr double frames_per_second() const noexcept {
        switch (rate) {
            case MtcFrameRate::Fps24: return 24.0;
            case MtcFrameRate::Fps25: return 25.0;
            case MtcFrameRate::Fps2997Drop: return 29.97002997;
            case MtcFrameRate::Fps30: return 30.0;
        }
        return 25.0;
    }

    [[nodiscard]] double total_seconds() const noexcept {
        if (rate == MtcFrameRate::Fps2997Drop) {
            int64_t total_minutes = static_cast<int64_t>(hours) * 60 + minutes;
            int64_t total_frames = (static_cast<int64_t>(hours) * 3600 + static_cast<int64_t>(minutes) * 60 + seconds) * 30 + frames;
            int64_t dropped = 2 * (total_minutes - total_minutes / 10);
            int64_t real_frames = total_frames - dropped;
            return static_cast<double>(real_frames) * (1001.0 / 30000.0);
        }
        const double fps = frames_per_second();
        return static_cast<double>(hours) * 3600.0 +
               static_cast<double>(minutes) * 60.0 +
               static_cast<double>(seconds) +
               static_cast<double>(frames) / fps;
    }

    static MtcTimecode from_seconds(double total_sec, MtcFrameRate rate = MtcFrameRate::Fps25) noexcept {
        MtcTimecode tc{};
        tc.rate = rate;
        tc.is_valid = true;
        if (total_sec < 0.0) total_sec = 0.0;

        if (rate == MtcFrameRate::Fps2997Drop) {
            // SMPTE 12M drop-frame conversion
            const double nominal_fps = 30000.0 / 1001.0;
            int64_t frame_num = static_cast<int64_t>(std::round(total_sec * nominal_fps));
            const int64_t d = frame_num / 17982;
            const int64_t m = frame_num % 17982;
            if (m >= 2) {
                frame_num += 18 * d + 2 * ((m - 2) / 1798);
            } else {
                frame_num += 18 * d;
            }
            tc.frames = static_cast<uint8_t>(frame_num % 30);
            tc.seconds = static_cast<uint8_t>((frame_num / 30) % 60);
            tc.minutes = static_cast<uint8_t>(((frame_num / 30) / 60) % 60);
            tc.hours = static_cast<uint8_t>((((frame_num / 30) / 60) / 60) % 24);
        } else {
            double fps = 25.0;
            if (rate == MtcFrameRate::Fps24) fps = 24.0;
            else if (rate == MtcFrameRate::Fps30) fps = 30.0;
            int64_t total_frames = static_cast<int64_t>(std::floor(total_sec * fps + 1e-6));
            int64_t ifps = static_cast<int64_t>(std::round(fps));
            tc.frames = static_cast<uint8_t>(total_frames % ifps);
            int64_t total_s = total_frames / ifps;
            tc.seconds = static_cast<uint8_t>(total_s % 60);
            int64_t total_m = total_s / 60;
            tc.minutes = static_cast<uint8_t>(total_m % 60);
            tc.hours = static_cast<uint8_t>((total_m / 60) % 24);
        }
        return tc;
    }

    [[nodiscard]] constexpr uint64_t pack() const noexcept {
        return (static_cast<uint64_t>(hours) << 0) |
               (static_cast<uint64_t>(minutes) << 8) |
               (static_cast<uint64_t>(seconds) << 16) |
               (static_cast<uint64_t>(frames) << 24) |
               (static_cast<uint64_t>(static_cast<uint8_t>(rate)) << 32) |
               (static_cast<uint64_t>(is_valid ? 1 : 0) << 40);
    }

    static constexpr MtcTimecode unpack(uint64_t val) noexcept {
        MtcTimecode tc{};
        tc.hours = static_cast<uint8_t>(val & 0xFF);
        tc.minutes = static_cast<uint8_t>((val >> 8) & 0xFF);
        tc.seconds = static_cast<uint8_t>((val >> 16) & 0xFF);
        tc.frames = static_cast<uint8_t>((val >> 24) & 0xFF);
        tc.rate = static_cast<MtcFrameRate>((val >> 32) & 0x03);
        tc.is_valid = ((val >> 40) & 0x01) != 0;
        return tc;
    }

    [[nodiscard]] std::string to_string() const {
        std::ostringstream oss;
        oss << std::setfill('0')
            << std::setw(2) << static_cast<int>(hours) << ":"
            << std::setw(2) << static_cast<int>(minutes) << ":"
            << std::setw(2) << static_cast<int>(seconds) << ":"
            << std::setw(2) << static_cast<int>(frames);
        switch (rate) {
            case MtcFrameRate::Fps24: oss << " (24 fps)"; break;
            case MtcFrameRate::Fps25: oss << " (25 fps)"; break;
            case MtcFrameRate::Fps2997Drop: oss << " (29.97df)"; break;
            case MtcFrameRate::Fps30: oss << " (30 fps)"; break;
        }
        return oss.str();
    }
};

// ============================================================================
// MidiClockTelemetry: Live Telemetry Snapshot for Beat Clock Tracking
// ============================================================================
struct MidiClockTelemetry {
    double estimated_bpm{120.0};
    uint64_t tick_count{0};
    uint16_t song_position_spp{0}; // In 1/16th notes (6 clocks per unit)
    double total_beats{0.0};
    uint32_t bar_index{0};
    uint32_t beat_within_bar{0};
    bool is_playing{false};
    bool is_locked{false};
    double jitter_ms{0.0};
};

// ============================================================================
// MidiSyncTracker: High-Precision Jitter-Free PLL Beat Clock & MTC Decoder
// Decodes:
// 1. 24 PPQN Beat Clock (0xF8) with outlier rejection and moving average PLL
// 2. Song Position Pointer (0xF2): 1 unit = 6 MIDI clocks = 1/16th note
// 3. Realtime Transport: Start (0xFA), Continue (0xFB), Stop (0xFC)
// 4. MIDI Time Code (MTC 0xF1): 8-piece quarter-frame SMPTE reassembler
// ============================================================================
class MidiSyncTracker {
public:
    MidiSyncTracker() {
        reset_clock_state();
    }

    void reset() noexcept {
        reset_clock_state();
        reset_mtc_state();
    }

    // ========================================================================
    // Ingestion: Beat Clock 24 PPQN (0xF8)
    // ========================================================================
    void on_clock_tick(uint64_t timestamp_ns = 0) noexcept {
        if (timestamp_ns == 0) {
            timestamp_ns = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count());
        }

        if (m_last_tick_ns > 0) {
            int64_t delta_ns = static_cast<int64_t>(timestamp_ns - m_last_tick_ns);

            // 1. Noise / bounce rejection: < 5ms (corresponds to > 400 BPM or jitter spike)
            // Discard spurious noise pulse completely without polluting last_tick_ns
            if (delta_ns < 5'000'000) {
                return;
            }

            // 2. Pause / gap rejection: > 150ms (corresponds to < 20 BPM or transport idle gap)
            // Resets reference timestamp without polluting moving average PLL buffer
            if (delta_ns > 150'000'000) {
                m_last_tick_ns = timestamp_ns;
                m_tick_count.fetch_add(1, std::memory_order_relaxed);
                return;
            }

            // 3. Valid musical clock tick (20 BPM .. 400 BPM)
            m_tick_intervals[m_interval_idx] = delta_ns;
            m_interval_idx = (m_interval_idx + 1) % kWindowSize;
            if (m_interval_count < kWindowSize) ++m_interval_count;

            // When at least 6 ticks are buffered, compute smoothed average
            if (m_interval_count >= 6) {
                int64_t sum = 0;
                for (size_t i = 0; i < m_interval_count; ++i) {
                    sum += m_tick_intervals[i];
                }
                double avg_delta_ns = static_cast<double>(sum) / static_cast<double>(m_interval_count);

                // BPM = (60.0 * 1e9) / (avg_delta_ns * 24.0) = (2.5 * 1e9) / avg_delta_ns
                double calculated_bpm = (2.5 * 1e9) / avg_delta_ns;
                if (calculated_bpm >= 20.0 && calculated_bpm <= 400.0) {
                    m_estimated_bpm.store(calculated_bpm, std::memory_order_relaxed);
                    m_is_locked.store(true, std::memory_order_relaxed);
                }

                // Jitter estimation (standard deviation in ms)
                double variance = 0.0;
                for (size_t i = 0; i < m_interval_count; ++i) {
                    double diff = static_cast<double>(m_tick_intervals[i]) - avg_delta_ns;
                    variance += diff * diff;
                }
                double std_dev_ms = std::sqrt(variance / static_cast<double>(m_interval_count)) * 1e-6;
                m_jitter_ms.store(std_dev_ms, std::memory_order_relaxed);
            }
        }

        m_last_tick_ns = timestamp_ns;
        uint64_t new_ticks = m_tick_count.fetch_add(1, std::memory_order_relaxed) + 1;
        m_total_beats.store(static_cast<double>(new_ticks) / 24.0, std::memory_order_relaxed);
    }

    // ========================================================================
    // Ingestion: Transport Realtime Commands
    // ========================================================================
    void on_start() noexcept {
        m_tick_count.store(0, std::memory_order_relaxed);
        m_total_beats.store(0.0, std::memory_order_relaxed);
        m_spp_sixteenths.store(0, std::memory_order_relaxed);
        m_is_playing.store(true, std::memory_order_relaxed);
    }

    void on_continue() noexcept {
        m_is_playing.store(true, std::memory_order_relaxed);
    }

    void on_stop() noexcept {
        m_is_playing.store(false, std::memory_order_relaxed);
    }

    // ========================================================================
    // Ingestion: Song Position Pointer (SPP) 0xF2
    // CRITICAL FORMAT INVARIANT: 1 SPP unit = 6 MIDI clocks = 1/16th note!
    // 4 units = 1 beat (quarter note = 24 clocks); 16 units = 1 4/4 bar (96 clocks)
    // ========================================================================
    void on_song_position_pointer(uint16_t spp_sixteenths) noexcept {
        uint16_t clamped = std::min<uint16_t>(spp_sixteenths, 16383);
        m_spp_sixteenths.store(clamped, std::memory_order_relaxed);
        uint64_t ticks = static_cast<uint64_t>(clamped) * 6;
        m_tick_count.store(ticks, std::memory_order_relaxed);
        m_total_beats.store(static_cast<double>(clamped) / 4.0, std::memory_order_relaxed);
    }

    // ========================================================================
    // Ingestion: MIDI Time Code (MTC) Quarter Frame 0xF1
    // Reassembles 8 consecutive pieces into full SMPTE timecode (Hours:Min:Sec:Fr)
    // ========================================================================
    void on_mtc_quarter_frame(uint8_t qframe_data) noexcept {
        uint8_t piece = (qframe_data >> 4) & 0x07;
        uint8_t nibble = qframe_data & 0x0F;

        m_mtc_pieces[piece] = nibble;
        m_mtc_piece_mask |= (1 << piece);

        if (piece == 7) {
            // Check if all 8 pieces (0..7) were successfully received
            if (m_mtc_piece_mask == 0xFF) {
                uint8_t fr = m_mtc_pieces[0] | ((m_mtc_pieces[1] & 0x01) << 4);
                uint8_t sc = m_mtc_pieces[2] | ((m_mtc_pieces[3] & 0x03) << 4);
                uint8_t mn = m_mtc_pieces[4] | ((m_mtc_pieces[5] & 0x03) << 4);
                uint8_t hr = m_mtc_pieces[6] | ((m_mtc_pieces[7] & 0x01) << 4);
                uint8_t rate_code = (m_mtc_pieces[7] >> 1) & 0x03;

                MtcTimecode tc{};
                tc.hours = hr;
                tc.minutes = mn;
                tc.seconds = sc;
                tc.frames = fr;
                tc.rate = static_cast<MtcFrameRate>(rate_code);
                tc.is_valid = true;
                m_mtc_packed.store(tc.pack(), std::memory_order_relaxed);
                m_mtc_is_playing.store(true, std::memory_order_relaxed);
            }
            m_mtc_piece_mask = 0;
        }
    }

    void on_mtc_full_frame(uint8_t hr, uint8_t mn, uint8_t sc, uint8_t fr, MtcFrameRate rate) noexcept {
        MtcTimecode tc{};
        tc.hours = hr;
        tc.minutes = mn;
        tc.seconds = sc;
        tc.frames = fr;
        tc.rate = rate;
        tc.is_valid = true;
        m_mtc_packed.store(tc.pack(), std::memory_order_relaxed);
        m_mtc_is_playing.store(true, std::memory_order_relaxed);
    }

    // ========================================================================
    // Synchronization Hook to TimelineClock
    // Strictly applies external master to TimelineClock based on Authority
    // ========================================================================
    void apply_to_timeline_clock(clock::TimelineClock& clock) noexcept {
        const auto auth = clock.authority();
        if (auth == clock::ClockAuthority::MidiClockSlave) {
            if (m_is_locked.load(std::memory_order_relaxed)) {
                clock.set_bpm(m_estimated_bpm.load(std::memory_order_relaxed));
            }
            clock.set_playing(m_is_playing.load(std::memory_order_relaxed));
            const double beats = m_total_beats.load(std::memory_order_relaxed);
            const double spb = clock.samples_per_beat();
            const uint64_t sample_pos = static_cast<uint64_t>(std::max(0.0, std::round(beats * spb)));
            clock.set_sample_position(sample_pos);
        } else if (auth == clock::ClockAuthority::MtcSlave) {
            MtcTimecode tc = mtc_timecode();
            if (tc.is_valid) {
                clock.sync_from_mtc(tc.total_seconds(), m_mtc_is_playing.load(std::memory_order_relaxed));
            }
        }
    }

    // ========================================================================
    // Live State & Telemetry Queries
    // ========================================================================
    [[nodiscard]] bool is_locked() const noexcept { return m_is_locked.load(std::memory_order_relaxed); }
    [[nodiscard]] double estimated_bpm() const noexcept { return m_estimated_bpm.load(std::memory_order_relaxed); }
    [[nodiscard]] uint64_t tick_count() const noexcept { return m_tick_count.load(std::memory_order_relaxed); }
    [[nodiscard]] uint16_t song_position_spp() const noexcept { return m_spp_sixteenths.load(std::memory_order_relaxed); }
    [[nodiscard]] double total_beats() const noexcept { return m_total_beats.load(std::memory_order_relaxed); }
    [[nodiscard]] bool is_playing() const noexcept { return m_is_playing.load(std::memory_order_relaxed); }
    [[nodiscard]] double jitter_ms() const noexcept { return m_jitter_ms.load(std::memory_order_relaxed); }

    [[nodiscard]] MtcTimecode mtc_timecode() const noexcept {
        return MtcTimecode::unpack(m_mtc_packed.load(std::memory_order_relaxed));
    }
    [[nodiscard]] bool is_mtc_playing() const noexcept { return m_mtc_is_playing.load(std::memory_order_relaxed); }

    [[nodiscard]] MidiClockTelemetry telemetry(uint16_t time_sig_num = 4) const noexcept {
        double beats = m_total_beats.load(std::memory_order_relaxed);
        double bars = beats / static_cast<double>(std::max<uint16_t>(1, time_sig_num));
        uint32_t bar_idx = static_cast<uint32_t>(std::max(0.0, std::floor(bars)));
        uint32_t beat_within = static_cast<uint32_t>(std::max(0.0, std::floor(std::fmod(beats, static_cast<double>(time_sig_num)))));

        return MidiClockTelemetry{
            .estimated_bpm = m_estimated_bpm.load(std::memory_order_relaxed),
            .tick_count = m_tick_count.load(std::memory_order_relaxed),
            .song_position_spp = m_spp_sixteenths.load(std::memory_order_relaxed),
            .total_beats = beats,
            .bar_index = bar_idx,
            .beat_within_bar = beat_within,
            .is_playing = m_is_playing.load(std::memory_order_relaxed),
            .is_locked = m_is_locked.load(std::memory_order_relaxed),
            .jitter_ms = m_jitter_ms.load(std::memory_order_relaxed)
        };
    }

private:
    void reset_clock_state() noexcept {
        m_last_tick_ns = 0;
        m_interval_idx = 0;
        m_interval_count = 0;
        std::fill(m_tick_intervals.begin(), m_tick_intervals.end(), 0);
        m_estimated_bpm.store(120.0, std::memory_order_relaxed);
        m_tick_count.store(0, std::memory_order_relaxed);
        m_spp_sixteenths.store(0, std::memory_order_relaxed);
        m_total_beats.store(0.0, std::memory_order_relaxed);
        m_is_playing.store(false, std::memory_order_relaxed);
        m_is_locked.store(false, std::memory_order_relaxed);
        m_jitter_ms.store(0.0, std::memory_order_relaxed);
    }

    void reset_mtc_state() noexcept {
        m_mtc_pieces.fill(0);
        m_mtc_piece_mask = 0;
        m_mtc_packed.store(0, std::memory_order_relaxed);
        m_mtc_is_playing.store(false, std::memory_order_relaxed);
    }

    static constexpr size_t kWindowSize = 24; // 1 full quarter note of clock intervals

    // Beat Clock PLL State
    uint64_t m_last_tick_ns{0};
    size_t m_interval_idx{0};
    size_t m_interval_count{0};
    std::array<int64_t, kWindowSize> m_tick_intervals{};

    std::atomic<double> m_estimated_bpm{120.0};
    std::atomic<uint64_t> m_tick_count{0};
    std::atomic<uint16_t> m_spp_sixteenths{0};
    std::atomic<double> m_total_beats{0.0};
    std::atomic<bool> m_is_playing{false};
    std::atomic<bool> m_is_locked{false};
    std::atomic<double> m_jitter_ms{0.0};

    // MTC State
    std::array<uint8_t, 8> m_mtc_pieces{};
    uint8_t m_mtc_piece_mask{0};
    std::atomic<uint64_t> m_mtc_packed{0};
    std::atomic<bool> m_mtc_is_playing{false};
};

// ============================================================================
// MidiClockGenerator: Master Clock Output Generator
// Generates:
// - 0xF8 Timing Clock (24 PPQN) synchronized to TimelineClock
// - 0xFA Start / 0xFB Continue / 0xFC Stop on transport transitions
// - 0xF2 Song Position Pointer on transport seek
// - 0xF1 MIDI Time Code (MTC) Quarter Frames (8 pieces/frame at 24/25/29.97/30 fps)
// - Full Frame SysEx locator packets
// ============================================================================
class MidiClockGenerator {
public:
    void reset() noexcept {
        m_clock_phase = 0.0;
        m_mtc_phase = 0.0;
        m_mtc_piece = 0;
        m_was_playing = false;
        m_tick_count.store(0, std::memory_order_relaxed);
        m_qframe_count.store(0, std::memory_order_relaxed);
        m_current_tc = MtcTimecode{};
    }

    void set_beat_clock_enabled(bool enabled) noexcept {
        m_beat_clock_enabled.store(enabled, std::memory_order_relaxed);
    }
    [[nodiscard]] bool is_beat_clock_enabled() const noexcept {
        return m_beat_clock_enabled.load(std::memory_order_relaxed);
    }

    void set_mtc_enabled(bool enabled) noexcept {
        m_mtc_enabled.store(enabled, std::memory_order_relaxed);
    }
    [[nodiscard]] bool is_mtc_enabled() const noexcept {
        return m_mtc_enabled.load(std::memory_order_relaxed);
    }

    void set_mtc_framerate(MtcFrameRate rate) noexcept {
        m_mtc_framerate.store(rate, std::memory_order_relaxed);
    }
    [[nodiscard]] MtcFrameRate mtc_framerate() const noexcept {
        return m_mtc_framerate.load(std::memory_order_relaxed);
    }

    [[nodiscard]] uint64_t tick_count() const noexcept {
        return m_tick_count.load(std::memory_order_relaxed);
    }
    [[nodiscard]] uint64_t qframe_count() const noexcept {
        return m_qframe_count.load(std::memory_order_relaxed);
    }
    [[nodiscard]] MtcTimecode current_mtc_timecode() const noexcept {
        return m_current_tc;
    }

    // Dual-Consumer process_block: separates Beat Clock bytes from MTC Quarter Frames
    template <typename ClockByteConsumer, typename MtcByteConsumer>
    void process_block(uint32_t frames, const clock::TimelineClock& clock,
                       ClockByteConsumer&& clock_consumer, MtcByteConsumer&& mtc_consumer) noexcept {
        const bool is_playing = clock.is_playing();
        const bool beat_enabled = m_beat_clock_enabled.load(std::memory_order_relaxed);
        const bool mtc_enabled = m_mtc_enabled.load(std::memory_order_relaxed);

        // 1. Transport State Transitions (Start / Continue / Stop)
        if (beat_enabled) {
            if (is_playing && !m_was_playing) {
                if (clock.sample_position() == 0) {
                    clock_consumer(0xFA); // Start from beginning
                    m_tick_count.store(0, std::memory_order_relaxed);
                    m_clock_phase = 0.0;
                } else {
                    clock_consumer(0xFB); // Continue from current position
                }
            } else if (!is_playing && m_was_playing) {
                clock_consumer(0xFC); // Stop
            }
        }

        // 2. Generate 24 PPQN Timing Clocks if playing
        if (beat_enabled && is_playing && frames > 0) {
            const double spb = clock.samples_per_beat();
            const double samples_per_clock = spb / 24.0;
            if (samples_per_clock > 0.0) {
                m_clock_phase += static_cast<double>(frames);
                while (m_clock_phase >= samples_per_clock) {
                    clock_consumer(0xF8); // Timing Clock
                    m_clock_phase -= samples_per_clock;
                    m_tick_count.fetch_add(1, std::memory_order_relaxed);
                }
            }
        }

        // 3. Generate MTC Quarter-Frames (8 pieces per frame) if playing
        if (mtc_enabled && is_playing && frames > 0) {
            const auto rate = m_mtc_framerate.load(std::memory_order_relaxed);
            double fps = 25.0;
            switch (rate) {
                case MtcFrameRate::Fps24: fps = 24.0; break;
                case MtcFrameRate::Fps25: fps = 25.0; break;
                case MtcFrameRate::Fps2997Drop: fps = 29.97002997; break;
                case MtcFrameRate::Fps30: fps = 30.0; break;
            }
            const double sr = static_cast<double>(clock.sample_rate());
            const double samples_per_qframe = sr / (8.0 * fps);

            if (samples_per_qframe > 0.0) {
                m_mtc_phase += static_cast<double>(frames);
                while (m_mtc_phase >= samples_per_qframe) {
                    m_mtc_phase -= samples_per_qframe;

                    // Snapshot timecode on piece 0 to guarantee consistency across 8 pieces
                    if (m_mtc_piece == 0) {
                        const double total_secs = static_cast<double>(clock.sample_position()) / sr;
                        m_current_tc = MtcTimecode::from_seconds(total_secs, rate);
                    }

                    uint8_t nibble = 0;
                    switch (m_mtc_piece) {
                        case 0: nibble = m_current_tc.frames & 0x0F; break;
                        case 1: nibble = (m_current_tc.frames >> 4) & 0x01; break;
                        case 2: nibble = m_current_tc.seconds & 0x0F; break;
                        case 3: nibble = (m_current_tc.seconds >> 4) & 0x03; break;
                        case 4: nibble = m_current_tc.minutes & 0x0F; break;
                        case 5: nibble = (m_current_tc.minutes >> 4) & 0x03; break;
                        case 6: nibble = m_current_tc.hours & 0x0F; break;
                        case 7: {
                            uint8_t rate_code = static_cast<uint8_t>(rate) & 0x03;
                            nibble = ((rate_code << 1) | ((m_current_tc.hours >> 4) & 0x01)) & 0x0F;
                            break;
                        }
                        default: break;
                    }

                    uint8_t qf_data = (static_cast<uint8_t>(m_mtc_piece) << 4) | (nibble & 0x0F);
                    mtc_consumer(qf_data);

                    m_mtc_piece = (m_mtc_piece + 1) & 0x07;
                    m_qframe_count.fetch_add(1, std::memory_order_relaxed);
                }
            }
        }

        m_was_playing = is_playing;
    }

    // Backwards-compatible single-consumer overload:
    // Emits Timing/Transport bytes directly, and prefixes MTC Quarter-Frames with 0xF1 status
    template <typename ByteConsumer>
    void process_block(uint32_t frames, const clock::TimelineClock& clock, ByteConsumer&& consumer) noexcept {
        process_block(
            frames, clock,
            [&](uint8_t b) { consumer(b); },
            [&](uint8_t qf) {
                consumer(0xF1);
                consumer(qf);
            }
        );
    }

    static std::array<uint8_t, 3> make_spp(uint16_t spp_units) noexcept {
        uint16_t clamped = std::min<uint16_t>(spp_units, 16383);
        uint8_t lsb = static_cast<uint8_t>(clamped & 0x7F);
        uint8_t msb = static_cast<uint8_t>((clamped >> 7) & 0x7F);
        return { 0xF2, lsb, msb };
    }

    static std::array<uint8_t, 10> make_mtc_full_frame(uint8_t hr, uint8_t mn, uint8_t sc, uint8_t fr, MtcFrameRate rate) noexcept {
        uint8_t hr_byte = ((static_cast<uint8_t>(rate) & 0x03) << 5) | (hr & 0x1F);
        return {
            0xF0, 0x7F, 0x7F, 0x01, 0x01,
            hr_byte,
            static_cast<uint8_t>(mn & 0x3F),
            static_cast<uint8_t>(sc & 0x3F),
            static_cast<uint8_t>(fr & 0x1F),
            0xF7
        };
    }

    static std::array<uint8_t, 10> make_mtc_full_frame(const MtcTimecode& tc) noexcept {
        return make_mtc_full_frame(tc.hours, tc.minutes, tc.seconds, tc.frames, tc.rate);
    }

private:
    double m_clock_phase{0.0};
    double m_mtc_phase{0.0};
    uint8_t m_mtc_piece{0};
    bool m_was_playing{false};
    std::atomic<bool> m_beat_clock_enabled{true};
    std::atomic<bool> m_mtc_enabled{true};
    std::atomic<MtcFrameRate> m_mtc_framerate{MtcFrameRate::Fps25};
    std::atomic<uint64_t> m_tick_count{0};
    std::atomic<uint64_t> m_qframe_count{0};
    MtcTimecode m_current_tc{};
};

} // namespace audio_core::midi
