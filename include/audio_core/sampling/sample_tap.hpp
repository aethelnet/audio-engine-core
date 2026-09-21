#pragma once

#include "audio_core/sampling/audio_clip.hpp"
#include "audio_core/sampling/loop_conditioner.hpp"
#include "audio_core/clock/timeline_clock.hpp"
#include <vector>
#include <string>
#include <atomic>
#include <memory>
#include <cstdint>
#include <algorithm>
#include <cmath>

namespace audio_core::sampling {

enum class TapSourceType : uint8_t {
    Disabled = 0,
    TrackInput,   // Pre-FX raw input (local app or incoming AoIP network stream)
    TrackOutput,  // Post-FX (after modular insert slots & console encode)
    BusOutput,    // Submix bus output (e.g. Drum bus bounce with glue compressor)
    MasterOutput  // Full mix bounce
};

enum class RecordMode : uint8_t {
    RollingBuffer = 0,  // Continuous circular buffer for retroactive capture
    QuantizedBounce     // Records exactly N frames from arm trigger, then stops
};

enum class QuantizeSyncMode : uint8_t {
    Immediate = 0, // Starts immediately on next block
    BeatSync,      // Starts recording at next beat boundary (e.g. quarter note)
    BarSync        // Starts recording at next bar downbeat (Measure 1.0)
};

enum class RecordState : uint8_t {
    Idle = 0,
    Armed,
    Recording,
    Complete
};

struct TapSource {
    TapSourceType type{TapSourceType::Disabled};
    uint32_t source_id{0}; // Track ID or Bus ID
};

// ============================================================================
// SampleTap: Clock-Synchronized Audio Tap & Quantized Loop Recorder
// Taps into any point of the MixerGraph: Track Pre/Post FX, Submix Bus, or Master.
// Integrates with TimelineClock for sample-accurate downbeat arming and seamless looping.
// ============================================================================
class SampleTap {
public:
    explicit SampleTap(uint32_t sample_rate = 48000, float capacity_seconds = 10.0f)
        : m_sample_rate(sample_rate),
          m_capacity_frames(std::max(1024u, static_cast<uint32_t>(sample_rate * capacity_seconds))),
          m_buffer_l(m_capacity_frames, 0.0f),
          m_buffer_r(m_capacity_frames, 0.0f) {}

    void set_source(TapSourceType type, uint32_t source_id = 0) noexcept {
        m_source_type.store(type, std::memory_order_relaxed);
        m_source_id.store(source_id, std::memory_order_relaxed);
    }

    [[nodiscard]] TapSource source() const noexcept {
        return TapSource{
            m_source_type.load(std::memory_order_relaxed),
            m_source_id.load(std::memory_order_relaxed)
        };
    }

    [[nodiscard]] bool is_active() const noexcept {
        return m_source_type.load(std::memory_order_relaxed) != TapSourceType::Disabled;
    }

    [[nodiscard]] uint32_t sample_rate() const noexcept { return m_sample_rate; }
    void set_sample_rate(uint32_t sample_rate) noexcept {
        if (sample_rate > 0) {
            m_sample_rate = sample_rate;
        }
    }
    [[nodiscard]] uint32_t capacity_frames() const noexcept { return m_capacity_frames; }

    // Arm a quantized bounce with explicit target frame length
    void arm_quantized_bounce(uint32_t target_frames, std::string clip_name = "QuantizedBounce",
                              QuantizeSyncMode sync_mode = QuantizeSyncMode::Immediate,
                              bool auto_condition_seamless = true) {
        m_quantized_name = std::move(clip_name);
        m_quantized_target_frames = target_frames;
        m_quantized_recorded_frames.store(0, std::memory_order_relaxed);
        m_quantized_clip.reset();
        m_sync_mode.store(sync_mode, std::memory_order_relaxed);
        m_auto_condition.store(auto_condition_seamless, std::memory_order_relaxed);
        m_record_mode.store(RecordMode::QuantizedBounce, std::memory_order_relaxed);
        m_record_state.store(RecordState::Armed, std::memory_order_release);
    }

    // Arm a bar-synchronized bounce (records exactly N musical bars on the next downbeat)
    void arm_bar_bounce(const clock::TimelineClock& clock, uint32_t num_bars = 1,
                        std::string clip_name = "BarBounce", bool auto_condition_seamless = true) {
        uint32_t target_frames = static_cast<uint32_t>(std::round(clock.samples_for_bars(num_bars)));
        arm_quantized_bounce(target_frames, std::move(clip_name), QuantizeSyncMode::BarSync, auto_condition_seamless);
    }

    // Arm a beat-synchronized bounce (records exactly N beats starting at next quarter note)
    void arm_beat_bounce(const clock::TimelineClock& clock, uint32_t num_beats = 4,
                         std::string clip_name = "BeatBounce", bool auto_condition_seamless = true) {
        uint32_t target_frames = static_cast<uint32_t>(std::round(clock.samples_per_beat() * static_cast<double>(num_beats)));
        arm_quantized_bounce(target_frames, std::move(clip_name), QuantizeSyncMode::BeatSync, auto_condition_seamless);
    }

    // Set rolling buffer mode for retroactive capture
    void set_rolling_mode() noexcept {
        m_record_mode.store(RecordMode::RollingBuffer, std::memory_order_relaxed);
        m_record_state.store(RecordState::Recording, std::memory_order_release);
    }

    [[nodiscard]] RecordState record_state() const noexcept {
        return m_record_state.load(std::memory_order_acquire);
    }

    [[nodiscard]] QuantizeSyncMode sync_mode() const noexcept {
        return m_sync_mode.load(std::memory_order_relaxed);
    }

    [[nodiscard]] RecordMode record_mode() const noexcept {
        return m_record_mode.load(std::memory_order_relaxed);
    }

    [[nodiscard]] uint32_t recorded_frames() const noexcept {
        return m_quantized_recorded_frames.load(std::memory_order_relaxed);
    }

    [[nodiscard]] uint32_t target_frames() const noexcept {
        return m_quantized_target_frames;
    }

    [[nodiscard]] float progress() const noexcept {
        if (m_record_mode.load(std::memory_order_relaxed) == RecordMode::QuantizedBounce && m_quantized_target_frames > 0) {
            return std::clamp(static_cast<float>(m_quantized_recorded_frames.load(std::memory_order_relaxed)) / static_cast<float>(m_quantized_target_frames), 0.0f, 1.0f);
        }
        return 0.0f;
    }

    void dismiss_bounce_to_rolling() noexcept {
        m_quantized_clip.reset();
        m_quantized_recorded_frames.store(0, std::memory_order_relaxed);
        set_rolling_mode();
    }

    // RT Audio Thread Record Hook (Lock-free, zero allocation)
    void record(const float* left, const float* right, uint32_t frames,
                const clock::BlockBoundaryEvents* boundary_events = nullptr) noexcept {
        if (!is_active() || frames == 0) return;

        const RecordMode mode = m_record_mode.load(std::memory_order_relaxed);
        RecordState state = m_record_state.load(std::memory_order_acquire);

        if (mode == RecordMode::QuantizedBounce) {
            uint32_t start_frame = 0;

            if (state == RecordState::Armed) {
                const auto sync_mode = m_sync_mode.load(std::memory_order_relaxed);
                bool should_start = false;

                if (sync_mode == QuantizeSyncMode::Immediate || boundary_events == nullptr) {
                    should_start = true;
                    start_frame = 0;
                } else if (sync_mode == QuantizeSyncMode::BarSync) {
                    if (boundary_events->has_bar_boundary) {
                        should_start = true;
                        start_frame = boundary_events->bar_sample_offset;
                    }
                } else if (sync_mode == QuantizeSyncMode::BeatSync) {
                    if (boundary_events->has_beat_boundary) {
                        should_start = true;
                        start_frame = boundary_events->beat_sample_offset;
                    }
                }

                if (!should_start) {
                    // Still waiting for musical sync point (Downbeat / Beat)
                    return;
                }

                // Armed -> Recording transition on the exact sub-block frame offset!
                m_record_state.store(RecordState::Recording, std::memory_order_relaxed);
                state = RecordState::Recording;
            }

            if (state == RecordState::Recording) {
                uint32_t recorded = m_quantized_recorded_frames.load(std::memory_order_relaxed);
                if (recorded >= m_quantized_target_frames) {
                    m_record_state.store(RecordState::Complete, std::memory_order_release);
                    return;
                }

                uint32_t frames_avail = (frames > start_frame) ? (frames - start_frame) : 0;
                uint32_t to_write = std::min(frames_avail, m_quantized_target_frames - recorded);

                for (uint32_t i = 0; i < to_write; ++i) {
                    uint32_t idx = (recorded + i) % m_capacity_frames;
                    m_buffer_l[idx] = left[start_frame + i];
                    m_buffer_r[idx] = right[start_frame + i];
                }

                recorded += to_write;
                m_quantized_recorded_frames.store(recorded, std::memory_order_relaxed);

                if (recorded >= m_quantized_target_frames) {
                    m_record_state.store(RecordState::Complete, std::memory_order_release);
                }
            }
        } else {
            // Rolling buffer mode
            uint32_t write_pos = m_write_head.load(std::memory_order_relaxed);
            for (uint32_t i = 0; i < frames; ++i) {
                uint32_t idx = (write_pos + i) % m_capacity_frames;
                m_buffer_l[idx] = left[i];
                m_buffer_r[idx] = right[i];
            }
            m_write_head.store((write_pos + frames) % m_capacity_frames, std::memory_order_release);
            m_total_frames_recorded.fetch_add(frames, std::memory_order_relaxed);
        }
    }

    // Retrieve the completed quantized bounce clip (auto-conditions seamless seam if configured)
    [[nodiscard]] std::shared_ptr<AudioClip> get_quantized_clip() {
        if (m_record_state.load(std::memory_order_acquire) != RecordState::Complete) {
            return nullptr;
        }
        if (!m_quantized_clip) {
            uint32_t frames = std::min(m_quantized_target_frames, m_capacity_frames);
            auto clip = std::make_shared<AudioClip>(m_quantized_name, m_sample_rate, 2, frames);
            float* dst_l = clip->channel(0);
            float* dst_r = clip->channel(1);

            for (uint32_t i = 0; i < frames; ++i) {
                dst_l[i] = m_buffer_l[i];
                dst_r[i] = m_buffer_r[i];
            }

            if (m_auto_condition.load(std::memory_order_relaxed)) {
                LoopConditioner::condition_seamless(*clip, 128);
            }

            m_quantized_clip = clip;
        }
        return m_quantized_clip;
    }

    // Capture the most recent N frames (e.g. last 4 bars of live jam) from the rolling buffer
    [[nodiscard]] std::shared_ptr<AudioClip> capture_retroactive(uint32_t frames, const std::string& clip_name = "RetroactiveBounce",
                                                                 bool auto_condition_seamless = true) const {
        uint32_t actual_frames = std::min(frames, m_capacity_frames);
        auto clip = std::make_shared<AudioClip>(clip_name, m_sample_rate, 2, actual_frames);
        float* dst_l = clip->channel(0);
        float* dst_r = clip->channel(1);

        uint32_t current_head = m_write_head.load(std::memory_order_acquire);
        uint32_t start_idx = (current_head >= actual_frames)
                           ? (current_head - actual_frames)
                           : (m_capacity_frames + current_head - actual_frames);

        for (uint32_t i = 0; i < actual_frames; ++i) {
            uint32_t idx = (start_idx + i) % m_capacity_frames;
            dst_l[i] = m_buffer_l[idx];
            dst_r[i] = m_buffer_r[idx];
        }

        if (auto_condition_seamless) {
            LoopConditioner::condition_seamless(*clip, 128);
        }

        return clip;
    }

    void reset() noexcept {
        m_write_head.store(0, std::memory_order_relaxed);
        m_total_frames_recorded.store(0, std::memory_order_relaxed);
        m_quantized_recorded_frames.store(0, std::memory_order_relaxed);
        m_record_state.store(RecordState::Idle, std::memory_order_relaxed);
        m_quantized_clip.reset();
        std::fill(m_buffer_l.begin(), m_buffer_l.end(), 0.0f);
        std::fill(m_buffer_r.begin(), m_buffer_r.end(), 0.0f);
    }

private:
    uint32_t m_sample_rate{48000};
    uint32_t m_capacity_frames{480000}; // 10 seconds default

    std::atomic<TapSourceType> m_source_type{TapSourceType::Disabled};
    std::atomic<uint32_t> m_source_id{0};

    std::atomic<RecordMode> m_record_mode{RecordMode::RollingBuffer};
    std::atomic<RecordState> m_record_state{RecordState::Idle};
    std::atomic<QuantizeSyncMode> m_sync_mode{QuantizeSyncMode::BarSync};
    std::atomic<bool> m_auto_condition{true};

    std::atomic<uint32_t> m_write_head{0};
    std::atomic<uint64_t> m_total_frames_recorded{0};

    // Quantized mode variables
    uint32_t m_quantized_target_frames{0};
    std::atomic<uint32_t> m_quantized_recorded_frames{0};
    std::string m_quantized_name{"QuantizedBounce"};
    std::shared_ptr<AudioClip> m_quantized_clip{nullptr};

    std::vector<float> m_buffer_l;
    std::vector<float> m_buffer_r;
};

} // namespace audio_core::sampling
