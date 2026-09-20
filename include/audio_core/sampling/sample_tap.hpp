#pragma once

#include "audio_core/sampling/audio_clip.hpp"
#include <vector>
#include <string>
#include <atomic>
#include <memory>
#include <cstdint>
#include <algorithm>

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
// SampleTap: Non-Blocking Audio Tap & Bounce Recorder
// Taps into any point of the MixerGraph: Track Pre/Post FX, Submix Bus, or Master
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
    [[nodiscard]] uint32_t capacity_frames() const noexcept { return m_capacity_frames; }

    // Arm a quantized bounce of exact length (e.g. 1 bar, 4 bars)
    void arm_quantized_bounce(uint32_t target_frames, std::string clip_name = "QuantizedBounce") {
        m_quantized_name = std::move(clip_name);
        m_quantized_target_frames = target_frames;
        m_quantized_recorded_frames.store(0, std::memory_order_relaxed);
        m_quantized_clip.reset();
        m_record_mode.store(RecordMode::QuantizedBounce, std::memory_order_relaxed);
        m_record_state.store(RecordState::Armed, std::memory_order_release);
    }

    // Set rolling buffer mode for retroactive capture
    void set_rolling_mode() noexcept {
        m_record_mode.store(RecordMode::RollingBuffer, std::memory_order_relaxed);
        m_record_state.store(RecordState::Recording, std::memory_order_release);
    }

    [[nodiscard]] RecordState record_state() const noexcept {
        return m_record_state.load(std::memory_order_acquire);
    }

    // RT Audio Thread Record Hook (Lock-free, zero allocation)
    void record(const float* left, const float* right, uint32_t frames) noexcept {
        if (!is_active() || frames == 0) return;

        const RecordMode mode = m_record_mode.load(std::memory_order_relaxed);
        RecordState state = m_record_state.load(std::memory_order_acquire);

        if (mode == RecordMode::QuantizedBounce) {
            if (state == RecordState::Armed) {
                // First frame transition to Recording
                m_record_state.store(RecordState::Recording, std::memory_order_relaxed);
                state = RecordState::Recording;
            }
            if (state == RecordState::Recording) {
                uint32_t recorded = m_quantized_recorded_frames.load(std::memory_order_relaxed);
                uint32_t to_write = std::min(frames, m_quantized_target_frames - recorded);

                for (uint32_t i = 0; i < to_write; ++i) {
                    uint32_t idx = (recorded + i) % m_capacity_frames;
                    m_buffer_l[idx] = left[i];
                    m_buffer_r[idx] = right[i];
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

    // Retrieve the completed quantized bounce clip
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
            m_quantized_clip = clip;
        }
        return m_quantized_clip;
    }

    // Capture the most recent N frames (e.g. last 4 bars of live jam) from the rolling buffer
    [[nodiscard]] std::shared_ptr<AudioClip> capture_retroactive(uint32_t frames, const std::string& clip_name = "RetroactiveBounce") const {
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
