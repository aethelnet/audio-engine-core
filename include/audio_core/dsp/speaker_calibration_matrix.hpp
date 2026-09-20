#pragma once

#include "audio_core/types.hpp"
#include "audio_core/dsp/biquad_filter.hpp"
#include "audio_core/dsp/resampler.hpp"
#include "audio_core/dsp/multichannel_bus.hpp"
#include "audio_core/analysis/measurement_engine.hpp"
#include <cmath>
#include <vector>
#include <array>
#include <algorithm>
#include <memory>
#include <cstring>

namespace audio_core::dsp {

// ============================================================================
// SpeakerCalibrationChannel: Per-Speaker Delay, Room EQ, and Alignment Strip
// ============================================================================
class SpeakerCalibrationChannel {
public:
    static constexpr size_t kMaxDelayFrames = 8192; // 8192 samples = 170.6 ms @ 48kHz (up to ~58m distance)
    static constexpr size_t kMaxNotches = 8;        // Up to 8 room mode correction filters per speaker

    explicit SpeakerCalibrationChannel(uint32_t sample_rate = 48000) noexcept
        : m_sample_rate(sample_rate ? sample_rate : 48000) {
        m_delay_buffer.assign(kMaxDelayFrames, 0.0f);
        reset();
    }

    void set_sample_rate(uint32_t sr) noexcept {
        if (sr == 0 || sr == m_sample_rate) return;
        m_sample_rate = sr;
        for (size_t i = 0; i < m_active_notches; ++i) {
            m_notches[i].init(m_sample_rate);
        }
        reset();
    }

    void reset() noexcept {
        std::fill(m_delay_buffer.begin(), m_delay_buffer.end(), 0.0f);
        m_write_pos = 0;
        for (auto& n : m_notches) {
            n.reset();
        }
    }

    void set_delay_samples(float delay_frames) noexcept {
        m_delay_samples = std::clamp(delay_frames, 0.0f, static_cast<float>(kMaxDelayFrames - 4));
    }
    [[nodiscard]] float delay_samples() const noexcept { return m_delay_samples; }

    void set_delay_ms(float delay_ms) noexcept {
        set_delay_samples(delay_ms * 0.001f * static_cast<float>(m_sample_rate));
    }
    [[nodiscard]] float delay_ms() const noexcept {
        return (m_delay_samples / static_cast<float>(m_sample_rate)) * 1000.0f;
    }

    void set_trim_gain_db(float gain_db) noexcept {
        m_trim_gain_linear = std::pow(10.0f, gain_db / 20.0f);
    }
    [[nodiscard]] float trim_gain_db() const noexcept {
        return 20.0f * std::log10(std::max(1e-5f, m_trim_gain_linear));
    }

    void set_invert_polarity(bool invert) noexcept { m_invert_polarity = invert; }
    [[nodiscard]] bool invert_polarity() const noexcept { return m_invert_polarity; }

    void set_mute(bool mute) noexcept { m_mute = mute; }
    [[nodiscard]] bool is_muted() const noexcept { return m_mute; }

    void set_solo(bool solo) noexcept { m_solo = solo; }
    [[nodiscard]] bool is_solo() const noexcept { return m_solo; }

    // Add a room mode notch filter
    bool add_notch(float frequency_hz, float q_factor) noexcept {
        if (m_active_notches >= kMaxNotches) return false;
        auto& notch = m_notches[m_active_notches++];
        notch.init(m_sample_rate);
        notch.set_type(FilterType::Notch);
        notch.set_cutoff(frequency_hz);
        notch.set_q(q_factor);
        return true;
    }

    void clear_notches() noexcept {
        m_active_notches = 0;
    }

    [[nodiscard]] size_t num_notches() const noexcept { return m_active_notches; }

    // Process a block of samples in-place
    void process_block(float* buffer, uint32_t frames) noexcept {
        if (!buffer || frames == 0) return;

        if (m_mute) {
            std::memset(buffer, 0, frames * sizeof(float));
            return;
        }

        const float gain = m_invert_polarity ? -m_trim_gain_linear : m_trim_gain_linear;
        const float delay = m_delay_samples;
        const size_t buf_mask = kMaxDelayFrames - 1;

        for (uint32_t i = 0; i < frames; ++i) {
            float in_sample = buffer[i];

            // 1. Write sample to circular delay ring
            m_delay_buffer[m_write_pos] = in_sample;

            // 2. Read with Hermite Spline fractional delay
            float delayed_sample = 0.0f;
            if (delay <= 1e-4f) {
                delayed_sample = in_sample;
            } else {
                const float read_pos = static_cast<float>(m_write_pos) - delay;
                int64_t i0 = static_cast<int64_t>(std::floor(read_pos));
                float frac = read_pos - static_cast<float>(i0);

                // Sample 4 points around fractional index
                size_t idx_m1 = static_cast<size_t>(i0 - 1 + static_cast<int64_t>(kMaxDelayFrames)) & buf_mask;
                size_t idx_0  = static_cast<size_t>(i0 + static_cast<int64_t>(kMaxDelayFrames)) & buf_mask;
                size_t idx_p1 = static_cast<size_t>(i0 + 1 + static_cast<int64_t>(kMaxDelayFrames)) & buf_mask;
                size_t idx_p2 = static_cast<size_t>(i0 + 2 + static_cast<int64_t>(kMaxDelayFrames)) & buf_mask;

                float ym1 = m_delay_buffer[idx_m1];
                float y0  = m_delay_buffer[idx_0];
                float yp1 = m_delay_buffer[idx_p1];
                float yp2 = m_delay_buffer[idx_p2];

                delayed_sample = hermite_interpolate(ym1, y0, yp1, yp2, frac);
            }

            m_write_pos = (m_write_pos + 1) & buf_mask;

            // 3. Room Mode Correction Filterbank
            float filtered_sample = delayed_sample;
            for (size_t n = 0; n < m_active_notches; ++n) {
                filtered_sample = m_notches[n].process_sample(filtered_sample);
            }

            // 4. Output Trim & Polarity
            buffer[i] = filtered_sample * gain;
        }
    }

private:
    uint32_t m_sample_rate{48000};
    float m_delay_samples{0.0f};
    float m_trim_gain_linear{1.0f};
    bool m_invert_polarity{false};
    bool m_mute{false};
    bool m_solo{false};

    std::vector<float> m_delay_buffer;
    size_t m_write_pos{0};

    std::array<BiquadFilter, kMaxNotches> m_notches{};
    size_t m_active_notches{0};
};

// ============================================================================
// SpeakerCalibrationMatrix: Multi-Channel Concert Hall & Immersion Matrix
// Manages N speaker outputs (up to 128 channels) for WFS, HOA and Dante/AES67
// ============================================================================
class SpeakerCalibrationMatrix {
public:
    static constexpr uint32_t kMaxChannels = 128;

    explicit SpeakerCalibrationMatrix(uint32_t num_speakers = 16, uint32_t sample_rate = 48000) noexcept
        : m_sample_rate(sample_rate ? sample_rate : 48000),
          m_num_speakers(std::clamp(num_speakers, 1u, kMaxChannels)) {
        m_channels.reserve(m_num_speakers);
        for (uint32_t i = 0; i < m_num_speakers; ++i) {
            m_channels.emplace_back(m_sample_rate);
        }
    }

    void set_sample_rate(uint32_t sr) noexcept {
        if (sr == 0 || sr == m_sample_rate) return;
        m_sample_rate = sr;
        for (auto& ch : m_channels) {
            ch.set_sample_rate(m_sample_rate);
        }
    }
    [[nodiscard]] uint32_t sample_rate() const noexcept { return m_sample_rate; }

    void resize(uint32_t num_speakers) noexcept {
        m_num_speakers = std::clamp(num_speakers, 1u, kMaxChannels);
        m_channels.clear();
        m_channels.reserve(m_num_speakers);
        for (uint32_t i = 0; i < m_num_speakers; ++i) {
            m_channels.emplace_back(m_sample_rate);
        }
    }
    [[nodiscard]] uint32_t num_speakers() const noexcept { return m_num_speakers; }

    [[nodiscard]] SpeakerCalibrationChannel& speaker(uint32_t speaker_idx) noexcept {
        return m_channels[std::min(speaker_idx, m_num_speakers - 1)];
    }
    [[nodiscard]] const SpeakerCalibrationChannel& speaker(uint32_t speaker_idx) const noexcept {
        return m_channels[std::min(speaker_idx, m_num_speakers - 1)];
    }

    // Automatically align time-of-flight based on measured TOF vector in milliseconds
    void calibrate_time_of_flight(const float* measured_tof_ms, uint32_t count) noexcept {
        if (!measured_tof_ms || count == 0) return;
        uint32_t n = std::min(count, m_num_speakers);

        // Find maximum distance (furthest speaker)
        float max_tof = 0.0f;
        for (uint32_t i = 0; i < n; ++i) {
            if (measured_tof_ms[i] > max_tof) max_tof = measured_tof_ms[i];
        }

        // Delay closer speakers so all wavefronts arrive simultaneously
        for (uint32_t i = 0; i < n; ++i) {
            float added_delay_ms = max_tof - measured_tof_ms[i];
            m_channels[i].set_delay_ms(added_delay_ms);
        }
    }

    // Automatically apply detected room modes as notch filters to all/selected speakers
    void apply_room_modes(const std::vector<analysis::RoomMode>& modes, uint32_t speaker_mask = 0xFFFFFFFF) noexcept {
        for (uint32_t c = 0; c < m_num_speakers; ++c) {
            if ((speaker_mask & (1ULL << c)) != 0) {
                m_channels[c].clear_notches();
                for (const auto& mode : modes) {
                    m_channels[c].add_notch(mode.frequency_hz, mode.q_factor);
                }
            }
        }
    }

    // Process a MultiChannelBus in-place
    void process_multichannel_bus(MultiChannelBus& bus, uint32_t frames) noexcept {
        const uint32_t channels = std::min(bus.num_channels(), m_num_speakers);
        const uint32_t f = std::min(bus.num_frames(), frames);

        // Check solo state
        bool any_solo = false;
        for (uint32_t c = 0; c < channels; ++c) {
            if (m_channels[c].is_solo()) {
                any_solo = true;
                break;
            }
        }

        for (uint32_t c = 0; c < channels; ++c) {
            float* channel_data = bus.channel(c);
            if (any_solo && !m_channels[c].is_solo()) {
                std::memset(channel_data, 0, f * sizeof(float));
                continue;
            }
            m_channels[c].process_block(channel_data, f);
        }
    }

    // Process planar float pointers
    void process_planar(float* const* channel_ptrs, uint32_t num_channels, uint32_t frames) noexcept {
        if (!channel_ptrs || num_channels == 0 || frames == 0) return;
        const uint32_t channels = std::min(num_channels, m_num_speakers);

        bool any_solo = false;
        for (uint32_t c = 0; c < channels; ++c) {
            if (m_channels[c].is_solo()) {
                any_solo = true;
                break;
            }
        }

        for (uint32_t c = 0; c < channels; ++c) {
            float* data = channel_ptrs[c];
            if (!data) continue;
            if (any_solo && !m_channels[c].is_solo()) {
                std::memset(data, 0, frames * sizeof(float));
                continue;
            }
            m_channels[c].process_block(data, frames);
        }
    }

    void reset() noexcept {
        for (auto& ch : m_channels) ch.reset();
    }

private:
    uint32_t m_sample_rate{48000};
    uint32_t m_num_speakers{16};
    std::vector<SpeakerCalibrationChannel> m_channels;
};

} // namespace audio_core::dsp
