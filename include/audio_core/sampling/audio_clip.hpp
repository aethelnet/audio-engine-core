#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <algorithm>
#include <span>

namespace audio_core::sampling {

// ============================================================================
// AudioClip: Memory-Backed Planar Audio Buffer
// Used for retroactive sampling, loop playback, stem bouncing, and timeline clips
// ============================================================================
class AudioClip {
public:
    AudioClip() = default;

    AudioClip(std::string name, uint32_t sample_rate, uint32_t channels, uint32_t frames)
        : m_name(std::move(name)),
          m_sample_rate(sample_rate),
          m_channels(channels),
          m_frames(frames),
          m_data(channels, std::vector<float>(frames, 0.0f)) {}

    [[nodiscard]] const std::string& name() const noexcept { return m_name; }
    void set_name(std::string name) noexcept { m_name = std::move(name); }

    [[nodiscard]] uint32_t sample_rate() const noexcept { return m_sample_rate; }
    [[nodiscard]] uint32_t num_channels() const noexcept { return m_channels; }
    [[nodiscard]] uint32_t num_frames() const noexcept { return m_frames; }

    [[nodiscard]] double bpm() const noexcept { return m_bpm; }
    void set_bpm(double bpm) noexcept { m_bpm = bpm; }

    [[nodiscard]] float* channel(uint32_t ch) noexcept {
        return (ch < m_channels) ? m_data[ch].data() : nullptr;
    }

    [[nodiscard]] const float* channel(uint32_t ch) const noexcept {
        return (ch < m_channels) ? m_data[ch].data() : nullptr;
    }

    // Lock-free read of audio frames with optional looping
    uint32_t read(uint64_t& playhead, float* dst_l, float* dst_r, uint32_t frames_to_read, bool loop = true) const noexcept {
        if (m_frames == 0 || m_channels == 0) {
            for (uint32_t i = 0; i < frames_to_read; ++i) {
                dst_l[i] = 0.0f;
                dst_r[i] = 0.0f;
            }
            return 0;
        }

        const float* src_l = m_data[0].data();
        const float* src_r = (m_channels > 1) ? m_data[1].data() : src_l;

        uint32_t frames_rendered = 0;
        for (uint32_t i = 0; i < frames_to_read; ++i) {
            if (playhead >= m_frames) {
                if (loop) {
                    playhead %= m_frames;
                } else {
                    for (uint32_t j = i; j < frames_to_read; ++j) {
                        dst_l[j] = 0.0f;
                        dst_r[j] = 0.0f;
                    }
                    return frames_rendered;
                }
            }
            dst_l[i] = src_l[playhead];
            dst_r[i] = src_r[playhead];
            playhead++;
            frames_rendered++;
        }
        return frames_rendered;
    }

private:
    std::string m_name{"UntitledClip"};
    uint32_t m_sample_rate{48000};
    uint32_t m_channels{2};
    uint32_t m_frames{0};
    double m_bpm{120.0};
    std::vector<std::vector<float>> m_data;
};

} // namespace audio_core::sampling
