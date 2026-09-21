#pragma once

#include "audio_core/dsp/resampler.hpp"
#include "audio_core/sampling/wav_reader.hpp"
#include <string>
#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <span>

namespace audio_core::sampling {

struct AudioSlice {
    uint32_t slice_id{0};
    uint32_t start_frame{0};
    uint32_t end_frame{0};
    float gain{1.0f};
};

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

    [[nodiscard]] const std::vector<AudioSlice>& slices() const noexcept { return m_slices; }
    [[nodiscard]] std::vector<AudioSlice>& slices() noexcept { return m_slices; }

    // Slice evenly across musical grid (e.g. 16 slices for 16th notes)
    void slice_grid(uint32_t num_slices) {
        m_slices.clear();
        if (num_slices == 0 || m_frames == 0) return;
        m_slices.reserve(num_slices);
        const uint32_t slice_len = m_frames / num_slices;
        for (uint32_t i = 0; i < num_slices; ++i) {
            uint32_t start = i * slice_len;
            uint32_t end = (i == num_slices - 1) ? m_frames : ((i + 1) * slice_len);
            m_slices.push_back(AudioSlice{
                .slice_id = i,
                .start_frame = start,
                .end_frame = end,
                .gain = 1.0f
            });
        }
    }

    // Slice at detected transient markers
    void slice_at_markers(const std::vector<uint32_t>& onsets, uint32_t min_slice_frames = 64) {
        m_slices.clear();
        if (m_frames == 0) return;
        if (onsets.empty()) {
            m_slices.push_back(AudioSlice{0, 0, m_frames, 1.0f});
            return;
        }

        std::vector<uint32_t> sorted_onsets = onsets;
        std::sort(sorted_onsets.begin(), sorted_onsets.end());

        // If the first onset is at or very close to the start (e.g. downbeat of beat 1),
        // it marks the beginning of slice 0, not a split point from 0.
        size_t start_idx = 0;
        uint32_t current_start = 0;
        if (sorted_onsets[0] < min_slice_frames) {
            current_start = 0;
            start_idx = 1;
        }

        uint32_t slice_id = 0;
        for (size_t i = start_idx; i < sorted_onsets.size(); ++i) {
            uint32_t onset = sorted_onsets[i];
            if (onset > current_start + min_slice_frames && onset < m_frames) {
                m_slices.push_back(AudioSlice{
                    .slice_id = slice_id++,
                    .start_frame = current_start,
                    .end_frame = onset,
                    .gain = 1.0f
                });
                current_start = onset;
            }
        }
        if (current_start < m_frames) {
            m_slices.push_back(AudioSlice{
                .slice_id = slice_id,
                .start_frame = current_start,
                .end_frame = m_frames,
                .gain = 1.0f
            });
        }
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

    // Read a specific slice for MPC pad triggers or tracker-style rearrangements
    uint32_t read_slice(uint32_t slice_idx, uint64_t& slice_playhead, float* dst_l, float* dst_r, uint32_t frames_to_read, bool loop = false) const noexcept {
        if (slice_idx >= m_slices.size() || m_channels == 0) {
            for (uint32_t i = 0; i < frames_to_read; ++i) {
                dst_l[i] = 0.0f;
                dst_r[i] = 0.0f;
            }
            return 0;
        }

        const auto& slice = m_slices[slice_idx];
        const uint32_t slice_len = (slice.end_frame > slice.start_frame) ? (slice.end_frame - slice.start_frame) : 0;
        if (slice_len == 0) return 0;

        const float* src_l = m_data[0].data();
        const float* src_r = (m_channels > 1) ? m_data[1].data() : src_l;
        const float gain = slice.gain;

        uint32_t frames_rendered = 0;
        for (uint32_t i = 0; i < frames_to_read; ++i) {
            if (slice_playhead >= slice_len) {
                if (loop) {
                    slice_playhead %= slice_len;
                } else {
                    for (uint32_t j = i; j < frames_to_read; ++j) {
                        dst_l[j] = 0.0f;
                        dst_r[j] = 0.0f;
                    }
                    return frames_rendered;
                }
            }
            uint32_t global_idx = slice.start_frame + static_cast<uint32_t>(slice_playhead);
            dst_l[i] = src_l[global_idx] * gain;
            dst_r[i] = src_r[global_idx] * gain;
            slice_playhead++;
            frames_rendered++;
        }
        return frames_rendered;
    }

    [[nodiscard]] double playback_ratio(uint32_t target_sample_rate, double pitch_ratio = 1.0) const noexcept {
        return (target_sample_rate > 0)
            ? (static_cast<double>(m_sample_rate) / static_cast<double>(target_sample_rate) * pitch_ratio)
            : 1.0;
    }

    // Lock-free resampled read with continuous Hermite spline interpolation & zero pitch drift
    uint32_t read_resampled(double& playhead, uint32_t target_sample_rate,
                            float* dst_l, float* dst_r, uint32_t frames_to_read,
                            bool loop = true, double pitch_ratio = 1.0) const noexcept {
        if (m_frames == 0 || m_channels == 0 || !dst_l || !dst_r || frames_to_read == 0) {
            if (dst_l && dst_r) {
                for (uint32_t i = 0; i < frames_to_read; ++i) {
                    dst_l[i] = 0.0f;
                    dst_r[i] = 0.0f;
                }
            }
            return 0;
        }

        const double ratio = playback_ratio(target_sample_rate, pitch_ratio);
        const float* src_l = m_data[0].data();
        const float* src_r = (m_channels > 1) ? m_data[1].data() : src_l;
        const double max_f = static_cast<double>(m_frames);

        uint32_t frames_rendered = 0;
        for (uint32_t i = 0; i < frames_to_read; ++i) {
            if (playhead >= max_f) {
                if (loop) {
                    playhead = std::fmod(playhead, max_f);
                } else {
                    for (uint32_t j = i; j < frames_to_read; ++j) {
                        dst_l[j] = 0.0f;
                        dst_r[j] = 0.0f;
                    }
                    return frames_rendered;
                }
            } else if (playhead < 0.0) {
                if (loop) {
                    playhead = std::fmod(playhead, max_f);
                    if (playhead < 0.0) playhead += max_f;
                } else {
                    for (uint32_t j = i; j < frames_to_read; ++j) {
                        dst_l[j] = 0.0f;
                        dst_r[j] = 0.0f;
                    }
                    return frames_rendered;
                }
            }

            dst_l[i] = loop ? dsp::sample_hermite_wrapped(src_l, playhead, m_frames)
                            : dsp::sample_hermite(src_l, playhead, m_frames);
            dst_r[i] = loop ? dsp::sample_hermite_wrapped(src_r, playhead, m_frames)
                            : dsp::sample_hermite(src_r, playhead, m_frames);

            playhead += ratio;
            frames_rendered++;
        }
        return frames_rendered;
    }

    // Read a specific slice resampled with continuous Hermite spline interpolation
    uint32_t read_slice_resampled(uint32_t slice_idx, double& slice_playhead,
                                  uint32_t target_sample_rate, float* dst_l, float* dst_r,
                                  uint32_t frames_to_read, bool loop = false,
                                  double pitch_ratio = 1.0) const noexcept {
        if (slice_idx >= m_slices.size() || m_channels == 0 || !dst_l || !dst_r || frames_to_read == 0) {
            if (dst_l && dst_r) {
                for (uint32_t i = 0; i < frames_to_read; ++i) {
                    dst_l[i] = 0.0f;
                    dst_r[i] = 0.0f;
                }
            }
            return 0;
        }

        const auto& slice = m_slices[slice_idx];
        const uint32_t slice_len = (slice.end_frame > slice.start_frame) ? (slice.end_frame - slice.start_frame) : 0;
        if (slice_len == 0) return 0;

        const double ratio = playback_ratio(target_sample_rate, pitch_ratio);
        const float* src_l = m_data[0].data();
        const float* src_r = (m_channels > 1) ? m_data[1].data() : src_l;
        const float gain = slice.gain;
        const double max_len = static_cast<double>(slice_len);

        uint32_t frames_rendered = 0;
        for (uint32_t i = 0; i < frames_to_read; ++i) {
            if (slice_playhead >= max_len) {
                if (loop) {
                    slice_playhead = std::fmod(slice_playhead, max_len);
                } else {
                    for (uint32_t j = i; j < frames_to_read; ++j) {
                        dst_l[j] = 0.0f;
                        dst_r[j] = 0.0f;
                    }
                    return frames_rendered;
                }
            } else if (slice_playhead < 0.0) {
                if (loop) {
                    slice_playhead = std::fmod(slice_playhead, max_len);
                    if (slice_playhead < 0.0) slice_playhead += max_len;
                } else {
                    for (uint32_t j = i; j < frames_to_read; ++j) {
                        dst_l[j] = 0.0f;
                        dst_r[j] = 0.0f;
                    }
                    return frames_rendered;
                }
            }

            double global_pos = static_cast<double>(slice.start_frame) + slice_playhead;
            dst_l[i] = dsp::sample_hermite(src_l, global_pos, m_frames) * gain;
            dst_r[i] = dsp::sample_hermite(src_r, global_pos, m_frames) * gain;

            slice_playhead += ratio;
            frames_rendered++;
        }
        return frames_rendered;
    }

    // ========================================================================
    // Audio File I/O & Mastering Operations
    // ========================================================================

    // Load audio from standard WAV file (16/24/32-bit PCM or 32-bit Float)
    bool load_from_wav(const std::string& filepath) {
        std::vector<std::vector<float>> channels;
        uint32_t sample_rate = 0;
        if (!WavReader::load_wav(filepath, channels, sample_rate)) {
            return false;
        }

        m_channels = static_cast<uint32_t>(channels.size());
        m_frames = static_cast<uint32_t>(channels[0].size());
        m_sample_rate = sample_rate;
        m_data = std::move(channels);
        m_slices.clear();

        // Extract base filename for clip name
        size_t last_slash = filepath.find_last_of("/\\");
        m_name = (last_slash != std::string::npos) ? filepath.substr(last_slash + 1) : filepath;
        return true;
    }

    // Save audio clip to standard WAV file
    bool save_to_wav(const std::string& filepath, uint16_t bits_per_sample = 24) const {
        if (m_frames == 0 || m_channels == 0) return false;
        return WavReader::save_wav(filepath, channel(0), channel(1), m_frames, m_sample_rate, bits_per_sample);
    }

    // Peak Normalization: scales clip so highest peak equals target_peak linear
    // target_peak = 0.9885f corresponds to -0.1 dBFS (broadcast ceiling)
    void normalize_peak(float target_peak = 0.9885f) noexcept {
        if (m_frames == 0 || m_channels == 0 || target_peak <= 0.0f) return;

        float max_val = 0.0f;
        for (uint32_t ch = 0; ch < m_channels; ++ch) {
            const float* src = m_data[ch].data();
            for (uint32_t i = 0; i < m_frames; ++i) {
                float val = std::abs(src[i]);
                if (val > max_val) max_val = val;
            }
        }

        if (max_val < 1e-7f) return; // Digital silence

        const float gain = target_peak / max_val;
        for (uint32_t ch = 0; ch < m_channels; ++ch) {
            float* src = m_data[ch].data();
            for (uint32_t i = 0; i < m_frames; ++i) {
                src[i] *= gain;
            }
        }
    }

    // RMS Loudness Normalization: scales clip to target RMS dBFS (e.g. -14 dBFS for K-14)
    // max_peak_ceiling prevents harsh digital clipping if crest factor is very high
    void normalize_rms(float target_rms_db = -14.0f, float max_peak_ceiling = 0.9885f) noexcept {
        if (m_frames == 0 || m_channels == 0) return;

        double sum_sq = 0.0;
        size_t total_samples = static_cast<size_t>(m_channels) * m_frames;
        for (uint32_t ch = 0; ch < m_channels; ++ch) {
            const float* src = m_data[ch].data();
            for (uint32_t i = 0; i < m_frames; ++i) {
                sum_sq += static_cast<double>(src[i]) * static_cast<double>(src[i]);
            }
        }

        double cur_rms = std::sqrt(sum_sq / static_cast<double>(total_samples));
        if (cur_rms < 1e-7) return;

        double target_rms_lin = std::pow(10.0, static_cast<double>(target_rms_db) / 20.0);
        float gain = static_cast<float>(target_rms_lin / cur_rms);

        // First pass: check what maximum peak would become
        float max_post_peak = 0.0f;
        for (uint32_t ch = 0; ch < m_channels; ++ch) {
            const float* src = m_data[ch].data();
            for (uint32_t i = 0; i < m_frames; ++i) {
                float v = std::abs(src[i]) * gain;
                if (v > max_post_peak) max_post_peak = v;
            }
        }

        // Clamp gain if it would violate the peak ceiling
        if (max_post_peak > max_peak_ceiling && max_post_peak > 1e-6f) {
            gain *= (max_peak_ceiling / max_post_peak);
        }

        // Apply gain
        for (uint32_t ch = 0; ch < m_channels; ++ch) {
            float* src = m_data[ch].data();
            for (uint32_t i = 0; i < m_frames; ++i) {
                src[i] *= gain;
            }
        }
    }

    // Remove DC Offset: subtract channel mean to center waveform at 0.0
    void remove_dc_offset() noexcept {
        if (m_frames == 0 || m_channels == 0) return;
        for (uint32_t ch = 0; ch < m_channels; ++ch) {
            float* src = m_data[ch].data();
            double sum = 0.0;
            for (uint32_t i = 0; i < m_frames; ++i) {
                sum += static_cast<double>(src[i]);
            }
            float dc = static_cast<float>(sum / m_frames);
            for (uint32_t i = 0; i < m_frames; ++i) {
                src[i] -= dc;
            }
        }
    }

private:
    std::string m_name{"UntitledClip"};
    uint32_t m_sample_rate{48000};
    uint32_t m_channels{2};
    uint32_t m_frames{0};
    double m_bpm{120.0};
    std::vector<std::vector<float>> m_data;
    std::vector<AudioSlice> m_slices;
};

} // namespace audio_core::sampling
