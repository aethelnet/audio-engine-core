#pragma once

#include "audio_core/types.hpp"
#include "audio_core/clock/timeline_clock.hpp"
#include "audio_core/sampling/audio_clip.hpp"
#include "audio_core/dsp/resampler.hpp"
#include <memory>
#include <atomic>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <numbers>

namespace audio_core::sampling {

// ============================================================================
// WsolaStreamer: Real-Time Zero-Allocation WSOLA Time-Stretcher & Pitch-Shifter
//
// Algorithm: Waveform Similarity Overlap-Add (WSOLA, Verhelst & Roelands 1993)
// - Continuous Real-Time Time-Scale Modification (TSM)
// - Normalized cross-correlation phase alignment (Zero comb-filtering or phase cancel)
// - Decoupled Pitch Shifting via WSOLA dilation + C1 Hermite output resampling
// - Lock-free, zero-allocation ring-buffer architecture for RT audio threads
// ============================================================================
class WsolaStreamer {
public:
    static constexpr uint32_t kWindowSize   = 1024; // Grain window size (~21.3ms @ 48kHz)
    static constexpr uint32_t kHopSize      = 256;  // Synthesis hop (75% overlap for artifact-free COLA)
    static constexpr uint32_t kSearchRange  = 128;  // Correlation search window (+/- 128 samples, ~2.7ms)
    static constexpr uint32_t kBufferSize   = 8192; // Overlap-add ring buffer size (Power of 2)
    static constexpr uint32_t kBufferMask   = kBufferSize - 1;
    static constexpr uint32_t kChunkSize    = 256;  // Internal streaming sub-block size

    explicit WsolaStreamer(float sample_rate = 48000.0f) noexcept {
        init_window();
        reset();
    }

    void reset() noexcept {
        std::memset(m_synth_l, 0, sizeof(m_synth_l));
        std::memset(m_synth_r, 0, sizeof(m_synth_r));
        std::memset(m_weights, 0, sizeof(m_weights));
        m_write_idx = 0;
        m_read_pos = 0.0;
        m_last_cleared_idx = -10;
        m_ana_pos = 0.0;
        m_is_first_grain = true;
        m_effective_stretch = 1.0f;
    }

    void set_clip(std::shared_ptr<AudioClip> clip) noexcept {
        m_clip = std::move(clip);
        reset();
    }
    [[nodiscard]] std::shared_ptr<AudioClip> clip() const noexcept { return m_clip; }
    [[nodiscard]] bool has_clip() const noexcept { return m_clip != nullptr; }

    // Playhead control (analysis position in source clip)
    [[nodiscard]] double playhead() const noexcept {
        return m_ana_pos;
    }
    void set_playhead(double ph) noexcept {
        m_ana_pos = ph;
        reset_buffer_for_seek();
    }

    // Parameters
    void set_pitch_semitones(float st) noexcept {
        m_pitch_semitones.store(std::clamp(st, -48.0f, 48.0f), std::memory_order_relaxed);
    }
    [[nodiscard]] float pitch_semitones() const noexcept {
        return m_pitch_semitones.load(std::memory_order_relaxed);
    }

    // Stretch factor = duration_out / duration_in (e.g. 2.0 = half speed / twice as long)
    void set_stretch_factor(float stretch) noexcept {
        m_stretch_factor.store(std::clamp(stretch, 0.05f, 16.0f), std::memory_order_relaxed);
    }
    [[nodiscard]] float stretch_factor() const noexcept {
        return m_stretch_factor.load(std::memory_order_relaxed);
    }

    void set_loop(bool loop) noexcept {
        m_loop.store(loop, std::memory_order_relaxed);
    }
    [[nodiscard]] bool is_loop() const noexcept {
        return m_loop.load(std::memory_order_relaxed);
    }

    void set_loop_range(uint32_t start_frame, uint32_t end_frame) noexcept {
        m_loop_start.store(start_frame, std::memory_order_relaxed);
        m_loop_end.store(end_frame, std::memory_order_relaxed);
    }
    [[nodiscard]] uint32_t loop_start() const noexcept { return m_loop_start.load(std::memory_order_relaxed); }
    [[nodiscard]] uint32_t loop_end() const noexcept { return m_loop_end.load(std::memory_order_relaxed); }

    void set_beat_sync(bool sync) noexcept {
        m_beat_sync.store(sync, std::memory_order_relaxed);
    }
    [[nodiscard]] bool is_beat_sync() const noexcept {
        return m_beat_sync.load(std::memory_order_relaxed);
    }

    void set_bar_length(float bars) noexcept {
        m_bar_length.store(std::clamp(bars, 0.0f, 64.0f), std::memory_order_relaxed);
    }
    [[nodiscard]] float bar_length() const noexcept {
        return m_bar_length.load(std::memory_order_relaxed);
    }

    [[nodiscard]] float effective_stretch_ratio() const noexcept {
        return m_effective_stretch;
    }

    // Render with timeline clock
    void render(Sample* dst_l, Sample* dst_r, uint32_t frames, const clock::TimelineClock& clock) noexcept {
        render(dst_l, dst_r, frames, clock.sample_rate(), clock.bpm(), clock.is_playing());
    }

    // Real-Time Render Entry: Sub-chunked for arbitrary block sizes with zero buffer overrun
    void render(Sample* dst_l, Sample* dst_r, uint32_t frames,
                uint32_t session_sr, double session_bpm, bool is_playing) noexcept {
        if (!dst_l || !dst_r || frames == 0) return;
        if (!m_clip || m_clip->num_frames() == 0 || m_clip->num_channels() == 0) {
            std::memset(dst_l, 0, frames * sizeof(Sample));
            std::memset(dst_r, 0, frames * sizeof(Sample));
            return;
        }

        uint32_t processed = 0;
        while (processed < frames) {
            const uint32_t chunk = std::min(frames - processed, kChunkSize);
            render_chunk(dst_l + processed, dst_r + processed, chunk, session_sr, session_bpm, is_playing);
            processed += chunk;
        }
    }

private:
    void render_chunk(Sample* dst_l, Sample* dst_r, uint32_t chunk_frames,
                      uint32_t session_sr, double session_bpm, bool is_playing) noexcept {
        const uint32_t clip_sr = m_clip->sample_rate();
        const uint32_t clip_total_frames = m_clip->num_frames();
        const float* src_l = m_clip->channel(0);
        const float* src_r = (m_clip->num_channels() > 1) ? m_clip->channel(1) : src_l;

        uint32_t l_start = m_loop_start.load(std::memory_order_relaxed);
        uint32_t l_end = m_loop_end.load(std::memory_order_relaxed);
        if (l_end == 0 || l_end > clip_total_frames || l_end <= l_start) {
            l_end = clip_total_frames;
        }
        const uint32_t loop_len = (l_end > l_start) ? (l_end - l_start) : clip_total_frames;
        const bool loop = m_loop.load(std::memory_order_relaxed);

        // 1. Calculate Nominal Loop Tempo and Stretch Ratio
        double loop_bpm = m_clip->bpm();
        float user_bars = m_bar_length.load(std::memory_order_relaxed);
        if (user_bars > 0.0f) {
            const double dur_sec = static_cast<double>(loop_len) / static_cast<double>(clip_sr);
            if (dur_sec > 1e-4) {
                loop_bpm = (static_cast<double>(user_bars) * 4.0 / dur_sec) * 60.0;
            }
        }

        double sr_factor = (session_sr > 0)
            ? (static_cast<double>(clip_sr) / static_cast<double>(session_sr))
            : 1.0;

        double tempo_stretch = 1.0;
        if (m_beat_sync.load(std::memory_order_relaxed) && loop_bpm > 10.0 && session_bpm > 10.0) {
            tempo_stretch = loop_bpm / session_bpm;
        }

        double manual_stretch = static_cast<double>(m_stretch_factor.load(std::memory_order_relaxed));
        double target_time_stretch = sr_factor * tempo_stretch * manual_stretch;
        target_time_stretch = std::clamp(target_time_stretch, 0.05, 16.0);

        // 2. Decoupled Pitch Shift
        float semitones = m_pitch_semitones.load(std::memory_order_relaxed);
        double pitch_ratio = std::pow(2.0, static_cast<double>(semitones) / 12.0);
        pitch_ratio = std::clamp(pitch_ratio, 0.05, 16.0);

        // WSOLA overlap-add dilation factor:
        double wsola_stretch = std::clamp(target_time_stretch * pitch_ratio, 0.05, 16.0);
        m_effective_stretch = static_cast<float>(target_time_stretch);

        const double hop_a_ideal = static_cast<double>(kHopSize) / wsola_stretch;

        auto fetch_sample = [&](const float* src, int64_t pos) -> float {
            if (loop) {
                double rel = static_cast<double>(pos - l_start);
                rel = std::fmod(rel, static_cast<double>(loop_len));
                if (rel < 0.0) rel += static_cast<double>(loop_len);
                return src[l_start + static_cast<uint32_t>(rel)];
            } else {
                if (pos < static_cast<int64_t>(l_start) || pos >= static_cast<int64_t>(l_end)) return 0.0f;
                return src[pos];
            }
        };

        // 3. WSOLA Synthesis: Ensure ring buffer has enough grains for this sub-chunk
        const double required_synth_samples = static_cast<double>(chunk_frames) * pitch_ratio + static_cast<double>(kWindowSize);

        while (static_cast<double>(m_write_idx) - m_read_pos < required_synth_samples) {
            int64_t nom_cand = static_cast<int64_t>(std::round(m_ana_pos));
            int64_t best_cand = nom_cand;

            if (!m_is_first_grain) {
                float max_corr = -1e12f;
                int64_t min_cand = nom_cand - static_cast<int64_t>(kSearchRange);
                int64_t max_cand = nom_cand + static_cast<int64_t>(kSearchRange);

                for (int64_t cand = min_cand; cand <= max_cand; cand += 2) {
                    float dot = 0.0f;
                    float energy = 1e-4f;
                    for (uint32_t k = 0; k < kHopSize; k += 4) {
                        uint32_t s_slot = (m_write_idx + k) & kBufferMask;
                        float s_val = m_synth_l[s_slot] + m_synth_r[s_slot];
                        float c_val = fetch_sample(src_l, cand + k) + fetch_sample(src_r, cand + k);
                        dot += s_val * c_val;
                        energy += c_val * c_val;
                    }
                    float norm_corr = dot / std::sqrt(energy);
                    if (norm_corr > max_corr) {
                        max_corr = norm_corr;
                        best_cand = cand;
                    }
                }
            } else {
                m_is_first_grain = false;
            }

            // Overlap-add windowed grain across both stereo channels
            for (uint32_t n = 0; n < kWindowSize; ++n) {
                uint32_t slot = (m_write_idx + n) & kBufferMask;
                float w = m_window[n];
                m_synth_l[slot] += fetch_sample(src_l, best_cand + n) * w;
                m_synth_r[slot] += fetch_sample(src_r, best_cand + n) * w;
                m_weights[slot] += w;
            }

            m_write_idx += kHopSize;
            m_ana_pos += hop_a_ideal;

            // Handle analysis playhead loop wrapping
            if (loop) {
                const double d_start = static_cast<double>(l_start);
                const double d_end = static_cast<double>(l_end);
                const double d_len = static_cast<double>(loop_len);
                if (m_ana_pos >= d_end) {
                    m_ana_pos = d_start + std::fmod(m_ana_pos - d_start, d_len);
                } else if (m_ana_pos < d_start) {
                    double rel = std::fmod(m_ana_pos - d_start, d_len);
                    if (rel < 0.0) rel += d_len;
                    m_ana_pos = d_start + rel;
                }
            }
        }

        // 4. Output Generation with Hermite Spline Resampling for Continuous Pitch Shifting
        auto get_norm_sample = [&](uint32_t ch, int64_t idx) -> float {
            uint32_t s = static_cast<uint32_t>(idx) & kBufferMask;
            float w = m_weights[s];
            float inv_w = (w > 1e-4f) ? (1.0f / w) : 0.0f;
            return (ch == 0 ? m_synth_l[s] : m_synth_r[s]) * inv_w;
        };

        for (uint32_t i = 0; i < chunk_frames; ++i) {
            int64_t base = static_cast<int64_t>(std::floor(m_read_pos));
            float frac = static_cast<float>(m_read_pos - static_cast<double>(base));

            float val_l = dsp::hermite_interpolate(
                get_norm_sample(0, base - 1),
                get_norm_sample(0, base),
                get_norm_sample(0, base + 1),
                get_norm_sample(0, base + 2),
                frac);
            float val_r = dsp::hermite_interpolate(
                get_norm_sample(1, base - 1),
                get_norm_sample(1, base),
                get_norm_sample(1, base + 1),
                get_norm_sample(1, base + 2),
                frac);

            dst_l[i] = val_l;
            dst_r[i] = val_r;

            // Clear old samples behind the read cursor to prepare for future grains
            int64_t clear_idx = base - 2;
            if (clear_idx >= 0 && clear_idx > m_last_cleared_idx) {
                int64_t start_c = std::max<int64_t>(0, m_last_cleared_idx + 1);
                for (int64_t c = start_c; c <= clear_idx; ++c) {
                    uint32_t s = static_cast<uint32_t>(c) & kBufferMask;
                    m_synth_l[s] = 0.0f;
                    m_synth_r[s] = 0.0f;
                    m_weights[s] = 0.0f;
                }
                m_last_cleared_idx = clear_idx;
            }

            m_read_pos += pitch_ratio;
        }
    }

    void init_window() noexcept {
        for (uint32_t n = 0; n < kWindowSize; ++n) {
            // Periodic Hanning window
            m_window[n] = 0.5f * (1.0f - std::cos(2.0f * std::numbers::pi_v<float> * n / kWindowSize));
        }
    }

    void reset_buffer_for_seek() noexcept {
        std::memset(m_synth_l, 0, sizeof(m_synth_l));
        std::memset(m_synth_r, 0, sizeof(m_synth_r));
        std::memset(m_weights, 0, sizeof(m_weights));
        m_write_idx = 0;
        m_read_pos = 0.0;
        m_last_cleared_idx = -10;
        m_is_first_grain = true;
    }

    std::shared_ptr<AudioClip> m_clip{nullptr};

    alignas(16) float m_synth_l[kBufferSize]{};
    alignas(16) float m_synth_r[kBufferSize]{};
    alignas(16) float m_weights[kBufferSize]{};
    alignas(16) float m_window[kWindowSize]{};

    uint64_t m_write_idx{0};
    double m_read_pos{0.0};
    int64_t m_last_cleared_idx{-10};
    double m_ana_pos{0.0};
    bool m_is_first_grain{true};
    float m_effective_stretch{1.0f};

    std::atomic<float> m_stretch_factor{1.0f};
    std::atomic<float> m_pitch_semitones{0.0f};
    std::atomic<bool> m_loop{true};
    std::atomic<uint32_t> m_loop_start{0};
    std::atomic<uint32_t> m_loop_end{0};
    std::atomic<bool> m_beat_sync{true};
    std::atomic<float> m_bar_length{0.0f};
};

} // namespace audio_core::sampling
