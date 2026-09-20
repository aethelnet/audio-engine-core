#pragma once

#include "audio_core/types.hpp"
#include "audio_core/dsp/anti_aliasing_filter.hpp"
#include <cmath>
#include <cstdint>
#include <vector>
#include <array>
#include <algorithm>

namespace audio_core::dsp {

// ============================================================================
// Hermite Interpolation: 4-Point 3rd-Order Continuous Spline Kernel
// Provides C1 continuity (smooth first derivative) for broadcast-grade
// anti-aliased resampling with zero trigonometric overhead.
// ============================================================================
[[nodiscard]] inline float hermite_interpolate(float y0, float y1, float y2, float y3, float frac) noexcept {
    const float c0 = y1;
    const float c1 = 0.5f * (y2 - y0);
    const float c2 = y0 - 2.5f * y1 + 2.0f * y2 - 0.5f * y3;
    const float c3 = 0.5f * (y3 - y0) + 1.5f * (y1 - y2);
    return ((c3 * frac + c2) * frac + c1) * frac + c0;
}

// 4-point boundary-safe sample fetch from planar buffer (clamped at boundaries)
[[nodiscard]] inline float fetch_sample_clamped(const float* buffer, int64_t index, int64_t max_frames) noexcept {
    if (!buffer || max_frames <= 0) return 0.0f;
    int64_t clamped = std::clamp<int64_t>(index, 0, max_frames - 1);
    return buffer[clamped];
}

// 4-point loop-safe sample fetch from planar buffer (wrapped modulo length)
[[nodiscard]] inline float fetch_sample_wrapped(const float* buffer, int64_t index, int64_t max_frames) noexcept {
    if (!buffer || max_frames <= 0) return 0.0f;
    int64_t wrapped = index % max_frames;
    if (wrapped < 0) wrapped += max_frames;
    return buffer[wrapped];
}

// Sample a continuous position from planar buffer using Hermite spline (clamped boundaries)
[[nodiscard]] inline float sample_hermite(const float* buffer, double position, int64_t max_frames) noexcept {
    if (!buffer || max_frames <= 0) return 0.0f;
    int64_t idx = static_cast<int64_t>(std::floor(position));
    float frac = static_cast<float>(position - static_cast<double>(idx));

    float y0 = fetch_sample_clamped(buffer, idx - 1, max_frames);
    float y1 = fetch_sample_clamped(buffer, idx,     max_frames);
    float y2 = fetch_sample_clamped(buffer, idx + 1, max_frames);
    float y3 = fetch_sample_clamped(buffer, idx + 2, max_frames);

    return hermite_interpolate(y0, y1, y2, y3, frac);
}

// Sample a continuous position from planar buffer using Hermite spline (wrapped loop seam)
[[nodiscard]] inline float sample_hermite_wrapped(const float* buffer, double position, int64_t max_frames) noexcept {
    if (!buffer || max_frames <= 0) return 0.0f;
    int64_t idx = static_cast<int64_t>(std::floor(position));
    float frac = static_cast<float>(position - static_cast<double>(idx));

    float y0 = fetch_sample_wrapped(buffer, idx - 1, max_frames);
    float y1 = fetch_sample_wrapped(buffer, idx,     max_frames);
    float y2 = fetch_sample_wrapped(buffer, idx + 1, max_frames);
    float y3 = fetch_sample_wrapped(buffer, idx + 2, max_frames);

    return hermite_interpolate(y0, y1, y2, y3, frac);
}

// ============================================================================
// StreamResampler: Arbitrary Ratio Real-Time Stereo Stream Resampler
// Bridges differing clock domains (e.g. 44.1kHz Android vs 48kHz / 96kHz / 192kHz Engine)
// Includes cascaded minimum-phase ultrasonic decimation filter to eliminate alias foldback
// ============================================================================
class StreamResampler {
public:
    static constexpr size_t kMaxFilterBlock = 4096;

    StreamResampler(uint32_t in_rate = 48000, uint32_t out_rate = 48000, bool anti_aliasing = false)
        : m_in_rate(in_rate), m_out_rate(out_rate), m_filter(in_rate, out_rate) {
        m_filter.set_enabled(anti_aliasing);
        update_ratio();
    }

    void set_rates(uint32_t in_rate, uint32_t out_rate) noexcept {
        if (in_rate > 0 && out_rate > 0) {
            m_in_rate = in_rate;
            m_out_rate = out_rate;
            update_ratio();
            m_filter.set_rates(in_rate, out_rate);
        }
    }

    void set_anti_aliasing(bool enable) noexcept {
        m_filter.set_enabled(enable);
    }

    [[nodiscard]] bool anti_aliasing() const noexcept { return m_filter.is_enabled(); }
    [[nodiscard]] uint32_t input_rate() const noexcept { return m_in_rate; }
    [[nodiscard]] uint32_t output_rate() const noexcept { return m_out_rate; }
    [[nodiscard]] double ratio() const noexcept { return m_ratio; }
    [[nodiscard]] const UltrasonicAntiAliasingFilter& filter() const noexcept { return m_filter; }
    [[nodiscard]] UltrasonicAntiAliasingFilter& filter() noexcept { return m_filter; }

    void reset() noexcept {
        m_history_l.fill(0.0f);
        m_history_r.fill(0.0f);
        m_phase = 0.0;
        m_filter.reset();
    }

    // Resample block from in_buf to out_buf
    // in_frames: number of input frames available
    // out_frames: number of output frames to produce
    uint32_t process_stereo(const float* in_l, const float* in_r, uint32_t in_frames,
                            float* out_l, float* out_r, uint32_t out_frames) noexcept {
        if (!in_l || !in_r || !out_l || !out_r || out_frames == 0) return 0;

        // Identity fast-path if rates match exactly
        if (m_in_rate == m_out_rate && in_frames >= out_frames) {
            std::copy_n(in_l, out_frames, out_l);
            std::copy_n(in_r, out_frames, out_r);
            return out_frames;
        }

        const float* src_l = in_l;
        const float* src_r = in_r;

        // Apply ultrasonic decimation filter prior to downsampling if active
        if (m_filter.is_active()) {
            uint32_t filter_frames = std::min<uint32_t>(in_frames, static_cast<uint32_t>(kMaxFilterBlock));
            m_filter.process_stereo(in_l, in_r, m_filter_buf_l.data(), m_filter_buf_r.data(), filter_frames);
            src_l = m_filter_buf_l.data();
            src_r = m_filter_buf_r.data();
        }

        const int64_t max_in = static_cast<int64_t>(in_frames);

        for (uint32_t i = 0; i < out_frames; ++i) {
            int64_t idx = static_cast<int64_t>(std::floor(m_phase));
            float frac = static_cast<float>(m_phase - static_cast<double>(idx));

            // Fetch 4 points for Left
            float y0_l = get_sample(src_l, idx - 1, max_in, 0);
            float y1_l = get_sample(src_l, idx,     max_in, 0);
            float y2_l = get_sample(src_l, idx + 1, max_in, 0);
            float y3_l = get_sample(src_l, idx + 2, max_in, 0);
            out_l[i] = hermite_interpolate(y0_l, y1_l, y2_l, y3_l, frac);

            // Fetch 4 points for Right
            float y0_r = get_sample(src_r, idx - 1, max_in, 1);
            float y1_r = get_sample(src_r, idx,     max_in, 1);
            float y2_r = get_sample(src_r, idx + 1, max_in, 1);
            float y3_r = get_sample(src_r, idx + 2, max_in, 1);
            out_r[i] = hermite_interpolate(y0_r, y1_r, y2_r, y3_r, frac);

            m_phase += m_ratio;
        }

        // Maintain phase continuity across blocks
        if (m_phase >= static_cast<double>(in_frames)) {
            m_phase -= static_cast<double>(in_frames);
            // Save last 3 samples into history from filtered source
            for (int k = 0; k < 3; ++k) {
                int64_t src_idx = static_cast<int64_t>(in_frames) - 3 + k;
                m_history_l[k] = (src_idx >= 0) ? src_l[src_idx] : 0.0f;
                m_history_r[k] = (src_idx >= 0) ? src_r[src_idx] : 0.0f;
            }
        }

        return out_frames;
    }

private:
    void update_ratio() noexcept {
        m_ratio = static_cast<double>(m_in_rate) / static_cast<double>(m_out_rate);
    }

    inline float get_sample(const float* buffer, int64_t idx, int64_t max_in, int ch) const noexcept {
        if (idx < 0) {
            int64_t hist_idx = 3 + idx;
            if (hist_idx >= 0 && hist_idx < 3) {
                return (ch == 0) ? m_history_l[hist_idx] : m_history_r[hist_idx];
            }
            return buffer[0];
        }
        if (idx >= max_in) {
            return buffer[max_in - 1];
        }
        return buffer[idx];
    }

    uint32_t m_in_rate{48000};
    uint32_t m_out_rate{48000};
    double m_ratio{1.0};
    double m_phase{0.0};

    UltrasonicAntiAliasingFilter m_filter{48000, 48000};
    std::array<float, kMaxFilterBlock> m_filter_buf_l{};
    std::array<float, kMaxFilterBlock> m_filter_buf_r{};

    std::array<float, 3> m_history_l{0.0f, 0.0f, 0.0f};
    std::array<float, 3> m_history_r{0.0f, 0.0f, 0.0f};
};

// ============================================================================
// BufferedResampler: Lock-Free Push-Pull Ring Buffer Resampler
// Bridges differing clock domains & buffer sizes (e.g. 48kHz / 256 frames engine -> 44.1kHz / 192 frames AAudio)
// Zero allocations in audio thread, ring buffer of 8192 stereo frames
// Incorporates minimum-phase ultrasonic decimation filter to eliminate alias foldback
// ============================================================================
class BufferedResampler {
public:
    static constexpr size_t kCapacity = 8192; // Stereo frames capacity (~170ms @ 48kHz)

    BufferedResampler(uint32_t in_rate = 48000, uint32_t out_rate = 48000, bool anti_aliasing = false)
        : m_in_rate(in_rate), m_out_rate(out_rate), m_filter(in_rate, out_rate) {
        m_filter.set_enabled(anti_aliasing);
        update_ratio();
        reset();
    }

    void set_rates(uint32_t in_rate, uint32_t out_rate) noexcept {
        if (in_rate > 0 && out_rate > 0) {
            m_in_rate = in_rate;
            m_out_rate = out_rate;
            update_ratio();
            m_filter.set_rates(in_rate, out_rate);
        }
    }

    void set_anti_aliasing(bool enable) noexcept {
        m_filter.set_enabled(enable);
    }

    [[nodiscard]] bool anti_aliasing() const noexcept { return m_filter.is_enabled(); }
    [[nodiscard]] uint32_t input_rate() const noexcept { return m_in_rate; }
    [[nodiscard]] uint32_t output_rate() const noexcept { return m_out_rate; }
    [[nodiscard]] double ratio() const noexcept { return m_ratio; }
    [[nodiscard]] const UltrasonicAntiAliasingFilter& filter() const noexcept { return m_filter; }
    [[nodiscard]] UltrasonicAntiAliasingFilter& filter() noexcept { return m_filter; }

    void reset() noexcept {
        m_write_head = 0;
        m_read_phase = 0.0;
        m_fifo_l.fill(0.0f);
        m_fifo_r.fill(0.0f);
        m_available_frames = 0;
        m_filter.reset();
    }

    // Push input audio frames into FIFO (with ultrasonic anti-aliasing decimation filter if downsampling)
    uint32_t push_stereo(const float* in_l, const float* in_r, uint32_t frames) noexcept {
        if (!in_l || !in_r || frames == 0) return 0;
        uint32_t to_write = std::min<uint32_t>(frames, static_cast<uint32_t>(kCapacity - m_available_frames));

        if (m_filter.is_active()) {
            for (uint32_t i = 0; i < to_write; ++i) {
                size_t idx = (m_write_head + i) % kCapacity;
                float fl = 0.0f, fr = 0.0f;
                m_filter.process_sample(in_l[i], in_r[i], fl, fr);
                m_fifo_l[idx] = fl;
                m_fifo_r[idx] = fr;
            }
        } else {
            for (uint32_t i = 0; i < to_write; ++i) {
                size_t idx = (m_write_head + i) % kCapacity;
                m_fifo_l[idx] = in_l[i];
                m_fifo_r[idx] = in_r[i];
            }
        }

        m_write_head = (m_write_head + to_write) % kCapacity;
        m_available_frames += to_write;
        return to_write;
    }

    // Pull resampled output frames from FIFO
    uint32_t pull_stereo(float* out_l, float* out_r, uint32_t frames) noexcept {
        if (!out_l || !out_r || frames == 0) return 0;

        // How many input frames are needed for this output request?
        double needed_input = static_cast<double>(frames) * m_ratio;
        if (static_cast<double>(m_available_frames) + 1e-4 < needed_input) {
            // Not enough input frames available
            return 0;
        }

        const int64_t cap = static_cast<int64_t>(kCapacity);

        for (uint32_t i = 0; i < frames; ++i) {
            out_l[i] = sample_hermite_wrapped(m_fifo_l.data(), m_read_phase, cap);
            out_r[i] = sample_hermite_wrapped(m_fifo_r.data(), m_read_phase, cap);

            m_read_phase += m_ratio;
            if (m_read_phase >= static_cast<double>(kCapacity)) {
                m_read_phase -= static_cast<double>(kCapacity);
            }
        }

        uint32_t consumed = static_cast<uint32_t>(std::floor(needed_input + 1e-4));
        m_available_frames = (m_available_frames >= consumed) ? (m_available_frames - consumed) : 0;
        return frames;
    }

    [[nodiscard]] uint32_t available_input_frames() const noexcept {
        return m_available_frames;
    }

    [[nodiscard]] uint32_t available_output_frames() const noexcept {
        if (m_ratio <= 0.0) return 0;
        double avail = static_cast<double>(m_available_frames) / m_ratio;
        return static_cast<uint32_t>(std::max(0.0, std::floor(avail + 1e-6)));
    }

private:
    void update_ratio() noexcept {
        m_ratio = static_cast<double>(m_in_rate) / static_cast<double>(m_out_rate);
    }

    uint32_t m_in_rate{48000};
    uint32_t m_out_rate{48000};
    double m_ratio{1.0};

    size_t m_write_head{0};
    double m_read_phase{0.0};
    uint32_t m_available_frames{0};

    UltrasonicAntiAliasingFilter m_filter{48000, 48000};
    std::array<float, kCapacity> m_fifo_l{};
    std::array<float, kCapacity> m_fifo_r{};
};

} // namespace audio_core::dsp
