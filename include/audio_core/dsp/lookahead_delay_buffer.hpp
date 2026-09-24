#pragma once

#include "audio_core/types.hpp"
#include <array>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <cstddef>

namespace audio_core::dsp {

// ============================================================================
// LookaheadDelayBuffer: Zero-Allocation Stereo Circular Delay Line
// ============================================================================
// - Fixed-capacity zero-heap-allocation ring buffer (default: 256 samples).
// - Designed specifically for zero-smear lookahead in real-time dynamics,
//   compressors, peak limiters, and dynamic noise reducers.
// - Features:
//   1. Bit-exact integer sample delay: read(D) == in[n - D] with 0 numerical error.
//   2. Sub-sample 4-point Hermite spline interpolation for smooth modulation.
//   3. Non-destructive peeking (inspect future/past without modifying write head).
//   4. Denormal flush and memory reset.
// ============================================================================
template <size_t MaxCapacity = 256>
class LookaheadDelayBuffer {
public:
    static_assert((MaxCapacity & (MaxCapacity - 1)) == 0, "MaxCapacity must be a power of 2 for fast bitwise masking");

    LookaheadDelayBuffer() noexcept {
        set_delay_frames(32);
        reset();
    }

    explicit LookaheadDelayBuffer(uint32_t delay_frames) noexcept {
        set_delay_frames(delay_frames);
        reset();
    }

    void reset() noexcept {
        m_buffer_l.fill(0.0f);
        m_buffer_r.fill(0.0f);
        m_write_pos = 0;
    }

    void set_delay_frames(uint32_t frames) noexcept {
        m_delay_frames = std::clamp(frames, 0u, static_cast<uint32_t>(MaxCapacity - 1));
    }

    [[nodiscard]] uint32_t delay_frames() const noexcept {
        return m_delay_frames;
    }

    [[nodiscard]] float delay_ms(uint32_t sample_rate) const noexcept {
        if (sample_rate == 0) return 0.0f;
        return (static_cast<float>(m_delay_frames) * 1000.0f) / static_cast<float>(sample_rate);
    }

    [[nodiscard]] static constexpr size_t capacity() noexcept {
        return MaxCapacity;
    }

    // Push stereo frame into delay buffer head
    inline void push(float in_l, float in_r) noexcept {
        // Anti-denormal flush
        if (std::abs(in_l) < 1e-15f) in_l = 0.0f;
        if (std::abs(in_r) < 1e-15f) in_r = 0.0f;

        m_buffer_l[m_write_pos] = in_l;
        m_buffer_r[m_write_pos] = in_r;
        m_write_pos = (m_write_pos + 1) & kMask;
    }

    // Read delayed sample from exactly `delay_samples` ago (0 <= delay_samples < MaxCapacity)
    // When delay_samples == 0: returns the sample that was just pushed!
    inline void read(uint32_t delay_samples, float& out_l, float& out_r) const noexcept {
        const uint32_t clamped_delay = std::min(delay_samples, static_cast<uint32_t>(MaxCapacity - 1));
        const size_t read_idx = (m_write_pos + MaxCapacity - 1 - clamped_delay) & kMask;
        out_l = m_buffer_l[read_idx];
        out_r = m_buffer_r[read_idx];
    }

    // Read with the default configured lookahead delay
    inline void read_delayed(float& out_l, float& out_r) const noexcept {
        read(m_delay_frames, out_l, out_r);
    }

    // Peek sample at offset from newest (0 = newest pushed sample, 1 = 1 sample ago)
    inline void peek(uint32_t offset_from_newest, float& out_l, float& out_r) const noexcept {
        read(offset_from_newest, out_l, out_r);
    }

    // Peek the most recent sample pushed to the buffer
    inline void peek_latest(float& out_l, float& out_r) const noexcept {
        read(0, out_l, out_r);
    }

    // Fractional read using 4-point, 3rd-order Hermite spline interpolation
    inline void read_fractional(float delay_samples, float& out_l, float& out_r) const noexcept {
        const float clamped_delay = std::clamp(delay_samples, 0.0f, static_cast<float>(MaxCapacity - 4));
        const int32_t int_delay = static_cast<int32_t>(clamped_delay);
        const float frac = clamped_delay - static_cast<float>(int_delay);

        // Indices: xm1 (x[-1]), x0 (x[0]), x1 (x[1]), x2 (x[2]) relative to read pos
        const size_t idx0 = (m_write_pos + MaxCapacity - 1 - int_delay) & kMask;
        const size_t idx_m1 = (idx0 + 1) & kMask;
        const size_t idx1 = (idx0 + MaxCapacity - 1) & kMask;
        const size_t idx2 = (idx0 + MaxCapacity - 2) & kMask;

        out_l = hermite_interpolate(m_buffer_l[idx_m1], m_buffer_l[idx0], m_buffer_l[idx1], m_buffer_l[idx2], frac);
        out_r = hermite_interpolate(m_buffer_r[idx_m1], m_buffer_r[idx0], m_buffer_r[idx1], m_buffer_r[idx2], frac);
    }

    // Streaming helper: pushes input and immediately reads the delayed output
    inline void process_sample(float in_l, float in_r, float& out_l, float& out_r) noexcept {
        push(in_l, in_r);
        read_delayed(out_l, out_r);
    }

    // Block streaming in-place or out-of-place
    void process_block(const float* in_l, const float* in_r, float* out_l, float* out_r, uint32_t frames) noexcept {
        if (!in_l || !in_r || !out_l || !out_r || frames == 0) return;
        for (uint32_t i = 0; i < frames; ++i) {
            process_sample(in_l[i], in_r[i], out_l[i], out_r[i]);
        }
    }

private:
    static constexpr size_t kMask = MaxCapacity - 1;

    alignas(16) std::array<float, MaxCapacity> m_buffer_l{};
    alignas(16) std::array<float, MaxCapacity> m_buffer_r{};
    size_t m_write_pos{0};
    uint32_t m_delay_frames{32};

    // 4-point, 3rd-order Hermite spline interpolation
    static inline float hermite_interpolate(float y0, float y1, float y2, float y3, float mu) noexcept {
        const float a0 = -0.5f * y0 + 1.5f * y1 - 1.5f * y2 + 0.5f * y3;
        const float a1 = y0 - 2.5f * y1 + 2.0f * y2 - 0.5f * y3;
        const float a2 = -0.5f * y0 + 0.5f * y2;
        const float a3 = y1;
        return ((a0 * mu + a1) * mu + a2) * mu + a3;
    }
};

} // namespace audio_core::dsp
