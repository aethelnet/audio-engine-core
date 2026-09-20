#pragma once

#include "audio_core/types.hpp"
#include <cmath>
#include <numbers>
#include <algorithm>

namespace audio_core::dsp {

enum class FilterType : uint8_t {
    Lowpass = 0,
    Highpass,
    Bandpass,
    Notch
};

class BiquadFilter {
public:
    BiquadFilter() = default;

    void init(uint32_t sample_rate) noexcept {
        m_sample_rate = sample_rate;
        reset();
        calculate_coefficients();
    }

    void set_type(FilterType type) noexcept {
        m_type = type;
        calculate_coefficients();
    }

    void set_cutoff(float cutoff_hz) noexcept {
        float nyquist = static_cast<float>(m_sample_rate) * 0.5f;
        m_cutoff = std::clamp(cutoff_hz, 20.0f, nyquist * 0.95f);
        calculate_coefficients();
    }

    void set_q(float q) noexcept {
        m_q = std::clamp(q, 0.1f, 25.0f);
        calculate_coefficients();
    }

    void reset() noexcept {
        m_s1 = 0.0f;
        m_s2 = 0.0f;
    }

    [[nodiscard]] Sample process_sample(Sample in) noexcept {
        // Direct Form II Transposed:
        // y[n] = b0 * x[n] + s1
        // s1 = b1 * x[n] - a1 * y[n] + s2
        // s2 = b2 * x[n] - a2 * y[n]
        Sample out = (m_b0 * in) + m_s1;
        m_s1 = (m_b1 * in) - (m_a1 * out) + m_s2;
        m_s2 = (m_b2 * in) - (m_a2 * out);

        // Denormal flushing
        if (std::abs(m_s1) < 1e-15f) m_s1 = 0.0f;
        if (std::abs(m_s2) < 1e-15f) m_s2 = 0.0f;

        return out;
    }

    void process_block(const Sample* input, Sample* output, uint32_t num_frames) noexcept {
        for (uint32_t i = 0; i < num_frames; ++i) {
            output[i] = process_sample(input[i]);
        }
    }

    void process_in_place(Sample* buffer, uint32_t num_frames) noexcept {
        for (uint32_t i = 0; i < num_frames; ++i) {
            buffer[i] = process_sample(buffer[i]);
        }
    }

private:
    void calculate_coefficients() noexcept {
        if (m_sample_rate == 0) return;

        const float omega = 2.0f * std::numbers::pi_v<float> * m_cutoff / static_cast<float>(m_sample_rate);
        const float sin_omega = std::sin(omega);
        const float cos_omega = std::cos(omega);
        const float alpha = sin_omega / (2.0f * m_q);

        float a0 = 1.0f + alpha;

        switch (m_type) {
            case FilterType::Lowpass:
                m_b0 = ((1.0f - cos_omega) * 0.5f) / a0;
                m_b1 = (1.0f - cos_omega) / a0;
                m_b2 = ((1.0f - cos_omega) * 0.5f) / a0;
                m_a1 = (-2.0f * cos_omega) / a0;
                m_a2 = (1.0f - alpha) / a0;
                break;

            case FilterType::Highpass:
                m_b0 = ((1.0f + cos_omega) * 0.5f) / a0;
                m_b1 = -(1.0f + cos_omega) / a0;
                m_b2 = ((1.0f + cos_omega) * 0.5f) / a0;
                m_a1 = (-2.0f * cos_omega) / a0;
                m_a2 = (1.0f - alpha) / a0;
                break;

            case FilterType::Bandpass:
                m_b0 = (alpha) / a0;
                m_b1 = 0.0f;
                m_b2 = (-alpha) / a0;
                m_a1 = (-2.0f * cos_omega) / a0;
                m_a2 = (1.0f - alpha) / a0;
                break;

            case FilterType::Notch:
                m_b0 = 1.0f / a0;
                m_b1 = (-2.0f * cos_omega) / a0;
                m_b2 = 1.0f / a0;
                m_a1 = (-2.0f * cos_omega) / a0;
                m_a2 = (1.0f - alpha) / a0;
                break;
        }
    }

    uint32_t m_sample_rate{kDefaultSampleRate};
    FilterType m_type{FilterType::Lowpass};
    float m_cutoff{1000.0f};
    float m_q{0.7071f}; // Butterworth Q

    // Normalized coefficients (divided by a0)
    float m_b0{1.0f};
    float m_b1{0.0f};
    float m_b2{0.0f};
    float m_a1{0.0f};
    float m_a2{0.0f};

    // Filter states (Direct Form II Transposed)
    float m_s1{0.0f};
    float m_s2{0.0f};
};

} // namespace audio_core::dsp
