#pragma once

#include "audio_core/insert_slot.hpp"
#include <cmath>
#include <numbers>
#include <algorithm>

namespace audio_core::dsp {

// ============================================================================
// Baxandall: Resonant-Protected Shelving EQ (Airwindows)
// Classic James Baxandall (1952) shelving curves wrapped in a non-linear
// Console5 carrier wave (sin -> Biquads -> asin). High boosts saturate smoothly
// into analog warmth instead of harsh digital clipping.
// ============================================================================
class Baxandall : public IProcessor {
public:
    Baxandall() = default;

    void init(uint32_t sample_rate) noexcept override {
        m_sample_rate = sample_rate > 0 ? sample_rate : 48000;
        update_coefficients();
        reset();
    }

    void reset() noexcept override {
        m_bass_x1_l = m_bass_x2_l = m_bass_y1_l = m_bass_y2_l = 0.0f;
        m_bass_x1_r = m_bass_x2_r = m_bass_y1_r = m_bass_y2_r = 0.0f;
        m_treb_x1_l = m_treb_x2_l = m_treb_y1_l = m_treb_y2_l = 0.0f;
        m_treb_x1_r = m_treb_x2_r = m_treb_y1_r = m_treb_y2_r = 0.0f;
    }

    [[nodiscard]] const char* name() const noexcept override {
        return "Airwindows Baxandall EQ";
    }

    // Parameters:
    // 0: Bass (-1.0 to 1.0 -> -15dB to +15dB)
    // 1: Treble (-1.0 to 1.0 -> -15dB to +15dB)
    // 2: Output Trim (0.0 to 2.0)
    void set_parameter(uint32_t index, float value) noexcept override {
        switch (index) {
            case 0:
                m_bass_gain_db = std::clamp(value, -15.0f, 15.0f);
                update_coefficients();
                break;
            case 1:
                m_treble_gain_db = std::clamp(value, -15.0f, 15.0f);
                update_coefficients();
                break;
            case 2:
                m_out_trim = std::clamp(value, 0.0f, 2.0f);
                break;
            default:
                break;
        }
    }

    [[nodiscard]] float get_parameter(uint32_t index) const noexcept override {
        switch (index) {
            case 0: return m_bass_gain_db;
            case 1: return m_treble_gain_db;
            case 2: return m_out_trim;
            default: return 0.0f;
        }
    }

    [[nodiscard]] uint32_t parameter_count() const noexcept override { return 3; }
    [[nodiscard]] const char* parameter_name(uint32_t index) const noexcept override {
        switch (index) {
            case 0: return "Bass (dB)";
            case 1: return "Treble (dB)";
            case 2: return "Trim";
            default: return "Param";
        }
    }
    [[nodiscard]] float parameter_min(uint32_t index) const noexcept override {
        switch (index) {
            case 0: return -15.0f;
            case 1: return -15.0f;
            case 2: return 0.0f;
            default: return 0.0f;
        }
    }
    [[nodiscard]] float parameter_max(uint32_t index) const noexcept override {
        switch (index) {
            case 0: return 15.0f;
            case 1: return 15.0f;
            case 2: return 2.0f;
            default: return 1.0f;
        }
    }
    [[nodiscard]] float parameter_default(uint32_t index) const noexcept override {
        switch (index) {
            case 0: return 0.0f;
            case 1: return 0.0f;
            case 2: return 1.0f;
            default: return 0.0f;
        }
    }

    void process_stereo(Sample* left, Sample* right, uint32_t frames) noexcept override {
        for (uint32_t i = 0; i < frames; ++i) {
            // 1. Enter Console carrier wave
            float in_l = std::clamp(left[i], -1.5707963f, 1.5707963f);
            float in_r = std::clamp(right[i], -1.5707963f, 1.5707963f);
            float w_l = std::sin(in_l);
            float w_r = std::sin(in_r);

            // 2. Low Shelf Biquad Filter
            float b_out_l = (m_b0_b * w_l) + (m_b1_b * m_bass_x1_l) + (m_b2_b * m_bass_x2_l)
                            - (m_a1_b * m_bass_y1_l) - (m_a2_b * m_bass_y2_l);
            m_bass_x2_l = m_bass_x1_l; m_bass_x1_l = w_l;
            m_bass_y2_l = m_bass_y1_l; m_bass_y1_l = b_out_l;

            float b_out_r = (m_b0_b * w_r) + (m_b1_b * m_bass_x1_r) + (m_b2_b * m_bass_x2_r)
                            - (m_a1_b * m_bass_y1_r) - (m_a2_b * m_bass_y2_r);
            m_bass_x2_r = m_bass_x1_r; m_bass_x1_r = w_r;
            m_bass_y2_r = m_bass_y1_r; m_bass_y1_r = b_out_r;

            // 3. High Shelf Biquad Filter
            float t_out_l = (m_b0_t * b_out_l) + (m_b1_t * m_treb_x1_l) + (m_b2_t * m_treb_x2_l)
                            - (m_a1_t * m_treb_y1_l) - (m_a2_t * m_treb_y2_l);
            m_treb_x2_l = m_treb_x1_l; m_treb_x1_l = b_out_l;
            m_treb_y2_l = m_treb_y1_l; m_treb_y1_l = t_out_l;

            float t_out_r = (m_b0_t * b_out_r) + (m_b1_t * m_treb_x1_r) + (m_b2_t * m_treb_x2_r)
                            - (m_a1_t * m_treb_y1_r) - (m_a2_t * m_treb_y2_r);
            m_treb_x2_r = m_treb_x1_r; m_treb_x1_r = b_out_r;
            m_treb_y2_r = m_treb_y1_r; m_treb_y1_r = t_out_r;

            // 4. Exit Console carrier wave (asin decoding + soft saturation limit)
            float dec_l = std::clamp(t_out_l, -1.0f, 1.0f);
            float dec_r = std::clamp(t_out_r, -1.0f, 1.0f);
            left[i] = std::asin(dec_l) * m_out_trim;
            right[i] = std::asin(dec_r) * m_out_trim;
        }
    }

private:
    void update_coefficients() noexcept {
        constexpr float pi = std::numbers::pi_v<float>;
        float fs = static_cast<float>(m_sample_rate);

        // Low Shelf: 150 Hz
        {
            float f0 = 150.0f;
            float a_gain = std::pow(10.0f, m_bass_gain_db / 40.0f);
            float w0 = 2.0f * pi * f0 / fs;
            float cos_w = std::cos(w0);
            float sin_w = std::sin(w0);
            float alpha = sin_w / (2.0f * 0.7071f);
            float two_sqrt_a_alpha = 2.0f * std::sqrt(a_gain) * alpha;

            float a0 = (a_gain + 1.0f) + ((a_gain - 1.0f) * cos_w) + two_sqrt_a_alpha;
            m_b0_b = (a_gain * ((a_gain + 1.0f) - ((a_gain - 1.0f) * cos_w) + two_sqrt_a_alpha)) / a0;
            m_b1_b = (2.0f * a_gain * ((a_gain - 1.0f) - ((a_gain + 1.0f) * cos_w))) / a0;
            m_b2_b = (a_gain * ((a_gain + 1.0f) - ((a_gain - 1.0f) * cos_w) - two_sqrt_a_alpha)) / a0;
            m_a1_b = (-2.0f * ((a_gain - 1.0f) + ((a_gain + 1.0f) * cos_w))) / a0;
            m_a2_b = ((a_gain + 1.0f) + ((a_gain - 1.0f) * cos_w) - two_sqrt_a_alpha) / a0;
        }

        // High Shelf: 8000 Hz
        {
            float f0 = 8000.0f;
            float a_gain = std::pow(10.0f, m_treble_gain_db / 40.0f);
            float w0 = 2.0f * pi * f0 / fs;
            float cos_w = std::cos(w0);
            float sin_w = std::sin(w0);
            float alpha = sin_w / (2.0f * 0.7071f);
            float two_sqrt_a_alpha = 2.0f * std::sqrt(a_gain) * alpha;

            float a0 = (a_gain + 1.0f) - ((a_gain - 1.0f) * cos_w) + two_sqrt_a_alpha;
            m_b0_t = (a_gain * ((a_gain + 1.0f) + ((a_gain - 1.0f) * cos_w) + two_sqrt_a_alpha)) / a0;
            m_b1_t = (-2.0f * a_gain * ((a_gain - 1.0f) + ((a_gain + 1.0f) * cos_w))) / a0;
            m_b2_t = (a_gain * ((a_gain + 1.0f) + ((a_gain - 1.0f) * cos_w) - two_sqrt_a_alpha)) / a0;
            m_a1_t = (2.0f * ((a_gain - 1.0f) - ((a_gain + 1.0f) * cos_w))) / a0;
            m_a2_t = ((a_gain + 1.0f) - ((a_gain - 1.0f) * cos_w) - two_sqrt_a_alpha) / a0;
        }
    }

    uint32_t m_sample_rate{48000};
    float m_bass_gain_db{0.0f};
    float m_treble_gain_db{0.0f};
    float m_out_trim{1.0f};

    // Low Shelf coefficients
    float m_b0_b{1.0f}, m_b1_b{0.0f}, m_b2_b{0.0f}, m_a1_b{0.0f}, m_a2_b{0.0f};
    float m_bass_x1_l{0.0f}, m_bass_x2_l{0.0f}, m_bass_y1_l{0.0f}, m_bass_y2_l{0.0f};
    float m_bass_x1_r{0.0f}, m_bass_x2_r{0.0f}, m_bass_y1_r{0.0f}, m_bass_y2_r{0.0f};

    // High Shelf coefficients
    float m_b0_t{1.0f}, m_b1_t{0.0f}, m_b2_t{0.0f}, m_a1_t{0.0f}, m_a2_t{0.0f};
    float m_treb_x1_l{0.0f}, m_treb_x2_l{0.0f}, m_treb_y1_l{0.0f}, m_treb_y2_l{0.0f};
    float m_treb_x1_r{0.0f}, m_treb_x2_r{0.0f}, m_treb_y1_r{0.0f}, m_treb_y2_r{0.0f};
};

} // namespace audio_core::dsp
