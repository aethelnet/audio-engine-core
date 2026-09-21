#pragma once

#include "audio_core/types.hpp"
#include "audio_core/insert_slot.hpp"
#include <cmath>
#include <cstdint>
#include <algorithm>

namespace audio_core::dsp {

// ============================================================================
// Airwindows DeRez2: Continuous Fractional Bit & Sample-Rate Decimator
// Based on Chris Johnson's open-source DeRez2 algorithm.
// Features:
// 1. Continuous sample rate reduction with sub-sample overrun interpolation and edge-softening.
// 2. Continuous fractional bit depth quantization without integer stepping artifacts.
// 3. u-Law (mu-law) companding for vintage 8-bit/12-bit Fairlight & Mirage sampler tone.
// 4. Soft/Hard control interpolating intermediate dry reconstruction samples.
// ============================================================================
class DeRez : public IProcessor {
public:
    DeRez() = default;

    void init(uint32_t sample_rate) noexcept override {
        m_sample_rate = (sample_rate > 0) ? sample_rate : 48000;
        reset();
    }

    void reset() noexcept override {
        m_held_l = 0.0;
        m_held_r = 0.0;
        m_last_sample_l = 0.0;
        m_last_sample_r = 0.0;
        m_last_output_l = 0.0;
        m_last_output_r = 0.0;
        m_last_dry_l = 0.0;
        m_last_dry_r = 0.0;
        m_position = 0.0;
        m_inc_a = 1.0;
        m_inc_b = 0.0;
        m_fpd_l = 1.0;
        m_fpd_r = 1.0;
    }

    [[nodiscard]] const char* name() const noexcept override {
        return "Airwindows DeRez2";
    }

    // Parameters:
    // param 0: Rate [0.0 = extreme derez ~40Hz .. 1.0 = native sample rate bypass]
    // param 1: Resolution [0.0 = extreme 1-bit .. 1.0 = 24-bit linear clean]
    // param 2: Hard [0.0 = mu-law companded soft DAC .. 1.0 = harsh raw digital truncation]
    // param 3: Wet [0.0 = dry .. 1.0 = fully wet]
    void set_parameter(uint32_t index, float value) noexcept override {
        float v = std::clamp(value, 0.0f, 1.0f);
        switch (index) {
            case 0: m_rate = v; break;
            case 1: m_resolution = v; break;
            case 2: m_hard = v; break;
            case 3: m_wet = v; break;
            default: break;
        }
    }

    [[nodiscard]] float get_parameter(uint32_t index) const noexcept override {
        switch (index) {
            case 0: return m_rate;
            case 1: return m_resolution;
            case 2: return m_hard;
            case 3: return m_wet;
            default: return 0.0f;
        }
    }

    void process_stereo(Sample* left, Sample* right, uint32_t frames) noexcept override {
        if (!left || !right || frames == 0) return;

        double overallscale = static_cast<double>(m_sample_rate) / 44100.0;

        // Target A: Frequency/Rate reduction (cubic taper for musical control)
        double target_a = std::pow(static_cast<double>(m_rate), 3.0) + 0.0005;
        if (target_a > 1.0) target_a = 1.0;
        double soften = (1.0 + target_a) * 0.5;
        target_a /= overallscale;

        // Target B: Bit depth quantization step (cubic taper)
        double target_b = std::pow(1.0 - static_cast<double>(m_resolution), 3.0) / 3.0;

        const double hard = static_cast<double>(m_hard);
        const double wet  = static_cast<double>(m_wet);
        const double log256 = std::log(256.0);

        for (uint32_t i = 0; i < frames; ++i) {
            double input_l = static_cast<double>(left[i]);
            double input_r = static_cast<double>(right[i]);
            const double dry_l = input_l;
            const double dry_r = input_r;

            // 1-Pole continuous parameter smoothing to prevent click zipper noise
            m_inc_a = ((m_inc_a * 999.0) + target_a) / 1000.0;
            m_inc_b = ((m_inc_b * 999.0) + target_b) / 1000.0;

            // ----------------------------------------------------------------
            // 1. Continuous Sub-Sample Rate Reduction & Edge-Softening
            // ----------------------------------------------------------------
            m_position += m_inc_a;
            double out_l = m_held_l;
            double out_r = m_held_r;

            if (m_position > 1.0) {
                m_position -= 1.0;
                // Sub-sample interpolation between previous and current input
                m_held_l = (m_last_sample_l * m_position) + (input_l * (1.0 - m_position));
                out_l = (out_l * (1.0 - soften)) + (m_held_l * soften);

                m_held_r = (m_last_sample_r * m_position) + (input_r * (1.0 - m_position));
                out_r = (out_r * (1.0 - soften)) + (m_held_r * soften);
            }

            input_l = out_l;
            input_r = out_r;

            // ----------------------------------------------------------------
            // 2. Soft/Hard Frequency Reconstruction Interpolation
            // ----------------------------------------------------------------
            double temp_l = input_l;
            double temp_r = input_r;

            if (input_l != m_last_output_l) {
                temp_l = input_l;
                input_l = (input_l * hard) + (m_last_dry_l * (1.0 - hard));
                m_last_output_l = temp_l;
            } else {
                m_last_output_l = input_l;
            }

            if (input_r != m_last_output_r) {
                temp_r = input_r;
                input_r = (input_r * hard) + (m_last_dry_r * (1.0 - hard));
                m_last_output_r = temp_r;
            } else {
                m_last_output_r = input_r;
            }

            m_last_dry_l = dry_l;
            m_last_dry_r = dry_r;

            // ----------------------------------------------------------------
            // 3. u-Law (mu-law) Encoding (when hard < 1.0)
            // ----------------------------------------------------------------
            temp_l = input_l;
            temp_r = input_r;

            input_l = std::clamp(input_l, -1.0, 1.0);
            input_r = std::clamp(input_r, -1.0, 1.0);

            if (hard < 1.0) {
                double ulaw_l = (input_l > 0.0)
                    ? (std::log(1.0 + 255.0 * std::abs(input_l)) / log256)
                    : -(std::log(1.0 + 255.0 * std::abs(input_l)) / log256);
                double ulaw_r = (input_r > 0.0)
                    ? (std::log(1.0 + 255.0 * std::abs(input_r)) / log256)
                    : -(std::log(1.0 + 255.0 * std::abs(input_r)) / log256);

                input_l = (temp_l * hard) + (ulaw_l * (1.0 - hard));
                input_r = (temp_r * hard) + (ulaw_r * (1.0 - hard));
            }

            // ----------------------------------------------------------------
            // 4. Continuous Fractional Bit-Depth Decimation
            // ----------------------------------------------------------------
            if (m_inc_b > 0.0005) {
                if (input_l > 0.0) {
                    double offset = input_l;
                    while (offset > 0.0) offset -= m_inc_b;
                    input_l -= offset;
                } else if (input_l < 0.0) {
                    double offset = input_l;
                    while (offset < 0.0) offset += m_inc_b;
                    input_l -= offset;
                }

                if (input_r > 0.0) {
                    double offset = input_r;
                    while (offset > 0.0) offset -= m_inc_b;
                    input_r -= offset;
                } else if (input_r < 0.0) {
                    double offset = input_r;
                    while (offset < 0.0) offset += m_inc_b;
                    input_r -= offset;
                }

                input_l *= (1.0 - m_inc_b);
                input_r *= (1.0 - m_inc_b);
            }

            // ----------------------------------------------------------------
            // 5. u-Law (mu-law) Decoding (when hard < 1.0)
            // ----------------------------------------------------------------
            temp_l = input_l;
            temp_r = input_r;

            input_l = std::clamp(input_l, -1.0, 1.0);
            input_r = std::clamp(input_r, -1.0, 1.0);

            if (hard < 1.0) {
                double dec_l = (input_l > 0.0)
                    ? ((std::pow(256.0, std::abs(input_l)) - 1.0) / 255.0)
                    : -((std::pow(256.0, std::abs(input_l)) - 1.0) / 255.0);
                double dec_r = (input_r > 0.0)
                    ? ((std::pow(256.0, std::abs(input_r)) - 1.0) / 255.0)
                    : -((std::pow(256.0, std::abs(input_r)) - 1.0) / 255.0);

                input_l = (temp_l * hard) + (dec_l * (1.0 - hard));
                input_r = (temp_r * hard) + (dec_r * (1.0 - hard));
            }

            // ----------------------------------------------------------------
            // 6. Dry/Wet Mix
            // ----------------------------------------------------------------
            if (wet != 1.0) {
                input_l = (input_l * wet) + (dry_l * (1.0 - wet));
                input_r = (input_r * wet) + (dry_r * (1.0 - wet));
            }

            m_last_sample_l = dry_l;
            m_last_sample_r = dry_r;

            left[i]  = static_cast<float>(input_l);
            right[i] = static_cast<float>(input_r);
        }
    }

private:
    uint32_t m_sample_rate{48000};
    float m_rate{1.0f};       // Parameter A
    float m_resolution{1.0f}; // Parameter B
    float m_hard{1.0f};       // Parameter C (1.0 = raw digital, 0.0 = mu-law vintage)
    float m_wet{1.0f};        // Parameter D

    double m_held_l{0.0};
    double m_held_r{0.0};
    double m_last_sample_l{0.0};
    double m_last_sample_r{0.0};
    double m_last_output_l{0.0};
    double m_last_output_r{0.0};
    double m_last_dry_l{0.0};
    double m_last_dry_r{0.0};

    double m_position{0.0};
    double m_inc_a{1.0};
    double m_inc_b{0.0};
    double m_fpd_l{1.0};
    double m_fpd_r{1.0};
};

} // namespace audio_core::dsp
