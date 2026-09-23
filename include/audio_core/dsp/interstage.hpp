#pragma once

#include "audio_core/insert_slot.hpp"
#include <cmath>
#include <cstdint>

namespace audio_core::dsp {

// ============================================================================
// Interstage: Airwindows Analog Transformer & Interstage Coupling Emulator
// Recreates analog interstage transformer behavior: slew-limiting high frequencies
// relative to low-frequency energy flux, capacitor coupling, and analog glue.
// Zero parameters, 0 latency, pure analog conditioning.
// ============================================================================
class Interstage : public IProcessor {
public:
    Interstage() noexcept {
        reset();
    }

    void init(uint32_t sample_rate) noexcept override {
        m_sample_rate = sample_rate > 0 ? sample_rate : 48000;
        reset();
    }

    void reset() noexcept override {
        m_iir_a_l = 0.0;
        m_iir_b_l = 0.0;
        m_iir_c_l = 0.0;
        m_iir_d_l = 0.0;
        m_iir_e_l = 0.0;
        m_iir_f_l = 0.0;
        m_last_sample_l = 0.0;

        m_iir_a_r = 0.0;
        m_iir_b_r = 0.0;
        m_iir_c_r = 0.0;
        m_iir_d_r = 0.0;
        m_iir_e_r = 0.0;
        m_iir_f_r = 0.0;
        m_last_sample_r = 0.0;

        m_flip = false;
    }

    void process_stereo(Sample* left, Sample* right, uint32_t frames) noexcept override {
        if (!left || !right || frames == 0) return;

        const double overallscale = static_cast<double>(m_sample_rate) / 44100.0;
        // 0.381966011250105 = (3 - sqrt(5)) / 2 = 1 - 1/phi (Golden Ratio conjugate squared)
        constexpr double kGoldenConjugateSq = 0.381966011250105;
        const double first_stage = kGoldenConjugateSq / overallscale;
        const double iir_amount = 0.00295 / overallscale;
        constexpr double threshold = kGoldenConjugateSq;

        for (uint32_t i = 0; i < frames; ++i) {
            double input_l = static_cast<double>(left[i]);
            double input_r = static_cast<double>(right[i]);

            const double dry_l = input_l;
            const double dry_r = input_r;

            // Start lowpassing with 2-point moving average
            input_l = (input_l + m_last_sample_l) * 0.5;
            input_r = (input_r + m_last_sample_r) * 0.5;

            if (m_flip) {
                // Left channel Stage 1 (Flip == true)
                m_iir_a_l = (m_iir_a_l * (1.0 - first_stage)) + (input_l * first_stage);
                input_l = m_iir_a_l;
                m_iir_c_l = (m_iir_c_l * (1.0 - iir_amount)) + (input_l * iir_amount);
                input_l = m_iir_c_l;
                m_iir_e_l = (m_iir_e_l * (1.0 - iir_amount)) + (input_l * iir_amount);
                input_l = m_iir_e_l;

                // Highpass derivation
                input_l = dry_l - input_l;

                // Slew rate limit against lowpassed magnetic reference point
                if (input_l - m_iir_a_l > threshold) input_l = m_iir_a_l + threshold;
                if (input_l - m_iir_a_l < -threshold) input_l = m_iir_a_l - threshold;

                // Right channel Stage 1 (Flip == true)
                m_iir_a_r = (m_iir_a_r * (1.0 - first_stage)) + (input_r * first_stage);
                input_r = m_iir_a_r;
                m_iir_c_r = (m_iir_c_r * (1.0 - iir_amount)) + (input_r * iir_amount);
                input_r = m_iir_c_r;
                m_iir_e_r = (m_iir_e_r * (1.0 - iir_amount)) + (input_r * iir_amount);
                input_r = m_iir_e_r;

                input_r = dry_r - input_r;

                if (input_r - m_iir_a_r > threshold) input_r = m_iir_a_r + threshold;
                if (input_r - m_iir_a_r < -threshold) input_r = m_iir_a_r - threshold;
            } else {
                // Left channel Stage 2 (Flip == false)
                m_iir_b_l = (m_iir_b_l * (1.0 - first_stage)) + (input_l * first_stage);
                input_l = m_iir_b_l;
                m_iir_d_l = (m_iir_d_l * (1.0 - iir_amount)) + (input_l * iir_amount);
                input_l = m_iir_d_l;
                m_iir_f_l = (m_iir_f_l * (1.0 - iir_amount)) + (input_l * iir_amount);
                input_l = m_iir_f_l;

                input_l = dry_l - input_l;

                if (input_l - m_iir_b_l > threshold) input_l = m_iir_b_l + threshold;
                if (input_l - m_iir_b_l < -threshold) input_l = m_iir_b_l - threshold;

                // Right channel Stage 2 (Flip == false)
                m_iir_b_r = (m_iir_b_r * (1.0 - first_stage)) + (input_r * first_stage);
                input_r = m_iir_b_r;
                m_iir_d_r = (m_iir_d_r * (1.0 - iir_amount)) + (input_r * iir_amount);
                input_r = m_iir_d_r;
                m_iir_f_r = (m_iir_f_r * (1.0 - iir_amount)) + (input_r * iir_amount);
                input_r = m_iir_f_r;

                input_r = dry_r - input_r;

                if (input_r - m_iir_b_r > threshold) input_r = m_iir_b_r + threshold;
                if (input_r - m_iir_b_r < -threshold) input_r = m_iir_b_r - threshold;
            }

            m_flip = !m_flip;
            m_last_sample_l = input_l;
            m_last_sample_r = input_r;

            left[i] = static_cast<Sample>(input_l);
            right[i] = static_cast<Sample>(input_r);
        }
    }

    void set_parameter([[maybe_unused]] uint32_t index, [[maybe_unused]] float value) noexcept override {}
    [[nodiscard]] float get_parameter([[maybe_unused]] uint32_t index) const noexcept override { return 0.0f; }
    [[nodiscard]] uint32_t parameter_count() const noexcept override { return 0; }
    [[nodiscard]] const char* name() const noexcept override { return "Interstage"; }

private:
    uint32_t m_sample_rate{48000};

    double m_iir_a_l{0.0};
    double m_iir_b_l{0.0};
    double m_iir_c_l{0.0};
    double m_iir_d_l{0.0};
    double m_iir_e_l{0.0};
    double m_iir_f_l{0.0};
    double m_last_sample_l{0.0};

    double m_iir_a_r{0.0};
    double m_iir_b_r{0.0};
    double m_iir_c_r{0.0};
    double m_iir_d_r{0.0};
    double m_iir_e_r{0.0};
    double m_iir_f_r{0.0};
    double m_last_sample_r{0.0};

    bool m_flip{false};
};

} // namespace audio_core::dsp
