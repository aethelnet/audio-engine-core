#pragma once

#include "audio_core/insert_slot.hpp"
#include <cmath>
#include <algorithm>

namespace audio_core::dsp {

// ============================================================================
// ButterComp2: Quad-Interleaved Bipolar-RMS Compressor (Airwindows)
// Separates signal into positive & negative half-waves and interleaves them
// with alternating smoothing filters (zipper mechanism). Eliminates low-end
// pumping completely while delivering invisible, buttery RMS leveling.
// ============================================================================
class ButterComp2 : public IProcessor {
public:
    ButterComp2() = default;

    void init(uint32_t sample_rate) noexcept override {
        m_sample_rate = sample_rate > 0 ? sample_rate : 48000;
        reset();
    }

    void reset() noexcept override {
        m_control_a_pos_l = 0.0f; m_control_a_neg_l = 0.0f;
        m_control_b_pos_l = 0.0f; m_control_b_neg_l = 0.0f;
        m_control_a_pos_r = 0.0f; m_control_a_neg_r = 0.0f;
        m_control_b_pos_r = 0.0f; m_control_b_neg_r = 0.0f;
        m_target_l = 1.0f;
        m_target_r = 1.0f;
        m_flip = false;
    }

    [[nodiscard]] const char* name() const noexcept override {
        return "Airwindows ButterComp2";
    }

    void set_parameter(uint32_t index, float value) noexcept override {
        switch (index) {
            case 0: m_compress = std::clamp(value, 0.0f, 1.0f); break; // Compress Amount
            case 1: m_output_gain = std::clamp(value, 0.0f, 2.0f); break; // Output Makeup
            case 2: m_wet = std::clamp(value, 0.0f, 1.0f); break; // Dry/Wet
            default: break;
        }
    }

    [[nodiscard]] float get_parameter(uint32_t index) const noexcept override {
        switch (index) {
            case 0: return m_compress;
            case 1: return m_output_gain;
            case 2: return m_wet;
            default: return 0.0f;
        }
    }

    [[nodiscard]] bool supports_sidechain() const noexcept override { return true; }

    void process_stereo(Sample* left, Sample* right, uint32_t frames) noexcept override {
        process_stereo_sidechain(left, right, nullptr, nullptr, frames);
    }

    void process_stereo_sidechain(Sample* left, Sample* right,
                                  const Sample* sc_left, const Sample* sc_right,
                                  uint32_t frames) noexcept override {
        if (m_compress <= 0.001f && m_output_gain == 1.0f && m_wet >= 0.999f) {
            return;
        }

        const float input_gain = 1.0f + (m_compress * 3.0f);
        const float rate = 0.0005f * (48000.0f / static_cast<float>(m_sample_rate));
        const float out_gain = m_output_gain;
        const float wet = m_wet;
        const float dry = 1.0f - wet;

        for (uint32_t i = 0; i < frames; ++i) {
            float in_l = left[i] * input_gain;
            float in_r = right[i] * input_gain;

            // Use external sidechain detector if provided, else self-audio
            float det_l = (sc_left != nullptr) ? (sc_left[i] * input_gain) : in_l;
            float det_r = (sc_right != nullptr) ? (sc_right[i] * input_gain) : in_r;

            // --- Left Channel Bipolar Tracking ---
            float pos_l = std::max(0.0f, det_l);
            float neg_l = std::max(0.0f, -det_l);

            if (m_flip) {
                m_control_a_pos_l = (m_control_a_pos_l * (1.0f - rate)) + (pos_l * rate);
                m_control_a_neg_l = (m_control_a_neg_l * (1.0f - rate)) + (neg_l * rate);
                m_target_l = 1.0f / (1.0f + m_control_a_pos_l + m_control_a_neg_l);
            } else {
                m_control_b_pos_l = (m_control_b_pos_l * (1.0f - rate)) + (pos_l * rate);
                m_control_b_neg_l = (m_control_b_neg_l * (1.0f - rate)) + (neg_l * rate);
                m_target_l = 1.0f / (1.0f + m_control_b_pos_l + m_control_b_neg_l);
            }

            // --- Right Channel Bipolar Tracking ---
            float pos_r = std::max(0.0f, det_r);
            float neg_r = std::max(0.0f, -det_r);

            if (m_flip) {
                m_control_a_pos_r = (m_control_a_pos_r * (1.0f - rate)) + (pos_r * rate);
                m_control_a_neg_r = (m_control_a_neg_r * (1.0f - rate)) + (neg_r * rate);
                m_target_r = 1.0f / (1.0f + m_control_a_pos_r + m_control_a_neg_r);
            } else {
                m_control_b_pos_r = (m_control_b_pos_r * (1.0f - rate)) + (pos_r * rate);
                m_control_b_neg_r = (m_control_b_neg_r * (1.0f - rate)) + (neg_r * rate);
                m_target_r = 1.0f / (1.0f + m_control_b_pos_r + m_control_b_neg_r);
            }

            m_flip = !m_flip;

            float comp_l = in_l * m_target_l * out_gain;
            float comp_r = in_r * m_target_r * out_gain;

            left[i] = (dry * left[i]) + (wet * comp_l);
            right[i] = (dry * right[i]) + (wet * comp_r);
        }
    }

private:
    uint32_t m_sample_rate{48000};
    float m_compress{0.3f};
    float m_output_gain{1.0f};
    float m_wet{1.0f};

    float m_control_a_pos_l{0.0f};
    float m_control_a_neg_l{0.0f};
    float m_control_b_pos_l{0.0f};
    float m_control_b_neg_l{0.0f};
    float m_target_l{1.0f};

    float m_control_a_pos_r{0.0f};
    float m_control_a_neg_r{0.0f};
    float m_control_b_pos_r{0.0f};
    float m_control_b_neg_r{0.0f};
    float m_target_r{1.0f};

    bool m_flip{false};
};

} // namespace audio_core::dsp
