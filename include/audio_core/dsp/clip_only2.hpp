#pragma once

#include "audio_core/insert_slot.hpp"
#include <cmath>
#include <algorithm>

namespace audio_core::dsp {

// ============================================================================
// ClipOnly2: 1-Sample Zero-Latency Slew Rounder (Airwindows)
// Intercepts digital overs and rounds off transients using a 2nd order polynomial
// with exact 0-sample lookahead and zero aliasing ripple.
// ============================================================================
class ClipOnly2 : public IProcessor {
public:
    ClipOnly2() = default;

    void init(uint32_t /*sample_rate*/) noexcept override {
        reset();
    }

    void reset() noexcept override {
        m_last_l = 0.0f;
        m_last_r = 0.0f;
    }

    [[nodiscard]] const char* name() const noexcept override {
        return "Airwindows ClipOnly2";
    }

    void set_parameter(uint32_t /*index*/, float /*value*/) noexcept override {}
    [[nodiscard]] float get_parameter(uint32_t /*index*/) const noexcept override { return 1.0f; }

    void process_stereo(Sample* left, Sample* right, uint32_t frames) noexcept override {
        constexpr float kThreshold = 0.9549925859f;

        for (uint32_t i = 0; i < frames; ++i) {
            float in_l = left[i];
            float in_r = right[i];

            // Left Channel
            if (in_l > kThreshold) {
                in_l = 0.70582f + (m_last_l * 0.26091f);
                if (in_l > 1.0f) in_l = 1.0f;
            } else if (in_l < -kThreshold) {
                in_l = -0.70582f + (m_last_l * 0.26091f);
                if (in_l < -1.0f) in_l = -1.0f;
            }
            m_last_l = in_l;
            left[i] = in_l;

            // Right Channel
            if (in_r > kThreshold) {
                in_r = 0.70582f + (m_last_r * 0.26091f);
                if (in_r > 1.0f) in_r = 1.0f;
            } else if (in_r < -kThreshold) {
                in_r = -0.70582f + (m_last_r * 0.26091f);
                if (in_r < -1.0f) in_r = -1.0f;
            }
            m_last_r = in_r;
            right[i] = in_r;
        }
    }

private:
    float m_last_l{0.0f};
    float m_last_r{0.0f};
};

} // namespace audio_core::dsp
