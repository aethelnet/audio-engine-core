#pragma once

#include "audio_core/insert_slot.hpp"
#include <cmath>
#include <algorithm>

namespace audio_core::dsp {

// ============================================================================
// PurestDrive: Phase-Coherent Harmonic Saturation (Airwindows)
// Uses dynamic frequency cancellation via sample differentiation to keep highs
// completely transparent while richly saturating low-mid fundamentals.
// ============================================================================
class PurestDrive : public IProcessor {
public:
    PurestDrive() = default;

    void init(uint32_t /*sample_rate*/) noexcept override {
        reset();
    }

    void reset() noexcept override {
        m_prev_l = 0.0f;
        m_prev_r = 0.0f;
    }

    [[nodiscard]] const char* name() const noexcept override {
        return "Airwindows PurestDrive";
    }

    void set_parameter(uint32_t index, float value) noexcept override {
        switch (index) {
            case 0: m_drive = std::clamp(value, 0.0f, 1.0f); break;
            case 1: m_wet = std::clamp(value, 0.0f, 1.0f); break;
            default: break;
        }
    }

    [[nodiscard]] float get_parameter(uint32_t index) const noexcept override {
        switch (index) {
            case 0: return m_drive;
            case 1: return m_wet;
            default: return 0.0f;
        }
    }

    void process_stereo(Sample* left, Sample* right, uint32_t frames) noexcept override {
        if (m_drive == 0.0f && m_wet == 0.0f) {
            return;
        }

        const float intensity = m_drive;
        const float wet = m_wet;
        const float dry = 1.0f - wet;

        for (uint32_t i = 0; i < frames; ++i) {
            float dry_l = left[i];
            float dry_r = right[i];

            // 1. Nonlinear transfer
            float input_l = std::sin(dry_l);
            float input_r = std::sin(dry_r);

            // 2. Cancellation of saturation when sign flips rapidly (high frequencies)
            float apply_l = (std::abs(m_prev_l + input_l) * 0.5f) * intensity;
            float apply_r = (std::abs(m_prev_r + input_r) * 0.5f) * intensity;

            float sat_l = (dry_l * (1.0f - apply_l)) + (input_l * apply_l);
            float sat_r = (dry_r * (1.0f - apply_r)) + (input_r * apply_r);

            m_prev_l = input_l;
            m_prev_r = input_r;

            // 3. Dry/Wet blend
            left[i] = (dry * dry_l) + (wet * sat_l);
            right[i] = (dry * dry_r) + (wet * sat_r);
        }
    }

private:
    float m_drive{0.5f};
    float m_wet{1.0f};
    float m_prev_l{0.0f};
    float m_prev_r{0.0f};
};

} // namespace audio_core::dsp
