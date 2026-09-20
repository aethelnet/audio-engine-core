#pragma once

#include "audio_core/types.hpp"
#include <cmath>
#include <algorithm>

namespace audio_core::dsp {

enum class EnvelopeStage : uint8_t {
    Idle = 0,
    Attack,
    Decay,
    Sustain,
    Release
};

class Envelope {
public:
    Envelope() = default;

    void init(uint32_t sample_rate) noexcept {
        m_sample_rate = sample_rate;
        update_rates();
    }

    void set_attack_ms(float ms) noexcept {
        m_attack_ms = std::max(ms, 0.5f);
        update_rates();
    }

    void set_decay_ms(float ms) noexcept {
        m_decay_ms = std::max(ms, 1.0f);
        update_rates();
    }

    void set_sustain_level(float level) noexcept {
        m_sustain_level = std::clamp(level, 0.0f, 1.0f);
    }

    void set_release_ms(float ms) noexcept {
        m_release_ms = std::max(ms, 1.0f);
        update_rates();
    }

    void gate(bool on) noexcept {
        if (on) {
            m_stage = EnvelopeStage::Attack;
        } else {
            if (m_stage != EnvelopeStage::Idle) {
                m_stage = EnvelopeStage::Release;
            }
        }
    }

    void reset() noexcept {
        m_stage = EnvelopeStage::Idle;
        m_current_val = 0.0f;
    }

    [[nodiscard]] bool is_active() const noexcept {
        return m_stage != EnvelopeStage::Idle;
    }

    [[nodiscard]] EnvelopeStage stage() const noexcept {
        return m_stage;
    }

    [[nodiscard]] Sample process_sample() noexcept {
        switch (m_stage) {
            case EnvelopeStage::Idle:
                m_current_val = 0.0f;
                break;

            case EnvelopeStage::Attack:
                m_current_val += m_attack_rate;
                if (m_current_val >= 1.0f) {
                    m_current_val = 1.0f;
                    m_stage = EnvelopeStage::Decay;
                }
                break;

            case EnvelopeStage::Decay:
                m_current_val -= m_decay_rate;
                if (m_current_val <= m_sustain_level) {
                    m_current_val = m_sustain_level;
                    m_stage = EnvelopeStage::Sustain;
                }
                break;

            case EnvelopeStage::Sustain:
                m_current_val = m_sustain_level;
                break;

            case EnvelopeStage::Release:
                m_current_val -= m_release_rate;
                if (m_current_val <= 0.0001f) {
                    m_current_val = 0.0f;
                    m_stage = EnvelopeStage::Idle;
                }
                break;
        }

        return m_current_val;
    }

    void process_block(Sample* output, uint32_t num_frames) noexcept {
        for (uint32_t i = 0; i < num_frames; ++i) {
            output[i] = process_sample();
        }
    }

private:
    void update_rates() noexcept {
        if (m_sample_rate == 0) return;

        const float sr = static_cast<float>(m_sample_rate);
        m_attack_rate = 1.0f / ((m_attack_ms * 0.001f) * sr);
        m_decay_rate = (1.0f - m_sustain_level) / ((m_decay_ms * 0.001f) * sr);
        m_release_rate = 1.0f / ((m_release_ms * 0.001f) * sr);
    }

    uint32_t m_sample_rate{kDefaultSampleRate};
    EnvelopeStage m_stage{EnvelopeStage::Idle};
    float m_current_val{0.0f};

    float m_attack_ms{10.0f};
    float m_decay_ms{100.0f};
    float m_sustain_level{0.7f};
    float m_release_ms{200.0f};

    float m_attack_rate{0.0f};
    float m_decay_rate{0.0f};
    float m_release_rate{0.0f};
};

} // namespace audio_core::dsp
