#pragma once

#include "audio_core/types.hpp"
#include <cmath>
#include <numbers>
#include <algorithm>

namespace audio_core::dsp {

enum class Waveform : uint8_t {
    Sine = 0,
    Saw,
    Square,
    Triangle
};

class Oscillator {
public:
    Oscillator() = default;

    void init(uint32_t sample_rate) noexcept {
        m_sample_rate = sample_rate;
        update_phase_increment();
    }

    void set_waveform(Waveform wf) noexcept {
        m_waveform = wf;
    }

    void set_frequency(float freq_hz) noexcept {
        m_frequency = freq_hz > 0.1f ? freq_hz : 0.1f;
        update_phase_increment();
    }

    void set_pulse_width(float pw) noexcept {
        m_pulse_width = std::clamp(pw, 0.05f, 0.95f);
    }

    void reset_phase() noexcept {
        m_phase = 0.0f;
    }

    [[nodiscard]] Sample process_sample() noexcept {
        Sample out = 0.0f;

        switch (m_waveform) {
            case Waveform::Sine:
                out = std::sin(m_phase * 2.0f * std::numbers::pi_v<float>);
                break;

            case Waveform::Saw: {
                // Naive saw
                float t = m_phase;
                out = (2.0f * t) - 1.0f;
                // PolyBLEP anti-aliasing correction
                out -= poly_blep(t);
                break;
            }

            case Waveform::Square: {
                float t = m_phase;
                float naive = (t < m_pulse_width) ? 1.0f : -1.0f;
                // PolyBLEP at rising edge (t = 0) and falling edge (t = m_pulse_width)
                naive += poly_blep(t);
                naive -= poly_blep(std::fmod(t + (1.0f - m_pulse_width), 1.0f));
                out = naive;
                break;
            }

            case Waveform::Triangle: {
                // Integrate square wave or direct formula
                float t = m_phase;
                out = 2.0f * std::abs(2.0f * (t - std::floor(t + 0.5f))) - 1.0f;
                break;
            }
        }

        // Advance phase
        m_phase += m_phase_inc;
        if (m_phase >= 1.0f) {
            m_phase -= 1.0f;
        }

        return out;
    }

    void process_block(Sample* output, uint32_t num_frames) noexcept {
        for (uint32_t i = 0; i < num_frames; ++i) {
            output[i] = process_sample();
        }
    }

private:
    void update_phase_increment() noexcept {
        if (m_sample_rate > 0) {
            m_phase_inc = m_frequency / static_cast<float>(m_sample_rate);
        }
    }

    // PolyBLEP: 2nd-order polynomial residual for band-limited step
    [[nodiscard]] float poly_blep(float t) const noexcept {
        float dt = m_phase_inc;
        // 0 <= t < dt
        if (t < dt) {
            t /= dt;
            return (t + t) - (t * t) - 1.0f;
        }
        // 1 - dt <= t < 1
        if (t > 1.0f - dt) {
            t = (t - 1.0f) / dt;
            return (t * t) + (t + t) + 1.0f;
        }
        return 0.0f;
    }

    uint32_t m_sample_rate{kDefaultSampleRate};
    Waveform m_waveform{Waveform::Sine};
    float m_frequency{440.0f};
    float m_phase{0.0f};
    float m_phase_inc{0.0f};
    float m_pulse_width{0.5f};
};

} // namespace audio_core::dsp
