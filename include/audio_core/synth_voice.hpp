#pragma once

#include "audio_core/types.hpp"
#include "audio_core/dsp/oscillator.hpp"
#include "audio_core/dsp/biquad_filter.hpp"
#include "audio_core/dsp/envelope.hpp"
#include <cmath>

namespace audio_core {

class SynthVoice {
public:
    SynthVoice() = default;

    void init(uint32_t sample_rate) noexcept {
        m_sample_rate = sample_rate;
        m_osc1.init(sample_rate);
        m_osc2.init(sample_rate);
        m_filter.init(sample_rate);
        m_amp_env.init(sample_rate);
        m_filter_env.init(sample_rate);

        m_osc1.set_waveform(dsp::Waveform::Saw);
        m_osc2.set_waveform(dsp::Waveform::Square);
        m_filter.set_type(dsp::FilterType::Lowpass);
        m_filter.set_cutoff(2500.0f);
        m_filter.set_q(1.5f);
    }

    void note_on(uint8_t note, uint8_t velocity) noexcept {
        m_current_note = note;
        m_velocity = static_cast<float>(velocity) / 127.0f;

        const float base_freq = midi_to_freq(note);
        m_base_freq = base_freq;

        m_osc1.set_frequency(base_freq);
        m_osc1.reset_phase();

        // Osc2 detuned slightly or 1 octave down
        m_osc2.set_frequency(base_freq * m_osc2_detune_ratio);
        m_osc2.reset_phase();

        m_filter.reset();
        m_amp_env.gate(true);
        m_filter_env.gate(true);
    }

    void note_off() noexcept {
        m_amp_env.gate(false);
        m_filter_env.gate(false);
    }

    [[nodiscard]] bool is_active() const noexcept {
        return m_amp_env.is_active();
    }

    [[nodiscard]] uint8_t current_note() const noexcept {
        return m_current_note;
    }

    void set_filter_cutoff(float cutoff_hz) noexcept {
        m_base_cutoff = cutoff_hz;
    }

    void set_filter_q(float q) noexcept {
        m_filter.set_q(q);
    }

    void set_filter_env_amount(float amount) noexcept {
        m_filter_env_amount = amount;
    }

    void set_osc_mix(float mix) noexcept {
        m_osc_mix = std::clamp(mix, 0.0f, 1.0f);
    }

    void set_osc2_detune_cents(float cents) noexcept {
        m_osc2_detune_ratio = std::pow(2.0f, cents / 1200.0f);
        m_osc2.set_frequency(m_base_freq * m_osc2_detune_ratio);
    }

    dsp::Oscillator& osc1() noexcept { return m_osc1; }
    dsp::Oscillator& osc2() noexcept { return m_osc2; }
    dsp::Envelope& amp_env() noexcept { return m_amp_env; }
    dsp::Envelope& filter_env() noexcept { return m_filter_env; }

    [[nodiscard]] Sample process_sample() noexcept {
        if (!is_active()) {
            return 0.0f;
        }

        // Generate oscillator signals
        const Sample s1 = m_osc1.process_sample();
        const Sample s2 = m_osc2.process_sample();
        const Sample mixed = (s1 * (1.0f - m_osc_mix)) + (s2 * m_osc_mix);

        // Filter envelope modulation
        const float f_env = m_filter_env.process_sample();
        const float modulated_cutoff = m_base_cutoff + (m_filter_env_amount * f_env);
        m_filter.set_cutoff(modulated_cutoff);

        // Apply filter
        const Sample filtered = m_filter.process_sample(mixed);

        // Amp envelope & velocity
        const float a_env = m_amp_env.process_sample();
        return filtered * a_env * m_velocity;
    }

    void process_block_add(Sample* left, Sample* right, uint32_t num_frames) noexcept {
        if (!is_active()) return;

        for (uint32_t i = 0; i < num_frames; ++i) {
            Sample s = process_sample();
            left[i] += s;
            right[i] += s;
        }
    }

    static float midi_to_freq(uint8_t note) noexcept {
        return 440.0f * std::pow(2.0f, (static_cast<float>(note) - 69.0f) / 12.0f);
    }

private:
    uint32_t m_sample_rate{kDefaultSampleRate};
    uint8_t m_current_note{0};
    float m_velocity{0.0f};
    float m_base_freq{440.0f};
    float m_osc2_detune_ratio{0.995f}; // slight detune default
    float m_osc_mix{0.5f};
    float m_base_cutoff{2000.0f};
    float m_filter_env_amount{3000.0f};

    dsp::Oscillator m_osc1;
    dsp::Oscillator m_osc2;
    dsp::BiquadFilter m_filter;
    dsp::Envelope m_amp_env;
    dsp::Envelope m_filter_env;
};

} // namespace audio_core
