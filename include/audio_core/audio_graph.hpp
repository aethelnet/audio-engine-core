#pragma once

#include "audio_core/types.hpp"
#include "audio_core/synth_voice.hpp"
#include <array>
#include <cmath>

namespace audio_core {

constexpr size_t kMaxVoices = 16;

class AudioGraph {
public:
    AudioGraph() = default;

    void init(uint32_t sample_rate) noexcept {
        m_sample_rate = sample_rate;
        for (auto& voice : m_voices) {
            voice.init(sample_rate);
        }
    }

    void handle_midi_event(const MidiEvent& event) noexcept {
        switch (event.type()) {
            case MidiStatus::NoteOn: {
                if (event.velocity() > 0) {
                    allocate_and_trigger_voice(event.note(), event.velocity());
                } else {
                    release_voice(event.note());
                }
                break;
            }

            case MidiStatus::NoteOff:
                release_voice(event.note());
                break;

            case MidiStatus::ControlChange:
                // Handle CC (e.g., CC 74 = Filter Cutoff, CC 7 = Volume, CC 1 = Mod Wheel)
                handle_control_change(event.data1, event.data2);
                break;

            default:
                break;
        }
    }

    void set_master_volume(float volume) noexcept {
        m_master_volume = std::clamp(volume, 0.0f, 2.0f);
    }

    void set_global_filter_cutoff(float cutoff_hz) noexcept {
        m_global_cutoff = cutoff_hz;
        for (auto& voice : m_voices) {
            voice.set_filter_cutoff(cutoff_hz);
        }
    }

    void set_global_resonance(float q) noexcept {
        for (auto& voice : m_voices) {
            voice.set_filter_q(q);
        }
    }

    void all_notes_off() noexcept {
        for (auto& voice : m_voices) {
            voice.note_off();
        }
    }

    // Real-Time Audio Render Callback: Zero allocations, zero mutexes!
    void render(AudioBufferView& out_buffer) noexcept {
        out_buffer.clear();

        Sample* left = out_buffer.channel(0);
        Sample* right = (out_buffer.num_channels() > 1) ? out_buffer.channel(1) : left;
        const uint32_t frames = out_buffer.num_frames();

        // Accumulate active voices into the buffer
        for (auto& voice : m_voices) {
            voice.process_block_add(left, right, frames);
        }

        // Apply Master Volume and Soft-Clipping Limiter (tanh saturation)
        const float vol = m_master_volume;
        for (uint32_t i = 0; i < frames; ++i) {
            left[i] = soft_clip(left[i] * vol);
            if (out_buffer.num_channels() > 1) {
                right[i] = soft_clip(right[i] * vol);
            }
        }
    }

    [[nodiscard]] size_t active_voice_count() const noexcept {
        size_t count = 0;
        for (const auto& voice : m_voices) {
            if (voice.is_active()) ++count;
        }
        return count;
    }

private:
    void allocate_and_trigger_voice(uint8_t note, uint8_t velocity) noexcept {
        // 1. Look for an idle voice
        for (size_t i = 0; i < kMaxVoices; ++i) {
            if (!m_voices[i].is_active()) {
                m_voices[i].set_filter_cutoff(m_global_cutoff);
                m_voices[i].note_on(note, velocity);
                m_voice_ages[i] = m_age_counter++;
                return;
            }
        }

        // 2. Voice stealing: Steal the oldest voice
        size_t oldest_idx = 0;
        uint64_t oldest_age = m_voice_ages[0];
        for (size_t i = 1; i < kMaxVoices; ++i) {
            if (m_voice_ages[i] < oldest_age) {
                oldest_age = m_voice_ages[i];
                oldest_idx = i;
            }
        }

        m_voices[oldest_idx].set_filter_cutoff(m_global_cutoff);
        m_voices[oldest_idx].note_on(note, velocity);
        m_voice_ages[oldest_idx] = m_age_counter++;
    }

    void release_voice(uint8_t note) noexcept {
        for (auto& voice : m_voices) {
            if (voice.is_active() && voice.current_note() == note) {
                voice.note_off();
            }
        }
    }

    void handle_control_change(uint8_t cc, uint8_t value) noexcept {
        const float normalized = static_cast<float>(value) / 127.0f;
        if (cc == 74) {
            // Filter Cutoff (20 Hz to 18000 Hz exponential mapping)
            float cutoff = 20.0f * std::pow(900.0f, normalized);
            set_global_filter_cutoff(cutoff);
        } else if (cc == 71) {
            // Filter Resonance (0.5 to 15.0)
            float q = 0.5f + (normalized * 14.5f);
            set_global_resonance(q);
        } else if (cc == 7) {
            // Master Volume
            set_master_volume(normalized);
        }
    }

    // Fast soft-clipper to prevent harsh digital wrap-around distortion
    [[nodiscard]] static Sample soft_clip(Sample x) noexcept {
        if (x > 1.25f) return 1.0f;
        if (x < -1.25f) return -1.0f;
        // Cubic soft saturation: f(x) = x - (x^3)/3
        return std::tanh(x);
    }

    uint32_t m_sample_rate{kDefaultSampleRate};
    float m_master_volume{0.8f};
    float m_global_cutoff{2500.0f};

    std::array<SynthVoice, kMaxVoices> m_voices;
    std::array<uint64_t, kMaxVoices> m_voice_ages{0};
    uint64_t m_age_counter{0};
};

} // namespace audio_core
