#pragma once

#include "audio_core/types.hpp"
#include "audio_core/dsp/oscillator.hpp"
#include "audio_core/dsp/biquad_filter.hpp"
#include "audio_core/modulation/multi_stage_envelope.hpp"
#include <array>
#include <vector>
#include <cmath>
#include <numbers>
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <limits>

namespace audio_core::modulation {

// ============================================================================
// Polyphony Playback Modes
// ============================================================================
enum class PolyphonyPlayMode : uint8_t {
    Polyphonic = 0, // Independent polyphonic voices (up to 16 notes)
    MonoLegato = 1, // Single voice with smooth portamento glide, non-retriggering legato
    Unison4x   = 2  // 4 voices stacked and detuned on every note for thick leads/basses
};

// ============================================================================
// PolyVoice: Zero-Allocation POD Runtime Voice State
// ============================================================================
struct PolyVoice {
    uint32_t id{0};
    uint8_t note{0};
    float velocity{0.0f};
    float base_freq{440.0f};
    float target_freq{440.0f};
    float current_freq{440.0f};
    uint64_t trigger_timestamp{0};
    bool gate_held{false};
    bool active{false};
    float pan{0.0f}; // [-1.0 .. +1.0]

    // Dual MSEG voice states (Amp + Mod)
    MsegVoice amp_env;
    MsegVoice mod_env;

    // Dual polyBLEP anti-aliased oscillators
    dsp::Oscillator osc1;
    dsp::Oscillator osc2;

    // Per-voice state-variable / biquad filter
    dsp::BiquadFilter filter;

    void init(uint32_t sample_rate, uint32_t voice_id) noexcept {
        id = voice_id;
        osc1.init(sample_rate);
        osc2.init(sample_rate);
        filter.init(sample_rate);
        reset();
    }

    void reset() noexcept {
        note = 0;
        velocity = 0.0f;
        base_freq = 440.0f;
        target_freq = 440.0f;
        current_freq = 440.0f;
        trigger_timestamp = 0;
        gate_held = false;
        active = false;
        pan = 0.0f;
        amp_env.reset();
        mod_env.reset();
        osc1.reset_phase();
        osc2.reset_phase();
        filter.reset();
    }

    void trigger(uint8_t midi_note, float vel, uint64_t timestamp, float voice_pan = 0.0f) noexcept {
        note = midi_note;
        velocity = std::clamp(vel, 0.0f, 1.0f);
        base_freq = 440.0f * std::pow(2.0f, (static_cast<float>(midi_note) - 69.0f) / 12.0f);
        target_freq = base_freq;
        current_freq = base_freq;
        trigger_timestamp = timestamp;
        gate_held = true;
        active = true;
        pan = voice_pan;

        amp_env.trigger(velocity);
        mod_env.trigger(velocity);
        osc1.reset_phase();
        osc2.reset_phase();
        filter.reset();
    }

    void release() noexcept {
        gate_held = false;
        amp_env.release();
        mod_env.release();
    }
};

// ============================================================================
// PolyVoiceTelemetry: Lock-Free UI Visualization Frame
// ============================================================================
struct PolyVoiceTelemetry {
    uint32_t id{0};
    bool active{false};
    uint8_t note{0};
    float velocity{0.0f};
    float amp_level{0.0f};
    float mod_level{0.0f};
    MsegStage stage{MsegStage::Idle};
    float pan{0.0f};
    double playhead_time{0.0};
};

// ============================================================================
// PolyphonicSynth: Mastering-Grade Polyphonic Voice Pool & Allocator
// Supports up to 16 voices, click-free LRU/Release voice stealing,
// dual MSEG voice modulation, unison stacking, and stereo panning spread.
// Zero heap allocations in audio rendering; real-time safe.
// ============================================================================
class PolyphonicSynth {
public:
    static constexpr size_t kMaxVoices = 16;

    PolyphonicSynth() {
        m_voices.fill(PolyVoice{});
        // Default internal MSEG presets
        m_internal_amp_mseg.preset_plucked_synth();
        m_internal_mod_mseg.preset_wobble_lfo();
    }

    void init(uint32_t sample_rate) noexcept {
        m_sample_rate = std::max(1000u, sample_rate);
        for (size_t i = 0; i < kMaxVoices; ++i) {
            m_voices[i].init(m_sample_rate, static_cast<uint32_t>(i));
        }
        reset();
    }

    void reset() noexcept {
        m_timestamp_counter = 0;
        m_sustain_pedal_held = false;
        m_pitch_bend_norm = 0.0f;
        m_pitch_bend_semitones = 0.0f;
        for (auto& v : m_voices) {
            v.reset();
        }
    }

    // ========================================================================
    // Envelope Binding: Share MSEGs from ModulationMatrix or Engine
    // ========================================================================
    void bind_envelopes(const MultiStageEnvelope* amp_mseg, const MultiStageEnvelope* mod_mseg) noexcept {
        m_bound_amp_mseg = amp_mseg;
        m_bound_mod_mseg = mod_mseg;
    }

    [[nodiscard]] MultiStageEnvelope& internal_amp_mseg() noexcept { return m_internal_amp_mseg; }
    [[nodiscard]] MultiStageEnvelope& internal_mod_mseg() noexcept { return m_internal_mod_mseg; }

    // ========================================================================
    // Voice Allocation & Stealing (Click-Free)
    // ========================================================================
    int32_t note_on(uint8_t midi_note, float velocity = 1.0f) noexcept {
        const uint64_t now_ts = ++m_timestamp_counter;

        // 1. Mode: Mono Legato
        if (m_play_mode == PolyphonyPlayMode::MonoLegato) {
            auto& v = m_voices[0];
            if (v.active && v.gate_held) {
                v.note = midi_note;
                v.velocity = velocity;
                v.target_freq = 440.0f * std::pow(2.0f, (static_cast<float>(midi_note) - 69.0f) / 12.0f);
                if (m_glide_time_ms <= 1.0f) {
                    v.current_freq = v.target_freq;
                }
                v.trigger_timestamp = now_ts;
                return 0;
            }
            v.trigger(midi_note, velocity, now_ts, 0.0f);
            return 0;
        }

        // 2. Mode: Unison 4x Stack
        if (m_play_mode == PolyphonyPlayMode::Unison4x) {
            constexpr float unison_pans[4] = { -0.75f, -0.25f, +0.25f, +0.75f };
            for (size_t u = 0; u < 4; ++u) {
                m_voices[u].trigger(midi_note, velocity, now_ts, unison_pans[u] * m_voice_pan_spread);
            }
            return 0;
        }

        // 3. Mode: Polyphonic
        // Priority A: If same note already active on a voice -> retrigger it
        for (size_t i = 0; i < m_polyphony_limit; ++i) {
            if (m_voices[i].active && m_voices[i].note == midi_note) {
                m_voices[i].trigger(midi_note, velocity, now_ts, m_voices[i].pan);
                return static_cast<int32_t>(i);
            }
        }

        // Priority B: Find idle voice
        for (size_t i = 0; i < m_polyphony_limit; ++i) {
            if (!m_voices[i].active) {
                float pan_pos = calculate_voice_pan(i, m_polyphony_limit);
                m_voices[i].trigger(midi_note, velocity, now_ts, pan_pos * m_voice_pan_spread);
                return static_cast<int32_t>(i);
            }
        }

        // Priority C: Voice Stealing
        // Sub-priority 1: Pick released voice with lowest envelope level
        int32_t best_steal_idx = -1;
        float lowest_env = 1e9f;
        for (size_t i = 0; i < m_polyphony_limit; ++i) {
            if (!m_voices[i].gate_held) {
                float cur_lvl = m_voices[i].amp_env.current_value();
                if (cur_lvl < lowest_env) {
                    lowest_env = cur_lvl;
                    best_steal_idx = static_cast<int32_t>(i);
                }
            }
        }

        // Sub-priority 2: If all gates held, steal oldest voice (smallest timestamp)
        if (best_steal_idx < 0) {
            uint64_t oldest_ts = std::numeric_limits<uint64_t>::max();
            for (size_t i = 0; i < m_polyphony_limit; ++i) {
                if (m_voices[i].trigger_timestamp < oldest_ts) {
                    oldest_ts = m_voices[i].trigger_timestamp;
                    best_steal_idx = static_cast<int32_t>(i);
                }
            }
        }

        if (best_steal_idx >= 0) {
            float pan_pos = calculate_voice_pan(static_cast<size_t>(best_steal_idx), m_polyphony_limit);
            m_voices[best_steal_idx].trigger(midi_note, velocity, now_ts, pan_pos * m_voice_pan_spread);
            return best_steal_idx;
        }

        return -1;
    }

    void note_off(uint8_t midi_note) noexcept {
        if (m_play_mode == PolyphonyPlayMode::Unison4x) {
            for (size_t u = 0; u < 4; ++u) {
                if (m_voices[u].note == midi_note) {
                    if (m_sustain_pedal_held) {
                        m_voices[u].gate_held = false;
                    } else {
                        m_voices[u].release();
                    }
                }
            }
            return;
        }
        for (size_t i = 0; i < m_polyphony_limit; ++i) {
            if (m_voices[i].active && m_voices[i].note == midi_note && m_voices[i].gate_held) {
                if (m_sustain_pedal_held) {
                    m_voices[i].gate_held = false;
                } else {
                    m_voices[i].release();
                }
            }
        }
    }

    void set_sustain_pedal(bool held) noexcept {
        m_sustain_pedal_held = held;
        if (!held) {
            // Release any voices whose keys are no longer physically held
            for (size_t i = 0; i < kMaxVoices; ++i) {
                if (m_voices[i].active && !m_voices[i].gate_held) {
                    m_voices[i].release();
                }
            }
        }
    }
    [[nodiscard]] bool is_sustain_pedal_held() const noexcept { return m_sustain_pedal_held; }

    void set_pitch_bend_norm(float norm) noexcept {
        m_pitch_bend_norm = std::clamp(norm, -1.0f, 1.0f);
        m_pitch_bend_semitones = m_pitch_bend_norm * m_pitch_bend_range_semitones;
    }
    [[nodiscard]] float pitch_bend_norm() const noexcept { return m_pitch_bend_norm; }
    [[nodiscard]] float pitch_bend_semitones() const noexcept { return m_pitch_bend_semitones; }

    void set_pitch_bend_range(float semitones) noexcept {
        m_pitch_bend_range_semitones = std::clamp(semitones, 0.0f, 24.0f);
        m_pitch_bend_semitones = m_pitch_bend_norm * m_pitch_bend_range_semitones;
    }
    [[nodiscard]] float pitch_bend_range() const noexcept { return m_pitch_bend_range_semitones; }

    void handle_midi_event(const MidiEvent& ev) noexcept {
        switch (ev.type()) {
            case MidiStatus::NoteOn:
                if (ev.velocity() > 0) {
                    note_on(ev.note(), static_cast<float>(ev.velocity()) / 127.0f);
                } else {
                    note_off(ev.note());
                }
                break;
            case MidiStatus::NoteOff:
                note_off(ev.note());
                break;
            case MidiStatus::ControlChange:
                if (ev.data1 == 64) {
                    set_sustain_pedal(ev.data2 >= 64);
                } else if (ev.data1 == 120 || ev.data1 == 123) {
                    all_notes_off();
                }
                break;
            case MidiStatus::PitchBend: {
                int pb = (static_cast<int>(ev.data2) << 7) | static_cast<int>(ev.data1);
                float norm_pb = (static_cast<float>(pb) - 8192.0f) / 8192.0f;
                set_pitch_bend_norm(norm_pb);
                break;
            }
            default:
                break;
        }
    }

    void all_notes_off() noexcept {
        for (size_t i = 0; i < kMaxVoices; ++i) {
            m_voices[i].release();
        }
    }

    void panic() noexcept {
        for (size_t i = 0; i < kMaxVoices; ++i) {
            m_voices[i].reset();
        }
    }

    // Chord triggers for testing and interactive auditioning
    void play_chord(const std::vector<uint8_t>& notes, float velocity = 0.85f) noexcept {
        for (uint8_t n : notes) {
            note_on(n, velocity);
        }
    }

    void release_chord(const std::vector<uint8_t>& notes) noexcept {
        for (uint8_t n : notes) {
            note_off(n);
        }
    }

    // ========================================================================
    // Parameter Controls
    // ========================================================================
    void set_polyphony_limit(size_t limit) noexcept {
        m_polyphony_limit = std::clamp(limit, size_t{1}, kMaxVoices);
    }
    [[nodiscard]] size_t polyphony_limit() const noexcept { return m_polyphony_limit; }

    void set_play_mode(PolyphonyPlayMode mode) noexcept { m_play_mode = mode; }
    [[nodiscard]] PolyphonyPlayMode play_mode() const noexcept { return m_play_mode; }

    void set_osc1_waveform(dsp::Waveform wf) noexcept {
        m_osc1_wf = wf;
        for (auto& v : m_voices) v.osc1.set_waveform(wf);
    }
    [[nodiscard]] dsp::Waveform osc1_waveform() const noexcept { return m_osc1_wf; }

    void set_osc2_waveform(dsp::Waveform wf) noexcept {
        m_osc2_wf = wf;
        for (auto& v : m_voices) v.osc2.set_waveform(wf);
    }
    [[nodiscard]] dsp::Waveform osc2_waveform() const noexcept { return m_osc2_wf; }

    void set_osc_mix(float mix) noexcept { m_osc_mix = std::clamp(mix, 0.0f, 1.0f); }
    [[nodiscard]] float osc_mix() const noexcept { return m_osc_mix; }

    void set_osc2_detune_cents(float cents) noexcept { m_osc2_detune_cents = std::clamp(cents, -50.0f, 50.0f); }
    [[nodiscard]] float osc2_detune_cents() const noexcept { return m_osc2_detune_cents; }

    void set_osc2_octave_offset(int oct) noexcept { m_osc2_octave_offset = std::clamp(oct, -2, 2); }
    [[nodiscard]] int osc2_octave_offset() const noexcept { return m_osc2_octave_offset; }

    void set_filter_type(dsp::FilterType type) noexcept {
        m_filter_type = type;
        for (auto& v : m_voices) v.filter.set_type(type);
    }
    [[nodiscard]] dsp::FilterType filter_type() const noexcept { return m_filter_type; }

    void set_base_cutoff(float hz) noexcept { m_base_cutoff = std::clamp(hz, 20.0f, 20000.0f); }
    [[nodiscard]] float base_cutoff() const noexcept { return m_base_cutoff; }

    void set_resonance_q(float q) noexcept { m_resonance_q = std::clamp(q, 0.5f, 20.0f); }
    [[nodiscard]] float resonance_q() const noexcept { return m_resonance_q; }

    void set_filter_env_amount(float amt) noexcept { m_filter_env_amount = std::clamp(amt, -10000.0f, 10000.0f); }
    [[nodiscard]] float filter_env_amount() const noexcept { return m_filter_env_amount; }

    void set_keytrack_amount(float kt) noexcept { m_keytrack_amount = std::clamp(kt, 0.0f, 1.0f); }
    [[nodiscard]] float keytrack_amount() const noexcept { return m_keytrack_amount; }

    void set_velocity_to_filter(float vtf) noexcept { m_velocity_to_filter = std::clamp(vtf, 0.0f, 1.0f); }
    [[nodiscard]] float velocity_to_filter() const noexcept { return m_velocity_to_filter; }

    void set_velocity_to_amp(float vta) noexcept { m_velocity_to_amp = std::clamp(vta, 0.0f, 1.0f); }
    [[nodiscard]] float velocity_to_amp() const noexcept { return m_velocity_to_amp; }

    void set_voice_pan_spread(float spread) noexcept { m_voice_pan_spread = std::clamp(spread, 0.0f, 1.0f); }
    [[nodiscard]] float voice_pan_spread() const noexcept { return m_voice_pan_spread; }

    void set_glide_time_ms(float ms) noexcept { m_glide_time_ms = std::clamp(ms, 0.0f, 500.0f); }
    [[nodiscard]] float glide_time_ms() const noexcept { return m_glide_time_ms; }

    void set_master_level(float lvl) noexcept { m_master_level = std::clamp(lvl, 0.0f, 2.0f); }
    [[nodiscard]] float master_level() const noexcept { return m_master_level; }

    [[nodiscard]] size_t active_voice_count() const noexcept {
        size_t count = 0;
        for (size_t i = 0; i < m_polyphony_limit; ++i) {
            if (m_voices[i].active) ++count;
        }
        return count;
    }

    void get_telemetry(std::array<PolyVoiceTelemetry, kMaxVoices>& out_telem) const noexcept {
        for (size_t i = 0; i < kMaxVoices; ++i) {
            const auto& v = m_voices[i];
            out_telem[i] = PolyVoiceTelemetry{
                .id = static_cast<uint32_t>(i),
                .active = v.active,
                .note = v.note,
                .velocity = v.velocity,
                .amp_level = v.amp_env.current_value(),
                .mod_level = v.mod_env.current_value(),
                .stage = v.amp_env.stage(),
                .pan = v.pan,
                .playhead_time = v.amp_env.playhead_time()
            };
        }
    }

    // ========================================================================
    // Audio Thread Real-Time Rendering
    // ========================================================================
    void process_sample(float& out_l, float& out_r,
                        double bpm = 120.0,
                        float mod_cutoff = 0.0f,
                        float mod_pitch = 0.0f,
                        float mod_amp = 0.0f) noexcept {
        out_l = 0.0f;
        out_r = 0.0f;

        const auto* amp_mseg = m_bound_amp_mseg ? m_bound_amp_mseg : &m_internal_amp_mseg;
        const auto* mod_mseg = m_bound_mod_mseg ? m_bound_mod_mseg : &m_internal_mod_mseg;

        const float osc2_detune_ratio = std::pow(2.0f, (m_osc2_detune_cents / 1200.0f) + static_cast<float>(m_osc2_octave_offset));
        const float glide_coeff = (m_glide_time_ms <= 1.0f) ? 1.0f :
            (1.0f - std::exp(-1000.0f / (m_glide_time_ms * static_cast<float>(m_sample_rate))));

        const size_t num_voices = (m_play_mode == PolyphonyPlayMode::MonoLegato) ? 1 :
                                  ((m_play_mode == PolyphonyPlayMode::Unison4x) ? 4 : m_polyphony_limit);

        for (size_t i = 0; i < num_voices; ++i) {
            auto& v = m_voices[i];
            if (!v.active) continue;

            // 1. Portamento / Glide
            if (std::abs(v.current_freq - v.target_freq) > 0.05f) {
                v.current_freq += (v.target_freq - v.current_freq) * glide_coeff;
            } else {
                v.current_freq = v.target_freq;
            }

            // 2. Frequency with pitch modulation and pitch bend
            float freq_mod = std::pow(2.0f, (mod_pitch + m_pitch_bend_semitones) / 12.0f);
            if (m_play_mode == PolyphonyPlayMode::Unison4x) {
                constexpr float unison_detunes[4] = { -10.0f, -3.0f, +3.0f, +10.0f };
                freq_mod *= std::pow(2.0f, unison_detunes[i] / 1200.0f);
            }
            const float f1 = v.current_freq * freq_mod;
            const float f2 = f1 * osc2_detune_ratio;
            v.osc1.set_frequency(f1);
            v.osc2.set_frequency(f2);

            // 3. Generate Oscillators
            Sample s1 = v.osc1.process_sample();
            Sample s2 = v.osc2.process_sample();
            Sample mixed = (s1 * (1.0f - m_osc_mix)) + (s2 * m_osc_mix);

            // 4. Process Envelopes
            float a_env = amp_mseg->process_voice_sample(v.amp_env, m_sample_rate, bpm);
            float m_env = mod_mseg->process_voice_sample(v.mod_env, m_sample_rate, bpm);

            if (!v.amp_env.is_active()) {
                v.active = false;
                continue;
            }

            // 5. Filter Cutoff with Keytracking, Mod Envelope, and Global Mod
            float key_ratio = std::pow(2.0f, ((static_cast<float>(v.note) - 60.0f) / 12.0f) * m_keytrack_amount);
            float vel_filter_scale = 1.0f + ((v.velocity - 0.5f) * m_velocity_to_filter);
            float raw_cutoff = (m_base_cutoff * key_ratio * vel_filter_scale) + (m_filter_env_amount * m_env) + mod_cutoff;
            float nyquist = static_cast<float>(m_sample_rate) * 0.48f;
            float clamped_cutoff = std::clamp(raw_cutoff, 20.0f, nyquist);

            v.filter.set_type(m_filter_type);
            v.filter.set_cutoff(clamped_cutoff);
            v.filter.set_q(m_resonance_q);

            Sample filtered = v.filter.process_sample(mixed);

            // 6. Voice VCA & Velocity
            float vel_amp_scale = (1.0f - m_velocity_to_amp) + (v.velocity * m_velocity_to_amp);
            float global_amp_scale = std::max(0.0f, 1.0f + mod_amp);
            float voice_level = filtered * a_env * vel_amp_scale * global_amp_scale * m_master_level;

            // 7. Stereo Constant Power Panning
            float pan_ang = (v.pan + 1.0f) * 0.25f * std::numbers::pi_v<float>;
            float pan_l = std::cos(pan_ang) * std::numbers::sqrt2_v<float>;
            float pan_r = std::sin(pan_ang) * std::numbers::sqrt2_v<float>;

            out_l += voice_level * pan_l;
            out_r += voice_level * pan_r;
        }

        // Headroom scaling to prevent clipping on dense chords
        const float headroom = 1.0f / std::sqrt(std::max(1.0f, static_cast<float>(m_polyphony_limit * 0.35f)));
        out_l *= headroom;
        out_r *= headroom;
    }

    void process_block(float* out_l, float* out_r, uint32_t frames,
                       double bpm = 120.0,
                       float mod_cutoff = 0.0f,
                       float mod_pitch = 0.0f,
                       float mod_amp = 0.0f) noexcept {
        if (!out_l || !out_r || frames == 0) return;
        for (uint32_t i = 0; i < frames; ++i) {
            float l = 0.0f;
            float r = 0.0f;
            process_sample(l, r, bpm, mod_cutoff, mod_pitch, mod_amp);
            out_l[i] += l;
            out_r[i] += r;
        }
    }

private:
    [[nodiscard]] float calculate_voice_pan(size_t voice_idx, size_t limit) const noexcept {
        if (limit <= 1) return 0.0f;
        int sign = (voice_idx % 2 == 0) ? -1 : 1;
        float tier = static_cast<float>((voice_idx / 2) + 1) / static_cast<float>((limit / 2) + 1);
        return std::clamp(static_cast<float>(sign) * tier, -1.0f, 1.0f);
    }

    uint32_t m_sample_rate{48000};
    uint64_t m_timestamp_counter{0};
    size_t m_polyphony_limit{8};
    PolyphonyPlayMode m_play_mode{PolyphonyPlayMode::Polyphonic};

    std::array<PolyVoice, kMaxVoices> m_voices;

    const MultiStageEnvelope* m_bound_amp_mseg{nullptr};
    const MultiStageEnvelope* m_bound_mod_mseg{nullptr};
    MultiStageEnvelope m_internal_amp_mseg;
    MultiStageEnvelope m_internal_mod_mseg;

    dsp::Waveform m_osc1_wf{dsp::Waveform::Saw};
    dsp::Waveform m_osc2_wf{dsp::Waveform::Square};
    float m_osc_mix{0.5f};
    float m_osc2_detune_cents{7.0f};
    int m_osc2_octave_offset{0};

    dsp::FilterType m_filter_type{dsp::FilterType::Lowpass};
    float m_base_cutoff{2500.0f};
    float m_resonance_q{1.8f};
    float m_filter_env_amount{3500.0f};
    float m_keytrack_amount{0.5f};
    float m_velocity_to_filter{0.5f};
    float m_velocity_to_amp{0.3f};

    float m_voice_pan_spread{0.6f};
    float m_glide_time_ms{0.0f};
    float m_master_level{1.0f};

    bool m_sustain_pedal_held{false};
    float m_pitch_bend_norm{0.0f};
    float m_pitch_bend_semitones{0.0f};
    float m_pitch_bend_range_semitones{2.0f};
};

} // namespace audio_core::modulation
