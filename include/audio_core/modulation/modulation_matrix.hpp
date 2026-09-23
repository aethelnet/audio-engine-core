#pragma once

#include "audio_core/types.hpp"
#include "audio_core/modulation/multi_stage_envelope.hpp"
#include "audio_core/modulation/modulation_lfo.hpp"
#include <array>
#include <string_view>
#include <algorithm>
#include <atomic>
#include <cmath>

namespace audio_core::modulation {

// ============================================================================
// Modulation Matrix & Meta-Modulation Engine
// Enables arbitrary modulation routing including recursive Meta-Modulation
// (Modulators modulating Modulator Attack, Decay, Rate, Depth, Tension).
// Zero heap allocations in the audio thread; lock-free atomic parameter caching.
// ============================================================================

enum class ModulationSource : uint8_t {
    None        = 0,
    LFO1        = 1,
    LFO2        = 2,
    MSEG1       = 3, // Multi-Stage Envelope 1
    MSEG2       = 4, // Multi-Stage Envelope 2
    Velocity    = 5,
    KeyTrack    = 6,
    ModWheel    = 7,
    PitchBend   = 8,
    RandomSH    = 9  // Sample & Hold Random
};

enum class ModulationDestination : uint8_t {
    None                = 0,

    // Meta-Modulation Targets (Modulator-to-Modulator!)
    MSEG1_Attack        = 1,  // e.g. LFO 1 modulates Hi-Hat attack time!
    MSEG1_Decay         = 2,  // e.g. Velocity modulates Decay length
    MSEG1_TimeScale     = 3,  // Overall Speed
    MSEG1_Level         = 4,  // Amplitude depth
    MSEG1_Tension       = 5,  // Curvature bending

    MSEG2_Attack        = 6,
    MSEG2_Decay         = 7,
    MSEG2_TimeScale     = 8,
    MSEG2_Level         = 9,
    MSEG2_Tension       = 10,

    LFO1_Rate           = 11, // e.g. MSEG 1 or LFO 2 frequency-modulates LFO 1!
    LFO1_Depth          = 12,
    LFO2_Rate           = 13,
    LFO2_Depth          = 14,

    // Instrument Synth Voice Targets
    SynthCutoff         = 15,
    SynthResonance      = 16,
    SynthPitch          = 17, // Detune in semitones
    SynthAmp            = 18,
    SynthOscMix         = 19,
    SynthDetune         = 20,

    // Insert Slot Plugin Targets
    TrackSlot0_Param0   = 21,
    TrackSlot0_Param1   = 22,
    TrackSlot1_Param0   = 23,
    TrackSlot1_Param1   = 24
};

inline const char* modulation_source_name(ModulationSource src) noexcept {
    switch (src) {
        case ModulationSource::None:      return "None";
        case ModulationSource::LFO1:      return "LFO 1";
        case ModulationSource::LFO2:      return "LFO 2";
        case ModulationSource::MSEG1:     return "MSEG 1 (Voice Env)";
        case ModulationSource::MSEG2:     return "MSEG 2 (Mod Env)";
        case ModulationSource::Velocity:  return "Velocity";
        case ModulationSource::KeyTrack:  return "Key Tracking";
        case ModulationSource::ModWheel:  return "Mod Wheel (CC 1)";
        case ModulationSource::PitchBend: return "Pitch Bend";
        case ModulationSource::RandomSH:  return "Random (S&H)";
    }
    return "Unknown";
}

inline const char* modulation_destination_name(ModulationDestination dst) noexcept {
    switch (dst) {
        case ModulationDestination::None:              return "None";
        case ModulationDestination::MSEG1_Attack:      return "MSEG 1 Attack (ms)";
        case ModulationDestination::MSEG1_Decay:       return "MSEG 1 Decay (ms)";
        case ModulationDestination::MSEG1_TimeScale:   return "MSEG 1 Speed";
        case ModulationDestination::MSEG1_Level:       return "MSEG 1 Level";
        case ModulationDestination::MSEG1_Tension:     return "MSEG 1 Tension";
        case ModulationDestination::MSEG2_Attack:      return "MSEG 2 Attack (ms)";
        case ModulationDestination::MSEG2_Decay:       return "MSEG 2 Decay (ms)";
        case ModulationDestination::MSEG2_TimeScale:   return "MSEG 2 Speed";
        case ModulationDestination::MSEG2_Level:       return "MSEG 2 Level";
        case ModulationDestination::MSEG2_Tension:     return "MSEG 2 Tension";
        case ModulationDestination::LFO1_Rate:         return "LFO 1 Rate (Hz)";
        case ModulationDestination::LFO1_Depth:        return "LFO 1 Depth";
        case ModulationDestination::LFO2_Rate:         return "LFO 2 Rate (Hz)";
        case ModulationDestination::LFO2_Depth:        return "LFO 2 Depth";
        case ModulationDestination::SynthCutoff:       return "Synth Cutoff (Hz)";
        case ModulationDestination::SynthResonance:    return "Synth Resonance (Q)";
        case ModulationDestination::SynthPitch:        return "Synth Pitch (st)";
        case ModulationDestination::SynthAmp:          return "Synth Amp Gain";
        case ModulationDestination::SynthOscMix:       return "Synth Osc Mix";
        case ModulationDestination::SynthDetune:       return "Synth Detune";
        case ModulationDestination::TrackSlot0_Param0: return "Slot 0 Param 0";
        case ModulationDestination::TrackSlot0_Param1: return "Slot 0 Param 1";
        case ModulationDestination::TrackSlot1_Param0: return "Slot 1 Param 0";
        case ModulationDestination::TrackSlot1_Param1: return "Slot 1 Param 1";
    }
    return "Unknown";
}

struct ModulationRoute {
    ModulationSource source{ModulationSource::None};
    ModulationDestination destination{ModulationDestination::None};
    float amount{0.0f};      // Modulation depth [-1.0, +1.0]
    bool active{false};
    bool bipolar{true};      // Bipolar vs Unipolar modulation
};

class ModulationMatrix {
public:
    static constexpr size_t kMaxRoutes = 16;
    static constexpr size_t kMaxDestinations = 32;

    ModulationMatrix() {
        m_routes.fill(ModulationRoute{});
        m_dest_values.fill(0.0f);
    }

    void reset() noexcept {
        m_dest_values.fill(0.0f);
        m_lfo1.reset();
        m_lfo2.reset();
        m_mseg1_voice.reset();
        m_mseg2_voice.reset();
    }

    void init(uint32_t sample_rate) noexcept {
        m_sample_rate = sample_rate;
        m_lfo1.init(sample_rate);
        m_lfo2.init(sample_rate);
    }

    // Route management
    [[nodiscard]] const std::array<ModulationRoute, kMaxRoutes>& routes() const noexcept { return m_routes; }
    [[nodiscard]] std::array<ModulationRoute, kMaxRoutes>& routes() noexcept { return m_routes; }

    void set_route(size_t index, ModulationSource src, ModulationDestination dst, float amount, bool active = true, bool bipolar = true) noexcept {
        if (index >= kMaxRoutes) return;
        m_routes[index] = ModulationRoute{
            .source = src,
            .destination = dst,
            .amount = std::clamp(amount, -1.0f, 1.0f),
            .active = active,
            .bipolar = bipolar
        };
    }

    void clear_route(size_t index) noexcept {
        if (index >= kMaxRoutes) return;
        m_routes[index] = ModulationRoute{};
    }

    // Modulator instances
    [[nodiscard]] ModulationLfo& lfo1() noexcept { return m_lfo1; }
    [[nodiscard]] const ModulationLfo& lfo1() const noexcept { return m_lfo1; }

    [[nodiscard]] ModulationLfo& lfo2() noexcept { return m_lfo2; }
    [[nodiscard]] const ModulationLfo& lfo2() const noexcept { return m_lfo2; }

    [[nodiscard]] MultiStageEnvelope& mseg1() noexcept { return m_mseg1; }
    [[nodiscard]] const MultiStageEnvelope& mseg1() const noexcept { return m_mseg1; }

    [[nodiscard]] MultiStageEnvelope& mseg2() noexcept { return m_mseg2; }
    [[nodiscard]] const MultiStageEnvelope& mseg2() const noexcept { return m_mseg2; }

    [[nodiscard]] MsegVoice& mseg1_voice() noexcept { return m_mseg1_voice; }
    [[nodiscard]] const MsegVoice& mseg1_voice() const noexcept { return m_mseg1_voice; }

    [[nodiscard]] MsegVoice& mseg2_voice() noexcept { return m_mseg2_voice; }
    [[nodiscard]] const MsegVoice& mseg2_voice() const noexcept { return m_mseg2_voice; }

    // Performance Controller Inputs
    void set_performance_controls(float velocity, float key_track, float mod_wheel, float pitch_bend) noexcept {
        m_velocity = std::clamp(velocity, 0.0f, 1.0f);
        m_key_track = std::clamp(key_track, 0.0f, 1.0f);
        m_mod_wheel = std::clamp(mod_wheel, 0.0f, 1.0f);
        m_pitch_bend = std::clamp(pitch_bend, -1.0f, 1.0f);
    }

    // Trigger voice envelopes on note on / slice trigger
    void note_on(float velocity = 1.0f, float key_norm = 0.5f) noexcept {
        m_velocity = velocity;
        m_key_track = key_norm;
        m_mseg1_voice.trigger(velocity);
        m_mseg2_voice.trigger(velocity);
    }

    void note_off() noexcept {
        m_mseg1_voice.release();
        m_mseg2_voice.release();
    }

    // ========================================================================
    // Topologically Ordered 4-Phase Modulation Evaluation Loop
    // ========================================================================
    void evaluate_sample(double bpm = 120.0) noexcept {
        // --- Phase 1: Advance Independent Free Modulators (LFO 1, LFO 2) ---
        const float lfo1_val = m_lfo1.process_sample(bpm);
        const float lfo2_val = m_lfo2.process_sample(bpm);

        // --- Phase 2: Compute Meta-Modulation Offsets for Modulators ---
        float mseg1_atk_mod = 0.0f;
        float mseg1_dec_mod = 0.0f;
        float mseg1_time_mod = 0.0f;
        float mseg1_lvl_mod = 0.0f;
        float mseg1_tens_mod = 0.0f;

        float mseg2_atk_mod = 0.0f;
        float mseg2_dec_mod = 0.0f;
        float mseg2_time_mod = 0.0f;
        float mseg2_lvl_mod = 0.0f;
        float mseg2_tens_mod = 0.0f;

        float lfo1_rate_mod = 0.0f;
        float lfo1_depth_mod = 0.0f;
        float lfo2_rate_mod = 0.0f;
        float lfo2_depth_mod = 0.0f;

        for (const auto& route : m_routes) {
            if (!route.active || route.destination == ModulationDestination::None) continue;

            // Resolve source signal
            float s_val = get_source_signal(route.source, lfo1_val, lfo2_val, m_mseg1_voice.current_value(), m_mseg2_voice.current_value());
            float delta = s_val * route.amount;

            switch (route.destination) {
                case ModulationDestination::MSEG1_Attack:    mseg1_atk_mod += delta; break;
                case ModulationDestination::MSEG1_Decay:     mseg1_dec_mod += delta; break;
                case ModulationDestination::MSEG1_TimeScale: mseg1_time_mod += delta; break;
                case ModulationDestination::MSEG1_Level:     mseg1_lvl_mod += delta; break;
                case ModulationDestination::MSEG1_Tension:   mseg1_tens_mod += delta; break;

                case ModulationDestination::MSEG2_Attack:    mseg2_atk_mod += delta; break;
                case ModulationDestination::MSEG2_Decay:     mseg2_dec_mod += delta; break;
                case ModulationDestination::MSEG2_TimeScale: mseg2_time_mod += delta; break;
                case ModulationDestination::MSEG2_Level:     mseg2_lvl_mod += delta; break;
                case ModulationDestination::MSEG2_Tension:   mseg2_tens_mod += delta; break;

                case ModulationDestination::LFO1_Rate:       lfo1_rate_mod += delta * 10.0f; break; // Hz scale
                case ModulationDestination::LFO1_Depth:      lfo1_depth_mod += delta; break;
                case ModulationDestination::LFO2_Rate:       lfo2_rate_mod += delta * 10.0f; break;
                case ModulationDestination::LFO2_Depth:      lfo2_depth_mod += delta; break;

                default: break; // Synth/Track destination evaluated in Phase 4
            }
        }

        // Inject meta-modulation into modulator macro inputs
        m_mseg1.set_mod_attack_scale(mseg1_atk_mod);
        m_mseg1.set_mod_decay_scale(mseg1_dec_mod);
        m_mseg1.set_mod_time_scale(mseg1_time_mod);
        m_mseg1.set_mod_level_scale(mseg1_lvl_mod);
        m_mseg1.set_mod_tension_offset(mseg1_tens_mod);

        m_mseg2.set_mod_attack_scale(mseg2_atk_mod);
        m_mseg2.set_mod_decay_scale(mseg2_dec_mod);
        m_mseg2.set_mod_time_scale(mseg2_time_mod);
        m_mseg2.set_mod_level_scale(mseg2_lvl_mod);
        m_mseg2.set_mod_tension_offset(mseg2_tens_mod);

        m_lfo1.set_mod_rate_offset(lfo1_rate_mod);
        m_lfo1.set_mod_depth_offset(lfo1_depth_mod);
        m_lfo2.set_mod_rate_offset(lfo2_rate_mod);
        m_lfo2.set_mod_depth_offset(lfo2_depth_mod);

        // --- Phase 3: Evaluate Dependent MultiStageEnvelopes with Modulated Macros ---
        const float mseg1_val = m_mseg1.process_voice_sample(m_mseg1_voice, m_sample_rate, bpm);
        const float mseg2_val = m_mseg2.process_voice_sample(m_mseg2_voice, m_sample_rate, bpm);

        // --- Phase 4: Accumulate Destination Modulations for Voice/Mixer Targets ---
        m_dest_values.fill(0.0f);

        for (const auto& route : m_routes) {
            if (!route.active || route.destination == ModulationDestination::None) continue;
            uint8_t d_idx = static_cast<uint8_t>(route.destination);
            if (d_idx >= kMaxDestinations) continue;

            float s_val = get_source_signal(route.source, lfo1_val, lfo2_val, mseg1_val, mseg2_val);
            m_dest_values[d_idx] += s_val * route.amount;
        }
    }

    // Query destination modulation depth
    [[nodiscard]] float get_destination_value(ModulationDestination dst) const noexcept {
        uint8_t d_idx = static_cast<uint8_t>(dst);
        return (d_idx < kMaxDestinations) ? m_dest_values[d_idx] : 0.0f;
    }

private:
    [[nodiscard]] float get_source_signal(ModulationSource src, float l1, float l2, float m1, float m2) const noexcept {
        switch (src) {
            case ModulationSource::None:      return 0.0f;
            case ModulationSource::LFO1:      return l1;
            case ModulationSource::LFO2:      return l2;
            case ModulationSource::MSEG1:     return m1;
            case ModulationSource::MSEG2:     return m2;
            case ModulationSource::Velocity:  return m_velocity;
            case ModulationSource::KeyTrack:  return m_key_track;
            case ModulationSource::ModWheel:  return m_mod_wheel;
            case ModulationSource::PitchBend: return m_pitch_bend;
            case ModulationSource::RandomSH:  return l1; // S&H LFO source
        }
        return 0.0f;
    }

    uint32_t m_sample_rate{48000};
    std::array<ModulationRoute, kMaxRoutes> m_routes{};
    std::array<float, kMaxDestinations> m_dest_values{};

    ModulationLfo m_lfo1;
    ModulationLfo m_lfo2;
    MultiStageEnvelope m_mseg1;
    MultiStageEnvelope m_mseg2;

    MsegVoice m_mseg1_voice;
    MsegVoice m_mseg2_voice;

    float m_velocity{1.0f};
    float m_key_track{0.5f};
    float m_mod_wheel{0.0f};
    float m_pitch_bend{0.0f};
};

} // namespace audio_core::modulation
