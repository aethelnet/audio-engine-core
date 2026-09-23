#pragma once

#include "audio_core/types.hpp"
#include <cmath>
#include <numbers>
#include <algorithm>
#include <atomic>
#include <random>

namespace audio_core::modulation {

enum class LfoWaveform : uint8_t {
    Sine        = 0,
    Triangle    = 1,
    SawUp       = 2,
    SawDown     = 3,
    Square      = 4,
    SampleHold  = 5, // Discrete stepped random on each cycle
    SmoothRandom= 6  // Linearly slewed random walk
};

// ============================================================================
// ModulationLfo: Precision Low-Frequency Oscillator
// Supports Hertz (Free) & BeatSync, multiple classic & chaotic waveforms,
// and modulatable Rate & Depth (LFO-on-LFO Frequency/Amplitude modulation).
// ============================================================================
class ModulationLfo {
public:
    ModulationLfo() = default;

    void init(uint32_t sample_rate) noexcept {
        m_sample_rate = std::max(1000u, sample_rate);
        reset();
    }

    void reset() noexcept {
        m_phase = 0.0;
        m_sh_val = 0.0f;
        m_sh_target = 0.0f;
        m_current_val = 0.0f;
    }

    void set_waveform(LfoWaveform wf) noexcept { m_waveform = wf; }
    [[nodiscard]] LfoWaveform waveform() const noexcept { return m_waveform; }

    void set_bipolar(bool bp) noexcept { m_bipolar = bp; }
    [[nodiscard]] bool is_bipolar() const noexcept { return m_bipolar; }

    // Frequency controls
    void set_frequency_hz(float hz) noexcept { m_base_rate_hz.store(std::clamp(hz, 0.01f, 200.0f), std::memory_order_relaxed); }
    [[nodiscard]] float frequency_hz() const noexcept { return m_base_rate_hz.load(std::memory_order_relaxed); }

    void set_beat_sync(bool sync) noexcept { m_beat_sync = sync; }
    [[nodiscard]] bool is_beat_sync() const noexcept { return m_beat_sync; }

    // In BeatSync mode: beats per cycle (e.g. 0.25 = 16th, 1.0 = 1/4 note, 4.0 = 1 bar @ 4/4)
    void set_beats_per_cycle(double beats) noexcept { m_beats_per_cycle = std::max(0.0625, beats); }
    [[nodiscard]] double beats_per_cycle() const noexcept { return m_beats_per_cycle; }

    void set_depth(float depth) noexcept { m_base_depth.store(std::clamp(depth, 0.0f, 1.0f), std::memory_order_relaxed); }
    [[nodiscard]] float depth() const noexcept { return m_base_depth.load(std::memory_order_relaxed); }

    // Dynamic modulation inputs (from other modulators)
    void set_mod_rate_offset(float offset) noexcept { m_mod_rate_offset.store(offset, std::memory_order_relaxed); }
    void set_mod_depth_offset(float offset) noexcept { m_mod_depth_offset.store(offset, std::memory_order_relaxed); }

    [[nodiscard]] float effective_rate_hz() const noexcept {
        return std::clamp(m_base_rate_hz.load(std::memory_order_relaxed) + m_mod_rate_offset.load(std::memory_order_relaxed), 0.005f, 250.0f);
    }

    [[nodiscard]] float effective_depth() const noexcept {
        return std::clamp(m_base_depth.load(std::memory_order_relaxed) + m_mod_depth_offset.load(std::memory_order_relaxed), 0.0f, 1.0f);
    }

    [[nodiscard]] float current_value() const noexcept { return m_current_val; }
    [[nodiscard]] double phase() const noexcept { return m_phase; }

    // Sample-rate evaluation
    float process_sample(double bpm = 120.0) noexcept {
        double d_phase = 0.0;
        if (m_beat_sync) {
            // Beat-sync phase advance
            const double bps = (bpm / 60.0) / static_cast<double>(m_sample_rate);
            const double rate_scale = std::max(0.05, 1.0 + static_cast<double>(m_mod_rate_offset.load(std::memory_order_relaxed) * 0.5f));
            d_phase = (bps / m_beats_per_cycle) * rate_scale;
        } else {
            // Free Hz phase advance
            const double eff_hz = static_cast<double>(effective_rate_hz());
            d_phase = eff_hz / static_cast<double>(m_sample_rate);
        }

        m_phase += d_phase;

        // Detect phase cycle wrap
        if (m_phase >= 1.0) {
            m_phase -= std::floor(m_phase);
            // Cycle wrapped: update S&H
            m_sh_val = m_sh_target;
            m_sh_target = next_random();
        }

        float raw = 0.0f;
        const float p = static_cast<float>(m_phase);

        switch (m_waveform) {
            case LfoWaveform::Sine:
                raw = std::sin(p * 2.0f * std::numbers::pi_v<float>);
                break;
            case LfoWaveform::Triangle:
                raw = (p < 0.5f) ? (4.0f * p - 1.0f) : (3.0f - 4.0f * p);
                break;
            case LfoWaveform::SawUp:
                raw = (2.0f * p) - 1.0f;
                break;
            case LfoWaveform::SawDown:
                raw = 1.0f - (2.0f * p);
                break;
            case LfoWaveform::Square:
                raw = (p < 0.5f) ? 1.0f : -1.0f;
                break;
            case LfoWaveform::SampleHold:
                raw = m_sh_val;
                break;
            case LfoWaveform::SmoothRandom:
                // Linear slew from m_sh_val to m_sh_target across cycle
                raw = m_sh_val + (p * (m_sh_target - m_sh_val));
                break;
        }

        if (!m_bipolar) {
            // Unipolar: map [-1, 1] to [0, 1]
            raw = (raw * 0.5f) + 0.5f;
        }

        m_current_val = raw * effective_depth();
        return m_current_val;
    }

private:
    float next_random() noexcept {
        // Fast Lehmer / LCG pseudo-random generator (zero allocations, real-time safe)
        m_rng_state = (m_rng_state * 1664525u) + 1013904223u;
        float norm = static_cast<float>(m_rng_state & 0x00FFFFFFu) / static_cast<float>(0x00FFFFFFu);
        return (norm * 2.0f) - 1.0f; // [-1.0, +1.0]
    }

    uint32_t m_sample_rate{48000};
    LfoWaveform m_waveform{LfoWaveform::Sine};
    bool m_bipolar{true};
    bool m_beat_sync{false};
    double m_beats_per_cycle{1.0}; // 1 beat default
    double m_phase{0.0};
    float m_current_val{0.0f};

    float m_sh_val{0.0f};
    float m_sh_target{0.0f};
    uint32_t m_rng_state{0x1337BEEF};

    std::atomic<float> m_base_rate_hz{2.0f};
    std::atomic<float> m_base_depth{1.0f};
    std::atomic<float> m_mod_rate_offset{0.0f};
    std::atomic<float> m_mod_depth_offset{0.0f};
};

} // namespace audio_core::modulation
