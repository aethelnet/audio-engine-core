#pragma once

#include "audio_core/types.hpp"
#include "audio_core/insert_slot.hpp"
#include "audio_core/dsp/lookahead_delay_buffer.hpp"
#include "audio_core/dsp/linkwitz_riley_crossover.hpp"
#include <cmath>
#include <numbers>
#include <algorithm>
#include <array>
#include <cstdint>

namespace audio_core::dsp {

// ============================================================================
// LookaheadDnlProcessor: Zero-Allocation Multiband Dynamic Noise Limiter (DNL)
// ============================================================================
// Architecture:
// 1. Linkwitz-Riley LR4 4-Band Phase-Compensated Crossover:
//    - Band 0: Sub (0 .. 120 Hz)
//    - Band 1: Low-Mid (120 .. 1200 Hz)
//    - Band 2: High-Mid (1200 .. 6000 Hz) - Dynamic expansion / noise gating
//    - Band 3: Air / Treble (6000 .. Nyquist) - Dynamic expansion / noise gating
//    - Prevents "Single-Band Noise Pumping": Low-frequency kick drums and basslines
//      are strictly isolated in Band 0/1 and cannot falsely modulate the Air band.
// 2. 32-Sample Zero-Allocation Lookahead Delay Buffer:
//    - Audio path is delayed by D samples (default: 32 samples = 0.667 ms @ 48 kHz).
//    - Sidechain detectors analyze the un-delayed incoming high-frequency signal.
//    - Gate envelope G(t) smoothly opens BEFORE the physical transient hits the output,
//      preventing transient blunting, clipped consonants, and high-frequency softening.
// 3. Continuous 1D-ODE Ballistic Smoothing:
//    - Smooth C^inf opening via hyperbolic tangent: target_g = tanh(excess * 1.5).
//    - Exponential ballistic release (default: 25 ms) eliminates gate chatter and clicks.
// 4. Bit-Exact Reconstruction:
//    - When gates are fully open, LinkwitzRiley4Way guarantees flat 0.00 dB magnitude sum.
// ============================================================================
class LookaheadDnlProcessor : public IProcessor {
public:
    static constexpr size_t kMaxLookahead = 64;

    explicit LookaheadDnlProcessor(uint32_t sample_rate = 48000) noexcept
        : m_sample_rate(sample_rate ? sample_rate : 48000),
          m_crossover(120.0f, 1200.0f, 6000.0f, sample_rate) {
        init(sample_rate);
    }

    void init(uint32_t sample_rate) noexcept override {
        if (sample_rate == 0) return;
        m_sample_rate = sample_rate;
        m_crossover.set_crossovers(m_f_sub, m_f_mid, m_f_air, m_sample_rate);
        update_ballistics();
        reset();
    }

    void reset() noexcept override {
        m_crossover.reset();
        for (auto& buf : m_band_delays) {
            buf.reset();
            buf.set_delay_frames(m_lookahead_frames);
        }
        m_env_highmid = 0.0f;
        m_env_air = 0.0f;
        m_gain_highmid = 1.0f;
        m_gain_air = 1.0f;
    }

    [[nodiscard]] const char* name() const noexcept override {
        return "Sovereign Lookahead Multiband DNL";
    }

    // Parameter count & reflection
    [[nodiscard]] uint32_t parameter_count() const noexcept override { return 6; }
    [[nodiscard]] const char* parameter_name(uint32_t index) const noexcept override {
        switch (index) {
            case 0: return "Threshold dB";
            case 1: return "Max Reduction dB";
            case 2: return "Lookahead Frames";
            case 3: return "Release ms";
            case 4: return "Air Split Hz";
            case 5: return "Mid Split Hz";
            default: return "Reserved";
        }
    }

    void set_parameter(uint32_t index, float value) noexcept override {
        switch (index) {
            case 0: // Threshold [-70.0, -10.0] dBFS
                m_threshold_db = std::clamp(value, -70.0f, -10.0f);
                m_thresh_linear = std::pow(10.0f, m_threshold_db / 20.0f);
                break;
            case 1: // Max Reduction [0.0, 36.0] dB
                m_max_reduction_db = std::clamp(value, 0.0f, 36.0f);
                m_min_gain = std::pow(10.0f, -m_max_reduction_db / 20.0f);
                break;
            case 2: // Lookahead [0, 64] frames
                set_lookahead_frames(static_cast<uint32_t>(std::clamp(value, 0.0f, static_cast<float>(kMaxLookahead))));
                break;
            case 3: // Release [5.0, 200.0] ms
                m_release_ms = std::clamp(value, 5.0f, 200.0f);
                update_ballistics();
                break;
            case 4: // Air Split [2000.0, 12000.0] Hz
                m_f_air = std::clamp(value, 2000.0f, 12000.0f);
                m_crossover.set_crossovers(m_f_sub, m_f_mid, m_f_air, m_sample_rate);
                break;
            case 5: // Mid Split [500.0, 3000.0] Hz
                m_f_mid = std::clamp(value, 500.0f, 3000.0f);
                m_crossover.set_crossovers(m_f_sub, m_f_mid, m_f_air, m_sample_rate);
                break;
            default:
                break;
        }
    }

    [[nodiscard]] float get_parameter(uint32_t index) const noexcept override {
        switch (index) {
            case 0: return m_threshold_db;
            case 1: return m_max_reduction_db;
            case 2: return static_cast<float>(m_lookahead_frames);
            case 3: return m_release_ms;
            case 4: return m_f_air;
            case 5: return m_f_mid;
            default: return 0.0f;
        }
    }

    void set_lookahead_frames(uint32_t frames) noexcept {
        m_lookahead_frames = std::clamp(frames, 0u, static_cast<uint32_t>(kMaxLookahead));
        for (auto& buf : m_band_delays) {
            buf.set_delay_frames(m_lookahead_frames);
        }
    }

    [[nodiscard]] uint32_t lookahead_frames() const noexcept { return m_lookahead_frames; }
    [[nodiscard]] float current_air_gain_reduction_db() const noexcept {
        if (m_gain_air <= 1e-5f) return -m_max_reduction_db;
        return 20.0f * std::log10(m_gain_air);
    }
    [[nodiscard]] float current_highmid_gain_reduction_db() const noexcept {
        if (m_gain_highmid <= 1e-5f) return -m_max_reduction_db;
        return 20.0f * std::log10(m_gain_highmid);
    }
    [[nodiscard]] float current_air_envelope() const noexcept { return m_env_air; }

    void process_stereo(Sample* left, Sample* right, uint32_t frames) noexcept override {
        if (!left || !right || frames == 0) return;

        float bands_l[LinkwitzRiley4Way::kNumBands];
        float bands_r[LinkwitzRiley4Way::kNumBands];

        for (uint32_t i = 0; i < frames; ++i) {
            const float in_l = left[i];
            const float in_r = right[i];

            // 1. Split incoming un-delayed signal into 4 bands
            m_crossover.process_sample(in_l, in_r, bands_l, bands_r);

            // 2. Un-delayed Sidechain Peak/RMS Detection on Band 2 (High-Mid) and Band 3 (Air)
            const float hm_mag = 0.5f * (std::abs(bands_l[2]) + std::abs(bands_r[2]));
            const float air_mag = 0.5f * (std::abs(bands_l[3]) + std::abs(bands_r[3]));

            // Fast ballistic attack (0.2 ms), smooth exponential release
            if (hm_mag > m_env_highmid) {
                m_env_highmid += m_attack_coeff * (hm_mag - m_env_highmid);
            } else {
                m_env_highmid += m_release_coeff * (hm_mag - m_env_highmid);
            }

            if (air_mag > m_env_air) {
                m_env_air += m_attack_coeff * (air_mag - m_env_air);
            } else {
                m_env_air += m_release_coeff * (air_mag - m_env_air);
            }

            // 3. Compute Target Openness via Smooth Hyperbolic Tangent
            // Below threshold: gain reduces towards m_min_gain
            // Above threshold: gain ramps to 1.0f (fully transparent)
            float target_g_hm = 1.0f;
            if (m_env_highmid < m_thresh_linear) {
                const float deficit = (m_thresh_linear - m_env_highmid) / m_thresh_linear;
                const float atten_factor = std::tanh(deficit * 2.0f);
                target_g_hm = 1.0f - atten_factor * (1.0f - m_min_gain);
            }

            float target_g_air = 1.0f;
            if (m_env_air < m_thresh_linear) {
                const float deficit = (m_thresh_linear - m_env_air) / m_thresh_linear;
                const float atten_factor = std::tanh(deficit * 2.5f);
                target_g_air = 1.0f - atten_factor * (1.0f - m_min_gain);
            }

            // ODE Parameter Smoother on band gains
            m_gain_highmid += 0.15f * (target_g_hm - m_gain_highmid);
            m_gain_air += 0.15f * (target_g_air - m_gain_air);

            // 4. Push all 4 bands into Lookahead Delay Lines
            float delayed_b_l[LinkwitzRiley4Way::kNumBands];
            float delayed_b_r[LinkwitzRiley4Way::kNumBands];

            for (size_t b = 0; b < LinkwitzRiley4Way::kNumBands; ++b) {
                m_band_delays[b].push(bands_l[b], bands_r[b]);
                m_band_delays[b].read_delayed(delayed_b_l[b], delayed_b_r[b]);
            }

            // 5. Apply Lookahead Attenuation to Band 2 and Band 3
            // Band 0 (Sub) and Band 1 (Low-Mid) are 100% clean and un-gated!
            delayed_b_l[2] *= m_gain_highmid;
            delayed_b_r[2] *= m_gain_highmid;

            delayed_b_l[3] *= m_gain_air;
            delayed_b_r[3] *= m_gain_air;

            // 6. Recombine all 4 bands into output stereo frame
            float out_l = 0.0f, out_r = 0.0f;
            m_crossover.sum_bands(delayed_b_l, delayed_b_r, out_l, out_r);

            left[i] = out_l;
            right[i] = out_r;
        }
    }

private:
    void update_ballistics() noexcept {
        const float dt = 1.0f / static_cast<float>(m_sample_rate);
        // Attack 0.2 ms
        m_attack_coeff = 1.0f - std::exp(-dt / 0.0002f);
        // Release
        const float rel_sec = std::max(0.005f, m_release_ms * 0.001f);
        m_release_coeff = 1.0f - std::exp(-dt / rel_sec);
    }

    uint32_t m_sample_rate{48000};
    uint32_t m_lookahead_frames{32};

    float m_f_sub{120.0f};
    float m_f_mid{1200.0f};
    float m_f_air{6000.0f};

    float m_threshold_db{-36.0f};
    float m_thresh_linear{0.01585f};
    float m_max_reduction_db{18.0f};
    float m_min_gain{0.12589f}; // -18 dB
    float m_release_ms{25.0f};

    float m_attack_coeff{0.5f};
    float m_release_coeff{0.001f};

    float m_env_highmid{0.0f};
    float m_env_air{0.0f};
    float m_gain_highmid{1.0f};
    float m_gain_air{1.0f};

    LinkwitzRiley4Way m_crossover;
    std::array<LookaheadDelayBuffer<kMaxLookahead>, LinkwitzRiley4Way::kNumBands> m_band_delays{};
};

} // namespace audio_core::dsp
