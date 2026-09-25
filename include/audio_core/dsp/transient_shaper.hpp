#pragma once

#include "audio_core/types.hpp"
#include "audio_core/insert_slot.hpp"
#include <cmath>
#include <algorithm>
#include <numbers>

namespace audio_core::dsp {

// ============================================================================
// TransientShaper: Real-Time Zero-Latency Transient Shaper Insert Processor
// Dual-Envelope Differential Follower (SPL / Tilgner topology)
// Features:
// - Fast onset follower vs. slow RMS/body follower differential ratio
// - Level-independent attack punch boost/cut (+/- 15 dB)
// - Selective sustain tail & room ring boost/cut (+/- 15 dB)
// - Sub-sample one-pole anti-zipper gain smoother
// - Analog tanh soft-saturation ceiling to cushion hard transient boosts
// - External Sidechain input support (e.g. kick-keyed synth shaper)
// - Strictly zero allocations, zero lookahead latency, 100% real-time safe
// ============================================================================
class TransientShaper : public IProcessor {
public:
    TransientShaper() = default;

    void init(uint32_t sample_rate) noexcept override {
        m_sample_rate = sample_rate > 0 ? sample_rate : 48000;
        update_coefficients();
        reset();
    }

    void reset() noexcept override {
        m_env_fast = 0.0f;
        m_env_slow = 0.0f;
        m_gain_smooth = 1.0f;
        m_held_peak = 0.0f;
        m_hold_count = 0;
    }

    [[nodiscard]] const char* name() const noexcept override {
        return "Sovereign Real-Time Transient Shaper";
    }

    [[nodiscard]] bool supports_sidechain() const noexcept override {
        return true;
    }

    // Parameters:
    // 0: Attack Gain (-15.0 dB to +15.0 dB)
    // 1: Sustain Gain (-15.0 dB to +15.0 dB)
    // 2: Attack Window (1.0 ms to 25.0 ms, default 8.0 ms)
    // 3: Output Trim (-15.0 dB to +15.0 dB, default 0.0 dB)
    // 4: Soft Saturation / Clip (0.0 = Linear Transparent, 1.0 = Analog Warmth Soft-Clip)
    void set_parameter(uint32_t index, float value) noexcept override {
        switch (index) {
            case 0:
                m_attack_gain_db = std::clamp(value, -15.0f, 15.0f);
                m_attack_gain_lin = std::pow(10.0f, m_attack_gain_db / 20.0f);
                break;
            case 1:
                m_sustain_gain_db = std::clamp(value, -15.0f, 15.0f);
                m_sustain_gain_lin = std::pow(10.0f, m_sustain_gain_db / 20.0f);
                break;
            case 2:
                m_attack_window_ms = std::clamp(value, 1.0f, 25.0f);
                update_coefficients();
                break;
            case 3:
                m_output_trim_db = std::clamp(value, -15.0f, 15.0f);
                m_output_trim_lin = std::pow(10.0f, m_output_trim_db / 20.0f);
                break;
            case 4:
                m_soft_clip = std::clamp(value, 0.0f, 1.0f);
                break;
            default:
                break;
        }
    }

    [[nodiscard]] float get_parameter(uint32_t index) const noexcept override {
        switch (index) {
            case 0: return m_attack_gain_db;
            case 1: return m_sustain_gain_db;
            case 2: return m_attack_window_ms;
            case 3: return m_output_trim_db;
            case 4: return m_soft_clip;
            default: return 0.0f;
        }
    }

    [[nodiscard]] uint32_t parameter_count() const noexcept override {
        return 5;
    }

    [[nodiscard]] const char* parameter_name(uint32_t index) const noexcept override {
        switch (index) {
            case 0: return "Attack (dB)";
            case 1: return "Sustain (dB)";
            case 2: return "Attack Window (ms)";
            case 3: return "Output Trim (dB)";
            case 4: return "Soft Saturation";
            default: return "Param";
        }
    }

    [[nodiscard]] float parameter_min(uint32_t index) const noexcept override {
        switch (index) {
            case 0: return -15.0f;
            case 1: return -15.0f;
            case 2: return 1.0f;
            case 3: return -15.0f;
            case 4: return 0.0f;
            default: return 0.0f;
        }
    }

    [[nodiscard]] float parameter_max(uint32_t index) const noexcept override {
        switch (index) {
            case 0: return 15.0f;
            case 1: return 15.0f;
            case 2: return 25.0f;
            case 3: return 15.0f;
            case 4: return 1.0f;
            default: return 1.0f;
        }
    }

    [[nodiscard]] float parameter_default(uint32_t index) const noexcept override {
        switch (index) {
            case 0: return 0.0f;
            case 1: return 0.0f;
            case 2: return 8.0f;
            case 3: return 0.0f;
            case 4: return 1.0f;
            default: return 0.0f;
        }
    }

    // Telemetry getters for visualization HUDs
    [[nodiscard]] float last_transient_intensity() const noexcept { return m_last_transient; }
    [[nodiscard]] float last_sustain_intensity() const noexcept { return m_last_sustain; }
    [[nodiscard]] float last_gain_mod() const noexcept { return m_last_gain; }

    void process_stereo(Sample* left, Sample* right, uint32_t frames) noexcept override {
        process_internal(left, right, nullptr, nullptr, frames);
    }

    void process_stereo_sidechain(Sample* left, Sample* right,
                                  const Sample* sc_left, const Sample* sc_right,
                                  uint32_t frames) noexcept override {
        process_internal(left, right, sc_left, sc_right, frames);
    }

private:
    void update_coefficients() noexcept {
        float fs = static_cast<float>(m_sample_rate);
        if (fs <= 0.0f) fs = 48000.0f;

        // Fast envelope time constants: attack 0.8ms, release = attack_window_ms
        constexpr float tau_att_fast_sec = 0.0008f;
        float tau_rel_fast_sec = m_attack_window_ms * 0.001f;

        m_alpha_att_fast = std::exp(-1.0f / (fs * tau_att_fast_sec));
        m_alpha_rel_fast = std::exp(-1.0f / (fs * tau_rel_fast_sec));

        // Slow envelope time constants: attack 20ms, release 180ms
        float tau_att_slow_sec = std::max(0.015f, m_attack_window_ms * 0.0025f);
        constexpr float tau_rel_slow_sec = 0.180f;

        m_alpha_att_slow = std::exp(-1.0f / (fs * tau_att_slow_sec));
        m_alpha_rel_slow = std::exp(-1.0f / (fs * tau_rel_slow_sec));

        // Sub-sample gain smoothing time constant: 0.6ms
        constexpr float tau_smooth_sec = 0.0006f;
        m_alpha_smooth = std::exp(-1.0f / (fs * tau_smooth_sec));

        // Peak-hold samples (3.0 ms bridges zero-crossings down to 150 Hz)
        m_hold_samples = static_cast<uint32_t>(fs * 0.003f);
    }

    void process_internal(Sample* left, Sample* right,
                          const Sample* sc_l, const Sample* sc_r,
                          uint32_t frames) noexcept {
        enable_ftz_daz();

        for (uint32_t i = 0; i < frames; ++i) {
            float in_l = left[i];
            float in_r = right[i];

            // 1. Determine detector input (Sidechain or main stereo peak)
            float peak = (sc_l && sc_r)
                ? std::max(std::abs(sc_l[i]), std::abs(sc_r[i]))
                : std::max(std::abs(in_l), std::abs(in_r));

            // 2. Crest-Bridging Peak-Hold Detector (eliminates zero-crossing ripple on steady tones)
            if (peak >= m_held_peak) {
                m_held_peak = peak;
                m_hold_count = m_hold_samples;
            } else if (m_hold_count > 0) {
                --m_hold_count;
            } else {
                m_held_peak += (1.0f - m_alpha_rel_fast) * (peak - m_held_peak);
            }

            // 3. Fast Envelope Follower
            if (m_held_peak > m_env_fast) {
                m_env_fast += (1.0f - m_alpha_att_fast) * (m_held_peak - m_env_fast);
            } else {
                m_env_fast += (1.0f - m_alpha_rel_fast) * (m_held_peak - m_env_fast);
            }

            // 4. Slow Envelope Follower
            if (m_held_peak > m_env_slow) {
                m_env_slow += (1.0f - m_alpha_att_slow) * (m_held_peak - m_env_slow);
            } else {
                m_env_slow += (1.0f - m_alpha_rel_slow) * (m_held_peak - m_env_slow);
            }

            // 5. Normalized Transient Detection (Differential Envelope)
            float delta = std::max(0.0f, m_env_fast - m_env_slow);
            float trans_ratio = delta / (m_env_fast + 1e-4f);
            trans_ratio = std::clamp(trans_ratio, 0.0f, 1.0f);

            // 5. Sustain Weighting (Suppressed during transient hits, active in tail)
            float sus_weight = std::max(0.0f, 1.0f - 1.8f * trans_ratio);
            // Activity gate: prevents boosting noise floor below -60 dBFS (~0.001)
            float activity = std::clamp((m_env_slow - 0.001f) / 0.008f, 0.0f, 1.0f);
            sus_weight *= activity;

            m_last_transient = trans_ratio;
            m_last_sustain = sus_weight;

            // 6. Compute Modulated Gains
            float g_att = 1.0f;
            if (m_attack_gain_lin >= 1.0f) {
                g_att = 1.0f + (m_attack_gain_lin - 1.0f) * trans_ratio;
            } else {
                g_att = 1.0f - (1.0f - m_attack_gain_lin) * trans_ratio;
            }

            float g_sus = 1.0f;
            if (m_sustain_gain_lin >= 1.0f) {
                g_sus = 1.0f + (m_sustain_gain_lin - 1.0f) * sus_weight;
            } else {
                g_sus = 1.0f - (1.0f - m_sustain_gain_lin) * sus_weight;
            }

            float target_gain = g_att * g_sus * m_output_trim_lin;

            // 7. Sub-sample Gain Smoother (anti-zipper filter)
            m_gain_smooth += (1.0f - m_alpha_smooth) * (target_gain - m_gain_smooth);
            m_last_gain = m_gain_smooth;

            // 8. Apply gain to audio channels
            float out_l = in_l * m_gain_smooth;
            float out_r = in_r * m_gain_smooth;

            // 9. Soft Saturation / Peak Cushioning
            if (m_soft_clip > 0.01f) {
                out_l = apply_soft_clip(out_l, m_soft_clip);
                out_r = apply_soft_clip(out_r, m_soft_clip);
            }

            left[i] = out_l;
            right[i] = out_r;

            // Denormal zeroing
            if (std::abs(m_env_fast) < 1e-15f) m_env_fast = 0.0f;
            if (std::abs(m_env_slow) < 1e-15f) m_env_slow = 0.0f;
            if (std::abs(m_gain_smooth - 1.0f) < 1e-6f && std::abs(target_gain - 1.0f) < 1e-6f) {
                m_gain_smooth = 1.0f;
            }
        }
    }

    [[nodiscard]] static inline float apply_soft_clip(float x, float amount) noexcept {
        constexpr float kCeiling = 0.75f;
        float ax = std::abs(x);
        if (ax <= kCeiling) {
            return x; // 100% bit-exact linear below -2.5 dBFS
        }
        float excess = ax - kCeiling;
        float sat_excess = 0.25f * std::tanh(excess / 0.25f);
        float sat_out = (kCeiling + sat_excess) * (x > 0.0f ? 1.0f : -1.0f);
        return x + amount * (sat_out - x);
    }

    uint32_t m_sample_rate{48000};

    // User Parameters
    float m_attack_gain_db{0.0f};
    float m_attack_gain_lin{1.0f};
    float m_sustain_gain_db{0.0f};
    float m_sustain_gain_lin{1.0f};
    float m_attack_window_ms{8.0f};
    float m_output_trim_db{0.0f};
    float m_output_trim_lin{1.0f};
    float m_soft_clip{1.0f};

    // Filter Coefficients
    float m_alpha_att_fast{0.0f};
    float m_alpha_rel_fast{0.0f};
    float m_alpha_att_slow{0.0f};
    float m_alpha_rel_slow{0.0f};
    float m_alpha_smooth{0.0f};
    uint32_t m_hold_samples{144};

    // Real-Time States
    float m_env_fast{0.0f};
    float m_env_slow{0.0f};
    float m_gain_smooth{1.0f};
    float m_held_peak{0.0f};
    uint32_t m_hold_count{0};

    // Telemetry
    float m_last_transient{0.0f};
    float m_last_sustain{0.0f};
    float m_last_gain{1.0f};
};

} // namespace audio_core::dsp
