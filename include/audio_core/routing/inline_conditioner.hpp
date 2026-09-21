#pragma once

#include "audio_core/types.hpp"
#include <cmath>
#include <numbers>
#include <algorithm>
#include <cstdint>
#include <array>

namespace audio_core::routing {

// ============================================================================
// Filter and Rectifier Modes for Inline Signal Conditioning
// ============================================================================
enum class ConditionerFilterMode : uint8_t {
    Bypass = 0,
    Lowpass,
    Highpass,
    Bandpass,
    Notch
};

enum class ConditionerRectifyMode : uint8_t {
    None = 0,           // Raw bipolar signal (-1.0 to +1.0)
    HalfWave,          // Positive half-wave only (max(0, x))
    FullWave,          // Absolute value (|x|)
    SmoothEnvelope     // 1st-order leaky peak/RMS detector envelope follower
};

// ============================================================================
// InlineConditionerConfig: Configuration for inline signal conditioning
// Zero-converter node paradigm: Every routing edge contains its own conditioner.
// ============================================================================
struct InlineConditionerConfig {
    ConditionerFilterMode filter_mode{ConditionerFilterMode::Bypass};
    float cutoff_hz{120.0f};          // Filter cutoff frequency (e.g. 120Hz for sub kick sidechain)
    float q{0.7071f};                 // Butterworth Q (0.1 to 20.0)
    float gain{1.0f};                 // Linear gain scaling (-10.0 to +10.0)
    bool invert_phase{false};         // 180-degree polarity inversion
    ConditionerRectifyMode rectify{ConditionerRectifyMode::None};
    float envelope_attack_ms{5.0f};   // Envelope follower attack time (ms)
    float envelope_release_ms{50.0f}; // Envelope follower release time (ms)
};

// ============================================================================
// InlineConditioner: Real-Time High-Performance Signal Conditioner
// Features:
// 1. Trapezoidal State-Variable Filter (Cytomic SVF):
//    - Unconditionally stable, zero delay feedback across 10Hz to Nyquist.
//    - Simultaneous LP, HP, BP outputs without coefficient recalculation explosions.
// 2. Rectification & Dynamic Envelope Follower:
//    - Turns raw audio into unipolar CV / compression squeeze envelope instantaneously.
// 3. Polarity Inversion & Vectorized Gain Scaling:
//    - Real-time lock-free parameter agility with 0 allocations.
// ============================================================================
class InlineConditioner {
public:
    InlineConditioner() noexcept
        : InlineConditioner(48000) {}

    explicit InlineConditioner(uint32_t sample_rate) noexcept
        : m_sample_rate(sample_rate ? sample_rate : 48000) {
        update_coefficients();
        reset();
    }

    void set_sample_rate(uint32_t sr) noexcept {
        if (sr == 0 || sr == m_sample_rate) return;
        m_sample_rate = sr;
        update_coefficients();
    }

    void set_config(const InlineConditionerConfig& cfg) noexcept {
        m_config = cfg;
        update_coefficients();
    }

    [[nodiscard]] const InlineConditionerConfig& config() const noexcept {
        return m_config;
    }

    void reset() noexcept {
        m_svf_s1 = 0.0f;
        m_svf_s2 = 0.0f;
        m_env_state = 0.0f;
    }

    // Process a single sample through the inline conditioning chain
    [[nodiscard]] inline float process_sample(float input) noexcept {
        float x = input;

        // 1. Cytomic SVF Filter stage
        if (m_config.filter_mode != ConditionerFilterMode::Bypass) {
            // Trapezoidal SVF state equations (Cytomic)
            const float v0 = x;
            const float v1 = (m_svf_a1 * m_svf_s1) + (m_svf_a2 * (v0 - m_svf_s2));
            const float v2 = m_svf_s2 + (m_svf_a2 * m_svf_s1) + (m_svf_a3 * (v0 - m_svf_s2));

            m_svf_s1 = (2.0f * v1) - m_svf_s1;
            m_svf_s2 = (2.0f * v2) - m_svf_s2;

            switch (m_config.filter_mode) {
                case ConditionerFilterMode::Lowpass:
                    x = v2;
                    break;
                case ConditionerFilterMode::Highpass:
                    x = v0 - (m_svf_k * v1) - v2;
                    break;
                case ConditionerFilterMode::Bandpass:
                    x = v1;
                    break;
                case ConditionerFilterMode::Notch:
                    x = v0 - (m_svf_k * v1);
                    break;
                case ConditionerFilterMode::Bypass:
                default:
                    break;
            }
        }

        // 2. Rectification / Envelope stage
        switch (m_config.rectify) {
            case ConditionerRectifyMode::HalfWave:
                x = std::max(0.0f, x);
                break;
            case ConditionerRectifyMode::FullWave:
                x = std::abs(x);
                break;
            case ConditionerRectifyMode::SmoothEnvelope: {
                const float abs_x = std::abs(x);
                const float coeff = (abs_x > m_env_state) ? m_env_attack_coeff : m_env_release_coeff;
                m_env_state += coeff * (abs_x - m_env_state);
                x = m_env_state;
                break;
            }
            case ConditionerRectifyMode::None:
            default:
                break;
        }

        // 3. Polarity inversion & Gain Scaling
        if (m_config.invert_phase) {
            x = -x;
        }

        return x * m_config.gain;
    }

    // Process a block from src into dst (in-place src == dst is safe)
    void process_block(const float* src, float* dst, uint32_t frames) noexcept {
        if (!src || !dst || frames == 0) return;

        #if defined(__GNUC__) || defined(__clang__)
        #pragma GCC ivdep
        #endif
        for (uint32_t i = 0; i < frames; ++i) {
            dst[i] = process_sample(src[i]);
        }
    }

    // Process and ACCUMULATE (sum) into dst: dst[i] += conditioned(src[i])
    // Enables multi-source grouping/summing (e.g. Track 2 Lowpass + Dante Ch 4 grouped into sidechain)
    void process_and_accumulate(const float* src, float* dst, uint32_t frames) noexcept {
        if (!src || !dst || frames == 0) return;

        #if defined(__GNUC__) || defined(__clang__)
        #pragma GCC ivdep
        #endif
        for (uint32_t i = 0; i < frames; ++i) {
            dst[i] += process_sample(src[i]);
        }
    }

private:
    void update_coefficients() noexcept {
        // Clamp parameters for stability
        const float nyquist = static_cast<float>(m_sample_rate) * 0.499f;
        const float fc = std::clamp(m_config.cutoff_hz, 10.0f, nyquist);
        const float q = std::clamp(m_config.q, 0.1f, 30.0f);

        // Pre-warped frequency for bilinear transform
        const float w = std::tan(std::numbers::pi_v<float> * fc / static_cast<float>(m_sample_rate));
        m_svf_k = 1.0f / q;
        m_svf_a1 = 1.0f / (1.0f + (w * (w + m_svf_k)));
        m_svf_a2 = w * m_svf_a1;
        m_svf_a3 = w * m_svf_a2;

        // Envelope ballistics
        const float att_s = std::max(0.0001f, m_config.envelope_attack_ms * 0.001f);
        const float rel_s = std::max(0.0001f, m_config.envelope_release_ms * 0.001f);
        m_env_attack_coeff = 1.0f - std::exp(-1.0f / (att_s * static_cast<float>(m_sample_rate)));
        m_env_release_coeff = 1.0f - std::exp(-1.0f / (rel_s * static_cast<float>(m_sample_rate)));
    }

    uint32_t m_sample_rate{48000};
    InlineConditionerConfig m_config;

    // Cytomic SVF State & Coefficients
    float m_svf_s1{0.0f};
    float m_svf_s2{0.0f};
    float m_svf_k{1.4142f};
    float m_svf_a1{0.0f};
    float m_svf_a2{0.0f};
    float m_svf_a3{0.0f};

    // Envelope Follower State & Coefficients
    float m_env_state{0.0f};
    float m_env_attack_coeff{0.1f};
    float m_env_release_coeff{0.01f};
};

} // namespace audio_core::routing
