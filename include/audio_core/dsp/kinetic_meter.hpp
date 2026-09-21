#pragma once

#include "audio_core/types.hpp"
#include "audio_core/protocol/telemetry_packet.hpp"

#include <cmath>
#include <array>
#include <algorithm>
#include <atomic>
#include <cstdint>

namespace audio_core::dsp {

// ============================================================================
// KineticMeter: Airwindows-Inspired Continuous ODE & Phase-Space Hit Meter
// - Authority (Green): Zero-crossing inertia & low-end fundamental power
// - Power (Blue): Mid-band sonority resonance & peak distribution cloud
// - Detail (Red): Differential slew velocity (dx/dt) & transient energy
// - Phase-Space Attractor: Lock-free (x, dx/dt) ring buffer for 2D orbit plotting
// - Real-time Diagnostic: Automatic Hit-Profile & Balance classifier
// ============================================================================
class KineticMeter {
public:
    static constexpr size_t kPhaseHistory = protocol::KineticTelemetryData::kPhasePoints;

    explicit KineticMeter(uint32_t sample_rate = 48000) noexcept
        : m_sample_rate(sample_rate > 0 ? sample_rate : 48000) {
        reset();
    }

    void set_sample_rate(uint32_t sample_rate) noexcept {
        if (sample_rate > 0) {
            m_sample_rate = sample_rate;
            reset();
        }
    }

    void reset() noexcept {
        m_authority.store(0.0f, std::memory_order_relaxed);
        m_power.store(0.0f, std::memory_order_relaxed);
        m_detail.store(0.0f, std::memory_order_relaxed);
        m_crest_factor_db.store(0.0f, std::memory_order_relaxed);
        m_diagnostic_id.store(0, std::memory_order_relaxed);

        m_last_sample = 0.0f;
        m_prev_zc_sample = 0.0f;
        m_lp_state = 0.0f;
        m_bp_state = 0.0f;
        m_zc_samples_counter = 0;
        m_zc_interval_avg = 100.0f;
        m_phase_write_idx = 0;

        for (size_t i = 0; i < kPhaseHistory; ++i) {
            m_phase_x[i] = 0.0f;
            m_phase_y[i] = 0.0f;
        }
    }

    // Real-time block process: zero allocations, called in RT audio thread
    void process_block(const float* left, const float* right, uint32_t frames) noexcept {
        if (!left || !right || frames == 0) return;

        // Exponential release factor (~150ms ballistics decay)
        const float dt_block = static_cast<float>(frames) / static_cast<float>(m_sample_rate);
        const float decay = std::exp(-dt_block / 0.150f);

        // Filter coefficients for low-end (160 Hz) and mid-range (1 kHz)
        const float lp_coeff = std::clamp(2.0f * 3.14159265f * 160.0f / static_cast<float>(m_sample_rate), 0.001f, 0.5f);
        const float bp_coeff = std::clamp(2.0f * 3.14159265f * 1200.0f / static_cast<float>(m_sample_rate), 0.001f, 0.5f);

        float peak_sq = 0.0f;
        float sum_sq = 0.0f;
        float sum_slew_sq = 0.0f;
        float sum_lp_sq = 0.0f;
        float sum_bp_sq = 0.0f;

        // Downsampling step for phase space buffer so it spans the audio block
        const uint32_t phase_step = std::max<uint32_t>(1, frames / static_cast<uint32_t>(kPhaseHistory));

        for (uint32_t i = 0; i < frames; ++i) {
            const float s_l = left[i];
            const float s_r = right[i];
            const float mono = 0.5f * (s_l + s_r);
            const float mono_sq = mono * mono;

            if (mono_sq > peak_sq) peak_sq = mono_sq;
            sum_sq += mono_sq;

            // 1. Slew rate calculation: dx/dt
            const float raw_slew = mono - m_last_sample;
            m_last_sample = mono;
            const float slew_normalized = raw_slew * 3.5f; // Scaled for phase space visibility
            sum_slew_sq += raw_slew * raw_slew;

            // 2. Zero-crossing detection & low-end period
            m_zc_samples_counter++;
            if ((mono >= 0.0f && m_prev_zc_sample < 0.0f) || (mono < 0.0f && m_prev_zc_sample >= 0.0f)) {
                // Update running average of zero-crossing distance
                m_zc_interval_avg = 0.9f * m_zc_interval_avg + 0.1f * static_cast<float>(m_zc_samples_counter);
                m_zc_samples_counter = 0;
            }
            m_prev_zc_sample = mono;

            // 3. Simple one-pole band isolation for tone coloring
            m_lp_state += lp_coeff * (mono - m_lp_state);
            sum_lp_sq += m_lp_state * m_lp_state;

            m_bp_state += bp_coeff * (mono - m_bp_state);
            const float bp_sample = mono - m_lp_state; // Mid/presence relative to deep sub
            sum_bp_sq += bp_sample * bp_sample;

            // 4. Capture phase space point (x, dx/dt)
            if (i % phase_step == 0) {
                const size_t idx = m_phase_write_idx;
                m_phase_x[idx] = std::clamp(mono, -1.0f, 1.0f);
                m_phase_y[idx] = std::clamp(slew_normalized, -1.0f, 1.0f);
                m_phase_write_idx = (idx + 1) % kPhaseHistory;
            }
        }

        const float inv_frames = 1.0f / static_cast<float>(frames);
        const float rms = std::sqrt(sum_sq * inv_frames);
        const float peak = std::sqrt(peak_sq);
        const float lp_rms = std::sqrt(sum_lp_sq * inv_frames);
        const float bp_rms = std::sqrt(sum_bp_sq * inv_frames);
        const float slew_rms = std::sqrt(sum_slew_sq * inv_frames) * 12.0f; // Scaled to [0..1] range

        // Calculate Crest Factor in dB
        float crest_db = 0.0f;
        if (rms > 0.0001f && peak > 0.0001f) {
            crest_db = 20.0f * std::log10(std::max(1.0f, peak / rms));
        }

        // ====================================================================
        // AIRWINDOWS TRI-METRIC EXTRACTION
        // ====================================================================
        // Authority (Green): Bass energy weighted by zero-crossing wavelength (low frequency inertia)
        const float zc_bass_factor = std::clamp(m_zc_interval_avg / 160.0f, 0.2f, 2.0f);
        const float raw_authority = std::clamp(lp_rms * 1.8f * zc_bass_factor, 0.0f, 1.25f);

        // Power (Blue): Mid-band resonance / sonority density
        const float raw_power = std::clamp(bp_rms * 1.6f, 0.0f, 1.25f);

        // Detail (Red): High-frequency slew rate & edge acceleration
        const float raw_detail = std::clamp(slew_rms * 2.2f, 0.0f, 1.25f);

        // Ballistics update: Instantaneous attack, smooth decay
        float cur_auth = m_authority.load(std::memory_order_relaxed);
        float cur_pow  = m_power.load(std::memory_order_relaxed);
        float cur_det  = m_detail.load(std::memory_order_relaxed);

        cur_auth = std::max(raw_authority, cur_auth * decay);
        cur_pow  = std::max(raw_power, cur_pow * decay);
        cur_det  = std::max(raw_detail, cur_det * decay);

        m_authority.store(cur_auth, std::memory_order_relaxed);
        m_power.store(cur_pow, std::memory_order_relaxed);
        m_detail.store(cur_det, std::memory_order_relaxed);
        m_crest_factor_db.store(crest_db, std::memory_order_relaxed);

        // ====================================================================
        // AIRWINDOWS HIT RECORD DIAGNOSTIC CLASSIFICATION
        // ====================================================================
        uint32_t diag = 0;
        if (cur_pow < 0.01f && cur_auth < 0.01f) {
            diag = 0; // Idle / Silence
        } else if (peak >= 0.995f && crest_db < 7.5f) {
            diag = 5; // Brickwall Overcompressed / Collapsed Orbit
        } else if (cur_det > 0.60f && cur_det > cur_pow * 1.45f) {
            diag = 2; // Excess Slew / Harsh Treble (Reduce Red)
        } else if (cur_auth < 0.15f && cur_pow > 0.25f) {
            diag = 3; // Lacks Authority / Thin Lows (Boost Green)
        } else if (cur_auth > cur_pow * 1.75f && cur_det < 0.25f) {
            diag = 4; // Allow Fullness / Over-dominant Sub Mud
        } else if (cur_pow >= 0.20f && std::abs(cur_pow - cur_auth) < 0.35f) {
            diag = 1; // Balanced Hit Sonority (Optimal Blue Cloud)
        } else {
            diag = 1; // General Musical Balance
        }
        m_diagnostic_id.store(diag, std::memory_order_relaxed);
    }

    // Lock-free telemetry query: populates UI snapshot frame without locking
    void capture_telemetry(protocol::KineticTelemetryData& out) const noexcept {
        out.authority = m_authority.load(std::memory_order_relaxed);
        out.power = m_power.load(std::memory_order_relaxed);
        out.detail = m_detail.load(std::memory_order_relaxed);
        out.crest_factor_db = m_crest_factor_db.load(std::memory_order_relaxed);
        out.diagnostic_id = m_diagnostic_id.load(std::memory_order_relaxed);

        for (size_t i = 0; i < kPhaseHistory; ++i) {
            out.phase_x[i] = m_phase_x[i];
            out.phase_y[i] = m_phase_y[i];
        }
    }

private:
    uint32_t m_sample_rate{48000};

    // Filter and transient states
    float m_last_sample{0.0f};
    float m_prev_zc_sample{0.0f};
    float m_lp_state{0.0f};
    float m_bp_state{0.0f};
    uint32_t m_zc_samples_counter{0};
    float m_zc_interval_avg{100.0f};

    // Phase history ring buffer
    size_t m_phase_write_idx{0};
    std::array<float, kPhaseHistory> m_phase_x{};
    std::array<float, kPhaseHistory> m_phase_y{};

    // Atomic telemetry metrics
    std::atomic<float> m_authority{0.0f};
    std::atomic<float> m_power{0.0f};
    std::atomic<float> m_detail{0.0f};
    std::atomic<float> m_crest_factor_db{0.0f};
    std::atomic<uint32_t> m_diagnostic_id{0};
};

} // namespace audio_core::dsp
