#pragma once

#include "audio_core/types.hpp"
#include "audio_core/insert_slot.hpp"
#include <cmath>
#include <numbers>
#include <algorithm>
#include <cstdint>
#include <atomic>

namespace audio_core::dsp {

// ============================================================================
// VactrolMode: Operational Topology of the Optical System
// ============================================================================
enum class VactrolMode : uint32_t {
    OptoCompressor = 0, // Classic Teletronix LA-2A smooth optical leveling (ratio ~3:1 - 4:1)
    OptoLimiter = 1,    // Steeper knee, higher ratio (~10:1 - 20:1) optical peak limiting
    BuchlaLPG = 2       // Buchla 292 Low-Pass Gate: simultaneous dynamic VCA + 2-pole lowpass filter ringing
};

// ============================================================================
// LiquidVactrolCell: Physical Optocoupler & Semiconductor Trap Memory Model
// ============================================================================
// Emulates a Cadmium Sulfide (CdS) / Cadmium Selenide (CdSe) photoresistor
// illuminated by an electroluminescent (EL) panel or LED (e.g. T4B or VTL5C3).
//
// Continuous-Time Physics:
// 1. Photo-Conductance C(t):
//      dC/dt = (1/tau_eff) * ( L(t) - C(t) )
// 2. Deep Electron Trap State Q(t) ("Photocell Dark Memory"):
//      dQ/dt = (1/tau_trap) * ( tanh(1.8 * C(t)) - Q(t) )
// 3. Dynamic Liquid Tau tau_eff(L, C, Q, dE/dt):
//      - Attack (L > C): Rapid electron excitation:
//          tau_eff = tau_attack / (1 + 0.5 * (L - C))
//      - Release (L <= C): Two-stage carrier recombination governed by trap charge:
//          tau_eff = tau_rel_fast + (tau_rel_slow - tau_rel_fast) * memory_depth * (Q^2 / (Q^2 + 0.09))
//        with inertial braking on rapid drops.
//
// Discretization:
//   A-Stable Bilinear Transform / Trapezoidal Integration on both C(t) and Q(t).
//   Guarantees 100% unconditional stability, no NaN/Inf, and pole |p| < 1.0.
// ============================================================================
class LiquidVactrolCell {
public:
    explicit LiquidVactrolCell(float sample_rate = 48000.0f) noexcept
        : m_sample_rate(sample_rate > 0.0f ? sample_rate : 48000.0f) {
        reset();
    }

    void reset() noexcept {
        m_conductance = 0.0f;
        m_trap_charge = 0.0f;
        m_prev_lum = 0.0f;
        m_prev_conductance = 0.0f;
        m_prev_target_q = 0.0f;
        m_instant_tau_ms = m_release_fast_ms;
    }

    void set_sample_rate(float sr) noexcept {
        if (sr > 0.0f) {
            m_sample_rate = sr;
        }
    }

    void set_attack_ms(float ms) noexcept {
        m_attack_ms = std::clamp(ms, 0.1f, 50.0f);
    }

    void set_release_fast_ms(float ms) noexcept {
        m_release_fast_ms = std::clamp(ms, 5.0f, 200.0f);
    }

    void set_release_slow_ms(float ms) noexcept {
        m_release_slow_ms = std::clamp(ms, 200.0f, 5000.0f);
    }

    void set_memory_depth(float depth) noexcept {
        m_memory_depth = std::clamp(depth, 0.0f, 1.0f);
    }

    [[nodiscard]] float conductance() const noexcept { return m_conductance; }
    [[nodiscard]] float trap_charge() const noexcept { return m_trap_charge; }
    [[nodiscard]] float instant_tau_ms() const noexcept { return m_instant_tau_ms; }

    // Step cell with target photon luminescence L(t) >= 0
    inline float step(float lum_in) noexcept {
        const float lum = std::max(0.0f, lum_in);

        // Calculate dynamic liquid tau
        float tau_sec = 0.0f;
        if (lum > m_conductance) {
            // Attack phase: rapid light-induced ionization
            const float excess = lum - m_conductance;
            tau_sec = (m_attack_ms * 0.001f) / (1.0f + 0.5f * excess);
            tau_sec = std::max(1e-5f, tau_sec);
        } else {
            // Release phase: two-stage recombination governed by deep trap state Q
            const float q2 = m_trap_charge * m_trap_charge;
            const float q_factor = q2 / (q2 + 0.09f); // Saturation curve
            const float base_rel_ms = m_release_fast_ms + (m_release_slow_ms - m_release_fast_ms) * m_memory_depth * q_factor;

            // Inertial braking on rapid drop
            const float vel = std::max(0.0f, m_prev_conductance - lum);
            tau_sec = (base_rel_ms * 0.001f) * (1.0f + 0.25f * std::tanh(vel * 4.0f));
            tau_sec = std::max(1e-4f, tau_sec);
        }

        m_instant_tau_ms = tau_sec * 1000.0f;

        // Bilinear Trapezoidal Integration for Conductance C(t):
        // alpha = dt / (2 * tau)
        const float dt = 1.0f / m_sample_rate;
        const float alpha = dt / (2.0f * tau_sec);
        const float denom = 1.0f + alpha;
        const float a1 = (1.0f - alpha) / denom;
        const float b0 = alpha / denom;

        float new_c = (a1 * m_conductance) + (b0 * (lum + m_prev_lum));
        if (new_c < 1e-15f) new_c = 0.0f;

        m_prev_lum = lum;
        m_prev_conductance = m_conductance;
        m_conductance = new_c;

        // Bilinear Trapezoidal Integration for Electron Trap Charge Q(t):
        // Deep crystal traps are actively populated by photon emission L(t);
        // in darkness (lum = 0), traps thermalize/discharge slowly without new excitation.
        const float target_q = std::tanh(lum * 1.8f);
        const float trap_tau = (target_q > m_trap_charge)
            ? (m_trap_charge_ms * 0.001f)
            : (m_trap_discharge_ms * 0.001f);
        const float q_alpha = dt / (2.0f * std::max(1e-3f, trap_tau));
        const float q_denom = 1.0f + q_alpha;
        const float q_a1 = (1.0f - q_alpha) / q_denom;
        const float q_b0 = q_alpha / q_denom;

        float new_q = (q_a1 * m_trap_charge) + (q_b0 * (target_q + m_prev_target_q));
        if (new_q < 1e-15f) new_q = 0.0f;

        m_prev_target_q = target_q;
        m_trap_charge = new_q;

        return m_conductance;
    }

private:
    float m_sample_rate{48000.0f};
    float m_attack_ms{2.5f};           // Fast optical attack (~2.5ms)
    float m_release_fast_ms{60.0f};    // First-stage fast drop (~60ms)
    float m_release_slow_ms{1800.0f};  // Second-stage deep dark memory tail (~1800ms)
    float m_memory_depth{0.75f};       // User control: 0.0 = fast punchy, 1.0 = deep vintage memory
    float m_trap_charge_ms{350.0f};    // Rate of filling deep traps
    float m_trap_discharge_ms{2200.0f};// Rate of dark thermalization

    float m_conductance{0.0f};
    float m_trap_charge{0.0f};
    float m_prev_lum{0.0f};
    float m_prev_conductance{0.0f};
    float m_prev_target_q{0.0f};
    float m_instant_tau_ms{60.0f};
};

// ============================================================================
// Cytomic SVF State for Buchla LPG Lowpass Ringing & Sidechain HF Filter
// ============================================================================
struct VactrolSvf {
    float s1{0.0f};
    float s2{0.0f};

    void reset() noexcept {
        s1 = 0.0f;
        s2 = 0.0f;
    }

    inline float process_lp(float v0, float fc, float q, float sr) noexcept {
        const float nyquist = sr * 0.49f;
        const float f_clamped = std::clamp(fc, 10.0f, nyquist);
        const float q_clamped = std::clamp(q, 0.5f, 10.0f);
        const float w = std::tan(std::numbers::pi_v<float> * f_clamped / sr);
        const float k = 1.0f / q_clamped;
        const float a1 = 1.0f / (1.0f + w * (w + k));
        const float a2 = w * a1;
        const float a3 = w * a2;

        const float v1 = (a1 * s1) + (a2 * (v0 - s2));
        const float v2 = s2 + (a2 * s1) + (a3 * (v0 - s2));

        s1 = (2.0f * v1) - s1;
        s2 = (2.0f * v2) - s2;

        if (std::abs(s1) < 1e-15f) s1 = 0.0f;
        if (std::abs(s2) < 1e-15f) s2 = 0.0f;

        return v2;
    }
};

// ============================================================================
// LiquidVactrol: Core Stereo Optical Leveler & Low-Pass Gate Engine
// ============================================================================
class LiquidVactrol {
public:
    explicit LiquidVactrol(uint32_t sample_rate = 48000) noexcept
        : m_sample_rate(sample_rate ? sample_rate : 48000),
          m_cell_l(static_cast<float>(m_sample_rate)),
          m_cell_r(static_cast<float>(m_sample_rate)) {
        reset();
    }

    void set_sample_rate(uint32_t sr) noexcept {
        if (sr == 0 || sr == m_sample_rate) return;
        m_sample_rate = sr;
        const float srf = static_cast<float>(m_sample_rate);
        m_cell_l.set_sample_rate(srf);
        m_cell_r.set_sample_rate(srf);
    }
    [[nodiscard]] uint32_t sample_rate() const noexcept { return m_sample_rate; }

    void reset() noexcept {
        m_cell_l.reset();
        m_cell_r.reset();
        m_sc_filter_l.reset();
        m_sc_filter_r.reset();
        m_lpg_filter_l.reset();
        m_lpg_filter_r.reset();
        m_gr_db_l = 0.0f;
        m_gr_db_r = 0.0f;
    }

    // Configuration
    void set_peak_reduction(float val) noexcept {
        m_peak_reduction = std::clamp(val, 0.0f, 1.0f);
    }
    [[nodiscard]] float peak_reduction() const noexcept { return m_peak_reduction; }

    void set_makeup_gain_db(float db) noexcept {
        m_makeup_gain_db = std::clamp(db, -12.0f, 24.0f);
        m_makeup_linear = std::pow(10.0f, m_makeup_gain_db / 20.0f);
    }
    [[nodiscard]] float makeup_gain_db() const noexcept { return m_makeup_gain_db; }

    void set_mode(VactrolMode mode) noexcept {
        m_mode = mode;
        if (m_mode == VactrolMode::BuchlaLPG) {
            // Buchla LPG tuning: fast percussive strike attack
            m_cell_l.set_attack_ms(1.2f);
            m_cell_r.set_attack_ms(1.2f);
            m_cell_l.set_release_fast_ms(80.0f);
            m_cell_r.set_release_fast_ms(80.0f);
        } else {
            // LA-2A T4B tuning
            m_cell_l.set_attack_ms(2.5f);
            m_cell_r.set_attack_ms(2.5f);
            m_cell_l.set_release_fast_ms(60.0f);
            m_cell_r.set_release_fast_ms(60.0f);
        }
    }
    [[nodiscard]] VactrolMode mode() const noexcept { return m_mode; }

    void set_memory_depth(float depth) noexcept {
        m_memory_depth = std::clamp(depth, 0.0f, 1.0f);
        m_cell_l.set_memory_depth(m_memory_depth);
        m_cell_r.set_memory_depth(m_memory_depth);
    }
    [[nodiscard]] float memory_depth() const noexcept { return m_memory_depth; }

    void set_hf_emphasis(float val) noexcept {
        m_hf_emphasis = std::clamp(val, 0.0f, 1.0f);
    }
    [[nodiscard]] float hf_emphasis() const noexcept { return m_hf_emphasis; }

    void set_mix(float mix) noexcept {
        m_mix = std::clamp(mix, 0.0f, 1.0f);
    }
    [[nodiscard]] float mix() const noexcept { return m_mix; }

    void set_lpg_resonance(float res) noexcept {
        m_lpg_resonance = std::clamp(res, 0.0f, 0.95f);
    }
    [[nodiscard]] float lpg_resonance() const noexcept { return m_lpg_resonance; }

    void set_stereo_link(float link) noexcept {
        m_stereo_link = std::clamp(link, 0.0f, 1.0f);
    }
    [[nodiscard]] float stereo_link() const noexcept { return m_stereo_link; }

    // Telemetry getters
    [[nodiscard]] float gain_reduction_db_l() const noexcept { return m_gr_db_l; }
    [[nodiscard]] float gain_reduction_db_r() const noexcept { return m_gr_db_r; }
    [[nodiscard]] float conductance_l() const noexcept { return m_cell_l.conductance(); }
    [[nodiscard]] float conductance_r() const noexcept { return m_cell_r.conductance(); }
    [[nodiscard]] float trap_charge_l() const noexcept { return m_cell_l.trap_charge(); }
    [[nodiscard]] float trap_charge_r() const noexcept { return m_cell_r.trap_charge(); }
    [[nodiscard]] float instant_tau_ms_l() const noexcept { return m_cell_l.instant_tau_ms(); }
    [[nodiscard]] float instant_tau_ms_r() const noexcept { return m_cell_r.instant_tau_ms(); }

    // Process stereo audio block
    void process_stereo(Sample* left, Sample* right, uint32_t frames,
                        const Sample* sc_left = nullptr, const Sample* sc_right = nullptr) noexcept {
        if (!left || !right || frames == 0) return;

        const float srf = static_cast<float>(m_sample_rate);
        const float max_gr_db = (m_mode == VactrolMode::OptoLimiter) ? 36.0f : 26.0f;
        const float q_lpg = 0.707f + (m_lpg_resonance * 4.0f);

        for (uint32_t i = 0; i < frames; ++i) {
            const float in_l = left[i];
            const float in_r = right[i];

            // 1. Determine sidechain signal (external or internal audio)
            const float raw_sc_l = sc_left ? sc_left[i] : in_l;
            const float raw_sc_r = sc_right ? sc_right[i] : in_r;

            // 2. High-Frequency Sidechain Conditioning (LA-2A R37 HF Trim)
            // 1200 Hz high-shelf / highpass boost
            float det_l = raw_sc_l;
            float det_r = raw_sc_r;
            if (m_hf_emphasis > 1e-4f) {
                const float lp_l = m_sc_filter_l.process_lp(raw_sc_l, 1200.0f, 0.707f, srf);
                const float lp_r = m_sc_filter_r.process_lp(raw_sc_r, 1200.0f, 0.707f, srf);
                const float hp_l = raw_sc_l - lp_l;
                const float hp_r = raw_sc_r - lp_r;
                det_l = raw_sc_l + (m_hf_emphasis * 2.5f) * hp_l;
                det_r = raw_sc_r + (m_hf_emphasis * 2.5f) * hp_r;
            }

            // 3. Compute Electroluminescent / LED Light Emission L(t)
            float drive_l = std::abs(det_l) * (0.2f + 5.0f * m_peak_reduction);
            float drive_r = std::abs(det_r) * (0.2f + 5.0f * m_peak_reduction);

            float lum_l = 0.0f;
            float lum_r = 0.0f;

            switch (m_mode) {
                case VactrolMode::OptoCompressor: {
                    // Soft knee threshold around 0.12f
                    const float ex_l = std::max(0.0f, drive_l - 0.12f);
                    const float ex_r = std::max(0.0f, drive_r - 0.12f);
                    lum_l = (ex_l > 0.0f) ? (std::tanh(ex_l / 0.35f) * (1.0f + 0.35f * ex_l)) : 0.0f;
                    lum_r = (ex_r > 0.0f) ? (std::tanh(ex_r / 0.35f) * (1.0f + 0.35f * ex_r)) : 0.0f;
                    break;
                }
                case VactrolMode::OptoLimiter: {
                    // Sharp knee threshold around 0.18f, steep slope
                    const float ex_l = std::max(0.0f, drive_l - 0.18f);
                    const float ex_r = std::max(0.0f, drive_r - 0.18f);
                    lum_l = (ex_l > 0.0f) ? (std::tanh(ex_l / 0.15f) * (1.0f + 0.8f * ex_l)) : 0.0f;
                    lum_r = (ex_r > 0.0f) ? (std::tanh(ex_r / 0.15f) * (1.0f + 0.8f * ex_r)) : 0.0f;
                    break;
                }
                case VactrolMode::BuchlaLPG: {
                    // Direct excitation strike: transient drives LED wide open
                    lum_l = std::min(2.5f, drive_l * 1.6f);
                    lum_r = std::min(2.5f, drive_r * 1.6f);
                    break;
                }
            }

            // 4. Stereo Coupling
            if (m_stereo_link > 1e-4f) {
                const float max_lum = std::max(lum_l, lum_r);
                lum_l = lum_l + m_stereo_link * (max_lum - lum_l);
                lum_r = lum_r + m_stereo_link * (max_lum - lum_r);
            }

            // 5. Step Optocoupler CdS Cells (Bilinear Trapezoidal Integration)
            const float cond_l = m_cell_l.step(lum_l);
            const float cond_r = m_cell_r.step(lum_r);

            // 6. Apply Gain / Filter Processing
            float wet_l = in_l;
            float wet_r = in_r;

            if (m_mode == VactrolMode::BuchlaLPG) {
                // Buchla 292 Low-Pass Gate: simultaneous dynamic VCA + lowpass filter cutoff sweep
                const float open_l = std::clamp(std::pow(cond_l, 1.25f), 0.0f, 1.0f);
                const float open_r = std::clamp(std::pow(cond_r, 1.25f), 0.0f, 1.0f);

                const float fc_min = 25.0f;
                const float fc_max = srf * 0.42f;
                const float fc_l = fc_min + (fc_max - fc_min) * open_l;
                const float fc_r = fc_min + (fc_max - fc_min) * open_r;

                const float filtered_l = m_lpg_filter_l.process_lp(in_l, fc_l, q_lpg, srf);
                const float filtered_r = m_lpg_filter_r.process_lp(in_r, fc_r, q_lpg, srf);

                // LPG amplitude envelope
                wet_l = filtered_l * open_l * m_makeup_linear;
                wet_r = filtered_r * open_r * m_makeup_linear;

                m_gr_db_l = (open_l > 1e-4f) ? (20.0f * std::log10(open_l)) : -80.0f;
                m_gr_db_r = (open_r > 1e-4f) ? (20.0f * std::log10(open_r)) : -80.0f;
            } else {
                // Opto Compressor / Limiter Mode
                m_gr_db_l = -max_gr_db * std::tanh(cond_l * 1.2f);
                m_gr_db_r = -max_gr_db * std::tanh(cond_r * 1.2f);

                const float gain_l = std::pow(10.0f, m_gr_db_l / 20.0f);
                const float gain_r = std::pow(10.0f, m_gr_db_r / 20.0f);

                float out_l = in_l * gain_l * m_makeup_linear;
                float out_r = in_r * gain_r * m_makeup_linear;

                // Subtle vintage tube saturation stage (12AX7 cathode follower emulation)
                // Adds gentle 2nd-order asymmetric warmth when driven hard
                out_l = out_l - (0.035f * out_l * out_l);
                out_r = out_r - (0.035f * out_r * out_r);
                wet_l = std::tanh(out_l);
                wet_r = std::tanh(out_r);
            }

            // 7. Parallel Wet/Dry Mix
            left[i] = in_l + m_mix * (wet_l - in_l);
            right[i] = in_r + m_mix * (wet_r - in_r);
        }
    }

private:
    uint32_t m_sample_rate{48000};
    VactrolMode m_mode{VactrolMode::OptoCompressor};
    float m_peak_reduction{0.5f};
    float m_makeup_gain_db{0.0f};
    float m_makeup_linear{1.0f};
    float m_memory_depth{0.75f};
    float m_hf_emphasis{0.0f};
    float m_mix{1.0f};
    float m_lpg_resonance{0.2f};
    float m_stereo_link{1.0f};

    LiquidVactrolCell m_cell_l;
    LiquidVactrolCell m_cell_r;
    VactrolSvf m_sc_filter_l;
    VactrolSvf m_sc_filter_r;
    VactrolSvf m_lpg_filter_l;
    VactrolSvf m_lpg_filter_r;

    float m_gr_db_l{0.0f};
    float m_gr_db_r{0.0f};
};

// ============================================================================
// LiquidVactrolProcessor: IProcessor Adapter for InsertSlot Hosting
// ============================================================================
class LiquidVactrolProcessor : public IProcessor {
public:
    explicit LiquidVactrolProcessor(uint32_t sample_rate = 48000)
        : m_vactrol(sample_rate) {}

    void init(uint32_t sample_rate) noexcept override {
        m_vactrol.set_sample_rate(sample_rate);
        m_vactrol.reset();
    }

    void reset() noexcept override {
        m_vactrol.reset();
    }

    [[nodiscard]] bool supports_sidechain() const noexcept override { return true; }

    void process_stereo(Sample* left, Sample* right, uint32_t frames) noexcept override {
        m_vactrol.process_stereo(left, right, frames, nullptr, nullptr);
    }

    void process_stereo_sidechain(Sample* left, Sample* right,
                                  const Sample* sc_left, const Sample* sc_right,
                                  uint32_t frames) noexcept override {
        m_vactrol.process_stereo(left, right, frames, sc_left, sc_right);
    }

    void set_parameter(uint32_t index, float value) noexcept override {
        switch (index) {
            case 0: m_vactrol.set_peak_reduction(value); break;
            case 1: m_vactrol.set_makeup_gain_db(value); break;
            case 2: {
                uint32_t m = static_cast<uint32_t>(std::clamp(value, 0.0f, 2.0f));
                m_vactrol.set_mode(static_cast<VactrolMode>(m));
                break;
            }
            case 3: m_vactrol.set_memory_depth(value); break;
            case 4: m_vactrol.set_hf_emphasis(value); break;
            case 5: m_vactrol.set_mix(value); break;
            case 6: m_vactrol.set_lpg_resonance(value); break;
            case 7: m_vactrol.set_stereo_link(value); break;
            default: break;
        }
    }

    [[nodiscard]] float get_parameter(uint32_t index) const noexcept override {
        switch (index) {
            case 0: return m_vactrol.peak_reduction();
            case 1: return m_vactrol.makeup_gain_db();
            case 2: return static_cast<float>(static_cast<uint32_t>(m_vactrol.mode()));
            case 3: return m_vactrol.memory_depth();
            case 4: return m_vactrol.hf_emphasis();
            case 5: return m_vactrol.mix();
            case 6: return m_vactrol.lpg_resonance();
            case 7: return m_vactrol.stereo_link();
            default: return 0.0f;
        }
    }

    [[nodiscard]] const char* name() const noexcept override {
        return "LiquidVactrol";
    }

    [[nodiscard]] LiquidVactrol& vactrol() noexcept { return m_vactrol; }
    [[nodiscard]] const LiquidVactrol& vactrol() const noexcept { return m_vactrol; }

private:
    LiquidVactrol m_vactrol;
};

} // namespace audio_core::dsp
