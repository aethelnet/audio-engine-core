#pragma once

#include "audio_core/types.hpp"
#include "audio_core/routing/automation_curve.hpp"
#include <vector>
#include <array>
#include <memory>
#include <atomic>
#include <cmath>
#include <algorithm>
#include <string>

namespace audio_core::modulation {

// ============================================================================
// MultiStageEnvelope (MSEG) & Function Generator
// FontLab-grade breakpoint typology, multi-stage loop sustain, and
// fully modulatable macro parameters (Meta-Modulation: LFO -> Attack/Decay).
// Zero heap allocations in the audio thread; lock-free RCU atomic snapshot.
// ============================================================================

enum class MsegTimeMode : uint8_t {
    Milliseconds = 0, // Absolute time in ms (ideal for percussion, one-shots, acoustic emulations)
    BeatSync     = 1  // Musical time in beats/bars (ideal for rhythmic drops, risers, LFO sweeps)
};

enum class MsegLoopMode : uint8_t {
    OneShot      = 0, // Plays through from 0 to end and goes idle (drums, hi-hats, percussive envelopes)
    SustainLoop  = 1, // Holds or loops between sustain points while key is pressed, releases on note-off
    FreeRunLoop  = 2  // Loops continuously from end back to start (Function Generator / Custom LFO)
};

enum class MsegStage : uint8_t {
    Idle    = 0,
    Attack  = 1,
    Decay   = 2,
    Sustain = 3,
    Release = 4
};

struct MsegPoint {
    double time{0.0};       // Time offset: ms (Milliseconds mode) or musical beats (BeatSync mode)
    float value{0.0f};      // Parameter level [0.0, 1.0] (unipolar) or [-1.0, +1.0] (bipolar)
    routing::NodeMode node_mode{routing::NodeMode::Smooth};
    float tension{0.0f};    // Curvature: -1.0 (concave / fast attack) .. 0.0 (linear) .. +1.0 (convex / slow swell)
};

// ============================================================================
// MsegSnapshot: Immutable RCU Spline Snapshot
// ============================================================================
class MsegSnapshot {
public:
    MsegSnapshot() {
        // Default 3-point ADSR-like curve (0 -> 1 -> 0)
        m_points = {
            MsegPoint{0.0,   0.0f, routing::NodeMode::Smooth, 0.0f},
            MsegPoint{50.0,  1.0f, routing::NodeMode::Smooth, 0.0f},
            MsegPoint{250.0, 0.0f, routing::NodeMode::Smooth, 0.0f}
        };
        analyze();
    }

    explicit MsegSnapshot(std::vector<MsegPoint> pts,
                          MsegTimeMode time_mode = MsegTimeMode::Milliseconds,
                          MsegLoopMode loop_mode = MsegLoopMode::OneShot,
                          int32_t sustain_idx = -1)
        : m_points(std::move(pts)),
          m_time_mode(time_mode),
          m_loop_mode(loop_mode),
          m_sustain_idx(sustain_idx) {
        sanitize();
        analyze();
    }

    [[nodiscard]] const std::vector<MsegPoint>& points() const noexcept { return m_points; }
    [[nodiscard]] MsegTimeMode time_mode() const noexcept { return m_time_mode; }
    [[nodiscard]] MsegLoopMode loop_mode() const noexcept { return m_loop_mode; }
    [[nodiscard]] int32_t sustain_index() const noexcept { return m_sustain_idx; }
    [[nodiscard]] double total_duration() const noexcept { return m_total_duration; }
    [[nodiscard]] size_t peak_index() const noexcept { return m_peak_idx; }

    // Evaluates the envelope curve at a given time position with segment scaling
    [[nodiscard]] float evaluate(double raw_time,
                                 float attack_scale = 1.0f,
                                 float decay_scale = 1.0f,
                                 float release_scale = 1.0f,
                                 float tension_bias = 0.0f) const noexcept {
        if (m_points.empty()) return 0.0f;
        if (m_points.size() == 1) return m_points[0].value;

        // Apply scale factors to map raw_time to effective unscaled curve time
        const double eval_t = map_time_to_curve(raw_time, attack_scale, decay_scale, release_scale);

        if (eval_t <= m_points.front().time) {
            return m_points.front().value;
        }
        if (eval_t >= m_points.back().time) {
            return m_points.back().value;
        }

        // Binary search for segment [i, i+1]
        size_t low = 0;
        size_t high = m_points.size() - 1;
        while (low + 1 < high) {
            size_t mid = low + (high - low) / 2;
            if (m_points[mid].time <= eval_t) {
                low = mid;
            } else {
                high = mid;
            }
        }

        const auto& p0 = m_points[low];
        const auto& p1 = m_points[high];
        const double dt = p1.time - p0.time;
        if (dt <= 1e-9) return p0.value;

        const float u = static_cast<float>((eval_t - p0.time) / dt);
        const float clamped_u = std::clamp(u, 0.0f, 1.0f);

        // Effective tension with dynamic bias
        const float effective_tension = std::clamp(p0.tension + tension_bias, -1.0f, 1.0f);

        // Interpolation modes
        if (p0.node_mode == routing::NodeMode::Hold) {
            return p0.value;
        }

        float warped_u = clamped_u;
        if (std::abs(effective_tension) > 1e-4f) {
            if (effective_tension > 0.0f) {
                // Convex: slow initial rise, sharp swell at end
                const float exp_val = 1.0f + (3.0f * effective_tension);
                warped_u = std::pow(clamped_u, exp_val);
            } else {
                // Concave: fast aggressive initial rise / punch
                const float exp_val = 1.0f + (3.0f * (-effective_tension));
                warped_u = 1.0f - std::pow(1.0f - clamped_u, exp_val);
            }
        }

        if (p0.node_mode == routing::NodeMode::Smooth) {
            // Hermite C^1 smoothstep
            warped_u = warped_u * warped_u * (3.0f - (2.0f * warped_u));
        }

        return p0.value + (warped_u * (p1.value - p0.value));
    }

private:
    void sanitize() {
        if (m_points.empty()) {
            m_points.push_back(MsegPoint{0.0, 0.0f, routing::NodeMode::Smooth, 0.0f});
            return;
        }
        std::sort(m_points.begin(), m_points.end(), [](const MsegPoint& a, const MsegPoint& b) {
            return a.time < b.time;
        });
        m_points[0].time = std::max(0.0, m_points[0].time);
        for (size_t i = 1; i < m_points.size(); ++i) {
            if (m_points[i].time <= m_points[i - 1].time) {
                m_points[i].time = m_points[i - 1].time + 1e-4;
            }
        }
    }

    void analyze() {
        m_total_duration = m_points.back().time;
        m_peak_idx = 0;
        float max_val = -1e9f;
        for (size_t i = 0; i < m_points.size(); ++i) {
            if (m_points[i].value > max_val) {
                max_val = m_points[i].value;
                m_peak_idx = i;
            }
        }
        m_peak_time = m_points[m_peak_idx].time;
    }

    // Maps scaled time to unscaled internal curve coordinate based on Attack / Decay / Release zones
    [[nodiscard]] double map_time_to_curve(double raw_t, float atk_scale, float dec_scale, float rel_scale) const noexcept {
        atk_scale = std::max(0.02f, atk_scale);
        dec_scale = std::max(0.02f, dec_scale);
        rel_scale = std::max(0.02f, rel_scale);

        const double scaled_peak_t = m_peak_time * static_cast<double>(atk_scale);

        if (raw_t <= scaled_peak_t) {
            // In Attack phase: unscale by attack_scale
            return raw_t / static_cast<double>(atk_scale);
        }

        // Post-peak Decay/Release phase
        double dt_post = raw_t - scaled_peak_t;
        double unscaled_post = dt_post / static_cast<double>(dec_scale);
        return m_peak_time + unscaled_post;
    }

    std::vector<MsegPoint> m_points;
    MsegTimeMode m_time_mode{MsegTimeMode::Milliseconds};
    MsegLoopMode m_loop_mode{MsegLoopMode::OneShot};
    int32_t m_sustain_idx{-1};
    double m_total_duration{250.0};
    size_t m_peak_idx{1};
    double m_peak_time{50.0};
};

// ============================================================================
// MsegVoice: Per-Voice State Machine (Zero-Allocation POD)
// Represents one polyphonic voice instance (e.g., Hi-Hat voice, Synth Voice 1)
// ============================================================================
class MsegVoice {
public:
    MsegVoice() = default;

    void trigger(float velocity = 1.0f) noexcept {
        m_time = 0.0;
        m_velocity = std::clamp(velocity, 0.0f, 1.0f);
        m_gate_held = true;
        m_stage = MsegStage::Attack;
        m_active = true;
        m_release_start_val = m_current_val;
    }

    void release() noexcept {
        if (!m_active) return;
        m_gate_held = false;
        m_stage = MsegStage::Release;
        m_release_start_val = m_current_val;
        m_release_time = 0.0;
    }

    void reset() noexcept {
        m_time = 0.0;
        m_current_val = 0.0f;
        m_velocity = 1.0f;
        m_gate_held = false;
        m_stage = MsegStage::Idle;
        m_active = false;
    }

    [[nodiscard]] bool is_active() const noexcept { return m_active; }
    [[nodiscard]] float current_value() const noexcept { return m_current_val; }
    [[nodiscard]] MsegStage stage() const noexcept { return m_stage; }
    [[nodiscard]] double playhead_time() const noexcept { return m_time; }

    // Evaluates a single audio sample
    float process_sample(const MsegSnapshot& snap,
                         uint32_t sample_rate,
                         double bpm = 120.0,
                         float attack_scale = 1.0f,
                         float decay_scale = 1.0f,
                         float time_scale = 1.0f,
                         float level_scale = 1.0f,
                         float tension_bias = 0.0f) noexcept {
        if (!m_active) {
            m_current_val = 0.0f;
            return 0.0f;
        }

        const double sr = static_cast<double>(std::max(1000u, sample_rate));
        time_scale = std::max(0.01f, time_scale);

        // Advance playhead time
        double dt = 0.0;
        if (snap.time_mode() == MsegTimeMode::Milliseconds) {
            // Milliseconds per sample: 1000.0 / sr
            dt = (1000.0 / sr) * static_cast<double>(time_scale);
        } else {
            // Beats per sample: (bpm / 60.0) / sr
            dt = ((bpm / 60.0) / sr) * static_cast<double>(time_scale);
        }

        m_time += dt;

        // Loop / Sustain Logic
        const double total_dur = snap.total_duration();

        if (snap.loop_mode() == MsegLoopMode::FreeRunLoop) {
            if (total_dur > 1e-4) {
                while (m_time >= total_dur) {
                    m_time -= total_dur;
                }
            }
        } else if (snap.loop_mode() == MsegLoopMode::SustainLoop && m_gate_held) {
            // If sustain point set, hold or loop at sustain point
            const int32_t sus_idx = snap.sustain_index();
            if (sus_idx >= 0 && sus_idx < static_cast<int32_t>(snap.points().size())) {
                const double sus_time = snap.points()[sus_idx].time;
                if (m_time >= sus_time) {
                    m_time = sus_time; // Park at sustain level while gate is held
                    m_stage = MsegStage::Sustain;
                }
            }
        } else {
            // OneShot or Released
            if (m_time >= total_dur) {
                m_time = total_dur;
                m_active = false;
                m_stage = MsegStage::Idle;
            }
        }

        float raw_val = snap.evaluate(m_time, attack_scale, decay_scale, 1.0f, tension_bias);

        // Apply amplitude scaling & velocity sensitivity
        m_current_val = raw_val * level_scale * m_velocity;
        return m_current_val;
    }

    // Block-rate vector evaluation into destination buffer
    void process_block(float* out_buffer,
                       uint32_t frames,
                       const MsegSnapshot& snap,
                       uint32_t sample_rate,
                       double bpm = 120.0,
                       float attack_scale = 1.0f,
                       float decay_scale = 1.0f,
                       float time_scale = 1.0f,
                       float level_scale = 1.0f,
                       float tension_bias = 0.0f) noexcept {
        if (!out_buffer || frames == 0) return;
        for (uint32_t i = 0; i < frames; ++i) {
            out_buffer[i] = process_sample(snap, sample_rate, bpm, attack_scale, decay_scale, time_scale, level_scale, tension_bias);
        }
    }

private:
    double m_time{0.0};
    double m_release_time{0.0};
    float m_current_val{0.0f};
    float m_release_start_val{0.0f};
    float m_velocity{1.0f};
    bool m_gate_held{false};
    bool m_active{false};
    MsegStage m_stage{MsegStage::Idle};
};

// ============================================================================
// MultiStageEnvelope: Shared Definition + Real-Time Engine Interface
// Owns the RCU snapshot and the modulatory macro controls.
// ============================================================================
class MultiStageEnvelope {
public:
    MultiStageEnvelope() {
        m_snapshot.store(std::make_shared<MsegSnapshot>(), std::memory_order_release);
    }

    explicit MultiStageEnvelope(std::vector<MsegPoint> pts,
                                MsegTimeMode time_mode = MsegTimeMode::Milliseconds,
                                MsegLoopMode loop_mode = MsegLoopMode::OneShot,
                                int32_t sustain_idx = -1) {
        m_snapshot.store(std::make_shared<MsegSnapshot>(std::move(pts), time_mode, loop_mode, sustain_idx),
                         std::memory_order_release);
    }

    // Audio-thread lock-free RCU acquire
    [[nodiscard]] std::shared_ptr<const MsegSnapshot> snapshot() const noexcept {
        return m_snapshot.load(std::memory_order_acquire);
    }

    // Main-thread mutation: publishes new snapshot atomically
    void set_points(std::vector<MsegPoint> pts,
                    MsegTimeMode time_mode = MsegTimeMode::Milliseconds,
                    MsegLoopMode loop_mode = MsegLoopMode::OneShot,
                    int32_t sustain_idx = -1) {
        auto new_snap = std::make_shared<MsegSnapshot>(std::move(pts), time_mode, loop_mode, sustain_idx);
        m_snapshot.store(std::move(new_snap), std::memory_order_release);
    }

    [[nodiscard]] std::vector<MsegPoint> get_points() const {
        auto snap = snapshot();
        return snap ? snap->points() : std::vector<MsegPoint>{};
    }

    [[nodiscard]] MsegTimeMode time_mode() const noexcept {
        auto snap = snapshot();
        return snap ? snap->time_mode() : MsegTimeMode::Milliseconds;
    }

    [[nodiscard]] MsegLoopMode loop_mode() const noexcept {
        auto snap = snapshot();
        return snap ? snap->loop_mode() : MsegLoopMode::OneShot;
    }

    [[nodiscard]] int32_t sustain_index() const noexcept {
        auto snap = snapshot();
        return snap ? snap->sustain_index() : -1;
    }

    // ========================================================================
    // Modulatable Macro Parameters (Meta-Modulation: LFO -> Attack/Decay)
    // ========================================================================
    void set_base_time_scale(float scale) noexcept { m_base_time_scale.store(scale, std::memory_order_relaxed); }
    [[nodiscard]] float base_time_scale() const noexcept { return m_base_time_scale.load(std::memory_order_relaxed); }

    void set_base_attack_scale(float scale) noexcept { m_base_attack_scale.store(scale, std::memory_order_relaxed); }
    [[nodiscard]] float base_attack_scale() const noexcept { return m_base_attack_scale.load(std::memory_order_relaxed); }

    void set_base_decay_scale(float scale) noexcept { m_base_decay_scale.store(scale, std::memory_order_relaxed); }
    [[nodiscard]] float base_decay_scale() const noexcept { return m_base_decay_scale.load(std::memory_order_relaxed); }

    void set_base_level_scale(float scale) noexcept { m_base_level_scale.store(scale, std::memory_order_relaxed); }
    [[nodiscard]] float base_level_scale() const noexcept { return m_base_level_scale.load(std::memory_order_relaxed); }

    void set_base_tension_offset(float tens) noexcept { m_base_tension_offset.store(tens, std::memory_order_relaxed); }
    [[nodiscard]] float base_tension_offset() const noexcept { return m_base_tension_offset.load(std::memory_order_relaxed); }

    // Dynamic modulation offsets injected by LFOs, Envelopes, or Velocity
    void set_mod_attack_scale(float offset) noexcept { m_mod_attack_scale.store(offset, std::memory_order_relaxed); }
    void set_mod_decay_scale(float offset) noexcept { m_mod_decay_scale.store(offset, std::memory_order_relaxed); }
    void set_mod_time_scale(float offset) noexcept { m_mod_time_scale.store(offset, std::memory_order_relaxed); }
    void set_mod_level_scale(float offset) noexcept { m_mod_level_scale.store(offset, std::memory_order_relaxed); }
    void set_mod_tension_offset(float offset) noexcept { m_mod_tension_offset.store(offset, std::memory_order_relaxed); }

    // Effective modulated values
    [[nodiscard]] float effective_attack_scale() const noexcept {
        return std::max(0.01f, m_base_attack_scale.load(std::memory_order_relaxed) + m_mod_attack_scale.load(std::memory_order_relaxed));
    }
    [[nodiscard]] float effective_decay_scale() const noexcept {
        return std::max(0.01f, m_base_decay_scale.load(std::memory_order_relaxed) + m_mod_decay_scale.load(std::memory_order_relaxed));
    }
    [[nodiscard]] float effective_time_scale() const noexcept {
        return std::max(0.01f, m_base_time_scale.load(std::memory_order_relaxed) + m_mod_time_scale.load(std::memory_order_relaxed));
    }
    [[nodiscard]] float effective_level_scale() const noexcept {
        return std::max(0.0f, m_base_level_scale.load(std::memory_order_relaxed) + m_mod_level_scale.load(std::memory_order_relaxed));
    }
    [[nodiscard]] float effective_tension_offset() const noexcept {
        return std::clamp(m_base_tension_offset.load(std::memory_order_relaxed) + m_mod_tension_offset.load(std::memory_order_relaxed), -1.0f, 1.0f);
    }

    // Convenient voice processor call using this MSEG's shared curve & modulated parameters
    float process_voice_sample(MsegVoice& voice, uint32_t sample_rate, double bpm = 120.0) const noexcept {
        auto snap = snapshot();
        if (!snap) return 0.0f;
        return voice.process_sample(*snap,
                                    sample_rate,
                                    bpm,
                                    effective_attack_scale(),
                                    effective_decay_scale(),
                                    effective_time_scale(),
                                    effective_level_scale(),
                                    effective_tension_offset());
    }

    // ========================================================================
    // Curated Musical & Percussive Presets
    // ========================================================================
    void preset_percussive_hihat() {
        std::vector<MsegPoint> pts = {
            MsegPoint{0.0,   0.0f, routing::NodeMode::Corner, 0.0f},
            MsegPoint{1.5,   1.0f, routing::NodeMode::Corner, 0.0f}, // 1.5ms instant click attack
            MsegPoint{35.0,  0.15f, routing::NodeMode::Corner, -0.6f}, // aggressive exponential decay
            MsegPoint{75.0,  0.0f, routing::NodeMode::Corner, -0.4f}  // quick choke release
        };
        set_points(std::move(pts), MsegTimeMode::Milliseconds, MsegLoopMode::OneShot, -1);
    }

    void preset_plucked_synth() {
        std::vector<MsegPoint> pts = {
            MsegPoint{0.0,   0.0f, routing::NodeMode::Corner, 0.0f},
            MsegPoint{3.0,   1.0f, routing::NodeMode::Smooth, 0.0f},
            MsegPoint{120.0, 0.3f, routing::NodeMode::Smooth, -0.3f},
            MsegPoint{280.0, 0.0f, routing::NodeMode::Smooth, -0.5f}
        };
        set_points(std::move(pts), MsegTimeMode::Milliseconds, MsegLoopMode::SustainLoop, 2);
    }

    void preset_pad_swell() {
        std::vector<MsegPoint> pts = {
            MsegPoint{0.0,    0.0f, routing::NodeMode::Smooth, 0.4f},
            MsegPoint{600.0,  1.0f, routing::NodeMode::Smooth, -0.2f},
            MsegPoint{1200.0, 0.8f, routing::NodeMode::Smooth, 0.0f},
            MsegPoint{2000.0, 0.0f, routing::NodeMode::Smooth, -0.3f}
        };
        set_points(std::move(pts), MsegTimeMode::Milliseconds, MsegLoopMode::SustainLoop, 2);
    }

    void preset_wobble_lfo(double beats = 4.0) {
        // Multi-stage rhythmic bounce
        std::vector<MsegPoint> pts = {
            MsegPoint{0.0,              0.0f, routing::NodeMode::Smooth, 0.0f},
            MsegPoint{beats * 0.25,     1.0f, routing::NodeMode::Smooth, 0.0f},
            MsegPoint{beats * 0.50,     0.3f, routing::NodeMode::Smooth, 0.0f},
            MsegPoint{beats * 0.75,     0.9f, routing::NodeMode::Smooth, 0.0f},
            MsegPoint{beats,            0.0f, routing::NodeMode::Smooth, 0.0f}
        };
        set_points(std::move(pts), MsegTimeMode::BeatSync, MsegLoopMode::FreeRunLoop, -1);
    }

    void preset_buchla_maths() {
        // Dual-stage logarithmic inflection
        std::vector<MsegPoint> pts = {
            MsegPoint{0.0,   0.0f, routing::NodeMode::Smooth, 0.7f},
            MsegPoint{15.0,  0.4f, routing::NodeMode::Corner, 0.0f},
            MsegPoint{45.0,  1.0f, routing::NodeMode::Smooth, -0.5f},
            MsegPoint{160.0, 0.2f, routing::NodeMode::Corner, -0.7f},
            MsegPoint{350.0, 0.0f, routing::NodeMode::Smooth, 0.0f}
        };
        set_points(std::move(pts), MsegTimeMode::Milliseconds, MsegLoopMode::OneShot, -1);
    }

private:
    std::atomic<std::shared_ptr<const MsegSnapshot>> m_snapshot{nullptr};

    std::atomic<float> m_base_time_scale{1.0f};
    std::atomic<float> m_base_attack_scale{1.0f};
    std::atomic<float> m_base_decay_scale{1.0f};
    std::atomic<float> m_base_level_scale{1.0f};
    std::atomic<float> m_base_tension_offset{0.0f};

    std::atomic<float> m_mod_attack_scale{0.0f};
    std::atomic<float> m_mod_decay_scale{0.0f};
    std::atomic<float> m_mod_time_scale{0.0f};
    std::atomic<float> m_mod_level_scale{0.0f};
    std::atomic<float> m_mod_tension_offset{0.0f};
};

} // namespace audio_core::modulation
