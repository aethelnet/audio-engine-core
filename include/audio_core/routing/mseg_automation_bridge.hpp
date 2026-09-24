#pragma once

#include "audio_core/types.hpp"
#include "audio_core/routing/automation_curve.hpp"
#include "audio_core/modulation/multi_stage_envelope.hpp"

#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>

namespace audio_core::routing {

// ============================================================================
// MsegAutomationBridge: Bidirectional Conversion & Arranger Stamping Engine
// Bridges the procedural MultiStageEnvelope (MSEG) modulation typology with
// the timeline-anchored AutomationCurve Hermite spline engine.
// Features:
// 1. Precise time-mode conversion (Milliseconds <-> BeatSync via Song BPM).
// 2. Value scaling & range normalization [min_val, max_val].
// 3. Stamping & Tiling N repetitions across [start_beat, end_beat].
// 4. Extraction: Capture timeline automation curves and turn them into reusable MSEG shapes.
// 5. Dynamic runtime modulation evaluation without baking.
// ============================================================================
class MsegAutomationBridge {
public:
    enum class FitMode : uint8_t {
        FitDuration = 0, // Compresses/stretches K repetitions to fit exactly into [start_beat, end_beat]
        NativeTempo = 1  // Retains native MSEG cycle length and repeats until end_beat
    };

    // ========================================================================
    // 1. Bake MSEG into an AutomationCurve
    // ========================================================================
    static void bake_to_curve(
        const modulation::MultiStageEnvelope& mseg,
        AutomationCurve& target_curve,
        double start_beat,
        double end_beat,
        uint32_t repetitions = 1,
        float min_val = 0.0f,
        float max_val = 1.0f,
        FitMode fit_mode = FitMode::FitDuration,
        bool replace_range = true,
        double bpm = 120.0
    ) {
        auto mseg_pts = mseg.get_points();
        if (mseg_pts.empty() || end_beat <= start_beat) {
            return;
        }

        // Determine MSEG duration in musical beats
        const auto time_mode = mseg.time_mode();
        double mseg_dur_raw = mseg_pts.back().time - mseg_pts.front().time;
        if (mseg_dur_raw <= 1e-6) mseg_dur_raw = 1.0;

        double mseg_dur_beats = mseg_dur_raw;
        if (time_mode == modulation::MsegTimeMode::Milliseconds) {
            // ms -> beats: ms / 1000.0 * (bpm / 60.0)
            const double beats_per_sec = bpm / 60.0;
            mseg_dur_beats = (mseg_dur_raw / 1000.0) * beats_per_sec;
        }

        const uint32_t num_cycles = std::max(1u, repetitions);
        const double total_span = end_beat - start_beat;

        double cycle_dur_beats = 0.0;
        double time_scale = 1.0;

        if (fit_mode == FitMode::FitDuration) {
            cycle_dur_beats = total_span / static_cast<double>(num_cycles);
            time_scale = (mseg_dur_beats > 1e-6) ? (cycle_dur_beats / mseg_dur_beats) : 1.0;
        } else {
            cycle_dur_beats = mseg_dur_beats;
            time_scale = 1.0;
        }

        std::vector<AutomationPoint> baked_pts;
        baked_pts.reserve(num_cycles * mseg_pts.size());

        const double t0 = mseg_pts.front().time;
        const float val_range = max_val - min_val;

        for (uint32_t c = 0; c < num_cycles; ++c) {
            const double cycle_start = start_beat + static_cast<double>(c) * cycle_dur_beats;
            if (cycle_start >= end_beat && c > 0) break;

            for (size_t i = 0; i < mseg_pts.size(); ++i) {
                const auto& mp = mseg_pts[i];
                double pt_offset_beats = mp.time - t0;
                if (time_mode == modulation::MsegTimeMode::Milliseconds) {
                    pt_offset_beats = (pt_offset_beats / 1000.0) * (bpm / 60.0);
                }

                double pt_beat = cycle_start + pt_offset_beats * time_scale;
                if (pt_beat > end_beat + 1e-4) {
                    continue; // Skip beyond end
                }

                float scaled_val = min_val + mp.value * val_range;

                // If this is the start of a repeating cycle and equals the last point's time
                if (!baked_pts.empty() && std::abs(baked_pts.back().time_beats - pt_beat) < 1e-4) {
                    if (std::abs(baked_pts.back().value - scaled_val) < 1e-4f) {
                        // Values match: weld the boundary point seamlessly
                        baked_pts.back().node_mode = mp.node_mode;
                        baked_pts.back().tension = mp.tension;
                        continue;
                    } else {
                        // Step discontinuity: advance new cycle start by micro-epsilon (1e-5 beats)
                        pt_beat += 1e-5;
                    }
                }
                baked_pts.push_back(AutomationPoint{
                    .time_beats = pt_beat,
                    .value = scaled_val,
                    .node_mode = mp.node_mode,
                    .tension = mp.tension
                });
            }
        }

        if (baked_pts.empty()) return;

        if (!replace_range) {
            // Append or insert points directly
            for (const auto& bp : baked_pts) {
                target_curve.add_point(bp.time_beats, bp.value, bp.node_mode, bp.tension);
            }
            return;
        }

        // Replace range: filter out points inside [start_beat, end_beat]
        auto existing = target_curve.get_points();
        std::vector<AutomationPoint> merged;
        merged.reserve(existing.size() + baked_pts.size());

        for (const auto& ep : existing) {
            if (ep.time_beats < start_beat - 1e-4 || ep.time_beats > end_beat + 1e-4) {
                merged.push_back(ep);
            }
        }

        for (const auto& bp : baked_pts) {
            merged.push_back(bp);
        }

        std::sort(merged.begin(), merged.end(), [](const AutomationPoint& a, const AutomationPoint& b) {
            return a.time_beats < b.time_beats;
        });

        // Ensure minimum 1 point
        if (merged.empty()) {
            merged.push_back(AutomationPoint{0.0, min_val, NodeMode::Smooth, 0.0f});
        }

        target_curve.set_points(std::move(merged));
    }

    // Creates a standalone baked AutomationCurve
    static AutomationCurve create_baked_curve(
        const modulation::MultiStageEnvelope& mseg,
        double start_beat,
        double end_beat,
        uint32_t repetitions = 1,
        float min_val = 0.0f,
        float max_val = 1.0f,
        FitMode fit_mode = FitMode::FitDuration,
        double bpm = 120.0
    ) {
        AutomationCurve curve;
        bake_to_curve(mseg, curve, start_beat, end_beat, repetitions, min_val, max_val, fit_mode, true, bpm);
        return curve;
    }

    // ========================================================================
    // 2. Extract MSEG from an AutomationCurve
    // ========================================================================
    static std::vector<modulation::MsegPoint> extract_points(
        const AutomationCurve& curve,
        double start_beat,
        double end_beat,
        modulation::MsegTimeMode time_mode = modulation::MsegTimeMode::BeatSync,
        float min_val = 0.0f,
        float max_val = 1.0f,
        double bpm = 120.0
    ) {
        auto pts = curve.get_points();
        std::vector<modulation::MsegPoint> mseg_pts;

        const float val_range = (std::abs(max_val - min_val) > 1e-6f) ? (max_val - min_val) : 1.0f;
        const double sec_per_beat = 60.0 / std::max(1.0, bpm);

        // Find points in [start_beat, end_beat]
        for (const auto& p : pts) {
            if (p.time_beats >= start_beat - 1e-4 && p.time_beats <= end_beat + 1e-4) {
                double rel_beat = std::max(0.0, p.time_beats - start_beat);
                double t = rel_beat;
                if (time_mode == modulation::MsegTimeMode::Milliseconds) {
                    t = rel_beat * sec_per_beat * 1000.0;
                }

                float norm_val = std::clamp((p.value - min_val) / val_range, 0.0f, 1.0f);
                mseg_pts.push_back(modulation::MsegPoint{
                    .time = t,
                    .value = norm_val,
                    .node_mode = p.node_mode,
                    .tension = p.tension
                });
            }
        }

        // If no points fall strictly inside, sample the start and end values
        if (mseg_pts.empty()) {
            float v0 = curve.evaluate_audio_sample(start_beat);
            float v1 = curve.evaluate_audio_sample(end_beat);
            double total_dur = (end_beat - start_beat);
            if (time_mode == modulation::MsegTimeMode::Milliseconds) {
                total_dur = total_dur * sec_per_beat * 1000.0;
            }

            mseg_pts.push_back(modulation::MsegPoint{0.0, std::clamp((v0 - min_val) / val_range, 0.0f, 1.0f), NodeMode::Smooth, 0.0f});
            mseg_pts.push_back(modulation::MsegPoint{total_dur, std::clamp((v1 - min_val) / val_range, 0.0f, 1.0f), NodeMode::Smooth, 0.0f});
        } else {
            // Ensure first point starts at t = 0
            if (mseg_pts.front().time > 1e-4) {
                float v0 = curve.evaluate_audio_sample(start_beat);
                mseg_pts.insert(mseg_pts.begin(), modulation::MsegPoint{0.0, std::clamp((v0 - min_val) / val_range, 0.0f, 1.0f), NodeMode::Smooth, 0.0f});
            }
            // Ensure last point ends at duration
            double end_dur = (end_beat - start_beat);
            if (time_mode == modulation::MsegTimeMode::Milliseconds) {
                end_dur = end_dur * sec_per_beat * 1000.0;
            }
            if (mseg_pts.back().time < end_dur - 1e-4) {
                float v1 = curve.evaluate_audio_sample(end_beat);
                mseg_pts.push_back(modulation::MsegPoint{end_dur, std::clamp((v1 - min_val) / val_range, 0.0f, 1.0f), NodeMode::Smooth, 0.0f});
            }
        }

        return mseg_pts;
    }

    static void extract_to_mseg(
        const AutomationCurve& curve,
        modulation::MultiStageEnvelope& target_mseg,
        double start_beat,
        double end_beat,
        modulation::MsegTimeMode time_mode = modulation::MsegTimeMode::BeatSync,
        float min_val = 0.0f,
        float max_val = 1.0f,
        double bpm = 120.0
    ) {
        auto pts = extract_points(curve, start_beat, end_beat, time_mode, min_val, max_val, bpm);
        target_mseg.set_points(std::move(pts), time_mode, modulation::MsegLoopMode::OneShot, -1);
    }

    static modulation::MultiStageEnvelope extract_mseg(
        const AutomationCurve& curve,
        double start_beat,
        double end_beat,
        modulation::MsegTimeMode time_mode = modulation::MsegTimeMode::BeatSync,
        float min_val = 0.0f,
        float max_val = 1.0f,
        double bpm = 120.0
    ) {
        auto pts = extract_points(curve, start_beat, end_beat, time_mode, min_val, max_val, bpm);
        return modulation::MultiStageEnvelope(std::move(pts), time_mode, modulation::MsegLoopMode::OneShot, -1);
    }
};

// ============================================================================
// DynamicMsegModulator: Real-time non-baking continuous parameter modulator
// Evaluates an active MSEG as a runtime automation source
// ============================================================================
class DynamicMsegModulator {
public:
    DynamicMsegModulator() = default;

    void arm(float min_val = 0.0f, float max_val = 1.0f) noexcept {
        m_min_val = min_val;
        m_max_val = max_val;
        m_voice.reset();
        m_voice.trigger(1.0f);
        m_active = true;
    }

    void disarm() noexcept {
        m_active = false;
        m_voice.reset();
    }

    [[nodiscard]] bool is_active() const noexcept { return m_active; }

    float process_sample(const modulation::MultiStageEnvelope& mseg, uint32_t sample_rate, double bpm = 120.0) noexcept {
        if (!m_active) return m_min_val;
        float raw = mseg.process_voice_sample(m_voice, sample_rate, bpm);
        return m_min_val + raw * (m_max_val - m_min_val);
    }

    void process_block(float* out_buffer, uint32_t frames, const modulation::MultiStageEnvelope& mseg, uint32_t sample_rate, double bpm = 120.0) noexcept {
        if (!out_buffer || frames == 0) return;
        if (!m_active) {
            std::fill_n(out_buffer, frames, m_min_val);
            return;
        }
        for (uint32_t i = 0; i < frames; ++i) {
            out_buffer[i] = process_sample(mseg, sample_rate, bpm);
        }
    }

    [[nodiscard]] modulation::MsegVoice& voice() noexcept { return m_voice; }
    [[nodiscard]] const modulation::MsegVoice& voice() const noexcept { return m_voice; }

private:
    bool m_active{false};
    float m_min_val{0.0f};
    float m_max_val{1.0f};
    modulation::MsegVoice m_voice;
};

} // namespace audio_core::routing
