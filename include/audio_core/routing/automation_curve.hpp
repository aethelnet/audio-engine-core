#pragma once

#include <vector>
#include <memory>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <span>
#include <string>

namespace audio_core::routing {

// ============================================================================
// FontLab-Inspired Node Typology
// Smooth: Collinear tangents, C^1 Hermite continuity (Circle)
// Corner: Independent tangents, sharp angle / sudden change (Diamond/Square)
// Hold: Constant step until next node (Right angle)
// ============================================================================
enum class NodeMode : uint8_t {
    Smooth = 0,  // C^1 continuous smooth transition
    Corner = 1,  // Sharp inflection / independent slopes
    Hold   = 2   // Immediate discrete jump at next node
};

// Target Parameter Lane in Channel Strip
enum class AutomationTarget : uint8_t {
    Gain = 0,  // Track Volume / Gain Multiplier [0.0, 1.25]
    Pan  = 1,  // Track Stereo Panning [-1.0, +1.0]
    Aux1 = 2,  // Auxiliary Send 1 (e.g. Reverb) [0.0, 1.0]
    Aux2 = 3   // Auxiliary Send 2 (e.g. Delay) [0.0, 1.0]
};

// ============================================================================
// AutomationPoint: Discrete Breakpoint on the Timeline
// time_beats: Song timeline coordinate in musical beats (quarter notes)
// value: Normalized or linear parameter level (e.g. 0.0 to 1.0, or gain multiplier)
// tension: Curvature [-1.0, +1.0] (0.0 = linear, >0 convex, <0 concave)
// ============================================================================
struct AutomationPoint {
    double time_beats{0.0};
    float value{1.0f};
    NodeMode node_mode{NodeMode::Smooth};
    float tension{0.0f}; // Curvature: -1.0 (concave/fast rise) .. 0.0 (linear) .. +1.0 (convex/slow rise)
};

// ============================================================================
// AutomationSnapshot: Immutable Cache-Friendly Vector of Sorted Breakpoints
// Safely exchanged between GUI/Main thread and Real-Time Audio thread via RCU
// ============================================================================
class AutomationSnapshot {
public:
    AutomationSnapshot() = default;
    explicit AutomationSnapshot(std::vector<AutomationPoint> pts) : m_points(std::move(pts)) {
        sort_and_sanitize();
    }

    [[nodiscard]] const std::vector<AutomationPoint>& points() const noexcept { return m_points; }
    [[nodiscard]] size_t size() const noexcept { return m_points.size(); }
    [[nodiscard]] bool empty() const noexcept { return m_points.empty(); }

    // Evaluates the continuous curve value at an instantaneous beat coordinate in O(log N)
    [[nodiscard]] float evaluate(double time_beats) const noexcept {
        if (m_points.empty()) return 1.0f; // Default unity
        if (m_points.size() == 1 || time_beats <= m_points.front().time_beats) {
            return m_points.front().value;
        }
        if (time_beats >= m_points.back().time_beats) {
            return m_points.back().value;
        }

        // Binary search to find segment [p0, p1] where p0.time <= time_beats < p1.time
        auto it = std::upper_bound(m_points.begin(), m_points.end(), time_beats,
            [](double t, const AutomationPoint& pt) {
                return t < pt.time_beats;
            });

        const auto& p1 = *it;
        const auto& p0 = *(it - 1);

        const double dt = p1.time_beats - p0.time_beats;
        if (dt <= 1e-9) return p1.value;

        // Normalized progression u in [0, 1]
        double u = (time_beats - p0.time_beats) / dt;
        u = std::clamp(u, 0.0, 1.0);

        return interpolate_segment(p0, p1, static_cast<float>(u));
    }

    // Audio-rate block evaluation: computes out_buffer[i] sample-accurately across [start_beat, end_beat]
    // 100% lock-free, zero allocation, vector-friendly
    void evaluate_block(double start_beat, double end_beat, float* out_buffer, uint32_t frames) const noexcept {
        if (!out_buffer || frames == 0) return;

        if (m_points.empty()) {
            std::fill_n(out_buffer, frames, 1.0f);
            return;
        }

        if (m_points.size() == 1 || end_beat <= m_points.front().time_beats) {
            std::fill_n(out_buffer, frames, m_points.front().value);
            return;
        }

        if (start_beat >= m_points.back().time_beats) {
            std::fill_n(out_buffer, frames, m_points.back().value);
            return;
        }

        const double beat_step = (frames > 1) ? ((end_beat - start_beat) / static_cast<double>(frames)) : 0.0;

        // Check if the entire block falls inside a single linear segment
        // In typical 64-frame blocks, 99.9% of blocks fall between the same two points!
        auto it = std::upper_bound(m_points.begin(), m_points.end(), start_beat,
            [](double t, const AutomationPoint& pt) {
                return t < pt.time_beats;
            });

        if (it != m_points.end() && it != m_points.begin()) {
            const auto& p1 = *it;
            const auto& p0 = *(it - 1);

            // If end_beat is also within this same segment and segment is purely linear
            if (end_beat <= p1.time_beats && p0.node_mode == NodeMode::Corner && std::abs(p0.tension) < 1e-4f) {
                const double dt = p1.time_beats - p0.time_beats;
                const float v_start = p0.value + static_cast<float>((start_beat - p0.time_beats) / dt) * (p1.value - p0.value);
                const float v_end   = p0.value + static_cast<float>((end_beat - p0.time_beats) / dt) * (p1.value - p0.value);
                const float step = (v_end - v_start) / static_cast<float>(frames);

                #if defined(__GNUC__) || defined(__clang__)
                #pragma GCC ivdep
                #endif
                for (uint32_t i = 0; i < frames; ++i) {
                    out_buffer[i] = v_start + step * static_cast<float>(i);
                }
                return;
            }
        }

        // General curved or multi-node block evaluation
        double cur_beat = start_beat;
        for (uint32_t i = 0; i < frames; ++i) {
            out_buffer[i] = evaluate(cur_beat);
            cur_beat += beat_step;
        }
    }

    // Static helper to interpolate a single segment [p0, p1] with progression u in [0, 1]
    [[nodiscard]] static float interpolate_segment(const AutomationPoint& p0, const AutomationPoint& p1, float u) noexcept {
        if (p0.node_mode == NodeMode::Hold) {
            return (u >= 1.0f) ? p1.value : p0.value;
        }

        const float val_diff = p1.value - p0.value;
        if (std::abs(val_diff) < 1e-7f) return p0.value;

        // FontLab Curvature Tension formula:
        // tension = 0 -> linear
        // tension > 0 -> convex (slow start, rapid finish)
        // tension < 0 -> concave (fast launch, smooth settle)
        float warped_u = u;
        const float tau = p0.tension;

        if (p0.node_mode == NodeMode::Smooth) {
            // C^1 Hermite Smoothstep base: S(u) = 3u^2 - 2u^3
            float s = u * u * (3.0f - 2.0f * u);
            if (std::abs(tau) > 1e-4f) {
                if (tau > 0.0f) {
                    // Convex warp
                    float gamma = 1.0f + 3.0f * tau;
                    warped_u = std::pow(s, gamma);
                } else {
                    // Concave warp
                    float gamma = 1.0f - 3.0f * tau;
                    warped_u = 1.0f - std::pow(1.0f - s, gamma);
                }
            } else {
                warped_u = s;
            }
        } else {
            // Corner mode with optional tension
            if (std::abs(tau) > 1e-4f) {
                if (tau > 0.0f) {
                    warped_u = std::pow(u, 1.0f + 3.0f * tau);
                } else {
                    warped_u = 1.0f - std::pow(1.0f - u, 1.0f - 3.0f * tau);
                }
            }
        }

        return p0.value + val_diff * std::clamp(warped_u, 0.0f, 1.0f);
    }

private:
    void sort_and_sanitize() noexcept {
        if (m_points.empty()) return;
        std::sort(m_points.begin(), m_points.end(), [](const AutomationPoint& a, const AutomationPoint& b) {
            return a.time_beats < b.time_beats;
        });

        // Sanitize values (prevent NaN/Inf) and clamp tension
        for (auto& pt : m_points) {
            if (std::isnan(pt.value) || std::isinf(pt.value)) pt.value = 0.0f;
            pt.tension = std::clamp(pt.tension, -1.0f, 1.0f);
        }
    }

    std::vector<AutomationPoint> m_points;
};

// ============================================================================
// AutomationCurve: Thread-Safe, Real-Time Automation Track Engine
// Lock-free RCU atomic snapshot pointer ensures audio thread reads without locks,
// while UI/Main thread safely mutates points and publishes new snapshots.
// ============================================================================
class AutomationCurve {
public:
    AutomationCurve() {
        // Initial default: 2 points at 0 dB unity gain (Bar 0 and Bar 16)
        std::vector<AutomationPoint> initial_pts = {
            AutomationPoint{0.0, 1.0f, NodeMode::Smooth, 0.0f},
            AutomationPoint{16.0, 1.0f, NodeMode::Smooth, 0.0f}
        };
        m_snapshot.store(std::make_shared<AutomationSnapshot>(std::move(initial_pts)), std::memory_order_release);
    }

    // Audio-Thread API: Lock-Free, Zero Allocations, Zero Mutex
    [[nodiscard]] std::shared_ptr<const AutomationSnapshot> snapshot() const noexcept {
        return m_snapshot.load(std::memory_order_acquire);
    }

    [[nodiscard]] float evaluate_audio_sample(double time_beats) const noexcept {
        auto snap = snapshot();
        return snap ? snap->evaluate(time_beats) : 1.0f;
    }

    void evaluate_audio_block(double start_beat, double end_beat, float* out_buffer, uint32_t frames) const noexcept {
        auto snap = snapshot();
        if (snap) {
            snap->evaluate_block(start_beat, end_beat, out_buffer, frames);
        } else if (out_buffer) {
            std::fill_n(out_buffer, frames, 1.0f);
        }
    }

    // ========================================================================
    // UI & Main-Thread Editing API (FontLab Ergonomics)
    // ========================================================================
    [[nodiscard]] std::vector<AutomationPoint> get_points() const {
        auto snap = snapshot();
        return snap ? snap->points() : std::vector<AutomationPoint>{};
    }

    // Publish a full vector of points
    void set_points(std::vector<AutomationPoint> pts) {
        auto new_snap = std::make_shared<AutomationSnapshot>(std::move(pts));
        m_snapshot.store(std::move(new_snap), std::memory_order_release);
    }

    // Add point: inserts into sorted timeline and publishes new snapshot
    size_t add_point(double time_beats, float value, NodeMode mode = NodeMode::Smooth, float tension = 0.0f) {
        auto current_pts = get_points();
        AutomationPoint new_pt{time_beats, value, mode, tension};

        auto it = std::upper_bound(current_pts.begin(), current_pts.end(), time_beats,
            [](double t, const AutomationPoint& pt) {
                return t < pt.time_beats;
            });

        size_t idx = std::distance(current_pts.begin(), it);
        current_pts.insert(it, new_pt);
        set_points(std::move(current_pts));
        return idx;
    }

    // Remove point by index
    bool remove_point(size_t index) {
        auto current_pts = get_points();
        // Never remove down to 0 points; keep at least 1 point
        if (index >= current_pts.size() || current_pts.size() <= 1) return false;

        current_pts.erase(current_pts.begin() + index);
        set_points(std::move(current_pts));
        return true;
    }

    // Reset curve to single default point (e.g. unity gain)
    void clear(float default_val = 1.0f) {
        std::vector<AutomationPoint> pts = {
            AutomationPoint{0.0, default_val, NodeMode::Smooth, 0.0f}
        };
        set_points(std::move(pts));
    }

    // Modify existing point (moving position or value)
    void update_point(size_t index, double new_time, float new_value) {
        auto current_pts = get_points();
        if (index >= current_pts.size()) return;

        current_pts[index].time_beats = std::max(0.0, new_time);
        current_pts[index].value = (std::isnan(new_value) || std::isinf(new_value)) ? 0.0f : new_value;
        set_points(std::move(current_pts));
    }

    // FontLab Direct Curvature Bending: update tension of segment starting at index
    void set_segment_tension(size_t index, float tension) {
        auto current_pts = get_points();
        if (index >= current_pts.size()) return;

        current_pts[index].tension = std::clamp(tension, -1.0f, 1.0f);
        set_points(std::move(current_pts));
    }

    // FontLab Toggle Node Mode (Double-click on node: Smooth <-> Corner)
    void toggle_node_mode(size_t index) {
        auto current_pts = get_points();
        if (index >= current_pts.size()) return;

        if (current_pts[index].node_mode == NodeMode::Smooth) {
            current_pts[index].node_mode = NodeMode::Corner;
        } else if (current_pts[index].node_mode == NodeMode::Corner) {
            current_pts[index].node_mode = NodeMode::Hold;
        } else {
            current_pts[index].node_mode = NodeMode::Smooth;
        }
        set_points(std::move(current_pts));
    }

    // ========================================================================
    // Quick Musical Presets
    // ========================================================================
    void preset_reset_unity(double total_beats = 16.0) {
        std::vector<AutomationPoint> pts = {
            AutomationPoint{0.0, 1.0f, NodeMode::Smooth, 0.0f},
            AutomationPoint{total_beats, 1.0f, NodeMode::Smooth, 0.0f}
        };
        set_points(std::move(pts));
    }

    void preset_fade_in(double length_beats = 4.0, double total_beats = 16.0) {
        std::vector<AutomationPoint> pts = {
            AutomationPoint{0.0, 0.0f, NodeMode::Smooth, -0.3f}, // Concave smooth rise
            AutomationPoint{length_beats, 1.0f, NodeMode::Smooth, 0.0f},
            AutomationPoint{total_beats, 1.0f, NodeMode::Smooth, 0.0f}
        };
        set_points(std::move(pts));
    }

    void preset_fade_out(double start_beat = 12.0, double total_beats = 16.0) {
        std::vector<AutomationPoint> pts = {
            AutomationPoint{0.0, 1.0f, NodeMode::Smooth, 0.0f},
            AutomationPoint{start_beat, 1.0f, NodeMode::Smooth, 0.3f},
            AutomationPoint{total_beats, 0.0f, NodeMode::Smooth, 0.0f}
        };
        set_points(std::move(pts));
    }

    // 4-on-the-Floor Sidechain Ducking Pump (1/4-note dip on each beat)
    void preset_sidechain_pump(uint32_t num_bars = 4) {
        std::vector<AutomationPoint> pts;
        pts.reserve(num_bars * 8);

        for (uint32_t bar = 0; bar < num_bars; ++bar) {
            for (uint32_t beat = 0; beat < 4; ++beat) {
                double t = static_cast<double>(bar * 4 + beat);
                // Beat downbeat: dip down to 0.05 (-26 dB)
                pts.push_back(AutomationPoint{t, 0.05f, NodeMode::Smooth, -0.6f});
                // Settle back to unity gain by 3/8ths of the beat
                pts.push_back(AutomationPoint{t + 0.65, 1.0f, NodeMode::Smooth, 0.0f});
            }
        }
        set_points(std::move(pts));
    }

    // Auto-Pan LFO sweep (alternates smoothly between Left and Right)
    void preset_sine_pan(double total_beats = 16.0, double cycle_beats = 4.0) {
        std::vector<AutomationPoint> pts;
        const int num_cycles = static_cast<int>(std::max(1.0, std::round(total_beats / cycle_beats)));
        for (int c = 0; c < num_cycles; ++c) {
            double base_t = c * cycle_beats;
            pts.push_back(AutomationPoint{base_t, 0.0f, NodeMode::Smooth, 0.0f});
            pts.push_back(AutomationPoint{base_t + cycle_beats * 0.25, -0.85f, NodeMode::Smooth, 0.0f});
            pts.push_back(AutomationPoint{base_t + cycle_beats * 0.50, 0.0f, NodeMode::Smooth, 0.0f});
            pts.push_back(AutomationPoint{base_t + cycle_beats * 0.75, 0.85f, NodeMode::Smooth, 0.0f});
        }
        pts.push_back(AutomationPoint{total_beats, 0.0f, NodeMode::Smooth, 0.0f});
        set_points(std::move(pts));
    }

    // Reverb / FX Send Build-Up Swell before drop
    void preset_reverb_swell(double start_beat = 12.0, double total_beats = 16.0, float max_send = 0.80f) {
        std::vector<AutomationPoint> pts = {
            AutomationPoint{0.0, 0.0f, NodeMode::Smooth, 0.0f},
            AutomationPoint{start_beat, 0.0f, NodeMode::Smooth, 0.4f},
            AutomationPoint{total_beats, max_send, NodeMode::Smooth, 0.0f}
        };
        set_points(std::move(pts));
    }

    // Delay Throw on specific musical bars
    void preset_delay_throw(uint32_t num_bars = 4, float throw_amount = 0.70f) {
        std::vector<AutomationPoint> pts;
        pts.reserve(num_bars * 4 + 2);
        pts.push_back(AutomationPoint{0.0, 0.0f, NodeMode::Hold, 0.0f});
        for (uint32_t bar = 0; bar < num_bars; ++bar) {
            double bar_start = bar * 4.0;
            if ((bar % 2) == 1) { // Throw on alternating bars
                pts.push_back(AutomationPoint{bar_start + 3.0, throw_amount, NodeMode::Smooth, 0.0f});
                pts.push_back(AutomationPoint{bar_start + 3.9, 0.0f, NodeMode::Smooth, 0.0f});
            }
        }
        pts.push_back(AutomationPoint{static_cast<double>(num_bars * 4), 0.0f, NodeMode::Smooth, 0.0f});
        set_points(std::move(pts));
    }

private:
    std::atomic<std::shared_ptr<const AutomationSnapshot>> m_snapshot;
};

} // namespace audio_core::routing
