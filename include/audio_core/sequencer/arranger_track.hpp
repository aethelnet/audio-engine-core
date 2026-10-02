#pragma once

#include "audio_core/types.hpp"
#include "audio_core/clock/timeline_clock.hpp"
#include "audio_core/sampling/audio_clip.hpp"
#include "audio_core/dsp/resampler.hpp"
#include "audio_core/dsp/time_stretcher.hpp"
#include "audio_core/analysis/transient_detector.hpp"
#include "audio_core/sequencer/step_sequencer.hpp"

#include <vector>
#include <memory>
#include <atomic>
#include <string>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <utility>
#include <cstdint>

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

namespace audio_core::sequencer {

// ============================================================================
// ArrangerClipType: Clip Medium Type (Prismatic Audio vs Renoise Tracker Pattern)
// ============================================================================
enum class ArrangerClipType : uint8_t {
    Audio = 0,    // Pure audio sample asset with elastic WSOLA/Hermite playback
    Pattern = 1   // Renoise/Tracker/Step pattern dispatching polyphonic slice voices
};

// ============================================================================
// WarpPin: Non-Linear Elastic Time-Warping Anchor Point
// Binds an exact source audio frame to a target musical timeline position.
// ============================================================================
struct WarpPin {
    uint32_t id{0};
    uint32_t source_frame{0};   // Frame index within pristine source asset
    float pinned_bar{0.0f};      // Relative musical position (in bars) within clip [0.0, len_bars]

    bool operator==(const WarpPin&) const = default;
};

// ============================================================================
// ArrangerClipInstance: Timeline Event Container
// Supports per-clip trimming, non-destructive slip-editing, 5 fade shapes,
// automatic equal-power crossfading, and granular WSOLA time-stretching.
// ============================================================================
struct ArrangerClipInstance {
    uint32_t id{0};
    std::string name{""};
    std::shared_ptr<sampling::AudioClip> clip{nullptr};
    std::shared_ptr<sampling::AudioClip> source_clip{nullptr}; // Pristine un-stretched master asset
    ArrangerClipType type{ArrangerClipType::Audio};            // Audio sample or Renoise/Tracker pattern
    uint32_t pattern_index{0};                                 // Pattern index in track's sequencer [0..15]

    float start_bar{0.0f};           // Absolute timeline position in musical bars
    float len_bars{4.0f};            // Timeline duration in musical bars
    float base_len_bars{4.0f};       // Base unstretched length of source_clip in bars
    float orig_len_bars{4.0f};       // Current audio asset duration in musical bars
    float stretch_ratio{1.0f};       // Time-stretch ratio (orig_len_bars / base_len_bars)
    dsp::PitchAlgorithm stretch_algo{dsp::PitchAlgorithm::RubberbandWsola};
    float pitch_semitones{0.0f};     // Decoupled pitch shift in semitones

    float offset_bars{0.0f};         // Non-destructive slip offset in bars
    uint32_t start_offset_frames{0}; // Start offset in frames inside underlying audio asset

    // Per-Clip Fade Curves
    float fade_in_bars{0.0f};
    float fade_out_bars{0.0f};
    sampling::FadeShape fade_in_shape{sampling::FadeShape::Linear};
    sampling::FadeShape fade_out_shape{sampling::FadeShape::Linear};
    float fade_in_tension{0.0f};
    float fade_out_tension{0.0f};

    // Auto-Crossfade / Crossover State
    bool is_auto_crossfade_in{false};
    bool is_auto_crossfade_out{false};

    [[nodiscard]] inline float end_bar() const noexcept {
        return start_bar + len_bars;
    }

    [[nodiscard]] inline bool contains_bar(float bar) const noexcept {
        return bar >= start_bar && bar < (start_bar + len_bars);
    }

    [[nodiscard]] inline bool overlaps(const ArrangerClipInstance& other) const noexcept {
        return (start_bar < other.end_bar()) && (other.start_bar < end_bar());
    }

    // Sample-accurate gain envelope evaluation at a specific bar
    [[nodiscard]] inline float evaluate_gain_at_bar(float bar) const noexcept {
        if (bar < start_bar || bar >= end_bar()) return 0.0f;
        float rel_bar = bar - start_bar;
        float gain = 1.0f;

        // Fade In
        if (fade_in_bars > 1e-5f && rel_bar < fade_in_bars) {
            float u = std::clamp(rel_bar / fade_in_bars, 0.0f, 1.0f);
            switch (fade_in_shape) {
                case sampling::FadeShape::Linear:
                    gain *= u;
                    break;
                case sampling::FadeShape::Exponential:
                    gain *= std::pow(u, 2.8f);
                    break;
                case sampling::FadeShape::Logarithmic:
                    gain *= 1.0f - std::pow(1.0f - u, 2.8f);
                    break;
                case sampling::FadeShape::SCurve:
                    gain *= u * u * (3.0f - 2.0f * u);
                    break;
                case sampling::FadeShape::EqualPower:
                    gain *= std::sin(static_cast<float>(M_PI_2) * u);
                    break;
            }
        }

        // Fade Out
        float rem_bar = end_bar() - bar;
        if (fade_out_bars > 1e-5f && rem_bar < fade_out_bars) {
            float u = std::clamp(rem_bar / fade_out_bars, 0.0f, 1.0f);
            switch (fade_out_shape) {
                case sampling::FadeShape::Linear:
                    gain *= u;
                    break;
                case sampling::FadeShape::Exponential:
                    gain *= std::pow(u, 2.8f);
                    break;
                case sampling::FadeShape::Logarithmic:
                    gain *= 1.0f - std::pow(1.0f - u, 2.8f);
                    break;
                case sampling::FadeShape::SCurve:
                    gain *= u * u * (3.0f - 2.0f * u);
                    break;
                case sampling::FadeShape::EqualPower:
                    gain *= std::sin(static_cast<float>(M_PI_2) * u);
                    break;
            }
        }

        return gain;
    }

    // Warp Pins for Non-Linear Elastic Audio & Groove Quantization
    std::vector<WarpPin> warp_pins{};
    std::vector<analysis::TransientSpan> detected_transients{};

    // Detect transients using the 3-band complementary filterbank
    void detect_transients(float sensitivity = 0.5f) {
        auto target = source_clip ? source_clip : clip;
        if (!target || target->num_frames() == 0) return;
        analysis::TransientDetector detector(target->sample_rate());
        const float* ch0 = target->channel(0);
        const float* ch1 = (target->num_channels() > 1) ? target->channel(1) : ch0;
        auto res = detector.analyze(ch0, ch1, target->num_frames(), sensitivity);
        detected_transients = std::move(res.transient_spans);
    }

    // Sort warp pins strictly by source_frame, secondary pinned_bar
    void sort_warp_pins() noexcept {
        std::sort(warp_pins.begin(), warp_pins.end(), [](const WarpPin& a, const WarpPin& b) {
            if (a.source_frame != b.source_frame) {
                return a.source_frame < b.source_frame;
            }
            return a.pinned_bar < b.pinned_bar;
        });
    }

    // Add a new warp pin binding source_frame to pinned_bar
    uint32_t add_warp_pin(uint32_t source_frame, float pinned_bar) {
        pinned_bar = std::clamp(pinned_bar, 0.001f, std::max(0.002f, len_bars - 0.001f));
        uint32_t new_id = 1;
        for (const auto& p : warp_pins) {
            if (p.id >= new_id) new_id = p.id + 1;
        }
        warp_pins.push_back(WarpPin{
            .id = new_id,
            .source_frame = source_frame,
            .pinned_bar = pinned_bar
        });
        sort_warp_pins();
        return new_id;
    }

    // Remove an existing warp pin by ID
    bool remove_warp_pin(uint32_t pin_id) {
        auto it = std::find_if(warp_pins.begin(), warp_pins.end(), [pin_id](const WarpPin& p) {
            return p.id == pin_id;
        });
        if (it == warp_pins.end()) return false;
        warp_pins.erase(it);
        return true;
    }

    // Move a warp pin while preserving strict monotonicity with its neighbors
    bool move_warp_pin(uint32_t pin_id, float new_pinned_bar) {
        auto it = std::find_if(warp_pins.begin(), warp_pins.end(), [pin_id](const WarpPin& p) {
            return p.id == pin_id;
        });
        if (it == warp_pins.end()) return false;

        size_t idx = static_cast<size_t>(std::distance(warp_pins.begin(), it));
        float min_bar = (idx > 0) ? (warp_pins[idx - 1].pinned_bar + 0.005f) : 0.005f;
        float max_bar = (idx + 1 < warp_pins.size()) ? (warp_pins[idx + 1].pinned_bar - 0.005f) : (len_bars - 0.005f);

        if (min_bar > max_bar) return false;
        it->pinned_bar = std::clamp(new_pinned_bar, min_bar, max_bar);
        return true;
    }

    void clear_warp_pins() noexcept {
        warp_pins.clear();
    }

    [[nodiscard]] WarpPin* find_warp_pin(uint32_t pin_id) noexcept {
        for (auto& p : warp_pins) {
            if (p.id == pin_id) return &p;
        }
        return nullptr;
    }

    [[nodiscard]] const WarpPin* find_warp_pin(uint32_t pin_id) const noexcept {
        for (const auto& p : warp_pins) {
            if (p.id == pin_id) return &p;
        }
        return nullptr;
    }

    // Real-Time Piecewise Time-Warping: Evaluates exact source frame at rel_bar
    // Zero allocations, O(M) lookup, C0-continuous Hermite compatible.
    [[nodiscard]] inline double evaluate_warped_frame(double rel_bar, uint32_t total_frames) const noexcept {
        if (warp_pins.empty() || total_frames == 0) {
            double base_dur = (orig_len_bars > 0.01f ? orig_len_bars : len_bars);
            return static_cast<double>(start_offset_frames) + (rel_bar / base_dur) * static_cast<double>(total_frames);
        }

        double p0_bar = 0.0;
        double p0_frame = static_cast<double>(start_offset_frames);
        double p1_bar = static_cast<double>(len_bars);
        double p1_frame = static_cast<double>(total_frames);

        if (rel_bar <= static_cast<double>(warp_pins.front().pinned_bar)) {
            p1_bar = static_cast<double>(warp_pins.front().pinned_bar);
            p1_frame = static_cast<double>(warp_pins.front().source_frame);
        } else if (rel_bar >= static_cast<double>(warp_pins.back().pinned_bar)) {
            p0_bar = static_cast<double>(warp_pins.back().pinned_bar);
            p0_frame = static_cast<double>(warp_pins.back().source_frame);
        } else {
            for (size_t k = 0; k + 1 < warp_pins.size(); ++k) {
                if (rel_bar >= static_cast<double>(warp_pins[k].pinned_bar) &&
                    rel_bar <= static_cast<double>(warp_pins[k + 1].pinned_bar)) {
                    p0_bar = static_cast<double>(warp_pins[k].pinned_bar);
                    p0_frame = static_cast<double>(warp_pins[k].source_frame);
                    p1_bar = static_cast<double>(warp_pins[k + 1].pinned_bar);
                    p1_frame = static_cast<double>(warp_pins[k + 1].source_frame);
                    break;
                }
            }
        }

        double span_bar = p1_bar - p0_bar;
        if (span_bar <= 1e-6) return p0_frame;
        double u = (rel_bar - p0_bar) / span_bar;
        u = std::clamp(u, 0.0, 1.0);
        return p0_frame + u * (p1_frame - p0_frame);
    }

    // Reciprocal mapping: Maps source audio frame to current warped timeline bar
    [[nodiscard]] inline double frame_to_bar(uint32_t frame, uint32_t total_frames) const noexcept {
        if (warp_pins.empty() || total_frames == 0) {
            double base_dur = (orig_len_bars > 0.01f ? orig_len_bars : len_bars);
            return (static_cast<double>(frame) / static_cast<double>(total_frames)) * base_dur;
        }

        double p0_frame = 0.0;
        double p0_bar = 0.0;
        double p1_frame = static_cast<double>(total_frames);
        double p1_bar = static_cast<double>(len_bars);

        if (frame <= warp_pins.front().source_frame) {
            p1_frame = static_cast<double>(warp_pins.front().source_frame);
            p1_bar = static_cast<double>(warp_pins.front().pinned_bar);
        } else if (frame >= warp_pins.back().source_frame) {
            p0_frame = static_cast<double>(warp_pins.back().source_frame);
            p0_bar = static_cast<double>(warp_pins.back().pinned_bar);
        } else {
            for (size_t k = 0; k + 1 < warp_pins.size(); ++k) {
                if (frame >= warp_pins[k].source_frame && frame <= warp_pins[k + 1].source_frame) {
                    p0_frame = static_cast<double>(warp_pins[k].source_frame);
                    p0_bar = static_cast<double>(warp_pins[k].pinned_bar);
                    p1_frame = static_cast<double>(warp_pins[k + 1].source_frame);
                    p1_bar = static_cast<double>(warp_pins[k + 1].pinned_bar);
                    break;
                }
            }
        }

        double span_f = p1_frame - p0_frame;
        if (span_f <= 1e-6) return p0_bar;
        double u = (static_cast<double>(frame) - p0_frame) / span_f;
        u = std::clamp(u, 0.0, 1.0);
        return p0_bar + u * (p1_bar - p0_bar);
    }

    // Groove Quantization: Automatically snaps detected transient peaks to nearest musical grid
    void quantize_transients_to_grid(float grid_subdivision_bars = 0.0625f) {
        if (detected_transients.empty()) {
            detect_transients();
        }
        auto target = clip ? clip : source_clip;
        if (detected_transients.empty() || !target || target->num_frames() == 0) return;

        clear_warp_pins();
        const uint32_t num_f = target->num_frames();
        const float base_bars = (orig_len_bars > 0.01f ? orig_len_bars : len_bars);

        for (const auto& span : detected_transients) {
            if (span.peak_frame < 32 || span.peak_frame >= num_f - 32) continue;
            float natural_bar = (static_cast<float>(span.peak_frame) / static_cast<float>(num_f)) * base_bars;
            float quantized_bar = std::round(natural_bar / grid_subdivision_bars) * grid_subdivision_bars;
            quantized_bar = std::clamp(quantized_bar, 0.02f, len_bars - 0.02f);
            add_warp_pin(span.peak_frame, quantized_bar);
        }
    }

    bool operator==(const ArrangerClipInstance& o) const noexcept {
        return id == o.id &&
               name == o.name &&
               clip == o.clip &&
               source_clip == o.source_clip &&
               type == o.type &&
               pattern_index == o.pattern_index &&
               start_bar == o.start_bar &&
               len_bars == o.len_bars &&
               base_len_bars == o.base_len_bars &&
               orig_len_bars == o.orig_len_bars &&
               stretch_ratio == o.stretch_ratio &&
               stretch_algo == o.stretch_algo &&
               pitch_semitones == o.pitch_semitones &&
               offset_bars == o.offset_bars &&
               start_offset_frames == o.start_offset_frames &&
               fade_in_bars == o.fade_in_bars &&
               fade_out_bars == o.fade_out_bars &&
               fade_in_shape == o.fade_in_shape &&
               fade_out_shape == o.fade_out_shape &&
               fade_in_tension == o.fade_in_tension &&
               fade_out_tension == o.fade_out_tension &&
               warp_pins == o.warp_pins;
    }
};

// ============================================================================
// ArrangerTrack: Multi-Clip Arrangement Timeline per Track Lane
// Real-time safe audio playback via lock-free RCU atomic snapshots,
// non-destructive razor split tool, slip-editing, and auto-crossfade logic.
// ============================================================================
class ArrangerTrack {
public:
    using ArrangerSnapshot = std::vector<ArrangerClipInstance>;

    ArrangerTrack() {
        publish_snapshot();
    }

    // ------------------------------------------------------------------------
    // Main-Thread Editing API
    // ------------------------------------------------------------------------

    uint32_t add_clip(ArrangerClipInstance clip) {
        if (clip.id == 0) {
            clip.id = m_next_clip_id++;
        } else if (clip.id >= m_next_clip_id) {
            m_next_clip_id = clip.id + 1;
        }

        if (!clip.source_clip && clip.clip) {
            clip.source_clip = clip.clip;
            clip.base_len_bars = clip.len_bars;
            clip.orig_len_bars = clip.len_bars;
            clip.stretch_ratio = 1.0f;
        }

        if (clip.detected_transients.empty() && clip.clip) {
            clip.detect_transients();
        }

        m_clips.push_back(std::move(clip));
        update_auto_crossfades();
        publish_snapshot();
        return m_clips.back().id;
    }

    // Warp Pin & Groove Editing API
    uint32_t add_clip_warp_pin(uint32_t clip_id, uint32_t source_frame, float pinned_bar) {
        auto* c = find_clip(clip_id);
        if (!c) return 0;
        uint32_t pid = c->add_warp_pin(source_frame, pinned_bar);
        publish_snapshot();
        return pid;
    }

    bool move_clip_warp_pin(uint32_t clip_id, uint32_t pin_id, float new_pinned_bar) {
        auto* c = find_clip(clip_id);
        if (!c) return false;
        bool res = c->move_warp_pin(pin_id, new_pinned_bar);
        if (res) publish_snapshot();
        return res;
    }

    bool remove_clip_warp_pin(uint32_t clip_id, uint32_t pin_id) {
        auto* c = find_clip(clip_id);
        if (!c) return false;
        bool res = c->remove_warp_pin(pin_id);
        if (res) publish_snapshot();
        return res;
    }

    bool clear_clip_warp_pins(uint32_t clip_id) {
        auto* c = find_clip(clip_id);
        if (!c) return false;
        c->clear_warp_pins();
        publish_snapshot();
        return true;
    }

    bool quantize_clip_transients(uint32_t clip_id, float grid = 0.0625f) {
        auto* c = find_clip(clip_id);
        if (!c) return false;
        c->quantize_transients_to_grid(grid);
        publish_snapshot();
        return true;
    }

    bool detect_clip_transients(uint32_t clip_id, float sensitivity = 0.5f) {
        auto* c = find_clip(clip_id);
        if (!c) return false;
        c->detect_transients(sensitivity);
        publish_snapshot();
        return true;
    }

    bool remove_clip(uint32_t clip_id) {
        auto it = std::find_if(m_clips.begin(), m_clips.end(), [clip_id](const ArrangerClipInstance& c) {
            return c.id == clip_id;
        });
        if (it == m_clips.end()) return false;
        m_clips.erase(it);
        update_auto_crossfades();
        publish_snapshot();
        return true;
    }

    void clear_clips() {
        m_clips.clear();
        publish_snapshot();
    }

    [[nodiscard]] size_t num_clips() const noexcept {
        return m_clips.size();
    }

    [[nodiscard]] const std::vector<ArrangerClipInstance>& clips() const noexcept {
        return m_clips;
    }

    [[nodiscard]] std::vector<ArrangerClipInstance>& clips() noexcept {
        return m_clips;
    }

    [[nodiscard]] ArrangerClipInstance* find_clip(uint32_t clip_id) noexcept {
        for (auto& c : m_clips) {
            if (c.id == clip_id) return &c;
        }
        return nullptr;
    }

    [[nodiscard]] const ArrangerClipInstance* find_clip(uint32_t clip_id) const noexcept {
        for (const auto& c : m_clips) {
            if (c.id == clip_id) return &c;
        }
        return nullptr;
    }

    // Time-Stretching & Pitch Shifting API:
    // Stretches a clip to new_len_bars using WSOLA, Vinyl, or Sovereign ODE
    // with zero generational loss (always derived from pristine source_clip).
    bool stretch_clip(uint32_t clip_id, float new_len_bars,
                      dsp::PitchAlgorithm algo,
                      float semitones = 0.0f) {
        auto it = std::find_if(m_clips.begin(), m_clips.end(), [clip_id](const ArrangerClipInstance& c) {
            return c.id == clip_id;
        });
        if (it == m_clips.end() || !it->clip) return false;

        if (!it->source_clip) {
            it->source_clip = it->clip;
            it->base_len_bars = it->len_bars;
            it->orig_len_bars = it->len_bars;
            it->stretch_ratio = 1.0f;
        }

        if (new_len_bars < 0.1f) new_len_bars = 0.1f;
        float base_len = (it->base_len_bars > 0.01f) ? it->base_len_bars : it->len_bars;

        float stretch_factor = new_len_bars / base_len;
        stretch_factor = std::clamp(stretch_factor, 0.1f, 10.0f);
        new_len_bars = base_len * stretch_factor;

        std::shared_ptr<sampling::AudioClip> stretched = nullptr;
        if (std::abs(stretch_factor - 1.0f) < 1e-4f && std::abs(semitones) < 1e-4f) {
            stretched = it->source_clip;
        } else {
            stretched = dsp::PitchTimeStretcher::process(*it->source_clip, algo, semitones, stretch_factor);
        }

        if (!stretched) {
            return false;
        }

        it->clip = stretched;
        it->len_bars = new_len_bars;
        it->orig_len_bars = new_len_bars;
        it->stretch_ratio = stretch_factor;
        it->stretch_algo = algo;
        it->pitch_semitones = semitones;

        it->fade_in_bars = std::min(it->fade_in_bars, it->len_bars);
        it->fade_out_bars = std::min(it->fade_out_bars, it->len_bars);

        update_auto_crossfades();
        publish_snapshot();
        return true;
    }

    bool stretch_clip(uint32_t clip_id, float new_len_bars) {
        const auto* c = find_clip(clip_id);
        if (!c) return false;
        return stretch_clip(clip_id, new_len_bars, c->stretch_algo, c->pitch_semitones);
    }

    bool set_clip_pitch(uint32_t clip_id, float semitones) {
        const auto* c = find_clip(clip_id);
        if (!c) return false;
        return stretch_clip(clip_id, c->len_bars, c->stretch_algo, semitones);
    }

    bool set_clip_stretch_algo(uint32_t clip_id, dsp::PitchAlgorithm algo) {
        const auto* c = find_clip(clip_id);
        if (!c) return false;
        return stretch_clip(clip_id, c->len_bars, algo, c->pitch_semitones);
    }

    // Non-destructive Razor / Split Tool:
    // Splits a clip at split_bar into two independent clip instances with
    // continuous start_offset_frames and accurate bar boundaries.
    std::pair<uint32_t, uint32_t> split_clip_at_bar(uint32_t clip_id, float split_bar,
                                                    double session_bpm = 120.0,
                                                    uint32_t sample_rate = 48000) {
        auto it = std::find_if(m_clips.begin(), m_clips.end(), [clip_id](const ArrangerClipInstance& c) {
            return c.id == clip_id;
        });
        if (it == m_clips.end()) return {0, 0};

        const ArrangerClipInstance orig = *it;
        if (split_bar <= orig.start_bar + 0.001f || split_bar >= orig.end_bar() - 0.001f) {
            return {0, 0};
        }

        const float len1 = split_bar - orig.start_bar;
        const float len2 = orig.end_bar() - split_bar;

        const double clip_bpm = (orig.clip && orig.clip->bpm() > 10.0) ? orig.clip->bpm() : session_bpm;
        const uint32_t clip_sr = (orig.clip && orig.clip->sample_rate() > 0) ? orig.clip->sample_rate() : sample_rate;
        const uint32_t num_f = orig.clip ? orig.clip->num_frames() : 0;
        uint32_t delta_frames = 0;
        if (std::abs(orig.stretch_ratio - 1.0f) < 1e-4f && std::abs(orig.pitch_semitones) < 1e-4f) {
            const double sec_len1 = (static_cast<double>(len1) * 4.0 / clip_bpm) * 60.0;
            delta_frames = static_cast<uint32_t>(std::round(sec_len1 * static_cast<double>(clip_sr)));
        } else {
            const double base_dur = (orig.orig_len_bars > 0.01f ? orig.orig_len_bars : orig.len_bars);
            delta_frames = static_cast<uint32_t>(std::round((static_cast<double>(len1) / base_dur) * static_cast<double>(num_f)));
        }
        uint32_t new_offset_f = orig.start_offset_frames + delta_frames;
        if (num_f > 0) new_offset_f %= num_f;

        ArrangerClipInstance c1 = orig;
        c1.len_bars = len1;
        c1.fade_in_bars = std::min(orig.fade_in_bars, len1);
        c1.fade_out_bars = 0.0f;
        c1.is_auto_crossfade_out = false;

        ArrangerClipInstance c2;
        c2.id = m_next_clip_id++;
        c2.name = orig.name + "_split";
        c2.clip = orig.clip;
        c2.source_clip = orig.source_clip ? orig.source_clip : orig.clip;
        c2.type = orig.type;
        c2.pattern_index = orig.pattern_index;
        c2.start_bar = split_bar;
        c2.len_bars = len2;
        c2.base_len_bars = orig.base_len_bars;
        c2.orig_len_bars = orig.orig_len_bars;
        c2.stretch_ratio = orig.stretch_ratio;
        c2.stretch_algo = orig.stretch_algo;
        c2.pitch_semitones = orig.pitch_semitones;
        c2.offset_bars = orig.offset_bars + len1;
        c2.start_offset_frames = new_offset_f;
        c2.fade_in_bars = 0.0f;
        c2.fade_out_bars = std::min(orig.fade_out_bars, len2);
        c2.fade_in_shape = orig.fade_in_shape;
        c2.fade_out_shape = orig.fade_out_shape;
        c2.is_auto_crossfade_in = false;
        c2.is_auto_crossfade_out = orig.is_auto_crossfade_out;
        // Distribute warp pins across split boundary
        c1.warp_pins.clear();
        c2.warp_pins.clear();
        for (const auto& pin : orig.warp_pins) {
            if (pin.pinned_bar < len1 - 0.005f) {
                c1.warp_pins.push_back(pin);
            } else if (pin.pinned_bar > len1 + 0.005f) {
                WarpPin p2 = pin;
                p2.pinned_bar = pin.pinned_bar - len1;
                c2.warp_pins.push_back(p2);
            }
        }
        c1.sort_warp_pins();
        c2.sort_warp_pins();
        c1.detect_transients();
        c2.detect_transients();

        *it = c1;
        m_clips.insert(it + 1, c2);

        update_auto_crossfades();
        publish_snapshot();
        return {c1.id, c2.id};
    }

    // Recalculate auto-crossfades for overlapping clips
    void update_auto_crossfades() {
        if (m_clips.size() < 2) return;
        std::sort(m_clips.begin(), m_clips.end(), [](const ArrangerClipInstance& a, const ArrangerClipInstance& b) {
            return a.start_bar < b.start_bar;
        });

        for (size_t i = 0; i + 1 < m_clips.size(); ++i) {
            auto& c_left = m_clips[i];
            auto& c_right = m_clips[i + 1];

            float overlap = c_left.end_bar() - c_right.start_bar;
            if (overlap > 0.005f) {
                float max_x = std::min({overlap, c_left.len_bars, c_right.len_bars});
                c_left.fade_out_bars = max_x;
                c_left.fade_out_shape = sampling::FadeShape::EqualPower;
                c_left.is_auto_crossfade_out = true;

                c_right.fade_in_bars = max_x;
                c_right.fade_in_shape = sampling::FadeShape::EqualPower;
                c_right.is_auto_crossfade_in = true;
            } else {
                if (c_left.is_auto_crossfade_out) {
                    c_left.fade_out_bars = 0.0f;
                    c_left.is_auto_crossfade_out = false;
                }
                if (c_right.is_auto_crossfade_in) {
                    c_right.fade_in_bars = 0.0f;
                    c_right.is_auto_crossfade_in = false;
                }
            }
        }
    }

    // Slip-editing: Adjust start offset within the underlying audio asset
    bool slip_edit_clip(uint32_t clip_id, float delta_bars, double session_bpm = 120.0) {
        auto it = std::find_if(m_clips.begin(), m_clips.end(), [clip_id](const ArrangerClipInstance& c) {
            return c.id == clip_id;
        });
        if (it == m_clips.end()) return false;

        it->offset_bars = std::max(0.0f, it->offset_bars + delta_bars);
        const double clip_bpm = (it->clip && it->clip->bpm() > 10.0) ? it->clip->bpm() : session_bpm;
        const uint32_t clip_sr = it->clip ? it->clip->sample_rate() : 48000;
        const uint32_t num_f = it->clip ? it->clip->num_frames() : 0;
        if (std::abs(it->stretch_ratio - 1.0f) < 1e-4f && std::abs(it->pitch_semitones) < 1e-4f) {
            const double dur_sec = (static_cast<double>(it->offset_bars) * 4.0 / clip_bpm) * 60.0;
            it->start_offset_frames = static_cast<uint32_t>(dur_sec * static_cast<double>(clip_sr));
        } else if (num_f > 0) {
            const double base_dur = (it->orig_len_bars > 0.01f ? it->orig_len_bars : it->len_bars);
            it->start_offset_frames = static_cast<uint32_t>(std::round((static_cast<double>(it->offset_bars) / base_dur) * static_cast<double>(num_f)));
        }
        if (num_f > 0) {
            it->start_offset_frames %= num_f;
        }
        publish_snapshot();
        return true;
    }

    // Publish RCU snapshot for lock-free audio thread rendering
    void publish_snapshot() {
        auto new_snap = std::make_shared<ArrangerSnapshot>(m_clips);
        std::atomic_store_explicit(&m_snapshot, new_snap, std::memory_order_release);
    }

    // ------------------------------------------------------------------------
    // Audio-Thread Real-Time Rendering (Zero Allocation)
    // ------------------------------------------------------------------------

    void render(Sample* dst_l, Sample* dst_r, uint32_t frames,
                const clock::TimelineClock& clock) noexcept {
        const auto musical_pos = clock.position_snapshot();
        render(dst_l, dst_r, frames, clock.sample_rate(), clock.bpm(),
               clock.is_playing(), musical_pos.total_beats, clock.is_scrubbing());
    }

    void render(Sample* dst_l, Sample* dst_r, uint32_t frames,
                uint32_t session_sr, double session_bpm, bool is_playing,
                double transport_total_beats, bool is_scrubbing = false) noexcept {
        if (!dst_l || !dst_r || frames == 0) return;
        if (!is_playing && !is_scrubbing) {
            std::memset(dst_l, 0, frames * sizeof(Sample));
            std::memset(dst_r, 0, frames * sizeof(Sample));
            return;
        }

        auto snap = std::atomic_load_explicit(&m_snapshot, std::memory_order_acquire);
        if (!snap || snap->empty()) {
            std::memset(dst_l, 0, frames * sizeof(Sample));
            std::memset(dst_r, 0, frames * sizeof(Sample));
            return;
        }

        std::memset(dst_l, 0, frames * sizeof(Sample));
        std::memset(dst_r, 0, frames * sizeof(Sample));

        const double beats_per_sec = (session_bpm > 1.0) ? (session_bpm / 60.0) : 2.0;
        const double beats_per_sample = beats_per_sec / static_cast<double>(session_sr > 0 ? session_sr : 48000);
        const double bars_per_sample = beats_per_sample * 0.25;

        const double start_bar = transport_total_beats * 0.25;
        const double end_bar = start_bar + bars_per_sample * static_cast<double>(frames);

        for (const auto& c : *snap) {
            if (c.type == ArrangerClipType::Pattern) continue;
            if (!c.clip || c.len_bars <= 0.0f) continue;
            if (c.end_bar() <= static_cast<float>(start_bar) || c.start_bar >= static_cast<float>(end_bar)) {
                continue;
            }

            const uint32_t num_f = c.clip->num_frames();
            if (num_f == 0) continue;
            const float* src_l = c.clip->channel(0);
            const float* src_r = (c.clip->num_channels() > 1) ? c.clip->channel(1) : src_l;
            if (!src_l) continue;

            const double clip_bpm = (c.clip->bpm() > 10.0) ? c.clip->bpm() : session_bpm;
            const double clip_sr = (c.clip->sample_rate() > 0) ? static_cast<double>(c.clip->sample_rate()) : static_cast<double>(session_sr);

            for (uint32_t i = 0; i < frames; ++i) {
                double cur_bar = start_bar + bars_per_sample * static_cast<double>(i);
                if (cur_bar >= c.start_bar && cur_bar < c.end_bar()) {
                    double rel_bar = cur_bar - c.start_bar;
                    double playhead_f = 0.0;
                    if (!c.warp_pins.empty()) {
                        playhead_f = c.evaluate_warped_frame(rel_bar, num_f);
                    } else if (std::abs(c.stretch_ratio - 1.0f) < 1e-4f && std::abs(c.pitch_semitones) < 1e-4f) {
                        double rel_sec = (rel_bar * 4.0 / clip_bpm) * 60.0;
                        playhead_f = static_cast<double>(c.start_offset_frames) + rel_sec * clip_sr;
                    } else {
                        double base_dur = (c.orig_len_bars > 0.01f ? c.orig_len_bars : c.len_bars);
                        playhead_f = static_cast<double>(c.start_offset_frames) + (rel_bar / base_dur) * static_cast<double>(num_f);
                    }

                    playhead_f = std::fmod(playhead_f, static_cast<double>(num_f));
                    if (playhead_f < 0.0) playhead_f += static_cast<double>(num_f);

                    float s_l = dsp::sample_hermite_wrapped(src_l, playhead_f, num_f);
                    float s_r = dsp::sample_hermite_wrapped(src_r, playhead_f, num_f);

                    float gain = c.evaluate_gain_at_bar(static_cast<float>(cur_bar));
                    dst_l[i] += s_l * gain;
                    dst_r[i] += s_r * gain;
                }
            }
        }

        // 2. Render Pattern Clips via Associated StepSequencer
        if (m_associated_sequencer) {
            TimelinePatternClip pat_clips[16];
            size_t num_pat_clips = 0;
            for (const auto& c : *snap) {
                if (c.type == ArrangerClipType::Pattern && num_pat_clips < 16) {
                    pat_clips[num_pat_clips++] = TimelinePatternClip{
                        .pattern_index = c.pattern_index,
                        .start_bar = c.start_bar,
                        .len_bars = c.len_bars,
                        .offset_bars = c.offset_bars,
                        .gain = c.evaluate_gain_at_bar(static_cast<float>(start_bar))
                    };
                }
            }
            if (num_pat_clips > 0 || m_associated_sequencer->is_voice_active()) {
                m_associated_sequencer->render_timeline_clips(dst_l, dst_r, frames,
                                                              session_sr, session_bpm, is_playing,
                                                              transport_total_beats,
                                                              pat_clips, num_pat_clips);
            }
        }
    }

    [[nodiscard]] bool is_empty() const noexcept {
        return m_clips.empty();
    }

    void set_enabled(bool enabled) noexcept {
        m_enabled.store(enabled, std::memory_order_relaxed);
    }

    [[nodiscard]] bool is_enabled() const noexcept {
        return m_enabled.load(std::memory_order_relaxed);
    }

    void set_associated_sequencer(StepSequencer* seq) noexcept {
        m_associated_sequencer = seq;
    }
    [[nodiscard]] StepSequencer* associated_sequencer() noexcept { return m_associated_sequencer; }
    [[nodiscard]] const StepSequencer* associated_sequencer() const noexcept { return m_associated_sequencer; }

private:
    std::vector<ArrangerClipInstance> m_clips{};
    uint32_t m_next_clip_id{1};
    std::atomic<bool> m_enabled{true};
    std::atomic<std::shared_ptr<ArrangerSnapshot>> m_snapshot{nullptr};
    StepSequencer* m_associated_sequencer{nullptr};
};

} // namespace audio_core::sequencer
