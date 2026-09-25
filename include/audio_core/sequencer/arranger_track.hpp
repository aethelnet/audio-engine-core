#pragma once

#include "audio_core/types.hpp"
#include "audio_core/clock/timeline_clock.hpp"
#include "audio_core/sampling/audio_clip.hpp"
#include "audio_core/dsp/resampler.hpp"

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
// ArrangerClipInstance: Timeline Event Container
// Supports per-clip trimming, non-destructive slip-editing, 5 fade shapes,
// and automatic equal-power crossfading across overlapping clip intervals.
// ============================================================================
struct ArrangerClipInstance {
    uint32_t id{0};
    std::string name{""};
    std::shared_ptr<sampling::AudioClip> clip{nullptr};

    float start_bar{0.0f};           // Absolute timeline position in musical bars
    float len_bars{4.0f};            // Timeline duration in musical bars
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

        m_clips.push_back(std::move(clip));
        update_auto_crossfades();
        publish_snapshot();
        return m_clips.back().id;
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
        const double sec_len1 = (static_cast<double>(len1) * 4.0 / clip_bpm) * 60.0;
        const uint32_t delta_frames = static_cast<uint32_t>(std::round(sec_len1 * static_cast<double>(clip_sr)));
        const uint32_t num_f = orig.clip ? orig.clip->num_frames() : 0;
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
        c2.start_bar = split_bar;
        c2.len_bars = len2;
        c2.offset_bars = orig.offset_bars + len1;
        c2.start_offset_frames = new_offset_f;
        c2.fade_in_bars = 0.0f;
        c2.fade_out_bars = std::min(orig.fade_out_bars, len2);
        c2.fade_in_shape = orig.fade_in_shape;
        c2.fade_out_shape = orig.fade_out_shape;
        c2.is_auto_crossfade_in = false;
        c2.is_auto_crossfade_out = orig.is_auto_crossfade_out;

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
        const double dur_sec = (static_cast<double>(it->offset_bars) * 4.0 / clip_bpm) * 60.0;
        it->start_offset_frames = static_cast<uint32_t>(dur_sec * static_cast<double>(clip_sr));
        if (it->clip && it->clip->num_frames() > 0) {
            it->start_offset_frames %= it->clip->num_frames();
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
                    double rel_sec = (rel_bar * 4.0 / clip_bpm) * 60.0;
                    double playhead_f = static_cast<double>(c.start_offset_frames) + rel_sec * clip_sr;

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

private:
    std::vector<ArrangerClipInstance> m_clips{};
    uint32_t m_next_clip_id{1};
    std::atomic<bool> m_enabled{true};
    std::atomic<std::shared_ptr<ArrangerSnapshot>> m_snapshot{nullptr};
};

} // namespace audio_core::sequencer
