#pragma once

#include "audio_core/sampling/audio_clip.hpp"
#include "audio_core/clock/timeline_clock.hpp"
#include "audio_core/types.hpp"
#include "audio_core/ring_buffer.hpp"

#include <array>
#include <vector>
#include <atomic>
#include <memory>
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace audio_core::sequencer {

enum class StepSubdivision : uint8_t {
    Eighth = 0,        // 2 steps per beat
    Sixteenth,         // 4 steps per beat (default)
    ThirtySecond,      // 8 steps per beat
    EighthTriplet,     // 3 steps per beat
    SixteenthTriplet   // 6 steps per beat
};

enum class PatternSwitchMode : uint8_t {
    Immediate = 0,     // Switch pattern instantly mid-bar / mid-beat
    BeatQuantized,     // Switch at next beat boundary
    BarQuantized       // Switch at next bar downbeat (default)
};

struct StepTrigger {
    bool active{false};
    uint32_t slice_id{0};
    float velocity{1.0f};      // [0.0f, 1.0f]
    float pitch_ratio{1.0f};   // 1.0 = normal, 0.5 = octave down, 2.0 = octave up
    uint8_t probability{100};  // [0..100%]
    bool reverse{false};
};

struct Pattern {
    static constexpr size_t kMaxSteps = 64;
    uint32_t num_steps{16};
    StepSubdivision subdivision{StepSubdivision::Sixteenth};
    std::array<StepTrigger, kMaxSteps> steps{};

    void clear() noexcept {
        for (auto& s : steps) {
            s = StepTrigger{};
        }
    }

    void set_step(size_t idx, uint32_t slice_id, float velocity = 1.0f,
                  float pitch_ratio = 1.0f, uint8_t prob = 100, bool reverse = false) noexcept {
        if (idx < kMaxSteps) {
            steps[idx] = StepTrigger{
                .active = true,
                .slice_id = slice_id,
                .velocity = std::clamp(velocity, 0.0f, 1.0f),
                .pitch_ratio = std::clamp(pitch_ratio, 0.125f, 8.0f),
                .probability = std::min<uint8_t>(prob, 100),
                .reverse = reverse
            };
        }
    }

    // Automatically fill steps linearly with slices [0 .. num_slices-1]
    void fill_linear_slices(uint32_t num_slices, float velocity = 1.0f) noexcept {
        clear();
        uint32_t count = std::min<uint32_t>(num_slices, num_steps);
        for (uint32_t i = 0; i < count; ++i) {
            set_step(i, i, velocity);
        }
    }

    std::string name{"Pattern"};

    [[nodiscard]] bool is_step_active(size_t idx) const noexcept {
        return (idx < kMaxSteps) ? steps[idx].active : false;
    }

    void toggle_step(size_t idx, uint32_t slice_id = 0, float velocity = 1.0f) noexcept {
        if (idx < kMaxSteps) {
            steps[idx].active = !steps[idx].active;
            if (steps[idx].active) {
                steps[idx].slice_id = slice_id;
                steps[idx].velocity = std::clamp(velocity, 0.0f, 1.0f);
                if (steps[idx].pitch_ratio <= 0.0f) steps[idx].pitch_ratio = 1.0f;
                if (steps[idx].probability == 0) steps[idx].probability = 100;
            }
        }
    }
};

// ============================================================================
// StepSequencer: Clock-Synchronized Slice Step-Sequencer & Pad Trigger Engine
// - 16/32/64-step grid synced to TimelineClock
// - Immediate & Quantized (Beat/Bar) pattern switching
// - Anti-click micro-fade (64 frames = ~1.33ms) choke voice crossfade
// - Lock-free manual MPC-style pad triggers
// ============================================================================
class StepSequencer {
public:
    static constexpr size_t kMaxPatterns = 16;
    static constexpr uint32_t kMicroFadeFrames = 64; // ~1.33 ms @ 48kHz for zero clicks

    struct ManualTrigger {
        uint32_t slice_id{0};
        float velocity{1.0f};
        float pitch_ratio{1.0f};
        bool reverse{false};
        uint32_t sample_offset{0};
    };

    explicit StepSequencer(std::shared_ptr<sampling::AudioClip> clip = nullptr)
        : m_clip(std::move(clip)), m_manual_queue(64) {
        for (auto& pat : m_patterns) {
            pat.clear();
        }
    }

    void set_clip(std::shared_ptr<sampling::AudioClip> clip) noexcept {
        m_clip = std::move(clip);
    }

    [[nodiscard]] const std::shared_ptr<sampling::AudioClip>& clip() const noexcept {
        return m_clip;
    }

    [[nodiscard]] Pattern& pattern(size_t index) noexcept {
        return m_patterns[index % kMaxPatterns];
    }

    [[nodiscard]] const Pattern& pattern(size_t index) const noexcept {
        return m_patterns[index % kMaxPatterns];
    }

    [[nodiscard]] uint32_t current_pattern_index() const noexcept {
        return m_current_pattern.load(std::memory_order_relaxed);
    }

    [[nodiscard]] uint32_t queued_pattern_index() const noexcept {
        return m_queued_pattern.load(std::memory_order_relaxed);
    }

    [[nodiscard]] bool has_queued_pattern() const noexcept {
        return m_has_queued_pattern.load(std::memory_order_relaxed);
    }

    [[nodiscard]] uint32_t current_step_index() const noexcept {
        return m_current_step.load(std::memory_order_relaxed);
    }

    [[nodiscard]] PatternSwitchMode switch_mode() const noexcept {
        return m_switch_mode.load(std::memory_order_relaxed);
    }

    [[nodiscard]] bool is_pattern_queued(uint32_t pat_idx) const noexcept {
        return m_has_queued_pattern.load(std::memory_order_relaxed) &&
               m_queued_pattern.load(std::memory_order_relaxed) == (pat_idx % kMaxPatterns);
    }

    // Direct / Immediate pattern switch (switches right now without waiting)
    void switch_pattern_immediate(uint32_t pattern_idx) noexcept {
        m_current_pattern.store(pattern_idx % kMaxPatterns, std::memory_order_relaxed);
        m_has_queued_pattern.store(false, std::memory_order_relaxed);
    }

    // Scheduled pattern switch (queued for next beat or bar boundary)
    void queue_pattern_switch(uint32_t pattern_idx,
                              PatternSwitchMode mode = PatternSwitchMode::BarQuantized) noexcept {
        if (mode == PatternSwitchMode::Immediate) {
            switch_pattern_immediate(pattern_idx);
            return;
        }
        m_queued_pattern.store(pattern_idx % kMaxPatterns, std::memory_order_relaxed);
        m_switch_mode.store(mode, std::memory_order_relaxed);
        m_has_queued_pattern.store(true, std::memory_order_relaxed);
    }

    // Manual Pad / MIDI trigger (lock-free dispatch)
    bool trigger_slice(uint32_t slice_id, float velocity = 1.0f,
                       float pitch_ratio = 1.0f, bool reverse = false,
                       uint32_t sample_offset = 0) noexcept {
        ManualTrigger ev{
            .slice_id = slice_id,
            .velocity = std::clamp(velocity, 0.0f, 1.0f),
            .pitch_ratio = std::clamp(pitch_ratio, 0.125f, 8.0f),
            .reverse = reverse,
            .sample_offset = sample_offset
        };
        return m_manual_queue.try_push(ev);
    }

    void stop() noexcept {
        m_primary_voice.active = false;
        m_choked_voice.active = false;
        m_current_step.store(0, std::memory_order_relaxed);
    }

    [[nodiscard]] bool is_voice_active() const noexcept {
        return m_primary_voice.active || m_choked_voice.active;
    }

    // Real-Time Render Pipeline: Lock-free, zero allocation, sample-accurate
    void render(Sample* out_l, Sample* out_r, uint32_t frames,
                const clock::TimelineClock& clock,
                const clock::BlockBoundaryEvents& boundary_events) noexcept {
        if (!out_l || !out_r || frames == 0) return;

        // Clear output block
        for (uint32_t i = 0; i < frames; ++i) {
            out_l[i] = 0.0f;
            out_r[i] = 0.0f;
        }

        if (!m_clip) return;

        // 1. Evaluate Scheduled Pattern Switches
        if (m_has_queued_pattern.load(std::memory_order_relaxed)) {
            const auto mode = m_switch_mode.load(std::memory_order_relaxed);
            if (mode == PatternSwitchMode::BarQuantized && boundary_events.has_bar_boundary) {
                m_current_pattern.store(m_queued_pattern.load(std::memory_order_relaxed), std::memory_order_relaxed);
                m_has_queued_pattern.store(false, std::memory_order_relaxed);
            } else if (mode == PatternSwitchMode::BeatQuantized && boundary_events.has_beat_boundary) {
                m_current_pattern.store(m_queued_pattern.load(std::memory_order_relaxed), std::memory_order_relaxed);
                m_has_queued_pattern.store(false, std::memory_order_relaxed);
            }
        }

        const uint32_t pat_idx = m_current_pattern.load(std::memory_order_relaxed);
        const auto& active_pattern = m_patterns[pat_idx];

        // 2. Compute Step Transitions within this block
        const double spb = clock.samples_per_beat();
        const double step_dur = calculate_step_duration(active_pattern.subdivision, spb);

        const uint64_t end_pos = clock.sample_position();
        const uint64_t start_pos = (end_pos >= frames) ? (end_pos - frames) : 0;
        const bool clock_playing = clock.is_playing();

        const uint32_t engine_sr = clock.sample_rate();
        const uint32_t clip_sr = m_clip ? m_clip->sample_rate() : engine_sr;
        const double rate_ratio = (engine_sr > 0) ? (static_cast<double>(clip_sr) / static_cast<double>(engine_sr)) : 1.0;

        // 3. Process frame-by-frame with sample-accurate step and pad triggers
        for (uint32_t i = 0; i < frames; ++i) {
            const uint64_t current_sample_pos = start_pos + i;

        // Process any queued manual pad triggers
        ManualTrigger manual_ev;
        while (m_manual_queue.try_pop(manual_ev)) {
            start_slice_voice(manual_ev.slice_id, manual_ev.velocity,
                              manual_ev.pitch_ratio, manual_ev.reverse);
        }

            // Check step grid trigger if transport is running
            if (clock_playing && step_dur > 1.0) {
                bool trigger_now = false;
                uint64_t step_index_total = 0;

                if (current_sample_pos == 0) {
                    trigger_now = true;
                    step_index_total = 0;
                } else {
                    const uint64_t prev_step = static_cast<uint64_t>(std::floor(static_cast<double>(current_sample_pos - 1) / step_dur));
                    const uint64_t curr_step = static_cast<uint64_t>(std::floor(static_cast<double>(current_sample_pos) / step_dur));
                    if (curr_step > prev_step) {
                        trigger_now = true;
                        step_index_total = curr_step;
                    }
                }

                if (trigger_now && active_pattern.num_steps > 0) {
                    const uint32_t step_idx = static_cast<uint32_t>(step_index_total % active_pattern.num_steps);
                    m_current_step.store(step_idx, std::memory_order_relaxed);
                    const auto& step = active_pattern.steps[step_idx];
                    if (step.active) {
                        bool fire = true;
                        if (step.probability < 100) {
                            // Simple deterministic pseudo-random hashing based on step position
                            uint32_t hash = static_cast<uint32_t>(step_index_total * 2654435761u);
                            fire = (hash % 100) < step.probability;
                        }
                        if (fire) {
                            start_slice_voice(step.slice_id, step.velocity, step.pitch_ratio, step.reverse);
                        }
                    }
                }
            }

            // Render active voices with micro-fade crossfade and sample rate agility
            render_sample(out_l[i], out_r[i], rate_ratio);
        }
    }

private:
    struct SliceVoice {
        bool active{false};
        uint32_t slice_id{0};
        uint32_t start_frame{0};
        uint32_t end_frame{0};
        double playhead{0.0};
        float velocity{1.0f};
        float slice_gain{1.0f};
        float pitch_ratio{1.0f};
        bool reverse{false};

        // Micro-fade parameters
        uint32_t fade_in_remaining{0};
        uint32_t fade_in_total{0};

        bool is_fading_out{false};
        uint32_t fade_out_remaining{0};
        uint32_t fade_out_total{0};
    };

    static double calculate_step_duration(StepSubdivision sub, double samples_per_beat) noexcept {
        switch (sub) {
            case StepSubdivision::Eighth:           return samples_per_beat / 2.0;
            case StepSubdivision::Sixteenth:        return samples_per_beat / 4.0;
            case StepSubdivision::ThirtySecond:     return samples_per_beat / 8.0;
            case StepSubdivision::EighthTriplet:    return samples_per_beat / 3.0;
            case StepSubdivision::SixteenthTriplet: return samples_per_beat / 6.0;
        }
        return samples_per_beat / 4.0;
    }

    void start_slice_voice(uint32_t slice_id, float velocity, float pitch_ratio, bool reverse) noexcept {
        if (!m_clip) return;
        const auto& slices = m_clip->slices();
        if (slice_id >= slices.size()) return;

        const auto& slice = slices[slice_id];
        const uint32_t slice_len = (slice.end_frame > slice.start_frame) ? (slice.end_frame - slice.start_frame) : 0;
        if (slice_len == 0) return;

        // Choke currently active primary voice with micro-fade out
        if (m_primary_voice.active) {
            m_choked_voice = m_primary_voice;
            m_choked_voice.is_fading_out = true;
            m_choked_voice.fade_out_remaining = kMicroFadeFrames;
            m_choked_voice.fade_out_total = kMicroFadeFrames;
        }

        // Initialize new primary voice with micro-fade in
        m_primary_voice.active = true;
        m_primary_voice.slice_id = slice_id;
        m_primary_voice.start_frame = slice.start_frame;
        m_primary_voice.end_frame = slice.end_frame;
        m_primary_voice.playhead = reverse ? static_cast<double>(slice_len - 1) : 0.0;
        m_primary_voice.velocity = velocity;
        m_primary_voice.slice_gain = slice.gain;
        m_primary_voice.pitch_ratio = std::clamp(pitch_ratio, 0.125f, 8.0f);
        m_primary_voice.reverse = reverse;

        m_primary_voice.fade_in_remaining = kMicroFadeFrames;
        m_primary_voice.fade_in_total = kMicroFadeFrames;
        m_primary_voice.is_fading_out = false;
    }

    inline void render_voice_frame(SliceVoice& v, float fade_gain, float& out_l, float& out_r, double rate_ratio = 1.0) noexcept {
        const uint32_t slice_len = (v.end_frame > v.start_frame) ? (v.end_frame - v.start_frame) : 0;
        if (slice_len == 0) {
            v.active = false;
            return;
        }

        if (v.playhead < 0.0 || v.playhead >= static_cast<double>(slice_len)) {
            v.active = false;
            return;
        }

        double global_pos = static_cast<double>(v.start_frame) + v.playhead;

        const float* src_l = m_clip->channel(0);
        const float* src_r = (m_clip->num_channels() > 1) ? m_clip->channel(1) : src_l;

        float sample_l = dsp::sample_hermite(src_l, global_pos, m_clip->num_frames());
        float sample_r = dsp::sample_hermite(src_r, global_pos, m_clip->num_frames());

        const float total_gain = v.velocity * v.slice_gain * fade_gain;
        out_l += sample_l * total_gain;
        out_r += sample_r * total_gain;

        const double step_advance = rate_ratio * static_cast<double>(v.pitch_ratio);

        if (v.reverse) {
            v.playhead -= step_advance;
            if (v.playhead < 0.0) {
                v.active = false;
            }
        } else {
            v.playhead += step_advance;
            if (v.playhead >= static_cast<double>(slice_len)) {
                v.active = false;
            }
        }
    }

    inline void render_sample(float& out_l, float& out_r, double rate_ratio = 1.0) noexcept {
        // Render primary voice
        if (m_primary_voice.active) {
            float in_gain = 1.0f;
            if (m_primary_voice.fade_in_remaining > 0) {
                in_gain = 1.0f - (static_cast<float>(m_primary_voice.fade_in_remaining) / static_cast<float>(m_primary_voice.fade_in_total));
                m_primary_voice.fade_in_remaining--;
            }
            render_voice_frame(m_primary_voice, in_gain, out_l, out_r, rate_ratio);
        }

        // Render choked voice (smooth fade out to 0)
        if (m_choked_voice.active) {
            if (m_choked_voice.is_fading_out && m_choked_voice.fade_out_remaining > 0) {
                float out_gain = static_cast<float>(m_choked_voice.fade_out_remaining) / static_cast<float>(m_choked_voice.fade_out_total);
                m_choked_voice.fade_out_remaining--;
                render_voice_frame(m_choked_voice, out_gain, out_l, out_r, rate_ratio);
                if (m_choked_voice.fade_out_remaining == 0) {
                    m_choked_voice.active = false;
                    m_choked_voice.is_fading_out = false;
                }
            } else {
                m_choked_voice.active = false;
            }
        }
    }

    std::shared_ptr<sampling::AudioClip> m_clip;
    std::array<Pattern, kMaxPatterns> m_patterns{};

    std::atomic<uint32_t> m_current_pattern{0};
    std::atomic<uint32_t> m_queued_pattern{0};
    std::atomic<bool> m_has_queued_pattern{false};
    std::atomic<PatternSwitchMode> m_switch_mode{PatternSwitchMode::BarQuantized};
    std::atomic<uint32_t> m_current_step{0};

    SliceVoice m_primary_voice{};
    SliceVoice m_choked_voice{};

    RingBuffer<ManualTrigger> m_manual_queue;
};

} // namespace audio_core::sequencer
