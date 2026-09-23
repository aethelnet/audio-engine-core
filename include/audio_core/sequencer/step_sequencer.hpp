#pragma once

#include "audio_core/sampling/audio_clip.hpp"
#include "audio_core/clock/timeline_clock.hpp"
#include "audio_core/types.hpp"
#include "audio_core/ring_buffer.hpp"
#include "audio_core/analysis/transient_detector.hpp"
#include "audio_core/dsp/biquad_filter.hpp"

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

enum class VoiceMode : uint8_t {
    Monophonic = 0,    // Classic monophonic choke: every trigger chokes previous voice
    Polyphonic         // Polyphonic mode: voices ring out up to 16 voices, honoring choke groups (1..15)
};

struct StepTrigger {
    bool active{false};
    uint32_t slice_id{0};
    float velocity{1.0f};      // [0.0f, 1.0f]
    float pitch_ratio{1.0f};   // 1.0 = normal, 0.5 = octave down, 2.0 = octave up
    uint8_t probability{100};  // [0..100%]
    bool reverse{false};
    float pan{0.0f};           // [-1.0f (hard L) .. +1.0f (hard R)], 0.0f = Center
    float micro_timing{0.0f};  // [-0.5f .. +0.5f] sub-step offset fraction (negative = early/rush, positive = late/drag)
    float quantize_pct{0.0f};  // [0.0f .. 1.0f] (0.0 = full human groove / keep micro_timing, 1.0 = hard grid snap)

    // Polyphonic Choke Groups (Exclusion Groups)
    uint8_t choke_group{0};    // 0 = polyphonic / no choke, 1..15 = exclusive choke group

    // Per-Step Parameter Locks & Modulation (Elektron / Bitwig style)
    float filter_cutoff{20000.0f}; // [20.0f .. 20000.0f] Hz (20000 = wide open / bypass)
    float filter_res{0.707f};      // Q factor [0.1f .. 10.0f]
    dsp::FilterType filter_type{dsp::FilterType::Lowpass};
    float decay_ms{0.0f};          // 0.0 = full slice duration, >0 = envelope decay length in ms
    float drive{0.0f};             // [0.0 .. 1.0] soft saturation drive
    float send_a{0.0f};            // [0.0 .. 1.0] Aux Send A (Reverb)
    float send_b{0.0f};            // [0.0 .. 1.0] Aux Send B (Delay)
};

struct Pattern {
    static constexpr size_t kMaxSteps = 64;
    uint32_t num_steps{16};
    StepSubdivision subdivision{StepSubdivision::Sixteenth};
    float swing{0.0f};         // [0.0f .. 1.0f] (0.0 = straight, 0.5 = 66.7% triplet, 1.0 = 75% dotted shuffle)
    std::string name{"Pattern"};
    std::array<StepTrigger, kMaxSteps> steps{};

    void clear() noexcept {
        for (auto& s : steps) {
            s = StepTrigger{};
        }
    }

    void set_step(size_t idx, uint32_t slice_id, float velocity = 1.0f,
                  float pitch_ratio = 1.0f, uint8_t prob = 100, bool reverse = false,
                  float pan = 0.0f, float micro_timing = 0.0f, float quantize_pct = 0.0f,
                  uint8_t choke_group = 0, float cutoff = 20000.0f, float res = 0.707f,
                  dsp::FilterType ftype = dsp::FilterType::Lowpass,
                  float decay_ms = 0.0f, float drive = 0.0f,
                  float send_a = 0.0f, float send_b = 0.0f) noexcept {
        if (idx < kMaxSteps) {
            steps[idx] = StepTrigger{
                .active = true,
                .slice_id = slice_id,
                .velocity = std::clamp(velocity, 0.0f, 1.0f),
                .pitch_ratio = std::clamp(pitch_ratio, 0.125f, 8.0f),
                .probability = std::min<uint8_t>(prob, 100),
                .reverse = reverse,
                .pan = std::clamp(pan, -1.0f, 1.0f),
                .micro_timing = std::clamp(micro_timing, -0.5f, 0.5f),
                .quantize_pct = std::clamp(quantize_pct, 0.0f, 1.0f),
                .choke_group = choke_group,
                .filter_cutoff = std::clamp(cutoff, 20.0f, 20000.0f),
                .filter_res = std::clamp(res, 0.1f, 10.0f),
                .filter_type = ftype,
                .decay_ms = std::max(0.0f, decay_ms),
                .drive = std::clamp(drive, 0.0f, 1.0f),
                .send_a = std::clamp(send_a, 0.0f, 1.0f),
                .send_b = std::clamp(send_b, 0.0f, 1.0f)
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

    void quantize_all(float target_pct = 1.0f) noexcept {
        float q = std::clamp(target_pct, 0.0f, 1.0f);
        for (uint32_t i = 0; i < num_steps; ++i) {
            if (steps[i].active) {
                steps[i].quantize_pct = q;
            }
        }
    }

    void nudge_micro_timing(size_t idx, float delta) noexcept {
        if (idx < kMaxSteps && steps[idx].active) {
            steps[idx].micro_timing = std::clamp(steps[idx].micro_timing + delta, -0.5f, 0.5f);
        }
    }

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
        float pan{0.0f};
        uint8_t choke_group{0};
        float cutoff{20000.0f};
        float res{0.707f};
        dsp::FilterType filter_type{dsp::FilterType::Lowpass};
        float decay_ms{0.0f};
        float drive{0.0f};
        float send_a{0.0f};
        float send_b{0.0f};
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

    // Voice Mode: Monophonic (all-choke MPC classic) vs Polyphonic (16 voices with exclusive Choke Groups)
    [[nodiscard]] VoiceMode voice_mode() const noexcept {
        return m_voice_mode.load(std::memory_order_relaxed);
    }
    void set_voice_mode(VoiceMode mode) noexcept {
        m_voice_mode.store(mode, std::memory_order_relaxed);
    }

    [[nodiscard]] uint32_t active_voice_count() const noexcept {
        uint32_t count = 0;
        for (const auto& v : m_voices) {
            if (v.active) ++count;
        }
        return count;
    }

    [[nodiscard]] AudioBuffer& send_a_buffer() noexcept { return m_send_a_buffer; }
    [[nodiscard]] const AudioBuffer& send_a_buffer() const noexcept { return m_send_a_buffer; }
    [[nodiscard]] AudioBuffer& send_b_buffer() noexcept { return m_send_b_buffer; }
    [[nodiscard]] const AudioBuffer& send_b_buffer() const noexcept { return m_send_b_buffer; }

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
                       float pan = 0.0f, uint8_t choke_group = 0,
                       float cutoff = 20000.0f, float res = 0.707f,
                       dsp::FilterType filter_type = dsp::FilterType::Lowpass,
                       float decay_ms = 0.0f, float drive = 0.0f,
                       float send_a = 0.0f, float send_b = 0.0f,
                       uint32_t sample_offset = 0) noexcept {
        ManualTrigger ev{
            .slice_id = slice_id,
            .velocity = std::clamp(velocity, 0.0f, 1.0f),
            .pitch_ratio = std::clamp(pitch_ratio, 0.125f, 8.0f),
            .pan = std::clamp(pan, -1.0f, 1.0f),
            .choke_group = choke_group,
            .cutoff = std::clamp(cutoff, 20.0f, 20000.0f),
            .res = std::clamp(res, 0.1f, 10.0f),
            .filter_type = filter_type,
            .decay_ms = std::max(0.0f, decay_ms),
            .drive = std::clamp(drive, 0.0f, 1.0f),
            .send_a = std::clamp(send_a, 0.0f, 1.0f),
            .send_b = std::clamp(send_b, 0.0f, 1.0f),
            .reverse = reverse,
            .sample_offset = sample_offset
        };
        return m_manual_queue.try_push(ev);
    }

    // Auto-Chop & Slice-to-MIDI Groove Engine:
    // 1. Analyzes audio clip using dual-envelope HF-flux onset detection.
    // 2. Snaps slice boundaries to nearest zero-crossings for zero clicks.
    // 3. Chops audio clip into slices.
    // 4. Quantizes slices to 16-step grid while capturing authentic sub-step micro-timing (mu in [-0.5, 0.5]).
    // 5. Populates target pattern with captured groove (quantize_pct = 0% to preserve raw human pocket).
    bool auto_chop_and_groove(float sensitivity = 0.5f, size_t target_pattern = 0) {
        if (!m_clip || m_clip->num_frames() == 0) return false;

        // 1. Run transient detection on clip
        analysis::TransientDetector detector(m_clip->sample_rate());
        const float* ch0 = m_clip->channel(0);
        const float* ch1 = (m_clip->num_channels() > 1) ? m_clip->channel(1) : ch0;
        auto analysis = detector.analyze(ch0, ch1, m_clip->num_frames(), sensitivity);

        if (analysis.onsets.empty()) {
            // Fallback: 16-slice even grid
            m_clip->slice_grid(16);
            m_patterns[target_pattern % kMaxPatterns].fill_linear_slices(16, 0.9f);
            return true;
        }

        // 2. Snap onsets to nearest zero-crossings within 64 samples to eliminate clicks
        std::vector<uint32_t> snapped_markers;
        snapped_markers.reserve(analysis.onsets.size());
        for (const auto& onset : analysis.onsets) {
            uint32_t zc = analysis::TransientDetector::find_nearest_zero_crossing(
                ch0, onset.sample_offset, 64, m_clip->num_frames());
            snapped_markers.push_back(zc);
        }

        // 3. Slice clip at zero-crossing markers
        m_clip->slice_at_markers(snapped_markers, 128);

        // 4. Map slices to 16-step pattern with micro-timing
        auto& pat = m_patterns[target_pattern % kMaxPatterns];
        pat.clear();
        pat.num_steps = 16;
        pat.subdivision = StepSubdivision::Sixteenth;

        const auto& slices = m_clip->slices();
        if (slices.empty()) return true;

        // Nominal step duration within the clip
        const double clip_step_dur = static_cast<double>(m_clip->num_frames()) / 16.0;

        for (size_t s = 0; s < slices.size(); ++s) {
            const auto& sl = slices[s];
            double step_pos = static_cast<double>(sl.start_frame) / std::max(1.0, clip_step_dur);
            int nominal_step = static_cast<int>(std::round(step_pos));
            nominal_step = std::clamp(nominal_step, 0, 15);

            // Sub-step micro timing in [-0.5, 0.5]
            double center_sample = static_cast<double>(nominal_step) * clip_step_dur;
            double diff_samples = static_cast<double>(sl.start_frame) - center_sample;
            float micro = static_cast<float>(diff_samples / clip_step_dur);
            micro = std::clamp(micro, -0.5f, 0.5f);

            // Find velocity from onset strength if available
            float vel = 0.9f;
            for (const auto& on : analysis.onsets) {
                if (std::abs(static_cast<int64_t>(on.sample_offset) - static_cast<int64_t>(sl.start_frame)) < 128) {
                    vel = std::clamp(on.strength, 0.4f, 1.0f);
                    break;
                }
            }

            // If step already occupied, attempt adjacent step or take strongest
            if (!pat.steps[nominal_step].active) {
                pat.set_step(nominal_step, static_cast<uint32_t>(s), vel, 1.0f, 100, false, 0.0f, micro, 0.0f);
            } else if (nominal_step + 1 < 16 && !pat.steps[nominal_step + 1].active) {
                double next_center = static_cast<double>(nominal_step + 1) * clip_step_dur;
                float next_micro = static_cast<float>((static_cast<double>(sl.start_frame) - next_center) / clip_step_dur);
                pat.set_step(nominal_step + 1, static_cast<uint32_t>(s), vel, 1.0f, 100, false, 0.0f, next_micro, 0.0f);
            }
        }

        return true;
    }

    void stop() noexcept {
        for (auto& v : m_voices) {
            v.active = false;
            v.is_fading_out = false;
        }
        m_current_step.store(0, std::memory_order_relaxed);
    }

    [[nodiscard]] bool is_voice_active() const noexcept {
        for (const auto& v : m_voices) {
            if (v.active) return true;
        }
        return false;
    }

    // Real-Time Render Pipeline: Lock-free, zero allocation, sample-accurate polyphonic micro-timing dispatch
    void render(Sample* out_l, Sample* out_r, uint32_t frames,
                const clock::TimelineClock& clock,
                const clock::BlockBoundaryEvents& boundary_events = {},
                Sample* ext_send_a_l = nullptr, Sample* ext_send_a_r = nullptr,
                Sample* ext_send_b_l = nullptr, Sample* ext_send_b_r = nullptr) noexcept {
        if (!out_l || !out_r || frames == 0) return;

        // Clear output block
        for (uint32_t i = 0; i < frames; ++i) {
            out_l[i] = 0.0f;
            out_r[i] = 0.0f;
        }

        // Ensure internal send buffers have at least frames capacity
        if (m_send_a_buffer.num_frames() < frames) {
            m_send_a_buffer.resize(2, frames);
            m_send_b_buffer.resize(2, frames);
        }

        Sample* s_a_l_ptr = m_send_a_buffer.channel(0);
        Sample* s_a_r_ptr = m_send_a_buffer.channel(1);
        Sample* s_b_l_ptr = m_send_b_buffer.channel(0);
        Sample* s_b_r_ptr = m_send_b_buffer.channel(1);

        for (uint32_t i = 0; i < frames; ++i) {
            s_a_l_ptr[i] = 0.0f;
            s_a_r_ptr[i] = 0.0f;
            s_b_l_ptr[i] = 0.0f;
            s_b_r_ptr[i] = 0.0f;
        }
        if (ext_send_a_l) {
            for (uint32_t i = 0; i < frames; ++i) {
                ext_send_a_l[i] = 0.0f;
                if (ext_send_a_r) ext_send_a_r[i] = 0.0f;
            }
        }
        if (ext_send_b_l) {
            for (uint32_t i = 0; i < frames; ++i) {
                ext_send_b_l[i] = 0.0f;
                if (ext_send_b_r) ext_send_b_r[i] = 0.0f;
            }
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

        // 2. Compute Step Transitions & Clock Parameters within this block
        const double spb = clock.samples_per_beat();
        const double step_dur = calculate_step_duration(active_pattern.subdivision, spb);

        const uint64_t end_pos = clock.sample_position();
        const uint64_t start_pos = (end_pos >= frames) ? (end_pos - frames) : 0;
        const bool clock_playing = clock.is_playing();

        const uint32_t engine_sr = clock.sample_rate();
        const uint32_t clip_sr = m_clip ? m_clip->sample_rate() : engine_sr;
        const double rate_ratio = (engine_sr > 0) ? (static_cast<double>(clip_sr) / static_cast<double>(engine_sr)) : 1.0;

        // 3. Precompute Sample-Accurate Micro-Timing Triggers in this block window
        struct PendingTrigger {
            uint32_t frame_offset{0};
            uint32_t slice_id{0};
            float velocity{1.0f};
            float pitch_ratio{1.0f};
            float pan{0.0f};
            uint8_t choke_group{0};
            float filter_cutoff{20000.0f};
            float filter_res{0.707f};
            dsp::FilterType filter_type{dsp::FilterType::Lowpass};
            float decay_ms{0.0f};
            float drive{0.0f};
            float send_a{0.0f};
            float send_b{0.0f};
            bool reverse{false};
        };

        static constexpr size_t kMaxPendingTriggers = 8;
        PendingTrigger pending[kMaxPendingTriggers];
        size_t num_pending = 0;

        if (clock_playing && step_dur > 1.0 && active_pattern.num_steps > 0) {
            const uint32_t N = active_pattern.num_steps;
            const float pat_swing = std::clamp(active_pattern.swing, 0.0f, 1.0f);

            // Update current step playhead for GUI LED
            uint64_t nominal_step = static_cast<uint64_t>(std::floor(static_cast<double>(start_pos) / step_dur));
            m_current_step.store(static_cast<uint32_t>(nominal_step % N), std::memory_order_relaxed);

            // Candidate range of global step indices that could trigger inside [start_pos, start_pos + frames)
            const double min_s_flt = (static_cast<double>(start_pos) / step_dur) - 1.0;
            const double max_s_flt = (static_cast<double>(start_pos + frames) / step_dur) + 0.5;

            const int64_t s_min = std::max<int64_t>(0, static_cast<int64_t>(std::floor(min_s_flt)));
            const int64_t s_max = static_cast<int64_t>(std::ceil(max_s_flt));

            for (int64_t S = s_min; S <= s_max && num_pending < kMaxPendingTriggers; ++S) {
                const uint32_t step_idx = static_cast<uint32_t>(S % N);
                const auto& step = active_pattern.steps[step_idx];
                if (!step.active) continue;

                // Micro-timing with per-note quantization
                const float effective_micro = step.micro_timing * (1.0f - step.quantize_pct);

                // Pattern swing (applied to odd 16th steps)
                const float swing_offset = (step_idx % 2 == 1) ? (pat_swing * 0.5f) : 0.0f;

                const double total_offset = static_cast<double>(effective_micro + swing_offset);
                double trig_sample_flt = (static_cast<double>(S) + total_offset) * step_dur;

                int64_t trig_sample = static_cast<int64_t>(std::round(trig_sample_flt));
                if (S == 0 && trig_sample < 0) {
                    trig_sample = 0; // Downbeat start guard
                }

                if (trig_sample >= static_cast<int64_t>(start_pos) &&
                    trig_sample < static_cast<int64_t>(start_pos + frames)) {

                    // Step probability check (deterministic hash per step instance)
                    bool fire = true;
                    if (step.probability < 100) {
                        uint32_t hash = static_cast<uint32_t>(S * 2654435761u);
                        fire = (hash % 100) < step.probability;
                    }

                    if (fire) {
                        const uint32_t f_idx = static_cast<uint32_t>(trig_sample - start_pos);
                        pending[num_pending++] = PendingTrigger{
                            .frame_offset = f_idx,
                            .slice_id = step.slice_id,
                            .velocity = step.velocity,
                            .pitch_ratio = step.pitch_ratio,
                            .pan = step.pan,
                            .choke_group = step.choke_group,
                            .filter_cutoff = step.filter_cutoff,
                            .filter_res = step.filter_res,
                            .filter_type = step.filter_type,
                            .decay_ms = step.decay_ms,
                            .drive = step.drive,
                            .send_a = step.send_a,
                            .send_b = step.send_b,
                            .reverse = step.reverse
                        };
                    }
                }
            }
        }

        // 4. Process frame-by-frame with sample-accurate step and pad triggers
        for (uint32_t i = 0; i < frames; ++i) {
            // Process any queued manual pad triggers
            if (i == 0) {
                ManualTrigger manual_ev;
                while (m_manual_queue.try_pop(manual_ev)) {
                    start_slice_voice(manual_ev.slice_id, manual_ev.velocity,
                                      manual_ev.pitch_ratio, manual_ev.reverse,
                                      manual_ev.pan, manual_ev.choke_group,
                                      manual_ev.cutoff, manual_ev.res,
                                      manual_ev.filter_type, manual_ev.decay_ms,
                                      manual_ev.drive, manual_ev.send_a, manual_ev.send_b,
                                      engine_sr);
                }
            }

            // Dispatch sample-accurate step triggers
            for (size_t p = 0; p < num_pending; ++p) {
                if (pending[p].frame_offset == i) {
                    start_slice_voice(pending[p].slice_id, pending[p].velocity,
                                      pending[p].pitch_ratio, pending[p].reverse,
                                      pending[p].pan, pending[p].choke_group,
                                      pending[p].filter_cutoff, pending[p].filter_res,
                                      pending[p].filter_type, pending[p].decay_ms,
                                      pending[p].drive, pending[p].send_a, pending[p].send_b,
                                      engine_sr);
                }
            }

            // Render active voices with micro-fade crossfade, stereo panning, filter, saturation & aux sends
            float sa_l = 0.0f, sa_r = 0.0f, sb_l = 0.0f, sb_r = 0.0f;
            render_sample(out_l[i], out_r[i], sa_l, sa_r, sb_l, sb_r, rate_ratio);

            s_a_l_ptr[i] = sa_l;
            s_a_r_ptr[i] = sa_r;
            s_b_l_ptr[i] = sb_l;
            s_b_r_ptr[i] = sb_r;

            if (ext_send_a_l) ext_send_a_l[i] = sa_l;
            if (ext_send_a_r) ext_send_a_r[i] = sa_r;
            if (ext_send_b_l) ext_send_b_l[i] = sb_l;
            if (ext_send_b_r) ext_send_b_r[i] = sb_r;
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
        float pan{0.0f};
        bool reverse{false};
        uint8_t choke_group{0};
        uint64_t age{0};

        // Per-Voice Filter Modulation
        dsp::BiquadFilter filter_l;
        dsp::BiquadFilter filter_r;
        bool filter_enabled{false};

        // Per-Voice Envelope Decay
        uint32_t decay_frames{0};
        uint32_t elapsed_frames{0};

        // Per-Voice Drive & Sends
        float drive{0.0f};
        float send_a{0.0f};
        float send_b{0.0f};

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

    void start_slice_voice(uint32_t slice_id, float velocity, float pitch_ratio, bool reverse,
                           float pan = 0.0f, uint8_t choke_group = 0,
                           float cutoff = 20000.0f, float res = 0.707f,
                           dsp::FilterType ftype = dsp::FilterType::Lowpass,
                           float decay_ms = 0.0f, float drive = 0.0f,
                           float send_a = 0.0f, float send_b = 0.0f,
                           uint32_t sample_rate = 48000) noexcept {
        if (!m_clip) return;
        const auto& slices = m_clip->slices();
        if (slice_id >= slices.size()) return;

        const auto& slice = slices[slice_id];
        const uint32_t slice_len = (slice.end_frame > slice.start_frame) ? (slice.end_frame - slice.start_frame) : 0;
        if (slice_len == 0) return;

        uint8_t eff_choke = (choke_group > 0) ? choke_group : slice.choke_group;
        const auto v_mode = m_voice_mode.load(std::memory_order_relaxed);

        // 1. Choke Evaluation:
        for (auto& v : m_voices) {
            if (!v.active || v.is_fading_out) continue;

            bool should_choke = false;
            if (v_mode == VoiceMode::Monophonic) {
                // In Monophonic mode: every trigger chokes all previous voices
                should_choke = true;
            } else {
                // In Polyphonic mode:
                // Case A: Exclusive Choke Group (e.g. Open Hat vs Closed Hat in Group 1)
                if (eff_choke > 0 && v.choke_group == eff_choke) {
                    should_choke = true;
                }
                // Case B: Same Slice Retrigger Choke (prevents acoustic comb filtering / flamming)
                else if (v.slice_id == slice_id) {
                    should_choke = true;
                }
            }

            if (should_choke) {
                v.is_fading_out = true;
                v.fade_out_remaining = kMicroFadeFrames;
                v.fade_out_total = kMicroFadeFrames;
            }
        }

        // 2. Allocate an idle voice, or recycle the oldest/fading voice:
        SliceVoice* target = nullptr;
        for (auto& v : m_voices) {
            if (!v.active) {
                target = &v;
                break;
            }
        }

        if (!target) {
            uint64_t oldest_age = UINT64_MAX;
            for (auto& v : m_voices) {
                if (v.is_fading_out) {
                    target = &v;
                    break;
                }
                if (v.age < oldest_age) {
                    oldest_age = v.age;
                    target = &v;
                }
            }
        }

        if (!target) return;

        // 3. Initialize target voice:
        target->active = true;
        target->slice_id = slice_id;
        target->start_frame = slice.start_frame;
        target->end_frame = slice.end_frame;
        target->playhead = reverse ? static_cast<double>(slice_len - 1) : 0.0;
        target->velocity = velocity;
        target->slice_gain = slice.gain;
        target->pitch_ratio = std::clamp(pitch_ratio, 0.125f, 8.0f);
        target->pan = std::clamp(pan, -1.0f, 1.0f);
        target->reverse = reverse;
        target->choke_group = eff_choke;
        target->age = ++m_voice_age_counter;

        // Envelope decay
        if (decay_ms > 0.0f && sample_rate > 0) {
            target->decay_frames = static_cast<uint32_t>((decay_ms * static_cast<float>(sample_rate)) / 1000.0f);
        } else {
            target->decay_frames = 0;
        }
        target->elapsed_frames = 0;

        // Drive & Sends
        target->drive = std::clamp(drive, 0.0f, 1.0f);
        target->send_a = std::clamp(send_a, 0.0f, 1.0f);
        target->send_b = std::clamp(send_b, 0.0f, 1.0f);

        // Filter initialization
        if (cutoff < 19500.0f || ftype != dsp::FilterType::Lowpass || res > 0.8f) {
            target->filter_enabled = true;
            target->filter_l.init(sample_rate);
            target->filter_l.set_type(ftype);
            target->filter_l.set_cutoff(cutoff);
            target->filter_l.set_q(res);

            target->filter_r.init(sample_rate);
            target->filter_r.set_type(ftype);
            target->filter_r.set_cutoff(cutoff);
            target->filter_r.set_q(res);
        } else {
            target->filter_enabled = false;
        }

        target->fade_in_remaining = kMicroFadeFrames;
        target->fade_in_total = kMicroFadeFrames;
        target->is_fading_out = false;
    }

    inline void render_voice_frame(SliceVoice& v, float fade_gain,
                                   float& out_l, float& out_r,
                                   float& s_a_l, float& s_a_r,
                                   float& s_b_l, float& s_b_r,
                                   double rate_ratio = 1.0) noexcept {
        const uint32_t slice_len = (v.end_frame > v.start_frame) ? (v.end_frame - v.start_frame) : 0;
        if (slice_len == 0) {
            v.active = false;
            return;
        }

        if (v.playhead < 0.0 || v.playhead >= static_cast<double>(slice_len)) {
            v.active = false;
            return;
        }

        // Trigger micro-fade out if decay envelope reached
        if (v.decay_frames > 0 && v.elapsed_frames >= v.decay_frames) {
            if (!v.is_fading_out) {
                v.is_fading_out = true;
                v.fade_out_remaining = kMicroFadeFrames;
                v.fade_out_total = kMicroFadeFrames;
            }
        }
        v.elapsed_frames++;

        double global_pos = static_cast<double>(v.start_frame) + v.playhead;

        const float* src_l = m_clip->channel(0);
        const float* src_r = (m_clip->num_channels() > 1) ? m_clip->channel(1) : src_l;

        float sample_l = dsp::sample_hermite(src_l, global_pos, m_clip->num_frames());
        float sample_r = dsp::sample_hermite(src_r, global_pos, m_clip->num_frames());

        // 1. Per-Voice Filter Processing (Parameter Locks)
        if (v.filter_enabled) {
            sample_l = v.filter_l.process_sample(sample_l);
            sample_r = v.filter_r.process_sample(sample_r);
        }

        // 2. Per-Voice Soft Saturation Drive
        if (v.drive > 0.001f) {
            float drive_gain = 1.0f + v.drive * 4.0f;
            sample_l = std::tanh(sample_l * drive_gain);
            sample_r = std::tanh(sample_r * drive_gain);
        }

        // 3. Constant power stereo panning (0 dB center, +3 dB hard pan)
        const float total_gain = v.velocity * v.slice_gain * fade_gain;
        const float norm_pan = std::clamp(v.pan, -1.0f, 1.0f);
        const float theta = (norm_pan + 1.0f) * 0.25f * 3.14159265358979323846f;
        const float pan_l = std::cos(theta) * 1.41421356f;
        const float pan_r = std::sin(theta) * 1.41421356f;

        out_l += sample_l * total_gain * pan_l;
        out_r += sample_r * total_gain * pan_r;

        // 4. Per-Step / Per-Voice Aux Sends
        if (v.send_a > 0.0001f) {
            const float gain_a = total_gain * v.send_a;
            s_a_l += sample_l * gain_a * pan_l;
            s_a_r += sample_r * gain_a * pan_r;
        }
        if (v.send_b > 0.0001f) {
            const float gain_b = total_gain * v.send_b;
            s_b_l += sample_l * gain_b * pan_l;
            s_b_r += sample_r * gain_b * pan_r;
        }

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

    inline void render_sample(float& out_l, float& out_r,
                              float& s_a_l, float& s_a_r,
                              float& s_b_l, float& s_b_r,
                              double rate_ratio = 1.0) noexcept {
        for (auto& v : m_voices) {
            if (!v.active) continue;

            float cur_gain = 1.0f;
            if (v.fade_in_remaining > 0) {
                cur_gain = 1.0f - (static_cast<float>(v.fade_in_remaining) / static_cast<float>(v.fade_in_total));
                v.fade_in_remaining--;
            }

            if (v.is_fading_out) {
                if (v.fade_out_remaining > 0) {
                    float out_g = static_cast<float>(v.fade_out_remaining) / static_cast<float>(v.fade_out_total);
                    v.fade_out_remaining--;
                    cur_gain *= out_g;
                    if (v.fade_out_remaining == 0) {
                        v.active = false;
                        v.is_fading_out = false;
                        continue;
                    }
                } else {
                    v.active = false;
                    v.is_fading_out = false;
                    continue;
                }
            }

            render_voice_frame(v, cur_gain, out_l, out_r, s_a_l, s_a_r, s_b_l, s_b_r, rate_ratio);
        }
    }

    inline void render_sample(float& out_l, float& out_r, double rate_ratio = 1.0) noexcept {
        float dummy_sa_l = 0.0f, dummy_sa_r = 0.0f, dummy_sb_l = 0.0f, dummy_sb_r = 0.0f;
        render_sample(out_l, out_r, dummy_sa_l, dummy_sa_r, dummy_sb_l, dummy_sb_r, rate_ratio);
    }

    std::shared_ptr<sampling::AudioClip> m_clip;
    std::array<Pattern, kMaxPatterns> m_patterns{};

    std::atomic<uint32_t> m_current_pattern{0};
    std::atomic<uint32_t> m_queued_pattern{0};
    std::atomic<bool> m_has_queued_pattern{false};
    std::atomic<PatternSwitchMode> m_switch_mode{PatternSwitchMode::BarQuantized};
    std::atomic<VoiceMode> m_voice_mode{VoiceMode::Monophonic};
    std::atomic<uint32_t> m_current_step{0};

    static constexpr size_t kMaxVoices = 16;
    std::array<SliceVoice, kMaxVoices> m_voices{};
    uint64_t m_voice_age_counter{0};

    RingBuffer<ManualTrigger> m_manual_queue;

    AudioBuffer m_send_a_buffer{2, 2048};
    AudioBuffer m_send_b_buffer{2, 2048};
};

} // namespace audio_core::sequencer
