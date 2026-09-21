#pragma once

#include "audio_core/types.hpp"
#include "audio_core/clock/timeline_clock.hpp"
#include "audio_core/sampling/audio_clip.hpp"
#include "audio_core/sampling/vari_speed_streamer.hpp"
#include "audio_core/sequencer/step_sequencer.hpp"

#include <array>
#include <memory>
#include <atomic>
#include <string>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cstdint>

namespace audio_core::sequencer {

// ============================================================================
// Launch Quantization Grid
// ============================================================================
enum class LaunchQuantize : uint8_t {
    Immediate = 0,     // Switch immediately at sample 0 of current render block
    Sixteenth = 1,     // Switch at next 16th note boundary
    Beat = 2,          // Switch at next beat (1/4 note) boundary
    Bar = 3,           // Switch at next bar downbeat (default)
    TwoBars = 4,       // Switch at next 2-bar boundary
    FourBars = 5       // Switch at next 4-bar boundary
};

// ============================================================================
// Launch Trigger Behavior
// ============================================================================
enum class LaunchMode : uint8_t {
    Trigger = 0,       // Starts clip; continues playing/looping
    Toggle = 1,        // If playing, triggers stop; if stopped, triggers play
    Gate = 2           // Plays while active
};

// ============================================================================
// Slot Real-Time Playback State
// ============================================================================
enum class SlotPlayState : uint8_t {
    Empty = 0,
    Stopped = 1,
    QueuedPlay = 2,    // Launch requested, waiting for quantization boundary
    Playing = 3,       // Actively rendering
    QueuedStop = 4     // Stop requested, waiting for quantization boundary
};

// ============================================================================
// ClipSlot: Container for Audio Loop, MIDI/Step Pattern, or Hybrid Clip
// ============================================================================
struct ClipSlot {
    std::string name{"Empty"};
    std::shared_ptr<sampling::AudioClip> clip{nullptr};
    bool has_midi_pattern{false};
    uint32_t midi_pattern_idx{0};

    // Playback settings for audio loop
    sampling::PlaybackMode playback_mode{sampling::PlaybackMode::BeatSyncTimeStretch};
    float pitch_semitones{0.0f};
    float speed_ratio{1.0f};
    bool loop{true};
    bool reverse{false};
    float bar_length{0.0f}; // 0 = auto-detect from clip BPM and frame count
    uint32_t loop_start{0};
    uint32_t loop_end{0};

    // Launch settings
    LaunchQuantize quantize{LaunchQuantize::Bar};
    LaunchMode mode{LaunchMode::Trigger};
    bool legato{false}; // If true, inherit playhead musical phase on launch

    // Real-time state
    std::atomic<SlotPlayState> state{SlotPlayState::Empty};

    ClipSlot() = default;

    ClipSlot(const ClipSlot& other)
        : name(other.name),
          clip(other.clip),
          has_midi_pattern(other.has_midi_pattern),
          midi_pattern_idx(other.midi_pattern_idx),
          playback_mode(other.playback_mode),
          pitch_semitones(other.pitch_semitones),
          speed_ratio(other.speed_ratio),
          loop(other.loop),
          reverse(other.reverse),
          bar_length(other.bar_length),
          loop_start(other.loop_start),
          loop_end(other.loop_end),
          quantize(other.quantize),
          mode(other.mode),
          legato(other.legato),
          state(other.state.load(std::memory_order_relaxed)) {}

    ClipSlot& operator=(const ClipSlot& other) {
        if (this != &other) {
            name = other.name;
            clip = other.clip;
            has_midi_pattern = other.has_midi_pattern;
            midi_pattern_idx = other.midi_pattern_idx;
            playback_mode = other.playback_mode;
            pitch_semitones = other.pitch_semitones;
            speed_ratio = other.speed_ratio;
            loop = other.loop;
            reverse = other.reverse;
            bar_length = other.bar_length;
            loop_start = other.loop_start;
            loop_end = other.loop_end;
            quantize = other.quantize;
            mode = other.mode;
            legato = other.legato;
            state.store(other.state.load(std::memory_order_relaxed), std::memory_order_relaxed);
        }
        return *this;
    }

    [[nodiscard]] bool is_empty() const noexcept {
        return (clip == nullptr) && !has_midi_pattern;
    }
};

// ============================================================================
// ClipLauncher: Zero-Allocation Real-Time Audio Loop & MIDI Clip Launcher Engine
// - Supports 8 Clip Slots per Track
// - Sample-accurate launch quantization (Immediate, 1/16, Beat, Bar, 2-Bar, 4-Bar)
// - Anti-click dual-voice crossfading (64-sample equal-power micro-fade)
// - Decoupled WSOLA time-stretching & vari-speed repitch
// - Seamless Legato phase locking
// - Direct coupling with StepSequencer for Drum/MIDI Pattern triggering
// ============================================================================
class ClipLauncher {
public:
    static constexpr size_t kMaxSlots = 8;
    static constexpr uint32_t kMicroFadeFrames = 64; // ~1.33 ms @ 48kHz for zero clicks

    enum class QueuedAction : uint8_t {
        None = 0,
        Play = 1,
        Stop = 2
    };

    struct Voice {
        bool active{false};
        int32_t slot_idx{-1};
        sampling::VariSpeedStreamer streamer{48000.0f};
        float fade_gain{1.0f};
        float fade_delta{0.0f};
    };

    explicit ClipLauncher(float sample_rate = 48000.0f) noexcept
        : m_voice_a{false, -1, sampling::VariSpeedStreamer(sample_rate), 1.0f, 0.0f},
          m_voice_b{false, -1, sampling::VariSpeedStreamer(sample_rate), 1.0f, 0.0f},
          m_primary_voice(&m_voice_a),
          m_choked_voice(&m_voice_b),
          m_sample_rate(sample_rate) {}

    void set_associated_sequencer(StepSequencer* seq) noexcept {
        m_associated_sequencer = seq;
    }
    [[nodiscard]] StepSequencer* associated_sequencer() noexcept { return m_associated_sequencer; }

    // ========================================================================
    // Slot Configuration APIs (Called from UI / Host thread)
    // ========================================================================

    void set_slot_audio(size_t slot_idx, std::shared_ptr<sampling::AudioClip> clip,
                        sampling::PlaybackMode mode = sampling::PlaybackMode::BeatSyncTimeStretch,
                        float pitch_st = 0.0f, float bars = 0.0f,
                        const std::string& name = "") {
        if (slot_idx >= kMaxSlots) return;
        auto& s = m_slots[slot_idx];
        s.clip = std::move(clip);
        s.has_midi_pattern = false;
        s.playback_mode = mode;
        s.pitch_semitones = pitch_st;
        s.bar_length = bars;
        s.name = !name.empty() ? name : (s.clip ? s.clip->name() : "Audio Loop");
        s.state.store(SlotPlayState::Stopped, std::memory_order_relaxed);
    }

    void set_slot_midi(size_t slot_idx, uint32_t pattern_idx, const std::string& name = "") {
        if (slot_idx >= kMaxSlots) return;
        auto& s = m_slots[slot_idx];
        s.clip = nullptr;
        s.has_midi_pattern = true;
        s.midi_pattern_idx = pattern_idx;
        s.name = !name.empty() ? name : ("Pattern " + std::to_string(pattern_idx + 1));
        s.state.store(SlotPlayState::Stopped, std::memory_order_relaxed);
    }

    void set_slot_hybrid(size_t slot_idx, std::shared_ptr<sampling::AudioClip> clip,
                         uint32_t pattern_idx,
                         sampling::PlaybackMode mode = sampling::PlaybackMode::BeatSyncTimeStretch,
                         const std::string& name = "") {
        if (slot_idx >= kMaxSlots) return;
        auto& s = m_slots[slot_idx];
        s.clip = std::move(clip);
        s.has_midi_pattern = true;
        s.midi_pattern_idx = pattern_idx;
        s.playback_mode = mode;
        s.name = !name.empty() ? name : (s.clip ? s.clip->name() : ("Hybrid " + std::to_string(slot_idx + 1)));
        s.state.store(SlotPlayState::Stopped, std::memory_order_relaxed);
    }

    void clear_slot(size_t slot_idx) noexcept {
        if (slot_idx >= kMaxSlots) return;
        if (m_current_slot.load(std::memory_order_relaxed) == static_cast<int32_t>(slot_idx)) {
            stop_immediate();
        }
        m_slots[slot_idx] = ClipSlot{};
    }

    [[nodiscard]] ClipSlot& slot(size_t idx) noexcept { return m_slots[idx % kMaxSlots]; }
    [[nodiscard]] const ClipSlot& slot(size_t idx) const noexcept { return m_slots[idx % kMaxSlots]; }

    // ========================================================================
    // Real-Time Control APIs (Lock-free, wait-free)
    // ========================================================================

    void set_timeline_clock(const clock::TimelineClock* clock) noexcept {
        m_timeline_clock = clock;
    }
    [[nodiscard]] const clock::TimelineClock* timeline_clock() const noexcept {
        return m_timeline_clock;
    }

    void launch_slot(size_t slot_idx, LaunchQuantize quantize = LaunchQuantize::Bar, bool legato = false,
                     const clock::TimelineClock* clock = nullptr) noexcept {
        if (slot_idx >= kMaxSlots) return;
        if (clock) m_timeline_clock = clock;
        const int32_t target = static_cast<int32_t>(slot_idx);
        auto& target_slot = m_slots[slot_idx];
        if (target_slot.is_empty()) return;

        // Handle Toggle mode
        if (target_slot.mode == LaunchMode::Toggle &&
            m_current_slot.load(std::memory_order_relaxed) == target &&
            m_queued_action.load(std::memory_order_relaxed) != QueuedAction::Stop) {
            stop(quantize);
            return;
        }

        if (quantize == LaunchQuantize::Immediate) {
            execute_launch(target, legato, m_timeline_clock, 0);
            return;
        }

        // Set queued state
        target_slot.state.store(SlotPlayState::QueuedPlay, std::memory_order_relaxed);
        m_queued_slot.store(target, std::memory_order_relaxed);
        m_queued_quantize.store(quantize, std::memory_order_relaxed);
        m_queued_legato.store(legato, std::memory_order_relaxed);
        m_queued_action.store(QueuedAction::Play, std::memory_order_relaxed);
        m_is_active.store(true, std::memory_order_relaxed);
    }

    void stop(LaunchQuantize quantize = LaunchQuantize::Bar) noexcept {
        if (m_current_slot.load(std::memory_order_relaxed) < 0 &&
            m_queued_action.load(std::memory_order_relaxed) == QueuedAction::None) {
            return;
        }

        if (quantize == LaunchQuantize::Immediate) {
            stop_immediate();
            return;
        }

        int32_t cur = m_current_slot.load(std::memory_order_relaxed);
        if (cur >= 0 && cur < static_cast<int32_t>(kMaxSlots)) {
            m_slots[cur].state.store(SlotPlayState::QueuedStop, std::memory_order_relaxed);
        }

        int32_t q = m_queued_slot.load(std::memory_order_relaxed);
        if (q >= 0 && q < static_cast<int32_t>(kMaxSlots) && q != cur) {
            m_slots[q].state.store(SlotPlayState::Stopped, std::memory_order_relaxed);
        }

        m_queued_quantize.store(quantize, std::memory_order_relaxed);
        m_queued_action.store(QueuedAction::Stop, std::memory_order_relaxed);
    }

    void stop_immediate() noexcept {
        execute_stop();
    }

    [[nodiscard]] bool is_active() const noexcept {
        return m_is_active.load(std::memory_order_relaxed);
    }

    [[nodiscard]] int32_t current_slot_idx() const noexcept {
        return m_current_slot.load(std::memory_order_relaxed);
    }

    [[nodiscard]] int32_t queued_slot_idx() const noexcept {
        return (m_queued_action.load(std::memory_order_relaxed) == QueuedAction::Play)
            ? m_queued_slot.load(std::memory_order_relaxed) : -1;
    }

    [[nodiscard]] bool is_stopping() const noexcept {
        return (m_queued_action.load(std::memory_order_relaxed) == QueuedAction::Stop);
    }

    [[nodiscard]] double playhead() const noexcept {
        return m_primary_voice ? m_primary_voice->streamer.playhead() : 0.0;
    }

    // ========================================================================
    // Real-Time Audio Render Pipeline
    // Sample-accurate boundary detection, sub-block slicing & micro-fading
    // ========================================================================
    void render(Sample* out_l, Sample* out_r, uint32_t frames,
                const clock::TimelineClock& clock,
                const clock::BlockBoundaryEvents& boundary_events) noexcept {
        if (!out_l || !out_r || frames == 0) return;

        // Clear output buffer
        std::memset(out_l, 0, frames * sizeof(Sample));
        std::memset(out_r, 0, frames * sizeof(Sample));

        m_timeline_clock = &clock;

        if (!m_is_active.load(std::memory_order_relaxed)) return;

        // 1. Check for scheduled launch/stop quantization boundary in this audio block
        uint32_t split_offset = frames; // default: no split
        bool has_boundary_trigger = false;

        const auto queued_act = m_queued_action.load(std::memory_order_relaxed);
        if (queued_act != QueuedAction::None) {
            const auto q_mode = m_queued_quantize.load(std::memory_order_relaxed);
            has_boundary_trigger = evaluate_quantization_boundary(clock, boundary_events, frames, q_mode, split_offset);
        }

        // 2. Render pre-boundary segment [0 .. split_offset)
        if (split_offset > 0) {
            render_voices_chunk(out_l, out_r, split_offset, clock);
        }

        // 3. Execute boundary event if hit within this block
        if (has_boundary_trigger && split_offset < frames) {
            if (queued_act == QueuedAction::Play) {
                int32_t target = m_queued_slot.load(std::memory_order_relaxed);
                bool legato = m_queued_legato.load(std::memory_order_relaxed);
                execute_launch(target, legato, &clock, split_offset);
            } else if (queued_act == QueuedAction::Stop) {
                execute_stop();
            }
        }

        // 4. Render post-boundary segment [split_offset .. frames)
        if (split_offset < frames) {
            const uint32_t post_frames = frames - split_offset;
            render_voices_chunk(out_l + split_offset, out_r + split_offset, post_frames, clock);
        }

        // Update overall active flag
        bool still_active = (m_current_slot.load(std::memory_order_relaxed) >= 0) ||
                            (m_queued_action.load(std::memory_order_relaxed) != QueuedAction::None) ||
                            m_primary_voice->active || m_choked_voice->active;
        m_is_active.store(still_active, std::memory_order_relaxed);
    }

private:
    // Helper to evaluate if the queued quantization mode triggers within this audio block
    bool evaluate_quantization_boundary(const clock::TimelineClock& clock,
                                        const clock::BlockBoundaryEvents& boundary_events,
                                        uint32_t frames,
                                        LaunchQuantize q_mode,
                                        uint32_t& out_offset) const noexcept {
        if (!clock.is_playing()) {
            // If transport is stopped, immediate trigger
            out_offset = 0;
            return true;
        }

        const uint64_t end_pos = clock.sample_position();
        const uint64_t start_pos = (end_pos >= frames) ? (end_pos - frames) : 0;
        const double spb = clock.samples_per_beat();
        const double spbar = clock.samples_per_bar();

        switch (q_mode) {
            case LaunchQuantize::Immediate: {
                out_offset = 0;
                return true;
            }
            case LaunchQuantize::Sixteenth: {
                if (spb <= 0.0) return false;
                const double sp16 = spb * 0.25;
                const uint64_t s_16 = static_cast<uint64_t>(std::floor(static_cast<double>(start_pos) / sp16));
                const uint64_t e_16 = static_cast<uint64_t>(std::floor(static_cast<double>(end_pos) / sp16));
                if (e_16 > s_16) {
                    uint64_t b_sample = static_cast<uint64_t>(std::round(static_cast<double>(e_16) * sp16));
                    out_offset = (b_sample >= start_pos && b_sample < end_pos) ? static_cast<uint32_t>(b_sample - start_pos) : 0;
                    return true;
                }
                return false;
            }
            case LaunchQuantize::Beat: {
                if (boundary_events.has_beat_boundary) {
                    out_offset = boundary_events.beat_sample_offset;
                    return true;
                }
                return false;
            }
            case LaunchQuantize::Bar: {
                if (boundary_events.has_bar_boundary) {
                    out_offset = boundary_events.bar_sample_offset;
                    return true;
                }
                return false;
            }
            case LaunchQuantize::TwoBars: {
                if (boundary_events.has_bar_boundary && spbar > 0.0) {
                    uint64_t b_sample = start_pos + boundary_events.bar_sample_offset;
                    uint64_t bar_idx = static_cast<uint64_t>(std::round(static_cast<double>(b_sample) / spbar));
                    if (bar_idx % 2 == 0) {
                        out_offset = boundary_events.bar_sample_offset;
                        return true;
                    }
                }
                return false;
            }
            case LaunchQuantize::FourBars: {
                if (boundary_events.has_bar_boundary && spbar > 0.0) {
                    uint64_t b_sample = start_pos + boundary_events.bar_sample_offset;
                    uint64_t bar_idx = static_cast<uint64_t>(std::round(static_cast<double>(b_sample) / spbar));
                    if (bar_idx % 4 == 0) {
                        out_offset = boundary_events.bar_sample_offset;
                        return true;
                    }
                }
                return false;
            }
        }
        return false;
    }

    // Executes immediate transition to target slot with dual-voice micro-fade
    void execute_launch(int32_t target_slot, bool legato,
                        const clock::TimelineClock* clock = nullptr,
                        uint32_t sample_offset = 0) noexcept {
        if (target_slot < 0 || target_slot >= static_cast<int32_t>(kMaxSlots)) return;
        auto& slot_info = m_slots[target_slot];
        if (slot_info.is_empty()) return;

        int32_t prev_slot = m_current_slot.load(std::memory_order_relaxed);

        // 1. If a primary voice is active, move it to choked voice to perform micro-fade out
        if (m_primary_voice->active) {
            std::swap(m_primary_voice, m_choked_voice);
            m_choked_voice->fade_gain = 1.0f;
            m_choked_voice->fade_delta = -1.0f / static_cast<float>(kMicroFadeFrames);

            if (prev_slot >= 0 && prev_slot < static_cast<int32_t>(kMaxSlots)) {
                m_slots[prev_slot].state.store(SlotPlayState::Stopped, std::memory_order_relaxed);
            }
        }

        // 2. Initialize new primary voice
        m_primary_voice->active = (slot_info.clip != nullptr);
        m_primary_voice->slot_idx = target_slot;

        if (m_choked_voice->active) {
            m_primary_voice->fade_gain = 0.0f;
            m_primary_voice->fade_delta = +1.0f / static_cast<float>(kMicroFadeFrames);
        } else {
            m_primary_voice->fade_gain = 1.0f;
            m_primary_voice->fade_delta = 0.0f;
        }

        if (slot_info.clip) {
            auto& st = m_primary_voice->streamer;
            st.set_clip(slot_info.clip);
            st.set_playback_mode(slot_info.playback_mode);
            st.set_pitch_semitones(slot_info.pitch_semitones);
            st.set_speed_ratio(slot_info.speed_ratio);
            st.set_loop(slot_info.loop);
            st.set_reverse(slot_info.reverse);
            st.set_bar_length(slot_info.bar_length);
            st.set_loop_range(slot_info.loop_start, slot_info.loop_end);

            const auto* clk = clock ? clock : m_timeline_clock;
            if (legato && clk) {
                // Calculate musical phase from timeline clock
                const double spb = clk->samples_per_beat();
                const uint64_t cur_pos = clk->sample_position() + sample_offset;
                const double cur_beats = (spb > 0.0) ? (static_cast<double>(cur_pos) / spb) : 0.0;
                const uint32_t len = (slot_info.loop_end > slot_info.loop_start)
                                   ? (slot_info.loop_end - slot_info.loop_start)
                                   : slot_info.clip->num_frames();
                double loop_bars = static_cast<double>(slot_info.bar_length);
                if (loop_bars <= 0.0) {
                    const uint32_t clip_sr = slot_info.clip->sample_rate();
                    const double dur_sec = (clip_sr > 0) ? (static_cast<double>(len) / static_cast<double>(clip_sr)) : 2.0;
                    const double clip_bpm = (slot_info.clip->bpm() > 0.0) ? slot_info.clip->bpm() : clk->bpm();
                    const double clip_beats = (dur_sec / 60.0) * clip_bpm;
                    loop_bars = std::max(0.25, std::round(clip_beats / 4.0));
                }
                const double loop_beats = loop_bars * 4.0;
                double phase_in_loop = std::fmod(cur_beats, loop_beats);
                if (phase_in_loop < 0.0) phase_in_loop += loop_beats;
                const double norm_phase = (loop_beats > 0.0) ? (phase_in_loop / loop_beats) : 0.0;

                double target_ph = static_cast<double>(slot_info.loop_start) + norm_phase * static_cast<double>(len);
                st.set_playhead(target_ph);
            } else {
                st.set_playhead(static_cast<double>(slot_info.loop_start));
            }
        }

        // 3. Trigger StepSequencer if this slot has a MIDI pattern
        if (slot_info.has_midi_pattern && m_associated_sequencer) {
            m_associated_sequencer->queue_pattern_switch(slot_info.midi_pattern_idx, PatternSwitchMode::Immediate);
        }

        // 4. Update states
        slot_info.state.store(SlotPlayState::Playing, std::memory_order_relaxed);
        m_current_slot.store(target_slot, std::memory_order_relaxed);
        m_queued_slot.store(-1, std::memory_order_relaxed);
        m_queued_action.store(QueuedAction::None, std::memory_order_relaxed);
        m_is_active.store(true, std::memory_order_relaxed);
    }

    // Executes stop with micro-fade out
    void execute_stop() noexcept {
        int32_t cur = m_current_slot.load(std::memory_order_relaxed);
        if (cur >= 0 && cur < static_cast<int32_t>(kMaxSlots)) {
            m_slots[cur].state.store(SlotPlayState::Stopped, std::memory_order_relaxed);
        }

        if (m_primary_voice->active) {
            m_primary_voice->fade_delta = -1.0f / static_cast<float>(kMicroFadeFrames);
        }

        if (m_associated_sequencer) {
            m_associated_sequencer->stop();
        }

        m_current_slot.store(-1, std::memory_order_relaxed);
        m_queued_slot.store(-1, std::memory_order_relaxed);
        m_queued_action.store(QueuedAction::None, std::memory_order_relaxed);
    }

    // Renders active and choked voices across a sub-chunk [0 .. chunk_frames)
    void render_voices_chunk(Sample* dst_l, Sample* dst_r, uint32_t chunk_frames,
                             const clock::TimelineClock& clock) noexcept {
        if (chunk_frames == 0) return;

        // 1. Render Choked Voice (Fading Out)
        if (m_choked_voice->active) {
            Sample tmp_l[2048];
            Sample tmp_r[2048];
            const uint32_t n = std::min(chunk_frames, 2048u);
            m_choked_voice->streamer.render(tmp_l, tmp_r, n, clock);

            for (uint32_t i = 0; i < n; ++i) {
                float g = std::max(0.0f, m_choked_voice->fade_gain);
                dst_l[i] += tmp_l[i] * g;
                dst_r[i] += tmp_r[i] * g;

                m_choked_voice->fade_gain += m_choked_voice->fade_delta;
                if (m_choked_voice->fade_gain <= 0.0f) {
                    m_choked_voice->active = false;
                    m_choked_voice->fade_gain = 0.0f;
                    m_choked_voice->fade_delta = 0.0f;
                    break;
                }
            }
        }

        // 2. Render Primary Voice (Active / Fading In)
        if (m_primary_voice->active) {
            if (m_primary_voice->fade_delta == 0.0f && m_primary_voice->fade_gain >= 1.0f) {
                // Steady state: render directly with additive summing
                Sample tmp_l[2048];
                Sample tmp_r[2048];
                const uint32_t n = std::min(chunk_frames, 2048u);
                m_primary_voice->streamer.render(tmp_l, tmp_r, n, clock);

                for (uint32_t i = 0; i < n; ++i) {
                    dst_l[i] += tmp_l[i];
                    dst_r[i] += tmp_r[i];
                }
            } else {
                // Fading in
                Sample tmp_l[2048];
                Sample tmp_r[2048];
                const uint32_t n = std::min(chunk_frames, 2048u);
                m_primary_voice->streamer.render(tmp_l, tmp_r, n, clock);

                for (uint32_t i = 0; i < n; ++i) {
                    float g = std::clamp(m_primary_voice->fade_gain, 0.0f, 1.0f);
                    dst_l[i] += tmp_l[i] * g;
                    dst_r[i] += tmp_r[i] * g;

                    m_primary_voice->fade_gain += m_primary_voice->fade_delta;
                    if (m_primary_voice->fade_gain >= 1.0f) {
                        m_primary_voice->fade_gain = 1.0f;
                        m_primary_voice->fade_delta = 0.0f;
                    } else if (m_primary_voice->fade_gain <= 0.0f && m_primary_voice->fade_delta < 0.0f) {
                        // Faded out to stop
                        m_primary_voice->active = false;
                        m_primary_voice->fade_gain = 0.0f;
                        m_primary_voice->fade_delta = 0.0f;
                        break;
                    }
                }
            }
        }
    }

    std::array<ClipSlot, kMaxSlots> m_slots{};

    Voice m_voice_a;
    Voice m_voice_b;
    Voice* m_primary_voice;
    Voice* m_choked_voice;

    StepSequencer* m_associated_sequencer{nullptr};
    const clock::TimelineClock* m_timeline_clock{nullptr};

    std::atomic<int32_t> m_current_slot{-1};
    std::atomic<int32_t> m_queued_slot{-1};
    std::atomic<QueuedAction> m_queued_action{QueuedAction::None};
    std::atomic<LaunchQuantize> m_queued_quantize{LaunchQuantize::Bar};
    std::atomic<bool> m_queued_legato{false};

    std::atomic<bool> m_is_active{false};
    float m_sample_rate{48000.0f};
};

} // namespace audio_core::sequencer
