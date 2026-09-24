#pragma once

#include "audio_core/types.hpp"
#include "audio_core/clock/timeline_clock.hpp"
#include "audio_core/sampling/audio_clip.hpp"
#include "audio_core/dsp/resampler.hpp"
#include "audio_core/dsp/liquid_ode.hpp"
#include "audio_core/sampling/wsola_streamer.hpp"
#include <memory>
#include <atomic>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <cstring>

namespace audio_core::sampling {

// ============================================================================
// PlaybackMode: Real-Time Streamer Operating Modes
// ============================================================================
enum class PlaybackMode : uint8_t {
    Free = 0,               // Standard playback: manual pitch semitones & speed ratio
    BeatSyncRepitch = 1,    // Continuous tape vari-speed: loop tempo dynamically matches TimelineClock BPM
    TransportPhaseLock = 2, // Hard phase-lock: loop playhead matches transport bar/beat phase exactly
    ReverseFree = 3,        // Reverse playback
    BeatSyncTimeStretch = 4,// Continuous WSOLA time-stretch: tempo matches TimelineClock, pitch locked
    PitchShiftWsola = 5     // Continuous WSOLA pitch-shift: pitch transposed, tempo locked
};

enum class TapeMotorState : uint8_t {
    Running = 0,
    Stopping = 1,   // Tape stop deceleration curve active
    Stopped = 2,    // Tape completely stationary
    Starting = 3    // Tape start acceleration curve active
};

// ============================================================================
// 4-point loop-range wrapped Hermite sample fetch
// ============================================================================
[[nodiscard]] inline float sample_hermite_range_wrapped(const float* buffer, double position,
                                                        int64_t loop_start, int64_t loop_len) noexcept {
    if (!buffer || loop_len <= 0) return 0.0f;
    double rel_pos = position - static_cast<double>(loop_start);
    rel_pos = std::fmod(rel_pos, static_cast<double>(loop_len));
    if (rel_pos < 0.0) rel_pos += static_cast<double>(loop_len);

    int64_t idx = static_cast<int64_t>(std::floor(rel_pos));
    float frac = static_cast<float>(rel_pos - static_cast<double>(idx));

    auto fetch_wrap = [&](int64_t i) -> float {
        int64_t w = i % loop_len;
        if (w < 0) w += loop_len;
        return buffer[loop_start + w];
    };

    float y0 = fetch_wrap(idx - 1);
    float y1 = fetch_wrap(idx);
    float y2 = fetch_wrap(idx + 1);
    float y3 = fetch_wrap(idx + 2);

    return dsp::hermite_interpolate(y0, y1, y2, y3, frac);
}

// ============================================================================
// VariSpeedStreamer: Zero-Allocation Live Vari-Speed Resampler & Beat-Sync Engine
// Features:
// 1. Continuous C1 Hermite Spline Resampling (Arbitrary SR & Speed ratios)
// 2. Liquid Parameter Slew Limiting (Analog Tape Capstan Motor Inertia)
// 3. Dynamic Beat-Sync & Real-Time Tempo Matching (Vari-Speed Tape Lock)
// 4. Transport Bar-Phase Hard Locking
// 5. Bidirectional Reverse Playback & Sample-Accurate Loop Range Wrapping
// 6. Analog Tape Stop (Brake ODE) and Tape Start (Torque ODE) Ballistics
// ============================================================================
class VariSpeedStreamer {
public:
    explicit VariSpeedStreamer(float sample_rate = 48000.0f) noexcept
        : m_smoother(sample_rate, 15.0f) {
        reset();
    }

    void reset() noexcept {
        m_playhead.store(0.0, std::memory_order_relaxed);
        m_motor_state.store(TapeMotorState::Running, std::memory_order_relaxed);
        m_motor_progress = 0.0f;
        m_needs_smoother_reset = true;
        m_effective_ratio = 1.0f;
        m_scrub_dc_x_l = 0.0f;
        m_scrub_dc_y_l = 0.0f;
        m_scrub_dc_x_r = 0.0f;
        m_scrub_dc_y_r = 0.0f;
        m_wsola.reset();
    }

    void set_clip(std::shared_ptr<AudioClip> clip) noexcept {
        m_clip = std::move(clip);
        m_needs_smoother_reset = true;
        m_wsola.set_clip(m_clip);
    }
    [[nodiscard]] std::shared_ptr<AudioClip> clip() const noexcept { return m_clip; }
    [[nodiscard]] bool has_clip() const noexcept { return m_clip != nullptr; }

    // Playhead control
    [[nodiscard]] double playhead() const noexcept {
        return m_playhead.load(std::memory_order_relaxed);
    }
    void set_playhead(double ph) noexcept {
        m_playhead.store(ph, std::memory_order_relaxed);
        m_wsola.set_playhead(ph);
    }

    [[nodiscard]] WsolaStreamer& wsola() noexcept { return m_wsola; }
    [[nodiscard]] const WsolaStreamer& wsola() const noexcept { return m_wsola; }

    // Playback modes
    void set_playback_mode(PlaybackMode mode) noexcept {
        m_mode.store(mode, std::memory_order_relaxed);
    }
    [[nodiscard]] PlaybackMode playback_mode() const noexcept {
        return m_mode.load(std::memory_order_relaxed);
    }

    // Pitch & Speed
    void set_pitch_semitones(float st) noexcept {
        m_pitch_semitones.store(std::clamp(st, -48.0f, 48.0f), std::memory_order_relaxed);
    }
    [[nodiscard]] float pitch_semitones() const noexcept {
        return m_pitch_semitones.load(std::memory_order_relaxed);
    }

    void set_speed_ratio(float ratio) noexcept {
        m_speed_ratio.store(std::clamp(ratio, 0.01f, 16.0f), std::memory_order_relaxed);
    }
    [[nodiscard]] float speed_ratio() const noexcept {
        return m_speed_ratio.load(std::memory_order_relaxed);
    }

    void set_reverse(bool rev) noexcept {
        m_reverse.store(rev, std::memory_order_relaxed);
    }
    [[nodiscard]] bool is_reverse() const noexcept {
        return m_reverse.load(std::memory_order_relaxed);
    }

    void set_loop(bool loop) noexcept {
        m_loop.store(loop, std::memory_order_relaxed);
    }
    [[nodiscard]] bool is_loop() const noexcept {
        return m_loop.load(std::memory_order_relaxed);
    }

    void set_loop_range(uint32_t start_frame, uint32_t end_frame) noexcept {
        m_loop_start.store(start_frame, std::memory_order_relaxed);
        m_loop_end.store(end_frame, std::memory_order_relaxed);
    }
    [[nodiscard]] uint32_t loop_start() const noexcept { return m_loop_start.load(std::memory_order_relaxed); }
    [[nodiscard]] uint32_t loop_end() const noexcept { return m_loop_end.load(std::memory_order_relaxed); }

    // Capstan inertia (ms)
    void set_capstan_inertia_ms(float ms) noexcept {
        m_capstan_inertia_ms.store(std::clamp(ms, 0.0f, 1000.0f), std::memory_order_relaxed);
    }
    [[nodiscard]] float capstan_inertia_ms() const noexcept {
        return m_capstan_inertia_ms.load(std::memory_order_relaxed);
    }

    // Explicit loop bar count for beat sync (e.g. 1.0, 2.0, 4.0, 8.0 bars)
    // 0.0 = auto-calculate from clip BPM and frame length
    void set_bar_length(float bars) noexcept {
        m_bar_length.store(std::clamp(bars, 0.0f, 64.0f), std::memory_order_relaxed);
    }
    [[nodiscard]] float bar_length() const noexcept {
        return m_bar_length.load(std::memory_order_relaxed);
    }

    // Tape Stop / Start
    void trigger_tape_stop(float duration_sec = 0.5f) noexcept {
        m_motor_stop_duration_sec = std::clamp(duration_sec, 0.05f, 5.0f);
        m_motor_progress = 0.0f;
        m_motor_state.store(TapeMotorState::Stopping, std::memory_order_relaxed);
    }

    void trigger_tape_start(float duration_sec = 0.3f) noexcept {
        m_motor_start_duration_sec = std::clamp(duration_sec, 0.05f, 5.0f);
        m_motor_progress = 0.0f;
        m_motor_state.store(TapeMotorState::Starting, std::memory_order_relaxed);
    }

    [[nodiscard]] TapeMotorState motor_state() const noexcept {
        return m_motor_state.load(std::memory_order_relaxed);
    }

    [[nodiscard]] float effective_playback_ratio() const noexcept {
        return m_effective_ratio;
    }

    // Render with timeline clock
    void render(Sample* dst_l, Sample* dst_r, uint32_t frames, const clock::TimelineClock& clock) noexcept {
        const auto musical_pos = clock.position_snapshot();
        render(dst_l, dst_r, frames, clock.sample_rate(), clock.bpm(), clock.is_playing(),
               musical_pos.total_beats, clock.is_scrubbing(), clock.scrub_velocity());
    }

    // Render with raw parameters
    void render(Sample* dst_l, Sample* dst_r, uint32_t frames,
                uint32_t session_sr, double session_bpm, bool is_playing,
                double transport_total_beats = 0.0,
                bool is_scrubbing = false, double scrub_velocity = 1.0) noexcept {
        if (!dst_l || !dst_r || frames == 0) return;
        if (!m_clip || m_clip->num_frames() == 0 || m_clip->num_channels() == 0) {
            std::memset(dst_l, 0, frames * sizeof(Sample));
            std::memset(dst_r, 0, frames * sizeof(Sample));
            return;
        }

        const uint32_t clip_sr = m_clip->sample_rate();
        const uint32_t clip_total_frames = m_clip->num_frames();
        const float* src_l = m_clip->channel(0);
        const float* src_r = (m_clip->num_channels() > 1) ? m_clip->channel(1) : src_l;

        uint32_t l_start = m_loop_start.load(std::memory_order_relaxed);
        uint32_t l_end = m_loop_end.load(std::memory_order_relaxed);
        if (l_end == 0 || l_end > clip_total_frames || l_end <= l_start) {
            l_end = clip_total_frames;
        }
        const uint32_t loop_len = (l_end > l_start) ? (l_end - l_start) : clip_total_frames;

        const auto mode = m_mode.load(std::memory_order_relaxed);
        const bool loop = m_loop.load(std::memory_order_relaxed);
        const bool rev = m_reverse.load(std::memory_order_relaxed) || (mode == PlaybackMode::ReverseFree);

        // 1. Calculate nominal loop BPM and bar count
        double loop_bpm = m_clip->bpm();
        float user_bars = m_bar_length.load(std::memory_order_relaxed);
        if (user_bars > 0.0f) {
            const double dur_sec = static_cast<double>(loop_len) / static_cast<double>(clip_sr);
            if (dur_sec > 1e-4) {
                loop_bpm = (static_cast<double>(user_bars) * 4.0 / dur_sec) * 60.0;
            }
        } else if (loop_bpm <= 0.0) {
            const double dur_sec = static_cast<double>(loop_len) / static_cast<double>(clip_sr);
            if (dur_sec > 1e-4) {
                double best_bpm = 120.0;
                double best_dist = 1e9;
                for (int b : {1, 2, 4, 8, 16}) {
                    double cand_bpm = (b * 4.0 / dur_sec) * 60.0;
                    if (cand_bpm >= 60.0 && cand_bpm <= 180.0) {
                        double dist = std::abs(cand_bpm - 120.0);
                        if (dist < best_dist) {
                            best_dist = dist;
                            best_bpm = cand_bpm;
                            user_bars = static_cast<float>(b);
                        }
                    }
                }
                loop_bpm = best_bpm;
            }
        }

        // 1b. Real-Time WSOLA Time-Stretch & Decoupled Pitch Branch
        if (mode == PlaybackMode::BeatSyncTimeStretch || mode == PlaybackMode::PitchShiftWsola) {
            double start_ph = m_wsola.playhead();
            float base_pitch = m_pitch_semitones.load(std::memory_order_relaxed);
            if (m_clip && m_clip->is_envelope_enabled(ClipEnvelopeTarget::Pitch)) {
                double b = m_clip->frame_to_envelope_beat(start_ph, l_start, l_end, user_bars);
                float env_st = m_clip->envelope(ClipEnvelopeTarget::Pitch).evaluate_audio_sample(b);
                base_pitch += env_st;
            }
            m_wsola.set_clip(m_clip);
            m_wsola.set_loop(loop);
            m_wsola.set_loop_range(l_start, l_end);
            m_wsola.set_bar_length(user_bars);
            m_wsola.set_pitch_semitones(base_pitch);
            if (mode == PlaybackMode::BeatSyncTimeStretch) {
                m_wsola.set_beat_sync(true);
                m_wsola.set_stretch_factor(m_speed_ratio.load(std::memory_order_relaxed));
            } else {
                m_wsola.set_beat_sync(false);
                m_wsola.set_stretch_factor(m_speed_ratio.load(std::memory_order_relaxed));
            }
            m_wsola.render(dst_l, dst_r, frames, session_sr, session_bpm, is_playing || is_scrubbing);
            double end_ph = m_wsola.playhead();
            m_playhead.store(end_ph, std::memory_order_relaxed);
            m_effective_ratio = m_wsola.effective_stretch_ratio();

            // Apply active Clip-Relative Gain & Pan Envelopes
            if (m_clip && m_clip->has_active_envelopes()) {
                m_clip->apply_envelopes(dst_l, dst_r, frames, start_ph, end_ph, l_start, l_end, user_bars);
            }
            return;
        }

        // 2. Base Sample Rate Ratio
        double sr_ratio = (session_sr > 0)
            ? (static_cast<double>(clip_sr) / static_cast<double>(session_sr))
            : 1.0;

        // 3. Tempo-Matching Ratio
        double tempo_ratio = 1.0;
        if ((mode == PlaybackMode::BeatSyncRepitch || mode == PlaybackMode::TransportPhaseLock)
            && loop_bpm > 10.0 && session_bpm > 10.0) {
            tempo_ratio = session_bpm / loop_bpm;
        }

        // 4. Pitch Transpose & Speed Factors
        float semitones = m_pitch_semitones.load(std::memory_order_relaxed);
        double pitch_ratio = std::pow(2.0, static_cast<double>(semitones) / 12.0);
        double manual_speed = static_cast<double>(m_speed_ratio.load(std::memory_order_relaxed));

        // Target Playhead Step per session frame
        double scrub_factor = 1.0;
        if (is_scrubbing) {
            scrub_factor = (std::abs(scrub_velocity) > 1e-4) ? scrub_velocity : 1.0;
        }
        const double nominal_step = sr_ratio * tempo_ratio * pitch_ratio * manual_speed * scrub_factor * (rev ? -1.0 : 1.0);

        // 5. Transport Phase Lock Hard Sync
        if (mode == PlaybackMode::TransportPhaseLock && is_playing && loop_bpm > 10.0) {
            const double bars = (user_bars > 0.0f) ? static_cast<double>(user_bars) : 1.0;
            const double loop_beats = bars * 4.0;
            const double phase_beats = std::fmod(transport_total_beats, loop_beats);
            const double norm_phase = (phase_beats >= 0.0) ? (phase_beats / loop_beats) : (1.0 + phase_beats / loop_beats);
            const double target_ph = static_cast<double>(l_start) + norm_phase * static_cast<double>(loop_len);

            double cur_ph = m_playhead.load(std::memory_order_relaxed);
            double ph_diff = target_ph - cur_ph;
            if (std::abs(ph_diff) > static_cast<double>(session_sr) * 0.05) {
                m_playhead.store(target_ph, std::memory_order_relaxed);
            }
        }

        // 6. Tape Motor & Capstan Inertia Setup
        const float total_stop_frames = std::max(1.0f, m_motor_stop_duration_sec * static_cast<float>(session_sr));
        const float total_start_frames = std::max(1.0f, m_motor_start_duration_sec * static_cast<float>(session_sr));
        auto motor = m_motor_state.load(std::memory_order_relaxed);

        const float inertia_ms = m_capstan_inertia_ms.load(std::memory_order_relaxed);
        m_smoother.set_sample_rate(static_cast<float>(session_sr));
        if (inertia_ms > 0.001f) {
            m_smoother.set_transition_time_ms(inertia_ms);
        }

        if (m_needs_smoother_reset) {
            float init_factor = 1.0f;
            if (!is_scrubbing) {
                if (motor == TapeMotorState::Stopped) init_factor = 0.0f;
                else if (motor == TapeMotorState::Starting) init_factor = 0.0f;
            }
            m_smoother.reset(static_cast<float>(nominal_step * init_factor));
            m_needs_smoother_reset = false;
        }

        double ph = m_playhead.load(std::memory_order_relaxed);
        double start_ph = ph;
        const double d_start = static_cast<double>(l_start);
        const double d_end = static_cast<double>(l_end);
        const double d_len = static_cast<double>(loop_len);
        float last_active_step = static_cast<float>(nominal_step);

        // 7. Render Frame-by-Frame with Continuous Hermite Spline Interpolation & Motor Ballistics
        for (uint32_t i = 0; i < frames; ++i) {
            float motor_factor = 1.0f;
            if (is_scrubbing) {
                motor_factor = 1.0f;
            } else if (motor == TapeMotorState::Stopping) {
                m_motor_progress += 1.0f;
                if (m_motor_progress >= total_stop_frames) {
                    motor = TapeMotorState::Stopped;
                    m_motor_state.store(TapeMotorState::Stopped, std::memory_order_relaxed);
                    motor_factor = 0.0f;
                } else {
                    float t_norm = m_motor_progress / total_stop_frames;
                    float brake = (1.0f - t_norm);
                    motor_factor = brake * brake;
                }
            } else if (motor == TapeMotorState::Starting) {
                m_motor_progress += 1.0f;
                if (m_motor_progress >= total_start_frames) {
                    motor = TapeMotorState::Running;
                    m_motor_state.store(TapeMotorState::Running, std::memory_order_relaxed);
                    motor_factor = 1.0f;
                } else {
                    float t_norm = m_motor_progress / total_start_frames;
                    float accel = 1.0f - (1.0f - t_norm) * (1.0f - t_norm);
                    motor_factor = accel;
                }
            } else if (motor == TapeMotorState::Stopped) {
                motor_factor = 0.0f;
            }

            float pitch_env_mult = 1.0f;
            if (m_clip && m_clip->is_envelope_enabled(ClipEnvelopeTarget::Pitch)) {
                double b = m_clip->frame_to_envelope_beat(ph, l_start, l_end, user_bars);
                float env_st = m_clip->envelope(ClipEnvelopeTarget::Pitch).evaluate_audio_sample(b);
                if (std::abs(env_st) > 1e-4f) {
                    pitch_env_mult = std::pow(2.0f, env_st / 12.0f);
                }
            }

            const float sample_target_step = static_cast<float>(nominal_step * motor_factor * pitch_env_mult);
            float active_step = sample_target_step;
            if (!is_scrubbing && inertia_ms > 0.001f) {
                m_smoother.set_target(sample_target_step);
                active_step = m_smoother.process_sample();
            }
            last_active_step = active_step;

            if (loop) {
                if (ph >= d_end) {
                    ph = d_start + std::fmod(ph - d_start, d_len);
                } else if (ph < d_start) {
                    double rel = std::fmod(ph - d_start, d_len);
                    if (rel < 0.0) rel += d_len;
                    ph = d_start + rel;
                }
            } else {
                if (ph >= d_end || ph < d_start) {
                    dst_l[i] = 0.0f;
                    dst_r[i] = 0.0f;
                    ph += static_cast<double>(active_step);
                    continue;
                }
            }

            float raw_l = 0.0f;
            float raw_r = 0.0f;
            if (loop) {
                raw_l = sample_hermite_range_wrapped(src_l, ph, l_start, loop_len);
                raw_r = sample_hermite_range_wrapped(src_r, ph, l_start, loop_len);
            } else {
                raw_l = dsp::sample_hermite(src_l, ph, clip_total_frames);
                raw_r = dsp::sample_hermite(src_r, ph, clip_total_frames);
            }

            if (is_scrubbing) {
                // DJ-Style Granular Micro-Windowing & Sub-Bass Rumble Protection:
                // 1. Single-pole highpass / DC-blocker (R = 0.995 => ~25 Hz cutoff at 48kHz)
                //    eliminates stagnant DC offsets and infrasonic speaker excursions when crawling or reversing.
                const float y_l = raw_l - m_scrub_dc_x_l + 0.995f * m_scrub_dc_y_l;
                m_scrub_dc_x_l = raw_l;
                m_scrub_dc_y_l = y_l;

                const float y_r = raw_r - m_scrub_dc_x_r + 0.995f * m_scrub_dc_y_r;
                m_scrub_dc_x_r = raw_r;
                m_scrub_dc_y_r = y_r;

                // 2. Smooth velocity taper: micro-window transitions smoothly towards 0 when stationary (|v| -> 0)
                //    avoiding sudden rectangular step clicks when scrubbing stops or reverses.
                const float vel_abs = static_cast<float>(std::abs(scrub_velocity));
                const float g_vel = std::tanh(vel_abs / 0.12f);

                dst_l[i] = y_l * g_vel;
                dst_r[i] = y_r * g_vel;
            } else {
                m_scrub_dc_x_l = 0.0f;
                m_scrub_dc_y_l = 0.0f;
                m_scrub_dc_x_r = 0.0f;
                m_scrub_dc_y_r = 0.0f;
                dst_l[i] = raw_l;
                dst_r[i] = raw_r;
            }

            ph += static_cast<double>(active_step);
        }

        m_playhead.store(ph, std::memory_order_relaxed);
        m_effective_ratio = last_active_step;

        // Apply active Clip-Relative Gain & Pan Envelopes
        if (m_clip && m_clip->has_active_envelopes()) {
            m_clip->apply_envelopes(dst_l, dst_r, frames, start_ph, ph, l_start, l_end, user_bars);
        }
    }

private:
    std::shared_ptr<AudioClip> m_clip{nullptr};
    std::atomic<double> m_playhead{0.0};
    std::atomic<PlaybackMode> m_mode{PlaybackMode::Free};
    std::atomic<float> m_pitch_semitones{0.0f};
    std::atomic<float> m_speed_ratio{1.0f};
    std::atomic<bool> m_reverse{false};
    std::atomic<bool> m_loop{true};
    std::atomic<uint32_t> m_loop_start{0};
    std::atomic<uint32_t> m_loop_end{0};
    std::atomic<float> m_capstan_inertia_ms{0.0f}; // Default 0.0ms for instant sample-accurate testing
    std::atomic<float> m_bar_length{0.0f};

    std::atomic<TapeMotorState> m_motor_state{TapeMotorState::Running};
    float m_motor_progress{0.0f};
    float m_motor_stop_duration_sec{0.5f};
    float m_motor_start_duration_sec{0.3f};

    dsp::LiquidParameterSmoother m_smoother;
    bool m_needs_smoother_reset{true};
    float m_effective_ratio{1.0f};

    // Granular Micro-Windowed Jog & Scrub DC Blocker State
    float m_scrub_dc_x_l{0.0f};
    float m_scrub_dc_y_l{0.0f};
    float m_scrub_dc_x_r{0.0f};
    float m_scrub_dc_y_r{0.0f};

    WsolaStreamer m_wsola;
};

} // namespace audio_core::sampling
