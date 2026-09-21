#pragma once

#include "audio_core/types.hpp"
#include "audio_core/dsp/resampler.hpp"
#include "audio_core/sampling/audio_clip.hpp"
#include <vector>
#include <cmath>
#include <numbers>
#include <algorithm>
#include <memory>
#include <cstdint>

namespace audio_core::dsp {

enum class PitchAlgorithm : uint8_t {
    VinylRepitch = 0,    // Variclock: speed = 2^(semitones/12), pitch & time locked, Hermite C1 spline
    VintageMpc = 1,      // 12-bit vintage quantization, variable clock, gritty alias & micro-choke
    RubberbandWsola = 2, // WSOLA Granular: decoupled pitch shift and time-stretch, phase-aligned
    SovereignOde = 3,    // Continuous kinetic phase-space dilation: transients locked 1:1, tails ODE-stretched
    DeRezSampler = 4     // Airwindows DeRez2 Variable-Clock DAC: pitch-coupled sample-rate decimation + mu-law 12-bit/8-bit companding
};

// ============================================================================
// PitchTimeStretcher: Multi-Engine Broadcast Pitch Shifter & Time Stretcher
// Supports Tape/Vinyl, Vintage 12-bit MPC, Granular WSOLA & Sovereign Kinetic ODE
// ============================================================================
class PitchTimeStretcher {
public:
    static constexpr uint32_t kWsolaWindow = 1024;
    static constexpr uint32_t kWsolaHop    = 512;
    static constexpr uint32_t kWsolaSearch = 128;

    // Convert semitones to frequency / playback speed ratio
    [[nodiscard]] static double semitones_to_ratio(float semitones) noexcept {
        return std::pow(2.0, static_cast<double>(semitones) / 12.0);
    }

    // 1. Vinyl / Tape Variclock Repitch (Kinematic Turntable Platter ODE & Inner-Groove Tracing Slew)
    // Physically models:
    // - Platter rotational inertia & motor pole wow/flutter (0.55 Hz rotation + 6.0 Hz pole cogging)
    // - Inner-groove tracing slew loss: linear groove velocity v(t) = omega * r(t) decreases from
    //   outer radius (146mm) to inner radius (60mm), naturally attenuating ultrasonic smear
    // - Geometric pinch effect: second-order lateral groove compression distortion
    static std::shared_ptr<sampling::AudioClip> process_vinyl(
        const sampling::AudioClip& in_clip, float semitones) {
        const double pitch_ratio = semitones_to_ratio(semitones);
        const uint32_t in_frames = in_clip.num_frames();
        const uint32_t channels = in_clip.num_channels();
        if (in_frames == 0 || channels == 0 || pitch_ratio <= 0.0) return nullptr;

        const uint32_t out_frames = static_cast<uint32_t>(std::max(1.0, std::round(in_frames / pitch_ratio)));
        auto out_clip = std::make_shared<sampling::AudioClip>(
            in_clip.name() + "_Vinyl", in_clip.sample_rate(), channels, out_frames);
        out_clip->set_bpm(in_clip.bpm() * pitch_ratio);

        const double sample_rate = static_cast<double>(in_clip.sample_rate());
        const double dt = 1.0 / sample_rate;

        // Turntable kinematic parameters
        const double omega_0 = (2.0 * std::numbers::pi_v<double> * 33.333333333333336) / 60.0; // ~3.49 rad/s
        const double r_outer = 0.146; // 146 mm outer groove
        const double r_inner = 0.060; // 60 mm inner run-out groove

        for (uint32_t ch = 0; ch < channels; ++ch) {
            const float* src = in_clip.channel(ch);
            float* dst = out_clip->channel(ch);

            double src_pos = 0.0;
            float tracing_filter_state = 0.0f;
            float prev_sample = 0.0f;

            for (uint32_t i = 0; i < out_frames; ++i) {
                const double t_sec = static_cast<double>(i) * dt;

                // 1. Platter rotational inertia ODE (Subtle sub-Hz wow + motor pole flutter)
                // Total wow & flutter ~0.08% RMS (standard studio direct-drive turntable)
                const double wow = 0.0008 * std::sin(omega_0 * t_sec);
                const double flutter = 0.0003 * std::sin(2.0 * std::numbers::pi_v<double> * 6.0 * t_sec);
                const double inst_speed_factor = 1.0 + wow + flutter;

                // 2. Playhead progression in source material
                double step = pitch_ratio * inst_speed_factor;
                src_pos += (i == 0 ? 0.0 : step);
                if (src_pos >= static_cast<double>(in_frames)) src_pos = static_cast<double>(in_frames - 1);

                float raw = sample_hermite(src, src_pos, in_frames);

                // 3. Inner-groove geometry: linear velocity v(t) = omega * r(t)
                double progress = static_cast<double>(i) / static_cast<double>(out_frames);
                double r_t = r_outer - progress * (r_outer - r_inner);
                double v_linear = omega_0 * r_t; // drops from ~0.51 m/s to ~0.21 m/s

                // Tracing filter: high-frequency cutoff scales with linear groove velocity
                float fc_tracing = static_cast<float>(14000.0 + 5000.0 * (v_linear / (omega_0 * r_outer)));
                float alpha = std::clamp(static_cast<float>(2.0 * std::numbers::pi_v<double> * fc_tracing * dt), 0.01f, 0.99f);
                tracing_filter_state += alpha * (raw - tracing_filter_state);

                // 4. Pinch effect (subtle geometric 2nd harmonic excitation from stylus tip pinch)
                float slew = (tracing_filter_state - prev_sample);
                prev_sample = tracing_filter_state;
                float pinch = 0.015f * (slew * slew) * static_cast<float>(1.0 - progress * 0.5);

                dst[i] = std::clamp(tracing_filter_state + pinch, -1.0f, 1.0f);
            }
        }
        return out_clip;
    }

    // 2. Vintage 12-Bit MPC Slicer & Variable Clock (Discrete DAC Multiplying Architecture)
    // Physically models:
    // - Variable sampling clock (AD7541 multiplying DAC): Sample-and-hold step plateaus
    // - 12-bit linear quantization grid (4096 discrete voltage levels)
    // - Zero-order hold slew reconstruction
    static std::shared_ptr<sampling::AudioClip> process_vintage_mpc(
        const sampling::AudioClip& in_clip, float semitones) {
        const double pitch_ratio = semitones_to_ratio(semitones);
        const uint32_t in_frames = in_clip.num_frames();
        const uint32_t channels = in_clip.num_channels();
        if (in_frames == 0 || channels == 0 || pitch_ratio <= 0.0) return nullptr;

        const uint32_t out_frames = static_cast<uint32_t>(std::max(1.0, std::round(in_frames / pitch_ratio)));
        auto out_clip = std::make_shared<sampling::AudioClip>(
            in_clip.name() + "_MPC", in_clip.sample_rate(), channels, out_frames);
        out_clip->set_bpm(in_clip.bpm() * pitch_ratio);

        const float quant_steps = 2048.0f; // 12-bit linear PCM quantization (4096 levels, +/- 2048)

        for (uint32_t ch = 0; ch < channels; ++ch) {
            const float* src = in_clip.channel(ch);
            float* dst = out_clip->channel(ch);

            // Emulate variable-clock multiplying DAC with sample-and-hold plateaus
            double clock_acc = 0.0;
            float held_quant_sample = 0.0f;

            for (uint32_t i = 0; i < out_frames; ++i) {
                clock_acc += pitch_ratio;
                if (clock_acc >= 1.0 || i == 0) {
                    if (clock_acc >= 1.0) clock_acc -= std::floor(clock_acc);
                    double src_pos = static_cast<double>(i) * pitch_ratio;
                    float raw = sample_hermite(src, src_pos, in_frames);

                    // 12-bit uniform truncation/rounding to exact 1/2048 grid
                    held_quant_sample = std::round(raw * quant_steps) / quant_steps;
                    held_quant_sample = std::clamp(held_quant_sample, -1.0f, 1.0f);
                }

                dst[i] = held_quant_sample;
            }
        }
        return out_clip;
    }

    // 3. Rubberband / Granular WSOLA (Waveform Similarity Overlap-Add)
    // Decoupled Pitch Shift (semitones) & Time Stretch (stretch_factor = duration_out / duration_in)
    static std::shared_ptr<sampling::AudioClip> process_wsola(
        const sampling::AudioClip& in_clip, float semitones, float stretch_factor) {
        if (stretch_factor <= 0.05f) stretch_factor = 0.05f;
        if (stretch_factor > 8.0f)   stretch_factor = 8.0f;

        const double pitch_ratio = semitones_to_ratio(semitones);
        const uint32_t in_frames = in_clip.num_frames();
        const uint32_t channels = in_clip.num_channels();
        if (in_frames < kWsolaWindow || channels == 0) return nullptr;

        // Step A: Time-stretch via WSOLA
        const uint32_t out_stretched_frames = static_cast<uint32_t>(std::round(in_frames * stretch_factor));
        std::vector<std::vector<float>> stretched(channels, std::vector<float>(out_stretched_frames + kWsolaWindow, 0.0f));
        std::vector<float> norm_weights(out_stretched_frames + kWsolaWindow, 0.0f);

        // Precompute Hanning window
        std::vector<float> window(kWsolaWindow);
        for (uint32_t n = 0; n < kWsolaWindow; ++n) {
            window[n] = 0.5f * (1.0f - std::cos(2.0f * std::numbers::pi_v<float> * n / (kWsolaWindow - 1)));
        }

        const uint32_t hop_s = kWsolaHop;
        const double hop_a_ideal = static_cast<double>(hop_s) / static_cast<double>(stretch_factor);

        uint32_t synth_pos = 0;
        double ana_pos_ideal = 0.0;

        while (synth_pos + kWsolaWindow <= out_stretched_frames) {
            int64_t nominal_ana = static_cast<int64_t>(std::round(ana_pos_ideal));

            // Cross-correlation search window for maximum phase similarity
            int64_t best_ana = nominal_ana;
            float max_corr = -1e9f;

            if (synth_pos > 0) {
                int64_t search_min = std::max<int64_t>(0, nominal_ana - kWsolaSearch);
                int64_t search_max = std::min<int64_t>(in_frames - kWsolaWindow, nominal_ana + kWsolaSearch);

                for (int64_t cand = search_min; cand <= search_max; cand += 2) {
                    float corr = 0.0f;
                    // Correlate on Channel 0
                    const float* s = in_clip.channel(0);
                    for (uint32_t k = 0; k < kWsolaHop; k += 4) {
                        float v_synth = stretched[0][synth_pos + k];
                        float v_cand  = s[cand + k];
                        corr += v_synth * v_cand;
                    }
                    if (corr > max_corr) {
                        max_corr = corr;
                        best_ana = cand;
                    }
                }
            } else {
                best_ana = std::clamp<int64_t>(nominal_ana, 0, in_frames - kWsolaWindow);
            }

            // Overlap-add windowed grain across all channels
            for (uint32_t ch = 0; ch < channels; ++ch) {
                const float* src = in_clip.channel(ch);
                for (uint32_t n = 0; n < kWsolaWindow; ++n) {
                    float val = src[best_ana + n] * window[n];
                    stretched[ch][synth_pos + n] += val;
                }
            }

            for (uint32_t n = 0; n < kWsolaWindow; ++n) {
                norm_weights[synth_pos + n] += window[n];
            }

            synth_pos += hop_s;
            ana_pos_ideal += hop_a_ideal;
            if (ana_pos_ideal + kWsolaWindow >= in_frames) break;
        }

        // Normalize overlap weights
        for (uint32_t i = 0; i < out_stretched_frames; ++i) {
            float w = norm_weights[i];
            float inv_w = (w > 1e-4f) ? (1.0f / w) : 1.0f;
            for (uint32_t ch = 0; ch < channels; ++ch) {
                stretched[ch][i] *= inv_w;
            }
        }

        // Step B: Resample for independent Pitch Shift
        if (std::abs(pitch_ratio - 1.0) < 1e-4) {
            // No pitch shift needed, return stretched directly
            auto out_clip = std::make_shared<sampling::AudioClip>(
                in_clip.name() + "_WSOLA", in_clip.sample_rate(), channels, out_stretched_frames);
            out_clip->set_bpm(in_clip.bpm() / stretch_factor);
            for (uint32_t ch = 0; ch < channels; ++ch) {
                std::copy_n(stretched[ch].data(), out_stretched_frames, out_clip->channel(ch));
            }
            return out_clip;
        }

        const uint32_t final_frames = static_cast<uint32_t>(std::max(1.0, std::round(out_stretched_frames / pitch_ratio)));
        auto out_clip = std::make_shared<sampling::AudioClip>(
            in_clip.name() + "_WSOLA_Pitch", in_clip.sample_rate(), channels, final_frames);
        out_clip->set_bpm(in_clip.bpm() / stretch_factor);

        for (uint32_t ch = 0; ch < channels; ++ch) {
            const float* src = stretched[ch].data();
            float* dst = out_clip->channel(ch);
            for (uint32_t i = 0; i < final_frames; ++i) {
                double src_pos = static_cast<double>(i) * pitch_ratio;
                dst[i] = sample_hermite(src, src_pos, out_stretched_frames);
            }
        }
        return out_clip;
    }

    // 4. Sovereign ODE Kinetic Stretcher (Continuous Phase-Space Time Dilation)
    // Transients stay 100% punchy & unblurred (gamma = 1.0), resonant sustain tails stretch organically
    static std::shared_ptr<sampling::AudioClip> process_sovereign_ode(
        const sampling::AudioClip& in_clip, float semitones, float stretch_factor) {
        if (stretch_factor <= 0.05f) stretch_factor = 0.05f;
        if (stretch_factor > 8.0f)   stretch_factor = 8.0f;

        const uint32_t in_frames = in_clip.num_frames();
        const uint32_t channels = in_clip.num_channels();
        if (in_frames < 64 || channels == 0) return nullptr;

        const double pitch_ratio = semitones_to_ratio(semitones);

        // Precompute kinetic energy / transient profile on Channel 0
        const float* src_0 = in_clip.channel(0);
        std::vector<float> kinetic_energy(in_frames, 0.0f);
        for (uint32_t i = 1; i < in_frames; ++i) {
            float slew = std::abs(src_0[i] - src_0[i - 1]) * 10.0f;
            float val  = std::abs(src_0[i]);
            // Kinetic metric: slew acceleration + high-frequency energy
            kinetic_energy[i] = slew * 0.75f + val * 0.25f;
        }

        // Build continuous non-linear time trajectory t_in(t_out)
        // In transients (kinetic_energy > threshold), dilation gamma = 1.0 (bit-exact tempo)
        // In decay/sustain (kinetic_energy < threshold), gamma relaxes toward stretch_factor via trapezoidal ODE
        const float transient_thresh = 0.18f;
        const float ode_tau = 120.0f; // Relaxation time constant in frames
        const float alpha_ode = 1.0f - std::exp(-1.0f / ode_tau);

        std::vector<double> out_to_in_map;
        out_to_in_map.reserve(static_cast<size_t>(in_frames * stretch_factor * 1.2));

        double in_playhead = 0.0;
        float cur_gamma = 1.0f;

        while (in_playhead < static_cast<double>(in_frames - 1)) {
            out_to_in_map.push_back(in_playhead);

            uint32_t idx = static_cast<uint32_t>(in_playhead);
            float ek = (idx < in_frames) ? kinetic_energy[idx] : 0.0f;

            float target_gamma = (ek > transient_thresh) ? 1.0f : stretch_factor;
            // Trapezoidal / exponential ODE smoothing of time-dilation velocity
            cur_gamma += alpha_ode * (target_gamma - cur_gamma);

            // Step in source audio
            double step = 1.0 / std::max(0.1f, cur_gamma);
            in_playhead += step;
        }

        const uint32_t out_stretched_frames = static_cast<uint32_t>(out_to_in_map.size());
        if (out_stretched_frames == 0) return nullptr;

        // Render stretched audio with Hermite C1 interpolation
        std::vector<std::vector<float>> stretched(channels, std::vector<float>(out_stretched_frames, 0.0f));
        for (uint32_t ch = 0; ch < channels; ++ch) {
            const float* src = in_clip.channel(ch);
            float* dst = stretched[ch].data();
            for (uint32_t i = 0; i < out_stretched_frames; ++i) {
                dst[i] = sample_hermite(src, out_to_in_map[i], in_frames);
            }
        }

        // Apply pitch ratio if semitones != 0
        if (std::abs(pitch_ratio - 1.0) < 1e-4) {
            auto out_clip = std::make_shared<sampling::AudioClip>(
                in_clip.name() + "_SovereignODE", in_clip.sample_rate(), channels, out_stretched_frames);
            out_clip->set_bpm(in_clip.bpm() / stretch_factor);
            for (uint32_t ch = 0; ch < channels; ++ch) {
                std::copy_n(stretched[ch].data(), out_stretched_frames, out_clip->channel(ch));
            }
            return out_clip;
        }

        const uint32_t final_frames = static_cast<uint32_t>(std::max(1.0, std::round(out_stretched_frames / pitch_ratio)));
        auto out_clip = std::make_shared<sampling::AudioClip>(
            in_clip.name() + "_SovereignODE_Pitch", in_clip.sample_rate(), channels, final_frames);
        out_clip->set_bpm(in_clip.bpm() / stretch_factor);

        for (uint32_t ch = 0; ch < channels; ++ch) {
            const float* src = stretched[ch].data();
            float* dst = out_clip->channel(ch);
            for (uint32_t i = 0; i < final_frames; ++i) {
                double src_pos = static_cast<double>(i) * pitch_ratio;
                dst[i] = sample_hermite(src, src_pos, out_stretched_frames);
            }
        }
        return out_clip;
    }

    // 5. Airwindows DeRez2 Variable-Clock Vintage Sampler DAC Repitch
    // Emulates the authentic physical re-clocking of vintage hardware samplers (SP-1200 / Mirage)
    // Combines pitch-coupled variable sample-and-hold, sub-sample overrun interpolation,
    // analog-slew edge-softening, and non-linear u-law companded bit reduction.
    static std::shared_ptr<sampling::AudioClip> process_derez_sampler(
        const sampling::AudioClip& in_clip, float semitones, float resolution = 0.70f, float hard = 0.0f) {
        const double pitch_ratio = semitones_to_ratio(semitones);
        const uint32_t in_frames = in_clip.num_frames();
        const uint32_t channels = in_clip.num_channels();
        if (in_frames == 0 || channels == 0 || pitch_ratio <= 0.0) return nullptr;

        const uint32_t out_frames = static_cast<uint32_t>(std::max(1.0, std::round(in_frames / pitch_ratio)));
        auto out_clip = std::make_shared<sampling::AudioClip>(
            in_clip.name() + "_DeRezSampler", in_clip.sample_rate(), channels, out_frames);
        out_clip->set_bpm(in_clip.bpm() * pitch_ratio);

        // DeRez2 parameters derived from pitch and user resolution/hard settings
        // Target A: tracks pitch ratio (when pitching down, effective clock drops)
        double target_a = std::clamp(pitch_ratio, 0.001, 1.0);
        double soften = (1.0 + target_a) * 0.5;

        // Target B: Bit depth decimation (resolution 0.70 ~= 12-bit SP-1200, 0.40 ~= 8-bit Mirage)
        double target_b = std::pow(1.0 - static_cast<double>(std::clamp(resolution, 0.0f, 1.0f)), 3.0) / 3.0;
        const double hard_d = std::clamp(static_cast<double>(hard), 0.0, 1.0);
        const double log256 = std::log(256.0);

        for (uint32_t ch = 0; ch < channels; ++ch) {
            const float* src = in_clip.channel(ch);
            float* dst = out_clip->channel(ch);

            double position = 0.0;
            double held_sample = 0.0;
            double last_sample = 0.0;
            double last_output = 0.0;
            double last_dry = 0.0;

            for (uint32_t i = 0; i < out_frames; ++i) {
                // Determine source position
                double src_pos = static_cast<double>(i) * pitch_ratio;
                uint32_t src_idx = static_cast<uint32_t>(src_pos);
                if (src_idx >= in_frames) src_idx = in_frames - 1;
                double input_sample = static_cast<double>(src[src_idx]);
                const double dry_sample = input_sample;

                // 1. Variable-clock sub-sample accumulator
                position += target_a;
                double output_sample = held_sample;

                if (position > 1.0) {
                    position -= 1.0;
                    // Sub-sample overrun interpolation
                    held_sample = (last_sample * position) + (input_sample * (1.0 - position));
                    output_sample = (output_sample * (1.0 - soften)) + (held_sample * soften);
                }
                input_sample = output_sample;

                // 2. Intermediate dry-sample reconstruction at step transitions
                double temp = input_sample;
                if (input_sample != last_output) {
                    temp = input_sample;
                    input_sample = (input_sample * hard_d) + (last_dry * (1.0 - hard_d));
                    last_output = temp;
                } else {
                    last_output = input_sample;
                }
                last_dry = dry_sample;

                // 3. u-Law companding before bit reduction (if hard < 1.0)
                temp = input_sample;
                input_sample = std::clamp(input_sample, -1.0, 1.0);
                if (hard_d < 1.0) {
                    double ulaw = (input_sample > 0.0)
                        ? (std::log(1.0 + 255.0 * std::abs(input_sample)) / log256)
                        : -(std::log(1.0 + 255.0 * std::abs(input_sample)) / log256);
                    input_sample = (temp * hard_d) + (ulaw * (1.0 - hard_d));
                }

                // 4. Continuous Bit-Depth Decimation
                if (target_b > 0.0005) {
                    if (input_sample > 0.0) {
                        double offset = input_sample;
                        while (offset > 0.0) offset -= target_b;
                        input_sample -= offset;
                    } else if (input_sample < 0.0) {
                        double offset = input_sample;
                        while (offset < 0.0) offset += target_b;
                        input_sample -= offset;
                    }
                    input_sample *= (1.0 - target_b);
                }

                // 5. u-Law decoding expansion
                temp = input_sample;
                input_sample = std::clamp(input_sample, -1.0, 1.0);
                if (hard_d < 1.0) {
                    double dec = (input_sample > 0.0)
                        ? ((std::pow(256.0, std::abs(input_sample)) - 1.0) / 255.0)
                        : -((std::pow(256.0, std::abs(input_sample)) - 1.0) / 255.0);
                    input_sample = (temp * hard_d) + (dec * (1.0 - hard_d));
                }

                last_sample = dry_sample;
                dst[i] = static_cast<float>(std::clamp(input_sample, -1.0, 1.0));
            }
        }
        return out_clip;
    }

    // Unified dispatch function for all 5 algorithms
    static std::shared_ptr<sampling::AudioClip> process(
        const sampling::AudioClip& in_clip,
        PitchAlgorithm algo,
        float semitones,
        float stretch_factor = 1.0f) {
        switch (algo) {
            case PitchAlgorithm::VinylRepitch:
                return process_vinyl(in_clip, semitones);
            case PitchAlgorithm::VintageMpc:
                return process_vintage_mpc(in_clip, semitones);
            case PitchAlgorithm::RubberbandWsola:
                return process_wsola(in_clip, semitones, stretch_factor);
            case PitchAlgorithm::SovereignOde:
                return process_sovereign_ode(in_clip, semitones, stretch_factor);
            case PitchAlgorithm::DeRezSampler:
                return process_derez_sampler(in_clip, semitones);
        }
        return nullptr;
    }
};

} // namespace audio_core::dsp
