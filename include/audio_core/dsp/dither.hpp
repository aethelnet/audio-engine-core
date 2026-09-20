#pragma once

#include "audio_core/types.hpp"
#include <cmath>
#include <cstdint>
#include <array>
#include <algorithm>

namespace audio_core::dsp {

// ============================================================================
// DitherType: Quantization Wordlength Reduction Dithering Modes
// Incorporating Airwindows & AES noise-shaping concepts:
// - None: Hard truncation / nearest rounding (benchmark baseline, generates harmonic distortion)
// - TPDF: Standard flat Triangular Probability Density Function (2 LSB white noise floor, no distortion)
// - PaulDither: Airwindows Paul Frindle highpassed TPDF (1 - z^-1), +6 dB/oct slope pushing dither
//               energy into ultrasonic frequencies (>15 kHz) leaving mid-frequencies pitch black
// - Dark: Airwindows de-emphasized lowpass dither ((1 + z^-1) * 0.5), rolling off high hiss for warm tape feel
// - NJAD: Airwindows Not Just Another Dither (Benford-law threshold noise shaping with zero-silence clamping)
// ============================================================================
enum class DitherType : uint8_t {
    None = 0,
    TPDF,
    PaulDither,
    Dark,
    NJAD
};

enum class BitDepth : uint8_t {
    Bit16 = 16,
    Bit24 = 24
};

class DitherEngine {
public:
    explicit DitherEngine(DitherType type = DitherType::TPDF) noexcept
        : m_type(type) {
        reset();
    }

    void set_type(DitherType type) noexcept {
        m_type = type;
    }

    [[nodiscard]] DitherType type() const noexcept {
        return m_type;
    }

    void reset() noexcept {
        m_rand_l = 0x12345678;
        m_rand_r = 0x87654321;
        m_prev_paul.fill(0.0f);
        m_prev_dark.fill(0.0f);
        m_njad_noise.fill(0.0);
        for (auto& ch_bins : m_njad_bins) {
            ch_bins.fill(0.0);
        }
    }

    // Process a single stereo frame to 16-bit signed PCM
    inline void process_sample_16(float in_l, float in_r, int16_t& out_l, int16_t& out_r) noexcept {
        out_l = static_cast<int16_t>(quantize_channel<BitDepth::Bit16>(in_l, 0));
        out_r = static_cast<int16_t>(quantize_channel<BitDepth::Bit16>(in_r, 1));
    }

    // Process a single stereo frame to 24-bit signed PCM
    inline void process_sample_24(float in_l, float in_r, int32_t& out_l, int32_t& out_r) noexcept {
        out_l = quantize_channel<BitDepth::Bit24>(in_l, 0);
        out_r = quantize_channel<BitDepth::Bit24>(in_r, 1);
    }

    // Process a single stereo frame in normalized float space [-1.0, 1.0] with quantized wordlength
    inline void process_sample_float(float in_l, float in_r, float& out_l, float& out_r,
                                     BitDepth depth = BitDepth::Bit16) noexcept {
        if (depth == BitDepth::Bit16) {
            constexpr float kScale16 = 32768.0f;
            out_l = static_cast<float>(quantize_channel<BitDepth::Bit16>(in_l, 0)) / kScale16;
            out_r = static_cast<float>(quantize_channel<BitDepth::Bit16>(in_r, 1)) / kScale16;
        } else {
            constexpr float kScale24 = 8388608.0f;
            out_l = static_cast<float>(quantize_channel<BitDepth::Bit24>(in_l, 0)) / kScale24;
            out_r = static_cast<float>(quantize_channel<BitDepth::Bit24>(in_r, 1)) / kScale24;
        }
    }

    // Block processing to 16-bit PCM
    void process_stereo_16(const float* in_l, const float* in_r,
                           int16_t* out_l, int16_t* out_r, uint32_t frames) noexcept {
        if (!in_l || !in_r || !out_l || !out_r || frames == 0) return;
        for (uint32_t i = 0; i < frames; ++i) {
            process_sample_16(in_l[i], in_r[i], out_l[i], out_r[i]);
        }
    }

    // Block processing to 24-bit PCM
    void process_stereo_24(const float* in_l, const float* in_r,
                           int32_t* out_l, int32_t* out_r, uint32_t frames) noexcept {
        if (!in_l || !in_r || !out_l || !out_r || frames == 0) return;
        for (uint32_t i = 0; i < frames; ++i) {
            process_sample_24(in_l[i], in_r[i], out_l[i], out_r[i]);
        }
    }

    // Block processing in normalized float space
    void process_stereo_float(const float* in_l, const float* in_r,
                              float* out_l, float* out_r, uint32_t frames,
                              BitDepth depth = BitDepth::Bit16) noexcept {
        if (!in_l || !in_r || !out_l || !out_r || frames == 0) return;
        for (uint32_t i = 0; i < frames; ++i) {
            process_sample_float(in_l[i], in_r[i], out_l[i], out_r[i], depth);
        }
    }

private:
    inline float next_rand(int ch) noexcept {
        uint32_t& state = (ch == 0) ? m_rand_l : m_rand_r;
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return static_cast<float>(state) / static_cast<float>(UINT32_MAX);
    }

    template<BitDepth Depth>
    inline int32_t quantize_channel(float in_sample, int ch) noexcept {
        constexpr float kScale = (Depth == BitDepth::Bit16) ? 32768.0f : 8388608.0f;
        constexpr int32_t kMinVal = (Depth == BitDepth::Bit16) ? -32768 : -8388608;
        constexpr int32_t kMaxVal = (Depth == BitDepth::Bit16) ? 32767 : 8388607;

        switch (m_type) {
            case DitherType::None: {
                float scaled = in_sample * kScale;
                int32_t q = static_cast<int32_t>(std::floor(scaled + 0.5f));
                return std::clamp(q, kMinVal, kMaxVal);
            }

            case DitherType::TPDF: {
                // Standard flat TPDF: two independent uniform random numbers in [0, 1)
                float r1 = next_rand(ch);
                float r2 = next_rand(ch);
                float dither = r1 - r2; // Range (-1, 1), triangular PDF
                float scaled = in_sample * kScale + dither;
                int32_t q = static_cast<int32_t>(std::floor(scaled + 0.5f));
                return std::clamp(q, kMinVal, kMaxVal);
            }

            case DitherType::PaulDither: {
                // Airwindows PaulDither: single-pole highpass filtered TPDF (1 - z^-1)
                // r_t - r_{t-1} provides exact triangular PDF with +6dB/octave highpass slope
                float r = next_rand(ch);
                float dither = r - m_prev_paul[ch];
                m_prev_paul[ch] = r;
                float scaled = in_sample * kScale + dither;
                int32_t q = static_cast<int32_t>(std::floor(scaled + 0.5f));
                return std::clamp(q, kMinVal, kMaxVal);
            }

            case DitherType::Dark: {
                // Airwindows Dark: de-emphasized low-passed dither
                // Smooths high-frequency noise using (1 + z^-1) * 0.5 filter
                float r1 = next_rand(ch);
                float r2 = next_rand(ch);
                float raw_dither = r1 - r2;
                float dither = 0.5f * (raw_dither + m_prev_dark[ch]);
                m_prev_dark[ch] = raw_dither;
                float scaled = in_sample * kScale + dither;
                int32_t q = static_cast<int32_t>(std::floor(scaled + 0.5f));
                return std::clamp(q, kMinVal, kMaxVal);
            }

            case DitherType::NJAD: {
                // Airwindows Not Just Another Dither (NJAD):
                // Benford-law threshold noise shaping with dynamic silence clamping
                const double dry = static_cast<double>(in_sample) * static_cast<double>(kScale);

                // Silence threshold: If signal is digital silence or subnormal, collapse to zero
                if (std::abs(in_sample) < 1e-12f) {
                    m_njad_noise[ch] = 0.0;
                    return 0;
                }

                double shaped = dry - m_njad_noise[ch];

                auto get_benford_bin = [](double val) noexcept -> int {
                    double b = std::abs(val);
                    if (b < 1e-9) return 10;
                    while (b >= 10.0) b *= 0.1;
                    while (b < 1.0) b *= 10.0;
                    int digit = static_cast<int>(std::floor(b));
                    return (digit >= 1 && digit <= 9) ? digit : 10;
                };

                int bin_floor = get_benford_bin(std::floor(shaped));
                int bin_ceil = get_benford_bin(std::ceil(shaped));

                static constexpr std::array<double, 10> kBenfordTargets = {
                    0.0, 301.0, 176.0, 125.0, 97.0, 79.0, 67.0, 58.0, 51.0, 46.0
                };

                auto eval_cost = [&](int test_bin) noexcept -> double {
                    if (test_bin < 1 || test_bin > 9) return 1000.0;
                    double cost = 0.0;
                    for (int d = 1; d <= 9; ++d) {
                        double current = m_njad_bins[ch][d] + (d == test_bin ? 1.0 : 0.0);
                        cost += std::abs(kBenfordTargets[d] - current);
                    }
                    return cost;
                };

                double cost_floor = eval_cost(bin_floor);
                double cost_ceil = eval_cost(bin_ceil);

                int32_t chosen_q = 0;
                int chosen_bin = 10;
                if (cost_floor < cost_ceil) {
                    chosen_q = static_cast<int32_t>(std::floor(shaped));
                    chosen_bin = bin_floor;
                } else {
                    chosen_q = static_cast<int32_t>(std::floor(shaped + 1.0));
                    chosen_bin = bin_ceil;
                }

                if (chosen_bin >= 1 && chosen_bin <= 9) {
                    m_njad_bins[ch][chosen_bin] += 1.0;
                    if (m_njad_bins[ch][chosen_bin] > 982.0) {
                        for (int d = 1; d <= 9; ++d) {
                            m_njad_bins[ch][d] *= 0.99;
                        }
                    }
                }

                // Error feedback accumulation
                m_njad_noise[ch] += (static_cast<double>(chosen_q) - dry);

                // Clamping noise shaping magnitude to input signal magnitude:
                // Prevents noise modulation on trailing reverb tails and decays to zero
                const double max_err = std::abs(dry);
                m_njad_noise[ch] = std::clamp(m_njad_noise[ch], -max_err, max_err);

                return std::clamp(chosen_q, kMinVal, kMaxVal);
            }
        }

        return 0;
    }

    DitherType m_type{DitherType::TPDF};

    uint32_t m_rand_l{0x12345678};
    uint32_t m_rand_r{0x87654321};

    std::array<float, 2> m_prev_paul{0.0f, 0.0f};
    std::array<float, 2> m_prev_dark{0.0f, 0.0f};

    std::array<double, 2> m_njad_noise{0.0, 0.0};
    std::array<std::array<double, 11>, 2> m_njad_bins{};
};

} // namespace audio_core::dsp
