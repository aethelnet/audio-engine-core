#pragma once

#include "audio_core/types.hpp"
#include "audio_core/dsp/crossover.hpp"
#include <array>
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace audio_core::dsp {

// ============================================================================
// LinkwitzRiley4Way: 4th-Order 4-Band Phase-Compensated Crossover (LR4, 24 dB/oct)
// ============================================================================
// Splits incoming stereo audio into 4 distinct frequency bands:
//   Band 0: Sub / Low       (0 .. f1)
//   Band 1: Low-Mid         (f1 .. f2)
//   Band 2: High-Mid        (f2 .. f3)
//   Band 3: Air / Treble    (f3 .. Nyquist)
//
// Phase-Compensated Tree Topology:
//   Band 0 = LP1 * AP2 * AP3
//   Band 1 = HP1 * LP2 * AP3
//   Band 2 = HP1 * HP2 * LP3
//   Band 3 = HP1 * HP2 * HP3
//
// Mathematical Invariants:
//   1. Flat Magnitude Sum: |Band0 + Band1 + Band2 + Band3| == 1.0000 (0.00 dB ripple).
//   2. 0° relative phase between adjacent bands at each crossover frequency.
//   3. Zero dynamic memory allocations during runtime audio processing.
//   4. Per-band Gain trim, Mute, Solo, and Bypass state controls.
// ============================================================================
class LinkwitzRiley4Way {
public:
    static constexpr size_t kNumBands = 4;
    static constexpr size_t kNumSplits = 3;

    LinkwitzRiley4Way(float f_sub_lowmid = 120.0f,
                      float f_lowmid_highmid = 1200.0f,
                      float f_highmid_air = 6000.0f,
                      uint32_t sample_rate = 48000) noexcept {
        set_crossovers(f_sub_lowmid, f_lowmid_highmid, f_highmid_air, sample_rate);
    }

    void set_crossovers(float f_split1, float f_split2, float f_split3, uint32_t sample_rate) noexcept {
        if (sample_rate == 0) return;
        m_sample_rate = sample_rate;
        const float nyquist = static_cast<float>(sample_rate) * 0.49f;

        // Ensure strictly ascending order with minimum 20 Hz separation
        m_f1 = std::clamp(f_split1, 20.0f, nyquist - 60.0f);
        m_f2 = std::clamp(std::max(f_split2, m_f1 + 20.0f), 40.0f, nyquist - 40.0f);
        m_f3 = std::clamp(std::max(f_split3, m_f2 + 20.0f), 60.0f, nyquist - 20.0f);

        // Configure split filters
        m_split1.set_crossover(m_f1, m_sample_rate);
        m_split2.set_crossover(m_f2, m_sample_rate);
        m_split3.set_crossover(m_f3, m_sample_rate);

        // Configure phase compensators (AP2 and AP3)
        // Band 0 passes through AP2 and AP3
        m_ap2_for_b0.set_crossover(m_f2, m_sample_rate);
        m_ap3_for_b0.set_crossover(m_f3, m_sample_rate);

        // Band 1 passes through AP3
        m_ap3_for_b1.set_crossover(m_f3, m_sample_rate);
    }

    void reset() noexcept {
        m_split1.reset();
        m_split2.reset();
        m_split3.reset();
        m_ap2_for_b0.reset();
        m_ap3_for_b0.reset();
        m_ap3_for_b1.reset();
    }

    [[nodiscard]] uint32_t sample_rate() const noexcept { return m_sample_rate; }
    [[nodiscard]] float f1() const noexcept { return m_f1; }
    [[nodiscard]] float f2() const noexcept { return m_f2; }
    [[nodiscard]] float f3() const noexcept { return m_f3; }

    void set_band_gain(size_t band, float gain) noexcept {
        if (band < kNumBands) m_bands[band].gain = gain;
    }
    void set_band_mute(size_t band, bool mute) noexcept {
        if (band < kNumBands) m_bands[band].mute = mute;
    }
    void set_band_solo(size_t band, bool solo) noexcept {
        if (band < kNumBands) m_bands[band].solo = solo;
    }
    void set_band_bypass(size_t band, bool bypass) noexcept {
        if (band < kNumBands) m_bands[band].bypass = bypass;
    }

    [[nodiscard]] const BandState& band_state(size_t band) const noexcept {
        return m_bands[std::min(band, kNumBands - 1)];
    }

    // Process single stereo sample into 4 discrete output band pairs
    inline void process_sample(float in_l, float in_r,
                               float out_bands_l[kNumBands],
                               float out_bands_r[kNumBands]) noexcept {
        // --- 1. Split 1 at f1 (Sub vs Above) ---
        float lp1_l = 0.0f, lp1_r = 0.0f;
        float hp1_l = 0.0f, hp1_r = 0.0f;
        m_split1.process_sample(in_l, in_r, lp1_l, lp1_r, hp1_l, hp1_r);

        // --- 2. Split 2 at f2 (Low-Mid vs Highs) ---
        float lp2_l = 0.0f, lp2_r = 0.0f;
        float hp2_l = 0.0f, hp2_r = 0.0f;
        m_split2.process_sample(hp1_l, hp1_r, lp2_l, lp2_r, hp2_l, hp2_r);

        // --- 3. Split 3 at f3 (High-Mid vs Air) ---
        float lp3_l = 0.0f, lp3_r = 0.0f;
        float hp3_l = 0.0f, hp3_r = 0.0f;
        m_split3.process_sample(hp2_l, hp2_r, lp3_l, lp3_r, hp3_l, hp3_r);

        // --- 4. Phase Compensation ---
        // In LR4, LP + HP = AP (Allpass).
        // Band 0 (Sub): lp1 -> AP2 -> AP3
        float b0_ap2_lp_l = 0.0f, b0_ap2_lp_r = 0.0f;
        float b0_ap2_hp_l = 0.0f, b0_ap2_hp_r = 0.0f;
        m_ap2_for_b0.process_sample(lp1_l, lp1_r, b0_ap2_lp_l, b0_ap2_lp_r, b0_ap2_hp_l, b0_ap2_hp_r);
        const float b0_mid_l = b0_ap2_lp_l + b0_ap2_hp_l;
        const float b0_mid_r = b0_ap2_lp_r + b0_ap2_hp_r;

        float b0_ap3_lp_l = 0.0f, b0_ap3_lp_r = 0.0f;
        float b0_ap3_hp_l = 0.0f, b0_ap3_hp_r = 0.0f;
        m_ap3_for_b0.process_sample(b0_mid_l, b0_mid_r, b0_ap3_lp_l, b0_ap3_lp_r, b0_ap3_hp_l, b0_ap3_hp_r);
        out_bands_l[0] = b0_ap3_lp_l + b0_ap3_hp_l;
        out_bands_r[0] = b0_ap3_lp_r + b0_ap3_hp_r;

        // Band 1 (Low-Mid): lp2 -> AP3
        float b1_ap3_lp_l = 0.0f, b1_ap3_lp_r = 0.0f;
        float b1_ap3_hp_l = 0.0f, b1_ap3_hp_r = 0.0f;
        m_ap3_for_b1.process_sample(lp2_l, lp2_r, b1_ap3_lp_l, b1_ap3_lp_r, b1_ap3_hp_l, b1_ap3_hp_r);
        out_bands_l[1] = b1_ap3_lp_l + b1_ap3_hp_l;
        out_bands_r[1] = b1_ap3_lp_r + b1_ap3_hp_r;

        // Band 2 (High-Mid): lp3 (already in phase with AP3)
        out_bands_l[2] = lp3_l;
        out_bands_r[2] = lp3_r;

        // Band 3 (Air): hp3 (already in phase with AP3)
        out_bands_l[3] = hp3_l;
        out_bands_r[3] = hp3_r;

        // --- 5. Solo / Mute / Gain Evaluation ---
        bool has_solo = false;
        for (size_t b = 0; b < kNumBands; ++b) {
            if (m_bands[b].solo) {
                has_solo = true;
                break;
            }
        }

        for (size_t b = 0; b < kNumBands; ++b) {
            if (has_solo) {
                if (!m_bands[b].solo) {
                    out_bands_l[b] = 0.0f;
                    out_bands_r[b] = 0.0f;
                    continue;
                }
            } else if (m_bands[b].mute) {
                out_bands_l[b] = 0.0f;
                out_bands_r[b] = 0.0f;
                continue;
            }
            out_bands_l[b] *= m_bands[b].gain;
            out_bands_r[b] *= m_bands[b].gain;
        }
    }

    // Recombine all 4 bands into a stereo master sum
    inline void sum_bands(const float out_bands_l[kNumBands],
                          const float out_bands_r[kNumBands],
                          float& sum_l, float& sum_r) const noexcept {
        sum_l = out_bands_l[0] + out_bands_l[1] + out_bands_l[2] + out_bands_l[3];
        sum_r = out_bands_r[0] + out_bands_r[1] + out_bands_r[2] + out_bands_r[3];
    }

    // Process full stereo frame buffer
    void process_block(const float* in_l, const float* in_r,
                       float* const out_bands_l[kNumBands],
                       float* const out_bands_r[kNumBands],
                       uint32_t frames) noexcept {
        if (!in_l || !in_r || frames == 0) return;
        float sample_bands_l[kNumBands];
        float sample_bands_r[kNumBands];

        for (uint32_t i = 0; i < frames; ++i) {
            process_sample(in_l[i], in_r[i], sample_bands_l, sample_bands_r);
            for (size_t b = 0; b < kNumBands; ++b) {
                if (out_bands_l[b]) out_bands_l[b][i] = sample_bands_l[b];
                if (out_bands_r[b]) out_bands_r[b][i] = sample_bands_r[b];
            }
        }
    }

private:
    uint32_t m_sample_rate{48000};
    float m_f1{120.0f};
    float m_f2{1200.0f};
    float m_f3{6000.0f};

    std::array<BandState, kNumBands> m_bands{};

    LinkwitzRiley2Way m_split1;
    LinkwitzRiley2Way m_split2;
    LinkwitzRiley2Way m_split3;

    LinkwitzRiley2Way m_ap2_for_b0;
    LinkwitzRiley2Way m_ap3_for_b0;
    LinkwitzRiley2Way m_ap3_for_b1;
};

} // namespace audio_core::dsp
