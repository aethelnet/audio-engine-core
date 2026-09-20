#include "audio_core/dsp/liquid_ode.hpp"
#include "audio_core/dsp/console_processor.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <numbers>
#include <fstream>
#include <string>
#include <iomanip>
#include <algorithm>

// ============================================================================
// Standard 24-Bit Stereo PCM WAV Writer
// ============================================================================
bool write_wav_24bit(const std::string& filename, const float* left, const float* right, size_t frames, uint32_t sample_rate) {
    std::ofstream out(filename, std::ios::binary);
    if (!out.is_open()) return false;

    const uint16_t num_channels = 2;
    const uint16_t bits_per_sample = 24;
    const uint16_t bytes_per_sample = 3;
    const uint16_t block_align = num_channels * bytes_per_sample;
    const uint32_t byte_rate = sample_rate * block_align;
    const uint32_t data_bytes = static_cast<uint32_t>(frames * block_align);
    const uint32_t file_size = 36 + data_bytes;

    // RIFF header
    out.write("RIFF", 4);
    out.write(reinterpret_cast<const char*>(&file_size), 4);
    out.write("WAVE", 4);

    // fmt subchunk
    out.write("fmt ", 4);
    const uint32_t fmt_size = 16;
    const uint16_t audio_format = 1; // PCM
    out.write(reinterpret_cast<const char*>(&fmt_size), 4);
    out.write(reinterpret_cast<const char*>(&audio_format), 2);
    out.write(reinterpret_cast<const char*>(&num_channels), 2);
    out.write(reinterpret_cast<const char*>(&sample_rate), 4);
    out.write(reinterpret_cast<const char*>(&byte_rate), 4);
    out.write(reinterpret_cast<const char*>(&block_align), 2);
    out.write(reinterpret_cast<const char*>(&bits_per_sample), 2);

    // data subchunk
    out.write("data", 4);
    out.write(reinterpret_cast<const char*>(&data_bytes), 4);

    // Convert float (-1.0 to +1.0) to 24-bit little-endian signed integer
    for (size_t i = 0; i < frames; ++i) {
        for (int ch = 0; ch < 2; ++ch) {
            float s = (ch == 0) ? left[i] : right[i];
            s = std::clamp(s, -1.0f, 1.0f);
            int32_t val = static_cast<int32_t>(s * 8388607.0f);
            if (val > 8388607) val = 8388607;
            if (val < -8388608) val = -8388608;

            uint8_t b0 = static_cast<uint8_t>(val & 0xFF);
            uint8_t b1 = static_cast<uint8_t>((val >> 8) & 0xFF);
            uint8_t b2 = static_cast<uint8_t>((val >> 16) & 0xFF);

            out.put(static_cast<char>(b0));
            out.put(static_cast<char>(b1));
            out.put(static_cast<char>(b2));
        }
    }
    return true;
}

// ============================================================================
// Multi-Track Groove Synthesizer (8 Stems, 120 BPM, 4 Bars = 8.0 Seconds)
// ============================================================================
struct StemTrack {
    std::string name;
    std::vector<float> left;
    std::vector<float> right;
};

std::vector<StemTrack> generate_multitrack_groove(uint32_t sr, size_t total_frames) {
    std::vector<StemTrack> stems(8);
    stems[0].name = "01_Kick";
    stems[1].name = "02_Snare";
    stems[2].name = "03_HiHats";
    stems[3].name = "04_SubBass";
    stems[4].name = "05_Chords";
    stems[5].name = "06_LeadArp";
    stems[6].name = "07_Percussion";
    stems[7].name = "08_ReverbReturn";

    for (auto& s : stems) {
        s.left.assign(total_frames, 0.0f);
        s.right.assign(total_frames, 0.0f);
    }

    const size_t beat_frames = sr / 2; // 120 BPM = 0.5s per beat
    const size_t bar_frames = beat_frames * 4;
    const size_t sixteenth_frames = beat_frames / 4;

    // Simple PRNG for snare and hat synthesis
    uint32_t prng = 0x54321ABC;
    auto white_noise = [&prng]() -> float {
        prng ^= prng << 13; prng ^= prng >> 17; prng ^= prng << 5;
        return (static_cast<float>(prng) * 4.6566129e-10f) - 1.0f;
    };

    // 1. Kick Drum: 4-on-the-floor
    for (size_t beat = 0; beat < 16; ++beat) {
        size_t start = beat * beat_frames;
        for (size_t i = 0; i < beat_frames && (start + i) < total_frames; ++i) {
            float t = static_cast<float>(i) / sr;
            // Exponential pitch drop: 140 Hz -> 48 Hz
            float phase = 2.0f * std::numbers::pi_v<float> * (48.0f * t + (92.0f / 35.0f) * (1.0f - std::exp(-t * 35.0f)));
            float amp = std::exp(-t * 9.0f); // Fast decay
            float click = (i < 48) ? (1.0f - static_cast<float>(i) / 48.0f) * 0.4f : 0.0f;
            float s = (std::sin(phase) * amp * 0.75f) + click;
            stems[0].left[start + i] += s;
            stems[0].right[start + i] += s;
        }
    }

    // 2. Snare Drum: Beats 2 and 4 (beats 1, 3, 5, 7, 9, 11, 13, 15 in 0-index)
    for (size_t bar = 0; bar < 4; ++bar) {
        for (int hit : {1, 3}) {
            size_t start = (bar * 4 + hit) * beat_frames;
            for (size_t i = 0; i < beat_frames && (start + i) < total_frames; ++i) {
                float t = static_cast<float>(i) / sr;
                float body = std::sin(2.0f * std::numbers::pi_v<float> * 185.0f * t) * std::exp(-t * 22.0f) * 0.45f;
                float snap = white_noise() * std::exp(-t * 14.0f) * 0.40f;
                float s = body + snap;
                stems[1].left[start + i] += s;
                stems[1].right[start + i] += s;

                // Send to Reverb Return
                float rev_t = t + 0.03f;
                float rev_decay = std::exp(-rev_t * 1.8f) * 0.15f;
                stems[7].left[start + i] += snap * rev_decay * 0.8f;
                stems[7].right[start + i] += snap * rev_decay * 1.2f;
            }
        }
    }

    // 3. Hi-Hats: 16th notes with velocity dynamics
    for (size_t sixteenth = 0; sixteenth < 64; ++sixteenth) {
        size_t start = sixteenth * sixteenth_frames;
        float vel = (sixteenth % 4 == 0) ? 0.28f : ((sixteenth % 2 == 0) ? 0.20f : 0.14f);
        // Highpass filtered noise
        float hp_state = 0.0f;
        for (size_t i = 0; i < sixteenth_frames && (start + i) < total_frames; ++i) {
            float t = static_cast<float>(i) / sr;
            float n = white_noise();
            hp_state = 0.85f * hp_state + (n - hp_state) * 0.15f; // highpass residue
            float hat = (n - hp_state) * std::exp(-t * 60.0f) * vel;
            stems[2].left[start + i] += hat * 0.8f;  // Panned slightly left
            stems[2].right[start + i] += hat * 1.1f; // Panned slightly right
        }
    }

    // 4. Sub-Bass: 16th-note syncopated groove (C1 -> Ab0 -> F0 -> G0)
    const float root_notes[4] = {32.70f, 25.96f, 21.83f, 24.50f}; // C1, Ab0, F0, G0
    for (size_t bar = 0; bar < 4; ++bar) {
        float base_f = root_notes[bar];
        // 16th-note pattern accents: 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0
        const int pat[16] = {1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0};
        for (int step = 0; step < 16; ++step) {
            if (!pat[step]) continue;
            size_t start = bar * bar_frames + step * sixteenth_frames;
            float note_f = base_f * ((step % 8 == 2) ? 2.0f : 1.0f); // Octave jump on accent
            for (size_t i = 0; i < sixteenth_frames && (start + i) < total_frames; ++i) {
                float t = static_cast<float>(i) / sr;
                float osc1 = std::sin(2.0f * std::numbers::pi_v<float> * note_f * t);
                float osc2 = std::sin(2.0f * std::numbers::pi_v<float> * (note_f * 2.0f) * t) * 0.35f;
                float env = (1.0f - std::exp(-t * 200.0f)) * std::exp(-t * 8.0f);
                float s = (osc1 + osc2) * env * 0.45f;
                stems[3].left[start + i] += s;
                stems[3].right[start + i] += s;
            }
        }
    }

    // 5. Chords: Lush minor 9th pads (Cm9, Abmaj7, Fm9, G7sus4)
    struct ChordFrequencies {
        std::vector<float> freqs;
    };
    std::vector<ChordFrequencies> chords = {
        {{130.81f, 196.00f, 233.08f, 293.66f, 311.13f}}, // Cm9 (C3, G3, Bb3, D4, Eb4)
        {{103.83f, 155.56f, 196.00f, 233.08f, 311.13f}}, // Abmaj7
        {{87.31f, 130.81f, 174.61f, 207.65f, 261.63f}},  // Fm9
        {{98.00f, 146.83f, 174.61f, 233.08f, 293.66f}}   // G7sus4
    };

    for (size_t bar = 0; bar < 4; ++bar) {
        size_t start = bar * bar_frames;
        for (size_t i = 0; i < bar_frames && (start + i) < total_frames; ++i) {
            float t = static_cast<float>(i) / sr;
            float pad_l = 0.0f, pad_r = 0.0f;
            float env = std::sin(std::numbers::pi_v<float> * i / bar_frames); // Smooth swelling arc
            for (float f : chords[bar].freqs) {
                // Stereo detuning (+- 0.8 Hz)
                pad_l += std::sin(2.0f * std::numbers::pi_v<float> * (f - 0.6f) * t);
                pad_r += std::sin(2.0f * std::numbers::pi_v<float> * (f + 0.6f) * t);
            }
            stems[4].left[start + i] += (pad_l / 5.0f) * env * 0.28f;
            stems[4].right[start + i] += (pad_r / 5.0f) * env * 0.28f;
        }
    }

    // 6. Lead Arpeggio (1.5 - 2.5 kHz high-mid melody)
    const float arp_notes[8] = {523.25f, 659.25f, 783.99f, 987.77f, 1046.50f, 783.99f, 659.25f, 587.33f};
    for (size_t step = 0; step < 64; ++step) {
        size_t start = step * sixteenth_frames;
        float f = arp_notes[step % 8];
        for (size_t i = 0; i < sixteenth_frames && (start + i) < total_frames; ++i) {
            float t = static_cast<float>(i) / sr;
            float osc = std::sin(2.0f * std::numbers::pi_v<float> * f * t);
            float env = std::exp(-t * 28.0f);
            float s = osc * env * 0.22f;
            stems[5].left[start + i] += s * 1.1f;
            stems[5].right[start + i] += s * 0.9f;

            // Reverb bleed
            float rev = s * std::exp(-t * 2.5f) * 0.12f;
            stems[7].left[start + i] += rev * 0.7f;
            stems[7].right[start + i] += rev * 1.3f;
        }
    }

    // 7. Percussion: Offbeat syncopated rimshots
    for (size_t beat = 0; beat < 16; ++beat) {
        size_t start = beat * beat_frames + sixteenth_frames * 3; // On the "and-a"
        if (beat % 2 == 0) continue;
        for (size_t i = 0; i < sixteenth_frames && (start + i) < total_frames; ++i) {
            float t = static_cast<float>(i) / sr;
            float rim = std::sin(2.0f * std::numbers::pi_v<float> * 1450.0f * t) * std::exp(-t * 80.0f) * 0.25f;
            stems[6].left[start + i] += rim * 0.6f;
            stems[6].right[start + i] += rim * 1.2f;
        }
    }

    return stems;
}

int main() {
    std::cout << "=========================================================" << std::endl;
    std::cout << "   SOVEREIGN AUDIO CORE: MULTI-TRACK SUMMING SHOOTOUT   " << std::endl;
    std::cout << "=========================================================" << std::endl;

    const uint32_t sample_rate = 48000;
    const size_t total_frames = sample_rate * 8; // 8.0 seconds = 4 bars at 120 BPM

    std::cout << "[1/4] Synthesizing 8-track arrangement (Kick, Snare, Hats, Bass, Chords, Lead, Perc, Reverb)..." << std::endl;
    auto stems = generate_multitrack_groove(sample_rate, total_frames);
    std::cout << "  -> 8 stems generated (" << total_frames << " frames / 8.0 seconds at 48 kHz)." << std::endl;

    // Mix fader level to simulate realistic console headroom (preventing master 24-bit clipping)
    constexpr float kMixFader = 0.45f;
    for (auto& stem : stems) {
        for (size_t i = 0; i < total_frames; ++i) {
            stem.left[i] *= kMixFader;
            stem.right[i] *= kMixFader;
        }
    }

    // --- Version A: Linear Digital Summing ---
    std::cout << "[2/4] Rendering Version 1: Pure Linear Digital Summing (DAW Standard)..." << std::endl;
    std::vector<float> linear_l(total_frames, 0.0f);
    std::vector<float> linear_r(total_frames, 0.0f);

    for (const auto& stem : stems) {
        for (size_t i = 0; i < total_frames; ++i) {
            linear_l[i] += stem.left[i];
            linear_r[i] += stem.right[i];
        }
    }

    // --- Version B: Airwindows EveryConsole Summing ---
    std::cout << "[3/4] Rendering Version 2: Airwindows EveryConsole (Console5/8 Purest)..." << std::endl;
    std::vector<float> airwin_l(total_frames, 0.0f);
    std::vector<float> airwin_r(total_frames, 0.0f);

    // Track encode
    std::vector<audio_core::dsp::ConsoleProcessor> track_consoles(stems.size());
    for (size_t s = 0; s < stems.size(); ++s) {
        track_consoles[s].set_type(audio_core::dsp::ConsoleType::Purest);
        track_consoles[s].set_mode(audio_core::dsp::ConsoleMode::Channel);

        std::vector<float> enc_l = stems[s].left;
        std::vector<float> enc_r = stems[s].right;
        track_consoles[s].process_stereo(enc_l.data(), enc_r.data(), total_frames);

        for (size_t i = 0; i < total_frames; ++i) {
            airwin_l[i] += enc_l[i];
            airwin_r[i] += enc_r[i];
        }
    }
    // Master bus decode
    audio_core::dsp::ConsoleProcessor master_console;
    master_console.set_type(audio_core::dsp::ConsoleType::Purest);
    master_console.set_mode(audio_core::dsp::ConsoleMode::Buss);
    master_console.process_stereo(airwin_l.data(), airwin_r.data(), total_frames);

    // --- Version C: Sovereign LiquidBusProcessor (Dual-Layer Core) ---
    std::cout << "[4/4] Rendering Version 3: Sovereign LiquidBusProcessor (Dual-Layer Core Glue)..." << std::endl;
    std::vector<float> liquid_l = linear_l;
    std::vector<float> liquid_r = linear_r;

    audio_core::dsp::LiquidBusProcessor liquid_bus(sample_rate, audio_core::dsp::LiquidBusProcessor::Mode::DifferentialMagneticGlue);
    liquid_bus.set_ballistics(12.0f, 180.0f, 0.70f); // 12ms attack, 180ms release, 70% glue depth
    liquid_bus.set_headroom(0.38f); // 0.38 linear threshold (~ -8.4 dBFS)
    liquid_bus.process_bus_sum(liquid_l.data(), liquid_r.data(), total_frames);

    // --- Version D: Isolated Difference / Delta Signal ---
    std::vector<float> diff_l(total_frames, 0.0f);
    std::vector<float> diff_r(total_frames, 0.0f);
    for (size_t i = 0; i < total_frames; ++i) {
        // Boosted 4x (+12 dB) so the subtle analog glue action can be easily heard in isolation
        diff_l[i] = (liquid_l[i] - linear_l[i]) * 4.0f;
        diff_r[i] = (liquid_r[i] - linear_r[i]) * 4.0f;
    }

    // Save all 4 files
    const std::string out_dir = "renders/";
    write_wav_24bit(out_dir + "sum_01_linear_digital.wav", linear_l.data(), linear_r.data(), total_frames, sample_rate);
    write_wav_24bit(out_dir + "sum_02_airwindows_console.wav", airwin_l.data(), airwin_r.data(), total_frames, sample_rate);
    write_wav_24bit(out_dir + "sum_03_liquid_bus_glue.wav", liquid_l.data(), liquid_r.data(), total_frames, sample_rate);
    write_wav_24bit(out_dir + "sum_04_difference_delta_glue.wav", diff_l.data(), diff_r.data(), total_frames, sample_rate);

    // Measure statistics
    auto get_stats = [](const std::vector<float>& l, const std::vector<float>& r) {
        float peak = 0.0f;
        double sum_sq = 0.0;
        size_t N = l.size();
        for (size_t i = 0; i < N; ++i) {
            float al = std::abs(l[i]);
            float ar = std::abs(r[i]);
            if (al > peak) peak = al;
            if (ar > peak) peak = ar;
            sum_sq += l[i] * l[i] + r[i] * r[i];
        }
        float rms = static_cast<float>(std::sqrt(sum_sq / (2.0 * N)));
        float peak_db = 20.0f * std::log10(std::max(1e-6f, peak));
        float rms_db = 20.0f * std::log10(std::max(1e-6f, rms));
        float crest_db = peak_db - rms_db;
        return std::make_tuple(peak_db, rms_db, crest_db);
    };

    auto [lin_peak, lin_rms, lin_crest] = get_stats(linear_l, linear_r);
    auto [aw_peak, aw_rms, aw_crest] = get_stats(airwin_l, airwin_r);
    auto [liq_peak, liq_rms, liq_crest] = get_stats(liquid_l, liquid_r);

    std::cout << "\n=========================================================" << std::endl;
    std::cout << "                  SHOOTOUT TELEMETRY                     " << std::endl;
    std::cout << "=========================================================" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  1. Linear Digital:     Peak = " << lin_peak << " dBFS | RMS = " << lin_rms << " dBFS | Crest = " << lin_crest << " dB" << std::endl;
    std::cout << "  2. Airwindows Console: Peak = " << aw_peak << " dBFS | RMS = " << aw_rms << " dBFS | Crest = " << aw_crest << " dB" << std::endl;
    std::cout << "  3. Liquid Bus Glue:    Peak = " << liq_peak << " dBFS | RMS = " << liq_rms << " dBFS | Crest = " << liq_crest << " dB" << std::endl;
    std::cout << "---------------------------------------------------------" << std::endl;
    std::cout << "  -> Peak Taming (Liquid vs Linear): " << (lin_peak - liq_peak) << " dB of natural transient rounding" << std::endl;
    std::cout << "  -> Density/RMS Preservation:       " << (liq_rms - lin_rms) << " dB change in overall loudness" << std::endl;
    std::cout << "=========================================================" << std::endl;
    std::cout << "\n[Audio Files Successfully Written to renders/]:" << std::endl;
    std::cout << "  -> renders/sum_01_linear_digital.wav" << std::endl;
    std::cout << "  -> renders/sum_02_airwindows_console.wav" << std::endl;
    std::cout << "  -> renders/sum_03_liquid_bus_glue.wav" << std::endl;
    std::cout << "  -> renders/sum_04_difference_delta_glue.wav" << std::endl;
    std::cout << "=========================================================" << std::endl;

    return 0;
}
