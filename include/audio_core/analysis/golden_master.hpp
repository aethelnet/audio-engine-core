#pragma once

#include "audio_core/types.hpp"
#include "audio_core/dsp/baxandall.hpp"
#include "audio_core/dsp/buttercomp2.hpp"
#include "audio_core/dsp/purest_drive.hpp"
#include "audio_core/dsp/derez.hpp"
#include "audio_core/dsp/liquid_vactrol.hpp"
#include "audio_core/dsp/multihead_ode_compressor.hpp"
#include "audio_core/dsp/clip_only2.hpp"
#include "audio_core/sampling/vari_speed_streamer.hpp"
#include "audio_core/mixer_graph.hpp"
#include "audio_core/sampling/wav_reader.hpp"

#include <vector>
#include <string>
#include <cstdint>
#include <cmath>
#include <numbers>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <functional>
#include <iostream>
#include <array>

namespace audio_core::analysis {

// ============================================================================
// 1. Bit-Exact Hash Functions: 64-Bit FNV-1a & Standard IEEE 802.3 CRC-32
// ============================================================================
class ChecksumEngine {
public:
    static constexpr uint64_t FNV1A_64_OFFSET = 14695981039346656037ULL;
    static constexpr uint64_t FNV1A_64_PRIME  = 1099511628211ULL;

    [[nodiscard]] static constexpr uint64_t fnv1a_64_update(uint64_t hash, uint8_t byte) noexcept {
        return (hash ^ byte) * FNV1A_64_PRIME;
    }

    [[nodiscard]] static constexpr uint32_t crc32_byte(uint32_t crc, uint8_t byte) noexcept {
        crc ^= byte;
        for (int k = 0; k < 8; ++k) {
            crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1u));
        }
        return crc;
    }
};

// ============================================================================
// 2. Audio Metrics & Golden Master Checksum Snapshot
// ============================================================================
struct GoldenMasterMetrics {
    std::string pipeline_name;
    uint32_t sample_rate{48000};
    size_t frames{0};
    uint32_t channels{2};

    // Bit-exact hashes
    uint64_t pcm24_fnv1a64{ChecksumEngine::FNV1A_64_OFFSET};
    uint32_t pcm24_crc32{0xFFFFFFFFu};
    uint64_t float32_fnv1a64{ChecksumEngine::FNV1A_64_OFFSET};

    // Analytical Audio Metrics
    float peak_linear{0.0f};
    float peak_dbfs{-180.0f};
    float rms_linear{0.0f};
    float rms_dbfs{-180.0f};
    float crest_factor_db{0.0f};
    float dc_offset{0.0f};
    uint32_t zero_crossings{0};
    double energy_sum{0.0};

    // Format metrics into a clean single-line string
    [[nodiscard]] std::string to_string() const {
        std::ostringstream ss;
        ss << std::left << std::setw(36) << pipeline_name
           << " | Frames: " << std::setw(6) << frames
           << " | Peak: " << std::fixed << std::setprecision(2) << std::setw(6) << peak_dbfs << " dBFS"
           << " | RMS: " << std::fixed << std::setprecision(2) << std::setw(6) << rms_dbfs << " dBFS"
           << " | FNV64: 0x" << std::hex << std::uppercase << std::setw(16) << std::setfill('0') << pcm24_fnv1a64
           << " | CRC32: 0x" << std::setw(8) << std::setfill('0') << pcm24_crc32;
        return ss.str();
    }
};

// ============================================================================
// 3. Audio Buffer Checksum & Analysis Calculator
// ============================================================================
class GoldenMasterAnalyzer {
public:
    static GoldenMasterMetrics analyze(const std::string& name,
                                       const float* left,
                                       const float* right,
                                       size_t frames,
                                       uint32_t sample_rate = 48000) {
        GoldenMasterMetrics m;
        m.pipeline_name = name;
        m.sample_rate = sample_rate;
        m.frames = frames;
        m.channels = 2;

        if (!left || frames == 0) return m;
        const float* r_ptr = right ? right : left;

        uint64_t pcm_hash = ChecksumEngine::FNV1A_64_OFFSET;
        uint32_t pcm_crc = 0xFFFFFFFFu;
        uint64_t flt_hash = ChecksumEngine::FNV1A_64_OFFSET;

        double sum_sq = 0.0;
        double sum_val = 0.0;
        float max_abs = 0.0f;
        uint32_t zc = 0;
        float prev_l = left[0];

        for (size_t i = 0; i < frames; ++i) {
            float sl = left[i];
            float sr = r_ptr[i];

            // Sanitize
            if (!std::isfinite(sl)) sl = 0.0f;
            if (!std::isfinite(sr)) sr = 0.0f;

            // Zero crossings on Left
            if (i > 0) {
                if ((sl >= 0.0f && prev_l < 0.0f) || (sl < 0.0f && prev_l >= 0.0f)) {
                    zc++;
                }
            }
            prev_l = sl;

            // Energy & Peak
            float abs_l = std::abs(sl);
            float abs_r = std::abs(sr);
            if (abs_l > max_abs) max_abs = abs_l;
            if (abs_r > max_abs) max_abs = abs_r;

            sum_sq += static_cast<double>(sl) * static_cast<double>(sl);
            sum_sq += static_cast<double>(sr) * static_cast<double>(sr);
            sum_val += static_cast<double>(sl) + static_cast<double>(sr);

            // Raw Float32 Bit Hashing
            const uint8_t* raw_l = reinterpret_cast<const uint8_t*>(&sl);
            const uint8_t* raw_r = reinterpret_cast<const uint8_t*>(&sr);
            for (int b = 0; b < 4; ++b) {
                flt_hash = ChecksumEngine::fnv1a_64_update(flt_hash, raw_l[b]);
            }
            for (int b = 0; b < 4; ++b) {
                flt_hash = ChecksumEngine::fnv1a_64_update(flt_hash, raw_r[b]);
            }

            // Standard 24-Bit PCM Quantization (144 dB Dynamic Range)
            for (int ch = 0; ch < 2; ++ch) {
                float s = (ch == 0) ? sl : sr;
                float clamped = std::clamp(s, -1.0f, 1.0f);
                int32_t val = static_cast<int32_t>(std::round(clamped * 8388607.0f));
                if (val > 8388607) val = 8388607;
                if (val < -8388608) val = -8388608;

                uint8_t b0 = static_cast<uint8_t>(val & 0xFF);
                uint8_t b1 = static_cast<uint8_t>((val >> 8) & 0xFF);
                uint8_t b2 = static_cast<uint8_t>((val >> 16) & 0xFF);

                pcm_hash = ChecksumEngine::fnv1a_64_update(pcm_hash, b0);
                pcm_hash = ChecksumEngine::fnv1a_64_update(pcm_hash, b1);
                pcm_hash = ChecksumEngine::fnv1a_64_update(pcm_hash, b2);

                pcm_crc = ChecksumEngine::crc32_byte(pcm_crc, b0);
                pcm_crc = ChecksumEngine::crc32_byte(pcm_crc, b1);
                pcm_crc = ChecksumEngine::crc32_byte(pcm_crc, b2);
            }
        }

        m.pcm24_fnv1a64 = pcm_hash;
        m.pcm24_crc32 = ~pcm_crc; // Final inversion
        m.float32_fnv1a64 = flt_hash;

        m.peak_linear = max_abs;
        m.peak_dbfs = (max_abs > 1e-9f) ? 20.0f * std::log10(max_abs) : -180.0f;

        double mean_sq = sum_sq / static_cast<double>(frames * 2);
        m.rms_linear = static_cast<float>(std::sqrt(mean_sq));
        m.rms_dbfs = (m.rms_linear > 1e-9f) ? 20.0f * std::log10(m.rms_linear) : -180.0f;

        m.crest_factor_db = m.peak_dbfs - m.rms_dbfs;
        m.dc_offset = static_cast<float>(sum_val / static_cast<double>(frames * 2));
        m.zero_crossings = zc;
        m.energy_sum = sum_sq;

        return m;
    }
};

// ============================================================================
// 4. Deterministic Reference Test Signal Generators
// ============================================================================
class ReferenceSignalGenerator {
public:
    // Dirac Impulse: Clean single sample spike
    static void generate_dirac_impulse(std::vector<float>& left,
                                       std::vector<float>& right,
                                       size_t frames,
                                       size_t spike_frame = 4,
                                       float amplitude = 1.0f) {
        left.assign(frames, 0.0f);
        right.assign(frames, 0.0f);
        if (spike_frame < frames) {
            left[spike_frame] = amplitude;
            right[spike_frame] = amplitude;
        }
    }

    // Pure Sine Tone
    static void generate_sine(std::vector<float>& left,
                              std::vector<float>& right,
                              size_t frames,
                              uint32_t sample_rate,
                              float freq_hz,
                              float amplitude = 0.8f) {
        left.resize(frames);
        right.resize(frames);
        const double phase_inc = 2.0 * std::numbers::pi * static_cast<double>(freq_hz) / static_cast<double>(sample_rate);
        for (size_t i = 0; i < frames; ++i) {
            float val = static_cast<float>(std::sin(phase_inc * static_cast<double>(i))) * amplitude;
            left[i] = val;
            right[i] = val;
        }
    }

    // Transient Burst: Fast attack (1ms), resonant decay (50ms), dual frequency (120Hz + 2400Hz)
    static void generate_transient_burst(std::vector<float>& left,
                                         std::vector<float>& right,
                                         size_t frames,
                                         uint32_t sample_rate,
                                         float peak_amp = 1.0f) {
        left.assign(frames, 0.0f);
        right.assign(frames, 0.0f);

        const size_t attack_frames = sample_rate / 1000;      // 1 ms attack
        const size_t decay_frames = sample_rate * 50 / 1000;  // 50 ms decay
        const double w_low = 2.0 * std::numbers::pi * 120.0 / sample_rate;
        const double w_high = 2.0 * std::numbers::pi * 2400.0 / sample_rate;

        for (size_t i = 0; i < frames; ++i) {
            float env = 0.0f;
            if (i < attack_frames) {
                env = static_cast<float>(i) / static_cast<float>(attack_frames);
            } else if (i < attack_frames + decay_frames) {
                float t = static_cast<float>(i - attack_frames) / static_cast<float>(decay_frames);
                env = std::exp(-3.5f * t);
            } else {
                env = 0.0f;
            }

            double s_low = std::sin(w_low * static_cast<double>(i));
            double s_high = std::sin(w_high * static_cast<double>(i));
            float s = static_cast<float>(s_low * 0.7 + s_high * 0.3) * env * peak_amp;
            left[i] = s;
            right[i] = s;
        }
    }

    // Complex Chord: Harmonic blend (C# minor: C#3 138.59Hz, E3 164.81Hz, G#3 207.65Hz, B3 246.94Hz)
    static void generate_complex_chord(std::vector<float>& left,
                                       std::vector<float>& right,
                                       size_t frames,
                                       uint32_t sample_rate,
                                       float peak_amp = 0.8f) {
        left.assign(frames, 0.0f);
        right.assign(frames, 0.0f);

        const std::array<double, 4> freqs = {138.59, 164.81, 207.65, 246.94};
        for (size_t i = 0; i < frames; ++i) {
            double sum_l = 0.0;
            double sum_r = 0.0;
            for (size_t f = 0; f < freqs.size(); ++f) {
                double phase = 2.0 * std::numbers::pi * freqs[f] * static_cast<double>(i) / static_cast<double>(sample_rate);
                double s = std::sin(phase);
                // Slight stereo pan
                sum_l += s * (0.8 - 0.1 * static_cast<double>(f));
                sum_r += s * (0.7 + 0.1 * static_cast<double>(f));
            }
            // Gentle decay
            float env = std::exp(-1.5f * static_cast<float>(i) / static_cast<float>(frames));
            left[i] = static_cast<float>(sum_l * 0.25) * env * peak_amp;
            right[i] = static_cast<float>(sum_r * 0.25) * env * peak_amp;
        }
    }

    // Overdriven Sine for Clipper Testing (Peak = 2.50 = +7.95 dBFS)
    static void generate_overdriven_sine(std::vector<float>& left,
                                        std::vector<float>& right,
                                        size_t frames,
                                        uint32_t sample_rate,
                                        float peak_amp = 2.5f) {
        left.resize(frames);
        right.resize(frames);
        const double phase_inc = 2.0 * std::numbers::pi * 1000.0 / static_cast<double>(sample_rate);
        for (size_t i = 0; i < frames; ++i) {
            float val = static_cast<float>(std::sin(phase_inc * static_cast<double>(i))) * peak_amp;
            left[i] = val;
            right[i] = val;
        }
    }
};

// ============================================================================
// 5. Verification Result & Forensic Diff Report
// ============================================================================
struct GoldenMasterDiff {
    bool passed{false};
    std::string pipeline_name;
    uint64_t expected_fnv64{0};
    uint64_t actual_fnv64{0};
    uint32_t expected_crc32{0};
    uint32_t actual_crc32{0};

    float expected_peak_db{0.0f};
    float actual_peak_db{0.0f};
    float expected_rms_db{0.0f};
    float actual_rms_db{0.0f};

    float max_abs_diff{0.0f};
    size_t first_mismatch_frame{0};
    int first_mismatch_channel{0};
    float expected_sample{0.0f};
    float actual_sample{0.0f};
    std::string error_message;

    [[nodiscard]] std::string format_report() const {
        std::ostringstream ss;
        if (passed) {
            ss << "[PASS] " << pipeline_name << " (Bit-exact checksums & audio metrics verified)";
        } else {
            ss << "[FAIL] REGRESSION DETECTED IN PIPELINE: " << pipeline_name << "\n"
               << "  -> Checksum Expected: 0x" << std::hex << std::uppercase << expected_fnv64
               << " | Actual: 0x" << actual_fnv64 << " (CRC: 0x" << expected_crc32 << " vs 0x" << actual_crc32 << ")\n"
               << "  -> Peak dBFS: Exp " << std::fixed << std::setprecision(3) << expected_peak_db
               << " | Act " << actual_peak_db
               << " (Delta: " << (actual_peak_db - expected_peak_db) << " dB)\n"
               << "  -> RMS dBFS: Exp " << expected_rms_db << " | Act " << actual_rms_db
               << " (Delta: " << (actual_rms_db - expected_rms_db) << " dB)\n"
               << "  -> Max Sample Delta: " << max_abs_diff << "\n"
               << "  -> First Mismatch: Frame " << std::dec << first_mismatch_frame
               << " (Ch " << first_mismatch_channel << ") Exp " << expected_sample << " vs Act " << actual_sample;
        }
        return ss.str();
    }
};

// ============================================================================
// 6. Pipeline Definition & Execution Engine
// ============================================================================
struct PipelineDefinition {
    std::string name;
    uint32_t sample_rate{48000};
    size_t frames{4096};
    std::function<void(std::vector<float>& out_l, std::vector<float>& out_r)> render_func;
};

// ============================================================================
// 7. Canonical Golden Master Reference Table
// ============================================================================
struct CanonicalReference {
    const char* name;
    size_t frames;
    uint64_t fnv1a64;
    uint32_t crc32;
    float peak_dbfs;
    float rms_dbfs;
    float crest_factor_db;
    uint32_t zero_crossings;
};

inline constexpr CanonicalReference kCanonicalGoldenMasters[10] = {
    {
        "01_Airwindows_Baxandall_Impulse",
        4096ULL,
        0x118E622168FE7729ULL,
        0xC7456474U,
        -2.6315f,
        -38.5813f,
        35.9498f,
        18U
    },
    {
        "02_Airwindows_ButterComp2_Pumping",
        8192ULL,
        0x52F17762AFEB9C53ULL,
        0x1DBC5536U,
        7.5739f,
        -10.9434f,
        18.5173f,
        74U
    },
    {
        "03_Airwindows_PurestDrive_Sat",
        4096ULL,
        0xD0F54E582D407BB9ULL,
        0xEC5B498DU,
        -1.1301f,
        -3.9239f,
        2.7937f,
        170U
    },
    {
        "04_Airwindows_DeRez2_MuLaw",
        4096ULL,
        0x396F666F0A9AE8C0ULL,
        0x0B3A42ACU,
        -3.4933f,
        -18.0208f,
        14.5275f,
        29U
    },
    {
        "05_LiquidVactrol_LA2A_Ballistics",
        16384ULL,
        0x0EA05F3638C79DA3ULL,
        0x843F7564U,
        -4.9680f,
        -27.4223f,
        22.4543f,
        277U
    },
    {
        "06_LiquidVactrol_BuchlaLPG_Ringing",
        8192ULL,
        0xB1367E5524618573ULL,
        0x2560E7EBU,
        -57.9305f,
        -81.6415f,
        23.7110f,
        44U
    },
    {
        "07_MultiHeadODE_Compressor_Glue",
        8192ULL,
        0x552A90E081CA1BFDULL,
        0x86052D96U,
        -0.3189f,
        -13.1100f,
        12.7911f,
        930U
    },
    {
        "08_Airwindows_ClipOnly2_Ceiling",
        4096ULL,
        0x52F9EECA5BD902C9ULL,
        0x2933E394U,
        -0.4001f,
        -1.2067f,
        0.8066f,
        170U
    },
    {
        "09_VariSpeed_Hermite_Spline_Resample",
        8192ULL,
        0x950D5DE66277BBA5ULL,
        0x326FD468U,
        0.0000f,
        -2.9391f,
        2.9391f,
        182U
    },
    {
        "10_MixerGraph_Console_FullSumming",
        4096ULL,
        0x0B2BEC2665F2E3F9ULL,
        0x73720FFAU,
        0.0000f,
        -2.6509f,
        2.6509f,
        88U
    }
};

// ============================================================================
// 8. Canonical Golden Master Registry & Suite
// ============================================================================
class GoldenMasterSuite {
public:
    // Builds the 10 canonical DSP pipelines covering the entire engine
    static std::vector<PipelineDefinition> create_canonical_pipelines() {
        std::vector<PipelineDefinition> pipes;

        // --------------------------------------------------------------------
        // Pipeline 1: Airwindows Baxandall EQ Impulse Response
        // --------------------------------------------------------------------
        pipes.push_back({
            "01_Airwindows_Baxandall_Impulse",
            48000,
            4096,
            [](std::vector<float>& out_l, std::vector<float>& out_r) {
                constexpr size_t N = 4096;
                ReferenceSignalGenerator::generate_dirac_impulse(out_l, out_r, N, 4, 1.0f);
                dsp::Baxandall bax;
                bax.init(48000);
                bax.set_parameter(0, 4.0f);  // +4.0 dB Bass
                bax.set_parameter(1, -3.0f); // -3.0 dB Treble
                bax.process_stereo(out_l.data(), out_r.data(), N);
            }
        });

        // --------------------------------------------------------------------
        // Pipeline 2: Airwindows ButterComp2 Dynamic Pumping & Release
        // --------------------------------------------------------------------
        pipes.push_back({
            "02_Airwindows_ButterComp2_Pumping",
            48000,
            8192,
            [](std::vector<float>& out_l, std::vector<float>& out_r) {
                constexpr size_t N = 8192;
                ReferenceSignalGenerator::generate_transient_burst(out_l, out_r, N, 48000, 1.0f);
                dsp::ButterComp2 comp;
                comp.init(48000);
                comp.set_parameter(0, 0.70f); // Compress
                comp.set_parameter(1, 0.85f); // Output
                comp.process_stereo(out_l.data(), out_r.data(), N);
            }
        });

        // --------------------------------------------------------------------
        // Pipeline 3: Airwindows PurestDrive Nonlinear Saturation
        // --------------------------------------------------------------------
        pipes.push_back({
            "03_Airwindows_PurestDrive_Sat",
            48000,
            4096,
            [](std::vector<float>& out_l, std::vector<float>& out_r) {
                constexpr size_t N = 4096;
                ReferenceSignalGenerator::generate_sine(out_l, out_r, N, 48000, 1000.0f, 0.95f);
                dsp::PurestDrive drive;
                drive.init(48000);
                drive.set_parameter(0, 0.65f); // Drive
                drive.process_stereo(out_l.data(), out_r.data(), N);
            }
        });

        // --------------------------------------------------------------------
        // Pipeline 4: Airwindows DeRez2 Decimation & Mu-Law Companding
        // --------------------------------------------------------------------
        pipes.push_back({
            "04_Airwindows_DeRez2_MuLaw",
            48000,
            4096,
            [](std::vector<float>& out_l, std::vector<float>& out_r) {
                constexpr size_t N = 4096;
                ReferenceSignalGenerator::generate_complex_chord(out_l, out_r, N, 48000, 0.85f);
                dsp::DeRez derez;
                derez.init(48000);
                derez.set_parameter(0, 0.35f); // Rate
                derez.set_parameter(1, 0.30f); // Resolution
                derez.set_parameter(2, 0.0f);  // Hard = 0.0 (mu-law active)
                derez.set_parameter(3, 1.0f);  // Wet
                derez.process_stereo(out_l.data(), out_r.data(), N);
            }
        });

        // --------------------------------------------------------------------
        // Pipeline 5: Liquid Vactrol LA-2A Leveler Photocell Dark Memory
        // --------------------------------------------------------------------
        pipes.push_back({
            "05_LiquidVactrol_LA2A_Ballistics",
            48000,
            16384,
            [](std::vector<float>& out_l, std::vector<float>& out_r) {
                constexpr size_t N = 16384;
                out_l.assign(N, 0.0f);
                out_r.assign(N, 0.0f);
                // Pulse 1 (10ms transient spike) + Pulse 2 (400ms sustained) + decay
                for (size_t i = 0; i < 480; ++i) {
                    float s = 1.4f * std::sin(2.0 * std::numbers::pi * 1000.0 * static_cast<double>(i) / 48000.0);
                    out_l[i] = s; out_r[i] = s;
                }
                for (size_t i = 1000; i < 1000 + 19200 && i < N; ++i) {
                    float s = 0.8f * std::sin(2.0 * std::numbers::pi * 400.0 * static_cast<double>(i) / 48000.0);
                    out_l[i] = s; out_r[i] = s;
                }
                dsp::LiquidVactrol vactrol(48000);
                vactrol.set_mode(dsp::VactrolMode::OptoCompressor);
                vactrol.set_peak_reduction(0.60f);
                vactrol.set_makeup_gain_db(3.0f);
                vactrol.set_memory_depth(1.0f);
                vactrol.set_hf_emphasis(0.5f);
                vactrol.process_stereo(out_l.data(), out_r.data(), N);
            }
        });

        // --------------------------------------------------------------------
        // Pipeline 6: Liquid Vactrol Buchla 292 LPG Acoustic Ringing Strike
        // --------------------------------------------------------------------
        pipes.push_back({
            "06_LiquidVactrol_BuchlaLPG_Ringing",
            48000,
            8192,
            [](std::vector<float>& out_l, std::vector<float>& out_r) {
                constexpr size_t N = 8192;
                ReferenceSignalGenerator::generate_dirac_impulse(out_l, out_r, N, 0, 1.2f);
                dsp::LiquidVactrol lpg(48000);
                lpg.set_mode(dsp::VactrolMode::BuchlaLPG);
                lpg.set_lpg_resonance(0.45f);
                lpg.process_stereo(out_l.data(), out_r.data(), N);
            }
        });

        // --------------------------------------------------------------------
        // Pipeline 7: MultiHead ODE Compressor 3-Band Kinetic Inertia
        // --------------------------------------------------------------------
        pipes.push_back({
            "07_MultiHeadODE_Compressor_Glue",
            48000,
            8192,
            [](std::vector<float>& out_l, std::vector<float>& out_r) {
                constexpr size_t N = 8192;
                // Composite signal: 50Hz sub + 800Hz mid + 6000Hz treble burst
                out_l.resize(N);
                out_r.resize(N);
                for (size_t i = 0; i < N; ++i) {
                    double t = static_cast<double>(i) / 48000.0;
                    double s_sub = 0.6 * std::sin(2.0 * std::numbers::pi * 50.0 * t);
                    double s_mid = 0.4 * std::sin(2.0 * std::numbers::pi * 800.0 * t);
                    double s_hi = 0.25 * std::sin(2.0 * std::numbers::pi * 6000.0 * t);
                    float env = std::exp(-2.0f * static_cast<float>(t));
                    float val = static_cast<float>(s_sub + s_mid + s_hi) * env;
                    out_l[i] = val; out_r[i] = val;
                }
                dsp::MultiHeadOdeCompressor ode_comp(48000, 4);
                ode_comp.set_crossover_mode(dsp::MultibandCrossoverMode::LinkwitzRileyPhaseCompensated);
                ode_comp.process_stereo(out_l.data(), out_r.data(), N);
            }
        });

        // --------------------------------------------------------------------
        // Pipeline 8: Airwindows ClipOnly2 True-Peak Safety Ceiling
        // --------------------------------------------------------------------
        pipes.push_back({
            "08_Airwindows_ClipOnly2_Ceiling",
            48000,
            4096,
            [](std::vector<float>& out_l, std::vector<float>& out_r) {
                constexpr size_t N = 4096;
                ReferenceSignalGenerator::generate_overdriven_sine(out_l, out_r, N, 48000, 2.5f);
                dsp::ClipOnly2 clipper;
                clipper.process_stereo(out_l.data(), out_r.data(), N);
            }
        });

        // --------------------------------------------------------------------
        // Pipeline 9: VariSpeed Streamer Hermite 4-Point Spline Resampler
        // --------------------------------------------------------------------
        pipes.push_back({
            "09_VariSpeed_Hermite_Spline_Resample",
            48000,
            8192,
            [](std::vector<float>& out_l, std::vector<float>& out_r) {
                constexpr size_t N = 8192;
                constexpr size_t clip_len = 480; // 10ms loop
                auto clip = std::make_shared<sampling::AudioClip>("SineLoop", 48000, 2, clip_len);
                for (size_t i = 0; i < clip_len; ++i) {
                    float s = std::sin(2.0f * std::numbers::pi_v<float> * 440.0f * static_cast<float>(i) / 48000.0f);
                    clip->channel(0)[i] = s;
                    clip->channel(1)[i] = s;
                }
                sampling::VariSpeedStreamer streamer;
                streamer.set_clip(clip);
                streamer.set_loop(true);
                streamer.set_capstan_inertia_ms(0.0f);
                streamer.set_speed_ratio(1.3333333f); // 4-point Hermite interpolation test
                out_l.assign(N, 0.0f);
                out_r.assign(N, 0.0f);
                streamer.render(out_l.data(), out_r.data(), N, 48000, 120.0, true);
            }
        });

        // --------------------------------------------------------------------
        // Pipeline 10: MixerGraph End-to-End Console Summing & Master Chain
        // --------------------------------------------------------------------
        pipes.push_back({
            "10_MixerGraph_Console_FullSumming",
            48000,
            4096,
            [](std::vector<float>& out_l, std::vector<float>& out_r) {
                constexpr size_t N = 4096;
                constexpr uint32_t block_size = 256;
                MixerGraph mixer(block_size, false, 48000);

                Track* trk1 = mixer.allocate_track("Kick");
                Track* trk2 = mixer.allocate_track("Snare");
                Track* trk3 = mixer.allocate_track("Bass");
                Track* trk4 = mixer.allocate_track("Lead");

                AudioBus* drum_bus = mixer.allocate_submix_bus("DrumBus");
                AudioBus* music_bus = mixer.allocate_submix_bus("MusicBus");

                if (trk1 && trk2 && trk3 && trk4 && drum_bus && music_bus) {
                    trk1->set_target_bus(static_cast<int32_t>(drum_bus->id()));
                    trk2->set_target_bus(static_cast<int32_t>(drum_bus->id()));
                    trk3->set_target_bus(static_cast<int32_t>(music_bus->id()));
                    trk4->set_target_bus(static_cast<int32_t>(music_bus->id()));

                    // Synthesize short clips for each track
                    auto clip1 = std::make_shared<sampling::AudioClip>("C1", 48000, 2, 2048);
                    auto clip2 = std::make_shared<sampling::AudioClip>("C2", 48000, 2, 2048);
                    auto clip3 = std::make_shared<sampling::AudioClip>("C3", 48000, 2, 2048);
                    auto clip4 = std::make_shared<sampling::AudioClip>("C4", 48000, 2, 2048);

                    for (size_t i = 0; i < 2048; ++i) {
                        float s1 = std::sin(2.0f * std::numbers::pi_v<float> * 60.0f * static_cast<float>(i) / 48000.0f);
                        float s2 = ((static_cast<float>(i % 37) / 18.5f) - 1.0f) * 0.5f;
                        float s3 = std::sin(2.0f * std::numbers::pi_v<float> * 110.0f * static_cast<float>(i) / 48000.0f);
                        float s4 = std::sin(2.0f * std::numbers::pi_v<float> * 440.0f * static_cast<float>(i) / 48000.0f);
                        clip1->channel(0)[i] = s1; clip1->channel(1)[i] = s1;
                        clip2->channel(0)[i] = s2; clip2->channel(1)[i] = s2;
                        clip3->channel(0)[i] = s3; clip3->channel(1)[i] = s3;
                        clip4->channel(0)[i] = s4; clip4->channel(1)[i] = s4;
                    }

                    trk1->set_clip(clip1, true);
                    trk2->set_clip(clip2, true);
                    trk3->set_clip(clip3, true);
                    trk4->set_clip(clip4, true);

                    trk1->set_sync_to_transport(false);
                    trk2->set_sync_to_transport(false);
                    trk3->set_sync_to_transport(false);
                    trk4->set_sync_to_transport(false);

                    trk1->set_console_type(dsp::ConsoleType::Purest);
                    trk2->set_console_type(dsp::ConsoleType::Purest);
                    trk3->set_console_type(dsp::ConsoleType::Purest);
                    trk4->set_console_type(dsp::ConsoleType::Purest);
                    drum_bus->set_console_type(dsp::ConsoleType::Purest);
                    music_bus->set_console_type(dsp::ConsoleType::Purest);
                    mixer.master_bus().set_console_type(dsp::ConsoleType::Purest);
                }

                out_l.assign(N, 0.0f);
                out_r.assign(N, 0.0f);

                AudioBuffer out_buf(2, block_size);
                auto view = out_buf.view();

                for (size_t offset = 0; offset < N; offset += block_size) {
                    mixer.render(view);
                    const float* bl = view.channel(0);
                    const float* br = view.channel(1);
                    for (size_t b = 0; b < block_size && (offset + b) < N; ++b) {
                        out_l[offset + b] = bl[b];
                        out_r[offset + b] = br[b];
                    }
                }
            }
        });

        return pipes;
    }

    static bool verify_all(std::vector<GoldenMasterDiff>* diffs_out = nullptr) {
        auto pipelines = create_canonical_pipelines();
        bool all_passed = true;

        for (size_t i = 0; i < pipelines.size() && i < 10; ++i) {
            const auto& pipe = pipelines[i];
            const auto& ref = kCanonicalGoldenMasters[i];

            std::vector<float> left;
            std::vector<float> right;
            pipe.render_func(left, right);

            GoldenMasterMetrics m = GoldenMasterAnalyzer::analyze(
                pipe.name, left.data(), right.data(), pipe.frames, pipe.sample_rate);

            GoldenMasterDiff diff;
            diff.pipeline_name = pipe.name;
            diff.expected_fnv64 = ref.fnv1a64;
            diff.actual_fnv64 = m.pcm24_fnv1a64;
            diff.expected_crc32 = ref.crc32;
            diff.actual_crc32 = m.pcm24_crc32;
            diff.expected_peak_db = ref.peak_dbfs;
            diff.actual_peak_db = m.peak_dbfs;
            diff.expected_rms_db = ref.rms_dbfs;
            diff.actual_rms_db = m.rms_dbfs;

            // Check bit-exact match or tight mathematical tolerance
            bool hash_match = (m.pcm24_fnv1a64 == ref.fnv1a64 && m.pcm24_crc32 == ref.crc32);
            bool metrics_match = (std::abs(m.peak_dbfs - ref.peak_dbfs) < 0.05f &&
                                  std::abs(m.rms_dbfs - ref.rms_dbfs) < 0.05f &&
                                  m.zero_crossings == ref.zero_crossings);

            diff.passed = hash_match && metrics_match;
            if (!diff.passed) {
                all_passed = false;
                diff.error_message = "Hash or metric mismatch against canonical reference";
            }

            if (diffs_out) {
                diffs_out->push_back(diff);
            }
        }

        return all_passed;
    }
};

} // namespace audio_core::analysis
