#pragma once

#include "audio_core/types.hpp"
#include "audio_core/sampling/audio_clip.hpp"
#include "audio_core/sampling/wav_reader.hpp"
#include "audio_core/mixer_graph.hpp"
#include "audio_core/clock/timeline_clock.hpp"
#include "audio_core/serialization/session_serializer.hpp"

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <numbers>

namespace audio_core::sampling {

// ============================================================================
// SampleDescriptor: Sononym & Renoise Inspired Perceptual Audio Metadata
// Captures musical pitch, dynamic range, transient density and content hash
// for robust, self-healing sample management and asset bundling.
// ============================================================================
struct SampleDescriptor {
    std::string asset_id;               // 16-hex deterministic content hash (FNV-1a 64-bit)
    std::string file_name;              // Original filename (e.g. "breakbeat_126bpm.wav")
    std::string relative_path;          // Path relative to bundle root (e.g. "assets/breakbeat.wav")
    uint32_t sample_rate{48000};
    uint32_t channels{2};
    uint64_t total_frames{0};
    double duration_sec{0.0};
    float peak_db{-96.0f};              // True peak amplitude in dBFS
    float rms_db{-96.0f};               // Integrated RMS level in dBFS
    float crest_factor_db{0.0f};        // Dynamic punchiness (Peak dB - RMS dB)
    float fundamental_hz{0.0f};         // Estimated F0 pitch
    std::string musical_key{"--"};      // Detected note (e.g. "C1", "F#3")
    float estimated_bpm{0.0f};          // Rhythmic loop tempo
    uint32_t transient_count{0};        // Detected percussive onset events
    std::string perceptual_category;    // "Kick / Sub", "Snare / Clap", "Loop / Stem", etc.

    [[nodiscard]] std::string to_json() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "{\n"
           << "  \"asset_id\": \"" << asset_id << "\",\n"
           << "  \"file_name\": \"" << file_name << "\",\n"
           << "  \"relative_path\": \"" << relative_path << "\",\n"
           << "  \"sample_rate\": " << sample_rate << ",\n"
           << "  \"channels\": " << channels << ",\n"
           << "  \"total_frames\": " << total_frames << ",\n"
           << "  \"duration_sec\": " << duration_sec << ",\n"
           << "  \"peak_db\": " << peak_db << ",\n"
           << "  \"rms_db\": " << rms_db << ",\n"
           << "  \"crest_factor_db\": " << crest_factor_db << ",\n"
           << "  \"fundamental_hz\": " << fundamental_hz << ",\n"
           << "  \"musical_key\": \"" << musical_key << "\",\n"
           << "  \"estimated_bpm\": " << estimated_bpm << ",\n"
           << "  \"transient_count\": " << transient_count << ",\n"
           << "  \"perceptual_category\": \"" << perceptual_category << "\"\n"
           << "}";
        return ss.str();
    }
};

// ============================================================================
// SampleAnalyzer: Lock-Free Feature Extraction & Audio Content Fingerprinting
// ============================================================================
class SampleAnalyzer {
public:
    static inline uint64_t fnv1a_64(const void* data, size_t num_bytes) noexcept {
        constexpr uint64_t kFnvPrime = 0x100000001b3ULL;
        constexpr uint64_t kFnvOffsetBasis = 0xcbf29ce484222325ULL;
        const auto* ptr = static_cast<const uint8_t*>(data);
        uint64_t hash = kFnvOffsetBasis;
        for (size_t i = 0; i < num_bytes; ++i) {
            hash ^= static_cast<uint64_t>(ptr[i]);
            hash *= kFnvPrime;
        }
        return hash;
    }

    static inline std::string hex_hash(uint64_t val) {
        std::ostringstream ss;
        ss << std::hex << std::setw(16) << std::setfill('0') << val;
        return ss.str();
    }

    static inline std::string hz_to_note_name(float hz) noexcept {
        if (hz < 20.0f || hz > 5000.0f) return "--";
        const float midi_num = 69.0f + 12.0f * std::log2(hz / 440.0f);
        const int note_idx = static_cast<int>(std::round(midi_num));
        if (note_idx < 0 || note_idx > 127) return "--";

        static const char* kNoteNames[12] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
        int octave = (note_idx / 12) - 1;
        int semitone = note_idx % 12;
        return std::string(kNoteNames[semitone]) + std::to_string(octave);
    }

    // Comprehensive audio feature extraction across planar channels
    static SampleDescriptor analyze(const AudioClip& clip, const std::string& file_name = "") {
        SampleDescriptor desc{};
        desc.file_name = file_name.empty() ? clip.name() : file_name;
        desc.sample_rate = clip.sample_rate();
        desc.channels = clip.num_channels();
        desc.total_frames = clip.num_frames();
        desc.duration_sec = (desc.sample_rate > 0) ? (static_cast<double>(desc.total_frames) / desc.sample_rate) : 0.0;

        if (desc.total_frames == 0 || desc.channels == 0) {
            desc.asset_id = "0000000000000000";
            desc.perceptual_category = "Empty Buffer";
            return desc;
        }

        // 1. Compute Deterministic 64-bit FNV-1a Content Hash across all samples
        uint64_t combined_hash = 0xcbf29ce484222325ULL;
        for (uint32_t ch = 0; ch < desc.channels; ++ch) {
            const auto span = clip.channel_data(ch);
            uint64_t ch_hash = fnv1a_64(span.data(), span.size() * sizeof(float));
            combined_hash ^= ch_hash;
            combined_hash *= 0x100000001b3ULL;
        }
        desc.asset_id = hex_hash(combined_hash);
        desc.relative_path = "assets/" + desc.asset_id + ".wav";

        // 2. Dynamics: Peak & RMS in dBFS
        float max_abs = 0.0f;
        double sum_sq = 0.0;
        const auto l_span = clip.channel_data(0);
        const auto r_span = (desc.channels > 1) ? clip.channel_data(1) : l_span;

        for (size_t i = 0; i < desc.total_frames; ++i) {
            float l = l_span[i];
            float r = r_span[i];
            float pk = std::max(std::abs(l), std::abs(r));
            if (pk > max_abs) max_abs = pk;
            sum_sq += static_cast<double>(l * l + r * r) * 0.5;
        }

        double mean_sq = sum_sq / static_cast<double>(desc.total_frames);
        float rms = static_cast<float>(std::sqrt(mean_sq));

        desc.peak_db = (max_abs > 1e-5f) ? (20.0f * std::log10(max_abs)) : -96.0f;
        desc.rms_db = (rms > 1e-5f) ? (20.0f * std::log10(rms)) : -96.0f;
        desc.crest_factor_db = std::max(0.0f, desc.peak_db - desc.rms_db);

        // 3. Pitch Estimation (Zero-Crossing + Short Autocorrelation on downsampled stream)
        const size_t max_corr_frames = std::min<size_t>(desc.total_frames, 4096);
        if (max_corr_frames >= 256) {
            int zero_crossings = 0;
            for (size_t i = 1; i < max_corr_frames; ++i) {
                if ((l_span[i] >= 0.0f && l_span[i - 1] < 0.0f) || (l_span[i] < 0.0f && l_span[i - 1] >= 0.0f)) {
                    zero_crossings++;
                }
            }
            float zc_freq = (static_cast<float>(zero_crossings) * 0.5f * desc.sample_rate) / max_corr_frames;

            // Autocorrelation peak refinement between 30 Hz and 1500 Hz
            size_t min_lag = desc.sample_rate / 1500;
            size_t max_lag = std::min<size_t>(desc.sample_rate / 30, max_corr_frames / 2);
            double best_corr = -1.0;
            size_t best_lag = 0;

            for (size_t lag = min_lag; lag < max_lag; ++lag) {
                double corr = 0.0;
                for (size_t i = 0; i < max_corr_frames - lag; ++i) {
                    corr += l_span[i] * l_span[i + lag];
                }
                if (corr > best_corr) {
                    best_corr = corr;
                    best_lag = lag;
                }
            }

            if (best_lag > 0 && best_corr > 0.0) {
                float auto_freq = static_cast<float>(desc.sample_rate) / static_cast<float>(best_lag);
                // Fuse autocorrelation with zero crossing confirmation
                desc.fundamental_hz = (std::abs(auto_freq - zc_freq) < auto_freq * 0.35f) ? auto_freq : zc_freq;
            } else {
                desc.fundamental_hz = zc_freq;
            }
            desc.musical_key = hz_to_note_name(desc.fundamental_hz);
        }

        // 4. Transient Count & Rhythmic Loop Detection
        uint32_t transients = 0;
        constexpr size_t kBlockSize = 256;
        float prev_energy = 0.0f;
        std::vector<size_t> transient_onsets;

        for (size_t i = 0; i + kBlockSize <= desc.total_frames; i += kBlockSize) {
            float energy = 0.0f;
            for (size_t j = 0; j < kBlockSize; ++j) {
                float s = l_span[i + j];
                energy += s * s;
            }
            if (i > 0 && energy > prev_energy * 2.2f && energy > 0.02f) {
                transients++;
                transient_onsets.push_back(i);
            }
            prev_energy = energy;
        }
        desc.transient_count = transients;

        // Estimate BPM if loop has rhythmic transients
        if (transient_onsets.size() >= 4 && desc.duration_sec >= 1.0) {
            double avg_spacing_sec = desc.duration_sec / static_cast<double>(transients);
            float rough_bpm = static_cast<float>(60.0 / avg_spacing_sec);
            // Fold into standard DAW tempo range [70, 180]
            while (rough_bpm < 70.0f) rough_bpm *= 2.0f;
            while (rough_bpm > 180.0f) rough_bpm *= 0.5f;
            desc.estimated_bpm = std::round(rough_bpm * 10.0f) / 10.0f;
        }

        // 5. Sononym Perceptual Category Classification
        if (desc.duration_sec > 6.0) {
            desc.perceptual_category = "Full Stem / Long Take";
        } else if (desc.crest_factor_db > 14.0f && desc.transient_count >= 3) {
            desc.perceptual_category = "Percussive Loop / Breakbeat";
        } else if (desc.crest_factor_db > 15.0f && desc.duration_sec < 0.6) {
            if (desc.fundamental_hz < 120.0f && desc.fundamental_hz > 30.0f) {
                desc.perceptual_category = "Kick / Sub Bass";
            } else {
                desc.perceptual_category = "Snare / Clap / Rimshot";
            }
        } else if (desc.fundamental_hz > 40.0f && desc.fundamental_hz < 220.0f) {
            desc.perceptual_category = "Bassline / 303 Acid Synth";
        } else if (desc.crest_factor_db < 9.0f) {
            desc.perceptual_category = "Sustained Pad / Texture";
        } else {
            desc.perceptual_category = "Melodic Hook / Vocal Chop";
        }

        return desc;
    }
};

// ============================================================================
// SampleAssetPool: Central Library & Content-Addressable Asset Cache
// Matches samples by SHA/FNV-1a hash rather than brittle file paths.
// ============================================================================
class SampleAssetPool {
public:
    static SampleAssetPool& instance() noexcept {
        static SampleAssetPool s_instance;
        return s_instance;
    }

    void register_asset(const std::shared_ptr<AudioClip>& clip, const std::string& path = "") {
        if (!clip) return;
        auto desc = SampleAnalyzer::analyze(*clip, path);
        m_pool[desc.asset_id] = { desc, clip };
        if (!desc.file_name.empty()) {
            m_name_to_id[desc.file_name] = desc.asset_id;
        }
    }

    [[nodiscard]] std::shared_ptr<AudioClip> find_by_id(const std::string& asset_id) const {
        auto it = m_pool.find(asset_id);
        return (it != m_pool.end()) ? it->second.clip : nullptr;
    }

    [[nodiscard]] std::shared_ptr<AudioClip> find_by_name(const std::string& file_name) const {
        auto it_name = m_name_to_id.find(file_name);
        if (it_name != m_name_to_id.end()) {
            return find_by_id(it_name->second);
        }
        return nullptr;
    }

    [[nodiscard]] const SampleDescriptor* get_descriptor(const std::string& asset_id) const {
        auto it = m_pool.find(asset_id);
        return (it != m_pool.end()) ? &it->second.descriptor : nullptr;
    }

    [[nodiscard]] size_t size() const noexcept {
        return m_pool.size();
    }

    void clear() noexcept {
        m_pool.clear();
        m_name_to_id.clear();
    }

    template <typename Callback>
    void for_each(Callback&& cb) const {
        for (const auto& [id, entry] : m_pool) {
            cb(entry.descriptor, entry.clip);
        }
    }

    [[nodiscard]] std::vector<SampleDescriptor> all_descriptors() const {
        std::vector<SampleDescriptor> res;
        res.reserve(m_pool.size());
        for (const auto& [id, entry] : m_pool) {
            res.push_back(entry.descriptor);
        }
        return res;
    }

private:
    struct Entry {
        SampleDescriptor descriptor;
        std::shared_ptr<AudioClip> clip;
    };
    std::unordered_map<std::string, Entry> m_pool;
    std::unordered_map<std::string, std::string> m_name_to_id;
};

// ============================================================================
// ProjectBundleManager: Self-Contained Project Packaging (.aethel)
// Packages JSON session topology + assets/ audio directory with 0 missing files.
// ============================================================================
class ProjectBundleManager {
public:
    static bool save_bundle(const std::string& bundle_dir,
                            const MixerGraph& mixer,
                            const clock::TimelineClock& clock,
                            const std::vector<std::shared_ptr<AudioClip>>& clips,
                            const std::string& project_name = "Aethel Project",
                            const modulation::ModulationMatrix* mod_matrix = nullptr,
                            const midi::MidiLearnRouter* midi_learn = nullptr) {
        namespace fs = std::filesystem;
        std::error_code ec;
        fs::create_directories(bundle_dir, ec);
        if (ec) return false;

        fs::path assets_path = fs::path(bundle_dir) / "assets";
        fs::create_directories(assets_path, ec);
        if (ec) return false;

        // 1. Export all clips into assets/ folder with content-hashed names
        std::ostringstream manifest;
        manifest << "[\n";
        bool first = true;

        for (const auto& clip : clips) {
            if (!clip || clip->num_frames() == 0) continue;
            auto desc = SampleAnalyzer::analyze(*clip);

            // Save WAV file to bundle assets directory
            std::string wav_filename = desc.asset_id + ".wav";
            fs::path dest_file = assets_path / wav_filename;
            desc.relative_path = "assets/" + wav_filename;

            const auto l_span = clip->channel_data(0);
            const auto r_span = (clip->num_channels() > 1) ? clip->channel_data(1) : l_span;
            (void)WavReader::save_wav(dest_file.string(), l_span.data(), r_span.data(),
                                      clip->num_frames(), clip->sample_rate(), 24);

            if (!first) manifest << ",\n";
            manifest << desc.to_json();
            first = false;
        }
        manifest << "\n]\n";

        // 2. Write Manifest JSON
        fs::path manifest_path = fs::path(bundle_dir) / "assets_manifest.json";
        std::ofstream manifest_file(manifest_path);
        if (!manifest_file.is_open()) return false;
        manifest_file << manifest.str();

        // 3. Write Session JSON
        fs::path project_path = fs::path(bundle_dir) / "project.json";
        return serialization::SessionSerializer::save_session_file(
            project_path.string(), mixer, clock, project_name, mod_matrix, midi_learn);
    }

    static bool load_bundle(const std::string& bundle_dir,
                            MixerGraph& mixer,
                            clock::TimelineClock& clock,
                            std::vector<std::shared_ptr<AudioClip>>& out_clips,
                            modulation::ModulationMatrix* mod_matrix = nullptr,
                            midi::MidiLearnRouter* midi_learn = nullptr) {
        namespace fs = std::filesystem;
        fs::path project_path = fs::path(bundle_dir) / "project.json";
        fs::path assets_path = fs::path(bundle_dir) / "assets";

        if (!fs::exists(project_path)) return false;

        // 1. Load Session Topology
        if (!serialization::SessionSerializer::load_session_file(
                project_path.string(), mixer, clock, mod_matrix, midi_learn)) {
            return false;
        }

        // 2. Scan assets directory and populate out_clips & SampleAssetPool
        out_clips.clear();
        std::error_code ec;
        if (fs::exists(assets_path, ec)) {
            for (const auto& entry : fs::directory_iterator(assets_path, ec)) {
                if (entry.is_regular_file() && entry.path().extension() == ".wav") {
                    auto clip = std::make_shared<AudioClip>();
                    if (clip->load_from_wav(entry.path().string())) {
                        out_clips.push_back(clip);
                        SampleAssetPool::instance().register_asset(clip, entry.path().filename().string());
                    }
                }
            }
        }

        return true;
    }
};

} // namespace audio_core::sampling
