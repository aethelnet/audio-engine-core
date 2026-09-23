#pragma once

#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <unordered_map>
#include <span>
#include "audio_core/sampling/wav_reader.hpp"

namespace audio_core::sampling {

// ============================================================================
// PeakPair: Bounded Amplitude Range & Energy RMS Metric for a Sample Block
// Provides dual-envelope rendering (transient peak spikes + solid RMS musical core)
// ============================================================================
struct PeakPair {
    float min_val{0.0f};
    float max_val{0.0f};
    float rms{0.0f};
};

struct ViewportPeak {
    float min_val{0.0f};
    float max_val{0.0f};
    float rms{0.0f};
};

// ============================================================================
// WaveformOverview: Multi-Resolution Waveform Peak Mipmapping Engine
// Solves 60 FPS rendering for multi-minute/multi-hour audio stems and long takes.
//
// Pyramidal reduction levels (Factor of 4 per level):
//   Level 0:    64 frames / peak  (1:64)   - Highest resolution zoomed-in view
//   Level 1:   256 frames / peak  (1:256)  - 4-bar phrase view
//   Level 2:  1024 frames / peak  (1:1024) - 16-bar section view
//   Level 3:  4096 frames / peak  (1:4096) - 1-minute overview
//   Level 4: 16384 frames / peak  (1:16384)- 5-minute song overview
//   Level 5: 65536 frames / peak  (1:65536)- Multi-hour stem overview
// ============================================================================
class WaveformOverview {
public:
    static constexpr size_t kNumLevels = 6;
    static constexpr uint32_t kLevelBlockSizes[kNumLevels] = {
        64, 256, 1024, 4096, 16384, 65536
    };
    static constexpr uint32_t kMagicHeader = 0x43564F41; // "AOVC" in Little Endian
    static constexpr uint32_t kFormatVersion = 1;

    WaveformOverview() = default;

    ~WaveformOverview() {
        cancel_and_join();
    }

    // Non-copyable, movable
    WaveformOverview(const WaveformOverview&) = delete;
    WaveformOverview& operator=(const WaveformOverview&) = delete;

    WaveformOverview(WaveformOverview&& other) noexcept {
        *this = std::move(other);
    }

    WaveformOverview& operator=(WaveformOverview&& other) noexcept {
        if (this != &other) {
            cancel_and_join();
            other.cancel_and_join();

            std::lock_guard<std::mutex> lock(other.m_data_mutex);
            m_sample_rate = other.m_sample_rate;
            m_num_channels = other.m_num_channels;
            m_total_frames = other.m_total_frames;
            m_channel_levels = std::move(other.m_channel_levels);
            m_is_ready.store(other.m_is_ready.load());
            m_progress.store(other.m_progress.load());
        }
        return *this;
    }

    // Cancel active background build thread and join safely
    void cancel_and_join() {
        m_cancel_requested.store(true, std::memory_order_release);
        if (m_worker && m_worker->joinable()) {
            m_worker->join();
        }
        m_worker.reset();
        m_cancel_requested.store(false, std::memory_order_relaxed);
    }

    [[nodiscard]] bool is_ready() const noexcept {
        return m_is_ready.load(std::memory_order_acquire);
    }

    [[nodiscard]] float progress() const noexcept {
        return m_progress.load(std::memory_order_relaxed);
    }

    [[nodiscard]] uint32_t sample_rate() const noexcept { return m_sample_rate; }
    [[nodiscard]] uint32_t num_channels() const noexcept { return m_num_channels; }
    [[nodiscard]] uint64_t total_frames() const noexcept { return m_total_frames; }

    [[nodiscard]] size_t num_peaks(uint32_t channel, size_t level) const noexcept {
        std::lock_guard<std::mutex> lock(m_data_mutex);
        if (channel >= m_channel_levels.size() || level >= kNumLevels) return 0;
        return m_channel_levels[channel][level].size();
    }

    [[nodiscard]] const std::vector<PeakPair>& peaks(uint32_t channel, size_t level) const {
        std::lock_guard<std::mutex> lock(m_data_mutex);
        static const std::vector<PeakPair> kEmpty;
        if (channel >= m_channel_levels.size() || level >= kNumLevels) return kEmpty;
        return m_channel_levels[channel][level];
    }

    // ========================================================================
    // Synchronous In-Memory Pyramid Builder
    // Builds all 6 mipmap levels directly from planar audio channels.
    // ========================================================================
    bool build_synchronous(const float* const* channel_data,
                           uint32_t channels,
                           uint64_t total_frames,
                           uint32_t sample_rate) {
        cancel_and_join();

        if (!channel_data || channels == 0 || total_frames == 0) {
            clear();
            return false;
        }

        m_sample_rate = sample_rate;
        m_num_channels = channels;
        m_total_frames = total_frames;

        std::lock_guard<std::mutex> lock(m_data_mutex);
        m_channel_levels.resize(channels);

        for (uint32_t ch = 0; ch < channels; ++ch) {
            m_channel_levels[ch].resize(kNumLevels);
            const float* src = channel_data[ch];

            // 1. Build Level 0 (Block size 64)
            const uint32_t b0 = kLevelBlockSizes[0];
            const size_t num_l0 = (total_frames + b0 - 1) / b0;
            auto& l0 = m_channel_levels[ch][0];
            l0.resize(num_l0);

            for (size_t i = 0; i < num_l0; ++i) {
                const uint64_t start_f = i * b0;
                const uint64_t end_f = std::min(start_f + b0, total_frames);
                const uint64_t block_len = end_f - start_f;

                float min_v = (src && block_len > 0) ? src[start_f] : 0.0f;
                float max_v = min_v;
                double sum_sq = 0.0;

                if (src) {
                    for (uint64_t f = start_f; f < end_f; ++f) {
                        const float v = src[f];
                        if (v < min_v) min_v = v;
                        if (v > max_v) max_v = v;
                        sum_sq += static_cast<double>(v) * static_cast<double>(v);
                    }
                }

                const float rms = (block_len > 0)
                    ? static_cast<float>(std::sqrt(sum_sq / static_cast<double>(block_len)))
                    : 0.0f;

                l0[i] = PeakPair{min_v, max_v, rms};
            }

            // 2. Build Levels 1..5 via 4:1 hierarchical reduction from previous level
            for (size_t lvl = 1; lvl < kNumLevels; ++lvl) {
                const auto& prev = m_channel_levels[ch][lvl - 1];
                const size_t num_prev = prev.size();
                const size_t num_cur = (num_prev + 3) / 4;
                auto& cur = m_channel_levels[ch][lvl];
                cur.resize(num_cur);

                for (size_t i = 0; i < num_cur; ++i) {
                    const size_t p_start = i * 4;
                    const size_t p_end = std::min(p_start + 4, num_prev);

                    float min_v = prev[p_start].min_val;
                    float max_v = prev[p_start].max_val;
                    double sum_rms_sq = 0.0;
                    size_t count = 0;

                    for (size_t p = p_start; p < p_end; ++p) {
                        if (prev[p].min_val < min_v) min_v = prev[p].min_val;
                        if (prev[p].max_val > max_v) max_v = prev[p].max_val;
                        sum_rms_sq += static_cast<double>(prev[p].rms) * static_cast<double>(prev[p].rms);
                        count++;
                    }

                    const float rms = (count > 0)
                        ? static_cast<float>(std::sqrt(sum_rms_sq / static_cast<double>(count)))
                        : 0.0f;

                    cur[i] = PeakPair{min_v, max_v, rms};
                }
            }
        }

        m_progress.store(1.0f, std::memory_order_relaxed);
        m_is_ready.store(true, std::memory_order_release);
        return true;
    }

    // Convenience overload for 1D or stereo planar buffers
    bool build_synchronous(const std::vector<std::vector<float>>& channels, uint32_t sample_rate) {
        if (channels.empty() || channels[0].empty()) return false;
        std::vector<const float*> ptrs(channels.size());
        for (size_t i = 0; i < channels.size(); ++i) {
            ptrs[i] = channels[i].data();
        }
        return build_synchronous(ptrs.data(), static_cast<uint32_t>(channels.size()),
                                 channels[0].size(), sample_rate);
    }

    // ========================================================================
    // Asynchronous Background Worker Builder
    // Spawns a background thread that calculates the peak pyramid in blocks,
    // continually reporting progress without blocking UI or audio thread.
    // ========================================================================
    void build_async(std::vector<std::vector<float>> channels, uint32_t sample_rate) {
        cancel_and_join();

        if (channels.empty() || channels[0].empty()) {
            clear();
            return;
        }

        m_sample_rate = sample_rate;
        m_num_channels = static_cast<uint32_t>(channels.size());
        m_total_frames = channels[0].size();
        m_is_ready.store(false, std::memory_order_release);
        m_progress.store(0.0f, std::memory_order_relaxed);

        m_worker = std::make_unique<std::thread>([this, audio_data = std::move(channels)]() mutable {
            const uint32_t ch_count = static_cast<uint32_t>(audio_data.size());
            const uint64_t frames = audio_data[0].size();
            const uint32_t b0 = kLevelBlockSizes[0];
            const size_t num_l0 = (frames + b0 - 1) / b0;

            std::vector<std::vector<std::vector<PeakPair>>> temp_levels(ch_count);
            for (uint32_t ch = 0; ch < ch_count; ++ch) {
                temp_levels[ch].resize(kNumLevels);
                temp_levels[ch][0].resize(num_l0);
            }

            // Chunked processing for progress updates and cancel responsiveness
            constexpr size_t kChunkBlocks = 1024; // 1024 * 64 = 65,536 frames per progress check
            for (size_t chunk_start = 0; chunk_start < num_l0; chunk_start += kChunkBlocks) {
                if (m_cancel_requested.load(std::memory_order_relaxed)) {
                    return;
                }

                const size_t chunk_end = std::min(chunk_start + kChunkBlocks, num_l0);
                for (uint32_t ch = 0; ch < ch_count; ++ch) {
                    const float* src = audio_data[ch].data();
                    auto& l0 = temp_levels[ch][0];

                    for (size_t i = chunk_start; i < chunk_end; ++i) {
                        const uint64_t start_f = i * b0;
                        const uint64_t end_f = std::min(start_f + b0, frames);
                        const uint64_t block_len = end_f - start_f;

                        float min_v = (block_len > 0) ? src[start_f] : 0.0f;
                        float max_v = min_v;
                        double sum_sq = 0.0;

                        for (uint64_t f = start_f; f < end_f; ++f) {
                            const float v = src[f];
                            if (v < min_v) min_v = v;
                            if (v > max_v) max_v = v;
                            sum_sq += static_cast<double>(v) * static_cast<double>(v);
                        }

                        const float rms = (block_len > 0)
                            ? static_cast<float>(std::sqrt(sum_sq / static_cast<double>(block_len)))
                            : 0.0f;

                        l0[i] = PeakPair{min_v, max_v, rms};
                    }
                }

                const float p = 0.85f * (static_cast<float>(chunk_end) / static_cast<float>(num_l0));
                m_progress.store(p, std::memory_order_relaxed);
            }

            // Build pyramid levels 1..5
            for (uint32_t ch = 0; ch < ch_count; ++ch) {
                if (m_cancel_requested.load(std::memory_order_relaxed)) return;

                for (size_t lvl = 1; lvl < kNumLevels; ++lvl) {
                    const auto& prev = temp_levels[ch][lvl - 1];
                    const size_t num_prev = prev.size();
                    const size_t num_cur = (num_prev + 3) / 4;
                    auto& cur = temp_levels[ch][lvl];
                    cur.resize(num_cur);

                    for (size_t i = 0; i < num_cur; ++i) {
                        const size_t p_start = i * 4;
                        const size_t p_end = std::min(p_start + 4, num_prev);

                        float min_v = prev[p_start].min_val;
                        float max_v = prev[p_start].max_val;
                        double sum_rms_sq = 0.0;
                        size_t count = 0;

                        for (size_t p = p_start; p < p_end; ++p) {
                            if (prev[p].min_val < min_v) min_v = prev[p].min_val;
                            if (prev[p].max_val > max_v) max_v = prev[p].max_val;
                            sum_rms_sq += static_cast<double>(prev[p].rms) * static_cast<double>(prev[p].rms);
                            count++;
                        }

                        const float rms = (count > 0)
                            ? static_cast<float>(std::sqrt(sum_rms_sq / static_cast<double>(count)))
                            : 0.0f;

                        cur[i] = PeakPair{min_v, max_v, rms};
                    }
                }
            }

            {
                std::lock_guard<std::mutex> lock(m_data_mutex);
                m_channel_levels = std::move(temp_levels);
            }

            m_progress.store(1.0f, std::memory_order_relaxed);
            m_is_ready.store(true, std::memory_order_release);
        });
    }

    // ========================================================================
    // O(W) Viewport Query: Fast Peak Extraction for Display
    // Returns exact min, max, and rms for each screen pixel column in O(W) time.
    // Handles arbitrary zoom from whole-song overview down to sub-beat detail.
    // ========================================================================
    void query_peaks(uint32_t channel,
                     uint64_t start_frame,
                     uint64_t end_frame,
                     size_t num_pixels,
                     std::vector<ViewportPeak>& out_peaks) const {
        out_peaks.resize(num_pixels);
        if (num_pixels == 0) return;

        if (!m_is_ready.load(std::memory_order_acquire) || m_total_frames == 0) {
            std::fill(out_peaks.begin(), out_peaks.end(), ViewportPeak{0.0f, 0.0f, 0.0f});
            return;
        }

        std::lock_guard<std::mutex> lock(m_data_mutex);
        if (channel >= m_channel_levels.size()) {
            std::fill(out_peaks.begin(), out_peaks.end(), ViewportPeak{0.0f, 0.0f, 0.0f});
            return;
        }

        if (end_frame <= start_frame) {
            std::fill(out_peaks.begin(), out_peaks.end(), ViewportPeak{0.0f, 0.0f, 0.0f});
            return;
        }

        const uint64_t clamped_start = std::min(start_frame, m_total_frames);
        const uint64_t clamped_end   = std::min(end_frame, m_total_frames);
        const uint64_t frame_range   = (clamped_end > clamped_start) ? (clamped_end - clamped_start) : 1;

        const double frames_per_pixel = static_cast<double>(frame_range) / static_cast<double>(num_pixels);

        // Select the finest mipmap level L such that kLevelBlockSizes[L] <= frames_per_pixel
        // If frames_per_pixel < 64, use Level 0 (highest detail)
        size_t chosen_level = 0;
        for (int lvl = static_cast<int>(kNumLevels) - 1; lvl >= 0; --lvl) {
            if (static_cast<double>(kLevelBlockSizes[lvl]) <= frames_per_pixel) {
                chosen_level = static_cast<size_t>(lvl);
                break;
            }
        }

        const auto& level_peaks = m_channel_levels[channel][chosen_level];
        const size_t total_level_peaks = level_peaks.size();
        const uint32_t block_size = kLevelBlockSizes[chosen_level];

        if (total_level_peaks == 0) {
            std::fill(out_peaks.begin(), out_peaks.end(), ViewportPeak{0.0f, 0.0f, 0.0f});
            return;
        }

        for (size_t px = 0; px < num_pixels; ++px) {
            const double px_start_f = static_cast<double>(clamped_start) + static_cast<double>(px) * frames_per_pixel;
            const double px_end_f   = px_start_f + frames_per_pixel;

            const uint64_t f0 = static_cast<uint64_t>(std::clamp(px_start_f, 0.0, static_cast<double>(m_total_frames)));
            const uint64_t f1 = static_cast<uint64_t>(std::clamp(px_end_f, static_cast<double>(f0), static_cast<double>(m_total_frames)));

            size_t idx0 = static_cast<size_t>(f0 / block_size);
            size_t idx1 = static_cast<size_t>((f1 + block_size - 1) / block_size);

            if (idx0 >= total_level_peaks) idx0 = (total_level_peaks > 0) ? (total_level_peaks - 1) : 0;
            if (idx1 > total_level_peaks) idx1 = total_level_peaks;
            if (idx1 <= idx0) idx1 = std::min(idx0 + 1, total_level_peaks);

            float min_v = level_peaks[idx0].min_val;
            float max_v = level_peaks[idx0].max_val;
            double sum_rms_sq = 0.0;
            size_t count = 0;

            for (size_t p = idx0; p < idx1; ++p) {
                if (level_peaks[p].min_val < min_v) min_v = level_peaks[p].min_val;
                if (level_peaks[p].max_val > max_v) max_v = level_peaks[p].max_val;
                sum_rms_sq += static_cast<double>(level_peaks[p].rms) * static_cast<double>(level_peaks[p].rms);
                count++;
            }

            const float rms = (count > 0)
                ? static_cast<float>(std::sqrt(sum_rms_sq / static_cast<double>(count)))
                : 0.0f;

            out_peaks[px] = ViewportPeak{min_v, max_v, rms};
        }
    }

    // ========================================================================
    // File Persistence: Save/Load Binary Overview Cache (.aov)
    // Avoids re-computing peaks upon subsequent project opens.
    // ========================================================================
    bool save_to_file(const std::string& path) const {
        std::lock_guard<std::mutex> lock(m_data_mutex);
        if (!m_is_ready.load(std::memory_order_acquire) || m_channel_levels.empty()) {
            return false;
        }

        std::ofstream out(path, std::ios::binary);
        if (!out.is_open()) return false;

        const uint32_t magic = kMagicHeader;
        const uint32_t version = kFormatVersion;
        const uint32_t sr = m_sample_rate;
        const uint32_t ch_count = static_cast<uint32_t>(m_channel_levels.size());
        const uint64_t total_f = m_total_frames;
        const uint32_t num_lvls = static_cast<uint32_t>(kNumLevels);

        out.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
        out.write(reinterpret_cast<const char*>(&version), sizeof(version));
        out.write(reinterpret_cast<const char*>(&sr), sizeof(sr));
        out.write(reinterpret_cast<const char*>(&ch_count), sizeof(ch_count));
        out.write(reinterpret_cast<const char*>(&total_f), sizeof(total_f));
        out.write(reinterpret_cast<const char*>(&num_lvls), sizeof(num_lvls));

        for (uint32_t ch = 0; ch < ch_count; ++ch) {
            for (size_t lvl = 0; lvl < kNumLevels; ++lvl) {
                const auto& p = m_channel_levels[ch][lvl];
                const uint64_t count = static_cast<uint64_t>(p.size());
                out.write(reinterpret_cast<const char*>(&count), sizeof(count));
                if (count > 0) {
                    out.write(reinterpret_cast<const char*>(p.data()), count * sizeof(PeakPair));
                }
            }
        }

        return out.good();
    }

    bool load_from_file(const std::string& path) {
        cancel_and_join();

        std::ifstream in(path, std::ios::binary);
        if (!in.is_open()) return false;

        uint32_t magic = 0;
        uint32_t version = 0;
        uint32_t sr = 0;
        uint32_t ch_count = 0;
        uint64_t total_f = 0;
        uint32_t num_lvls = 0;

        in.read(reinterpret_cast<char*>(&magic), sizeof(magic));
        in.read(reinterpret_cast<char*>(&version), sizeof(version));
        if (magic != kMagicHeader || version != kFormatVersion) {
            return false;
        }

        in.read(reinterpret_cast<char*>(&sr), sizeof(sr));
        in.read(reinterpret_cast<char*>(&ch_count), sizeof(ch_count));
        in.read(reinterpret_cast<char*>(&total_f), sizeof(total_f));
        in.read(reinterpret_cast<char*>(&num_lvls), sizeof(num_lvls));

        if (ch_count == 0 || num_lvls != kNumLevels || total_f == 0) {
            return false;
        }

        std::lock_guard<std::mutex> lock(m_data_mutex);
        m_sample_rate = sr;
        m_num_channels = ch_count;
        m_total_frames = total_f;
        m_channel_levels.resize(ch_count);

        for (uint32_t ch = 0; ch < ch_count; ++ch) {
            m_channel_levels[ch].resize(kNumLevels);
            for (size_t lvl = 0; lvl < kNumLevels; ++lvl) {
                uint64_t count = 0;
                in.read(reinterpret_cast<char*>(&count), sizeof(count));
                auto& p = m_channel_levels[ch][lvl];
                p.resize(count);
                if (count > 0) {
                    in.read(reinterpret_cast<char*>(p.data()), count * sizeof(PeakPair));
                }
            }
        }

        if (!in.good()) {
            clear();
            return false;
        }

        m_progress.store(1.0f, std::memory_order_relaxed);
        m_is_ready.store(true, std::memory_order_release);
        return true;
    }

    void clear() {
        cancel_and_join();
        std::lock_guard<std::mutex> lock(m_data_mutex);
        m_channel_levels.clear();
        m_total_frames = 0;
        m_num_channels = 0;
        m_sample_rate = 0;
        m_is_ready.store(false, std::memory_order_release);
        m_progress.store(0.0f, std::memory_order_relaxed);
    }

private:
    uint32_t m_sample_rate{48000};
    uint32_t m_num_channels{1};
    uint64_t m_total_frames{0};

    mutable std::mutex m_data_mutex;
    std::vector<std::vector<std::vector<PeakPair>>> m_channel_levels;

    std::atomic<bool> m_is_ready{false};
    std::atomic<float> m_progress{0.0f};

    std::atomic<bool> m_cancel_requested{false};
    std::unique_ptr<std::thread> m_worker;
};

// ============================================================================
// WaveformOverviewCache: Global Registry & Disk-Cached Overview Manager
// ============================================================================
class WaveformOverviewCache {
public:
    static WaveformOverviewCache& instance() {
        static WaveformOverviewCache s_inst;
        return s_inst;
    }

    std::shared_ptr<WaveformOverview> get_or_build(const std::string& path,
                                                   bool async = false,
                                                   bool persist_aov = true) {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_cache.find(path);
        if (it != m_cache.end() && it->second) {
            return it->second;
        }

        auto overview = std::make_shared<WaveformOverview>();
        std::string aov_path = path + ".aov";

        // 1. Try loading cached binary .aov overview from disk
        if (overview->load_from_file(aov_path)) {
            m_cache[path] = overview;
            return overview;
        }

        // 2. Otherwise load audio file and build pyramid
        std::vector<std::vector<float>> channels;
        uint32_t sample_rate = 0;
        if (WavReader::load_wav(path, channels, sample_rate)) {
            if (async) {
                overview->build_async(std::move(channels), sample_rate);
            } else {
                overview->build_synchronous(channels, sample_rate);
                if (persist_aov) {
                    overview->save_to_file(aov_path);
                }
            }
            m_cache[path] = overview;
            return overview;
        }

        return nullptr;
    }

    void put(const std::string& path, std::shared_ptr<WaveformOverview> overview) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_cache[path] = std::move(overview);
    }

    void remove(const std::string& path) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_cache.erase(path);
    }

    void clear() {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_cache.clear();
    }

private:
    std::mutex m_mutex;
    std::unordered_map<std::string, std::shared_ptr<WaveformOverview>> m_cache;
};

} // namespace audio_core::sampling
