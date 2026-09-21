#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <fstream>
#include <cstring>
#include <algorithm>
#include <cmath>

namespace audio_core::sampling {

// ============================================================================
// Standard RIFF / WAVE Audio File Reader & Writer
// Zero external dependencies, IEEE 754 Float & 16/24/32-Bit PCM support.
// Robust against arbitrary chunk ordering (LIST, JUNK, bext, metadata).
// ============================================================================
class WavReader {
public:
    struct WavInfo {
        uint32_t sample_rate{48000};
        uint16_t channels{2};
        uint16_t bits_per_sample{16};
        uint16_t audio_format{1}; // 1 = PCM, 3 = IEEE Float
        uint32_t total_frames{0};
        bool valid{false};
    };

    // Load WAV file from disk into planar stereo/multichannel float buffers [-1.0, 1.0]
    static bool load_wav(const std::string& filepath,
                         std::vector<std::vector<float>>& out_channels,
                         uint32_t& out_sample_rate) {
        std::ifstream file(filepath, std::ios::binary);
        if (!file.is_open()) return false;

        char header_tag[4];
        file.read(header_tag, 4);
        if (std::strncmp(header_tag, "RIFF", 4) != 0) return false;

        uint32_t file_size_minus_8 = 0;
        file.read(reinterpret_cast<char*>(&file_size_minus_8), 4);

        file.read(header_tag, 4);
        if (std::strncmp(header_tag, "WAVE", 4) != 0) return false;

        WavInfo info{};
        std::vector<uint8_t> raw_audio_data;

        // Traverse all RIFF subchunks until both fmt and data chunks are processed
        while (file) {
            char chunk_id[4];
            uint32_t chunk_size = 0;
            if (!file.read(chunk_id, 4)) break;
            if (!file.read(reinterpret_cast<char*>(&chunk_size), 4)) break;

            if (std::strncmp(chunk_id, "fmt ", 4) == 0) {
                if (chunk_size < 16) return false;
                file.read(reinterpret_cast<char*>(&info.audio_format), 2);
                file.read(reinterpret_cast<char*>(&info.channels), 2);
                file.read(reinterpret_cast<char*>(&info.sample_rate), 4);

                uint32_t byte_rate = 0;
                uint16_t block_align = 0;
                file.read(reinterpret_cast<char*>(&byte_rate), 4);
                file.read(reinterpret_cast<char*>(&block_align), 2);
                file.read(reinterpret_cast<char*>(&info.bits_per_sample), 2);

                // Skip any extended header bytes
                if (chunk_size > 16) {
                    file.seekg(chunk_size - 16, std::ios::cur);
                }
                info.valid = true;
            } else if (std::strncmp(chunk_id, "data", 4) == 0) {
                raw_audio_data.resize(chunk_size);
                file.read(reinterpret_cast<char*>(raw_audio_data.data()), chunk_size);
                // RIFF chunks are padded to 2-byte alignment
                if (chunk_size % 2 != 0) {
                    file.seekg(1, std::ios::cur);
                }
                break; // Got audio data
            } else {
                // Skip unknown chunk (JUNK, LIST, bext, PEAK, etc.)
                file.seekg(chunk_size + (chunk_size % 2), std::ios::cur);
            }
        }

        if (!info.valid || raw_audio_data.empty() || info.channels == 0) {
            return false;
        }

        const uint32_t bytes_per_sample = info.bits_per_sample / 8;
        if (bytes_per_sample == 0) return false;

        const uint32_t block_align = info.channels * bytes_per_sample;
        const uint32_t total_frames = static_cast<uint32_t>(raw_audio_data.size() / block_align);
        if (total_frames == 0) return false;

        // If mono, expand to stereo channels for consistent playback
        uint32_t channels_to_alloc = (info.channels == 1) ? 2 : info.channels;
        out_channels.assign(channels_to_alloc, std::vector<float>(total_frames, 0.0f));
        out_sample_rate = info.sample_rate;

        const uint8_t* ptr = raw_audio_data.data();

        // 1. 16-Bit Signed Little-Endian PCM
        if (info.audio_format == 1 && info.bits_per_sample == 16) {
            for (uint32_t i = 0; i < total_frames; ++i) {
                for (uint32_t ch = 0; ch < info.channels; ++ch) {
                    int16_t sample_int = static_cast<int16_t>(ptr[0] | (ptr[1] << 8));
                    ptr += 2;
                    float val = static_cast<float>(sample_int) / 32768.0f;
                    val = std::clamp(val, -1.0f, 1.0f);
                    out_channels[ch][i] = val;
                    if (info.channels == 1) {
                        out_channels[1][i] = val; // duplicate mono
                    }
                }
            }
        }
        // 2. 24-Bit Signed Little-Endian PCM
        else if (info.audio_format == 1 && info.bits_per_sample == 24) {
            for (uint32_t i = 0; i < total_frames; ++i) {
                for (uint32_t ch = 0; ch < info.channels; ++ch) {
                    int32_t sample_int = static_cast<int32_t>(ptr[0] | (ptr[1] << 8) | (ptr[2] << 16));
                    if (sample_int & 0x800000) {
                        sample_int |= ~0xFFFFFF; // Sign extension
                    }
                    ptr += 3;
                    float val = static_cast<float>(sample_int) / 8388608.0f;
                    val = std::clamp(val, -1.0f, 1.0f);
                    out_channels[ch][i] = val;
                    if (info.channels == 1) {
                        out_channels[1][i] = val;
                    }
                }
            }
        }
        // 3. 32-Bit Signed Little-Endian PCM
        else if (info.audio_format == 1 && info.bits_per_sample == 32) {
            for (uint32_t i = 0; i < total_frames; ++i) {
                for (uint32_t ch = 0; ch < info.channels; ++ch) {
                    int32_t sample_int = 0;
                    std::memcpy(&sample_int, ptr, 4);
                    ptr += 4;
                    float val = static_cast<float>(sample_int) / 2147483648.0f;
                    val = std::clamp(val, -1.0f, 1.0f);
                    out_channels[ch][i] = val;
                    if (info.channels == 1) {
                        out_channels[1][i] = val;
                    }
                }
            }
        }
        // 4. 32-Bit IEEE 754 Floating Point PCM
        else if (info.audio_format == 3 && info.bits_per_sample == 32) {
            for (uint32_t i = 0; i < total_frames; ++i) {
                for (uint32_t ch = 0; ch < info.channels; ++ch) {
                    float val = 0.0f;
                    std::memcpy(&val, ptr, 4);
                    ptr += 4;
                    // Sanitize NaN / Inf
                    if (!std::isfinite(val)) val = 0.0f;
                    out_channels[ch][i] = val;
                    if (info.channels == 1) {
                        out_channels[1][i] = val;
                    }
                }
            }
        } else {
            return false; // Unsupported format combination
        }

        return true;
    }

    // Save planar audio channels to disk as 16-Bit or 24-Bit PCM or 32-Bit Float WAV
    static bool save_wav(const std::string& filepath,
                         const float* left,
                         const float* right,
                         size_t frames,
                         uint32_t sample_rate,
                         uint16_t bits_per_sample = 24) {
        if (!left || frames == 0 || sample_rate == 0) return false;
        const float* r_ptr = right ? right : left;

        std::ofstream out(filepath, std::ios::binary);
        if (!out.is_open()) return false;

        const uint16_t num_channels = 2;
        const uint16_t bytes_per_sample = bits_per_sample / 8;
        const uint16_t block_align = num_channels * bytes_per_sample;
        const uint32_t byte_rate = sample_rate * block_align;
        const uint32_t data_bytes = static_cast<uint32_t>(frames * block_align);
        const uint32_t file_size = 36 + data_bytes;
        const uint16_t audio_format = (bits_per_sample == 32) ? 3 : 1; // 3=Float, 1=PCM

        // RIFF Header
        out.write("RIFF", 4);
        out.write(reinterpret_cast<const char*>(&file_size), 4);
        out.write("WAVE", 4);

        // fmt Subchunk
        out.write("fmt ", 4);
        const uint32_t fmt_size = 16;
        out.write(reinterpret_cast<const char*>(&fmt_size), 4);
        out.write(reinterpret_cast<const char*>(&audio_format), 2);
        out.write(reinterpret_cast<const char*>(&num_channels), 2);
        out.write(reinterpret_cast<const char*>(&sample_rate), 4);
        out.write(reinterpret_cast<const char*>(&byte_rate), 4);
        out.write(reinterpret_cast<const char*>(&block_align), 2);
        out.write(reinterpret_cast<const char*>(&bits_per_sample), 2);

        // data Subchunk
        out.write("data", 4);
        out.write(reinterpret_cast<const char*>(&data_bytes), 4);

        if (bits_per_sample == 16) {
            for (size_t i = 0; i < frames; ++i) {
                for (int ch = 0; ch < 2; ++ch) {
                    float s = (ch == 0) ? left[i] : r_ptr[i];
                    s = std::clamp(s, -1.0f, 1.0f);
                    int16_t val = static_cast<int16_t>(std::round(s * 32767.0f));
                    out.write(reinterpret_cast<const char*>(&val), 2);
                }
            }
        } else if (bits_per_sample == 24) {
            for (size_t i = 0; i < frames; ++i) {
                for (int ch = 0; ch < 2; ++ch) {
                    float s = (ch == 0) ? left[i] : r_ptr[i];
                    s = std::clamp(s, -1.0f, 1.0f);
                    int32_t val = static_cast<int32_t>(std::round(s * 8388607.0f));
                    if (val > 8388607) val = 8388607;
                    if (val < -8388608) val = -8388608;
                    uint8_t b0 = static_cast<uint8_t>(val & 0xFF);
                    uint8_t b1 = static_cast<uint8_t>((val >> 8) & 0xFF);
                    uint8_t b2 = static_cast<uint8_t>((val >> 16) & 0xFF);
                    out.write(reinterpret_cast<const char*>(&b0), 1);
                    out.write(reinterpret_cast<const char*>(&b1), 1);
                    out.write(reinterpret_cast<const char*>(&b2), 1);
                }
            }
        } else if (bits_per_sample == 32) {
            for (size_t i = 0; i < frames; ++i) {
                for (int ch = 0; ch < 2; ++ch) {
                    float s = (ch == 0) ? left[i] : r_ptr[i];
                    if (!std::isfinite(s)) s = 0.0f;
                    out.write(reinterpret_cast<const char*>(&s), 4);
                }
            }
        }

        return true;
    }
};

} // namespace audio_core::sampling
