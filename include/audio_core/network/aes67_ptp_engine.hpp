#pragma once

#include "audio_core/types.hpp"
#include <cstdint>
#include <cstddef>
#include <span>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <cstring>

namespace audio_core::network {

// ============================================================================
// IEEE 1588-2008 Precision Time Protocol (PTPv2) Epoch Timestamp
// Time distribution clock for Dante / AES67 / Ravenna interoperability
// ============================================================================
struct PtpTimestamp {
    uint64_t seconds{0};        // TAI seconds since 1970-01-01 00:00:00 TAI
    uint32_t nanoseconds{0};    // Nanoseconds (0 to 999,999,999)

    [[nodiscard]] double to_seconds() const noexcept {
        return static_cast<double>(seconds) + static_cast<double>(nanoseconds) * 1e-9;
    }

    [[nodiscard]] uint64_t to_nanoseconds() const noexcept {
        return (seconds * 1'000'000'000ULL) + static_cast<uint64_t>(nanoseconds);
    }

    static PtpTimestamp from_nanoseconds(uint64_t ns) noexcept {
        return PtpTimestamp{
            .seconds = ns / 1'000'000'000ULL,
            .nanoseconds = static_cast<uint32_t>(ns % 1'000'000'000ULL)
        };
    }

    // Convert PTP continuous time into an RTP Media Clock 32-bit timestamp
    [[nodiscard]] uint32_t to_rtp_timestamp(uint32_t sample_rate) const noexcept {
        const double total_samples = to_seconds() * static_cast<double>(sample_rate);
        const auto samples_u64 = static_cast<uint64_t>(std::floor(total_samples));
        return static_cast<uint32_t>(samples_u64 & 0xFFFFFFFF);
    }
};

// ============================================================================
// RFC 3550 Real-time Transport Protocol (RTP) Header for AES67 / Dante
// Exactly 12 bytes packed, network big-endian byte order
// ============================================================================
#pragma pack(push, 1)
struct RtpHeader {
    uint8_t flags{0x80};         // V=2, P=0, X=0, CC=0 (0x80 standard)
    uint8_t payload_type{96};    // Dynamic payload type (e.g. 96 for L24/48000)
    uint16_t sequence_number{0}; // Monotonic packet counter (Big-Endian)
    uint32_t timestamp{0};       // Media Clock Timestamp at Fs (Big-Endian)
    uint32_t ssrc{0};            // Synchronization Source Identifier (Big-Endian)
};
#pragma pack(pop)

static_assert(sizeof(RtpHeader) == 12, "RtpHeader must be exactly 12 bytes packed");

// Endianness conversion helpers (Host <-> Network Big-Endian)
inline uint16_t host_to_net16(uint16_t val) noexcept {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return static_cast<uint16_t>((val << 8) | (val >> 8));
#else
    return val;
#endif
}

inline uint16_t net_to_host16(uint16_t val) noexcept {
    return host_to_net16(val);
}

inline uint32_t host_to_net32(uint32_t val) noexcept {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return __builtin_bswap32(val);
#else
    return val;
#endif
}

inline uint32_t net_to_host32(uint32_t val) noexcept {
    return host_to_net32(val);
}

// ============================================================================
// Aes67StreamConfig: Parameters for an AES67 / Dante RTP Multicast Stream
// ============================================================================
enum class Aes67PayloadEncoding : uint8_t {
    L24 = 0, // Standard 24-bit linear PCM (3 bytes/sample, Big-Endian)
    L16 = 1, // 16-bit linear PCM (2 bytes/sample, Big-Endian)
};

struct Aes67StreamConfig {
    uint32_t sample_rate{48000};
    uint16_t num_channels{8};          // Up to 64 channels
    uint16_t packet_frames{48};        // 48 frames = 1.0 ms packet time at 48kHz (Standard AES67 profile)
    uint8_t payload_type{96};          // Dynamic RTP PT (typically 96-127)
    uint32_t ssrc{0x41455448};         // 'AETH'
    Aes67PayloadEncoding encoding{Aes67PayloadEncoding::L24};

    [[nodiscard]] size_t bytes_per_sample() const noexcept {
        return (encoding == Aes67PayloadEncoding::L24) ? 3 : 2;
    }

    [[nodiscard]] size_t payload_bytes() const noexcept {
        return static_cast<size_t>(num_channels) * packet_frames * bytes_per_sample();
    }

    [[nodiscard]] size_t total_packet_bytes() const noexcept {
        return sizeof(RtpHeader) + payload_bytes();
    }
};

// ============================================================================
// Aes67PacketSerializer: Real-Time Zero-Allocation AES67 Transmitter Frame Pack
// Serializes planar float audio buffers into RFC 3550 RTP + L24/L16 packets
// ============================================================================
class Aes67PacketSerializer {
public:
    explicit Aes67PacketSerializer(const Aes67StreamConfig& config) noexcept
        : m_config(config) {}

    void set_config(const Aes67StreamConfig& config) noexcept {
        m_config = config;
    }
    [[nodiscard]] const Aes67StreamConfig& config() const noexcept { return m_config; }

    // Serialize planar channel pointers into caller-provided packet buffer
    // Returns number of bytes written, or 0 on buffer overflow / invalid input
    size_t serialize_packet(const float* const* channel_data,
                            uint32_t frames,
                            const PtpTimestamp& ptp_time,
                            uint8_t* out_buffer,
                            size_t out_buffer_capacity) noexcept {
        if (!channel_data || !out_buffer || frames == 0) return 0;
        const size_t req_bytes = sizeof(RtpHeader) + (static_cast<size_t>(m_config.num_channels) * frames * m_config.bytes_per_sample());
        if (out_buffer_capacity < req_bytes) return 0;

        // 1. Populate RFC 3550 RTP Header
        auto* hdr = reinterpret_cast<RtpHeader*>(out_buffer);
        hdr->flags = 0x80; // V=2, P=0, X=0, CC=0
        hdr->payload_type = m_config.payload_type & 0x7F;
        hdr->sequence_number = host_to_net16(m_sequence_number++);
        hdr->timestamp = host_to_net32(ptp_time.to_rtp_timestamp(m_config.sample_rate));
        hdr->ssrc = host_to_net32(m_config.ssrc);

        // 2. Interleave and encode planar float samples into big-endian PCM
        uint8_t* payload_ptr = out_buffer + sizeof(RtpHeader);

        if (m_config.encoding == Aes67PayloadEncoding::L24) {
            // L24: 24-bit Signed Integer, Big-Endian (MSB first)
            for (uint32_t f = 0; f < frames; ++f) {
                for (uint16_t c = 0; c < m_config.num_channels; ++c) {
                    float s = channel_data[c] ? channel_data[c][f] : 0.0f;
                    s = std::clamp(s, -1.0f, 1.0f);
                    int32_t val = static_cast<int32_t>(s * 8388607.0f);
                    if (val > 8388607) val = 8388607;
                    if (val < -8388608) val = -8388608;

                    *payload_ptr++ = static_cast<uint8_t>((val >> 16) & 0xFF); // MSB
                    *payload_ptr++ = static_cast<uint8_t>((val >> 8) & 0xFF);
                    *payload_ptr++ = static_cast<uint8_t>(val & 0xFF);         // LSB
                }
            }
        } else {
            // L16: 16-bit Signed Integer, Big-Endian
            for (uint32_t f = 0; f < frames; ++f) {
                for (uint16_t c = 0; c < m_config.num_channels; ++c) {
                    float s = channel_data[c] ? channel_data[c][f] : 0.0f;
                    s = std::clamp(s, -1.0f, 1.0f);
                    int16_t val = static_cast<int16_t>(s * 32767.0f);
                    *payload_ptr++ = static_cast<uint8_t>((val >> 8) & 0xFF); // MSB
                    *payload_ptr++ = static_cast<uint8_t>(val & 0xFF);        // LSB
                }
            }
        }

        return req_bytes;
    }

    void reset() noexcept {
        m_sequence_number = 0;
    }

private:
    Aes67StreamConfig m_config;
    uint16_t m_sequence_number{0};
};

// ============================================================================
// Aes67PacketDeserializer: Zero-Allocation AES67 Receiver & Depacketizer
// Unpacks network RTP packets into planar float buffers
// ============================================================================
class Aes67PacketDeserializer {
public:
    explicit Aes67PacketDeserializer(const Aes67StreamConfig& config) noexcept
        : m_config(config) {}

    void set_config(const Aes67StreamConfig& config) noexcept {
        m_config = config;
    }

    // Depacketize incoming RTP byte payload into planar float buffers
    // Returns number of frames extracted, or 0 on error
    uint32_t deserialize_packet(const uint8_t* in_buffer,
                                size_t in_buffer_size,
                                float* const* out_channels,
                                uint16_t max_channels,
                                uint32_t max_frames,
                                uint16_t& out_seq,
                                uint32_t& out_rtp_timestamp) noexcept {
        if (!in_buffer || !out_channels || in_buffer_size < sizeof(RtpHeader)) return 0;

        const auto* hdr = reinterpret_cast<const RtpHeader*>(in_buffer);
        // Validate RTP version 2
        if ((hdr->flags & 0xC0) != 0x80) return 0;

        out_seq = net_to_host16(hdr->sequence_number);
        out_rtp_timestamp = net_to_host32(hdr->timestamp);

        const size_t payload_bytes = in_buffer_size - sizeof(RtpHeader);
        const size_t bytes_per_frame = static_cast<size_t>(m_config.num_channels) * m_config.bytes_per_sample();
        if (bytes_per_frame == 0) return 0;

        const uint32_t frames_in_packet = static_cast<uint32_t>(payload_bytes / bytes_per_frame);
        const uint32_t frames_to_write = std::min(frames_in_packet, max_frames);
        const uint16_t channels_to_write = std::min(m_config.num_channels, max_channels);

        const uint8_t* payload_ptr = in_buffer + sizeof(RtpHeader);

        if (m_config.encoding == Aes67PayloadEncoding::L24) {
            constexpr float kInv24 = 1.0f / 8388607.0f;
            for (uint32_t f = 0; f < frames_to_write; ++f) {
                for (uint16_t c = 0; c < m_config.num_channels; ++c) {
                    uint8_t b0 = *payload_ptr++;
                    uint8_t b1 = *payload_ptr++;
                    uint8_t b2 = *payload_ptr++;

                    // Sign-extend 24-bit to 32-bit int
                    int32_t val = (static_cast<int32_t>(b0) << 24) |
                                  (static_cast<int32_t>(b1) << 16) |
                                  (static_cast<int32_t>(b2) << 8);
                    val >>= 8; // Arithmetic shift preserves sign bit

                    if (c < channels_to_write && out_channels[c]) {
                        out_channels[c][f] = static_cast<float>(val) * kInv24;
                    }
                }
            }
        } else {
            constexpr float kInv16 = 1.0f / 32767.0f;
            for (uint32_t f = 0; f < frames_to_write; ++f) {
                for (uint16_t c = 0; c < m_config.num_channels; ++c) {
                    uint8_t b0 = *payload_ptr++;
                    uint8_t b1 = *payload_ptr++;

                    int16_t val = static_cast<int16_t>((static_cast<int32_t>(b0) << 8) | b1);
                    if (c < channels_to_write && out_channels[c]) {
                        out_channels[c][f] = static_cast<float>(val) * kInv16;
                    }
                }
            }
        }

        // Detect packet loss
        if (m_has_previous_seq) {
            uint16_t expected = m_last_seq + 1;
            if (out_seq != expected) {
                uint16_t diff = out_seq - expected;
                m_packets_lost += diff;
            }
        }
        m_last_seq = out_seq;
        m_has_previous_seq = true;

        return frames_to_write;
    }

    [[nodiscard]] uint64_t packets_lost() const noexcept { return m_packets_lost; }

    void reset() noexcept {
        m_has_previous_seq = false;
        m_last_seq = 0;
        m_packets_lost = 0;
    }

private:
    Aes67StreamConfig m_config;
    bool m_has_previous_seq{false};
    uint16_t m_last_seq{0};
    uint64_t m_packets_lost{0};
};

} // namespace audio_core::network
