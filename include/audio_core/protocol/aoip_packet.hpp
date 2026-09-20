#pragma once

#include <cstdint>
#include <cstddef>
#include <span>
#include <cstring>

namespace audio_core::protocol {

// Magic Header: "AETH" in ASCII = 0x41455448
constexpr uint32_t kAoipMagic = 0x41455448;
constexpr uint16_t kAoipVersion = 1;

enum class AoipPayloadFormat : uint16_t {
    Float32_Interleaved = 0,
    Float32_Planar = 1,
    Int24_Packed = 2,
    Int16_Interleaved = 3,
};

// ============================================================================
// AoipHeader: 32-Byte Packed Binary Network Header
// Wire format for Aethel-AoIP streaming across Linux (PipeWire) and Android
// ============================================================================
#pragma pack(push, 1)
struct AoipHeader {
    uint32_t magic;            // 0x41455448 ('AETH')
    uint16_t version;          // Protocol version (1)
    uint16_t payload_format;   // AoipPayloadFormat
    uint64_t sequence_number;  // Monotonic packet counter for ROC FEC & loss detection
    uint64_t timestamp_ns;     // Monotonic nanosecond timestamp (PTP / Ableton Link)
    uint32_t sample_rate;      // e.g. 48000, 44100, 96000
    uint16_t channels;         // e.g. 2 (stereo), 8 (multichannel)
    uint16_t frames;           // Frames in packet (e.g. 64, 128, 256)
};
#pragma pack(pop)

static_assert(sizeof(AoipHeader) == 32, "AoipHeader must be exactly 32 bytes packed");

// Helper to validate incoming network packet header in zero-copy manner
[[nodiscard]] inline bool validate_aoip_packet(const uint8_t* data, size_t size_bytes) noexcept {
    if (size_bytes < sizeof(AoipHeader)) {
        return false;
    }
    const auto* hdr = reinterpret_cast<const AoipHeader*>(data);
    if (hdr->magic != kAoipMagic || hdr->version != kAoipVersion) {
        return false;
    }
    // Check expected payload size for Float32
    if (hdr->payload_format == static_cast<uint16_t>(AoipPayloadFormat::Float32_Interleaved) ||
        hdr->payload_format == static_cast<uint16_t>(AoipPayloadFormat::Float32_Planar)) {
        size_t expected_payload = static_cast<size_t>(hdr->channels) * hdr->frames * sizeof(float);
        return (size_bytes == sizeof(AoipHeader) + expected_payload);
    }
    return true;
}

} // namespace audio_core::protocol
