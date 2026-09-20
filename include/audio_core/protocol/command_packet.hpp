#pragma once

#include <cstdint>
#include <cstddef>

namespace audio_core::protocol {

enum class MixerCommandType : uint16_t {
    None = 0,
    SetTrackGain,
    SetTrackPan,
    SetTrackMute,
    SetTrackSolo,
    SetTrackTargetBus,
    SetTrackSend,
    SetTrackConsoleType,
    SetBusGain,
    SetBusConsoleType,
    SetMasterGain,
    SetMasterLimiter,
    ResetMeters,
};

// ============================================================================
// MixerCommand: 32-Byte Cache-Aligned POD Packet
// Designed for zero-copy, lock-free dispatch between UI/Network and RT Audio
// ============================================================================
struct alignas(32) MixerCommand {
    MixerCommandType type{MixerCommandType::None};
    uint16_t sample_offset{0};   // Sample-exact timing within audio block [0..frames-1]
    uint32_t target_id{0};       // Track ID or Bus ID
    uint32_t secondary_id{0};    // e.g. Aux Bus ID for sends, or Submix target ID
    float value1{0.0f};          // Fader gain, pan (-1..1), send amount, etc.
    float value2{0.0f};          // Secondary value
    uint32_t flags{0};           // Bitflags (e.g., bool mute/solo, pre_fader flag, enum)
    uint32_t reserved{0};        // Explicit 4-byte padding to exact 32 bytes
};

static_assert(sizeof(MixerCommand) == 32, "MixerCommand must be exactly 32 bytes for cache alignment");

} // namespace audio_core::protocol
