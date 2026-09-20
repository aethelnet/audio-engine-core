#pragma once

#include <cstdint>
#include <cstddef>

namespace audio_core::protocol {

// Compact 16-byte aligned meter data (L/R Peak & RMS)
struct alignas(16) ChannelMeterData {
    float peak_l{0.0f};
    float peak_r{0.0f};
    float rms_l{0.0f};
    float rms_r{0.0f};
};

static_assert(sizeof(ChannelMeterData) == 16, "ChannelMeterData must be 16 bytes");

constexpr size_t kMaxTelemetryTracks = 32;

// ============================================================================
// MixerTelemetryFrame: Cache-aligned snapshot for 60Hz UI refresh
// Written lock-free by RT audio thread, consumed zero-copy by UI
// ============================================================================
struct alignas(64) MixerTelemetryFrame {
    uint64_t render_cycle{0};     // Monotonic audio callback counter
    uint64_t timestamp_ns{0};     // Monotonic timestamp
    uint32_t active_tracks{0};    // Number of active tracks
    uint32_t active_buses{0};     // Number of active buses
    ChannelMeterData master_meter{};
    ChannelMeterData track_meters[kMaxTelemetryTracks]{};
};

} // namespace audio_core::protocol
