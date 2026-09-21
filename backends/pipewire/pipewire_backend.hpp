#pragma once

#include "audio_core/types.hpp"
#include "audio_core/mixer_graph.hpp"
#include <memory>
#include <string>
#include <vector>
#include <cstdint>

namespace audio_core {
namespace network {
class AoipReceiver;
}

// ============================================================================
// DiscoveredStreamPair: Discovered PipeWire Audio Port Pair
// Represents external apps (Firefox, Spotify, Carla), mics, and monitors
// ============================================================================
struct DiscoveredStreamPair {
    std::string node_name;
    std::string display_name;
    std::string port_l;
    std::string port_r;
    bool is_hardware_capture{false};
    bool is_monitor{false};
};

// ============================================================================
// PipeWireBackend: Native Linux Audio Server Integration via pw_filter
// Exposes virtual input sinks for each mixer track (so external apps can route in)
// and output sinks for master/buses (to DAC or network).
// Features:
// 1. Live Stream Discovery: Asynchronous crawler for Linux apps and devices
// 2. 1-Click Track Patching: Connect external audio directly to any channel strip
// 3. Selective Ingestion: Respects TrackInputMode (Clip vs Stream vs Merge)
// ============================================================================
class PipeWireBackend {
public:
    explicit PipeWireBackend(MixerGraph& mixer);
    ~PipeWireBackend();

    PipeWireBackend(const PipeWireBackend&) = delete;
    PipeWireBackend& operator=(const PipeWireBackend&) = delete;
    PipeWireBackend(PipeWireBackend&&) noexcept;
    PipeWireBackend& operator=(PipeWireBackend&&) noexcept;

    // Initialize PipeWire connection and create filter node
    bool init(const std::string& node_name = "Aethel Engine", uint32_t sample_rate = 48000);
    bool start();
    void stop();

    [[nodiscard]] bool is_running() const noexcept;
    [[nodiscard]] uint32_t sample_rate() const noexcept;

    // Stream & Device Discovery
    void refresh_discovery();
    [[nodiscard]] std::vector<DiscoveredStreamPair> get_available_sources() const;
    [[nodiscard]] std::vector<DiscoveredStreamPair> get_available_sinks() const;

    // Real-time Dynamic Port Patching (Connecting external apps to tracks)
    bool link_source_to_track(const DiscoveredStreamPair& stream, uint32_t track_id);
    bool unlink_source_from_track(const DiscoveredStreamPair& stream, uint32_t track_id);
    bool unlink_all_for_track(uint32_t track_id);

    // AoIP Network Receiver Integration
    void set_aoip_receiver(network::AoipReceiver* receiver) noexcept;
    [[nodiscard]] network::AoipReceiver* aoip_receiver() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace audio_core
