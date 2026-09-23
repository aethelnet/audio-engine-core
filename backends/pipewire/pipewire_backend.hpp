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
    bool is_mono{false};
    uint32_t node_id{0};
    uint32_t channel_count{2};
    std::string physical_port_name;
};

// ============================================================================
// PipeWireBackend: Native Linux Audio Server Integration via pw_filter
// Exposes virtual input sinks for each mixer track (so external apps can route in)
// and output sinks for master/buses (to DAC or network).
// Features:
// 1. Live Stream Discovery: Asynchronous crawler for Linux apps and devices
// 2. 1-Click Track Patching: Connect external audio directly to any channel strip
// 3. Selective Ingestion: Respects TrackInputMode (Clip vs Stream vs Merge)
// 4. Multi-Channel Hardware I/O: Discrete mono and stereo routing per track
// 5. Dynamic Master Sink Switching: Seamless DAC / soundcard switching on the fly
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
    [[nodiscard]] std::vector<DiscoveredStreamPair> get_hardware_inputs() const;
    [[nodiscard]] std::vector<DiscoveredStreamPair> get_app_sources() const;

    // Real-time Dynamic Port Patching (Connecting external apps/hardware to tracks)
    bool link_source_to_track(const DiscoveredStreamPair& stream, uint32_t track_id);
    bool unlink_source_from_track(const DiscoveredStreamPair& stream, uint32_t track_id);
    bool unlink_all_for_track(uint32_t track_id);
    [[nodiscard]] std::optional<DiscoveredStreamPair> get_track_source(uint32_t track_id) const;
    [[nodiscard]] bool is_track_linked(uint32_t track_id) const;

    // Master Audio Hardware Output Device Selection (Dynamic DAC / Sink Switching)
    bool connect_master_to_sink(const std::string& sink_node_name);
    bool disconnect_master_output();
    [[nodiscard]] std::string active_master_sink_node_name() const;
    [[nodiscard]] std::string active_master_sink_display_name() const;

    // AoIP Network Receiver Integration
    void set_aoip_receiver(network::AoipReceiver* receiver) noexcept;
    [[nodiscard]] network::AoipReceiver* aoip_receiver() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace audio_core
