#pragma once

#include "audio_core/types.hpp"
#include "audio_core/mixer_graph.hpp"
#include <memory>
#include <string>
#include <cstdint>

namespace audio_core {

// ============================================================================
// PipeWireBackend: Native Linux Audio Server Integration via pw_filter
// Exposes virtual input sinks for each mixer track (so external apps can route in)
// and output sinks for master/buses (to DAC or network).
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

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace audio_core
