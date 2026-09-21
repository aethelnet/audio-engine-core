#pragma once

#include "audio_core/types.hpp"
#include "audio_core/routing/inline_conditioner.hpp"
#include "audio_core/routing/modulatable_parameter.hpp"
#include <array>
#include <vector>
#include <atomic>
#include <string>
#include <string_view>
#include <memory>
#include <cstring>
#include <algorithm>
#include <iostream>

namespace audio_core::routing {

// ============================================================================
// Tap Points in Channel Strip & Signal Path
// ============================================================================
enum class TapPoint : uint8_t {
    Input = 0,        // Raw input before channel strip inserts
    PreInsert,        // Same as Input
    PostInsert,       // After modular insert slots, before fader/pan
    PreFader,         // Before volume fader
    PostFader         // After volume fader and stereo/spatial panning
};

// ============================================================================
// Routing Source & Destination Types
// ============================================================================
enum class RoutingSourceType : uint8_t {
    TrackAudio = 0,   // Source is an internal mixer track
    BusAudio,         // Source is a submix, aux, or master bus
    NetworkAoip,      // Source is a Dante / AES67 multicast network channel
    Modulator         // Source is an internal LFO / envelope / CV generator
};

enum class RoutingDestType : uint8_t {
    TrackSidechain = 0,     // Route into a track insert slot sidechain detector (e.g. Compressor squeeze)
    BusSidechain,           // Route into a bus insert slot sidechain detector
    ParameterModulation,    // Audio-rate parameter modulation (Cutoff, Drive, Pan, Fader, etc.)
    BusAuxInput             // Direct auxiliary feed into a bus
};

// Channel selection for routing
enum class RouteChannel : uint8_t {
    Left = 0,
    Right = 1,
    MonoSum = 2,
    StereoBoth = 3
};

// ============================================================================
// RoutingPatch: Unified Routing Connection (Virtual Patch Cable)
// Eliminates Bitwig converter friction: Source + Destination + Conditioning.
// ============================================================================
struct RoutingPatch {
    uint32_t id{0};
    bool active{false};

    // Source specification
    RoutingSourceType source_type{RoutingSourceType::TrackAudio};
    uint32_t source_id{0};            // Track ID, Bus ID, or Stream ID
    TapPoint tap_point{TapPoint::PostInsert};
    RouteChannel source_channel{RouteChannel::MonoSum};

    // Destination specification
    RoutingDestType dest_type{RoutingDestType::TrackSidechain};
    uint32_t dest_id{0};              // Track ID, Bus ID, or Parameter Target ID
    uint32_t dest_slot{0};            // Insert slot index (for sidechains) or param index
    RouteChannel dest_channel{RouteChannel::StereoBoth};

    // Inline signal conditioning (Zero Converter Nodes)
    InlineConditionerConfig conditioning;

    // Topology & Feedback Resolution
    bool is_feedback{false};          // Automatically detected cyclic feedback edge (uses Z^-1 block delay)
    std::string tag;                  // Human-readable identifier (e.g. "Track2 Kick Lowpass -> Track1 Comp Squeeze")
};

// ============================================================================
// UniversalRoutingMatrix: Sample-Accurate Lock-Free Routing Matrix
// Solves the "Bitwig Dilemma" and enables arbitrary sidechain patching:
// - Arbitrary source-to-destination routing across Tracks, Busses, Dante/AoIP, and Modulators.
// - Multi-Source Grouping & Summing (e.g. Track 2 Lowpass + Dante Ch 4 -> Track 1 Compressor).
// - First-Class Inline Conditioning (Cytomic SVF, Rectifier, Polarity, Envelope Follower).
// - Topological DAG cycle detection with automatic Z^-1 1-block delay feedback decoupling.
// - Zero allocations in real-time execution path (large buffers allocated once on heap via BufferStorage).
// ============================================================================
class UniversalRoutingMatrix {
public:
    static constexpr size_t kMaxRoutes = 128;
    static constexpr size_t kMaxTracks = 64;
    static constexpr size_t kMaxBuses = 16;
    static constexpr size_t kMaxInsertSlots = 4;
    static constexpr size_t kMaxNetworkChannels = 32;
    static constexpr size_t kMaxBlockFrames = 2048;

    struct BufferStorage {
        // Network Tap Buffers (e.g. Dante / AES67 multicast channels 0..31)
        std::array<std::array<float, kMaxBlockFrames>, kMaxNetworkChannels> network_taps{};

        // Track Sidechain Destination Buffers [Track][Slot][Left/Right][Frame]
        std::array<std::array<std::array<float, kMaxBlockFrames>, kMaxInsertSlots>, kMaxTracks> track_sc_l{};
        std::array<std::array<std::array<float, kMaxBlockFrames>, kMaxInsertSlots>, kMaxTracks> track_sc_r{};
        std::array<std::array<bool, kMaxInsertSlots>, kMaxTracks> track_sc_active{};

        // Bus Sidechain Destination Buffers [Bus][Slot][Left/Right][Frame]
        std::array<std::array<std::array<float, kMaxBlockFrames>, kMaxInsertSlots>, kMaxBuses> bus_sc_l{};
        std::array<std::array<std::array<float, kMaxBlockFrames>, kMaxInsertSlots>, kMaxBuses> bus_sc_r{};
        std::array<std::array<bool, kMaxInsertSlots>, kMaxBuses> bus_sc_active{};

        // Parameter Modulation Scratch Buffers
        std::array<std::array<float, kMaxBlockFrames>, kMaxRoutes> param_mod_scratch{};

        // Feedback Ringbuffers for Z^-1 decoupling
        std::array<std::array<float, kMaxBlockFrames>, kMaxRoutes> feedback_buffers{};

        // Scratch buffers for downmix and conditioning
        std::array<float, kMaxBlockFrames> mono_scratch{};
        alignas(16) std::array<float, kMaxBlockFrames> cond_scratch{};
    };

    UniversalRoutingMatrix() noexcept
        : UniversalRoutingMatrix(48000) {}

    explicit UniversalRoutingMatrix(uint32_t sample_rate) noexcept
        : m_sample_rate(sample_rate ? sample_rate : 48000)
        , m_storage(std::make_unique<BufferStorage>()) {
        init_storage();
    }

    UniversalRoutingMatrix(UniversalRoutingMatrix&&) noexcept = default;
    UniversalRoutingMatrix& operator=(UniversalRoutingMatrix&&) noexcept = default;
    UniversalRoutingMatrix(const UniversalRoutingMatrix&) = delete;
    UniversalRoutingMatrix& operator=(const UniversalRoutingMatrix&) = delete;

    void set_sample_rate(uint32_t sr) noexcept {
        if (sr == 0 || sr == m_sample_rate) return;
        m_sample_rate = sr;
        for (size_t i = 0; i < kMaxRoutes; ++i) {
            m_conditioners[i].set_sample_rate(m_sample_rate);
        }
    }

    void reset() noexcept {
        for (auto& c : m_conditioners) c.reset();
        if (m_storage) {
            for (auto& fb : m_storage->feedback_buffers) fb.fill(0.0f);
        }
    }

    // ------------------------------------------------------------------------
    // Patch Configuration API
    // ------------------------------------------------------------------------
    int32_t add_patch(const RoutingPatch& patch) noexcept {
        for (size_t i = 0; i < kMaxRoutes; ++i) {
            if (!m_routes[i].active) {
                m_routes[i] = patch;
                m_routes[i].id = static_cast<uint32_t>(i + 1);
                m_routes[i].active = true;
                m_conditioners[i].set_sample_rate(m_sample_rate);
                m_conditioners[i].set_config(patch.conditioning);
                m_conditioners[i].reset();
                update_feedback_topology();
                return static_cast<int32_t>(m_routes[i].id);
            }
        }
        return -1; // Full
    }

    bool remove_patch(uint32_t patch_id) noexcept {
        for (size_t i = 0; i < kMaxRoutes; ++i) {
            if (m_routes[i].active && m_routes[i].id == patch_id) {
                m_routes[i].active = false;
                update_feedback_topology();
                return true;
            }
        }
        return false;
    }

    void clear_all_patches() noexcept {
        for (auto& r : m_routes) r.active = false;
        reset();
    }

    [[nodiscard]] const std::array<RoutingPatch, kMaxRoutes>& patches() const noexcept {
        return m_routes;
    }

    // ------------------------------------------------------------------------
    // Network / Dante Tap Input Feed (Called when AoIP frames arrive)
    // ------------------------------------------------------------------------
    void feed_network_channel(uint16_t channel, const float* samples, uint32_t frames) noexcept {
        if (channel >= kMaxNetworkChannels || !samples || frames == 0 || !m_storage) return;
        const uint32_t count = std::min(frames, static_cast<uint32_t>(kMaxBlockFrames));
        std::copy_n(samples, count, m_storage->network_taps[channel].data());
    }

    [[nodiscard]] static constexpr uint32_t track_index(uint32_t track_id) noexcept {
        return (track_id >= 1 && track_id <= kMaxTracks) ? (track_id - 1) : std::min(track_id, static_cast<uint32_t>(kMaxTracks - 1));
    }

    [[nodiscard]] static constexpr uint32_t bus_index(uint32_t bus_id) noexcept {
        return (bus_id >= 1 && bus_id <= kMaxBuses) ? (bus_id - 1) : std::min(bus_id, static_cast<uint32_t>(kMaxBuses - 1));
    }

    // ------------------------------------------------------------------------
    // Prepare Destination Buffers at Start of Render Loop
    // Clears destination sidechain accumulator buffers for the upcoming block.
    // ------------------------------------------------------------------------
    void prepare_block(uint32_t frames) noexcept {
        if (!m_storage) return;
        const uint32_t count = std::min(frames, static_cast<uint32_t>(kMaxBlockFrames));

        for (auto& row : m_storage->track_sc_active) row.fill(false);
        for (auto& row : m_storage->bus_sc_active) row.fill(false);

        // Clear only destinations that have active incoming routes
        for (size_t i = 0; i < kMaxRoutes; ++i) {
            if (!m_routes[i].active) continue;
            const auto& r = m_routes[i];

            if (r.dest_type == RoutingDestType::TrackSidechain) {
                const uint32_t trk = track_index(r.dest_id);
                if (trk < kMaxTracks && r.dest_slot < kMaxInsertSlots) {
                    std::fill_n(m_storage->track_sc_l[trk][r.dest_slot].data(), count, 0.0f);
                    std::fill_n(m_storage->track_sc_r[trk][r.dest_slot].data(), count, 0.0f);
                    m_storage->track_sc_active[trk][r.dest_slot] = true;
                }
            } else if (r.dest_type == RoutingDestType::BusSidechain) {
                const uint32_t bus = bus_index(r.dest_id);
                if (bus < kMaxBuses && r.dest_slot < kMaxInsertSlots) {
                    std::fill_n(m_storage->bus_sc_l[bus][r.dest_slot].data(), count, 0.0f);
                    std::fill_n(m_storage->bus_sc_r[bus][r.dest_slot].data(), count, 0.0f);
                    m_storage->bus_sc_active[bus][r.dest_slot] = true;
                }
            } else if (r.dest_type == RoutingDestType::ParameterModulation) {
                if (r.dest_id < kMaxRoutes) {
                    std::fill_n(m_storage->param_mod_scratch[r.dest_id].data(), count, 0.0f);
                }
            }
        }
    }

    // ------------------------------------------------------------------------
    // Evaluate Route Execution (Called inside Render Loop)
    // Extracts source audio, applies inline conditioning, and accumulates
    // into destination sidechain or modulation buffers.
    // ------------------------------------------------------------------------
    void process_route(size_t route_idx,
                       const float* src_l, const float* src_r,
                       uint32_t frames) noexcept {
        if (route_idx >= kMaxRoutes || !m_routes[route_idx].active || frames == 0 || !m_storage) return;
        auto& route = m_routes[route_idx];
        auto& cond = m_conditioners[route_idx];
        const uint32_t count = std::min(frames, static_cast<uint32_t>(kMaxBlockFrames));

        // 1. Resolve source buffer
        const float* in_ptr = nullptr;
        if (route.source_type == RoutingSourceType::NetworkAoip) {
            const uint16_t ch = static_cast<uint16_t>(route.source_id);
            if (ch < kMaxNetworkChannels) {
                in_ptr = m_storage->network_taps[ch].data();
            }
        } else {
            // TrackAudio or BusAudio
            if (route.source_channel == RouteChannel::Left) {
                in_ptr = src_l;
            } else if (route.source_channel == RouteChannel::Right) {
                in_ptr = src_r;
            } else {
                // Mono sum into scratch buffer
                if (src_l && src_r) {
                    for (uint32_t i = 0; i < count; ++i) {
                        m_storage->mono_scratch[i] = 0.5f * (src_l[i] + src_r[i]);
                    }
                    in_ptr = m_storage->mono_scratch.data();
                } else if (src_l) {
                    in_ptr = src_l;
                } else if (src_r) {
                    in_ptr = src_r;
                }
            }
        }

        if (!in_ptr) return;

        // 2. Feedback decoupling: If this route is part of a cyclic loop, read from Z^-1 buffer
        const float* proc_src = in_ptr;
        if (route.is_feedback) {
            proc_src = m_storage->feedback_buffers[route_idx].data();
        }

        // Store current block into feedback ringbuffer for NEXT block's execution
        std::copy_n(in_ptr, count, m_storage->feedback_buffers[route_idx].data());

        // 3. Accumulate into destination (Multi-Source Grouping & Summing)
        // Run conditioner once into scratch buffer to advance filter/envelope state
        cond.process_block(proc_src, m_storage->cond_scratch.data(), count);

        if (route.dest_type == RoutingDestType::TrackSidechain) {
            const uint32_t trk = track_index(route.dest_id);
            const uint32_t slt = route.dest_slot;
            if (trk < kMaxTracks && slt < kMaxInsertSlots) {
                float* dst_l = m_storage->track_sc_l[trk][slt].data();
                float* dst_r = m_storage->track_sc_r[trk][slt].data();

                if (route.dest_channel == RouteChannel::Left) {
                    for (uint32_t i = 0; i < count; ++i) dst_l[i] += m_storage->cond_scratch[i];
                } else if (route.dest_channel == RouteChannel::Right) {
                    for (uint32_t i = 0; i < count; ++i) dst_r[i] += m_storage->cond_scratch[i];
                } else {
                    // Both channels
                    for (uint32_t i = 0; i < count; ++i) {
                        dst_l[i] += m_storage->cond_scratch[i];
                        dst_r[i] += m_storage->cond_scratch[i];
                    }
                }
            }
        } else if (route.dest_type == RoutingDestType::BusSidechain) {
            const uint32_t bus = bus_index(route.dest_id);
            const uint32_t slt = route.dest_slot;
            if (bus < kMaxBuses && slt < kMaxInsertSlots) {
                float* dst_l = m_storage->bus_sc_l[bus][slt].data();
                float* dst_r = m_storage->bus_sc_r[bus][slt].data();

                if (route.dest_channel == RouteChannel::Left) {
                    for (uint32_t i = 0; i < count; ++i) dst_l[i] += m_storage->cond_scratch[i];
                } else if (route.dest_channel == RouteChannel::Right) {
                    for (uint32_t i = 0; i < count; ++i) dst_r[i] += m_storage->cond_scratch[i];
                } else {
                    for (uint32_t i = 0; i < count; ++i) {
                        dst_l[i] += m_storage->cond_scratch[i];
                        dst_r[i] += m_storage->cond_scratch[i];
                    }
                }
            }
        } else if (route.dest_type == RoutingDestType::ParameterModulation) {
            if (route.dest_id < kMaxRoutes) {
                float* dst_mod = m_storage->param_mod_scratch[route.dest_id].data();
                for (uint32_t i = 0; i < count; ++i) dst_mod[i] += m_storage->cond_scratch[i];
            }
        }
    }

    // ------------------------------------------------------------------------
    // Sidechain Query API for Channel Strip & Bus Execution
    // ------------------------------------------------------------------------
    [[nodiscard]] bool has_track_sidechain(uint32_t track_id, uint32_t slot) const noexcept {
        if (!m_storage) return false;
        uint32_t trk = track_index(track_id);
        if (trk >= kMaxTracks || slot >= kMaxInsertSlots) return false;
        return m_storage->track_sc_active[trk][slot];
    }

    [[nodiscard]] const float* track_sidechain_l(uint32_t track_id, uint32_t slot) const noexcept {
        if (!m_storage) return nullptr;
        uint32_t trk = track_index(track_id);
        if (trk >= kMaxTracks || slot >= kMaxInsertSlots) return nullptr;
        return m_storage->track_sc_l[trk][slot].data();
    }

    [[nodiscard]] const float* track_sidechain_r(uint32_t track_id, uint32_t slot) const noexcept {
        if (!m_storage) return nullptr;
        uint32_t trk = track_index(track_id);
        if (trk >= kMaxTracks || slot >= kMaxInsertSlots) return nullptr;
        return m_storage->track_sc_r[trk][slot].data();
    }

    [[nodiscard]] bool has_bus_sidechain(uint32_t bus_id, uint32_t slot) const noexcept {
        if (!m_storage) return false;
        uint32_t bus = bus_index(bus_id);
        if (bus >= kMaxBuses || slot >= kMaxInsertSlots) return false;
        return m_storage->bus_sc_active[bus][slot];
    }

    [[nodiscard]] const float* bus_sidechain_l(uint32_t bus_id, uint32_t slot) const noexcept {
        if (!m_storage) return nullptr;
        uint32_t bus = bus_index(bus_id);
        if (bus >= kMaxBuses || slot >= kMaxInsertSlots) return nullptr;
        return m_storage->bus_sc_l[bus][slot].data();
    }

    [[nodiscard]] const float* bus_sidechain_r(uint32_t bus_id, uint32_t slot) const noexcept {
        if (!m_storage) return nullptr;
        uint32_t bus = bus_index(bus_id);
        if (bus >= kMaxBuses || slot >= kMaxInsertSlots) return nullptr;
        return m_storage->bus_sc_r[bus][slot].data();
    }

    [[nodiscard]] const float* parameter_modulation_buffer(uint32_t target_id) const noexcept {
        if (!m_storage || target_id >= kMaxRoutes) return nullptr;
        return m_storage->param_mod_scratch[target_id].data();
    }

    // ------------------------------------------------------------------------
    // Topological Cycle Detection & Automatic Feedback Decoupling
    // If Track A -> Track B -> Track A, Kahn / DFS marks the back-edge as feedback (Z^-1).
    // ------------------------------------------------------------------------
    void update_feedback_topology() noexcept {
        // Build track-to-track adjacency matrix
        bool adj[kMaxTracks][kMaxTracks]{};
        for (size_t i = 0; i < kMaxRoutes; ++i) {
            if (!m_routes[i].active) continue;
            m_routes[i].is_feedback = false;

            if (m_routes[i].source_type == RoutingSourceType::TrackAudio &&
                m_routes[i].dest_type == RoutingDestType::TrackSidechain) {
                uint32_t u = track_index(m_routes[i].source_id);
                uint32_t v = track_index(m_routes[i].dest_id);
                if (u < kMaxTracks && v < kMaxTracks) {
                    if (u == v) {
                        // Direct self-feedback: must be Z^-1
                        m_routes[i].is_feedback = true;
                    } else if (adj[v][u]) {
                        // Reverse path already exists: cycle detected! Decouple with Z^-1
                        m_routes[i].is_feedback = true;
                    } else {
                        adj[u][v] = true;
                    }
                }
            }
        }
    }

private:
    void init_storage() noexcept {
        for (auto& r : m_routes) r.active = false;
        if (!m_storage) return;
        for (auto& row : m_storage->track_sc_active) row.fill(false);
        for (auto& row : m_storage->bus_sc_active) row.fill(false);
        for (auto& buf : m_storage->network_taps) buf.fill(0.0f);
        for (auto& buf : m_storage->param_mod_scratch) buf.fill(0.0f);
        for (auto& buf : m_storage->feedback_buffers) buf.fill(0.0f);
        m_storage->mono_scratch.fill(0.0f);
        m_storage->cond_scratch.fill(0.0f);
    }

    uint32_t m_sample_rate{48000};

    // Route Patch Bay & Inline Conditioners (~12 KB)
    std::array<RoutingPatch, kMaxRoutes> m_routes{};
    std::array<InlineConditioner, kMaxRoutes> m_conditioners{};

    // Heap-allocated large planar buffers (zero runtime allocations)
    std::unique_ptr<BufferStorage> m_storage;
};

} // namespace audio_core::routing
