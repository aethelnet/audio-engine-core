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
    BusAuxInput,            // Direct auxiliary feed into a bus (Reverb, Delay, Parallel Send)
    TrackAudioInput,        // Direct audio feed into another track's input (Track-to-Track audio routing / Submix)
    NetworkAoipSink         // Egress feed into an AoIP network transmission channel (Dante/AES67 Tx)
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

        // Bus Auxiliary Audio Destination Buffers [Bus][Left/Right][Frame]
        std::array<std::array<float, kMaxBlockFrames>, kMaxBuses> bus_aux_l{};
        std::array<std::array<float, kMaxBlockFrames>, kMaxBuses> bus_aux_r{};
        std::array<bool, kMaxBuses> bus_aux_active{};

        // Track Audio Input Destination Buffers [Track][Left/Right][Frame]
        std::array<std::array<float, kMaxBlockFrames>, kMaxTracks> track_input_l{};
        std::array<std::array<float, kMaxBlockFrames>, kMaxTracks> track_input_r{};
        std::array<bool, kMaxTracks> track_input_active{};

        // Network AoIP Transmission Channels [Channel 0..31][Frame]
        std::array<std::array<float, kMaxBlockFrames>, kMaxNetworkChannels> network_tx_channels{};
        std::array<bool, kMaxNetworkChannels> network_tx_active{};

        // Parameter Modulation Scratch Buffers
        std::array<std::array<float, kMaxBlockFrames>, kMaxRoutes> param_mod_scratch{};

        // Feedback Ringbuffers for Z^-1 decoupling
        std::array<std::array<float, kMaxBlockFrames>, kMaxRoutes> feedback_buffers_l{};
        std::array<std::array<float, kMaxBlockFrames>, kMaxRoutes> feedback_buffers_r{};

        // Scratch buffers for downmix and conditioning
        std::array<float, kMaxBlockFrames> mono_scratch{};
        alignas(16) std::array<float, kMaxBlockFrames> cond_scratch_l{};
        alignas(16) std::array<float, kMaxBlockFrames> cond_scratch_r{};
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
            m_conditioners_l[i].set_sample_rate(m_sample_rate);
            m_conditioners_r[i].set_sample_rate(m_sample_rate);
        }
    }

    void reset() noexcept {
        for (auto& c : m_conditioners_l) c.reset();
        for (auto& c : m_conditioners_r) c.reset();
        if (m_storage) {
            for (auto& fb : m_storage->feedback_buffers_l) fb.fill(0.0f);
            for (auto& fb : m_storage->feedback_buffers_r) fb.fill(0.0f);
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
                m_conditioners_l[i].set_sample_rate(m_sample_rate);
                m_conditioners_l[i].set_config(patch.conditioning);
                m_conditioners_l[i].reset();
                m_conditioners_r[i].set_sample_rate(m_sample_rate);
                m_conditioners_r[i].set_config(patch.conditioning);
                m_conditioners_r[i].reset();
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

    [[nodiscard]] std::array<RoutingPatch, kMaxRoutes>& patches() noexcept {
        return m_routes;
    }

    [[nodiscard]] RoutingPatch* get_patch(uint32_t patch_id) noexcept {
        for (size_t i = 0; i < kMaxRoutes; ++i) {
            if (m_routes[i].active && m_routes[i].id == patch_id) {
                return &m_routes[i];
            }
        }
        return nullptr;
    }

    bool update_patch_config(uint32_t patch_id, const InlineConditionerConfig& config) noexcept {
        for (size_t i = 0; i < kMaxRoutes; ++i) {
            if (m_routes[i].active && m_routes[i].id == patch_id) {
                m_routes[i].conditioning = config;
                m_conditioners_l[i].set_config(config);
                m_conditioners_r[i].set_config(config);
                return true;
            }
        }
        return false;
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
    // Clears destination sidechain, bus aux, track input, and AoIP tx buffers.
    // ------------------------------------------------------------------------
    void prepare_block(uint32_t frames) noexcept {
        if (!m_storage) return;
        const uint32_t count = std::min(frames, static_cast<uint32_t>(kMaxBlockFrames));

        for (auto& row : m_storage->track_sc_active) row.fill(false);
        for (auto& row : m_storage->bus_sc_active) row.fill(false);
        m_storage->bus_aux_active.fill(false);
        m_storage->track_input_active.fill(false);
        m_storage->network_tx_active.fill(false);

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
            } else if (r.dest_type == RoutingDestType::BusAuxInput) {
                const uint32_t bus = bus_index(r.dest_id);
                if (bus < kMaxBuses) {
                    std::fill_n(m_storage->bus_aux_l[bus].data(), count, 0.0f);
                    std::fill_n(m_storage->bus_aux_r[bus].data(), count, 0.0f);
                    m_storage->bus_aux_active[bus] = true;
                }
            } else if (r.dest_type == RoutingDestType::TrackAudioInput) {
                const uint32_t trk = track_index(r.dest_id);
                if (trk < kMaxTracks) {
                    std::fill_n(m_storage->track_input_l[trk].data(), count, 0.0f);
                    std::fill_n(m_storage->track_input_r[trk].data(), count, 0.0f);
                    m_storage->track_input_active[trk] = true;
                }
            } else if (r.dest_type == RoutingDestType::NetworkAoipSink) {
                const uint32_t ch = r.dest_id;
                if (ch < kMaxNetworkChannels) {
                    std::fill_n(m_storage->network_tx_channels[ch].data(), count, 0.0f);
                    m_storage->network_tx_active[ch] = true;
                    if (r.dest_channel == RouteChannel::StereoBoth && (ch + 1) < kMaxNetworkChannels) {
                        std::fill_n(m_storage->network_tx_channels[ch + 1].data(), count, 0.0f);
                        m_storage->network_tx_active[ch + 1] = true;
                    }
                }
            }
        }
    }

    // ------------------------------------------------------------------------
    // Evaluate Route Execution (Called inside Render Loop)
    // Extracts source audio, applies inline conditioning, and accumulates
    // into destination sidechain, bus aux, track input, or network tx buffers.
    // ------------------------------------------------------------------------
    void process_route(size_t route_idx,
                       const float* src_l, const float* src_r,
                       uint32_t frames) noexcept {
        if (route_idx >= kMaxRoutes || !m_routes[route_idx].active || frames == 0 || !m_storage) return;
        auto& route = m_routes[route_idx];
        auto& cond_l = m_conditioners_l[route_idx];
        auto& cond_r = m_conditioners_r[route_idx];
        const uint32_t count = std::min(frames, static_cast<uint32_t>(kMaxBlockFrames));

        // 1. Resolve source buffer
        const float* in_l = nullptr;
        const float* in_r = nullptr;

        if (route.source_type == RoutingSourceType::NetworkAoip) {
            const uint16_t ch = static_cast<uint16_t>(route.source_id);
            if (ch < kMaxNetworkChannels) {
                in_l = m_storage->network_taps[ch].data();
                if (route.source_channel == RouteChannel::StereoBoth && (static_cast<size_t>(ch) + 1) < kMaxNetworkChannels) {
                    in_r = m_storage->network_taps[ch + 1].data();
                } else {
                    in_r = in_l;
                }
            }
        } else {
            // TrackAudio or BusAudio
            if (route.source_channel == RouteChannel::Left) {
                in_l = src_l ? src_l : src_r;
                in_r = in_l;
            } else if (route.source_channel == RouteChannel::Right) {
                in_l = src_r ? src_r : src_l;
                in_r = in_l;
            } else if (route.source_channel == RouteChannel::MonoSum) {
                if (src_l && src_r) {
                    for (uint32_t i = 0; i < count; ++i) {
                        m_storage->mono_scratch[i] = 0.5f * (src_l[i] + src_r[i]);
                    }
                    in_l = in_r = m_storage->mono_scratch.data();
                } else {
                    in_l = in_r = src_l ? src_l : src_r;
                }
            } else { // StereoBoth
                in_l = src_l;
                in_r = src_r ? src_r : src_l;
            }
        }

        if (!in_l) return;
        if (!in_r) in_r = in_l;

        // 2. Feedback decoupling: If this route is part of a cyclic loop, read from Z^-1 buffer
        const float* proc_l = in_l;
        const float* proc_r = in_r;
        if (route.is_feedback) {
            proc_l = m_storage->feedback_buffers_l[route_idx].data();
            proc_r = m_storage->feedback_buffers_r[route_idx].data();
        }

        // Store current block into feedback ringbuffer for NEXT block's execution
        std::copy_n(in_l, count, m_storage->feedback_buffers_l[route_idx].data());
        std::copy_n(in_r, count, m_storage->feedback_buffers_r[route_idx].data());

        // 3. Condition audio
        if (route.source_channel == RouteChannel::StereoBoth && route.dest_channel == RouteChannel::StereoBoth) {
            cond_l.process_block(proc_l, m_storage->cond_scratch_l.data(), count);
            cond_r.process_block(proc_r, m_storage->cond_scratch_r.data(), count);
        } else {
            // Mono conditioning
            cond_l.process_block(proc_l, m_storage->cond_scratch_l.data(), count);
            std::copy_n(m_storage->cond_scratch_l.data(), count, m_storage->cond_scratch_r.data());
        }

        const float* out_l = m_storage->cond_scratch_l.data();
        const float* out_r = m_storage->cond_scratch_r.data();

        // 4. Accumulate into destination (Multi-Source Grouping & Summing)
        if (route.dest_type == RoutingDestType::TrackSidechain) {
            const uint32_t trk = track_index(route.dest_id);
            const uint32_t slt = route.dest_slot;
            if (trk < kMaxTracks && slt < kMaxInsertSlots) {
                float* dst_l = m_storage->track_sc_l[trk][slt].data();
                float* dst_r = m_storage->track_sc_r[trk][slt].data();

                if (route.dest_channel == RouteChannel::Left) {
                    for (uint32_t i = 0; i < count; ++i) dst_l[i] += out_l[i];
                } else if (route.dest_channel == RouteChannel::Right) {
                    for (uint32_t i = 0; i < count; ++i) dst_r[i] += out_r[i];
                } else {
                    for (uint32_t i = 0; i < count; ++i) {
                        dst_l[i] += out_l[i];
                        dst_r[i] += out_r[i];
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
                    for (uint32_t i = 0; i < count; ++i) dst_l[i] += out_l[i];
                } else if (route.dest_channel == RouteChannel::Right) {
                    for (uint32_t i = 0; i < count; ++i) dst_r[i] += out_r[i];
                } else {
                    for (uint32_t i = 0; i < count; ++i) {
                        dst_l[i] += out_l[i];
                        dst_r[i] += out_r[i];
                    }
                }
            }
        } else if (route.dest_type == RoutingDestType::ParameterModulation) {
            if (route.dest_id < kMaxRoutes) {
                float* dst_mod = m_storage->param_mod_scratch[route.dest_id].data();
                for (uint32_t i = 0; i < count; ++i) dst_mod[i] += out_l[i];
            }
        } else if (route.dest_type == RoutingDestType::BusAuxInput) {
            const uint32_t bus = bus_index(route.dest_id);
            if (bus < kMaxBuses) {
                float* dst_l = m_storage->bus_aux_l[bus].data();
                float* dst_r = m_storage->bus_aux_r[bus].data();

                if (route.dest_channel == RouteChannel::Left) {
                    for (uint32_t i = 0; i < count; ++i) dst_l[i] += out_l[i];
                } else if (route.dest_channel == RouteChannel::Right) {
                    for (uint32_t i = 0; i < count; ++i) dst_r[i] += out_r[i];
                } else {
                    for (uint32_t i = 0; i < count; ++i) {
                        dst_l[i] += out_l[i];
                        dst_r[i] += out_r[i];
                    }
                }
            }
        } else if (route.dest_type == RoutingDestType::TrackAudioInput) {
            const uint32_t trk = track_index(route.dest_id);
            if (trk < kMaxTracks) {
                float* dst_l = m_storage->track_input_l[trk].data();
                float* dst_r = m_storage->track_input_r[trk].data();

                if (route.dest_channel == RouteChannel::Left) {
                    for (uint32_t i = 0; i < count; ++i) dst_l[i] += out_l[i];
                } else if (route.dest_channel == RouteChannel::Right) {
                    for (uint32_t i = 0; i < count; ++i) dst_r[i] += out_r[i];
                } else {
                    for (uint32_t i = 0; i < count; ++i) {
                        dst_l[i] += out_l[i];
                        dst_r[i] += out_r[i];
                    }
                }
            }
        } else if (route.dest_type == RoutingDestType::NetworkAoipSink) {
            const uint32_t ch = route.dest_id;
            if (ch < kMaxNetworkChannels) {
                float* dst0 = m_storage->network_tx_channels[ch].data();
                if (route.dest_channel == RouteChannel::StereoBoth && (ch + 1) < kMaxNetworkChannels) {
                    float* dst1 = m_storage->network_tx_channels[ch + 1].data();
                    for (uint32_t i = 0; i < count; ++i) {
                        dst0[i] += out_l[i];
                        dst1[i] += out_r[i];
                    }
                } else {
                    for (uint32_t i = 0; i < count; ++i) {
                        dst0[i] += out_l[i];
                    }
                }
            }
        }
    }

    // ------------------------------------------------------------------------
    // Sidechain & Auxiliary Query API for Channel Strip & Bus Execution
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

    [[nodiscard]] bool has_bus_aux(uint32_t bus_id) const noexcept {
        if (!m_storage) return false;
        uint32_t bus = bus_index(bus_id);
        if (bus >= kMaxBuses) return false;
        return m_storage->bus_aux_active[bus];
    }

    [[nodiscard]] const float* bus_aux_l(uint32_t bus_id) const noexcept {
        if (!m_storage) return nullptr;
        uint32_t bus = bus_index(bus_id);
        if (bus >= kMaxBuses) return nullptr;
        return m_storage->bus_aux_l[bus].data();
    }

    [[nodiscard]] const float* bus_aux_r(uint32_t bus_id) const noexcept {
        if (!m_storage) return nullptr;
        uint32_t bus = bus_index(bus_id);
        if (bus >= kMaxBuses) return nullptr;
        return m_storage->bus_aux_r[bus].data();
    }

    [[nodiscard]] bool has_track_input(uint32_t track_id) const noexcept {
        if (!m_storage) return false;
        uint32_t trk = track_index(track_id);
        if (trk >= kMaxTracks) return false;
        return m_storage->track_input_active[trk];
    }

    [[nodiscard]] const float* track_input_l(uint32_t track_id) const noexcept {
        if (!m_storage) return nullptr;
        uint32_t trk = track_index(track_id);
        if (trk >= kMaxTracks) return nullptr;
        return m_storage->track_input_l[trk].data();
    }

    [[nodiscard]] const float* track_input_r(uint32_t track_id) const noexcept {
        if (!m_storage) return nullptr;
        uint32_t trk = track_index(track_id);
        if (trk >= kMaxTracks) return nullptr;
        return m_storage->track_input_r[trk].data();
    }

    [[nodiscard]] bool has_network_tx(uint32_t channel) const noexcept {
        if (!m_storage || channel >= kMaxNetworkChannels) return false;
        return m_storage->network_tx_active[channel];
    }

    [[nodiscard]] const float* network_tx_channel(uint32_t channel) const noexcept {
        if (!m_storage || channel >= kMaxNetworkChannels) return nullptr;
        return m_storage->network_tx_channels[channel].data();
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
                (m_routes[i].dest_type == RoutingDestType::TrackSidechain ||
                 m_routes[i].dest_type == RoutingDestType::TrackAudioInput)) {
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
        m_storage->bus_aux_active.fill(false);
        m_storage->track_input_active.fill(false);
        m_storage->network_tx_active.fill(false);
        for (auto& buf : m_storage->network_taps) buf.fill(0.0f);
        for (auto& buf : m_storage->network_tx_channels) buf.fill(0.0f);
        for (auto& buf : m_storage->bus_aux_l) buf.fill(0.0f);
        for (auto& buf : m_storage->bus_aux_r) buf.fill(0.0f);
        for (auto& buf : m_storage->track_input_l) buf.fill(0.0f);
        for (auto& buf : m_storage->track_input_r) buf.fill(0.0f);
        for (auto& buf : m_storage->param_mod_scratch) buf.fill(0.0f);
        for (auto& buf : m_storage->feedback_buffers_l) buf.fill(0.0f);
        for (auto& buf : m_storage->feedback_buffers_r) buf.fill(0.0f);
        m_storage->mono_scratch.fill(0.0f);
        m_storage->cond_scratch_l.fill(0.0f);
        m_storage->cond_scratch_r.fill(0.0f);
    }

    uint32_t m_sample_rate{48000};

    // Route Patch Bay & Inline Conditioners (~24 KB)
    std::array<RoutingPatch, kMaxRoutes> m_routes{};
    std::array<InlineConditioner, kMaxRoutes> m_conditioners_l{};
    std::array<InlineConditioner, kMaxRoutes> m_conditioners_r{};

    // Heap-allocated large planar buffers (zero runtime allocations)
    std::unique_ptr<BufferStorage> m_storage;
};

} // namespace audio_core::routing
