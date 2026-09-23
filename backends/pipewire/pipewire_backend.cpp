#include "backends/pipewire/pipewire_backend.hpp"
#include "audio_core/network/aoip_receiver.hpp"

#include <pipewire/pipewire.h>
#include <pipewire/filter.h>
#include <pipewire/core.h>
#include <pipewire/node.h>
#include <pipewire/port.h>
#include <pipewire/link.h>
#include <pipewire/keys.h>
#include <pipewire/properties.h>
#include <pipewire/proxy.h>
#include <spa/param/audio/dsp-utils.h>
#include <spa/param/audio/format-utils.h>
#include <spa/utils/dict.h>
#include <spa/utils/hook.h>

#include <iostream>
#include <vector>
#include <array>
#include <cstring>
#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>
#include <unordered_map>
#include <sstream>
#include <optional>
#include <algorithm>

namespace audio_core {

struct PipeWireBackend::Impl {
    MixerGraph& mixer;
    std::string node_name;
    uint32_t sample_rate{48000};
    std::atomic<bool> running{false};
    std::atomic<network::AoipReceiver*> aoip_receiver{nullptr};

    struct pw_thread_loop* loop{nullptr};
    struct pw_filter* filter{nullptr};
    struct pw_core* core{nullptr};
    struct pw_registry* registry{nullptr};
    struct spa_hook registry_listener{};

    // Output ports (Master Out L / R)
    void* port_out_l{nullptr};
    void* port_out_r{nullptr};

    // Virtual Input Sink ports per track (Track 1..32)
    struct TrackPortPair {
        void* port_in_l{nullptr};
        void* port_in_r{nullptr};
    };
    std::array<TrackPortPair, MixerGraph::kMaxTracks> track_ports{};

    AudioBuffer master_buffer{2, 2048};

    // Native Node and Port tracking IDs for our own filter
    uint32_t our_node_id{0};
    uint32_t our_master_out_l_id{0};
    uint32_t our_master_out_r_id{0};
    std::array<uint32_t, MixerGraph::kMaxTracks> our_track_in_l_ids{};
    std::array<uint32_t, MixerGraph::kMaxTracks> our_track_in_r_ids{};

    // Native Graph Registry State (Zero-fork in-memory topology)
    struct PwNodeInfo {
        uint32_t id{0};
        std::string name;
        std::string description;
        std::string media_class;
        std::string app_name;
    };

    struct PwPortInfo {
        uint32_t id{0};
        uint32_t node_id{0};
        std::string name;
        std::string direction; // "in" or "out"
        std::string channel;   // "FL", "FR", "MONO", etc.
        std::string alias;
    };

    mutable std::mutex registry_mutex;
    std::unordered_map<uint32_t, PwNodeInfo> nodes;
    std::unordered_map<uint32_t, PwPortInfo> ports;

    // Active native link proxies
    std::unordered_map<uint32_t, std::vector<struct pw_proxy*>> track_links;
    std::vector<struct pw_proxy*> master_links;
    std::string active_master_sink_node;
    std::string active_master_sink_display;
    std::unordered_map<uint32_t, DiscoveredStreamPair> active_track_sources;

    // Discovered Streams Snapshot
    mutable std::mutex discovery_mutex;
    std::vector<DiscoveredStreamPair> discovered_sources;
    std::vector<DiscoveredStreamPair> discovered_sinks;
    std::thread discovery_thread;

    explicit Impl(MixerGraph& m) : mixer(m) {}

    static void on_process(void* userdata, struct spa_io_position* position) noexcept {
        auto* self = static_cast<Impl*>(userdata);
        if (!self || !self->running.load(std::memory_order_relaxed)) {
            return;
        }

        uint32_t n_samples = position ? position->clock.duration : 1024;
        if (n_samples == 0 || n_samples > 2048) {
            n_samples = 1024;
        }

        const uint32_t engine_max = self->mixer.buffer_frames();
        const uint32_t frames_to_process = std::min(n_samples, engine_max);

        // 1. Pull audio from Virtual Input Sinks (external apps patched into tracks)
        for (size_t i = 0; i < MixerGraph::kMaxTracks; ++i) {
            auto* track = self->mixer.get_track(static_cast<uint32_t>(i + 1));
            if (!track || !track->is_active()) {
                continue;
            }

            auto mode = track->input_mode();
            if (mode == TrackInputMode::InternalClip || mode == TrackInputMode::NetworkAoip) {
                continue;
            }

            const auto& p_pair = self->track_ports[i];
            if (p_pair.port_in_l && p_pair.port_in_r) {
                const auto* src_l = static_cast<const float*>(pw_filter_get_dsp_buffer(p_pair.port_in_l, n_samples));
                const auto* src_r = static_cast<const float*>(pw_filter_get_dsp_buffer(p_pair.port_in_r, n_samples));

                if (src_l || src_r) {
                    Sample* dst_l = track->buffer().view().channel(0);
                    Sample* dst_r = track->buffer().view().channel(1);
                    uint32_t track_max = std::min(frames_to_process, track->buffer().num_frames());

                    if (mode == TrackInputMode::PipeWireStream) {
                        if (src_l) std::memcpy(dst_l, src_l, track_max * sizeof(float));
                        else std::memset(dst_l, 0, track_max * sizeof(float));
                        if (src_r) std::memcpy(dst_r, src_r, track_max * sizeof(float));
                        else std::memset(dst_r, 0, track_max * sizeof(float));
                    } else if (mode == TrackInputMode::MergeAll) {
                        for (uint32_t f = 0; f < track_max; ++f) {
                            dst_l[f] = src_l ? src_l[f] : 0.0f;
                            dst_r[f] = src_r ? src_r[f] : 0.0f;
                        }
                    }
                }
            }
        }

        // 1b. Ingest AoIP network packets into mapped tracks
        auto* receiver = self->aoip_receiver.load(std::memory_order_relaxed);
        if (receiver) {
            self->mixer.ingest_aoip(*receiver, frames_to_process);
        }

        // 2. Execute Real-Time Channel Strips, Inserts, DAG Submixes & Master Summing
        auto master_view = self->master_buffer.view_frames(frames_to_process);
        self->mixer.render(master_view);

        // 3. Push Master Output to PipeWire Master Out Ports
        if (self->port_out_l && self->port_out_r) {
            auto* dst_l = static_cast<float*>(pw_filter_get_dsp_buffer(self->port_out_l, n_samples));
            auto* dst_r = static_cast<float*>(pw_filter_get_dsp_buffer(self->port_out_r, n_samples));

            const Sample* m_l = master_view.channel(0);
            const Sample* m_r = master_view.channel(1);

            if (dst_l) {
                std::memcpy(dst_l, m_l, frames_to_process * sizeof(float));
                if (n_samples > frames_to_process) {
                    std::memset(dst_l + frames_to_process, 0, (n_samples - frames_to_process) * sizeof(float));
                }
            }
            if (dst_r) {
                std::memcpy(dst_r, m_r, frames_to_process * sizeof(float));
                if (n_samples > frames_to_process) {
                    std::memset(dst_r + frames_to_process, 0, (n_samples - frames_to_process) * sizeof(float));
                }
            }
        }
    }

    static void on_registry_global(void* data, uint32_t id, uint32_t /*permissions*/,
                                   const char* type, uint32_t /*version*/,
                                   const struct spa_dict* props) noexcept {
        auto* self = static_cast<Impl*>(data);
        if (!self || !props) return;

        std::lock_guard<std::mutex> lock(self->registry_mutex);

        if (std::strcmp(type, PW_TYPE_INTERFACE_Node) == 0) {
            PwNodeInfo node{};
            node.id = id;
            const char* n = spa_dict_lookup(props, PW_KEY_NODE_NAME);
            const char* d = spa_dict_lookup(props, PW_KEY_NODE_DESCRIPTION);
            const char* mc = spa_dict_lookup(props, PW_KEY_MEDIA_CLASS);
            const char* app = spa_dict_lookup(props, PW_KEY_APP_NAME);
            if (n) node.name = n;
            if (d) node.description = d;
            if (mc) node.media_class = mc;
            if (app) node.app_name = app;
            self->nodes[id] = std::move(node);

        } else if (std::strcmp(type, PW_TYPE_INTERFACE_Port) == 0) {
            PwPortInfo port{};
            port.id = id;
            const char* nid = spa_dict_lookup(props, PW_KEY_NODE_ID);
            const char* pn = spa_dict_lookup(props, PW_KEY_PORT_NAME);
            const char* pd = spa_dict_lookup(props, PW_KEY_PORT_DIRECTION);
            const char* ch = spa_dict_lookup(props, PW_KEY_AUDIO_CHANNEL);
            const char* al = spa_dict_lookup(props, PW_KEY_PORT_ALIAS);
            if (nid) port.node_id = static_cast<uint32_t>(std::strtoul(nid, nullptr, 10));
            if (pn) port.name = pn;
            if (pd) port.direction = pd;
            if (ch) port.channel = ch;
            if (al) port.alias = al;

            // Associate ports with our own filter node
            if (self->our_node_id != 0 && port.node_id == self->our_node_id) {
                if (port.name == "Master Out L") self->our_master_out_l_id = id;
                else if (port.name == "Master Out R") self->our_master_out_r_id = id;
                else {
                    for (size_t t = 0; t < MixerGraph::kMaxTracks; ++t) {
                        std::string exp_l = "Track " + std::to_string(t + 1) + " In L";
                        std::string exp_r = "Track " + std::to_string(t + 1) + " In R";
                        if (port.name == exp_l) self->our_track_in_l_ids[t] = id;
                        else if (port.name == exp_r) self->our_track_in_r_ids[t] = id;
                    }
                }
            }

            self->ports[id] = std::move(port);
        }
    }

    static void on_registry_global_remove(void* data, uint32_t id) noexcept {
        auto* self = static_cast<Impl*>(data);
        if (!self) return;

        std::lock_guard<std::mutex> lock(self->registry_mutex);
        self->nodes.erase(id);
        self->ports.erase(id);
    }

    static void on_state_changed(void* userdata, enum pw_filter_state /*old_state*/,
                                 enum pw_filter_state state, const char* error) noexcept {
        auto* self = static_cast<Impl*>(userdata);
        if (!self) return;

        if (state == PW_FILTER_STATE_ERROR) {
            std::cerr << "[PipeWireBackend] Filter error: " << (error ? error : "unknown") << std::endl;
            self->running.store(false, std::memory_order_relaxed);
        } else if (state == PW_FILTER_STATE_UNCONNECTED) {
            self->running.store(false, std::memory_order_relaxed);
        } else if (state == PW_FILTER_STATE_STREAMING || state == PW_FILTER_STATE_PAUSED) {
            self->running.store(true, std::memory_order_relaxed);

            // Hook native registry listener upon connection
            self->our_node_id = pw_filter_get_node_id(self->filter);

            if (!self->registry) {
                self->core = pw_filter_get_core(self->filter);
                if (self->core) {
                    static const struct pw_registry_events reg_events = {
                        .version = PW_VERSION_REGISTRY_EVENTS,
                        .global = on_registry_global,
                        .global_remove = on_registry_global_remove,
                    };
                    self->registry = pw_core_get_registry(self->core, PW_VERSION_REGISTRY, 0);
                    if (self->registry) {
                        pw_registry_add_listener(self->registry, &self->registry_listener, &reg_events, self);
                    }
                }
            }
        }
    }

    struct pw_proxy* create_native_link(uint32_t out_node, uint32_t out_port,
                                         uint32_t in_node, uint32_t in_port) {
        if (!core) return nullptr;

        struct pw_properties* props = pw_properties_new(
            PW_KEY_LINK_OUTPUT_NODE, std::to_string(out_node).c_str(),
            PW_KEY_LINK_OUTPUT_PORT, std::to_string(out_port).c_str(),
            PW_KEY_LINK_INPUT_NODE, std::to_string(in_node).c_str(),
            PW_KEY_LINK_INPUT_PORT, std::to_string(in_port).c_str(),
            PW_KEY_OBJECT_LINGER, "true",
            nullptr);
        if (!props) return nullptr;

        pw_thread_loop_lock(loop);
        auto* link = static_cast<struct pw_proxy*>(pw_core_create_object(
            core,
            "link-factory",
            PW_TYPE_INTERFACE_Link,
            PW_VERSION_LINK,
            &props->dict,
            0));
        pw_thread_loop_unlock(loop);

        pw_properties_free(props);
        return link;
    }

    bool connect_sink_internal(const std::string& sink_name_or_id) {
        if (!core || our_node_id == 0 || our_master_out_l_id == 0 || our_master_out_r_id == 0) {
            return false;
        }

        if (loop) pw_thread_loop_lock(loop);
        for (auto* proxy : master_links) {
            if (proxy) pw_proxy_destroy(proxy);
        }
        master_links.clear();
        active_master_sink_node.clear();
        active_master_sink_display.clear();
        if (loop) pw_thread_loop_unlock(loop);

        if (sink_name_or_id.empty()) return true;

        uint32_t sink_node_id = 0;
        std::string display_name;

        {
            std::lock_guard<std::mutex> lock(registry_mutex);
            for (const auto& [nid, ninfo] : nodes) {
                if (nid == our_node_id) continue;
                if (ninfo.name == sink_name_or_id || (!ninfo.description.empty() && ninfo.description == sink_name_or_id)) {
                    sink_node_id = nid;
                    display_name = !ninfo.description.empty() ? ninfo.description : ninfo.name;
                    break;
                }
            }

            if (sink_node_id == 0) {
                try {
                    uint32_t parsed_id = static_cast<uint32_t>(std::stoul(sink_name_or_id));
                    if (nodes.find(parsed_id) != nodes.end()) {
                        sink_node_id = parsed_id;
                        display_name = !nodes[parsed_id].description.empty() ? nodes[parsed_id].description : nodes[parsed_id].name;
                    }
                } catch (...) {}
            }
        }

        if (sink_node_id == 0) return false;

        uint32_t sink_port_l = 0;
        uint32_t sink_port_r = 0;

        {
            std::lock_guard<std::mutex> lock(registry_mutex);
            for (const auto& [pid, pinfo] : ports) {
                if (pinfo.node_id != sink_node_id || pinfo.direction != "in") continue;

                if (pinfo.channel == "FL" || pinfo.name.find("playback_FL") != std::string::npos ||
                    pinfo.name.find("playback_0") != std::string::npos || pinfo.name.find("1") != std::string::npos) {
                    if (sink_port_l == 0) sink_port_l = pid;
                } else if (pinfo.channel == "FR" || pinfo.name.find("playback_FR") != std::string::npos ||
                           pinfo.name.find("playback_1") != std::string::npos || pinfo.name.find("2") != std::string::npos) {
                    if (sink_port_r == 0) sink_port_r = pid;
                }
            }

            if (sink_port_l == 0) {
                for (const auto& [pid, pinfo] : ports) {
                    if (pinfo.node_id == sink_node_id && pinfo.direction == "in") {
                        sink_port_l = pid;
                        break;
                    }
                }
            }
            if (sink_port_r == 0) sink_port_r = sink_port_l;
        }

        if (sink_port_l != 0 && our_master_out_l_id != 0) {
            auto* link_l = create_native_link(our_node_id, our_master_out_l_id, sink_node_id, sink_port_l);
            if (link_l) master_links.push_back(link_l);
        }

        if (sink_port_r != 0 && our_master_out_r_id != 0) {
            auto* link_r = create_native_link(our_node_id, our_master_out_r_id, sink_node_id, sink_port_r);
            if (link_r) master_links.push_back(link_r);
        }

        active_master_sink_node = sink_name_or_id;
        active_master_sink_display = display_name;
        return !master_links.empty();
    }

    void auto_connect_master_output() {
        if (!core || our_node_id == 0 || our_master_out_l_id == 0 || our_master_out_r_id == 0) {
            return;
        }

        if (!active_master_sink_node.empty()) {
            connect_sink_internal(active_master_sink_node);
            return;
        }

        std::string sink_to_connect;
        {
            std::lock_guard<std::mutex> lock(registry_mutex);
            for (const auto& [nid, ninfo] : nodes) {
                if (nid == our_node_id) continue;
                if (ninfo.media_class == "Audio/Sink" || ninfo.name.rfind("alsa_output", 0) == 0) {
                    sink_to_connect = ninfo.name;
                    break;
                }
            }
        }

        if (!sink_to_connect.empty()) {
            connect_sink_internal(sink_to_connect);
        }
    }
};

PipeWireBackend::PipeWireBackend(MixerGraph& mixer)
    : m_impl(std::make_unique<Impl>(mixer)) {}

PipeWireBackend::~PipeWireBackend() {
    stop();
}

PipeWireBackend::PipeWireBackend(PipeWireBackend&&) noexcept = default;
PipeWireBackend& PipeWireBackend::operator=(PipeWireBackend&&) noexcept = default;

bool PipeWireBackend::init(const std::string& node_name, uint32_t sample_rate) {
    m_impl->node_name = node_name;
    m_impl->sample_rate = sample_rate;

    pw_init(nullptr, nullptr);

    m_impl->loop = pw_thread_loop_new(node_name.c_str(), nullptr);
    if (!m_impl->loop) {
        std::cerr << "[PipeWireBackend] Failed to create pw_thread_loop" << std::endl;
        return false;
    }

    if (pw_thread_loop_start(m_impl->loop) < 0) {
        std::cerr << "[PipeWireBackend] Failed to start pw_thread_loop" << std::endl;
        return false;
    }

    pw_thread_loop_lock(m_impl->loop);

    struct pw_properties* props = pw_properties_new(
        PW_KEY_MEDIA_TYPE, "Audio",
        PW_KEY_MEDIA_CATEGORY, "Filter",
        PW_KEY_MEDIA_ROLE, "DSP",
        PW_KEY_APP_NAME, node_name.c_str(),
        PW_KEY_NODE_NAME, "aethel_mixer_graph",
        PW_KEY_NODE_DESCRIPTION, "Aethel Engine Multitrack Virtual Sinks & Console Desk",
        PW_KEY_NODE_AUTOCONNECT, "true",
        nullptr);

    static const struct pw_filter_events events = {
        .version = PW_VERSION_FILTER_EVENTS,
        .state_changed = Impl::on_state_changed,
        .process = Impl::on_process,
    };

    m_impl->filter = pw_filter_new_simple(
        pw_thread_loop_get_loop(m_impl->loop),
        node_name.c_str(),
        props,
        &events,
        m_impl.get());

    if (!m_impl->filter) {
        pw_thread_loop_unlock(m_impl->loop);
        std::cerr << "[PipeWireBackend] Failed to create pw_filter" << std::endl;
        return false;
    }

    // 32-bit Float DSP format parameter
    uint8_t buffer[1024];
    struct spa_pod_builder b = SPA_POD_BUILDER_INIT(buffer, sizeof(buffer));
    struct spa_audio_info_dsp info{};
    info.format = SPA_AUDIO_FORMAT_DSP_F32;
    const struct spa_pod* params[1];
    params[0] = spa_format_audio_dsp_build(&b, SPA_PARAM_EnumFormat, &info);

    // 1. Create Master Output Ports
    struct pw_properties* p_out_l = pw_properties_new(
        PW_KEY_PORT_NAME, "Master Out L",
        PW_KEY_PORT_DIRECTION, "out",
        PW_KEY_AUDIO_CHANNEL, "FL",
        nullptr);
    m_impl->port_out_l = pw_filter_add_port(m_impl->filter, PW_DIRECTION_OUTPUT,
                                             PW_FILTER_PORT_FLAG_MAP_BUFFERS, 0, p_out_l, params, 1);

    struct pw_properties* p_out_r = pw_properties_new(
        PW_KEY_PORT_NAME, "Master Out R",
        PW_KEY_PORT_DIRECTION, "out",
        PW_KEY_AUDIO_CHANNEL, "FR",
        nullptr);
    m_impl->port_out_r = pw_filter_add_port(m_impl->filter, PW_DIRECTION_OUTPUT,
                                             PW_FILTER_PORT_FLAG_MAP_BUFFERS, 0, p_out_r, params, 1);

    // 2. Create Virtual Input Sinks for Mixer Tracks (external apps can patch into any track)
    for (size_t i = 0; i < MixerGraph::kMaxTracks; ++i) {
        uint32_t trk_num = static_cast<uint32_t>(i + 1);
        std::string name_l = "Track " + std::to_string(trk_num) + " In L";
        std::string name_r = "Track " + std::to_string(trk_num) + " In R";

        struct pw_properties* p_in_l = pw_properties_new(
            PW_KEY_PORT_NAME, name_l.c_str(),
            PW_KEY_PORT_DIRECTION, "in",
            PW_KEY_AUDIO_CHANNEL, "FL",
            nullptr);
        m_impl->track_ports[i].port_in_l = pw_filter_add_port(m_impl->filter, PW_DIRECTION_INPUT,
                                                               PW_FILTER_PORT_FLAG_MAP_BUFFERS, 0, p_in_l, params, 1);

        struct pw_properties* p_in_r = pw_properties_new(
            PW_KEY_PORT_NAME, name_r.c_str(),
            PW_KEY_PORT_DIRECTION, "in",
            PW_KEY_AUDIO_CHANNEL, "FR",
            nullptr);
        m_impl->track_ports[i].port_in_r = pw_filter_add_port(m_impl->filter, PW_DIRECTION_INPUT,
                                                               PW_FILTER_PORT_FLAG_MAP_BUFFERS, 0, p_in_r, params, 1);
    }

    pw_thread_loop_unlock(m_impl->loop);
    return true;
}

bool PipeWireBackend::start() {
    if (!m_impl->filter || !m_impl->loop) return false;
    if (m_impl->running.load(std::memory_order_relaxed)) return true;

    pw_thread_loop_lock(m_impl->loop);
    int res = pw_filter_connect(m_impl->filter, PW_FILTER_FLAG_RT_PROCESS, nullptr, 0);
    pw_thread_loop_unlock(m_impl->loop);

    if (res < 0) {
        std::cerr << "[PipeWireBackend] pw_filter_connect failed: " << res << std::endl;
        return false;
    }

    m_impl->running.store(true, std::memory_order_relaxed);

    // Native Master Sink Auto-Connection & Live Discovery in background thread
    m_impl->discovery_thread = std::thread([this]() {
        // Wait briefly for initial PipeWire registry globals to populate
        for (int i = 0; i < 20; ++i) {
            if (!m_impl->running.load(std::memory_order_relaxed)) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (!m_impl->running.load(std::memory_order_relaxed)) return;

        refresh_discovery();
        m_impl->auto_connect_master_output();
        refresh_discovery();
    });

    return true;
}

void PipeWireBackend::stop() {
    if (!m_impl) return;

    m_impl->running.store(false, std::memory_order_relaxed);
    if (m_impl->discovery_thread.joinable()) {
        m_impl->discovery_thread.join();
    }

    if (!m_impl->loop) return;

    pw_thread_loop_lock(m_impl->loop);

    // 1. Destroy active native master links
    for (auto* proxy : m_impl->master_links) {
        if (proxy) pw_proxy_destroy(proxy);
    }
    m_impl->master_links.clear();

    // 2. Destroy active native track links
    for (auto& [tid, proxies] : m_impl->track_links) {
        for (auto* proxy : proxies) {
            if (proxy) pw_proxy_destroy(proxy);
        }
    }
    m_impl->track_links.clear();

    // 3. Unbind registry listener
    if (m_impl->registry) {
        spa_hook_remove(&m_impl->registry_listener);
        pw_proxy_destroy(reinterpret_cast<struct pw_proxy*>(m_impl->registry));
        m_impl->registry = nullptr;
    }

    // 4. Disconnect and free filter
    if (m_impl->filter) {
        pw_filter_disconnect(m_impl->filter);
        pw_filter_destroy(m_impl->filter);
        m_impl->filter = nullptr;
    }
    pw_thread_loop_unlock(m_impl->loop);

    pw_thread_loop_stop(m_impl->loop);
    pw_thread_loop_destroy(m_impl->loop);
    m_impl->loop = nullptr;

    pw_deinit();
}

bool PipeWireBackend::is_running() const noexcept {
    return m_impl && m_impl->running.load(std::memory_order_relaxed);
}

uint32_t PipeWireBackend::sample_rate() const noexcept {
    return m_impl ? m_impl->sample_rate : 48000;
}

// ----------------------------------------------------------------------------
// Stream & Device Discovery Implementation (Pure C-API / pw_registry)
// ----------------------------------------------------------------------------
void PipeWireBackend::refresh_discovery() {
    std::lock_guard<std::mutex> lock(m_impl->registry_mutex);
    std::vector<DiscoveredStreamPair> sources;
    std::vector<DiscoveredStreamPair> sinks;

    for (const auto& [nid, ninfo] : m_impl->nodes) {
        if (nid == m_impl->our_node_id || ninfo.name == "aethel_mixer_graph" ||
            ninfo.name.find("Midi") != std::string::npos || ninfo.name.find("bluez_midi") != std::string::npos) {
            continue;
        }

        std::vector<Impl::PwPortInfo> in_ports;
        std::vector<Impl::PwPortInfo> out_ports;

        for (const auto& [pid, pinfo] : m_impl->ports) {
            if (pinfo.node_id != nid) continue;
            if (pinfo.direction == "out") out_ports.push_back(pinfo);
            else if (pinfo.direction == "in") in_ports.push_back(pinfo);
        }

        std::string dev_name = !ninfo.description.empty() ? ninfo.description : (!ninfo.app_name.empty() ? ninfo.app_name : ninfo.name);

        bool is_hw_capture = (ninfo.media_class == "Audio/Source" || ninfo.media_class.find("Source") != std::string::npos || ninfo.name.rfind("alsa_input", 0) == 0);
        bool is_monitor = (ninfo.media_class == "Audio/Sink" || ninfo.name.rfind("alsa_output", 0) == 0);

        // Sort out_ports by ID
        std::sort(out_ports.begin(), out_ports.end(), [](const Impl::PwPortInfo& a, const Impl::PwPortInfo& b) {
            return a.id < b.id;
        });

        // 1. Process Output Ports (Capture Sources & Apps)
        if (!out_ports.empty()) {
            // Find FL and FR (or 1 and 2)
            const Impl::PwPortInfo* p_fl = nullptr;
            const Impl::PwPortInfo* p_fr = nullptr;

            for (const auto& p : out_ports) {
                if (p.channel == "FL" || p.name.find("FL") != std::string::npos ||
                    p.name.find("capture_1") != std::string::npos || p.name.find("1") != std::string::npos) {
                    if (!p_fl) p_fl = &p;
                } else if (p.channel == "FR" || p.name.find("FR") != std::string::npos ||
                           p.name.find("capture_2") != std::string::npos || p.name.find("2") != std::string::npos) {
                    if (!p_fr) p_fr = &p;
                }
            }

            // A. If multiple ports exist, add primary Stereo Pair
            if (out_ports.size() >= 2) {
                DiscoveredStreamPair pair{};
                pair.node_id = nid;
                pair.node_name = ninfo.name;
                pair.display_name = dev_name + " (In 1+2 / Stereo)";
                pair.port_l = ninfo.name + ":" + (p_fl ? p_fl->name : out_ports[0].name);
                pair.port_r = ninfo.name + ":" + (p_fr ? p_fr->name : out_ports[1].name);
                pair.is_hardware_capture = is_hw_capture;
                pair.is_monitor = is_monitor;
                pair.is_mono = false;
                pair.channel_count = 2;
                sources.push_back(std::move(pair));

                // If > 2 ports (e.g. 4, 8, 16 channel interfaces), add subsequent pairs (3+4, 5+6...)
                for (size_t p = 2; p + 1 < out_ports.size(); p += 2) {
                    DiscoveredStreamPair sub_pair{};
                    sub_pair.node_id = nid;
                    sub_pair.node_name = ninfo.name;
                    sub_pair.display_name = dev_name + " (In " + std::to_string(p + 1) + "+" + std::to_string(p + 2) + " / Stereo)";
                    sub_pair.port_l = ninfo.name + ":" + out_ports[p].name;
                    sub_pair.port_r = ninfo.name + ":" + out_ports[p + 1].name;
                    sub_pair.is_hardware_capture = is_hw_capture;
                    sub_pair.is_monitor = is_monitor;
                    sub_pair.is_mono = false;
                    sub_pair.channel_count = 2;
                    sources.push_back(std::move(sub_pair));
                }
            }

            // B. Add Granular Mono Sources for each port
            for (size_t p = 0; p < out_ports.size(); ++p) {
                const auto& port = out_ports[p];
                DiscoveredStreamPair mono_pair{};
                mono_pair.node_id = nid;
                mono_pair.node_name = ninfo.name;
                mono_pair.port_l = ninfo.name + ":" + port.name;
                mono_pair.port_r = mono_pair.port_l;
                mono_pair.is_hardware_capture = is_hw_capture;
                mono_pair.is_monitor = is_monitor;
                mono_pair.is_mono = true;
                mono_pair.channel_count = 1;

                std::string ch_name = !port.channel.empty() ? port.channel : port.name;
                if (out_ports.size() == 1) {
                    mono_pair.display_name = dev_name + " (Mono)";
                } else {
                    mono_pair.display_name = dev_name + " - In " + ch_name + " (Mono)";
                }
                sources.push_back(std::move(mono_pair));
            }
        }

        // 2. Process Input Ports (Playback Sinks)
        if (!in_ports.empty()) {
            DiscoveredStreamPair sink_pair{};
            sink_pair.node_id = nid;
            sink_pair.node_name = ninfo.name;
            sink_pair.display_name = dev_name;
            sink_pair.is_hardware_capture = false;
            sink_pair.is_monitor = false;
            sink_pair.is_mono = (in_ports.size() == 1);
            sink_pair.channel_count = static_cast<uint32_t>(in_ports.size());

            for (const auto& p : in_ports) {
                if (p.channel == "FL" || p.name.find("FL") != std::string::npos ||
                    p.name.find("playback_FL") != std::string::npos || p.name.find("playback_0") != std::string::npos || p.name.find("1") != std::string::npos) {
                    if (sink_pair.port_l.empty()) sink_pair.port_l = ninfo.name + ":" + p.name;
                } else if (p.channel == "FR" || p.name.find("FR") != std::string::npos ||
                           p.name.find("playback_FR") != std::string::npos || p.name.find("playback_1") != std::string::npos || p.name.find("2") != std::string::npos) {
                    if (sink_pair.port_r.empty()) sink_pair.port_r = ninfo.name + ":" + p.name;
                }
            }
            if (sink_pair.port_l.empty() && !in_ports.empty()) sink_pair.port_l = ninfo.name + ":" + in_ports[0].name;
            if (sink_pair.port_r.empty()) sink_pair.port_r = sink_pair.port_l;

            sinks.push_back(std::move(sink_pair));
        }
    }

    std::lock_guard<std::mutex> dlock(m_impl->discovery_mutex);
    m_impl->discovered_sources = std::move(sources);
    m_impl->discovered_sinks = std::move(sinks);
}

std::vector<DiscoveredStreamPair> PipeWireBackend::get_available_sources() const {
    std::lock_guard<std::mutex> lock(m_impl->discovery_mutex);
    return m_impl->discovered_sources;
}

std::vector<DiscoveredStreamPair> PipeWireBackend::get_available_sinks() const {
    std::lock_guard<std::mutex> lock(m_impl->discovery_mutex);
    return m_impl->discovered_sinks;
}

std::vector<DiscoveredStreamPair> PipeWireBackend::get_hardware_inputs() const {
    auto all = get_available_sources();
    std::vector<DiscoveredStreamPair> hw;
    for (const auto& s : all) {
        if (s.is_hardware_capture) hw.push_back(s);
    }
    return hw;
}

std::vector<DiscoveredStreamPair> PipeWireBackend::get_app_sources() const {
    auto all = get_available_sources();
    std::vector<DiscoveredStreamPair> apps;
    for (const auto& s : all) {
        if (!s.is_hardware_capture && !s.is_monitor) apps.push_back(s);
    }
    return apps;
}

bool PipeWireBackend::link_source_to_track(const DiscoveredStreamPair& stream, uint32_t track_id) {
    if (stream.port_l.empty() || track_id == 0 || track_id > MixerGraph::kMaxTracks) return false;

    // Unlink any existing links for this track first to prevent double-patching
    unlink_all_for_track(track_id);

    auto* track = m_impl->mixer.get_track(track_id);
    if (track) {
        track->set_input_mode(TrackInputMode::PipeWireStream);
    }

    // Attempt Native PipeWire Linking via pw_core_create_object
    if (m_impl->core && m_impl->our_node_id != 0) {
        uint32_t our_in_l = m_impl->our_track_in_l_ids[track_id - 1];
        uint32_t our_in_r = m_impl->our_track_in_r_ids[track_id - 1];

        uint32_t src_node_id = 0;
        uint32_t src_port_l_id = 0;
        uint32_t src_port_r_id = 0;

        {
            std::lock_guard<std::mutex> lock(m_impl->registry_mutex);
            for (const auto& [nid, ninfo] : m_impl->nodes) {
                if (ninfo.name == stream.node_name || nid == stream.node_id) {
                    src_node_id = nid;
                    break;
                }
            }

            if (src_node_id != 0) {
                for (const auto& [pid, pinfo] : m_impl->ports) {
                    if (pinfo.node_id != src_node_id) continue;
                    std::string full_name = stream.node_name + ":" + pinfo.name;
                    if (full_name == stream.port_l || pinfo.name == stream.port_l) src_port_l_id = pid;
                    if (full_name == stream.port_r || pinfo.name == stream.port_r) src_port_r_id = pid;
                }
            }
        }

        if (src_node_id != 0 && our_in_l != 0 && src_port_l_id != 0) {
            uint32_t right_target = (stream.is_mono || src_port_r_id == 0) ? src_port_l_id : src_port_r_id;

            auto* link_l = m_impl->create_native_link(src_node_id, src_port_l_id, m_impl->our_node_id, our_in_l);
            auto* link_r = m_impl->create_native_link(src_node_id, right_target, m_impl->our_node_id, our_in_r != 0 ? our_in_r : our_in_l);

            if (link_l) m_impl->track_links[track_id].push_back(link_l);
            if (link_r) m_impl->track_links[track_id].push_back(link_r);
            m_impl->active_track_sources[track_id] = stream;
            return true;
        }
    }

    m_impl->active_track_sources[track_id] = stream;
    return true;
}

bool PipeWireBackend::unlink_source_from_track(const DiscoveredStreamPair& /*stream*/, uint32_t track_id) {
    if (track_id == 0 || track_id > MixerGraph::kMaxTracks) return false;

    auto it = m_impl->track_links.find(track_id);
    if (it != m_impl->track_links.end()) {
        if (m_impl->loop) pw_thread_loop_lock(m_impl->loop);
        for (auto* proxy : it->second) {
            if (proxy) pw_proxy_destroy(proxy);
        }
        if (m_impl->loop) pw_thread_loop_unlock(m_impl->loop);
        m_impl->track_links.erase(it);
    }
    m_impl->active_track_sources.erase(track_id);

    auto* track = m_impl->mixer.get_track(track_id);
    if (track) {
        track->set_input_mode(TrackInputMode::InternalClip);
    }
    return true;
}

bool PipeWireBackend::unlink_all_for_track(uint32_t track_id) {
    return unlink_source_from_track({}, track_id);
}

std::optional<DiscoveredStreamPair> PipeWireBackend::get_track_source(uint32_t track_id) const {
    if (!m_impl) return std::nullopt;
    auto it = m_impl->active_track_sources.find(track_id);
    if (it != m_impl->active_track_sources.end()) {
        return it->second;
    }
    return std::nullopt;
}

bool PipeWireBackend::is_track_linked(uint32_t track_id) const {
    return m_impl && m_impl->active_track_sources.find(track_id) != m_impl->active_track_sources.end();
}

bool PipeWireBackend::connect_master_to_sink(const std::string& sink_node_name) {
    return m_impl && m_impl->connect_sink_internal(sink_node_name);
}

bool PipeWireBackend::disconnect_master_output() {
    return m_impl && m_impl->connect_sink_internal("");
}

std::string PipeWireBackend::active_master_sink_node_name() const {
    return m_impl ? m_impl->active_master_sink_node : "";
}

std::string PipeWireBackend::active_master_sink_display_name() const {
    return m_impl ? m_impl->active_master_sink_display : "";
}

void PipeWireBackend::set_aoip_receiver(network::AoipReceiver* receiver) noexcept {
    m_impl->aoip_receiver.store(receiver, std::memory_order_relaxed);
}

network::AoipReceiver* PipeWireBackend::aoip_receiver() const noexcept {
    return m_impl->aoip_receiver.load(std::memory_order_relaxed);
}

} // namespace audio_core
