#pragma once

#include "audio_core/types.hpp"
#include "audio_core/ring_buffer.hpp"
#include "audio_core/modulation/polyphonic_synth.hpp"
#include "audio_core/modulation/modulation_matrix.hpp"
#include "audio_core/midi/midi_sync.hpp"
#include "audio_core/midi/midi_learn_router.hpp"
#include "audio_core/mixer_graph.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sound/asound.h>
#include <sound/asequencer.h>
#include <dirent.h>

#include <thread>
#include <atomic>
#include <vector>
#include <string>
#include <cstring>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <memory>
#include <mutex>

namespace audio_core::midi {

// ============================================================================
// AlsaSeqSubscription: Information about an active ALSA Sequencer subscription
// ============================================================================
struct AlsaSeqSubscription {
    int client_id{-1};
    int port_id{-1};
    std::string client_name;
    std::string port_name;
    bool is_system_announce{false};
};

// ============================================================================
// MidiDeviceInfo: Information about an available hardware/virtual MIDI port
// ============================================================================
struct MidiDeviceInfo {
    std::string path;         // Node path (e.g. "/dev/snd/midiC1D0")
    std::string name;         // Device / Interface name (e.g. "USB MIDI Interface")
    std::string card_id;      // ALSA card ID (e.g. "KeyLab")
    int card_index{0};
    int device_index{0};
    bool is_available{true};
};

// ============================================================================
// HardwareMidiReceiver: Real-Time Linux Hardware RawMIDI & Sequencer Ingestion
// Discovers ALSA/USB rawmidi interfaces, parses 1.0 byte streams with running
// status, and dispatches lock-free MidiEvents into the audio synth pipeline.
// ============================================================================
class HardwareMidiReceiver {
public:
    static constexpr size_t kDefaultQueueCapacity = 2048;

    explicit HardwareMidiReceiver(size_t queue_capacity = kDefaultQueueCapacity)
        : m_queue(queue_capacity) {
        m_stop_pipe[0] = -1;
        m_stop_pipe[1] = -1;
    }

    ~HardwareMidiReceiver() {
        close_device();
    }

    HardwareMidiReceiver(const HardwareMidiReceiver&) = delete;
    HardwareMidiReceiver& operator=(const HardwareMidiReceiver&) = delete;

    // ========================================================================
    // Device Enumeration (Linux ALSA RawMIDI)
    // ========================================================================
    static std::vector<MidiDeviceInfo> enumerate_devices() {
        std::vector<MidiDeviceInfo> list;

        // 1. Scan /dev/snd/ for midiC*D*
        DIR* dir = opendir("/dev/snd");
        if (dir) {
            struct dirent* entry = nullptr;
            while ((entry = readdir(dir)) != nullptr) {
                if (std::strncmp(entry->d_name, "midiC", 5) == 0) {
                    int c = 0;
                    int d = 0;
                    if (std::sscanf(entry->d_name, "midiC%dD%d", &c, &d) == 2) {
                        std::string full_path = std::string("/dev/snd/") + entry->d_name;
                        MidiDeviceInfo info;
                        info.path = full_path;
                        info.card_index = c;
                        info.device_index = d;
                        info.name = "USB MIDI Device (" + full_path + ")";
                        info.card_id = "Card " + std::to_string(c);

                        // Try to probe ALSA info via ioctl
                        int fd = ::open(full_path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
                        if (fd >= 0) {
                            struct snd_rawmidi_info r_info{};
                            r_info.stream = SNDRV_RAWMIDI_STREAM_INPUT;
                            if (ioctl(fd, SNDRV_RAWMIDI_IOCTL_INFO, &r_info) >= 0) {
                                if (r_info.name[0] != '\0') info.name = reinterpret_cast<const char*>(r_info.name);
                                if (r_info.id[0] != '\0') info.card_id = reinterpret_cast<const char*>(r_info.id);
                            }
                            ::close(fd);
                            info.is_available = true;
                        } else {
                            info.is_available = false;
                        }

                        // Try reading /proc/asound/cardX/id for better name
                        std::string card_path = "/proc/asound/card" + std::to_string(c) + "/id";
                        std::ifstream id_file(card_path);
                        if (id_file.is_open()) {
                            std::string line;
                            if (std::getline(id_file, line) && !line.empty()) {
                                info.card_id = line;
                            }
                        }

                        list.push_back(info);
                    }
                }
            }
            closedir(dir);
        }

        // Sort by card and device index
        std::sort(list.begin(), list.end(), [](const MidiDeviceInfo& a, const MidiDeviceInfo& b) {
            if (a.card_index != b.card_index) return a.card_index < b.card_index;
            return a.device_index < b.device_index;
        });

        return list;
    }

    // ========================================================================
    // Connection Lifecycle
    // ========================================================================
    bool open_rawmidi_device(const std::string& path) {
        if (m_fd >= 0) { ::close(m_fd); m_fd = -1; }

        m_fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (m_fd < 0) {
            return false;
        }

        m_rawmidi_device_path = path;
        m_is_mock = false;
        ensure_listener_thread_running();
        update_device_path_string();
        return true;
    }

    bool open_device(const std::string& path) {
        return open_rawmidi_device(path);
    }

    bool open_alsa_sequencer(const std::string& client_name = "Aethel Audio Desk", const std::string& port_name = "Aethel MIDI In") {
        if (m_seq_fd >= 0) return true;

        int fd = ::open("/dev/snd/seq", O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) return false;

        int client_id = -1;
        if (ioctl(fd, SNDRV_SEQ_IOCTL_CLIENT_ID, &client_id) < 0) {
            ::close(fd);
            return false;
        }

        struct snd_seq_client_info cinfo{};
        cinfo.client = client_id;
        if (ioctl(fd, SNDRV_SEQ_IOCTL_GET_CLIENT_INFO, &cinfo) >= 0) {
            std::strncpy(cinfo.name, client_name.c_str(), sizeof(cinfo.name) - 1);
            (void)ioctl(fd, SNDRV_SEQ_IOCTL_SET_CLIENT_INFO, &cinfo);
        }

        struct snd_seq_port_info pinfo{};
        pinfo.addr.client = static_cast<unsigned char>(client_id);
        std::strncpy(pinfo.name, port_name.c_str(), sizeof(pinfo.name) - 1);
        pinfo.capability = SNDRV_SEQ_PORT_CAP_WRITE | SNDRV_SEQ_PORT_CAP_SUBS_WRITE | SNDRV_SEQ_PORT_CAP_READ;
        pinfo.type = SNDRV_SEQ_PORT_TYPE_MIDI_GENERIC | SNDRV_SEQ_PORT_TYPE_APPLICATION | SNDRV_SEQ_PORT_TYPE_SYNTH;
        pinfo.midi_channels = 16;
        if (ioctl(fd, SNDRV_SEQ_IOCTL_CREATE_PORT, &pinfo) < 0) {
            ::close(fd);
            return false;
        }

        m_seq_fd = fd;
        m_seq_client_id = client_id;
        m_seq_port_id = pinfo.addr.port;
        m_seq_port_name = client_name + ":" + std::to_string(pinfo.addr.port) + " (" + port_name + ")";

        // Maximize ALSA Sequencer Kernel Event Pool (up to 2000 cells)
        struct snd_seq_client_pool pool{};
        pool.client = client_id;
        if (ioctl(fd, SNDRV_SEQ_IOCTL_GET_CLIENT_POOL, &pool) >= 0) {
            pool.output_pool = 2000;
            pool.input_pool = 2000;
            pool.output_room = 1;
            (void)ioctl(fd, SNDRV_SEQ_IOCTL_SET_CLIENT_POOL, &pool);
        }

        m_is_mock = false;
        ensure_listener_thread_running();
        update_device_path_string();

        // Auto-discover and subscribe to available MIDI outputs and system announce
        auto_subscribe_all(true);
        return true;
    }

    bool auto_connect(bool enable_alsa_seq = true) {
        bool raw_ok = false;
        auto devs = enumerate_devices();
        for (const auto& dev : devs) {
            if (dev.is_available) {
                if (open_rawmidi_device(dev.path)) {
                    std::cout << "[HardwareMidi] Connected RawMIDI to: " << dev.name << " (" << dev.path << ")" << std::endl;
                    raw_ok = true;
                    break;
                }
            }
        }

        bool seq_ok = false;
        if (enable_alsa_seq) {
            seq_ok = open_alsa_sequencer("Aethel Audio Desk", "Aethel MIDI In");
            if (seq_ok) {
                std::cout << "[HardwareMidi] Opened ALSA Sequencer port: " << m_seq_port_name << std::endl;
            }
        }

        if (raw_ok || seq_ok) {
            m_is_mock = false;
            update_device_path_string();
            return true;
        }

        // Fallback: Virtual / Mock mode for headless testing & machines without plugged USB MIDI or /dev/snd/seq
        m_is_mock = true;
        m_current_device_path = "Virtual / Mock MIDI Receiver";
        return true;
    }

    void close_device() noexcept {
        if (m_running.load(std::memory_order_relaxed)) {
            m_running.store(false, std::memory_order_release);
            if (m_stop_pipe[1] >= 0) {
                uint8_t b = 1;
                (void)::write(m_stop_pipe[1], &b, sizeof(b));
            }
            if (m_thread.joinable()) {
                m_thread.join();
            }
        }

        unsubscribe_all();

        if (m_stop_pipe[0] >= 0) { ::close(m_stop_pipe[0]); m_stop_pipe[0] = -1; }
        if (m_stop_pipe[1] >= 0) { ::close(m_stop_pipe[1]); m_stop_pipe[1] = -1; }
        if (m_fd >= 0) { ::close(m_fd); m_fd = -1; }
        if (m_seq_fd >= 0) { ::close(m_seq_fd); m_seq_fd = -1; }

        m_rawmidi_device_path.clear();
        m_seq_client_id = -1;
        m_seq_port_id = -1;
        m_seq_port_name.clear();
        m_current_device_path.clear();
        m_is_mock = false;
        m_running_status = 0;
        m_expected_bytes = 0;
        m_received_bytes = 0;
        m_in_sysex = false;
        m_sysex_len = 0;
        m_clock_gen.reset();
        m_event_count.store(0, std::memory_order_relaxed);
        m_dropped_events.store(0, std::memory_order_relaxed);
    }

    [[nodiscard]] bool is_connected() const noexcept {
        return ((m_fd >= 0 || m_seq_fd >= 0) && m_running.load(std::memory_order_relaxed)) || m_is_mock;
    }

    [[nodiscard]] bool is_mock() const noexcept { return m_is_mock; }
    [[nodiscard]] bool has_rawmidi() const noexcept { return m_fd >= 0; }
    [[nodiscard]] bool has_alsa_seq() const noexcept { return m_seq_fd >= 0; }
    [[nodiscard]] int seq_client_id() const noexcept { return m_seq_client_id; }
    [[nodiscard]] int seq_port_id() const noexcept { return m_seq_port_id; }
    [[nodiscard]] const std::string& seq_port_name() const noexcept { return m_seq_port_name; }
    [[nodiscard]] const std::string& current_device_path() const noexcept { return m_current_device_path; }

    bool send_seq_event(const struct snd_seq_event& ev) noexcept {
        if (m_is_mock) {
            parse_seq_event(ev);
            return true;
        }
        if (m_seq_fd < 0) return false;
        ssize_t w = ::write(m_seq_fd, &ev, sizeof(ev));
        return (w == static_cast<ssize_t>(sizeof(ev)));
    }

    void inject_seq_event(const struct snd_seq_event& ev) noexcept {
        parse_seq_event(ev);
    }

    // ========================================================================
    // ALSA Sequencer Dynamic Port Subscriptions & Hotplug
    // ========================================================================
    bool subscribe_to(int client_id, int port_id) {
        if (m_is_mock) {
            std::lock_guard<std::recursive_mutex> lock(m_subs_mutex);
            for (const auto& s : m_subscriptions) {
                if (s.client_id == client_id && s.port_id == port_id) return true;
            }
            AlsaSeqSubscription sub;
            sub.client_id = client_id;
            sub.port_id = port_id;
            sub.client_name = "Mock Client " + std::to_string(client_id);
            sub.port_name = "Mock Port " + std::to_string(port_id);
            sub.is_system_announce = (client_id == 0 && port_id == 1);
            m_subscriptions.push_back(std::move(sub));
            update_device_path_string();
            return true;
        }

        if (m_seq_fd < 0 || client_id == m_seq_client_id) return false;

        {
            std::lock_guard<std::recursive_mutex> lock(m_subs_mutex);
            for (const auto& s : m_subscriptions) {
                if (s.client_id == client_id && s.port_id == port_id) return true;
            }
        }

        std::string client_name = "Client " + std::to_string(client_id);
        std::string port_name = "Port " + std::to_string(port_id);
        bool is_announce = (client_id == 0 && port_id == 1);

        struct snd_seq_client_info cinfo{};
        cinfo.client = client_id;
        if (ioctl(m_seq_fd, SNDRV_SEQ_IOCTL_GET_CLIENT_INFO, &cinfo) >= 0) {
            if (cinfo.name[0] != '\0') client_name = cinfo.name;
        }

        struct snd_seq_port_info pinfo{};
        pinfo.addr.client = static_cast<unsigned char>(client_id);
        pinfo.addr.port = static_cast<unsigned char>(port_id);
        if (ioctl(m_seq_fd, SNDRV_SEQ_IOCTL_GET_PORT_INFO, &pinfo) >= 0) {
            if (pinfo.name[0] != '\0') port_name = pinfo.name;
            if (!is_announce && !(pinfo.capability & SNDRV_SEQ_PORT_CAP_SUBS_READ)) {
                return false;
            }
        }

        struct snd_seq_port_subscribe subs{};
        subs.sender.client = static_cast<unsigned char>(client_id);
        subs.sender.port = static_cast<unsigned char>(port_id);
        subs.dest.client = static_cast<unsigned char>(m_seq_client_id);
        subs.dest.port = static_cast<unsigned char>(m_seq_port_id);

        int res = ioctl(m_seq_fd, SNDRV_SEQ_IOCTL_SUBSCRIBE_PORT, &subs);
        if (res < 0 && errno != EEXIST && errno != EBUSY) {
            return false;
        }

        {
            std::lock_guard<std::recursive_mutex> lock(m_subs_mutex);
            bool exists = false;
            for (const auto& s : m_subscriptions) {
                if (s.client_id == client_id && s.port_id == port_id) {
                    exists = true;
                    break;
                }
            }
            if (!exists) {
                AlsaSeqSubscription sub;
                sub.client_id = client_id;
                sub.port_id = port_id;
                sub.client_name = client_name;
                sub.port_name = port_name;
                sub.is_system_announce = is_announce;
                m_subscriptions.push_back(std::move(sub));
            }
        }
        update_device_path_string();
        return true;
    }

    bool unsubscribe_from(int client_id, int port_id) {
        if (m_is_mock) {
            remove_subscription_record(client_id, port_id);
            return true;
        }
        if (m_seq_fd < 0) return false;

        struct snd_seq_port_subscribe subs{};
        subs.sender.client = static_cast<unsigned char>(client_id);
        subs.sender.port = static_cast<unsigned char>(port_id);
        subs.dest.client = static_cast<unsigned char>(m_seq_client_id);
        subs.dest.port = static_cast<unsigned char>(m_seq_port_id);

        (void)ioctl(m_seq_fd, SNDRV_SEQ_IOCTL_UNSUBSCRIBE_PORT, &subs);
        remove_subscription_record(client_id, port_id);
        return true;
    }

    size_t auto_subscribe_all(bool subscribe_system_announce = true) {
        if (m_is_mock) {
            if (m_subscriptions.empty()) {
                if (subscribe_system_announce) subscribe_to(0, 1);
                subscribe_to(14, 0);
            }
            return m_subscriptions.size();
        }

        if (m_seq_fd < 0) return 0;
        size_t count = 0;

        if (subscribe_system_announce) {
            if (subscribe_to(0, 1)) {
                ++count;
            }
        }

        struct snd_seq_client_info cinfo{};
        cinfo.client = -1;
        while (ioctl(m_seq_fd, SNDRV_SEQ_IOCTL_QUERY_NEXT_CLIENT, &cinfo) >= 0) {
            if (cinfo.client == m_seq_client_id || cinfo.client == 0) continue;

            struct snd_seq_port_info pinfo{};
            pinfo.addr.client = static_cast<unsigned char>(cinfo.client);
            pinfo.addr.port = static_cast<unsigned char>(-1);
            while (ioctl(m_seq_fd, SNDRV_SEQ_IOCTL_QUERY_NEXT_PORT, &pinfo) >= 0) {
                if (pinfo.capability & SNDRV_SEQ_PORT_CAP_SUBS_READ) {
                    if (subscribe_to(cinfo.client, pinfo.addr.port)) {
                        ++count;
                    }
                }
            }
        }
        return count;
    }

    void unsubscribe_all() noexcept {
        if (m_is_mock) {
            try {
                std::lock_guard<std::recursive_mutex> lock(m_subs_mutex);
                m_subscriptions.clear();
                update_device_path_string();
            } catch (...) {}
            return;
        }

        if (m_seq_fd < 0) return;
        std::vector<AlsaSeqSubscription> subs_copy;
        try {
            std::lock_guard<std::recursive_mutex> lock(m_subs_mutex);
            subs_copy = std::move(m_subscriptions);
            m_subscriptions.clear();
        } catch (...) {}

        for (const auto& s : subs_copy) {
            struct snd_seq_port_subscribe subs{};
            subs.sender.client = static_cast<unsigned char>(s.client_id);
            subs.sender.port = static_cast<unsigned char>(s.port_id);
            subs.dest.client = static_cast<unsigned char>(m_seq_client_id);
            subs.dest.port = static_cast<unsigned char>(m_seq_port_id);
            (void)ioctl(m_seq_fd, SNDRV_SEQ_IOCTL_UNSUBSCRIBE_PORT, &subs);
        }
        update_device_path_string();
    }

    [[nodiscard]] std::vector<AlsaSeqSubscription> active_subscriptions() const {
        std::lock_guard<std::recursive_mutex> lock(m_subs_mutex);
        return m_subscriptions;
    }

    [[nodiscard]] size_t subscription_count(bool include_system_announce = false) const noexcept {
        try {
            std::lock_guard<std::recursive_mutex> lock(m_subs_mutex);
            if (include_system_announce) return m_subscriptions.size();
            size_t cnt = 0;
            for (const auto& s : m_subscriptions) {
                if (!s.is_system_announce) ++cnt;
            }
            return cnt;
        } catch (...) {
            return 0;
        }
    }

    [[nodiscard]] bool is_subscribed(int client_id, int port_id) const noexcept {
        try {
            std::lock_guard<std::recursive_mutex> lock(m_subs_mutex);
            for (const auto& s : m_subscriptions) {
                if (s.client_id == client_id && s.port_id == port_id) return true;
            }
        } catch (...) {}
        return false;
    }

    // ========================================================================
    // MIDI Synchronization (Beat Clock 24 PPQN & MTC SMPTE Time Code)
    // ========================================================================
    [[nodiscard]] MidiSyncTracker& sync_tracker() noexcept { return m_sync_tracker; }
    [[nodiscard]] const MidiSyncTracker& sync_tracker() const noexcept { return m_sync_tracker; }

    [[nodiscard]] MidiClockGenerator& clock_generator() noexcept { return m_clock_gen; }
    [[nodiscard]] const MidiClockGenerator& clock_generator() const noexcept { return m_clock_gen; }

    void sync_to_clock(clock::TimelineClock& clock) noexcept {
        m_sync_tracker.apply_to_timeline_clock(clock);
    }

    void broadcast_seek_position(const clock::TimelineClock& clock) noexcept {
        if (clock.authority() != clock::ClockAuthority::Master) {
            return;
        }

        const uint64_t sample_pos = clock.sample_position();
        const double sr = static_cast<double>(clock.sample_rate());
        const double total_secs = static_cast<double>(sample_pos) / std::max(1.0, sr);

        // 1. Send SPP if Beat Clock is enabled
        if (m_clock_gen.is_beat_clock_enabled()) {
            const double total_beats = clock.position_snapshot().total_beats;
            uint16_t spp = static_cast<uint16_t>(std::clamp(std::floor(total_beats * 4.0), 0.0, 16383.0));
            send_midi_spp(spp);
            m_spp_tx_count.fetch_add(1, std::memory_order_relaxed);
            m_last_tx_spp.store(spp, std::memory_order_relaxed);
        }

        // 2. Send MTC Full Frame SysEx if MTC is enabled
        if (m_clock_gen.is_mtc_enabled()) {
            auto tc = MtcTimecode::from_seconds(total_secs, m_clock_gen.mtc_framerate());
            send_mtc_full_frame(tc);
            m_mtc_full_frame_tx_count.fetch_add(1, std::memory_order_relaxed);
            m_last_tx_mtc_packed.store(tc.pack(), std::memory_order_relaxed);
        }

        // 3. Re-align phase in clock generator
        m_clock_gen.align_to_sample_position(sample_pos, clock);
    }

    void process_master_clock(uint32_t frames, const clock::TimelineClock& clock) noexcept {
        if (clock.authority() != clock::ClockAuthority::Master) {
            return;
        }

        // 1. Detect discontinuous seek or timeline scrub and broadcast locator messages
        const uint64_t cur_seek_gen = clock.seek_generation();
        if (cur_seek_gen != m_last_handled_seek_gen.load(std::memory_order_relaxed)) {
            m_last_handled_seek_gen.store(cur_seek_gen, std::memory_order_relaxed);
            broadcast_seek_position(clock);
        }

        if (frames == 0 && !clock.is_playing()) {
            return;
        }

        const uint64_t sr = std::max(1u, clock.sample_rate());
        const uint64_t block_ts_ns = (clock.sample_position() * 1'000'000'000ULL) / sr;

        m_clock_gen.process_block(
            frames, clock,
            [this, block_ts_ns](uint8_t byte) {
                switch (byte) {
                    case 0xF8: send_midi_clock_tick(block_ts_ns); break;
                    case 0xFA: send_midi_start(); break;
                    case 0xFB: send_midi_continue(); break;
                    case 0xFC: send_midi_stop(); break;
                    default: break;
                }
            },
            [this](uint8_t qf_data) {
                send_mtc_qframe(qf_data);
            }
        );
    }

    [[nodiscard]] uint64_t spp_tx_count() const noexcept { return m_spp_tx_count.load(std::memory_order_relaxed); }
    [[nodiscard]] uint64_t mtc_full_frame_tx_count() const noexcept { return m_mtc_full_frame_tx_count.load(std::memory_order_relaxed); }
    [[nodiscard]] uint16_t last_tx_spp() const noexcept { return m_last_tx_spp.load(std::memory_order_relaxed); }
    [[nodiscard]] MtcTimecode last_tx_mtc() const noexcept {
        return MtcTimecode::unpack(m_last_tx_mtc_packed.load(std::memory_order_relaxed));
    }


    bool send_midi_clock_tick(uint64_t timestamp_ns = 0) noexcept {
        if (m_fd >= 0) {
            uint8_t b = 0xF8;
            (void)::write(m_fd, &b, 1);
        }
        if (m_seq_fd < 0 && !m_is_mock) return false;
        struct snd_seq_event ev{};
        ev.type = SNDRV_SEQ_EVENT_CLOCK;
        ev.source.client = static_cast<unsigned char>(m_seq_client_id);
        ev.source.port = static_cast<unsigned char>(m_seq_port_id);
        ev.dest.client = SNDRV_SEQ_ADDRESS_SUBSCRIBERS;
        ev.dest.port = SNDRV_SEQ_ADDRESS_UNKNOWN;
        ev.queue = SNDRV_SEQ_QUEUE_DIRECT;
        ev.flags = SNDRV_SEQ_TIME_STAMP_REAL;
        ev.time.time.tv_sec = static_cast<unsigned int>(timestamp_ns / 1'000'000'000ULL);
        ev.time.time.tv_nsec = static_cast<unsigned int>(timestamp_ns % 1'000'000'000ULL);
        return send_seq_event(ev);
    }

    bool send_midi_start() noexcept {
        if (m_fd >= 0) {
            uint8_t b = 0xFA;
            (void)::write(m_fd, &b, 1);
        }
        if (m_seq_fd < 0 && !m_is_mock) return false;
        struct snd_seq_event ev{};
        ev.type = SNDRV_SEQ_EVENT_START;
        ev.source.client = static_cast<unsigned char>(m_seq_client_id);
        ev.source.port = static_cast<unsigned char>(m_seq_port_id);
        ev.dest.client = SNDRV_SEQ_ADDRESS_SUBSCRIBERS;
        ev.dest.port = SNDRV_SEQ_ADDRESS_UNKNOWN;
        ev.queue = SNDRV_SEQ_QUEUE_DIRECT;
        return send_seq_event(ev);
    }

    bool send_midi_continue() noexcept {
        if (m_fd >= 0) {
            uint8_t b = 0xFB;
            (void)::write(m_fd, &b, 1);
        }
        if (m_seq_fd < 0 && !m_is_mock) return false;
        struct snd_seq_event ev{};
        ev.type = SNDRV_SEQ_EVENT_CONTINUE;
        ev.source.client = static_cast<unsigned char>(m_seq_client_id);
        ev.source.port = static_cast<unsigned char>(m_seq_port_id);
        ev.dest.client = SNDRV_SEQ_ADDRESS_SUBSCRIBERS;
        ev.dest.port = SNDRV_SEQ_ADDRESS_UNKNOWN;
        ev.queue = SNDRV_SEQ_QUEUE_DIRECT;
        return send_seq_event(ev);
    }

    bool send_midi_stop() noexcept {
        if (m_fd >= 0) {
            uint8_t b = 0xFC;
            (void)::write(m_fd, &b, 1);
        }
        if (m_seq_fd < 0 && !m_is_mock) return false;
        struct snd_seq_event ev{};
        ev.type = SNDRV_SEQ_EVENT_STOP;
        ev.source.client = static_cast<unsigned char>(m_seq_client_id);
        ev.source.port = static_cast<unsigned char>(m_seq_port_id);
        ev.dest.client = SNDRV_SEQ_ADDRESS_SUBSCRIBERS;
        ev.dest.port = SNDRV_SEQ_ADDRESS_UNKNOWN;
        ev.queue = SNDRV_SEQ_QUEUE_DIRECT;
        return send_seq_event(ev);
    }

    bool send_midi_spp(uint16_t spp_sixteenths) noexcept {
        if (m_fd >= 0) {
            uint8_t raw[3] = { 0xF2, static_cast<uint8_t>(spp_sixteenths & 0x7F), static_cast<uint8_t>((spp_sixteenths >> 7) & 0x7F) };
            (void)::write(m_fd, raw, 3);
        }
        if (m_seq_fd < 0 && !m_is_mock) return false;
        struct snd_seq_event ev{};
        ev.type = SNDRV_SEQ_EVENT_SONGPOS;
        ev.source.client = static_cast<unsigned char>(m_seq_client_id);
        ev.source.port = static_cast<unsigned char>(m_seq_port_id);
        ev.dest.client = SNDRV_SEQ_ADDRESS_SUBSCRIBERS;
        ev.dest.port = SNDRV_SEQ_ADDRESS_UNKNOWN;
        ev.queue = SNDRV_SEQ_QUEUE_DIRECT;
        ev.data.control.value = spp_sixteenths;
        return send_seq_event(ev);
    }

    bool send_mtc_qframe(uint8_t qframe_byte) noexcept {
        if (m_fd >= 0) {
            uint8_t raw[2] = { 0xF1, qframe_byte };
            (void)::write(m_fd, raw, 2);
        }
        if (m_seq_fd < 0 && !m_is_mock) return false;
        struct snd_seq_event ev{};
        ev.type = SNDRV_SEQ_EVENT_QFRAME;
        ev.source.client = static_cast<unsigned char>(m_seq_client_id);
        ev.source.port = static_cast<unsigned char>(m_seq_port_id);
        ev.dest.client = SNDRV_SEQ_ADDRESS_SUBSCRIBERS;
        ev.dest.port = SNDRV_SEQ_ADDRESS_UNKNOWN;
        ev.queue = SNDRV_SEQ_QUEUE_DIRECT;
        ev.data.control.value = qframe_byte;
        return send_seq_event(ev);
    }

    bool send_mtc_full_frame(uint8_t hr, uint8_t mn, uint8_t sc, uint8_t fr, MtcFrameRate rate) noexcept {
        auto full_frame = MidiClockGenerator::make_mtc_full_frame(hr, mn, sc, fr, rate);
        if (m_is_mock) {
            m_sync_tracker.on_mtc_full_frame(hr, mn, sc, fr, rate);
            return true;
        }
        bool ok = false;
        if (m_fd >= 0) {
            ssize_t w = ::write(m_fd, full_frame.data(), full_frame.size());
            if (w == static_cast<ssize_t>(full_frame.size())) ok = true;
        }
        if (m_seq_fd >= 0) {
            struct snd_seq_event ev{};
            ev.type = SNDRV_SEQ_EVENT_SYSEX;
            ev.flags = SNDRV_SEQ_EVENT_LENGTH_VARIABLE;
            ev.source.client = static_cast<unsigned char>(m_seq_client_id);
            ev.source.port = static_cast<unsigned char>(m_seq_port_id);
            ev.dest.client = SNDRV_SEQ_ADDRESS_SUBSCRIBERS;
            ev.dest.port = SNDRV_SEQ_ADDRESS_UNKNOWN;
            ev.queue = SNDRV_SEQ_QUEUE_DIRECT;
            ev.data.ext.ptr = full_frame.data();
            ev.data.ext.len = static_cast<unsigned int>(full_frame.size());
            if (send_seq_event(ev)) ok = true;
        }
        return ok;
    }

    bool send_mtc_full_frame(const MtcTimecode& tc) noexcept {
        return send_mtc_full_frame(tc.hours, tc.minutes, tc.seconds, tc.frames, tc.rate);
    }

    // ========================================================================
    // Virtual / Mock Raw MIDI Ingestion (Testing & Offline Staging)
    // ========================================================================
    void inject_raw_bytes(const uint8_t* data, size_t length) noexcept {
        if (!data || length == 0) return;
        for (size_t i = 0; i < length; ++i) {
            parse_byte(data[i]);
        }
    }

    void inject_event(const MidiEvent& ev) noexcept {
        if (m_queue.try_push(ev)) {
            m_event_count.fetch_add(1, std::memory_order_relaxed);
            m_last_status.store(ev.status, std::memory_order_relaxed);
            m_last_note.store(ev.data1, std::memory_order_relaxed);
            m_last_velocity.store(ev.data2, std::memory_order_relaxed);
            m_activity_flag.store(true, std::memory_order_relaxed);
        } else {
            m_dropped_events.fetch_add(1, std::memory_order_relaxed);
        }
    }

    // ========================================================================
    // Lock-Free SPSC Drain Methods (Audio Thread or Main Event Loop)
    // ========================================================================
    size_t drain_to(modulation::PolyphonicSynth& synth) noexcept {
        MidiEvent ev{};
        size_t count = 0;
        while (m_queue.try_pop(ev)) {
            synth.handle_midi_event(ev);
            ++count;
        }
        return count;
    }

    size_t drain_to(modulation::ModulationMatrix& matrix) noexcept {
        MidiEvent ev{};
        size_t count = 0;
        while (m_queue.try_pop(ev)) {
            switch (ev.type()) {
                case MidiStatus::NoteOn:
                    if (ev.velocity() > 0) {
                        matrix.poly_note_on(ev.note(), static_cast<float>(ev.velocity()) / 127.0f);
                    } else {
                        matrix.poly_note_off(ev.note());
                    }
                    break;
                case MidiStatus::NoteOff:
                    matrix.poly_note_off(ev.note());
                    break;
                case MidiStatus::ControlChange:
                    if (ev.data1 == 1) { // Mod Wheel
                        matrix.set_performance_controls(matrix.velocity(), matrix.key_track(), static_cast<float>(ev.data2) / 127.0f, matrix.pitch_bend());
                    } else if (ev.data1 == 64) { // Sustain Pedal
                        matrix.poly_synth().set_sustain_pedal(ev.data2 >= 64);
                    } else if (ev.data1 == 120 || ev.data1 == 123) { // All sound / notes off
                        matrix.poly_all_notes_off();
                    }
                    break;
                case MidiStatus::PitchBend: {
                    int pb = (static_cast<int>(ev.data2) << 7) | static_cast<int>(ev.data1);
                    float norm_pb = (static_cast<float>(pb) - 8192.0f) / 8192.0f;
                    matrix.set_performance_controls(matrix.velocity(), matrix.key_track(), matrix.mod_wheel(), norm_pb);
                    matrix.poly_synth().set_pitch_bend_norm(norm_pb);
                    break;
                }
                default:
                    break;
            }
            ++count;
        }
        return count;
    }

    size_t drain_to(MidiLearnRouter& router, MixerGraph& mixer, modulation::ModulationMatrix* matrix = nullptr) noexcept {
        MidiEvent ev{};
        size_t count = 0;
        while (m_queue.try_pop(ev)) {
            // 1. Dispatch CC to MidiLearnRouter (updates parameters or handles learn capture)
            (void)router.process_midi_event(ev, mixer, matrix);

            // 2. Dispatch performance and note events to modulation matrix / synth
            if (matrix) {
                switch (ev.type()) {
                    case MidiStatus::NoteOn:
                        if (ev.velocity() > 0) {
                            matrix->poly_note_on(ev.note(), static_cast<float>(ev.velocity()) / 127.0f);
                        } else {
                            matrix->poly_note_off(ev.note());
                        }
                        break;
                    case MidiStatus::NoteOff:
                        matrix->poly_note_off(ev.note());
                        break;
                    case MidiStatus::ControlChange:
                        if (ev.data1 == 1) { // Mod Wheel
                            matrix->set_performance_controls(matrix->velocity(), matrix->key_track(), static_cast<float>(ev.data2) / 127.0f, matrix->pitch_bend());
                        } else if (ev.data1 == 64) { // Sustain Pedal
                            matrix->poly_synth().set_sustain_pedal(ev.data2 >= 64);
                        } else if (ev.data1 == 120 || ev.data1 == 123) { // All sound / notes off
                            matrix->poly_all_notes_off();
                        }
                        break;
                    case MidiStatus::PitchBend: {
                        int pb = (static_cast<int>(ev.data2) << 7) | static_cast<int>(ev.data1);
                        float norm_pb = (static_cast<float>(pb) - 8192.0f) / 8192.0f;
                        matrix->set_performance_controls(matrix->velocity(), matrix->key_track(), matrix->mod_wheel(), norm_pb);
                        matrix->poly_synth().set_pitch_bend_norm(norm_pb);
                        break;
                    }
                    default:
                        break;
                }
            }
            ++count;
        }
        return count;
    }

    size_t drain_to(std::vector<MidiEvent>& out_events) {
        MidiEvent ev{};
        size_t count = 0;
        while (m_queue.try_pop(ev)) {
            out_events.push_back(ev);
            ++count;
        }
        return count;
    }

    // ========================================================================
    // Live Telemetry Queries
    // ========================================================================
    [[nodiscard]] uint64_t event_count() const noexcept { return m_event_count.load(std::memory_order_relaxed); }
    [[nodiscard]] uint64_t dropped_events() const noexcept { return m_dropped_events.load(std::memory_order_relaxed); }
    [[nodiscard]] uint8_t last_status() const noexcept { return m_last_status.load(std::memory_order_relaxed); }
    [[nodiscard]] uint8_t last_note() const noexcept { return m_last_note.load(std::memory_order_relaxed); }
    [[nodiscard]] uint8_t last_velocity() const noexcept { return m_last_velocity.load(std::memory_order_relaxed); }

    [[nodiscard]] size_t queue_capacity() const noexcept { return m_queue.capacity(); }
    [[nodiscard]] size_t queue_size() const noexcept { return m_queue.size(); }

    void reset_counters() noexcept {
        m_event_count.store(0, std::memory_order_relaxed);
        m_dropped_events.store(0, std::memory_order_relaxed);
    }

    [[nodiscard]] bool has_activity_and_clear() noexcept {
        return m_activity_flag.exchange(false, std::memory_order_relaxed);
    }

private:
    void ensure_listener_thread_running() {
        if (!m_running.load(std::memory_order_relaxed)) {
            if (m_stop_pipe[0] < 0 || m_stop_pipe[1] < 0) {
                if (pipe2(m_stop_pipe, O_CLOEXEC) < 0) {
                    return;
                }
            }
            m_running.store(true, std::memory_order_release);
            m_thread = std::thread(&HardwareMidiReceiver::read_loop, this);
        }
    }

    void update_device_path_string() {
        std::string seq_desc;
        if (m_seq_fd >= 0) {
            seq_desc = "ALSA Seq [" + m_seq_port_name + "]";
            size_t sub_count = 0;
            {
                std::lock_guard<std::recursive_mutex> lock(m_subs_mutex);
                for (const auto& s : m_subscriptions) {
                    if (!s.is_system_announce) ++sub_count;
                }
            }
            if (sub_count > 0) {
                seq_desc += " (" + std::to_string(sub_count) + " subs)";
            }
        }

        if (!m_rawmidi_device_path.empty() && !seq_desc.empty()) {
            m_current_device_path = m_rawmidi_device_path + " + " + seq_desc;
        } else if (!m_rawmidi_device_path.empty()) {
            m_current_device_path = m_rawmidi_device_path;
        } else if (!seq_desc.empty()) {
            m_current_device_path = seq_desc;
        } else if (m_is_mock) {
            size_t sub_count = 0;
            {
                std::lock_guard<std::recursive_mutex> lock(m_subs_mutex);
                for (const auto& s : m_subscriptions) {
                    if (!s.is_system_announce) ++sub_count;
                }
            }
            if (sub_count > 0) {
                m_current_device_path = "Virtual / Mock MIDI Receiver (" + std::to_string(sub_count) + " subs)";
            } else {
                m_current_device_path = "Virtual / Mock MIDI Receiver";
            }
        } else {
            m_current_device_path.clear();
        }
    }

    void read_loop() {
        struct pollfd fds[3];
        uint8_t buffer[256];

        while (m_running.load(std::memory_order_relaxed)) {
            int nfds = 0;
            int raw_idx = -1;
            int seq_idx = -1;
            int stop_idx = -1;

            if (m_fd >= 0) {
                raw_idx = nfds;
                fds[nfds].fd = m_fd;
                fds[nfds].events = POLLIN | POLLERR | POLLHUP;
                nfds++;
            }
            if (m_seq_fd >= 0) {
                seq_idx = nfds;
                fds[nfds].fd = m_seq_fd;
                fds[nfds].events = POLLIN | POLLERR | POLLHUP;
                nfds++;
            }
            if (m_stop_pipe[0] >= 0) {
                stop_idx = nfds;
                fds[nfds].fd = m_stop_pipe[0];
                fds[nfds].events = POLLIN;
                nfds++;
            }

            if (nfds == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            int ret = poll(fds, nfds, 100);
            if (ret < 0) {
                if (errno == EINTR) continue;
                break;
            }

            if (stop_idx >= 0 && (fds[stop_idx].revents & POLLIN)) {
                break; // Stop signal received
            }

            // RawMIDI input
            if (raw_idx >= 0 && (fds[raw_idx].revents & POLLIN)) {
                ssize_t bytes_read = ::read(m_fd, buffer, sizeof(buffer));
                if (bytes_read > 0) {
                    for (ssize_t i = 0; i < bytes_read; ++i) {
                        parse_byte(buffer[i]);
                    }
                } else if (bytes_read < 0 && (errno != EAGAIN && errno != EWOULDBLOCK)) {
                    ::close(m_fd);
                    m_fd = -1;
                    m_rawmidi_device_path.clear();
                    update_device_path_string();
                }
            } else if (raw_idx >= 0 && (fds[raw_idx].revents & (POLLERR | POLLHUP))) {
                ::close(m_fd);
                m_fd = -1;
                m_rawmidi_device_path.clear();
                update_device_path_string();
            }

            // ALSA Sequencer input
            if (seq_idx >= 0 && (fds[seq_idx].revents & POLLIN)) {
                struct snd_seq_event seq_batch[64];
                while (true) {
                    ssize_t bytes_read = ::read(m_seq_fd, seq_batch, sizeof(seq_batch));
                    if (bytes_read > 0) {
                        size_t num_events = static_cast<size_t>(bytes_read) / sizeof(struct snd_seq_event);
                        for (size_t i = 0; i < num_events; ++i) {
                            parse_seq_event(seq_batch[i]);
                        }
                    } else {
                        break;
                    }
                }
            } else if (seq_idx >= 0 && (fds[seq_idx].revents & (POLLERR | POLLHUP))) {
                ::close(m_seq_fd);
                m_seq_fd = -1;
                m_seq_client_id = -1;
                m_seq_port_id = -1;
                m_seq_port_name.clear();
                update_device_path_string();
            }
        }
    }

    void remove_subscription_record(int client, int port) noexcept {
        try {
            std::lock_guard<std::recursive_mutex> lock(m_subs_mutex);
            auto it = std::remove_if(m_subscriptions.begin(), m_subscriptions.end(),
                [client, port](const AlsaSeqSubscription& s) {
                    return s.client_id == client && s.port_id == port;
                });
            if (it != m_subscriptions.end()) {
                m_subscriptions.erase(it, m_subscriptions.end());
                update_device_path_string();
            }
        } catch (...) {}
    }

    void remove_client_subscriptions(int client) noexcept {
        try {
            std::lock_guard<std::recursive_mutex> lock(m_subs_mutex);
            auto it = std::remove_if(m_subscriptions.begin(), m_subscriptions.end(),
                [client](const AlsaSeqSubscription& s) {
                    return s.client_id == client;
                });
            if (it != m_subscriptions.end()) {
                m_subscriptions.erase(it, m_subscriptions.end());
                update_device_path_string();
            }
        } catch (...) {}
    }

    void parse_seq_event(const struct snd_seq_event& ev) noexcept {
        try {
            switch (ev.type) {
                case SNDRV_SEQ_EVENT_NOTEON: {
                    uint8_t ch = ev.data.note.channel & 0x0F;
                    uint8_t note = ev.data.note.note & 0x7F;
                    uint8_t vel = ev.data.note.velocity & 0x7F;
                    dispatch_parsed_message(0x90 | ch, note, vel);
                    break;
                }
                case SNDRV_SEQ_EVENT_NOTEOFF: {
                    uint8_t ch = ev.data.note.channel & 0x0F;
                    uint8_t note = ev.data.note.note & 0x7F;
                    uint8_t vel = ev.data.note.velocity & 0x7F;
                    dispatch_parsed_message(0x80 | ch, note, vel);
                    break;
                }
                case SNDRV_SEQ_EVENT_NOTE: {
                    uint8_t ch = ev.data.note.channel & 0x0F;
                    uint8_t note = ev.data.note.note & 0x7F;
                    uint8_t vel = ev.data.note.velocity & 0x7F;
                    if (vel > 0) {
                        dispatch_parsed_message(0x90 | ch, note, vel);
                    } else {
                        dispatch_parsed_message(0x80 | ch, note, 0);
                    }
                    break;
                }
                case SNDRV_SEQ_EVENT_KEYPRESS: {
                    uint8_t ch = ev.data.note.channel & 0x0F;
                    uint8_t note = ev.data.note.note & 0x7F;
                    uint8_t vel = ev.data.note.velocity & 0x7F;
                    dispatch_parsed_message(0xA0 | ch, note, vel);
                    break;
                }
                case SNDRV_SEQ_EVENT_CONTROLLER: {
                    uint8_t ch = ev.data.control.channel & 0x0F;
                    uint8_t param = ev.data.control.param & 0x7F;
                    uint8_t val = static_cast<uint8_t>(std::clamp(ev.data.control.value, 0, 127));
                    dispatch_parsed_message(0xB0 | ch, param, val);
                    break;
                }
                case SNDRV_SEQ_EVENT_PGMCHANGE: {
                    uint8_t ch = ev.data.control.channel & 0x0F;
                    uint8_t val = static_cast<uint8_t>(std::clamp(ev.data.control.value, 0, 127));
                    dispatch_parsed_message(0xC0 | ch, val, 0);
                    break;
                }
                case SNDRV_SEQ_EVENT_CHANPRESS: {
                    uint8_t ch = ev.data.control.channel & 0x0F;
                    uint8_t val = static_cast<uint8_t>(std::clamp(ev.data.control.value, 0, 127));
                    dispatch_parsed_message(0xD0 | ch, val, 0);
                    break;
                }
                case SNDRV_SEQ_EVENT_PITCHBEND: {
                    uint8_t ch = ev.data.control.channel & 0x0F;
                    int raw_val = std::clamp(static_cast<int>(ev.data.control.value) + 8192, 0, 16383);
                    uint8_t d1 = static_cast<uint8_t>(raw_val & 0x7F);
                    uint8_t d2 = static_cast<uint8_t>((raw_val >> 7) & 0x7F);
                    dispatch_parsed_message(0xE0 | ch, d1, d2);
                    break;
                }
                // Realtime Sync Events (Beat Clock & MTC)
                case SNDRV_SEQ_EVENT_CLOCK: { // 36
                    uint64_t ts = static_cast<uint64_t>(ev.time.time.tv_sec) * 1'000'000'000ULL + ev.time.time.tv_nsec;
                    m_sync_tracker.on_clock_tick(ts);
                    m_event_count.fetch_add(1, std::memory_order_relaxed);
                    m_activity_flag.store(true, std::memory_order_relaxed);
                    break;
                }
                case SNDRV_SEQ_EVENT_START: { // 30
                    m_sync_tracker.on_start();
                    m_event_count.fetch_add(1, std::memory_order_relaxed);
                    m_activity_flag.store(true, std::memory_order_relaxed);
                    break;
                }
                case SNDRV_SEQ_EVENT_CONTINUE: { // 31
                    m_sync_tracker.on_continue();
                    m_event_count.fetch_add(1, std::memory_order_relaxed);
                    m_activity_flag.store(true, std::memory_order_relaxed);
                    break;
                }
                case SNDRV_SEQ_EVENT_STOP: { // 32
                    m_sync_tracker.on_stop();
                    m_event_count.fetch_add(1, std::memory_order_relaxed);
                    m_activity_flag.store(true, std::memory_order_relaxed);
                    break;
                }
                case SNDRV_SEQ_EVENT_SONGPOS: { // 20
                    uint16_t spp = static_cast<uint16_t>(std::clamp(ev.data.control.value, 0, 16383));
                    m_sync_tracker.on_song_position_pointer(spp);
                    m_event_count.fetch_add(1, std::memory_order_relaxed);
                    m_activity_flag.store(true, std::memory_order_relaxed);
                    break;
                }
                case SNDRV_SEQ_EVENT_QFRAME: { // 22
                    uint8_t qf = static_cast<uint8_t>(ev.data.control.value & 0xFF);
                    m_sync_tracker.on_mtc_quarter_frame(qf);
                    m_event_count.fetch_add(1, std::memory_order_relaxed);
                    m_activity_flag.store(true, std::memory_order_relaxed);
                    break;
                }
                case SNDRV_SEQ_EVENT_SYSEX: { // 130
                    const uint8_t* ptr = static_cast<const uint8_t*>(ev.data.ext.ptr);
                    size_t len = ev.data.ext.len;
                    if (ptr && len >= 10) {
                        if (ptr[0] == 0xF0 && ptr[1] == 0x7F && ptr[2] == 0x7F &&
                            ptr[3] == 0x01 && ptr[4] == 0x01 && ptr[9] == 0xF7) {
                            uint8_t hr_rate = ptr[5];
                            uint8_t mn = ptr[6];
                            uint8_t sc = ptr[7];
                            uint8_t fr = ptr[8];
                            auto rate = static_cast<MtcFrameRate>((hr_rate >> 5) & 0x03);
                            uint8_t hr = hr_rate & 0x1F;
                            m_sync_tracker.on_mtc_full_frame(hr, mn, sc, fr, rate);
                        }
                    }
                    m_event_count.fetch_add(1, std::memory_order_relaxed);
                    m_activity_flag.store(true, std::memory_order_relaxed);
                    break;
                }
                // Dynamic hotplug & kernel announce events
                case SNDRV_SEQ_EVENT_PORT_START:
                case SNDRV_SEQ_EVENT_PORT_CHANGE: {
                    int client = static_cast<int>(ev.data.addr.client);
                    int port = static_cast<int>(ev.data.addr.port);
                    if (client != m_seq_client_id && client != 0) {
                        struct snd_seq_port_info pinfo{};
                        pinfo.addr.client = static_cast<unsigned char>(client);
                        pinfo.addr.port = static_cast<unsigned char>(port);
                        if (ioctl(m_seq_fd, SNDRV_SEQ_IOCTL_GET_PORT_INFO, &pinfo) >= 0) {
                            if (pinfo.capability & SNDRV_SEQ_PORT_CAP_SUBS_READ) {
                                subscribe_to(client, port);
                            }
                        }
                    }
                    break;
                }
                case SNDRV_SEQ_EVENT_PORT_EXIT: {
                    int client = static_cast<int>(ev.data.addr.client);
                    int port = static_cast<int>(ev.data.addr.port);
                    remove_subscription_record(client, port);
                    break;
                }
                case SNDRV_SEQ_EVENT_CLIENT_EXIT: {
                    int client = static_cast<int>(ev.data.addr.client);
                    remove_client_subscriptions(client);
                    break;
                }
                case SNDRV_SEQ_EVENT_PORT_SUBSCRIBED: {
                    if (static_cast<int>(ev.data.connect.dest.client) == m_seq_client_id &&
                        static_cast<int>(ev.data.connect.dest.port) == m_seq_port_id) {
                        subscribe_to(static_cast<int>(ev.data.connect.sender.client),
                                     static_cast<int>(ev.data.connect.sender.port));
                    }
                    break;
                }
                case SNDRV_SEQ_EVENT_PORT_UNSUBSCRIBED: {
                    if (static_cast<int>(ev.data.connect.dest.client) == m_seq_client_id &&
                        static_cast<int>(ev.data.connect.dest.port) == m_seq_port_id) {
                        remove_subscription_record(static_cast<int>(ev.data.connect.sender.client),
                                                   static_cast<int>(ev.data.connect.sender.port));
                    }
                    break;
                }
                default:
                    break;
            }
        } catch (...) {}
    }

    void parse_byte(uint8_t byte) noexcept {
        // 1. System Realtime messages (0xF8..0xFF) can appear ANYWHERE in the byte stream
        // without interrupting or corrupting running status for channel messages!
        if (byte >= 0xF8) {
            m_event_count.fetch_add(1, std::memory_order_relaxed);
            m_activity_flag.store(true, std::memory_order_relaxed);
            switch (byte) {
                case 0xF8: // Timing Clock (24 PPQN)
                    m_sync_tracker.on_clock_tick();
                    break;
                case 0xFA: // Start
                    m_sync_tracker.on_start();
                    break;
                case 0xFB: // Continue
                    m_sync_tracker.on_continue();
                    break;
                case 0xFC: // Stop
                    m_sync_tracker.on_stop();
                    break;
                default:
                    break;
            }
            return;
        }

        // 2. Status Byte (MSB == 1)
        if (byte & 0x80) {
            // System Common (0xF0..0xF7) cancels running status
            if (byte >= 0xF0) {
                m_running_status = 0;
                m_expected_bytes = 0;
                m_received_bytes = 0;

                if (byte == 0xF0) { // SysEx Begin
                    m_in_sysex = true;
                    m_sysex_len = 0;
                    m_syscommon_status = 0xF0;
                } else if (byte == 0xF7) { // SysEx End (EOX)
                    if (m_in_sysex && m_sysex_len == 8) {
                        // Check MTC Full Frame: 7F 7F 01 01 hr_rate mn sc fr
                        if (m_sysex_buf[0] == 0x7F && m_sysex_buf[1] == 0x7F &&
                            m_sysex_buf[2] == 0x01 && m_sysex_buf[3] == 0x01) {
                            uint8_t hr_rate = m_sysex_buf[4];
                            uint8_t mn = m_sysex_buf[5];
                            uint8_t sc = m_sysex_buf[6];
                            uint8_t fr = m_sysex_buf[7];
                            auto rate = static_cast<MtcFrameRate>((hr_rate >> 5) & 0x03);
                            uint8_t hr = hr_rate & 0x1F;
                            m_sync_tracker.on_mtc_full_frame(hr, mn, sc, fr, rate);
                        }
                    }
                    m_in_sysex = false;
                    m_sysex_len = 0;
                    m_syscommon_status = 0;
                } else if (byte == 0xF1) { // MTC Quarter Frame
                    m_in_sysex = false;
                    m_syscommon_status = 0xF1;
                    m_expected_bytes = 1;
                } else if (byte == 0xF2) { // Song Position Pointer (SPP)
                    m_in_sysex = false;
                    m_syscommon_status = 0xF2;
                    m_expected_bytes = 2;
                } else {
                    m_in_sysex = false;
                    m_syscommon_status = byte;
                    m_expected_bytes = (byte == 0xF3) ? 1 : 0;
                }
                return;
            }

            // Channel Voice Message (0x80..0xEF)
            m_in_sysex = false;
            m_syscommon_status = 0;
            m_running_status = byte;
            m_received_bytes = 0;
            uint8_t type = byte & 0xF0;
            if (type == 0xC0 || type == 0xD0) {
                m_expected_bytes = 1; // Program Change, Channel Pressure
            } else {
                m_expected_bytes = 2; // NoteOff, NoteOn, PolyAftertouch, CC, PitchBend
            }
            return;
        }

        // 3. Data Byte (MSB == 0)
        if (m_in_sysex) {
            if (m_sysex_len < sizeof(m_sysex_buf)) {
                m_sysex_buf[m_sysex_len++] = byte;
            }
            return;
        }

        if (m_syscommon_status == 0xF1) {
            m_sync_tracker.on_mtc_quarter_frame(byte);
            m_syscommon_status = 0;
            m_expected_bytes = 0;
            m_received_bytes = 0;
            return;
        } else if (m_syscommon_status == 0xF2) {
            if (m_received_bytes == 0) {
                m_data1 = byte; // SPP LSB
                m_received_bytes = 1;
            } else {
                uint8_t spp_msb = byte;
                uint16_t spp = static_cast<uint16_t>(m_data1) | (static_cast<uint16_t>(spp_msb) << 7);
                m_sync_tracker.on_song_position_pointer(spp);
                m_syscommon_status = 0;
                m_expected_bytes = 0;
                m_received_bytes = 0;
            }
            return;
        }

        if (m_running_status == 0) return; // Stray data byte without active status

        if (m_received_bytes == 0) {
            m_data1 = byte;
            m_received_bytes = 1;
            if (m_expected_bytes == 1) {
                dispatch_parsed_message(m_running_status, m_data1, 0);
                m_received_bytes = 0;
            }
        } else if (m_received_bytes == 1 && m_expected_bytes == 2) {
            m_data2 = byte;
            dispatch_parsed_message(m_running_status, m_data1, m_data2);
            m_received_bytes = 0;
        }
    }

    void dispatch_parsed_message(uint8_t status, uint8_t d1, uint8_t d2) noexcept {
        // Normalize Note On with velocity 0 to Note Off
        if ((status & 0xF0) == 0x90 && d2 == 0) {
            status = 0x80 | (status & 0x0F);
        }

        MidiEvent ev{
            .frame_offset = 0,
            .status = status,
            .data1 = d1,
            .data2 = d2
        };

        if (m_queue.try_push(ev)) {
            m_event_count.fetch_add(1, std::memory_order_relaxed);
            m_last_status.store(status, std::memory_order_relaxed);
            m_last_note.store(d1, std::memory_order_relaxed);
            m_last_velocity.store(d2, std::memory_order_relaxed);
            m_activity_flag.store(true, std::memory_order_relaxed);
        } else {
            m_dropped_events.fetch_add(1, std::memory_order_relaxed);
        }
    }

    int m_fd{-1};
    int m_seq_fd{-1};
    int m_seq_client_id{-1};
    int m_seq_port_id{-1};
    std::string m_seq_port_name;
    std::string m_rawmidi_device_path;

    int m_stop_pipe[2]{-1, -1};
    std::string m_current_device_path;
    bool m_is_mock{false};

    std::atomic<bool> m_running{false};
    std::thread m_thread;

    RingBuffer<MidiEvent> m_queue;

    // Running-status parser state
    uint8_t m_running_status{0};
    uint8_t m_syscommon_status{0};
    uint8_t m_data1{0};
    uint8_t m_data2{0};
    uint8_t m_expected_bytes{0};
    uint8_t m_received_bytes{0};

    // Live atomic telemetry
    std::atomic<uint64_t> m_event_count{0};
    std::atomic<uint64_t> m_dropped_events{0};
    std::atomic<uint8_t> m_last_status{0};
    std::atomic<uint8_t> m_last_note{0};
    std::atomic<uint8_t> m_last_velocity{0};
    std::atomic<bool> m_activity_flag{false};

    // ALSA Sequencer Subscriptions
    mutable std::recursive_mutex m_subs_mutex;
    std::vector<AlsaSeqSubscription> m_subscriptions;

    // MIDI Clock & MTC Synchronization Tracker & Master Generator
    MidiSyncTracker m_sync_tracker;
    MidiClockGenerator m_clock_gen;

    // Master Clock & Locator Outgoing Telemetry
    std::atomic<uint64_t> m_spp_tx_count{0};
    std::atomic<uint64_t> m_mtc_full_frame_tx_count{0};
    std::atomic<uint16_t> m_last_tx_spp{0};
    std::atomic<uint64_t> m_last_tx_mtc_packed{0};
    std::atomic<uint64_t> m_last_handled_seek_gen{0};


    // SysEx Parser State
    bool m_in_sysex{false};
    uint8_t m_sysex_buf[16]{};
    size_t m_sysex_len{0};
};

} // namespace audio_core::midi
