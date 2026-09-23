#pragma once

#include "audio_core/types.hpp"
#include "audio_core/ring_buffer.hpp"
#include "audio_core/modulation/polyphonic_synth.hpp"
#include "audio_core/modulation/modulation_matrix.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sound/asound.h>
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

namespace audio_core::midi {

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

    HardwareMidiReceiver()
        : m_queue(kDefaultQueueCapacity) {
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
    bool open_device(const std::string& path) {
        close_device();

        m_fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (m_fd < 0) {
            return false;
        }

        if (pipe2(m_stop_pipe, O_CLOEXEC) < 0) {
            ::close(m_fd);
            m_fd = -1;
            return false;
        }

        m_current_device_path = path;
        m_is_mock = false;
        m_running.store(true, std::memory_order_release);
        m_thread = std::thread(&HardwareMidiReceiver::read_loop, this);
        return true;
    }

    bool auto_connect() {
        auto devs = enumerate_devices();
        for (const auto& dev : devs) {
            if (dev.is_available) {
                if (open_device(dev.path)) {
                    std::cout << "[HardwareMidi] Connected to: " << dev.name << " (" << dev.path << ")" << std::endl;
                    return true;
                }
            }
        }

        // Fallback: Virtual / Mock mode for headless testing & machines without plugged USB MIDI
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

        if (m_stop_pipe[0] >= 0) { ::close(m_stop_pipe[0]); m_stop_pipe[0] = -1; }
        if (m_stop_pipe[1] >= 0) { ::close(m_stop_pipe[1]); m_stop_pipe[1] = -1; }
        if (m_fd >= 0) { ::close(m_fd); m_fd = -1; }

        m_current_device_path.clear();
        m_is_mock = false;
        m_running_status = 0;
        m_expected_bytes = 0;
        m_received_bytes = 0;
    }

    [[nodiscard]] bool is_connected() const noexcept {
        return (m_fd >= 0 && m_running.load(std::memory_order_relaxed)) || m_is_mock;
    }

    [[nodiscard]] bool is_mock() const noexcept { return m_is_mock; }
    [[nodiscard]] const std::string& current_device_path() const noexcept { return m_current_device_path; }

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
        m_queue.try_push(ev);
        m_event_count.fetch_add(1, std::memory_order_relaxed);
        m_last_status.store(ev.status, std::memory_order_relaxed);
        m_last_note.store(ev.data1, std::memory_order_relaxed);
        m_last_velocity.store(ev.data2, std::memory_order_relaxed);
        m_activity_flag.store(true, std::memory_order_relaxed);
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
    [[nodiscard]] uint8_t last_status() const noexcept { return m_last_status.load(std::memory_order_relaxed); }
    [[nodiscard]] uint8_t last_note() const noexcept { return m_last_note.load(std::memory_order_relaxed); }
    [[nodiscard]] uint8_t last_velocity() const noexcept { return m_last_velocity.load(std::memory_order_relaxed); }

    [[nodiscard]] bool has_activity_and_clear() noexcept {
        return m_activity_flag.exchange(false, std::memory_order_relaxed);
    }

private:
    void read_loop() {
        struct pollfd fds[2];
        fds[0].fd = m_fd;
        fds[0].events = POLLIN | POLLERR | POLLHUP;
        fds[1].fd = m_stop_pipe[0];
        fds[1].events = POLLIN;

        uint8_t buffer[256];

        while (m_running.load(std::memory_order_relaxed)) {
            int ret = poll(fds, 2, 100);
            if (ret < 0) {
                if (errno == EINTR) continue;
                break;
            }

            if (fds[1].revents & POLLIN) {
                break; // Stop signal received
            }

            if (fds[0].revents & POLLIN) {
                ssize_t bytes_read = ::read(m_fd, buffer, sizeof(buffer));
                if (bytes_read > 0) {
                    for (ssize_t i = 0; i < bytes_read; ++i) {
                        parse_byte(buffer[i]);
                    }
                } else if (bytes_read < 0 && (errno != EAGAIN && errno != EWOULDBLOCK)) {
                    break; // Error or disconnection
                }
            } else if (fds[0].revents & (POLLERR | POLLHUP)) {
                break; // Device disconnected
            }
        }
    }

    void parse_byte(uint8_t byte) noexcept {
        // 1. Realtime messages (0xF8..0xFF) can appear anywhere without interrupting running status
        if (byte >= 0xF8) {
            return;
        }

        // 2. Status Byte (MSB == 1)
        if (byte & 0x80) {
            // System Common (0xF0..0xF7) cancels running status
            if (byte >= 0xF0) {
                m_running_status = 0;
                m_expected_bytes = 0;
                m_received_bytes = 0;
                return;
            }

            // Channel Voice Message (0x80..0xEF)
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

        m_queue.try_push(ev);
        m_event_count.fetch_add(1, std::memory_order_relaxed);
        m_last_status.store(status, std::memory_order_relaxed);
        m_last_note.store(d1, std::memory_order_relaxed);
        m_last_velocity.store(d2, std::memory_order_relaxed);
        m_activity_flag.store(true, std::memory_order_relaxed);
    }

    int m_fd{-1};
    int m_stop_pipe[2]{-1, -1};
    std::string m_current_device_path;
    bool m_is_mock{false};

    std::atomic<bool> m_running{false};
    std::thread m_thread;

    RingBuffer<MidiEvent> m_queue;

    // Running-status parser state
    uint8_t m_running_status{0};
    uint8_t m_data1{0};
    uint8_t m_data2{0};
    uint8_t m_expected_bytes{0};
    uint8_t m_received_bytes{0};

    // Live atomic telemetry
    std::atomic<uint64_t> m_event_count{0};
    std::atomic<uint8_t> m_last_status{0};
    std::atomic<uint8_t> m_last_note{0};
    std::atomic<uint8_t> m_last_velocity{0};
    std::atomic<bool> m_activity_flag{false};
};

} // namespace audio_core::midi
