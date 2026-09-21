#pragma once

#include "audio_core/protocol/aoip_packet.hpp"
#include "audio_core/network/ptp_hardware_engine.hpp"
#include "audio_core/ring_buffer.hpp"
#include <cstdint>
#include <vector>
#include <array>
#include <memory>
#include <atomic>
#include <thread>
#include <string>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

namespace audio_core::network {

struct AoipStreamStats {
    std::atomic<uint64_t> packets_received{0};
    std::atomic<uint64_t> frames_received{0};
    std::atomic<uint64_t> packets_dropped{0};
    std::atomic<uint64_t> out_of_order{0};
    std::atomic<uint64_t> last_sequence{0};
    std::atomic<uint64_t> last_timestamp_ns{0};
    std::atomic<uint32_t> sample_rate{48000};
    std::atomic<uint16_t> channels{2};
    std::atomic<int64_t>  jitter_ns{0};
    std::atomic<int64_t>  avg_jitter_ns{0};
    std::atomic<uint64_t> max_jitter_ns{0};
    std::atomic<uint8_t>  timestamp_source{static_cast<uint8_t>(PtpTimestampSource::UserspaceMonotonic)};
    std::atomic<bool>     hardware_locked{false};
};

struct TrackMapping {
    uint32_t target_track_id{0};
    uint16_t stream_ch_l{0};
    uint16_t stream_ch_r{1};
    bool active{false};
};

// ============================================================================
// AoipReceiver: Low-Latency UDP Audio-over-IP Stream Receiver
// Ingests real-time audio streams (Android, Remote Laptop, DAW) directly into MixerGraph
// ============================================================================
class AoipReceiver {
public:
    static constexpr size_t kMaxTrackMappings = 32;
    static constexpr size_t kJitterBufferCapacity = 16384; // ~340ms @ 48kHz
    static constexpr size_t kMaxUdpPayloadSize = 65536;

    explicit AoipReceiver(uint16_t port = 4848)
        : m_port(port) {
        for (size_t i = 0; i < kMaxTrackMappings; ++i) {
            m_jitter_buffers_l[i] = std::make_unique<RingBuffer<float>>(kJitterBufferCapacity);
            m_jitter_buffers_r[i] = std::make_unique<RingBuffer<float>>(kJitterBufferCapacity);
        }
    }

    ~AoipReceiver() {
        stop();
        close_socket();
    }

    bool bind_port(uint16_t port, const std::string& bind_ip = "0.0.0.0", const std::string& iface = "") {
        close_socket();
        m_port = port;

        m_sockfd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
        if (m_sockfd < 0) {
            return false;
        }

        int opt = 1;
        ::setsockopt(m_sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        // Enlarge kernel UDP receive buffer (512 KB) to prevent kernel drops under bursts
        int rcvbuf = 512 * 1024;
        ::setsockopt(m_sockfd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

        // Configure Tiered PTP Hardware / Kernel Timestamping Engine
        m_ptp_engine.configure_socket(m_sockfd, iface);

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(m_port);
        if (::inet_pton(AF_INET, bind_ip.c_str(), &addr.sin_addr) <= 0) {
            addr.sin_addr.s_addr = htonl(INADDR_ANY);
        }

        if (::bind(m_sockfd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) < 0) {
            close_socket();
            return false;
        }

        return true;
    }

    void map_channel_pair(uint32_t target_track_id, uint16_t stream_ch_l = 0, uint16_t stream_ch_r = 1) noexcept {
        for (auto& mapping : m_mappings) {
            if (mapping.active && mapping.target_track_id == target_track_id) {
                mapping.stream_ch_l = stream_ch_l;
                mapping.stream_ch_r = stream_ch_r;
                return;
            }
        }
        for (auto& mapping : m_mappings) {
            if (!mapping.active) {
                mapping.target_track_id = target_track_id;
                mapping.stream_ch_l = stream_ch_l;
                mapping.stream_ch_r = stream_ch_r;
                mapping.active = true;
                return;
            }
        }
    }

    void unmap_track(uint32_t target_track_id) noexcept {
        for (auto& mapping : m_mappings) {
            if (mapping.active && mapping.target_track_id == target_track_id) {
                mapping.active = false;
                return;
            }
        }
    }

    void start() {
        if (m_running.load(std::memory_order_relaxed)) return;
        if (m_sockfd < 0 && !bind_port(m_port)) return;

        m_running.store(true, std::memory_order_release);
        m_worker_thread = std::thread(&AoipReceiver::worker_loop, this);
    }

    void stop() {
        if (!m_running.load(std::memory_order_relaxed)) return;
        m_running.store(false, std::memory_order_release);
        if (m_worker_thread.joinable()) {
            m_worker_thread.join();
        }
    }

    // Synchronous poll: reads all currently pending UDP datagrams without worker thread
    uint32_t poll_available_packets() {
        if (m_sockfd < 0) return 0;
        uint32_t count = 0;
        std::vector<uint8_t> buffer(kMaxUdpPayloadSize);

        while (true) {
            PtpTimestampInfo ts_info{};
            ssize_t bytes = m_ptp_engine.recvmsg_with_timestamp(m_sockfd, buffer.data(), buffer.size(), nullptr, ts_info);
            if (bytes <= 0) break;
            if (process_datagram(buffer.data(), static_cast<size_t>(bytes), ts_info.rx_timestamp_ns)) {
                count++;
            }
        }
        return count;
    }

    // Process a raw packet buffer directly (useful for testing & in-memory feeds)
    bool process_datagram(const uint8_t* data, size_t size_bytes, uint64_t rx_timestamp_ns = 0) noexcept {
        if (!protocol::validate_aoip_packet(data, size_bytes)) {
            return false;
        }

        const auto* hdr = reinterpret_cast<const protocol::AoipHeader*>(data);
        const uint64_t seq = hdr->sequence_number;
        const uint64_t last_seq = m_stats.last_sequence.load(std::memory_order_relaxed);

        if (m_stats.packets_received.load(std::memory_order_relaxed) > 0) {
            if (seq > last_seq + 1) {
                m_stats.packets_dropped.fetch_add(seq - last_seq - 1, std::memory_order_relaxed);
            } else if (seq <= last_seq) {
                m_stats.out_of_order.fetch_add(1, std::memory_order_relaxed);
            }
        }
        m_stats.last_sequence.store(seq, std::memory_order_relaxed);
        m_stats.last_timestamp_ns.store(hdr->timestamp_ns, std::memory_order_relaxed);
        m_stats.sample_rate.store(hdr->sample_rate, std::memory_order_relaxed);
        m_stats.channels.store(hdr->channels, std::memory_order_relaxed);
        m_stats.packets_received.fetch_add(1, std::memory_order_relaxed);
        m_stats.frames_received.fetch_add(hdr->frames, std::memory_order_relaxed);

        // Update physical transit jitter via PtpSocketTimestampEngine
        if (rx_timestamp_ns > 0) {
            m_ptp_engine.record_packet_transit(rx_timestamp_ns, hdr->timestamp_ns);
            m_stats.jitter_ns.store(m_ptp_engine.current_jitter_ns(), std::memory_order_relaxed);
            m_stats.avg_jitter_ns.store(static_cast<int64_t>(m_ptp_engine.avg_jitter_ns()), std::memory_order_relaxed);
            m_stats.max_jitter_ns.store(m_ptp_engine.max_jitter_ns(), std::memory_order_relaxed);
            m_stats.timestamp_source.store(static_cast<uint8_t>(m_ptp_engine.source()), std::memory_order_relaxed);
            m_stats.hardware_locked.store(m_ptp_engine.is_hardware_locked(), std::memory_order_relaxed);
        }

        const float* pcm_data = reinterpret_cast<const float*>(data + sizeof(protocol::AoipHeader));
        const uint16_t channels = hdr->channels;
        const uint16_t frames = hdr->frames;
        const bool is_planar = (hdr->payload_format == static_cast<uint16_t>(protocol::AoipPayloadFormat::Float32_Planar));

        // Dispatch frames to mapped track jitter buffers
        for (size_t map_idx = 0; map_idx < kMaxTrackMappings; ++map_idx) {
            const auto& mapping = m_mappings[map_idx];
            if (!mapping.active) continue;

            auto& ring_l = *m_jitter_buffers_l[map_idx];
            auto& ring_r = *m_jitter_buffers_r[map_idx];

            const uint16_t ch_l = mapping.stream_ch_l;
            const uint16_t ch_r = mapping.stream_ch_r;

            if (is_planar) {
                const float* ptr_l = (ch_l < channels) ? (pcm_data + (ch_l * frames)) : nullptr;
                const float* ptr_r = (ch_r < channels) ? (pcm_data + (ch_r * frames)) : nullptr;

                for (uint16_t f = 0; f < frames; ++f) {
                    ring_l.try_push(ptr_l ? ptr_l[f] : 0.0f);
                    ring_r.try_push(ptr_r ? ptr_r[f] : 0.0f);
                }
            } else {
                // Interleaved (L0, R0, Ch2_0, ... L1, R1, ...)
                for (uint16_t f = 0; f < frames; ++f) {
                    size_t base = f * channels;
                    float val_l = (ch_l < channels) ? pcm_data[base + ch_l] : 0.0f;
                    float val_r = (ch_r < channels) ? pcm_data[base + ch_r] : 0.0f;
                    ring_l.try_push(val_l);
                    ring_r.try_push(val_r);
                }
            }
        }

        return true;
    }

    // Called from RT Audio Thread: Fetches buffered samples for a given track
    uint32_t read_track_frames(uint32_t target_track_id, float* dst_l, float* dst_r, uint32_t frames) noexcept {
        for (size_t map_idx = 0; map_idx < kMaxTrackMappings; ++map_idx) {
            const auto& mapping = m_mappings[map_idx];
            if (mapping.active && mapping.target_track_id == target_track_id) {
                auto& ring_l = *m_jitter_buffers_l[map_idx];
                auto& ring_r = *m_jitter_buffers_r[map_idx];

                uint32_t popped = 0;
                for (uint32_t f = 0; f < frames; ++f) {
                    float s_l = 0.0f, s_r = 0.0f;
                    if (ring_l.try_pop(s_l) && ring_r.try_pop(s_r)) {
                        dst_l[f] = s_l;
                        dst_r[f] = s_r;
                        popped++;
                    } else {
                        // Underrun: fill remaining frames with silence
                        dst_l[f] = 0.0f;
                        dst_r[f] = 0.0f;
                    }
                }
                return popped;
            }
        }

        // Unmapped: fill silence
        for (uint32_t f = 0; f < frames; ++f) {
            dst_l[f] = 0.0f;
            dst_r[f] = 0.0f;
        }
        return 0;
    }

    [[nodiscard]] const AoipStreamStats& stats() const noexcept { return m_stats; }
    [[nodiscard]] uint16_t port() const noexcept { return m_port; }

    [[nodiscard]] const PtpSocketTimestampEngine& ptp_engine() const noexcept { return m_ptp_engine; }
    [[nodiscard]] PtpSocketTimestampEngine& ptp_engine() noexcept { return m_ptp_engine; }
    [[nodiscard]] int64_t jitter_ns() const noexcept { return m_ptp_engine.current_jitter_ns(); }
    [[nodiscard]] double avg_jitter_ns() const noexcept { return m_ptp_engine.avg_jitter_ns(); }
    [[nodiscard]] uint64_t max_jitter_ns() const noexcept { return m_ptp_engine.max_jitter_ns(); }
    [[nodiscard]] bool is_hardware_ptp_locked() const noexcept { return m_ptp_engine.is_hardware_locked(); }
    [[nodiscard]] PtpTimestampSource timestamp_source() const noexcept { return m_ptp_engine.source(); }

private:
    void close_socket() noexcept {
        if (m_sockfd >= 0) {
            ::close(m_sockfd);
            m_sockfd = -1;
        }
    }

    void worker_loop() {
        std::vector<uint8_t> buffer(kMaxUdpPayloadSize);
        pollfd pfd{};
        pfd.fd = m_sockfd;
        pfd.events = POLLIN;

        while (m_running.load(std::memory_order_acquire)) {
            int ret = ::poll(&pfd, 1, 10); // 10ms poll timeout
            if (ret > 0 && (pfd.revents & POLLIN)) {
                while (true) {
                    PtpTimestampInfo ts_info{};
                    ssize_t bytes = m_ptp_engine.recvmsg_with_timestamp(m_sockfd, buffer.data(), buffer.size(), nullptr, ts_info);
                    if (bytes <= 0) break;
                    process_datagram(buffer.data(), static_cast<size_t>(bytes), ts_info.rx_timestamp_ns);
                }
            }
        }
    }

    uint16_t m_port{4848};
    int m_sockfd{-1};
    std::atomic<bool> m_running{false};
    std::thread m_worker_thread;

    PtpSocketTimestampEngine m_ptp_engine;
    AoipStreamStats m_stats;
    std::array<TrackMapping, kMaxTrackMappings> m_mappings{};
    std::array<std::unique_ptr<RingBuffer<float>>, kMaxTrackMappings> m_jitter_buffers_l;
    std::array<std::unique_ptr<RingBuffer<float>>, kMaxTrackMappings> m_jitter_buffers_r;
};

} // namespace audio_core::network
