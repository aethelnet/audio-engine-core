#pragma once

#include "audio_core/protocol/aoip_packet.hpp"
#include <cstdint>
#include <vector>
#include <string>
#include <atomic>
#include <chrono>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace audio_core::network {

// ============================================================================
// AoipTransmitter: Low-Latency UDP Audio-over-IP Stream Transmitter
// Broadcasts or streams audio frames (Stereo or Multi-channel) across the network
// ============================================================================
class AoipTransmitter {
public:
    AoipTransmitter() = default;

    ~AoipTransmitter() {
        close();
    }

    bool open(const std::string& destination_ip, uint16_t destination_port) {
        close();

        m_sockfd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (m_sockfd < 0) {
            return false;
        }

        // Enable broadcast in case destination is 255.255.255.255 or subnet broadcast
        int broadcast_opt = 1;
        ::setsockopt(m_sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast_opt, sizeof(broadcast_opt));

        // Enlarge send buffer (512 KB)
        int sndbuf = 512 * 1024;
        ::setsockopt(m_sockfd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

        std::memset(&m_dest_addr, 0, sizeof(m_dest_addr));
        m_dest_addr.sin_family = AF_INET;
        m_dest_addr.sin_port = htons(destination_port);

        if (::inet_pton(AF_INET, destination_ip.c_str(), &m_dest_addr.sin_addr) <= 0) {
            close();
            return false;
        }

        m_destination_ip = destination_ip;
        m_destination_port = destination_port;
        m_packet_buffer.resize(65536);
        return true;
    }

    void close() noexcept {
        if (m_sockfd >= 0) {
            ::close(m_sockfd);
            m_sockfd = -1;
        }
    }

    [[nodiscard]] bool is_open() const noexcept { return m_sockfd >= 0; }

    bool send_stereo(const float* left, const float* right, uint16_t frames, uint32_t sample_rate = 48000, bool planar = true) noexcept {
        const float* channels[2] = {left, right};
        return send_multichannel(channels, 2, frames, sample_rate, planar);
    }

    bool send_multichannel(const float* const* channel_ptrs, uint16_t channels, uint16_t frames, uint32_t sample_rate = 48000, bool planar = true) noexcept {
        if (m_sockfd < 0 || channels == 0 || frames == 0) return false;

        const size_t pcm_bytes = static_cast<size_t>(channels) * frames * sizeof(float);
        const size_t total_packet_bytes = sizeof(protocol::AoipHeader) + pcm_bytes;

        if (total_packet_bytes > m_packet_buffer.size()) {
            return false;
        }

        auto* hdr = reinterpret_cast<protocol::AoipHeader*>(m_packet_buffer.data());
        hdr->magic = protocol::kAoipMagic;
        hdr->version = protocol::kAoipVersion;
        hdr->payload_format = static_cast<uint16_t>(planar ? protocol::AoipPayloadFormat::Float32_Planar
                                                           : protocol::AoipPayloadFormat::Float32_Interleaved);
        hdr->sequence_number = m_sequence.fetch_add(1, std::memory_order_relaxed);

        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        hdr->timestamp_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
        hdr->sample_rate = sample_rate;
        hdr->channels = channels;
        hdr->frames = frames;

        float* dst = reinterpret_cast<float*>(m_packet_buffer.data() + sizeof(protocol::AoipHeader));

        if (planar) {
            for (uint16_t ch = 0; ch < channels; ++ch) {
                const float* src = channel_ptrs[ch];
                float* ch_dst = dst + (ch * frames);
                if (src) {
                    std::memcpy(ch_dst, src, frames * sizeof(float));
                } else {
                    std::memset(ch_dst, 0, frames * sizeof(float));
                }
            }
        } else {
            // Interleaved packing
            for (uint16_t f = 0; f < frames; ++f) {
                for (uint16_t ch = 0; ch < channels; ++ch) {
                    const float* src = channel_ptrs[ch];
                    *dst++ = src ? src[f] : 0.0f;
                }
            }
        }

        ssize_t sent = ::sendto(m_sockfd, m_packet_buffer.data(), total_packet_bytes, 0,
                                reinterpret_cast<const sockaddr*>(&m_dest_addr), sizeof(m_dest_addr));

        if (sent == static_cast<ssize_t>(total_packet_bytes)) {
            m_packets_sent.fetch_add(1, std::memory_order_relaxed);
            m_bytes_sent.fetch_add(sent, std::memory_order_relaxed);
            return true;
        }

        return false;
    }

    [[nodiscard]] uint64_t packets_sent() const noexcept { return m_packets_sent.load(std::memory_order_relaxed); }
    [[nodiscard]] uint64_t bytes_sent() const noexcept { return m_bytes_sent.load(std::memory_order_relaxed); }

private:
    int m_sockfd{-1};
    std::string m_destination_ip{"127.0.0.1"};
    uint16_t m_destination_port{4848};
    sockaddr_in m_dest_addr{};

    std::atomic<uint64_t> m_sequence{0};
    std::atomic<uint64_t> m_packets_sent{0};
    std::atomic<uint64_t> m_bytes_sent{0};

    std::vector<uint8_t> m_packet_buffer;
};

} // namespace audio_core::network
