#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/sockios.h>
#include <linux/net_tstamp.h>
#include <net/if.h>
#include <netinet/in.h>
#include <time.h>
#include <unistd.h>

namespace audio_core::network {

// ============================================================================
// PtpTimestampSource: Tiered Timestamp Resolution Origin
// ============================================================================
enum class PtpTimestampSource : uint8_t {
    HardwareNicPhy = 0,    // Physical NIC PHY/MAC hardware clock (SOF_TIMESTAMPING_RAW_HARDWARE)
    KernelDriverStack = 1, // Linux kernel network stack timestamp (SOF_TIMESTAMPING_RX_SOFTWARE / SO_TIMESTAMPNS)
    UserspaceMonotonic = 2 // Fallback userspace timestamp (clock_gettime CLOCK_MONOTONIC_RAW)
};

[[nodiscard]] inline const char* ptp_source_name(PtpTimestampSource src) noexcept {
    switch (src) {
        case PtpTimestampSource::HardwareNicPhy: return "Hardware NIC PHY";
        case PtpTimestampSource::KernelDriverStack: return "Kernel Driver Stack";
        case PtpTimestampSource::UserspaceMonotonic: return "Userspace Monotonic";
    }
    return "Unknown";
}

// ============================================================================
// PtpTimestampInfo: Live Datagram Timestamp & Jitter Metrics
// ============================================================================
struct PtpTimestampInfo {
    uint64_t rx_timestamp_ns{0};
    PtpTimestampSource source{PtpTimestampSource::UserspaceMonotonic};
    int64_t transit_jitter_ns{0};
    double avg_jitter_ns{0.0};
    uint64_t max_jitter_ns{0};
    bool hardware_locked{false};
};

// ============================================================================
// PtpSocketTimestampEngine: Tiered Hardware & Kernel Timestamping Engine
// Configures Linux socket timestamping and extracts sub-microsecond arrival times
// ============================================================================
class PtpSocketTimestampEngine {
public:
    PtpSocketTimestampEngine() = default;

    // Configure Linux socket for hardware/kernel timestamping
    bool configure_socket(int sockfd, const std::string& iface_name = "") noexcept {
        if (sockfd < 0) return false;

        m_hardware_capable = false;
        m_so_timestamping_active = false;

        // 1. Non-fatal attempt at physical NIC hardware timestamping configuration via SIOCSHWTSTAMP
        if (!iface_name.empty()) {
            struct hwtstamp_config hw_config{};
            hw_config.flags = 0;
            hw_config.tx_type = HWTSTAMP_TX_OFF;
            hw_config.rx_filter = HWTSTAMP_FILTER_ALL;

            struct ifreq ifr{};
            std::strncpy(ifr.ifr_name, iface_name.c_str(), sizeof(ifr.ifr_name) - 1);
            ifr.ifr_data = reinterpret_cast<char*>(&hw_config);

            if (::ioctl(sockfd, SIOCSHWTSTAMP, &ifr) == 0) {
                m_hardware_capable = true;
            }
        }

        // 2. Request SO_TIMESTAMPING with both hardware and kernel options
        int flags = SOF_TIMESTAMPING_RX_HARDWARE |
                    SOF_TIMESTAMPING_RAW_HARDWARE |
                    SOF_TIMESTAMPING_SYS_HARDWARE |
                    SOF_TIMESTAMPING_RX_SOFTWARE |
                    SOF_TIMESTAMPING_SOFTWARE;

        if (::setsockopt(sockfd, SOL_SOCKET, SO_TIMESTAMPING, &flags, sizeof(flags)) == 0) {
            m_so_timestamping_active = true;
        } else {
            // Fallback: Software/driver timestamping only
            flags = SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE;
            if (::setsockopt(sockfd, SOL_SOCKET, SO_TIMESTAMPING, &flags, sizeof(flags)) == 0) {
                m_so_timestamping_active = true;
            } else {
                // Fallback: SO_TIMESTAMPNS
                int one = 1;
                if (::setsockopt(sockfd, SOL_SOCKET, SO_TIMESTAMPNS, &one, sizeof(one)) == 0) {
                    m_so_timestamping_active = true;
                }
            }
        }

        return m_so_timestamping_active;
    }

    // Zero-allocation stack msghdr receive with hardware/kernel timestamp extraction
    ssize_t recvmsg_with_timestamp(int sockfd,
                                   uint8_t* out_buf,
                                   size_t capacity,
                                   sockaddr_in* src_addr,
                                   PtpTimestampInfo& out_ts) noexcept {
        if (sockfd < 0 || !out_buf || capacity == 0) return -1;

        struct iovec iov{};
        iov.iov_base = out_buf;
        iov.iov_len = capacity;

        alignas(alignof(struct cmsghdr)) char control_buf[512];

        struct msghdr msg{};
        msg.msg_name = src_addr;
        msg.msg_namelen = src_addr ? sizeof(sockaddr_in) : 0;
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_control = control_buf;
        msg.msg_controllen = sizeof(control_buf);

        ssize_t bytes = ::recvmsg(sockfd, &msg, MSG_DONTWAIT);
        if (bytes <= 0) return bytes;

        uint64_t rx_ns = 0;
        PtpTimestampSource src = PtpTimestampSource::UserspaceMonotonic;

        // Parse control message for hardware or kernel timestamps
        for (struct cmsghdr* cmsg = CMSG_FIRSTHDR(&msg); cmsg != nullptr; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
            if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMPING) {
                auto* ts = reinterpret_cast<struct timespec*>(CMSG_DATA(cmsg));
                // ts[2] is raw hardware NIC PHY timestamp
                if (ts[2].tv_sec != 0 || ts[2].tv_nsec != 0) {
                    rx_ns = static_cast<uint64_t>(ts[2].tv_sec) * 1'000'000'000ULL + static_cast<uint64_t>(ts[2].tv_nsec);
                    src = PtpTimestampSource::HardwareNicPhy;
                    break;
                }
                // ts[0] is software / kernel driver timestamp
                if (ts[0].tv_sec != 0 || ts[0].tv_nsec != 0) {
                    rx_ns = static_cast<uint64_t>(ts[0].tv_sec) * 1'000'000'000ULL + static_cast<uint64_t>(ts[0].tv_nsec);
                    src = PtpTimestampSource::KernelDriverStack;
                    break;
                }
            } else if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMPNS) {
                auto* ts = reinterpret_cast<struct timespec*>(CMSG_DATA(cmsg));
                rx_ns = static_cast<uint64_t>(ts->tv_sec) * 1'000'000'000ULL + static_cast<uint64_t>(ts->tv_nsec);
                src = PtpTimestampSource::KernelDriverStack;
                break;
            }
        }

        // Fallback to CLOCK_MONOTONIC_RAW if no kernel/hardware timestamp was populated
        if (rx_ns == 0) {
            struct timespec now{};
            ::clock_gettime(CLOCK_MONOTONIC_RAW, &now);
            rx_ns = static_cast<uint64_t>(now.tv_sec) * 1'000'000'000ULL + static_cast<uint64_t>(now.tv_nsec);
            src = PtpTimestampSource::UserspaceMonotonic;
        }

        out_ts.rx_timestamp_ns = rx_ns;
        out_ts.source = src;
        out_ts.hardware_locked = (src == PtpTimestampSource::HardwareNicPhy);

        m_rx_timestamp_ns.store(rx_ns, std::memory_order_relaxed);
        m_timestamp_source.store(static_cast<uint8_t>(src), std::memory_order_relaxed);
        m_hardware_locked.store(out_ts.hardware_locked, std::memory_order_relaxed);
        m_timestamp_count.fetch_add(1, std::memory_order_relaxed);

        return bytes;
    }

    // Update physical transit jitter when packet contains sender tx_timestamp_ns
    void record_packet_transit(uint64_t rx_ns, uint64_t tx_ns) noexcept {
        if (rx_ns == 0) return;

        if (m_has_last_transit && tx_ns > 0 && m_last_tx_ns > 0) {
            int64_t d_rx = static_cast<int64_t>(rx_ns) - static_cast<int64_t>(m_last_rx_ns);
            int64_t d_tx = static_cast<int64_t>(tx_ns) - static_cast<int64_t>(m_last_tx_ns);
            int64_t jitter = std::abs(d_rx - d_tx);

            m_instant_jitter_ns.store(jitter, std::memory_order_relaxed);

            // Filtered average jitter (exponential moving average ODE, alpha = 0.05)
            int64_t current_avg = m_avg_jitter_ns.load(std::memory_order_relaxed);
            if (current_avg == 0) {
                m_avg_jitter_ns.store(jitter, std::memory_order_relaxed);
            } else {
                int64_t updated_avg = static_cast<int64_t>(0.95 * static_cast<double>(current_avg) + 0.05 * static_cast<double>(jitter));
                m_avg_jitter_ns.store(updated_avg, std::memory_order_relaxed);
            }

            // Max jitter tracking
            uint64_t curr_max = m_max_jitter_ns.load(std::memory_order_relaxed);
            if (static_cast<uint64_t>(jitter) > curr_max) {
                m_max_jitter_ns.store(static_cast<uint64_t>(jitter), std::memory_order_relaxed);
            }
        }

        m_last_rx_ns = rx_ns;
        m_last_tx_ns = tx_ns;
        m_has_last_transit = true;
    }

    void reset_stats() noexcept {
        m_instant_jitter_ns.store(0, std::memory_order_relaxed);
        m_avg_jitter_ns.store(0, std::memory_order_relaxed);
        m_max_jitter_ns.store(0, std::memory_order_relaxed);
        m_timestamp_count.store(0, std::memory_order_relaxed);
        m_has_last_transit = false;
        m_last_rx_ns = 0;
        m_last_tx_ns = 0;
    }

    // Getters
    [[nodiscard]] PtpTimestampSource source() const noexcept {
        return static_cast<PtpTimestampSource>(m_timestamp_source.load(std::memory_order_relaxed));
    }
    [[nodiscard]] bool is_hardware_locked() const noexcept {
        return m_hardware_locked.load(std::memory_order_relaxed);
    }
    [[nodiscard]] int64_t current_jitter_ns() const noexcept {
        return m_instant_jitter_ns.load(std::memory_order_relaxed);
    }
    [[nodiscard]] double avg_jitter_ns() const noexcept {
        return static_cast<double>(m_avg_jitter_ns.load(std::memory_order_relaxed));
    }
    [[nodiscard]] uint64_t max_jitter_ns() const noexcept {
        return m_max_jitter_ns.load(std::memory_order_relaxed);
    }
    [[nodiscard]] uint64_t last_rx_timestamp_ns() const noexcept {
        return m_rx_timestamp_ns.load(std::memory_order_relaxed);
    }
    [[nodiscard]] uint64_t timestamp_count() const noexcept {
        return m_timestamp_count.load(std::memory_order_relaxed);
    }
    [[nodiscard]] bool is_so_timestamping_active() const noexcept {
        return m_so_timestamping_active;
    }
    [[nodiscard]] bool is_hardware_capable() const noexcept {
        return m_hardware_capable;
    }

    [[nodiscard]] PtpTimestampInfo last_info() const noexcept {
        PtpTimestampInfo info;
        info.rx_timestamp_ns = m_rx_timestamp_ns.load(std::memory_order_relaxed);
        info.source = source();
        info.transit_jitter_ns = current_jitter_ns();
        info.avg_jitter_ns = avg_jitter_ns();
        info.max_jitter_ns = max_jitter_ns();
        info.hardware_locked = is_hardware_locked();
        return info;
    }

private:
    bool m_hardware_capable{false};
    bool m_so_timestamping_active{false};

    bool m_has_last_transit{false};
    uint64_t m_last_rx_ns{0};
    uint64_t m_last_tx_ns{0};

    std::atomic<uint64_t> m_rx_timestamp_ns{0};
    std::atomic<uint8_t> m_timestamp_source{static_cast<uint8_t>(PtpTimestampSource::UserspaceMonotonic)};
    std::atomic<bool> m_hardware_locked{false};
    std::atomic<int64_t> m_instant_jitter_ns{0};
    std::atomic<int64_t> m_avg_jitter_ns{0};
    std::atomic<uint64_t> m_max_jitter_ns{0};
    std::atomic<uint64_t> m_timestamp_count{0};
};

} // namespace audio_core::network
