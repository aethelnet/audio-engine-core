#pragma once

#include "audio_core/types.hpp"
#include "audio_core/network/aes67_ptp_engine.hpp"
#include "audio_core/network/ptp_hardware_engine.hpp"

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>
#include <vector>
#include <memory>
#include <atomic>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <arpa/inet.h>

namespace audio_core::network {

// ============================================================================
// IEEE 1588-2008 Precision Time Protocol Version 2 (PTPv2) Protocol Constants
// Standard Multicast Ports:
//   319: Event Messages (Sync, Delay_Req) - Requires Hardware/Kernel Timestamps
//   320: General Messages (Announce, Follow_Up, Delay_Resp, Management)
// ============================================================================
constexpr uint16_t kPtpEventPort = 319;
constexpr uint16_t kPtpGeneralPort = 320;
constexpr const char* kPtpPrimaryMulticastIp = "224.0.1.129"; // Domain 0 Primary

enum class PtpMessageType : uint8_t {
    Sync                  = 0x0,
    Delay_Req             = 0x1,
    Pdelay_Req            = 0x2,
    Pdelay_Resp           = 0x3,
    Follow_Up             = 0x8,
    Delay_Resp            = 0x9,
    Pdelay_Resp_Follow_Up = 0xA,
    Announce              = 0xB,
    Signaling             = 0xC,
    Management            = 0xD
};

enum class PtpPortState : uint8_t {
    Initializing = 0,
    Faulty,
    Disabled,
    Listening,
    PreMaster,
    Master,
    Passive,
    Uncalibrated,
    Slave
};

[[nodiscard]] inline const char* ptp_port_state_name(PtpPortState state) noexcept {
    switch (state) {
        case PtpPortState::Initializing: return "Initializing";
        case PtpPortState::Faulty:       return "Faulty";
        case PtpPortState::Disabled:     return "Disabled";
        case PtpPortState::Listening:    return "Listening";
        case PtpPortState::PreMaster:    return "Pre-Master";
        case PtpPortState::Master:       return "Master";
        case PtpPortState::Passive:      return "Passive";
        case PtpPortState::Uncalibrated: return "Uncalibrated";
        case PtpPortState::Slave:        return "Slave";
    }
    return "Unknown";
}

#pragma pack(push, 1)

struct PtpClockIdentity {
    uint8_t id[8]{0};

    bool operator==(const PtpClockIdentity& other) const noexcept {
        return std::memcmp(id, other.id, 8) == 0;
    }
    bool operator!=(const PtpClockIdentity& other) const noexcept {
        return !(*this == other);
    }
    bool operator<(const PtpClockIdentity& other) const noexcept {
        return std::memcmp(id, other.id, 8) < 0;
    }
};

struct PtpPortIdentity {
    PtpClockIdentity clock_id{};
    uint16_t port_number{1}; // Host order inside struct, converted on wire
};

struct PtpHeader {
    uint8_t message_type{0};           // Lower 4 bits = PtpMessageType, upper 4 bits = transportSpecific (0)
    uint8_t version_ptp{0x02};         // 0x02 = IEEE 1588-2008 PTPv2
    uint16_t message_length{0};        // Network Big-Endian
    uint8_t domain_number{0};          // 0 = Default domain
    uint8_t reserved1{0};
    uint16_t flags{0};                 // Bit 9 (0x0200 on BE) = Two-Step flag
    int64_t correction_field{0};       // Scaled ns * 2^16
    uint32_t reserved2{0};
    PtpClockIdentity source_clock_id{};
    uint16_t source_port_number{0};    // Network Big-Endian
    uint16_t sequence_id{0};           // Network Big-Endian
    uint8_t control_field{0};
    int8_t log_message_interval{0};
};
static_assert(sizeof(PtpHeader) == 34, "PtpHeader must be exactly 34 bytes packed");

struct PtpTimestampWire {
    uint16_t seconds_hi{0};            // Network Big-Endian upper 16 bits of 48-bit seconds
    uint32_t seconds_lo{0};            // Network Big-Endian lower 32 bits of 48-bit seconds
    uint32_t nanoseconds{0};           // Network Big-Endian nanoseconds

    [[nodiscard]] uint64_t to_nanoseconds() const noexcept {
        uint64_t s_hi = net_to_host16(seconds_hi);
        uint64_t s_lo = net_to_host32(seconds_lo);
        uint64_t secs = (s_hi << 32) | s_lo;
        uint64_t ns = net_to_host32(nanoseconds);
        return (secs * 1'000'000'000ULL) + ns;
    }

    static PtpTimestampWire from_nanoseconds(uint64_t total_ns) noexcept {
        uint64_t secs = total_ns / 1'000'000'000ULL;
        uint32_t ns = static_cast<uint32_t>(total_ns % 1'000'000'000ULL);
        return PtpTimestampWire{
            .seconds_hi = host_to_net16(static_cast<uint16_t>((secs >> 32) & 0xFFFF)),
            .seconds_lo = host_to_net32(static_cast<uint32_t>(secs & 0xFFFFFFFF)),
            .nanoseconds = host_to_net32(ns)
        };
    }
};
static_assert(sizeof(PtpTimestampWire) == 10, "PtpTimestampWire must be exactly 10 bytes packed");

struct PtpClockQuality {
    uint8_t clock_class{248};                  // 248 = Default / Uncalibrated, 6 = Primary reference (GPS)
    uint8_t clock_accuracy{0xFE};              // 0xFE = Unknown, 0x21 = < 100ns
    uint16_t offset_scaled_log_variance{0xFFFF}; // Network Big-Endian
};

struct PtpAnnounceBody {
    PtpTimestampWire origin_timestamp{};
    int16_t current_utc_offset{static_cast<int16_t>(host_to_net16(37))}; // +37s TAI vs UTC leap seconds
    uint8_t reserved{0};
    uint8_t gm_priority1{128};                 // Default 128 (Lower is higher priority)
    PtpClockQuality gm_clock_quality{};
    uint8_t gm_priority2{128};                 // Tie-breaker priority
    PtpClockIdentity gm_identity{};
    uint16_t steps_removed{0};                 // 0 for Grandmaster
    uint8_t time_source{0xA0};                 // 0xA0 = Internal oscillator, 0x20 = GPS
};
static_assert(sizeof(PtpAnnounceBody) == 30, "PtpAnnounceBody must be exactly 30 bytes packed");

struct PtpSyncFollowUpBody {
    PtpTimestampWire precise_origin_timestamp{};
};
static_assert(sizeof(PtpSyncFollowUpBody) == 10, "PtpSyncFollowUpBody must be exactly 10 bytes packed");

struct PtpDelayReqBody {
    PtpTimestampWire origin_timestamp{};
};
static_assert(sizeof(PtpDelayReqBody) == 10, "PtpDelayReqBody must be exactly 10 bytes packed");

struct PtpDelayRespBody {
    PtpTimestampWire receive_timestamp{};
    PtpClockIdentity requesting_clock_id{};
    uint16_t requesting_port_number{0};        // Network Big-Endian
};
static_assert(sizeof(PtpDelayRespBody) == 20, "PtpDelayRespBody must be exactly 20 bytes packed");

#pragma pack(pop)

// ============================================================================
// IEEE 1588-2008 Best Master Clock Algorithm (BMCA)
// Compares priority vectors of two clocks to deterministically select Grandmaster
// ============================================================================
struct PtpPriorityVector {
    uint8_t priority1{128};
    PtpClockQuality clock_quality{};
    uint8_t priority2{128};
    PtpClockIdentity identity{};
    uint16_t steps_removed{0};

    // Returns: -1 if this is better, +1 if other is better, 0 if equal
    [[nodiscard]] int compare(const PtpPriorityVector& other) const noexcept {
        if (priority1 < other.priority1) return -1;
        if (priority1 > other.priority1) return 1;

        if (clock_quality.clock_class < other.clock_quality.clock_class) return -1;
        if (clock_quality.clock_class > other.clock_quality.clock_class) return 1;

        if (clock_quality.clock_accuracy < other.clock_quality.clock_accuracy) return -1;
        if (clock_quality.clock_accuracy > other.clock_quality.clock_accuracy) return 1;

        uint16_t v1 = net_to_host16(clock_quality.offset_scaled_log_variance);
        uint16_t v2 = net_to_host16(other.clock_quality.offset_scaled_log_variance);
        if (v1 < v2) return -1;
        if (v1 > v2) return 1;

        if (priority2 < other.priority2) return -1;
        if (priority2 > other.priority2) return 1;

        if (identity < other.identity) return -1;
        if (other.identity < identity) return 1;

        if (steps_removed < other.steps_removed) return -1;
        if (steps_removed > other.steps_removed) return 1;

        return 0;
    }
};

// ============================================================================
// Proportional-Integral (PI) Clock Servo
// Disciplines boundary clock time with zero pitch flutter / phase clicks
// ============================================================================
class PtpClockServo {
public:
    explicit PtpClockServo(double kp = 0.6, double ki = 0.05, double max_ppb = 500'000.0) noexcept
        : m_kp(kp), m_ki(ki), m_max_ppb(max_ppb) {}

    void reset() noexcept {
        m_integral = 0.0;
        m_offset_ns = 0.0;
        m_freq_drift_ppb = 0.0;
        m_locked = false;
    }

    // Process measured offset from Grandmaster (e[k] = offset_ns)
    // Returns frequency adjustment in parts-per-billion (ppb) clamped to physical oscillator range
    double update(double offset_ns) noexcept {
        m_offset_ns = offset_ns;
        m_integral += offset_ns * m_ki;

        // Anti-windup limit (+/- max_ppb / 2)
        const double int_limit = m_max_ppb * 0.5;
        m_integral = std::clamp(m_integral, -int_limit, int_limit);

        // Clamped total frequency adjustment to prevent pitch flutter runaway
        m_freq_drift_ppb = std::clamp((m_kp * offset_ns) + m_integral, -m_max_ppb, m_max_ppb);
        m_locked = (std::abs(offset_ns) < 1'000.0); // Sub-microsecond lock (< 1 µs)
        return m_freq_drift_ppb;
    }

    [[nodiscard]] double offset_ns() const noexcept { return m_offset_ns; }
    [[nodiscard]] double freq_drift_ppb() const noexcept { return m_freq_drift_ppb; }
    [[nodiscard]] bool is_locked() const noexcept { return m_locked; }
    [[nodiscard]] double max_ppb() const noexcept { return m_max_ppb; }

private:
    double m_kp{0.6};
    double m_ki{0.05};
    double m_max_ppb{500'000.0}; // +/- 500 ppm max pull
    double m_integral{0.0};
    double m_offset_ns{0.0};
    double m_freq_drift_ppb{0.0};
    bool m_locked{false};
};

// ============================================================================
// PtpBoundaryClock: IEEE 1588-2008 Multi-Interface Boundary Clock
// Bridges network timing across upstream and downstream segments
// ============================================================================
class PtpBoundaryClock {
public:
    struct PortConfig {
        uint16_t port_number{1};
        std::string iface_name{"lo"};
        bool can_be_master{true};
        bool can_be_slave{true};
    };

    struct PortTelemetry {
        uint16_t port_number{1};
        PtpPortState state{PtpPortState::Listening};
        int64_t offset_from_master_ns{0};
        int64_t mean_path_delay_ns{0};
        uint64_t sync_packets_rx{0};
        uint64_t sync_packets_tx{0};
        uint64_t announce_packets_rx{0};
        uint64_t announce_packets_tx{0};
        bool hardware_locked{false};
    };

    explicit PtpBoundaryClock(PtpClockIdentity clock_id, uint8_t priority1 = 128, uint8_t priority2 = 128) noexcept
        : m_clock_id(clock_id), m_priority1(priority1), m_priority2(priority2) {
        m_local_vector.priority1 = priority1;
        m_local_vector.priority2 = priority2;
        m_local_vector.identity = clock_id;
        m_local_vector.clock_quality.clock_class = 248; // Default boundary clock
        m_local_vector.steps_removed = 0;
        m_best_vector = m_local_vector;
    }

    void add_port(uint16_t port_num, std::string iface_name = "lo",
                  bool can_be_master = true, bool can_be_slave = true) {
        PortConfig cfg{
            .port_number = port_num,
            .iface_name = std::move(iface_name),
            .can_be_master = can_be_master,
            .can_be_slave = can_be_slave
        };
        m_port_configs.push_back(cfg);

        PortTelemetry telem{
            .port_number = port_num,
            .state = can_be_slave ? PtpPortState::Listening : PtpPortState::Master
        };
        m_port_telemetry.push_back(telem);
    }

    [[nodiscard]] size_t num_ports() const noexcept { return m_port_configs.size(); }
    [[nodiscard]] const PtpClockIdentity& clock_identity() const noexcept { return m_clock_id; }
    [[nodiscard]] const PtpPriorityVector& local_vector() const noexcept { return m_local_vector; }
    [[nodiscard]] const PtpPriorityVector& best_vector() const noexcept { return m_best_vector; }
    [[nodiscard]] bool has_foreign_master() const noexcept { return m_has_foreign_master; }

    [[nodiscard]] PtpPriorityVector announce_vector_for_master() const noexcept {
        PtpPriorityVector vec = m_best_vector;
        vec.steps_removed++;
        return vec;
    }

    [[nodiscard]] PtpPortState port_state(size_t port_idx) const noexcept {
        if (port_idx < m_port_telemetry.size()) {
            return m_port_telemetry[port_idx].state;
        }
        return PtpPortState::Disabled;
    }

    void set_port_state(size_t port_idx, PtpPortState state) noexcept {
        if (port_idx < m_port_telemetry.size()) {
            m_port_telemetry[port_idx].state = state;
        }
    }

    [[nodiscard]] const PortTelemetry& port_telemetry(size_t port_idx) const noexcept {
        return m_port_telemetry[port_idx];
    }

    [[nodiscard]] const PortConfig& port_config(size_t port_idx) const noexcept {
        return m_port_configs[port_idx];
    }

    [[nodiscard]] const PtpClockServo& servo() const noexcept { return m_servo; }
    [[nodiscard]] bool is_locked() const noexcept { return m_servo.is_locked(); }
    [[nodiscard]] int64_t current_offset_ns() const noexcept { return static_cast<int64_t>(m_servo.offset_ns()); }

    void record_port_rx(size_t port_idx, PtpMessageType type, const PtpTimestampInfo& ts_info) noexcept {
        if (port_idx >= m_port_telemetry.size()) return;
        auto& t = m_port_telemetry[port_idx];
        t.hardware_locked = ts_info.hardware_locked;
        switch (type) {
            case PtpMessageType::Sync: t.sync_packets_rx++; break;
            case PtpMessageType::Announce: t.announce_packets_rx++; break;
            default: break;
        }
    }

    void record_port_tx(size_t port_idx, PtpMessageType type) noexcept {
        if (port_idx >= m_port_telemetry.size()) return;
        auto& t = m_port_telemetry[port_idx];
        switch (type) {
            case PtpMessageType::Sync: t.sync_packets_tx++; break;
            case PtpMessageType::Announce: t.announce_packets_tx++; break;
            default: break;
        }
    }

    // BMCA Execution: Evaluate received Announce vector against local and current best clock
    void evaluate_announce(size_t port_idx, const PtpPriorityVector& foreign_vector) noexcept {
        if (port_idx >= m_port_telemetry.size()) return;
        m_port_telemetry[port_idx].announce_packets_rx++;

        int cmp_local = foreign_vector.compare(m_local_vector);
        if (cmp_local < 0 && m_port_configs[port_idx].can_be_slave) {
            // Foreign vector is better Grandmaster than local clock!
            m_has_foreign_master = true;
            m_best_vector = foreign_vector;

            // Port transitions to Slave mode
            m_port_telemetry[port_idx].state = PtpPortState::Slave;

            // Other ports in the Boundary Clock transition to Master mode to distribute time
            for (size_t i = 0; i < m_port_telemetry.size(); ++i) {
                if (i != port_idx && m_port_configs[i].can_be_master) {
                    m_port_telemetry[i].state = PtpPortState::Master;
                }
            }
        } else if (cmp_local > 0 && m_port_configs[port_idx].can_be_master) {
            // Local clock is superior Grandmaster
            m_has_foreign_master = false;
            m_best_vector = m_local_vector;
            m_port_telemetry[port_idx].state = PtpPortState::Master;
        }
    }

    // Process IEEE 1588 Timing Message Exchange:
    // t1: Sync / Follow_Up PreciseOriginTimestamp (Egress Master)
    // t2: Sync Ingress Timestamp (Ingress Slave)
    // t3: Delay_Req Egress Timestamp (Egress Slave)
    // t4: Delay_Resp ReceiveTimestamp (Ingress Master)
    // Mean Path Delay: D = ((t2 - t1) + (t4 - t3)) / 2
    // Offset From Master: O = ((t2 - t1) - (t4 - t3)) / 2
    void process_timing_exchange(size_t port_idx, uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4) noexcept {
        if (port_idx >= m_port_telemetry.size()) return;

        int64_t d_t2_t1 = static_cast<int64_t>(t2) - static_cast<int64_t>(t1);
        int64_t d_t4_t3 = static_cast<int64_t>(t4) - static_cast<int64_t>(t3);

        int64_t mean_delay = (d_t2_t1 + d_t4_t3) / 2;
        int64_t offset = (d_t2_t1 - d_t4_t3) / 2;

        m_port_telemetry[port_idx].mean_path_delay_ns = mean_delay;
        m_port_telemetry[port_idx].offset_from_master_ns = offset;
        m_port_telemetry[port_idx].sync_packets_rx++;

        // Update PI Servo
        m_servo.update(static_cast<double>(offset));
    }

    // Pack IEEE 1588-2008 PTPv2 Messages (Zero-Allocation Buffer Serialization)
    static size_t build_sync_packet(uint8_t* buffer, size_t max_len,
                                    const PtpClockIdentity& clock_id, uint16_t port_num,
                                    uint16_t seq_id, uint64_t origin_timestamp_ns,
                                    bool two_step = true) noexcept {
        constexpr size_t kTotalLen = sizeof(PtpHeader) + sizeof(PtpSyncFollowUpBody);
        if (!buffer || max_len < kTotalLen) return 0;

        auto* hdr = reinterpret_cast<PtpHeader*>(buffer);
        *hdr = PtpHeader{};
        hdr->message_type = static_cast<uint8_t>(PtpMessageType::Sync) & 0x0F;
        hdr->version_ptp = 0x02;
        hdr->message_length = host_to_net16(static_cast<uint16_t>(kTotalLen));
        hdr->domain_number = 0;
        hdr->flags = two_step ? host_to_net16(0x0200) : 0; // Two-step flag
        hdr->source_clock_id = clock_id;
        hdr->source_port_number = host_to_net16(port_num);
        hdr->sequence_id = host_to_net16(seq_id);
        hdr->log_message_interval = 0; // 1 packet per second (2^0)

        auto* body = reinterpret_cast<PtpSyncFollowUpBody*>(buffer + sizeof(PtpHeader));
        body->precise_origin_timestamp = PtpTimestampWire::from_nanoseconds(origin_timestamp_ns);

        return kTotalLen;
    }

    static size_t build_follow_up_packet(uint8_t* buffer, size_t max_len,
                                         const PtpClockIdentity& clock_id, uint16_t port_num,
                                         uint16_t seq_id, uint64_t precise_origin_ns) noexcept {
        constexpr size_t kTotalLen = sizeof(PtpHeader) + sizeof(PtpSyncFollowUpBody);
        if (!buffer || max_len < kTotalLen) return 0;

        auto* hdr = reinterpret_cast<PtpHeader*>(buffer);
        *hdr = PtpHeader{};
        hdr->message_type = static_cast<uint8_t>(PtpMessageType::Follow_Up) & 0x0F;
        hdr->version_ptp = 0x02;
        hdr->message_length = host_to_net16(static_cast<uint16_t>(kTotalLen));
        hdr->domain_number = 0;
        hdr->source_clock_id = clock_id;
        hdr->source_port_number = host_to_net16(port_num);
        hdr->sequence_id = host_to_net16(seq_id);
        hdr->log_message_interval = 0;

        auto* body = reinterpret_cast<PtpSyncFollowUpBody*>(buffer + sizeof(PtpHeader));
        body->precise_origin_timestamp = PtpTimestampWire::from_nanoseconds(precise_origin_ns);

        return kTotalLen;
    }

    static size_t build_delay_req_packet(uint8_t* buffer, size_t max_len,
                                         const PtpClockIdentity& clock_id, uint16_t port_num,
                                         uint16_t seq_id, uint64_t origin_timestamp_ns) noexcept {
        constexpr size_t kTotalLen = sizeof(PtpHeader) + sizeof(PtpDelayReqBody);
        if (!buffer || max_len < kTotalLen) return 0;

        auto* hdr = reinterpret_cast<PtpHeader*>(buffer);
        *hdr = PtpHeader{};
        hdr->message_type = static_cast<uint8_t>(PtpMessageType::Delay_Req) & 0x0F;
        hdr->version_ptp = 0x02;
        hdr->message_length = host_to_net16(static_cast<uint16_t>(kTotalLen));
        hdr->domain_number = 0;
        hdr->source_clock_id = clock_id;
        hdr->source_port_number = host_to_net16(port_num);
        hdr->sequence_id = host_to_net16(seq_id);
        hdr->log_message_interval = 0;

        auto* body = reinterpret_cast<PtpDelayReqBody*>(buffer + sizeof(PtpHeader));
        body->origin_timestamp = PtpTimestampWire::from_nanoseconds(origin_timestamp_ns);

        return kTotalLen;
    }

    static size_t build_delay_resp_packet(uint8_t* buffer, size_t max_len,
                                          const PtpClockIdentity& clock_id, uint16_t port_num,
                                          uint16_t seq_id, uint64_t receive_timestamp_ns,
                                          const PtpClockIdentity& requesting_clock_id,
                                          uint16_t requesting_port_num) noexcept {
        constexpr size_t kTotalLen = sizeof(PtpHeader) + sizeof(PtpDelayRespBody);
        if (!buffer || max_len < kTotalLen) return 0;

        auto* hdr = reinterpret_cast<PtpHeader*>(buffer);
        *hdr = PtpHeader{};
        hdr->message_type = static_cast<uint8_t>(PtpMessageType::Delay_Resp) & 0x0F;
        hdr->version_ptp = 0x02;
        hdr->message_length = host_to_net16(static_cast<uint16_t>(kTotalLen));
        hdr->domain_number = 0;
        hdr->source_clock_id = clock_id;
        hdr->source_port_number = host_to_net16(port_num);
        hdr->sequence_id = host_to_net16(seq_id);
        hdr->log_message_interval = 0;

        auto* body = reinterpret_cast<PtpDelayRespBody*>(buffer + sizeof(PtpHeader));
        body->receive_timestamp = PtpTimestampWire::from_nanoseconds(receive_timestamp_ns);
        body->requesting_clock_id = requesting_clock_id;
        body->requesting_port_number = host_to_net16(requesting_port_num);

        return kTotalLen;
    }

    static size_t build_announce_packet(uint8_t* buffer, size_t max_len,
                                        const PtpClockIdentity& clock_id, uint16_t port_num,
                                        uint16_t seq_id, const PtpPriorityVector& vector) noexcept {
        constexpr size_t kTotalLen = sizeof(PtpHeader) + sizeof(PtpAnnounceBody);
        if (!buffer || max_len < kTotalLen) return 0;

        auto* hdr = reinterpret_cast<PtpHeader*>(buffer);
        *hdr = PtpHeader{};
        hdr->message_type = static_cast<uint8_t>(PtpMessageType::Announce) & 0x0F;
        hdr->version_ptp = 0x02;
        hdr->message_length = host_to_net16(static_cast<uint16_t>(kTotalLen));
        hdr->domain_number = 0;
        hdr->source_clock_id = clock_id;
        hdr->source_port_number = host_to_net16(port_num);
        hdr->sequence_id = host_to_net16(seq_id);
        hdr->log_message_interval = 1; // Every 2 seconds (2^1)

        auto* body = reinterpret_cast<PtpAnnounceBody*>(buffer + sizeof(PtpHeader));
        *body = PtpAnnounceBody{};
        body->origin_timestamp = PtpTimestampWire::from_nanoseconds(0);
        body->current_utc_offset = static_cast<int16_t>(host_to_net16(37));
        body->gm_priority1 = vector.priority1;
        body->gm_clock_quality = vector.clock_quality;
        body->gm_priority2 = vector.priority2;
        body->gm_identity = vector.identity;
        body->steps_removed = host_to_net16(vector.steps_removed);
        body->time_source = 0xA0; // Internal Oscillator

        return kTotalLen;
    }

    // Zero-allocation wire deserializer
    struct ParsedPtpMessage {
        PtpMessageType type{PtpMessageType::Sync};
        uint8_t domain{0};
        uint16_t flags{0};
        int64_t correction_field{0};
        PtpClockIdentity source_clock_id{};
        uint16_t source_port_number{0};
        uint16_t sequence_id{0};
        uint64_t timestamp_ns{0}; // Precise origin (Sync/Follow_Up/Delay_Req) or receive timestamp (Delay_Resp)
        PtpPriorityVector announce_vector{};
        PtpClockIdentity requesting_clock_id{};
        uint16_t requesting_port_number{0};
        bool valid{false};
    };

    static ParsedPtpMessage parse_packet(const uint8_t* buffer, size_t length) noexcept {
        ParsedPtpMessage msg{};
        if (!buffer || length < sizeof(PtpHeader)) return msg;

        const auto* hdr = reinterpret_cast<const PtpHeader*>(buffer);
        if (hdr->version_ptp != 0x02) return msg;

        msg.type = static_cast<PtpMessageType>(hdr->message_type & 0x0F);
        msg.domain = hdr->domain_number;
        msg.flags = net_to_host16(hdr->flags);
        msg.correction_field = hdr->correction_field;
        msg.source_clock_id = hdr->source_clock_id;
        msg.source_port_number = net_to_host16(hdr->source_port_number);
        msg.sequence_id = net_to_host16(hdr->sequence_id);

        uint16_t declared_len = net_to_host16(hdr->message_length);
        if (length < declared_len) return msg;

        const uint8_t* body_ptr = buffer + sizeof(PtpHeader);

        switch (msg.type) {
            case PtpMessageType::Sync:
            case PtpMessageType::Follow_Up: {
                if (length < sizeof(PtpHeader) + sizeof(PtpSyncFollowUpBody)) return msg;
                const auto* body = reinterpret_cast<const PtpSyncFollowUpBody*>(body_ptr);
                msg.timestamp_ns = body->precise_origin_timestamp.to_nanoseconds();
                msg.valid = true;
                break;
            }
            case PtpMessageType::Delay_Req: {
                if (length < sizeof(PtpHeader) + sizeof(PtpDelayReqBody)) return msg;
                const auto* body = reinterpret_cast<const PtpDelayReqBody*>(body_ptr);
                msg.timestamp_ns = body->origin_timestamp.to_nanoseconds();
                msg.valid = true;
                break;
            }
            case PtpMessageType::Delay_Resp: {
                if (length < sizeof(PtpHeader) + sizeof(PtpDelayRespBody)) return msg;
                const auto* body = reinterpret_cast<const PtpDelayRespBody*>(body_ptr);
                msg.timestamp_ns = body->receive_timestamp.to_nanoseconds();
                msg.requesting_clock_id = body->requesting_clock_id;
                msg.requesting_port_number = net_to_host16(body->requesting_port_number);
                msg.valid = true;
                break;
            }
            case PtpMessageType::Announce: {
                if (length < sizeof(PtpHeader) + sizeof(PtpAnnounceBody)) return msg;
                const auto* body = reinterpret_cast<const PtpAnnounceBody*>(body_ptr);
                msg.announce_vector.priority1 = body->gm_priority1;
                msg.announce_vector.clock_quality = body->gm_clock_quality;
                msg.announce_vector.priority2 = body->gm_priority2;
                msg.announce_vector.identity = body->gm_identity;
                msg.announce_vector.steps_removed = net_to_host16(body->steps_removed);
                msg.valid = true;
                break;
            }
            default:
                msg.valid = true;
                break;
        }

        return msg;
    }

private:
    PtpClockIdentity m_clock_id{};
    uint8_t m_priority1{128};
    uint8_t m_priority2{128};
    PtpPriorityVector m_local_vector{};
    PtpPriorityVector m_best_vector{};
    bool m_has_foreign_master{false};
    std::vector<PortConfig> m_port_configs{};
    std::vector<PortTelemetry> m_port_telemetry{};
    PtpClockServo m_servo{};
};

// ============================================================================
// PtpBoundaryPortSocket: Linux Dual-Socket Interface with Hardware Timestamping
// Manages Event (319) and General (320) sockets with multicast & interface binding
// ============================================================================
class PtpBoundaryPortSocket {
public:
    PtpBoundaryPortSocket() = default;
    ~PtpBoundaryPortSocket() { close(); }

    PtpBoundaryPortSocket(const PtpBoundaryPortSocket&) = delete;
    PtpBoundaryPortSocket& operator=(const PtpBoundaryPortSocket&) = delete;

    PtpBoundaryPortSocket(PtpBoundaryPortSocket&& other) noexcept
        : m_event_sock(other.m_event_sock),
          m_general_sock(other.m_general_sock),
          m_iface_name(std::move(other.m_iface_name)),
          m_domain(other.m_domain),
          m_ts_engine(std::move(other.m_ts_engine)) {
        other.m_event_sock = -1;
        other.m_general_sock = -1;
    }

    PtpBoundaryPortSocket& operator=(PtpBoundaryPortSocket&& other) noexcept {
        if (this != &other) {
            close();
            m_event_sock = other.m_event_sock;
            m_general_sock = other.m_general_sock;
            m_iface_name = std::move(other.m_iface_name);
            m_domain = other.m_domain;
            m_ts_engine = std::move(other.m_ts_engine);
            other.m_event_sock = -1;
            other.m_general_sock = -1;
        }
        return *this;
    }

    // Open and bind dual sockets on specified network interface
    bool open(const std::string& iface_name,
              uint16_t event_port = kPtpEventPort,
              uint16_t general_port = kPtpGeneralPort,
              uint8_t domain = 0) noexcept {
        close();
        m_iface_name = iface_name;
        m_domain = domain;

        m_event_sock = create_and_bind_socket(event_port, iface_name, true);
        if (m_event_sock < 0) {
            close();
            return false;
        }

        m_general_sock = create_and_bind_socket(general_port, iface_name, false);
        if (m_general_sock < 0) {
            close();
            return false;
        }

        // Configure hardware timestamping on event socket
        m_ts_engine.configure_socket(m_event_sock, iface_name, true);

        return true;
    }

    void close() noexcept {
        if (m_event_sock >= 0) {
            ::close(m_event_sock);
            m_event_sock = -1;
        }
        if (m_general_sock >= 0) {
            ::close(m_general_sock);
            m_general_sock = -1;
        }
    }

    [[nodiscard]] bool is_open() const noexcept { return m_event_sock >= 0 && m_general_sock >= 0; }
    [[nodiscard]] int event_fd() const noexcept { return m_event_sock; }
    [[nodiscard]] int general_fd() const noexcept { return m_general_sock; }
    [[nodiscard]] const std::string& iface_name() const noexcept { return m_iface_name; }

    [[nodiscard]] PtpSocketTimestampEngine& ts_engine() noexcept { return m_ts_engine; }
    [[nodiscard]] const PtpSocketTimestampEngine& ts_engine() const noexcept { return m_ts_engine; }

    // Send PTP Event datagram (Sync, Delay_Req) with hardware egress timestamping
    ssize_t send_event(const uint8_t* data, size_t len,
                       uint64_t& out_tx_ns, PtpTimestampSource& out_source,
                       const char* dest_ip = kPtpPrimaryMulticastIp,
                       uint16_t dest_port = kPtpEventPort) noexcept {
        if (m_event_sock < 0 || !data || len == 0) return -1;

        sockaddr_in dst{};
        dst.sin_family = AF_INET;
        dst.sin_port = htons(dest_port);
        ::inet_pton(AF_INET, dest_ip, &dst.sin_addr);

        ssize_t sent = ::sendto(m_event_sock, data, len, 0,
                                reinterpret_cast<const sockaddr*>(&dst), sizeof(dst));
        if (sent <= 0) return sent;

        // Try reading physical PHY hardware transmit timestamp from error queue
        out_tx_ns = m_ts_engine.fetch_tx_timestamp(m_event_sock, out_source);
        if (out_tx_ns == 0) {
            // Fallback to high-resolution monotonic time
            struct timespec now{};
            ::clock_gettime(CLOCK_MONOTONIC_RAW, &now);
            out_tx_ns = static_cast<uint64_t>(now.tv_sec) * 1'000'000'000ULL + static_cast<uint64_t>(now.tv_nsec);
            out_source = PtpTimestampSource::UserspaceMonotonic;
        }

        return sent;
    }

    // Send PTP General datagram (Announce, Follow_Up, Delay_Resp)
    ssize_t send_general(const uint8_t* data, size_t len,
                         const char* dest_ip = kPtpPrimaryMulticastIp,
                         uint16_t dest_port = kPtpGeneralPort) noexcept {
        if (m_general_sock < 0 || !data || len == 0) return -1;

        sockaddr_in dst{};
        dst.sin_family = AF_INET;
        dst.sin_port = htons(dest_port);
        ::inet_pton(AF_INET, dest_ip, &dst.sin_addr);

        return ::sendto(m_general_sock, data, len, 0,
                        reinterpret_cast<const sockaddr*>(&dst), sizeof(dst));
    }

    // Receive PTP Event datagram with hardware/kernel ingress timestamping
    ssize_t recv_event(uint8_t* buf, size_t cap, sockaddr_in* src_addr, PtpTimestampInfo& out_ts) noexcept {
        if (m_event_sock < 0) return -1;
        return m_ts_engine.recvmsg_with_timestamp(m_event_sock, buf, cap, src_addr, out_ts);
    }

    // Receive PTP General datagram
    ssize_t recv_general(uint8_t* buf, size_t cap, sockaddr_in* src_addr) noexcept {
        if (m_general_sock < 0 || !buf || cap == 0) return -1;
        socklen_t addr_len = src_addr ? sizeof(sockaddr_in) : 0;
        return ::recvfrom(m_general_sock, buf, cap, MSG_DONTWAIT,
                          reinterpret_cast<sockaddr*>(src_addr), src_addr ? &addr_len : nullptr);
    }

private:
    int create_and_bind_socket(uint16_t port, const std::string& iface_name, bool is_event) noexcept {
        int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
        if (fd < 0) return -1;

        int reuse = 1;
        ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
#ifdef SO_REUSEPORT
        ::setsockopt(fd, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));
#endif

        // Non-fatal interface binding (SO_BINDTODEVICE requires CAP_NET_RAW)
        if (!iface_name.empty() && iface_name != "lo") {
            struct ifreq ifr{};
            std::strncpy(ifr.ifr_name, iface_name.c_str(), sizeof(ifr.ifr_name) - 1);
            ::setsockopt(fd, SOL_SOCKET, SO_BINDTODEVICE, &ifr, sizeof(ifr));
        }

        // Bind to port
        sockaddr_in bind_addr{};
        bind_addr.sin_family = AF_INET;
        bind_addr.sin_port = htons(port);
        bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (::bind(fd, reinterpret_cast<const sockaddr*>(&bind_addr), sizeof(bind_addr)) < 0) {
            ::close(fd);
            return -1;
        }

        // Multicast group joining
        struct ip_mreqn mreq{};
        ::inet_pton(AF_INET, kPtpPrimaryMulticastIp, &mreq.imr_multiaddr);
        mreq.imr_address.s_addr = htonl(INADDR_ANY);
        if (!iface_name.empty()) {
            mreq.imr_ifindex = static_cast<int>(::if_nametoindex(iface_name.c_str()));
        }
        ::setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));

        // Multicast loopback enabled
        int loop = 1;
        ::setsockopt(fd, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));

        return fd;
    }

    int m_event_sock{-1};
    int m_general_sock{-1};
    std::string m_iface_name{"lo"};
    uint8_t m_domain{0};
    PtpSocketTimestampEngine m_ts_engine{};
};

} // namespace audio_core::network
