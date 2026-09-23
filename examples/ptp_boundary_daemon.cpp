#include "audio_core/network/ptp_boundary_clock.hpp"

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <filesystem>
#include <algorithm>
#include <poll.h>

static std::atomic<bool> g_running{true};

static void sigint_handler(int) {
    g_running.store(false);
}

static void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [OPTIONS]\n"
              << "Options:\n"
              << "  --upstream <iface>    Upstream network interface (default: eth0)\n"
              << "  --downstream <iface>  Downstream network interface (default: eth1)\n"
              << "  --domain <id>         PTP domain number [0-255] (default: 0)\n"
              << "  --priority1 <val>     BMCA Priority1 [0-255] (default: 128)\n"
              << "  --priority2 <val>     BMCA Priority2 [0-255] (default: 128)\n"
              << "  --event-port <port>   UDP Event Port (default: 319)\n"
              << "  --general-port <port> UDP General Port (default: 320)\n"
              << "  --probe, --audit      Audit interfaces & probe LAN for PTP Grandmasters\n"
              << "  --help                Show this help message\n";
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, sigint_handler);
    std::signal(SIGTERM, sigint_handler);

    std::string upstream_iface = "eth0";
    std::string downstream_iface = "eth1";
    uint8_t domain = 0;
    uint8_t priority1 = 128;
    uint8_t priority2 = 128;
    uint16_t event_port = audio_core::network::kPtpEventPort;
    uint16_t general_port = audio_core::network::kPtpGeneralPort;
    bool run_probe_only = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--probe" || arg == "--audit") {
            run_probe_only = true;
        } else if (arg == "--upstream" && i + 1 < argc) {
            upstream_iface = argv[++i];
        } else if (arg == "--downstream" && i + 1 < argc) {
            downstream_iface = argv[++i];
        } else if (arg == "--domain" && i + 1 < argc) {
            domain = static_cast<uint8_t>(std::stoi(argv[++i]));
        } else if (arg == "--priority1" && i + 1 < argc) {
            priority1 = static_cast<uint8_t>(std::stoi(argv[++i]));
        } else if (arg == "--priority2" && i + 1 < argc) {
            priority2 = static_cast<uint8_t>(std::stoi(argv[++i]));
        } else if (arg == "--event-port" && i + 1 < argc) {
            event_port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--general-port" && i + 1 < argc) {
            general_port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        }
    }

    if (run_probe_only) {
        std::cout << "\033[1;36m====================================================================\033[0m\n";
        std::cout << "\033[1;37m   AETHEL PTPv2 GRANDMASTER HARDWARE AUDIT & LAN PROBE (IEEE 1588)\033[0m\n";
        std::cout << "\033[1;36m====================================================================\033[0m\n\n";

        std::vector<std::string> ifaces;
        try {
            for (const auto& entry : std::filesystem::directory_iterator("/sys/class/net")) {
                ifaces.push_back(entry.path().filename().string());
            }
        } catch (...) {
            ifaces = { "lo", "enp2s0", "wlp4s0", "eth0" };
        }
        std::sort(ifaces.begin(), ifaces.end());

        std::cout << "\033[1;33m[SECTION 1: NETWORK INTERFACE & TIMESTAMPING CAPABILITY AUDIT]\033[0m\n";
        for (const auto& iface : ifaces) {
            auto audit = audio_core::network::audit_interface(iface);
            std::cout << "  Interface: \033[1;37m" << std::left << std::setw(10) << iface << "\033[0m"
                      << " | Status: " << (audit.is_up ? "\033[1;32mUP\033[0m  " : "\033[1;31mDOWN\033[0m")
                      << " | Running: " << (audit.is_running ? "\033[1;32mYES\033[0m" : "\033[1;30mNO \033[0m")
                      << " | PHC Clock: ";
            if (audit.phc_index >= 0) {
                std::cout << "\033[1;32m/dev/ptp" << audit.phc_index << " [HARDWARE PHY]\033[0m";
            } else {
                std::cout << "\033[1;30mNone [NO PHC]\033[0m";
            }
            std::cout << " | Effective Tier: ";
            if (audit.highest_capable_tier == audio_core::network::PtpTimestampSource::HardwareNicPhy) {
                std::cout << "\033[1;32mHARDWARE NIC PHY\033[0m";
            } else if (audit.highest_capable_tier == audio_core::network::PtpTimestampSource::KernelDriverStack) {
                std::cout << "\033[1;36mKERNEL DRIVER STACK\033[0m";
            } else {
                std::cout << "\033[1;33mUSERSPACE FALLBACK\033[0m";
            }
            std::cout << "\n";
        }

        std::cout << "\n\033[1;33m[SECTION 2: ACTIVE PTPv2 MULTICAST DISCOVERY PROBE (224.0.1.129)]\033[0m\n";
        bool any_gm_found = false;
        for (const auto& iface : ifaces) {
            auto audit = audio_core::network::audit_interface(iface);
            if (!audit.is_up && iface != "lo") continue;

            std::cout << "  Probing on " << std::left << std::setw(10) << iface << " (Port " << general_port << ", timeout 1.0s)... " << std::flush;
            auto probe = audio_core::network::probe_grandmaster(iface, general_port, 1000);
            if (!probe.socket_bound) {
                std::cout << "\033[1;31mSKIPPED\033[0m (" << probe.error_message << ")\n";
                continue;
            }

            if (probe.grandmaster_detected) {
                any_gm_found = true;
                std::cout << "\033[1;32mGRANDMASTER DETECTED!\033[0m\n";
                std::cout << "    -> Clock ID:       " << probe.gm_identity_str << "\n"
                          << "    -> Clock Class:    " << static_cast<int>(probe.clock_class) << " (" << probe.clock_class_name << ")\n"
                          << "    -> Clock Accuracy: 0x" << std::hex << static_cast<int>(probe.clock_accuracy) << std::dec << " (" << probe.clock_accuracy_name << ")\n"
                          << "    -> Time Source:    0x" << std::hex << static_cast<int>(probe.time_source) << std::dec << " (" << probe.time_source_name << ")\n"
                          << "    -> Priority 1 / 2: " << static_cast<int>(probe.priority1) << " / " << static_cast<int>(probe.priority2) << "\n"
                          << "    -> Steps Removed:  " << probe.steps_removed << "\n"
                          << "    -> Grandmaster IP: " << probe.grandmaster_ip << "\n";
            } else {
                std::cout << "\033[1;30mNo Grandmaster Announce detected\033[0m\n";
            }
        }

        std::cout << "\n\033[1;33m[SECTION 3: CLOCK SYNCHRONIZATION VERDICT]\033[0m\n";
        if (any_gm_found) {
            std::cout << "  \033[1;32mRESULT: Active IEEE 1588 / AES67 Grandmaster present on network.\033[0m\n"
                      << "  Boundary Clock will lock as Slave to upstream Grandmaster and discipline local audio jitter.\n";
        } else {
            std::cout << "  \033[1;33mRESULT: No external hardware Grandmaster broadcasting on local network interfaces.\033[0m\n"
                      << "  Aethel Engine Boundary Clock will deterministically operate as Self-Master (Class 248 / Internal Oscillator)\n"
                      << "  to discipline and synchronize downstream AES67 / Dante audio endpoints without drift.\n";
        }

        std::cout << "\033[1;36m====================================================================\033[0m\n";
        return 0;
    }

    std::cout << "\033[1;36m====================================================================\033[0m\n";
    std::cout << "\033[1;37m   AETHEL PTPv2 BOUNDARY CLOCK & MASTER SYNC DAEMON (IEEE 1588-2008)\033[0m\n";
    std::cout << "\033[1;36m====================================================================\033[0m\n";

    // Create unique Clock Identity based on pseudo-MAC or host ID
    audio_core::network::PtpClockIdentity clock_id{};
    clock_id.id[0] = 0x00;
    clock_id.id[1] = 0x1A;
    clock_id.id[2] = 0x2B;
    clock_id.id[3] = 0xFF;
    clock_id.id[4] = 0xFE;
    clock_id.id[5] = 0x3C;
    clock_id.id[6] = 0x4D;
    clock_id.id[7] = 0x5E;

    audio_core::network::PtpBoundaryClock boundary(clock_id, priority1, priority2);

    // Port 0: Upstream (Default: slave to upstream Grandmaster, can be master)
    boundary.add_port(1, upstream_iface, true, true);
    // Port 1: Downstream (Boundary master distribution port to local audio nodes)
    boundary.add_port(2, downstream_iface, true, false);

    // Initialize Network Sockets
    audio_core::network::PtpBoundaryPortSocket upstream_sock;
    audio_core::network::PtpBoundaryPortSocket downstream_sock;

    bool up_ok = upstream_sock.open(upstream_iface, event_port, general_port, domain);
    if (!up_ok) {
        std::cerr << "\033[1;33m[WARN] Failed to bind upstream interface '" << upstream_iface
                  << "'. Falling back to 'lo' loopback.\033[0m\n";
        upstream_iface = "lo";
        up_ok = upstream_sock.open("lo", event_port, general_port, domain);
    }

    bool down_ok = downstream_sock.open(downstream_iface, event_port, general_port, domain);
    if (!down_ok) {
        std::cerr << "\033[1;33m[WARN] Failed to bind downstream interface '" << downstream_iface
                  << "'. Falling back to 'lo' loopback.\033[0m\n";
        downstream_iface = "lo";
        down_ok = downstream_sock.open("lo", event_port, general_port, domain);
    }

    std::cout << "\n[INIT] Boundary Clock Configuration:\n"
              << "  -> Domain: " << static_cast<int>(domain) << "\n"
              << "  -> Priority 1 / 2: " << static_cast<int>(priority1) << " / " << static_cast<int>(priority2) << "\n"
              << "  -> Upstream Interface:   " << upstream_iface
              << (upstream_sock.ts_engine().is_hardware_capable() ? " [PHY HW-TSTAMP]" : " [KERNEL/SW-TSTAMP]")
              << (up_ok ? " [ONLINE]" : " [OFFLINE]") << "\n"
              << "  -> Downstream Interface: " << downstream_iface
              << (downstream_sock.ts_engine().is_hardware_capable() ? " [PHY HW-TSTAMP]" : " [KERNEL/SW-TSTAMP]")
              << (down_ok ? " [ONLINE]" : " [OFFLINE]") << "\n"
              << "  -> Ports: Event=" << event_port << ", General=" << general_port << "\n"
              << "  -> Multicast Group: " << audio_core::network::kPtpPrimaryMulticastIp << "\n\n";

    std::cout << "\033[1;32m[STATUS] Daemon running. Press Ctrl+C to terminate.\033[0m\n\n";

    uint8_t rx_buffer[1024];
    uint8_t tx_buffer[1024];
    uint16_t sync_seq = 0;
    uint16_t announce_seq = 0;

    auto last_status_time = std::chrono::steady_clock::now();
    auto last_announce_time = std::chrono::steady_clock::now();
    auto last_sync_time = std::chrono::steady_clock::now();

    uint64_t last_t1 = 0;
    uint64_t last_t2 = 0;
    uint64_t last_t3 = 0;

    while (g_running.load()) {
        auto now = std::chrono::steady_clock::now();

        // 1. Process Upstream Sockets (Event & General)
        if (upstream_sock.is_open()) {
            sockaddr_in src_addr{};
            audio_core::network::PtpTimestampInfo ts_info{};

            // Ingress Event Messages (Sync)
            ssize_t bytes = upstream_sock.recv_event(rx_buffer, sizeof(rx_buffer), &src_addr, ts_info);
            if (bytes > 0) {
                auto parsed = audio_core::network::PtpBoundaryClock::parse_packet(rx_buffer, static_cast<size_t>(bytes));
                if (parsed.valid && parsed.type == audio_core::network::PtpMessageType::Sync) {
                    last_t2 = ts_info.rx_timestamp_ns;
                    last_t1 = parsed.timestamp_ns;
                    boundary.record_port_rx(0, audio_core::network::PtpMessageType::Sync, ts_info);

                    // Send Delay_Req to measure two-way path delay
                    uint64_t t3 = 0;
                    audio_core::network::PtpTimestampSource t3_src = audio_core::network::PtpTimestampSource::UserspaceMonotonic;
                    size_t req_len = audio_core::network::PtpBoundaryClock::build_delay_req_packet(
                        tx_buffer, sizeof(tx_buffer), boundary.clock_identity(), 1, sync_seq, 0
                    );
                    upstream_sock.send_event(tx_buffer, req_len, t3, t3_src,
                                             audio_core::network::kPtpPrimaryMulticastIp, event_port);
                    last_t3 = t3;
                }
            }

            // Ingress General Messages (Announce, Follow_Up, Delay_Resp)
            bytes = upstream_sock.recv_general(rx_buffer, sizeof(rx_buffer), &src_addr);
            if (bytes > 0) {
                auto parsed = audio_core::network::PtpBoundaryClock::parse_packet(rx_buffer, static_cast<size_t>(bytes));
                if (parsed.valid) {
                    if (parsed.type == audio_core::network::PtpMessageType::Announce) {
                        boundary.evaluate_announce(0, parsed.announce_vector);
                    } else if (parsed.type == audio_core::network::PtpMessageType::Follow_Up) {
                        last_t1 = parsed.timestamp_ns;
                    } else if (parsed.type == audio_core::network::PtpMessageType::Delay_Resp) {
                        if (parsed.requesting_clock_id == boundary.clock_identity()) {
                            uint64_t t4 = parsed.timestamp_ns;
                            if (last_t1 > 0 && last_t2 > 0 && last_t3 > 0 && t4 > 0) {
                                boundary.process_timing_exchange(0, last_t1, last_t2, last_t3, t4);
                            }
                        }
                    }
                }
            }
        }

        // 2. Periodic Downstream Transmissions (Distribution to audio slaves)
        if (downstream_sock.is_open() && boundary.port_state(1) == audio_core::network::PtpPortState::Master) {
            // Send Announce every 2 seconds
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_announce_time).count() >= 2) {
                last_announce_time = now;
                auto master_vec = boundary.announce_vector_for_master();
                size_t ann_len = audio_core::network::PtpBoundaryClock::build_announce_packet(
                    tx_buffer, sizeof(tx_buffer), boundary.clock_identity(), 2, ++announce_seq, master_vec
                );
                downstream_sock.send_general(tx_buffer, ann_len,
                                             audio_core::network::kPtpPrimaryMulticastIp, general_port);
                boundary.record_port_tx(1, audio_core::network::PtpMessageType::Announce);
            }

            // Send Sync + Follow_Up every 1 second
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_sync_time).count() >= 1000) {
                last_sync_time = now;
                uint64_t t1_egress = 0;
                audio_core::network::PtpTimestampSource t1_src = audio_core::network::PtpTimestampSource::UserspaceMonotonic;

                size_t sync_len = audio_core::network::PtpBoundaryClock::build_sync_packet(
                    tx_buffer, sizeof(tx_buffer), boundary.clock_identity(), 2, ++sync_seq, 0, true
                );
                downstream_sock.send_event(tx_buffer, sync_len, t1_egress, t1_src,
                                           audio_core::network::kPtpPrimaryMulticastIp, event_port);

                // Follow_Up with precise egress timestamp
                size_t fup_len = audio_core::network::PtpBoundaryClock::build_follow_up_packet(
                    tx_buffer, sizeof(tx_buffer), boundary.clock_identity(), 2, sync_seq, t1_egress
                );
                downstream_sock.send_general(tx_buffer, fup_len,
                                             audio_core::network::kPtpPrimaryMulticastIp, general_port);

                boundary.record_port_tx(1, audio_core::network::PtpMessageType::Sync);
            }
        }

        // 3. Periodic CLI Status Dashboard Output (Every 1 second)
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_status_time).count() >= 1000) {
            last_status_time = now;

            const auto& t0 = boundary.port_telemetry(0);
            const auto& t1 = boundary.port_telemetry(1);
            const auto& servo = boundary.servo();

            std::cout << "\r\033[K"
                      << "[PTPv2] "
                      << "P0(" << upstream_iface << "): " << audio_core::network::ptp_port_state_name(t0.state)
                      << " | P1(" << downstream_iface << "): " << audio_core::network::ptp_port_state_name(t1.state)
                      << " | Lock: " << (servo.is_locked() ? "\033[1;32mLOCKED\033[0m" : "\033[1;33mACQUIRING\033[0m")
                      << " | Offset: " << std::setw(6) << boundary.current_offset_ns() << " ns"
                      << " | Delay: " << std::setw(6) << t0.mean_path_delay_ns << " ns"
                      << " | Drift: " << std::fixed << std::setprecision(1) << servo.freq_drift_ppb() << " ppb"
                      << " | RX: " << (t0.sync_packets_rx + t0.announce_packets_rx)
                      << " | TX: " << (t1.sync_packets_tx + t1.announce_packets_tx)
                      << std::flush;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "\n\n\033[1;32m[SHUTDOWN] Closing PTPv2 Boundary Clock sockets.\033[0m\n";
    upstream_sock.close();
    downstream_sock.close();
    std::cout << "[DONE] Daemon stopped cleanly.\n";

    return 0;
}
