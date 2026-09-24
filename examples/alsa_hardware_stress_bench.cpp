#include "audio_core/types.hpp"
#include "audio_core/mixer_graph.hpp"
#include "audio_core/midi/hardware_midi_receiver.hpp"
#include "audio_core/modulation/polyphonic_synth.hpp"
#include "audio_core/modulation/modulation_matrix.hpp"
#include "audio_core/sampling/audio_clip.hpp"
#include "audio_core/sampling/vari_speed_streamer.hpp"
#include "audio_core/dsp/purest_drive.hpp"
#include "audio_core/dsp/buttercomp2.hpp"

#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <numbers>

using namespace audio_core;
using namespace audio_core::midi;
using namespace audio_core::sampling;

// ============================================================================
// ALSA Hardware Loopback & Real-Time Audio Engine Stress Benchmark
// Saturates Linux ALSA Sequencer Kernel Subsystem (/dev/snd/seq and Midi Through 14:0)
// concurrently with multi-track real-time audio rendering (PolySynth + Effects + Scrubbing).
// ============================================================================

struct BenchConfig {
    uint32_t total_events = 50000;
    uint32_t sample_rate = 48000;
    uint32_t buffer_frames = 128; // 2.667 ms real-time deadline
    uint32_t num_tracks = 8;
    uint32_t batch_size = 250;
    uint32_t pacing_us = 2500;    // ~100,000 events/sec pace
    bool use_midi_through = true;
};

struct AudioStats {
    uint64_t total_blocks{0};
    double min_us{1e9};
    double max_us{0.0};
    double sum_us{0.0};
    uint64_t xrun_count{0};
    float max_peak{0.0f};
    bool has_nan_or_inf{false};
    uint64_t synth_events_drained{0};
};

int main(int argc, char* argv[]) {
    BenchConfig cfg;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--events" && i + 1 < argc) {
            cfg.total_events = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--tracks" && i + 1 < argc) {
            cfg.num_tracks = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--buffer" && i + 1 < argc) {
            cfg.buffer_frames = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--direct") {
            cfg.use_midi_through = false;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: alsa_hardware_stress_bench [OPTIONS]\n"
                      << "Options:\n"
                      << "  --events <N>     Total MIDI events to blast (default: 50000)\n"
                      << "  --tracks <N>     Active mixer tracks under load (default: 8)\n"
                      << "  --buffer <N>     Audio block buffer size in frames (default: 128)\n"
                      << "  --direct         Use direct ALSA port delivery instead of 14:0 Midi Through\n"
                      << "  --help           Show this help message\n";
            return 0;
        }
    }

    std::cout << "========================================================================================\n";
    std::cout << "       AETHELNET AUDIO CORE // ALSA HARDWARE LOOPBACK STRESS BENCHMARK                  \n";
    std::cout << "========================================================================================\n\n";

    std::cout << "  Configuration:\n";
    std::cout << "    - Target Events:        " << cfg.total_events << " kernel events\n";
    std::cout << "    - Audio Sample Rate:    " << cfg.sample_rate << " Hz\n";
    std::cout << "    - Audio Block Size:     " << cfg.buffer_frames << " frames ("
              << std::fixed << std::setprecision(3) << (1000.0 * cfg.buffer_frames / cfg.sample_rate) << " ms budget)\n";
    std::cout << "    - Mixer Tracks:         " << cfg.num_tracks << " tracks (PolySynth, Scrubbing Stem, Buses)\n";
    std::cout << "    - ALSA Routing:         " << (cfg.use_midi_through ? "Kernel Loopback (client 14:0 Midi Through)" : "Direct Port Addressing") << "\n\n";

    // ------------------------------------------------------------------------
    // Step 1: Initialize Hardware MIDI Receiver (65536 event capacity)
    // ------------------------------------------------------------------------
    HardwareMidiReceiver rx(65536);
    if (!rx.open_alsa_sequencer("Aethel Stress RX", "Stress MIDI In")) {
        std::cerr << "[ERROR] Failed to open ALSA Sequencer receiver on /dev/snd/seq!\n";
        return 1;
    }

    std::cout << "[ALSA RX] Opened client " << rx.seq_client_id() << ", port " << rx.seq_port_id()
              << " (" << rx.seq_port_name() << ")\n";
    std::cout << "[ALSA RX] Kernel input queue pool maximized to 2000 cells.\n";
    std::cout << "[ALSA RX] Subscribed to Midi Through 14:0: " << (rx.is_subscribed(14, 0) ? "YES" : "NO") << "\n\n";

    // ------------------------------------------------------------------------
    // Step 2: Initialize ALSA Stress Transmitter on /dev/snd/seq
    // ------------------------------------------------------------------------
    int tx_fd = ::open("/dev/snd/seq", O_RDWR | O_CLOEXEC);
    if (tx_fd < 0) {
        std::cerr << "[ERROR] Failed to open /dev/snd/seq for transmitter: " << strerror(errno) << "\n";
        return 1;
    }

    int tx_client = -1;
    if (ioctl(tx_fd, SNDRV_SEQ_IOCTL_CLIENT_ID, &tx_client) < 0) {
        std::cerr << "[ERROR] Failed to get transmitter client ID!\n";
        ::close(tx_fd);
        return 1;
    }

    struct snd_seq_client_info c_info{};
    c_info.client = tx_client;
    if (ioctl(tx_fd, SNDRV_SEQ_IOCTL_GET_CLIENT_INFO, &c_info) >= 0) {
        std::strncpy(c_info.name, "Aethel Stress TX", sizeof(c_info.name) - 1);
        (void)ioctl(tx_fd, SNDRV_SEQ_IOCTL_SET_CLIENT_INFO, &c_info);
    }

    struct snd_seq_port_info p_info{};
    p_info.addr.client = static_cast<unsigned char>(tx_client);
    p_info.capability = SNDRV_SEQ_PORT_CAP_WRITE | SNDRV_SEQ_PORT_CAP_SUBS_WRITE | SNDRV_SEQ_PORT_CAP_READ;
    p_info.type = SNDRV_SEQ_PORT_TYPE_MIDI_GENERIC | SNDRV_SEQ_PORT_TYPE_APPLICATION;
    std::strncpy(p_info.name, "Stress TX Out", sizeof(p_info.name) - 1);
    if (ioctl(tx_fd, SNDRV_SEQ_IOCTL_CREATE_PORT, &p_info) < 0) {
        std::cerr << "[ERROR] Failed to create transmitter port!\n";
        ::close(tx_fd);
        return 1;
    }
    int tx_port = p_info.addr.port;

    // Maximize transmitter output pool to 2000 cells
    struct snd_seq_client_pool pool{};
    pool.client = tx_client;
    if (ioctl(tx_fd, SNDRV_SEQ_IOCTL_GET_CLIENT_POOL, &pool) >= 0) {
        pool.output_pool = 2000;
        pool.input_pool = 2000;
        pool.output_room = 1;
        (void)ioctl(tx_fd, SNDRV_SEQ_IOCTL_SET_CLIENT_POOL, &pool);
    }

    std::cout << "[ALSA TX] Transmitter initialized: client " << tx_client << ", port " << tx_port << "\n";
    std::cout << "[ALSA TX] Kernel output pool maximized to 2000 cells.\n\n";

    // ------------------------------------------------------------------------
    // Step 3: Configure Multi-Track Audio Engine (MixerGraph)
    // ------------------------------------------------------------------------
    MixerGraph mixer(cfg.buffer_frames, false, cfg.sample_rate);
    mixer.clock().set_bpm(135.0);
    mixer.clock().set_playing(true);

    // Track 0: PolySynth + ModulationMatrix + Insert Slots (PurestDrive + ButterComp2)
    modulation::ModulationMatrix mod_matrix;
    mod_matrix.init(cfg.sample_rate);
    mod_matrix.poly_synth().set_polyphony_limit(16);
    mod_matrix.poly_synth().set_play_mode(modulation::PolyphonyPlayMode::Polyphonic);
    mod_matrix.poly_synth().set_osc1_waveform(dsp::Waveform::Saw);
    mod_matrix.poly_synth().set_osc2_waveform(dsp::Waveform::Square);
    mod_matrix.poly_synth().set_osc_mix(0.6f);
    mod_matrix.poly_synth().set_base_cutoff(2200.0f);
    mod_matrix.poly_synth().set_resonance_q(2.5f);

    auto trk_synth = mixer.add_track("Lead Synth");
    trk_synth->set_input_mode(TrackInputMode::PolySynth);
    trk_synth->set_poly_synth(&mod_matrix.poly_synth(), &mod_matrix);
    trk_synth->slot(0).set_processor(std::make_shared<dsp::PurestDrive>());
    trk_synth->slot(0).processor()->set_parameter(0, 0.4f); // Subtle saturation
    trk_synth->slot(1).set_processor(std::make_shared<dsp::ButterComp2>());

    // Track 1: VariSpeedStreamer Audio Stem with Dynamic Granular Micro-Windowed Scrubbing
    constexpr size_t kStemFrames = 48000 * 4; // 4-second synthesized stem
    auto stem_clip = std::make_shared<AudioClip>("TestStem", 48000, 2, static_cast<uint32_t>(kStemFrames));
    for (size_t i = 0; i < kStemFrames; ++i) {
        float t = static_cast<float>(i) / 48000.0f;
        float s = 0.5f * std::sin(2.0f * std::numbers::pi_v<float> * 220.0f * t)
                + 0.3f * std::sin(2.0f * std::numbers::pi_v<float> * 440.0f * t);
        stem_clip->channel(0)[i] = s;
        stem_clip->channel(1)[i] = s;
    }
    auto trk_scrub = mixer.add_track("Scrub Stem");
    trk_scrub->set_clip(stem_clip);
    trk_scrub->set_sync_to_transport(true);

    // Remaining tracks: Summing Buses & Aux Channels
    for (uint32_t t = 2; t < cfg.num_tracks; ++t) {
        mixer.add_track("Bus " + std::to_string(t));
    }

    std::cout << "[AUDIO] MixerGraph configured with " << mixer.track_count() << " tracks under RT processing.\n\n";

    // ------------------------------------------------------------------------
    // Step 4: Launch Concurrent Audio Render Loop & ALSA Stress Transmitter
    // ------------------------------------------------------------------------
    std::atomic<bool> stress_running{true};
    AudioStats audio_stats;
    std::vector<double> block_times;
    block_times.reserve(10000);

    const double budget_us = 1e6 * cfg.buffer_frames / static_cast<double>(cfg.sample_rate);

    // Audio thread: continuously renders 128-frame blocks while draining MIDI
    std::thread audio_thread([&]() {
        AudioBuffer master_buf(2, cfg.buffer_frames);
        auto master_view = master_buf.view();
        uint64_t block_idx = 0;

        while (stress_running.load(std::memory_order_relaxed)) {
            auto t_start = std::chrono::high_resolution_clock::now();

            // 1. Drain arriving ALSA MIDI events into ModulationMatrix & PolySynth lock-free
            size_t drained = rx.drain_to(mod_matrix);
            audio_stats.synth_events_drained += drained;

            // 2. Modulate scrub position & velocity to stress Granular Micro-Windowing & DC Blocker
            double scrub_phase = static_cast<double>(block_idx) * 0.05;
            double scrub_target = 48000.0 + 24000.0 * std::sin(scrub_phase);
            double scrub_vel = 1.2 * std::cos(scrub_phase); // Oscillating speed with direction reversals
            mixer.update_scrub(static_cast<uint64_t>(std::clamp(scrub_target, 0.0, static_cast<double>(kStemFrames - 100))), scrub_vel);

            // 3. Render complete 8-track audio mixer graph
            mixer.render(master_view);

            auto t_end = std::chrono::high_resolution_clock::now();
            double dur_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();
            if (block_idx >= 10) {
                audio_stats.total_blocks++;
                audio_stats.sum_us += dur_us;
                if (dur_us < audio_stats.min_us) audio_stats.min_us = dur_us;
                if (dur_us > audio_stats.max_us) audio_stats.max_us = dur_us;
                if (dur_us > budget_us) {
                    audio_stats.xrun_count++;
                    std::cout << "  [AUDIO WARNING] Block " << block_idx << " took "
                              << std::fixed << std::setprecision(1) << dur_us
                              << " us > budget " << budget_us << " us (drained "
                              << drained << " events)\n";
                }
                block_times.push_back(dur_us);
            }

            // Check audio buffer integrity (no NaN, no Inf)
            for (uint32_t f = 0; f < cfg.buffer_frames; ++f) {
                float l = master_view.channel(0)[f];
                float r = master_view.channel(1)[f];
                if (std::isnan(l) || std::isnan(r) || std::isinf(l) || std::isinf(r)) {
                    audio_stats.has_nan_or_inf = true;
                }
                float pk = std::max(std::abs(l), std::abs(r));
                if (pk > audio_stats.max_peak) audio_stats.max_peak = pk;
            }

            block_idx++;
            // Pace audio blocks to real-time deadline
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(budget_us * 0.85)));
        }
    });

    // Brief warmup period for audio thread before launching stress flood
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Stress thread: blasts rich MIDI messages through ALSA sequencer
    std::cout << "[STRESS] Beginning ALSA hardware saturation stream (" << cfg.total_events << " events)...\n";
    auto t_stress_start = std::chrono::high_resolution_clock::now();

    const uint32_t batch_size = cfg.batch_size;
    const uint32_t num_batches = cfg.total_events / batch_size;
    std::vector<struct snd_seq_event> batch(batch_size);
    uint32_t total_sent = 0;

    const unsigned char dest_client = cfg.use_midi_through ? 14 : static_cast<unsigned char>(rx.seq_client_id());
    const unsigned char dest_port = cfg.use_midi_through ? 0 : static_cast<unsigned char>(rx.seq_port_id());

    for (uint32_t b = 0; b < num_batches; ++b) {
        for (uint32_t i = 0; i < batch_size; ++i) {
            uint32_t ev_idx = b * batch_size + i;
            struct snd_seq_event& ev = batch[i];
            ev = {};
            ev.queue = SNDRV_SEQ_QUEUE_DIRECT;
            ev.source.client = static_cast<unsigned char>(tx_client);
            ev.source.port = static_cast<unsigned char>(tx_port);
            ev.dest.client = dest_client;
            ev.dest.port = dest_port;

            // Generate realistic high-density mix of MIDI event types:
            uint32_t pattern = ev_idx % 8;
            switch (pattern) {
                case 0: // Note-On Chord Voicing (voices 0..7)
                case 1:
                    ev.type = SNDRV_SEQ_EVENT_NOTEON;
                    ev.data.note.channel = ev_idx % 4;
                    ev.data.note.note = 36 + ((ev_idx * 7) % 52); // Dense chord distribution across C2..E6
                    ev.data.note.velocity = 60 + (ev_idx % 67);
                    break;
                case 2: // Note-Off
                    ev.type = SNDRV_SEQ_EVENT_NOTEOFF;
                    ev.data.note.channel = ev_idx % 4;
                    ev.data.note.note = 36 + (((ev_idx - 2) * 7) % 52);
                    ev.data.note.velocity = 0;
                    break;
                case 3: // Fast 14-bit Pitch Bend Modulation Sweep
                    ev.type = SNDRV_SEQ_EVENT_PITCHBEND;
                    ev.data.control.channel = ev_idx % 4;
                    ev.data.control.value = static_cast<int>(8191.0 * std::sin(static_cast<double>(ev_idx) * 0.1));
                    break;
                case 4: // Mod Wheel CC 1 Sweep
                    ev.type = SNDRV_SEQ_EVENT_CONTROLLER;
                    ev.data.control.channel = ev_idx % 4;
                    ev.data.control.param = 1;
                    ev.data.control.value = static_cast<int>(64.0 + 63.0 * std::sin(static_cast<double>(ev_idx) * 0.05));
                    break;
                case 5: // Expression CC 11 Sweep
                    ev.type = SNDRV_SEQ_EVENT_CONTROLLER;
                    ev.data.control.channel = ev_idx % 4;
                    ev.data.control.param = 11;
                    ev.data.control.value = static_cast<int>(64.0 + 63.0 * std::cos(static_cast<double>(ev_idx) * 0.08));
                    break;
                case 6: // Real-Time 24 PPQN Beat Clock
                    ev.type = SNDRV_SEQ_EVENT_CLOCK;
                    break;
                case 7: // MTC Quarter-Frame
                    ev.type = SNDRV_SEQ_EVENT_QFRAME;
                    ev.data.control.value = (ev_idx % 8) << 4;
                    break;
            }
        }

        size_t bytes_to_write = batch_size * sizeof(struct snd_seq_event);
        size_t written = 0;
        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(batch.data());

        while (written < bytes_to_write) {
            ssize_t w = ::write(tx_fd, ptr + written, bytes_to_write - written);
            if (w > 0) {
                written += w;
            } else if (errno == EAGAIN || errno == ENOSPC) {
                std::this_thread::yield();
            } else {
                break;
            }
        }
        total_sent += batch_size;
        std::this_thread::sleep_for(std::chrono::microseconds(cfg.pacing_us));
    }

    auto t_stress_end = std::chrono::high_resolution_clock::now();
    double stress_elapsed_ms = std::chrono::duration<double, std::milli>(t_stress_end - t_stress_start).count();

    // Allow brief drain period for remaining events in the kernel buffer
    for (int wait = 0; wait < 100; ++wait) {
        if (rx.event_count() >= static_cast<uint64_t>(total_sent)) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    stress_running.store(false, std::memory_order_release);
    audio_thread.join();

    uint64_t total_received = rx.event_count();
    uint64_t total_dropped = rx.dropped_events();
    double total_throughput = total_sent / (stress_elapsed_ms / 1000.0);

    // Compute percentile metrics for audio thread
    std::sort(block_times.begin(), block_times.end());
    double median_us = block_times.empty() ? 0.0 : block_times[block_times.size() / 2];
    double p99_us = block_times.empty() ? 0.0 : block_times[static_cast<size_t>(block_times.size() * 0.99)];
    double avg_us = audio_stats.total_blocks > 0 ? (audio_stats.sum_us / audio_stats.total_blocks) : 0.0;
    double headroom_pct = 100.0 * (1.0 - (audio_stats.max_us / budget_us));

    // ------------------------------------------------------------------------
    // Step 5: Output Forensic Telemetry Dashboard
    // ------------------------------------------------------------------------
    std::cout << "\n========================================================================================\n";
    std::cout << "                         FORENSIC TELEMETRY DASHBOARD                                   \n";
    std::cout << "========================================================================================\n";

    std::cout << "\n[1] ALSA KERNEL SEQUENCER THROUGHPUT:\n";
    std::cout << "  - Events Transmitted:       " << total_sent << " events\n";
    std::cout << "  - Events Received by RX:    " << total_received << " events\n";
    std::cout << "  - Reception Success Rate:   " << std::fixed << std::setprecision(2)
              << (100.0 * total_received / total_sent) << " %\n";
    std::cout << "  - Kernel Dropped Events:    " << total_dropped << " (0 dropped = PERFECT)\n";
    std::cout << "  - Saturation Burst Time:    " << std::fixed << std::setprecision(2) << stress_elapsed_ms << " ms\n";
    std::cout << "  - Kernel Sustained Rate:    " << std::fixed << std::setprecision(1) << total_throughput << " events/sec\n";

    std::cout << "\n[2] SYNTHESIS & MODULATION DISPATCH:\n";
    std::cout << "  - MIDI Events Drained to Synth: " << audio_stats.synth_events_drained << " events\n";
    std::cout << "  - Active Polyphony Limit:       " << mod_matrix.poly_synth().polyphony_limit() << " voices\n";
    std::cout << "  - Max Master Output Peak:       " << std::fixed << std::setprecision(4) << audio_stats.max_peak << "\n";
    std::cout << "  - Numerical Safety (No NaN/Inf): " << (!audio_stats.has_nan_or_inf ? "VERIFIED (100% CLEAN)" : "FAILED (NaN/Inf Detected!)") << "\n";

    std::cout << "\n[3] REAL-TIME AUDIO ENGINE STABILITY:\n";
    std::cout << "  - Real-Time Budget per Block:  " << std::fixed << std::setprecision(1) << budget_us << " us (" << cfg.buffer_frames << " frames @ " << cfg.sample_rate << " Hz)\n";
    std::cout << "  - Total Blocks Rendered:       " << audio_stats.total_blocks << " blocks\n";
    std::cout << "  - Min Render Time:             " << std::fixed << std::setprecision(1) << audio_stats.min_us << " us\n";
    std::cout << "  - Avg Render Time:             " << std::fixed << std::setprecision(1) << avg_us << " us\n";
    std::cout << "  - Median (p50) Render Time:    " << std::fixed << std::setprecision(1) << median_us << " us\n";
    std::cout << "  - 99th Percentile (p99):       " << std::fixed << std::setprecision(1) << p99_us << " us\n";
    std::cout << "  - Worst-Case (Max) Block:      " << std::fixed << std::setprecision(1) << audio_stats.max_us << " us\n";
    std::cout << "  - Worst-Case Headroom:         " << std::fixed << std::setprecision(1) << headroom_pct << " % margin\n";
    std::cout << "  - Audio Deadline Misses/XRuns: " << audio_stats.xrun_count << "\n";

    std::cout << "\n========================================================================================\n";
    bool bench_passed = (total_received == total_sent) && (total_dropped == 0) &&
                        !audio_stats.has_nan_or_inf && (audio_stats.max_peak > 0.05f) &&
                        (audio_stats.xrun_count == 0);

    if (bench_passed) {
        std::cout << "  >>> BENCHMARK STATUS: 100% PASSED (ROCK-SOLID HARDWARE RESILIENCE) <<<\n";
    } else {
        std::cout << "  >>> BENCHMARK STATUS: COMPLETED WITH WARNINGS <<<\n";
    }
    std::cout << "========================================================================================\n\n";

    ::close(tx_fd);
    return bench_passed ? 0 : 1;
}
