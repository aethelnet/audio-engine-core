#include "audio_core/mixer_graph.hpp"
#include "audio_core/insert_slot.hpp"
#include "audio_core/dsp/purest_drive.hpp"
#include "audio_core/dsp/buttercomp2.hpp"
#include "audio_core/dsp/baxandall.hpp"
#include "audio_core/dsp/clip_only2.hpp"
#include "backends/pipewire/pipewire_backend.hpp"

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

static std::atomic<bool> g_running{true};

static void sigint_handler(int) {
    g_running.store(false);
}

int main() {
    std::signal(SIGINT, sigint_handler);
    std::signal(SIGTERM, sigint_handler);

    std::cout << "==========================================================" << std::endl;
    std::cout << "  AETHEL PIPEWIRE MULTITRACK CONSOLE DESK (AIRWINDOWS DSP)" << std::endl;
    std::cout << "==========================================================" << std::endl;

    constexpr uint32_t kFrames = 1024;
    constexpr uint32_t kSampleRate = 48000;

    audio_core::MixerGraph mixer(kFrames);

    // 1. Allocate initial tracks for external apps
    auto* trk1 = mixer.allocate_track("External App 1");
    auto* trk2 = mixer.allocate_track("External App 2");
    auto* trk3 = mixer.allocate_track("Synth Track");
    auto* trk4 = mixer.allocate_track("Vocal / Guitar");
    (void)trk3;
    (void)trk4;

    // 2. Insert Airwindows Analog Strip DSPs
    auto eq = std::make_shared<audio_core::dsp::Baxandall>();
    eq->init(kSampleRate);
    eq->set_parameter(0, 3.0f); // +3dB Low-shelf warmth
    trk1->slot(0).set_processor(eq);

    auto comp = std::make_shared<audio_core::dsp::ButterComp2>();
    comp->init(kSampleRate);
    comp->set_parameter(0, 0.4f); // Buttery RMS compression
    trk1->slot(1).set_processor(comp);

    auto drive = std::make_shared<audio_core::dsp::PurestDrive>();
    drive->init(kSampleRate);
    drive->set_parameter(0, 0.6f); // Harmonic saturation
    trk2->slot(0).set_processor(drive);

    auto clipper = std::make_shared<audio_core::dsp::ClipOnly2>();
    clipper->init(kSampleRate);
    mixer.master_bus().slot(0).set_processor(clipper);

    std::cout << "[Init] Pre-allocated channel pool: 32 Tracks, 16 Buses." << std::endl;
    std::cout << "[Init] Airwindows Insert Strip engaged (Baxandall EQ, ButterComp2, PurestDrive, ClipOnly2)." << std::endl;

    // 3. Connect to PipeWire daemon and open virtual input sinks
    audio_core::PipeWireBackend pw(mixer);
    if (!pw.init("Aethel Console Desk", kSampleRate)) {
        std::cerr << "[Error] Failed to initialize PipeWire filter node!" << std::endl;
        return 1;
    }

    if (!pw.start()) {
        std::cerr << "[Error] Failed to start PipeWire stream!" << std::endl;
        return 1;
    }

    std::cout << "\n>>> [PIPEWIRE RUNNING] <<<" << std::endl;
    std::cout << " * Virtual Input Sinks exposed in PipeWire graph:" << std::endl;
    std::cout << "   - Track 1 In L / R (Baxandall EQ + ButterComp2)" << std::endl;
    std::cout << "   - Track 2 In L / R (PurestDrive Saturation)" << std::endl;
    std::cout << "   - Track 3 In L / R (Direct Linear Console)" << std::endl;
    std::cout << "   - Track 4 In L / R (Direct Linear Console)" << std::endl;
    std::cout << " * Master Out L / R auto-connected to system sink." << std::endl;
    std::cout << "\nYou can now patch ANY app (Bitwig, Renoise, Chrome, Spotify, VCV Rack)" << std::endl;
    std::cout << "using Helvum, qpwgraph, or pw-link into 'aethel_mixer_graph'!" << std::endl;
    std::cout << "Press Ctrl+C to terminate.\n" << std::endl;

    audio_core::protocol::MixerTelemetryFrame telemetry{};

    int ticker = 0;
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        mixer.capture_telemetry_snapshot(telemetry);

        if (++ticker % 4 == 0) {
            std::cout << "\r[Telemetry] Cycles: " << std::setw(8) << telemetry.render_cycle
                      << " | Master Peak: L=" << std::fixed << std::setprecision(2) << telemetry.master_meter.peak_l
                      << " R=" << telemetry.master_meter.peak_r
                      << " | Trk1 Peak: " << telemetry.track_meters[0].peak_l
                      << " | Trk2 Peak: " << telemetry.track_meters[1].peak_l
                      << std::flush;
        }
    }

    std::cout << "\n\nShutting down PipeWire node cleanly..." << std::endl;
    pw.stop();
    std::cout << "Done." << std::endl;
    return 0;
}
