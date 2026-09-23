#include "audio_core/engine.hpp"
#include "audio_core/network/websocket_bridge.hpp"
#include "audio_core/dsp/purest_drive.hpp"
#include "audio_core/dsp/buttercomp2.hpp"
#include "audio_core/dsp/baxandall.hpp"
#include "audio_core/dsp/clip_only2.hpp"
#include "backends/desktop/desktop_backend.hpp"

#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

static std::atomic<bool> g_running{true};

static void sigint_handler(int) {
    g_running.store(false);
}

int main(int argc, char** argv) {
    std::signal(SIGINT, sigint_handler);
    std::signal(SIGTERM, sigint_handler);

    uint16_t port = 8088;
    if (argc > 1) {
        port = static_cast<uint16_t>(std::atoi(argv[1]));
    }

    std::cout << "==========================================================" << std::endl;
    std::cout << "  AETHEL SOVEREIGN AUDIO ENGINE // WEBMIXER DAEMON [RT]   " << std::endl;
    std::cout << "==========================================================" << std::endl;

    constexpr uint32_t kSampleRate = 48000;
    constexpr uint32_t kBufferSize = 512;

    audio_core::Engine engine(kSampleRate, kBufferSize);

    // 1. Allocate demo multitrack strips with Airwindows modeled DSPs
    auto* trk1 = engine.mixer().allocate_track("808 Kick & Sub");
    auto* trk2 = engine.mixer().allocate_track("Acid Synth 303");
    auto* trk3 = engine.mixer().allocate_track("Percussion Bus");
    auto* trk4 = engine.mixer().allocate_track("Master Stem L/R");
    (void)trk3;
    (void)trk4;

    auto eq = std::make_shared<audio_core::dsp::Baxandall>();
    eq->init(kSampleRate);
    eq->set_parameter(0, 2.5f); // +2.5 dB LF warmth
    trk1->slot(0).set_processor(eq);

    auto comp = std::make_shared<audio_core::dsp::ButterComp2>();
    comp->init(kSampleRate);
    comp->set_parameter(0, 0.45f); // Buttery gain reduction
    trk1->slot(1).set_processor(comp);

    auto drive = std::make_shared<audio_core::dsp::PurestDrive>();
    drive->init(kSampleRate);
    drive->set_parameter(0, 0.5f); // Harmonic saturation
    trk2->slot(0).set_processor(drive);

    auto limiter = std::make_shared<audio_core::dsp::ClipOnly2>();
    limiter->init(kSampleRate);
    engine.mixer().master_bus().slot(0).set_processor(limiter);

    // 2. Attach Desktop Audio Output Backend (miniaudio / ALSA / PulseAudio)
    auto backend = std::make_unique<audio_core::DesktopBackend>();
    engine.attach_backend(std::move(backend));
    if (!engine.start()) {
        std::cerr << "[Warning] Could not initialize physical audio output device. Running headless DSP mode." << std::endl;
    }

    // 3. Launch WebSocket Bridge
    audio_core::network::WebSocketBridge::Config ws_cfg{};
    ws_cfg.port = port;
    ws_cfg.host = "0.0.0.0";
    ws_cfg.telemetry_rate_hz = 30; // 30 FPS UI refresh
    ws_cfg.serve_embedded_gui = true;

    audio_core::network::WebSocketBridge ws_bridge(engine, ws_cfg);
    if (!ws_bridge.start()) {
        std::cerr << "[Error] Failed to start WebSocket Bridge on port " << port << std::endl;
        return 1;
    }

    std::cout << "\n>>> [WEBMIXER READY] <<<" << std::endl;
    std::cout << " * Web UI:      http://localhost:" << port << " / http://127.0.0.1:" << port << std::endl;
    std::cout << " * WebSocket:   ws://localhost:" << port << std::endl;
    std::cout << " * Rate:        30 Hz real-time telemetry streaming" << std::endl;
    std::cout << " * Protocols:   JSON text commands + 32-byte POD MixerCommand binary" << std::endl;
    std::cout << "\nOpen http://localhost:" << port << " in your browser to mix audio and trigger synth voices." << std::endl;
    std::cout << "Press Ctrl+C to terminate.\n" << std::endl;

    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "\n[Shutdown] Stopping WebMixer daemon..." << std::endl;
    ws_bridge.stop();
    engine.stop();
    std::cout << "[Shutdown] Clean exit." << std::endl;
    return 0;
}
