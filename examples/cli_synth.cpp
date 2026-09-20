#include "audio_core/engine.hpp"
#include "backends/desktop/desktop_backend.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "   AUDIO-ENGINE-CORE: Polyphonic Synth   " << std::endl;
    std::cout << "========================================" << std::endl;

    const uint32_t sample_rate = 48000;
    const uint32_t buffer_size = 256; // ~5.3 ms latency @ 48kHz
    const uint32_t channels = 2;

    auto engine = std::make_unique<audio_core::Engine>(sample_rate, buffer_size);
    auto backend = std::make_unique<audio_core::DesktopBackend>();

    if (!backend->init(sample_rate, channels, buffer_size)) {
        std::cerr << "Failed to initialize desktop audio backend!" << std::endl;
        return 1;
    }

    std::cout << "[Audio Engine] Actual Sample Rate: " << backend->actual_sample_rate() << " Hz" << std::endl;
    std::cout << "[Audio Engine] Buffer Size: " << buffer_size << " frames ("
              << (static_cast<double>(buffer_size) / backend->actual_sample_rate() * 1000.0) << " ms latency)" << std::endl;

    backend->set_callback([&engine](audio_core::Sample* output, uint32_t frames, uint32_t ch) {
        engine->process_interleaved(output, frames, ch);
    });

    if (!backend->start()) {
        std::cerr << "Failed to start audio playback!" << std::endl;
        return 1;
    }

    std::cout << "\n[Audio Engine] PLAYBACK STARTED!" << std::endl;

    // Load Sandboxed WASM DSP Saturator Plugin
    if (engine->load_master_plugin("plugins/saturator/saturator.wasm")) {
        std::cout << "[WasmHost] Successfully loaded master effect: plugins/saturator/saturator.wasm" << std::endl;
    } else {
        std::cerr << "[WasmHost] Warning: Failed to load saturator.wasm" << std::endl;
    }

    // Chord progression in C minor: Cm9 -> Abmaj7 -> Fm9 -> G7sus4
    struct ChordStep {
        std::vector<uint8_t> notes;
        int duration_ms;
        float cutoff;
    };

    std::vector<ChordStep> progression = {
        {{48, 55, 58, 62, 63}, 750, 1400.0f}, // Cm9 (C3, G3, Bb3, D4, Eb4)
        {{44, 51, 55, 58, 63}, 750, 2600.0f}, // Abmaj7 (Ab2, Eb3, G3, Bb3, Eb4)
        {{41, 48, 53, 56, 60}, 750, 4200.0f}, // Fm9 (F2, C3, F3, Ab3, C4)
        {{43, 50, 53, 58, 62}, 750, 6800.0f}, // G7sus4 (G2, D3, F3, Bb3, D4)
        {{48, 55, 58, 62, 63}, 1000, 2000.0f} // Final Cm9 resolve
    };

    for (int loop = 0; loop < 2; ++loop) {
        if (loop == 0) {
            std::cout << "\n>>> LOOP 1: Pure Native Synth (WASM Plugin BYPASSED / DRY) <<<" << std::endl;
            engine->set_plugin_parameter(3, 0.0f); // Mix = 0%
        } else {
            std::cout << "\n>>> LOOP 2: Sandboxed WASM Tube Saturator ENGAGED (WET) <<<" << std::endl;
            engine->set_plugin_parameter(1, 4.0f); // Drive = 4.0x
            engine->set_plugin_parameter(2, 0.5f); // Tone = 0.5
            engine->set_plugin_parameter(3, 0.85f); // Mix = 85% Wet
        }

        for (const auto& step : progression) {
            engine->set_parameter(2, step.cutoff);

            for (uint8_t note : step.notes) {
                engine->note_on(note, 100);
            }

            std::cout << "  Chord [" << static_cast<int>(step.notes[0]) << "] | Cutoff: "
                      << step.cutoff << " Hz | RT CPU Load: "
                      << engine->cpu_load_percent() << " %" << std::endl;

            std::this_thread::sleep_for(std::chrono::milliseconds(step.duration_ms));

            for (uint8_t note : step.notes) {
                engine->note_off(note);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    std::cout << "\nDemo sequence finished. Stopping audio stream..." << std::endl;
    backend->stop();
    std::cout << "Done." << std::endl;

    return 0;
}
