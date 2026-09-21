#include "audio_core/mixer_graph.hpp"
#include "audio_core/insert_slot.hpp"
#include "audio_core/dsp/purest_drive.hpp"
#include "audio_core/dsp/buttercomp2.hpp"
#include "audio_core/dsp/baxandall.hpp"
#include "audio_core/dsp/clip_only2.hpp"
#include "audio_core/protocol/command_packet.hpp"
#include "audio_core/protocol/telemetry_packet.hpp"
#include "audio_core/ui/theme.hpp"
#include "audio_core/ui/custom_widgets.hpp"
#include "audio_core/sampling/wav_reader.hpp"
#include "audio_core/sampling/sample_repair.hpp"
#include "audio_core/dsp/time_stretcher.hpp"
#include "audio_core/dsp/derez.hpp"
#include "backends/pipewire/pipewire_backend.hpp"


#include <GLFW/glfw3.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <chrono>
#include <algorithm>

using namespace audio_core;

// Synthetic drum loop generator
static std::vector<float> generate_synthetic_drum_loop(size_t num_samples) {
    std::vector<float> buf(num_samples, 0.0f);
    for (size_t i = 0; i < num_samples; ++i) {
        float t = static_cast<float>(i) / 48000.0f;
        float beat = std::fmod(t * 2.0f, 1.0f); // 120 BPM quarter note

        // Kick transient on quarter notes
        float kick = std::exp(-beat * 24.0f) * std::sin(2.0f * 3.14159265f * 55.0f * (1.0f - beat * 0.7f) * t);
        // Snare on 2 and 4
        float snare_t = std::fmod((t + 0.25f) * 2.0f, 1.0f);
        float noise = (static_cast<float>(std::rand()) / RAND_MAX * 2.0f - 1.0f);
        float snare = (snare_t < 0.2f) ? std::exp(-snare_t * 30.0f) * noise * 0.7f : 0.0f;
        // Hihat on 8th notes
        float hat_t = std::fmod(t * 4.0f, 1.0f);
        float hat = (hat_t < 0.08f) ? std::exp(-hat_t * 60.0f) * noise * 0.35f : 0.0f;

        buf[i] = std::clamp(kick + snare + hat, -1.0f, 1.0f);
    }
    return buf;
}

// Synthetic 303 acid lead generator
static std::vector<float> generate_synthetic_acid_loop(size_t num_samples) {
    std::vector<float> buf(num_samples, 0.0f);
    float phase = 0.0f;
    float filter_state = 0.0f;
    const float notes[16] = { 65.4f, 65.4f, 130.8f, 65.4f, 98.0f, 65.4f, 116.5f, 130.8f,
                              65.4f, 87.3f, 65.4f, 130.8f, 98.0f, 116.5f, 146.8f, 130.8f };
    for (size_t i = 0; i < num_samples; ++i) {
        float t = static_cast<float>(i) / 48000.0f;
        int step = static_cast<int>(t * 8.0f) % 16;
        float freq = notes[step];
        phase += freq / 48000.0f;
        if (phase >= 1.0f) phase -= 1.0f;
        float saw = 2.0f * phase - 1.0f;
        float step_t = std::fmod(t * 8.0f, 1.0f);
        float cutoff = 400.0f + 2800.0f * std::exp(-step_t * 14.0f);
        float alpha = std::clamp(cutoff / 48000.0f * 6.28f, 0.01f, 0.8f);
        filter_state += alpha * (saw - filter_state);
        buf[i] = std::tanh(filter_state * 2.5f) * 0.45f;
    }
    return buf;
}

// Synthetic vocal chops generator
static std::vector<float> generate_synthetic_vocal_chops(size_t num_samples) {
    std::vector<float> buf(num_samples, 0.0f);
    for (size_t i = 0; i < num_samples; ++i) {
        float t = static_cast<float>(i) / 48000.0f;
        float bar_t = std::fmod(t * 0.5f, 1.0f);
        float chop = 0.0f;
        if (bar_t < 0.20f) {
            chop = std::sin(t * 261.63f * 6.283185f) * std::exp(-std::fmod(t * 4.0f, 1.0f) * 10.0f) * 0.40f;
        } else if (bar_t >= 0.50f && bar_t < 0.70f) {
            chop = std::sin(t * 392.00f * 6.283185f) * std::exp(-std::fmod(t * 4.0f, 1.0f) * 12.0f) * 0.35f;
        }
        buf[i] = chop;
    }
    return buf;
}

// Static variables for GLFW Drag & Drop Audio Import
static std::string g_dropped_wav_path = "";
static bool g_has_dropped_wav = false;

int main(int argc, char** argv) {
    // 1. Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "[Error] Failed to initialize GLFW!" << std::endl;
        return 1;
    }

    // OpenGL 3.3 Core Profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(1440, 900, "AETHEL AUDIO DESK // SOVEREIGN ENGINE [RT 48kHz]", nullptr, nullptr);
    if (!window) {
        std::cerr << "[Error] Failed to create GLFW window!" << std::endl;
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable V-Sync (60/120 Hz refresh)

    // Install GLFW Drag & Drop Callback for Audio Import (.wav)
    glfwSetDropCallback(window, [](GLFWwindow*, int count, const char** paths) {
        if (count > 0 && paths && paths[0]) {
            g_dropped_wav_path = paths[0];
            g_has_dropped_wav = true;
        }
    });

    // 2. Initialize Dear ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // Apply Orderly Architect's Desk Theme
    ui::apply_architect_desk_theme();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    // 3. Initialize Audio Engine & Tracks
    constexpr uint32_t kBlockFrames = 1024;
    constexpr uint32_t kSampleRate = 48000;
    MixerGraph mixer(kBlockFrames);

    auto* trk0 = mixer.allocate_track("Kick / 808 Sub");
    auto* trk1 = mixer.allocate_track("Acid 303 Lead");
    auto* trk2 = mixer.allocate_track("Vocal Chops");
    auto* trk3 = mixer.allocate_track("Drums / Bus");

    // Allocate Submix and Auxiliary Buses
    auto* bus_drums  = mixer.allocate_submix_bus("Bus A: Drums");
    auto* bus_music  = mixer.allocate_submix_bus("Bus B: Music");
    auto* bus_reverb = mixer.allocate_submix_bus("Aux 1: Reverb");
    auto* bus_delay  = mixer.allocate_submix_bus("Aux 2: Delay");
    (void)bus_reverb;
    (void)bus_delay;

    // Route tracks to Submix buses by default:
    // Track 1 (Kick/808) -> Bus A (Drums)
    // Track 2 (Acid 303) -> Bus B (Music)
    // Track 3 (Vocal)    -> Bus B (Music)
    // Track 4 (Drums)    -> Master
    trk0->set_target_bus(bus_drums->id());
    trk1->set_target_bus(bus_music->id());
    trk2->set_target_bus(bus_music->id());
    trk3->set_target_bus(0); // Master Out

    // Pre-insert standard Airwindows DSPs on tracks and buses
    auto bax = std::make_shared<dsp::Baxandall>();
    bax->init(kSampleRate);
    bax->set_parameter(0, 2.5f); // +2.5dB Bass warmth
    trk0->slot(0).set_processor(bax);

    auto comp = std::make_shared<dsp::ButterComp2>();
    comp->init(kSampleRate);
    comp->set_parameter(0, 0.45f); // Smooth RMS compression
    trk0->slot(1).set_processor(comp);

    auto drive = std::make_shared<dsp::PurestDrive>();
    drive->init(kSampleRate);
    drive->set_parameter(0, 0.60f); // Harmonic saturation
    trk1->slot(0).set_processor(drive);

    auto derez = std::make_shared<dsp::DeRez>();
    derez->init(kSampleRate);
    derez->set_parameter(0, 0.90f); // 32kHz rate decimation
    derez->set_parameter(1, 0.80f); // 12-bit SP-1200 grit
    derez->set_parameter(2, 0.0f);  // mu-Law vintage companding
    derez->set_parameter(3, 1.0f);  // 100% wet
    trk2->slot(0).set_processor(derez);

    auto drum_bus_comp = std::make_shared<dsp::ButterComp2>();
    drum_bus_comp->init(kSampleRate);
    drum_bus_comp->set_parameter(0, 0.35f); // Bus glue compression
    bus_drums->slot(0).set_processor(drum_bus_comp);

    // Master bus processors: Baxandall (Master EQ) + ClipOnly2 (Master Limiter)
    auto master_bax = std::make_shared<dsp::Baxandall>();
    master_bax->init(kSampleRate);
    mixer.master_bus().slot(0).set_processor(master_bax);

    auto master_clipper = std::make_shared<dsp::ClipOnly2>();
    master_clipper->init(kSampleRate);
    mixer.master_bus().slot(1).set_processor(master_clipper);

    // Audio test loop waveform data (4 seconds @ 48kHz)
    std::vector<float> sample_waveform = generate_synthetic_drum_loop(48000 * 4);
    std::vector<float> slice_points = { 0.0f, 0.125f, 0.25f, 0.375f, 0.5f, 0.625f, 0.75f, 0.875f };

    // Load Drum Loop AudioClip into Track 0
    auto drum_clip = std::make_shared<sampling::AudioClip>("Drum Loop", kSampleRate, 2, static_cast<uint32_t>(sample_waveform.size()));
    for (size_t i = 0; i < sample_waveform.size(); ++i) {
        drum_clip->channel(0)[i] = sample_waveform[i];
        drum_clip->channel(1)[i] = sample_waveform[i];
    }
    trk0->set_clip(drum_clip, true);
    trk0->set_sync_to_transport(true);

    // Load Acid Loop AudioClip into Track 1
    std::vector<float> acid_waveform = generate_synthetic_acid_loop(48000 * 4);
    auto acid_clip = std::make_shared<sampling::AudioClip>("Acid 303", kSampleRate, 2, static_cast<uint32_t>(acid_waveform.size()));
    for (size_t i = 0; i < acid_waveform.size(); ++i) {
        acid_clip->channel(0)[i] = acid_waveform[i];
        acid_clip->channel(1)[i] = acid_waveform[i];
    }
    trk1->set_clip(acid_clip, true);
    trk1->set_sync_to_transport(true);

    // Load Vocal Chops AudioClip into Track 2
    std::vector<float> vocal_waveform = generate_synthetic_vocal_chops(48000 * 4);
    auto vocal_clip = std::make_shared<sampling::AudioClip>("Vocal Chops", kSampleRate, 2, static_cast<uint32_t>(vocal_waveform.size()));
    for (size_t i = 0; i < vocal_waveform.size(); ++i) {
        vocal_clip->channel(0)[i] = vocal_waveform[i];
        vocal_clip->channel(1)[i] = vocal_waveform[i];
    }
    trk2->set_clip(vocal_clip, true);
    trk2->set_sync_to_transport(true);

    // Initialize Native PipeWire Audio Server Integration
    PipeWireBackend pw(mixer);
    bool pw_online = pw.init("Aethel Audio Desk", kSampleRate);
    if (pw_online) {
        pw_online = pw.start();
        if (pw_online) {
            std::cout << "[PipeWire] Audio stream successfully running!" << std::endl;
        }
    }


    // Workspace UI State
    bool is_playing = false;
    float bpm = 126.0f;
    float playhead_seconds = 0.0f;
    const float loop_length_seconds = 8.0f; // 4 bars at 120bpm
    int selected_track = 0;
    int active_slice = 0;

    // Multi-Track Clip Pool & Original Backups
    std::shared_ptr<sampling::AudioClip> track_clips[4] = { drum_clip, acid_clip, vocal_clip, nullptr };
    std::shared_ptr<sampling::AudioClip> track_clips_orig[4] = { drum_clip, acid_clip, vocal_clip, nullptr };

    // Sample Editor, Pitch & Repair state
    int pitch_algo_mode = 0; // 0=Vinyl, 1=Vintage MPC, 2=WSOLA, 3=Sovereign ODE
    float sample_pitch_shift = 0.0f;
    float sample_time_stretch = 1.0f;
    bool sample_reverse = false;
    bool sample_choke = true;
    bool creative_click_bypass = false;
    float derez_rate = 0.85f;
    float derez_res = 0.70f;
    float derez_hard = 0.0f; // 0 = vintage mu-law soft, 1 = raw digital
    float derez_wet = 1.0f;
    char manual_wav_path[512] = "";
    char status_toast[256] = "READY // DRAG & DROP ANY .WAV AUDIO FILE ONTO THE WORKSTATION";

    auto sync_track_clip = [&](int t, std::shared_ptr<sampling::AudioClip> clip) {
        if (t < 0 || t >= 4 || !clip) return;
        track_clips[t] = clip;
        if (t == 0) trk0->set_clip(clip, true);
        else if (t == 1) trk1->set_clip(clip, true);
        else if (t == 2) trk2->set_clip(clip, true);
        else if (t == 3) trk3->set_clip(clip, true);

        if (t == selected_track) {
            const uint32_t f = clip->num_frames();
            const float* src = clip->channel(0);
            sample_waveform.resize(f);
            if (src) std::copy_n(src, f, sample_waveform.data());
            slice_points.clear();
            for (const auto& s : clip->slices()) {
                slice_points.push_back(static_cast<float>(s.start_frame) / static_cast<float>(f));
            }
            if (slice_points.empty()) {
                slice_points = { 0.0f, 0.125f, 0.25f, 0.375f, 0.5f, 0.625f, 0.75f, 0.875f };
            }
        }
    };

    // Track UI state caches
    float track_gains[4] = { 0.85f, 0.70f, 0.80f, 0.90f };
    float track_pans[4] = { 0.0f, -0.25f, 0.30f, 0.0f };
    bool track_mutes[4] = { false, false, false, false };
    bool track_solos[4] = { false, false, false, false };
    bool track_solo_safes[4] = { false, false, false, false };
    int track_consoles[4] = { 1, 2, 3, 0 }; // Warm, Lush, etc.

    // Clip Launcher & Arranger state
    int track_active_clip[4] = { 0, 0, 0, 0 }; // 0 = Arranger, 1 = Clip 1, 2 = Clip 2, 3 = Clip 3, -1 = Stopped
    int track_queued_clip[4] = { -1, -1, -1, -1 };
    bool track_back_to_arranger[4] = { true, true, true, true };
    bool loop_region_active = true;

    // Tactile Submix & Aux Send routing state
    int track_target_buses[4] = { 1, 2, 2, 0 }; // 0=Master, 1=Bus A (Drums), 2=Bus B (Music)
    float track_aux1_sends[4] = { 0.0f, 0.25f, 0.40f, 0.0f }; // Aux 1 Reverb
    float track_aux2_sends[4] = { 0.0f, 0.35f, 0.15f, 0.0f }; // Aux 2 Delay

    // Submix Bus Strips state
    float bus_gains[2] = { 0.95f, 0.90f };
    bool bus_mutes[2] = { false, false };
    bool bus_solos[2] = { false, false };
    int bus_consoles[2] = { 1, 2 }; // Bus A Warm Analog, Bus B Lush Console

    // Mastering Suite & K-System state
    int k_system_mode = 1; // 0=K-12 (Broadcast), 1=K-14 (Modern Hit), 2=K-20 (Audiophile)
    float master_bax_bass = 2.0f;
    float master_bax_treble = 1.5f;
    float master_glue_comp = 0.35f;
    float master_glue_makeup = 1.05f;
    bool master_clipper_active = true;
    float master_gain = 0.90f;
    bool master_limiter = true;

    // Envelope State
    float env_attack = 15.0f;
    float env_decay = 80.0f;
    float env_sustain = 0.65f;
    float env_release = 120.0f;
    float env_tau = 1.0f;

    // WASM Gas Watchdog Mock State
    float wasm_gas_used = 9.4f;
    float wasm_gas_limit = 25.0f;
    bool wasm_tripped = false;
    float wasm_param1 = 0.55f;
    float wasm_param2 = 0.30f;

    // Matrix routing crosspoints (4 sources x 3 destinations)
    bool matrix_routes[4][3] = {
        { true, false, false },
        { true, true,  false },
        { true, false, true  },
        { true, false, false }
    };
    float matrix_gains[4][3] = {
        { 1.0f, 0.0f, 0.0f },
        { 1.0f, 0.4f, 0.0f },
        { 1.0f, 0.0f, 0.6f },
        { 1.0f, 0.0f, 0.0f }
    };

    protocol::MixerTelemetryFrame telemetry{};
    auto last_time = std::chrono::high_resolution_clock::now();

    // 4. Main Window Render Loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        auto now = std::chrono::high_resolution_clock::now();
        float dt = std::chrono::duration<float>(now - last_time).count();
        last_time = now;

        if (is_playing) {
            if (pw_online) {
                playhead_seconds = static_cast<float>(mixer.clock().sample_position()) / static_cast<float>(kSampleRate);
                if (playhead_seconds >= loop_length_seconds) {
                    playhead_seconds = std::fmod(playhead_seconds, loop_length_seconds);
                }
            } else {
                playhead_seconds += dt * (bpm / 120.0f);
                if (playhead_seconds >= loop_length_seconds) {
                    playhead_seconds = std::fmod(playhead_seconds, loop_length_seconds);
                }
            }
        }

        // Lock-free telemetry query
        mixer.capture_telemetry_snapshot(telemetry);

        // Fallback: If PipeWire is offline, synthesize visual meters for active animation when playing
        if (!pw_online && is_playing) {
            float pulse = std::abs(std::sin(playhead_seconds * 3.14159f * 2.0f));
            telemetry.master_meter.peak_l = std::clamp(pulse * 0.92f, 0.0f, 1.05f);
            telemetry.master_meter.peak_r = std::clamp(pulse * 0.88f, 0.0f, 1.02f);
            telemetry.master_meter.rms_l  = telemetry.master_meter.peak_l * 0.72f;
            telemetry.master_meter.rms_r  = telemetry.master_meter.peak_r * 0.70f;

            for (size_t t = 0; t < 4; ++t) {
                float trk_pulse = std::abs(std::sin(playhead_seconds * 3.14159f * (t + 1)));
                telemetry.track_meters[t].peak_l = trk_pulse * track_gains[t];
                telemetry.track_meters[t].peak_r = trk_pulse * track_gains[t] * 0.95f;
                telemetry.track_meters[t].rms_l  = telemetry.track_meters[t].peak_l * 0.68f;
                telemetry.track_meters[t].rms_r  = telemetry.track_meters[t].peak_r * 0.65f;
            }

            // Synthesize Submix Bus meters
            telemetry.bus_meters[0].peak_l = telemetry.track_meters[0].peak_l * 0.95f;
            telemetry.bus_meters[0].peak_r = telemetry.track_meters[0].peak_r * 0.95f;
            telemetry.bus_meters[0].rms_l  = telemetry.bus_meters[0].peak_l * 0.70f;
            telemetry.bus_meters[0].rms_r  = telemetry.bus_meters[0].peak_r * 0.70f;

            telemetry.bus_meters[1].peak_l = std::max(telemetry.track_meters[1].peak_l, telemetry.track_meters[2].peak_l);
            telemetry.bus_meters[1].peak_r = std::max(telemetry.track_meters[1].peak_r, telemetry.track_meters[2].peak_r);
            telemetry.bus_meters[1].rms_l  = telemetry.bus_meters[1].peak_l * 0.68f;
            telemetry.bus_meters[1].rms_r  = telemetry.bus_meters[1].peak_r * 0.68f;

            // Synthesize Kinetic ODE telemetry & Poincaré Phase-Space Attractor Orbit
            float t = playhead_seconds;
            telemetry.kinetic_meter.authority = 0.65f + 0.25f * std::abs(std::sin(t * 3.14159f * 1.0f));
            telemetry.kinetic_meter.power = 0.58f + 0.32f * pulse;
            telemetry.kinetic_meter.detail = 0.40f + 0.40f * std::abs(std::sin(t * 3.14159f * 4.0f));
            telemetry.kinetic_meter.crest_factor_db = 11.6f + 2.4f * std::cos(t * 2.0f);
            telemetry.kinetic_meter.diagnostic_id = 1; // Balanced Hit Sonority

            for (size_t i = 0; i < protocol::KineticTelemetryData::kPhasePoints; ++i) {
                float angle = static_cast<float>(i) / protocol::KineticTelemetryData::kPhasePoints * 6.283185f;
                float kick_rad = 0.32f * pulse;
                float r = (0.44f + kick_rad) * (1.0f + 0.12f * std::sin(angle * 3.0f + t * 4.0f));
                telemetry.kinetic_meter.phase_x[i] = std::clamp(r * std::cos(angle), -0.98f, 0.98f);
                telemetry.kinetic_meter.phase_y[i] = std::clamp(r * 1.22f * std::sin(angle) + 0.08f * std::sin(angle * 7.0f), -0.98f, 0.98f);
            }
        }

        // Process any Drag & Drop Audio File Import from GLFW
        if (g_has_dropped_wav) {
            g_has_dropped_wav = false;
            auto imported = std::make_shared<sampling::AudioClip>();
            if (imported->load_from_wav(g_dropped_wav_path)) {
                track_clips_orig[selected_track] = imported;
                sync_track_clip(selected_track, imported);
                std::snprintf(status_toast, sizeof(status_toast),
                              "IMPORTED WAV: %s (%u Hz, %u ch, %.2fs)",
                              imported->name().c_str(), imported->sample_rate(),
                              imported->num_channels(),
                              static_cast<float>(imported->num_frames()) / imported->sample_rate());
            } else {
                std::snprintf(status_toast, sizeof(status_toast),
                              "ERROR: FAILED TO LOAD WAV: %s", g_dropped_wav_path.c_str());
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Workstation Fullscreen Dock Window
        int win_w, win_h;
        glfwGetFramebufferSize(window, &win_w, &win_h);
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(static_cast<float>(win_w), static_cast<float>(win_h)));
        ImGui::Begin("AethelAudioDeskRoot", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus);

        // ====================================================================
        // ZONE 1: TOP GLOBAL TRANSPORT & HUD BAR
        // ====================================================================
        ImGui::BeginChild("TopTransportBar", ImVec2(0, 52), true, ImGuiWindowFlags_NoScrollbar);
        {
            // Transport Controls
            if (ImGui::Button(is_playing ? "[ || PAUSE ]" : "[ > PLAY ]", ImVec2(90, 32))) {
                is_playing = !is_playing;
                mixer.clock().set_playing(is_playing);
            }
            ImGui::SameLine();
            if (ImGui::Button("[ [] STOP ]", ImVec2(80, 32))) {
                is_playing = false;
                mixer.clock().set_playing(false);
                mixer.clock().set_sample_position(0);
                trk0->set_clip_playhead(0.0);
                playhead_seconds = 0.0f;
            }

            ImGui::SameLine();
            ImGui::SetNextItemWidth(100);
            if (ImGui::SliderFloat("BPM", &bpm, 60.0f, 200.0f, "%.1f")) {
                mixer.clock().set_bpm(bpm);
            }

            ImGui::SameLine(0, 20);
            if (pw_online) {
                ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "[PIPEWIRE RT // SINK CONNECTED]");
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.40f, 0.45f, 0.52f, 1.0f), "| 48.0 kHz | 256s (5.3ms) | HW DAC ACTIVE");
            } else {
                ImGui::TextColored(ImVec4(0.85f, 0.25f, 0.20f, 1.0f), "[PIPEWIRE OFFLINE // HEADLESS]");
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.40f, 0.45f, 0.52f, 1.0f), "| Simulation Fallback");
            }

            ImGui::SameLine(0, 30);
            ImGui::TextColored(ImVec4(0.85f, 0.48f, 0.05f, 1.0f), "MASTER:");
            ImGui::SameLine();


            // Compact Master VU Meter
            ImVec2 meter_pos = ImGui::GetCursorScreenPos();
            meter_pos.y += 2.0f;
            ui::DrawDbMeter(ImGui::GetWindowDrawList(), meter_pos, ImVec2(24, 28),
                            telemetry.master_meter.peak_l, telemetry.master_meter.peak_r,
                            telemetry.master_meter.rms_l, telemetry.master_meter.rms_r,
                            telemetry.master_meter.peak_l >= 1.0f);
            ImGui::Dummy(ImVec2(28, 30));

            ImGui::SameLine(0, 15);
            ImGui::SetNextItemWidth(120);
            if (ImGui::SliderFloat("Vol##Master", &master_gain, 0.0f, 1.25f, "%.2f")) {
                protocol::MixerCommand cmd{};
                cmd.type = protocol::MixerCommandType::SetMasterGain;
                cmd.value1 = master_gain;
                mixer.post_command(cmd);
            }

            ImGui::SameLine(0, 15);
            if (ImGui::Checkbox("LIMITER", &master_limiter)) {
                protocol::MixerCommand cmd{};
                cmd.type = protocol::MixerCommandType::SetMasterLimiter;
                cmd.flags = master_limiter ? 1 : 0;
                mixer.post_command(cmd);
            }
        }
        ImGui::EndChild();

        // Calculate available vertical space for Zone 2 (Center) and Zone 3 (Bottom Dock)
        const float available_h = static_cast<float>(win_h) - 68.0f;
        const float zone2_h = available_h * 0.46f;
        const float zone3_h = available_h * 0.54f;

        // ====================================================================
        // ZONE 2: CENTER WORKSPACE (TIMELINE ARRANGER vs. ROUTING MATRIX)
        // ====================================================================
        ImGui::BeginChild("CenterWorkspace", ImVec2(0, zone2_h), true);
        {
            if (ImGui::BeginTabBar("MainWorkspaceTabs", ImGuiTabBarFlags_None)) {
                // ------------------------------------------------------------
                // TAB A: TIMELINE & CLIP LAUNCHER (HYBRID ARRANGER)
                // ------------------------------------------------------------
                if (ImGui::BeginTabItem("  TIMELINE & CLIP LAUNCHER (HYBRID ARRANGER)  ")) {
                    const char* track_names[4] = { "Track 1: Kick & 808", "Track 2: Acid 303 Lead", "Track 3: Vocal Slices", "Track 4: Drum Bus" };
                    const float launcher_w = 330.0f;
                    ImVec2 avail_sz = ImGui::GetContentRegionAvail();
                    avail_sz.y = std::max(avail_sz.y - 4.0f, 130.0f);

                    // ========================================================
                    // LEFT COLUMN: BITWIG-STYLE CLIP / PATTERN LAUNCHER
                    // ========================================================
                    ImGui::BeginChild("ClipLauncherPane", ImVec2(launcher_w, avail_sz.y), true, ImGuiWindowFlags_NoScrollbar);
                    {
                        // Scene Launcher Row
                        ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "SCENES:");
                        ImGui::SameLine();
                        if (ImGui::SmallButton("▶ S1")) {
                            for (int t = 0; t < 4; ++t) {
                                track_back_to_arranger[t] = false;
                                track_active_clip[t] = 1;
                                track_queued_clip[t] = -1;
                            }
                        }
                        ImGui::SameLine();
                        if (ImGui::SmallButton("▶ S2")) {
                            for (int t = 0; t < 4; ++t) {
                                track_back_to_arranger[t] = false;
                                track_active_clip[t] = 2;
                                track_queued_clip[t] = -1;
                            }
                        }
                        ImGui::SameLine();
                        if (ImGui::SmallButton("▶ S3")) {
                            for (int t = 0; t < 4; ++t) {
                                track_back_to_arranger[t] = false;
                                track_active_clip[t] = 3;
                                track_queued_clip[t] = -1;
                            }
                        }
                        ImGui::SameLine();
                        if (ImGui::SmallButton("■ ALL")) {
                            for (int t = 0; t < 4; ++t) {
                                track_active_clip[t] = -1;
                            }
                        }
                        ImGui::Separator();

                        // Track Clip Slots
                        for (int t = 0; t < 4; ++t) {
                            ImGui::PushID(200 + t);
                            ImGui::BeginGroup();
                            {
                                // Track miniature status & selection
                                bool is_sel = (selected_track == t);
                                if (ImGui::Selectable(track_names[t], is_sel, 0, ImVec2(135, 18))) {
                                    selected_track = t;
                                }
                                ImGui::SameLine();

                                // Back to Arranger button
                                if (track_back_to_arranger[t]) {
                                    ImGui::TextColored(ImVec4(0.40f, 0.45f, 0.52f, 1.0f), "[ARR]");
                                } else {
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.48f, 0.05f, 0.85f));
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                    if (ImGui::SmallButton("⮌ ARR")) {
                                        track_back_to_arranger[t] = true;
                                        track_active_clip[t] = 0;
                                    }
                                    ImGui::PopStyleColor(2);
                                }

                                // 3 Clip Slots + Stop button
                                for (int s = 1; s <= 3; ++s) {
                                    ImGui::SameLine();
                                    bool is_playing_slot = (!track_back_to_arranger[t] && track_active_clip[t] == s);
                                    bool is_queued = (track_queued_clip[t] == s);

                                    char slot_lbl[24];
                                    if (is_playing_slot) {
                                        std::snprintf(slot_lbl, sizeof(slot_lbl), "▶ %d##s", s);
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.38f, 0.85f, 1.0f));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                    } else if (is_queued) {
                                        std::snprintf(slot_lbl, sizeof(slot_lbl), "⧗ %d##s", s);
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.48f, 0.05f, 1.0f));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                    } else {
                                        std::snprintf(slot_lbl, sizeof(slot_lbl), "%d##s", s);
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.93f, 0.95f, 0.97f, 1.0f));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.25f, 0.30f, 0.40f, 1.0f));
                                    }

                                    if (ImGui::Button(slot_lbl, ImVec2(28, 20))) {
                                        track_back_to_arranger[t] = false;
                                        track_active_clip[t] = s;
                                        track_queued_clip[t] = -1;
                                        if (t == 0) trk0->set_clip_playhead(static_cast<double>(s - 1) * 24000.0);
                                        if (t == 1) trk1->set_clip_playhead(static_cast<double>(s - 1) * 24000.0);
                                        if (t == 2) trk2->set_clip_playhead(static_cast<double>(s - 1) * 24000.0);
                                    }
                                    ImGui::PopStyleColor(2);
                                }

                                ImGui::SameLine();
                                if (ImGui::SmallButton("■##stp")) {
                                    track_active_clip[t] = -1;
                                    track_back_to_arranger[t] = false;
                                }
                            }
                            ImGui::EndGroup();
                            if (t < 3) ImGui::Separator();
                            ImGui::PopID();
                        }
                    }
                    ImGui::EndChild();

                    ImGui::SameLine();

                    // ========================================================
                    // RIGHT COLUMN: MULTITRACK ARRANGER TIMELINE
                    // ========================================================
                    ImGui::BeginChild("ArrangerTimelinePane", ImVec2(0, avail_sz.y), true, ImGuiWindowFlags_NoScrollbar);
                    {
                        ImDrawList* draw_list = ImGui::GetWindowDrawList();
                        ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
                        ImVec2 canvas_size = ImGui::GetContentRegionAvail();
                        canvas_size.y = std::max(canvas_size.y - 2.0f, 120.0f);

                        // Draw Background
                        draw_list->AddRectFilled(canvas_pos,
                                                 ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
                                                 ImColor(255, 255, 255, 255), 2.0f);
                        draw_list->AddRect(canvas_pos,
                                           ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
                                           ImColor(190, 196, 206, 255), 2.0f);

                        // Bar Grid lines (16 bars)
                        constexpr int kTotalBars = 16;
                        const float bar_w = canvas_size.x / kTotalBars;
                        for (int b = 0; b <= kTotalBars; ++b) {
                            float bx = canvas_pos.x + b * bar_w;
                            draw_list->AddLine(ImVec2(bx, canvas_pos.y),
                                               ImVec2(bx, canvas_pos.y + canvas_size.y),
                                               ImColor(230, 235, 242, 255), 1.0f);
                            if (b < kTotalBars) {
                                char b_txt[16];
                                std::snprintf(b_txt, sizeof(b_txt), "%d.1", b + 1);
                                draw_list->AddText(ImVec2(bx + 4.0f, canvas_pos.y + 2.0f),
                                                   ImColor(100, 110, 125, 255), b_txt);
                            }
                        }

                        // Loop Region Highlighting (Bars 1 to 5)
                        if (loop_region_active) {
                            float loop_x1 = canvas_pos.x;
                            float loop_x2 = canvas_pos.x + 4.0f * bar_w;
                            draw_list->AddRectFilled(ImVec2(loop_x1, canvas_pos.y),
                                                     ImVec2(loop_x2, canvas_pos.y + 16.0f),
                                                     ImColor(31, 97, 217, 35));
                            draw_list->AddLine(ImVec2(loop_x1, canvas_pos.y + 16.0f),
                                               ImVec2(loop_x2, canvas_pos.y + 16.0f),
                                               ImColor(31, 97, 217, 200), 2.0f);
                            draw_list->AddText(ImVec2(loop_x1 + 4.0f, canvas_pos.y + 2.0f),
                                               ImColor(31, 97, 217, 255), "[ LOOP: 4 BARS ]");
                        }

                        // 4 Track Lanes
                        const float lane_h = (canvas_size.y - 20.0f) / 4.0f;
                        for (int t = 0; t < 4; ++t) {
                            float ly = canvas_pos.y + 20.0f + t * lane_h;
                            draw_list->AddLine(ImVec2(canvas_pos.x, ly),
                                               ImVec2(canvas_pos.x + canvas_size.x, ly),
                                               ImColor(230, 235, 242, 255), 1.0f);

                            if (selected_track == t) {
                                draw_list->AddRectFilled(ImVec2(canvas_pos.x, ly),
                                                         ImVec2(canvas_pos.x + canvas_size.x, ly + lane_h),
                                                         ImColor(31, 97, 217, 18));
                            }

                            // If Clip Launcher is overriding this track:
                            if (!track_back_to_arranger[t] && track_active_clip[t] > 0) {
                                float blk_x1 = canvas_pos.x + 4.0f;
                                float blk_x2 = canvas_pos.x + canvas_size.x - 4.0f;
                                draw_list->AddRectFilled(ImVec2(blk_x1, ly + 4.0f),
                                                         ImVec2(blk_x2, ly + lane_h - 4.0f),
                                                         ImColor(254, 243, 199, 180), 2.0f);
                                draw_list->AddRect(ImVec2(blk_x1, ly + 4.0f),
                                                   ImVec2(blk_x2, ly + lane_h - 4.0f),
                                                   ImColor(217, 123, 13, 220), 1.5f);
                                char act_txt[64];
                                std::snprintf(act_txt, sizeof(act_txt), "[ CLIP LAUNCHER ACTIVE // PATTERN %d PLAYING ]", track_active_clip[t]);
                                draw_list->AddText(ImVec2(blk_x1 + 12.0f, ly + 6.0f),
                                                   ImColor(180, 83, 9, 255), act_txt);
                            } else {
                                // Arranger blocks
                                float clip_x1 = canvas_pos.x + (t * 2.0f) * bar_w;
                                float clip_x2 = clip_x1 + (4.0f) * bar_w;
                                draw_list->AddRectFilled(ImVec2(clip_x1 + 2.0f, ly + 4.0f),
                                                         ImVec2(clip_x2 - 2.0f, ly + lane_h - 4.0f),
                                                         (selected_track == t) ? ImColor(215, 230, 255, 240) : ImColor(235, 242, 255, 220),
                                                         2.0f);
                                draw_list->AddRect(ImVec2(clip_x1 + 2.0f, ly + 4.0f),
                                                   ImVec2(clip_x2 - 2.0f, ly + lane_h - 4.0f),
                                                   ImColor(31, 97, 217, 220), 2.0f);
                                draw_list->AddText(ImVec2(clip_x1 + 8.0f, ly + 6.0f),
                                                   ImColor(15, 30, 70, 255), track_names[t]);
                            }
                        }

                        // Playhead Needle
                        float play_ratio = playhead_seconds / loop_length_seconds;
                        float playhead_x = canvas_pos.x + play_ratio * canvas_size.x;
                        draw_list->AddLine(ImVec2(playhead_x, canvas_pos.y),
                                           ImVec2(playhead_x, canvas_pos.y + canvas_size.y),
                                           ImColor(20, 25, 35, 255), 2.0f);
                        draw_list->AddTriangleFilled(ImVec2(playhead_x - 6.0f, canvas_pos.y),
                                                     ImVec2(playhead_x + 6.0f, canvas_pos.y),
                                                     ImVec2(playhead_x, canvas_pos.y + 10.0f),
                                                     ImColor(20, 25, 35, 255));

                        // Interactive scrubbing on ruler
                        ImGui::InvisibleButton("ArrangerCanvasSeekBtn", canvas_size);
                        if (ImGui::IsItemActive()) {
                            ImVec2 m = ImGui::GetIO().MousePos;
                            float ratio = std::clamp((m.x - canvas_pos.x) / canvas_size.x, 0.0f, 1.0f);
                            playhead_seconds = ratio * loop_length_seconds;
                            uint64_t target_sample = static_cast<uint64_t>(playhead_seconds * kSampleRate);
                            mixer.clock().set_sample_position(target_sample);
                            trk0->set_clip_playhead(static_cast<double>(target_sample));
                            trk1->set_clip_playhead(static_cast<double>(target_sample));
                            trk2->set_clip_playhead(static_cast<double>(target_sample));
                        }
                    }
                    ImGui::EndChild();

                    ImGui::EndTabItem();
                }

                // ------------------------------------------------------------
                // TAB B: MASTERING SUITE // KINETIC ODE & HIT RECORD METER
                // ------------------------------------------------------------
                if (ImGui::BeginTabItem("  MASTERING SUITE // KINETIC ODE & HIT RECORD METER  ")) {
                    ImVec2 avail_sz = ImGui::GetContentRegionAvail();
                    avail_sz.y = std::max(avail_sz.y - 4.0f, 130.0f);
                    const float meter_w = std::min(avail_sz.x * 0.44f, 500.0f);

                    // 1. LEFT PANE: KINETIC HIT METER & POINCARE ATTRACTOR
                    ImVec2 meter_pos = ImGui::GetCursorScreenPos();
                    ui::DrawKineticHitMeter(ImGui::GetWindowDrawList(), meter_pos, ImVec2(meter_w, avail_sz.y), telemetry.kinetic_meter);
                    ImGui::Dummy(ImVec2(meter_w + 10.0f, avail_sz.y));

                    ImGui::SameLine();

                    // 2. RIGHT PANE: BOB KATZ K-SYSTEM & MASTERING DSP RACK
                    ImGui::BeginChild("MasteringControlsPane", ImVec2(0, avail_sz.y), true);
                    {
                        ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "BOB KATZ K-SYSTEM & TELEMETRY MONITOR");
                        ImGui::Separator();

                        // K-System Mode Selector
                        ImGui::Text("K-System Standard: ");
                        ImGui::SameLine();
                        if (ImGui::RadioButton("K-12 (Broadcast)", k_system_mode == 0)) k_system_mode = 0;
                        ImGui::SameLine();
                        if (ImGui::RadioButton("K-14 (Modern Pop/Rock)", k_system_mode == 1)) k_system_mode = 1;
                        ImGui::SameLine();
                        if (ImGui::RadioButton("K-20 (Audiophile)", k_system_mode == 2)) k_system_mode = 2;

                        const float k_target_db = (k_system_mode == 0) ? -12.0f : ((k_system_mode == 1) ? -14.0f : -20.0f);
                        float master_peak_db = ui::linear_to_db(std::max(telemetry.master_meter.peak_l, telemetry.master_meter.peak_r));
                        float master_rms_db  = ui::linear_to_db(std::max(telemetry.master_meter.rms_l, telemetry.master_meter.rms_r));
                        float k_margin = master_rms_db - k_target_db;

                        // Telemetry Metric Badges
                        ImGui::Spacing();
                        ImGui::Columns(4, "MasterMetersGrid", false);
                        ImGui::Text("True Peak (dBFS)");
                        ImGui::TextColored((master_peak_db >= -0.1f) ? ImVec4(0.85f, 0.18f, 0.22f, 1.0f) : ImVec4(0.15f, 0.20f, 0.30f, 1.0f),
                                           "%.2f dB", master_peak_db);
                        ImGui::NextColumn();

                        ImGui::Text("RMS AES-17");
                        ImGui::TextColored(ImVec4(0.15f, 0.20f, 0.30f, 1.0f), "%.2f dB", master_rms_db);
                        ImGui::NextColumn();

                        ImGui::Text("Crest Factor");
                        ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "%.1f dB", telemetry.kinetic_meter.crest_factor_db);
                        ImGui::NextColumn();

                        ImGui::Text("K-Meter (0dBK ref)");
                        ImGui::TextColored((k_margin > 2.0f) ? ImVec4(0.85f, 0.48f, 0.05f, 1.0f) : ImVec4(0.13f, 0.65f, 0.35f, 1.0f),
                                           "%+.1f dB", k_margin);
                        ImGui::Columns(1);

                        ImGui::Separator();
                        ImGui::Spacing();
                        ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "MASTER BUS ANALOG DSP INSERT CHAIN");

                        // Master Baxandall EQ Controls
                        ImGui::Text("Master Baxandall EQ (C^inf Smooth Air):");
                        ImGui::SetNextItemWidth(140);
                        if (ImGui::SliderFloat("Bass (Warmth)##MBax", &master_bax_bass, -6.0f, 6.0f, "%.1f dB")) {
                            master_bax->set_parameter(0, master_bax_bass);
                        }
                        ImGui::SameLine(0, 20);
                        ImGui::SetNextItemWidth(140);
                        if (ImGui::SliderFloat("Treble (Air)##MBax", &master_bax_treble, -6.0f, 6.0f, "%.1f dB")) {
                            master_bax->set_parameter(1, master_bax_treble);
                        }

                        // Master Glue Compressor & ClipOnly2 Controls
                        ImGui::Spacing();
                        ImGui::Text("Master ButterComp2 & ClipOnly2:");
                        ImGui::SetNextItemWidth(140);
                        if (ImGui::SliderFloat("Glue Amount##MComp", &master_glue_comp, 0.0f, 1.0f, "%.2f")) {
                            // Update bus comp
                        }
                        ImGui::SameLine(0, 20);
                        ImGui::SetNextItemWidth(140);
                        if (ImGui::SliderFloat("Makeup Gain##MComp", &master_glue_makeup, 0.5f, 2.0f, "%.2f")) {
                            // Update makeup
                        }
                        ImGui::SameLine(0, 20);
                        if (ImGui::Checkbox("ClipOnly2 True-Peak", &master_clipper_active)) {
                            mixer.master_bus().slot(1).set_bypass(!master_clipper_active);
                        }
                    }
                    ImGui::EndChild();

                    ImGui::EndTabItem();
                }

                // ------------------------------------------------------------
                // TAB B: UNIVERSAL ROUTING MATRIX
                // ------------------------------------------------------------
                if (ImGui::BeginTabItem("  UNIVERSAL ROUTING MATRIX  ")) {
                    ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f),
                                       "Universal Lock-Free Matrix Grid (Zero Bitwig-Converters / Native Vectorized FMA)");
                    ImGui::Separator();


                    const char* row_names[4] = { "Track 1 (Kick/808)", "Track 2 (Acid 303)", "Track 3 (Vocal)", "Track 4 (Drums)" };
                    const char* col_names[3] = { "Master Bus", "Reverb Return", "Sidechain Ducker" };

                    if (ImGui::BeginTable("MatrixTable", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                        ImGui::TableSetupColumn("Source \\ Destination", ImGuiTableColumnFlags_WidthFixed, 180.0f);
                        ImGui::TableSetupColumn(col_names[0], ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableSetupColumn(col_names[1], ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableSetupColumn(col_names[2], ImGuiTableColumnFlags_WidthStretch);
                        ImGui::TableHeadersRow();

                        for (int r = 0; r < 4; ++r) {
                            ImGui::TableNextRow();
                            ImGui::TableSetColumnIndex(0);
                            ImGui::Text("%s", row_names[r]);

                            for (int c = 0; c < 3; ++c) {
                                ImGui::TableSetColumnIndex(c + 1);
                                ImGui::PushID(r * 10 + c);
                                if (ImGui::Checkbox("Route", &matrix_routes[r][c])) {
                                    // Lock-free matrix patch command
                                }
                                if (matrix_routes[r][c]) {
                                    ImGui::SameLine();
                                    ImGui::SetNextItemWidth(70);
                                    ImGui::SliderFloat("dB", &matrix_gains[r][c], 0.0f, 1.0f, "%.2f");
                                }
                                ImGui::PopID();
                            }
                        }
                        ImGui::EndTable();
                    }
                    ImGui::EndTabItem();
                }

                ImGui::EndTabBar();
            }
        }
        ImGui::EndChild();

        // ====================================================================
        // ZONE 3: BOTTOM CONTEXT-SENSITIVE DOCKED TABS
        // ====================================================================
        ImGui::BeginChild("BottomDockWorkspace", ImVec2(0, zone3_h), true);
        {
            if (ImGui::BeginTabBar("BottomDockTabs", ImGuiTabBarFlags_None)) {
                // ------------------------------------------------------------
                // TAB 1: MIXER / CHANNEL STRIP
                // ------------------------------------------------------------
                if (ImGui::BeginTabItem("  MIXER / CHANNEL STRIP  ")) {
                    const char* track_names[4] = { "1: Kick / 808", "2: Acid 303", "3: Vocal", "4: Drums" };
                    const char* console_types[4] = { "Digital", "Warm Analog", "Lush Console", "Direct Clean" };

                    for (int t = 0; t < 4; ++t) {
                        ImGui::PushID(t);
                        ImGui::BeginChild(track_names[t], ImVec2(240, 0), true);
                        {
                            // Track header
                            bool is_sel = (selected_track == t);
                            if (ImGui::Selectable(track_names[t], is_sel)) {
                                selected_track = t;
                            }
                            ImGui::Separator();

                            // Tactile Bus Assignment Buttons: [ MST ] [ BUS A ] [ BUS B ]
                            ImGui::TextDisabled("Route:");
                            bool to_mst   = (track_target_buses[t] == 0);
                            bool to_bus_a = (track_target_buses[t] == 1);
                            bool to_bus_b = (track_target_buses[t] == 2);

                            auto set_track_bus = [&](int bus_id) {
                                track_target_buses[t] = bus_id;
                                protocol::MixerCommand cmd{};
                                cmd.type = protocol::MixerCommandType::SetTrackTargetBus;
                                cmd.target_id = t + 1; // 1-based track ID
                                cmd.secondary_id = (bus_id == 1) ? bus_drums->id() : ((bus_id == 2) ? bus_music->id() : 0);
                                mixer.post_command(cmd);
                            };

                            if (to_mst) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.38f, 0.85f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            }
                            if (ImGui::SmallButton("MST")) set_track_bus(0);
                            if (to_mst) ImGui::PopStyleColor(2);

                            ImGui::SameLine();
                            if (to_bus_a) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.38f, 0.85f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            }
                            if (ImGui::SmallButton("BUS A")) set_track_bus(1);
                            if (to_bus_a) ImGui::PopStyleColor(2);

                            ImGui::SameLine();
                            if (to_bus_b) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.38f, 0.85f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            }
                            if (ImGui::SmallButton("BUS B")) set_track_bus(2);
                            if (to_bus_b) ImGui::PopStyleColor(2);

                            // Rotary Aux Sends: Aux 1 (Reverb), Aux 2 (Delay)
                            ImGui::TextDisabled("Aux Sends:");
                            ImGui::SetNextItemWidth(60);
                            if (ui::DrawRotaryKnob("Aux 1", &track_aux1_sends[t], 0.0f, 1.0f, "", 13.0f)) {
                                protocol::MixerCommand cmd{};
                                cmd.type = protocol::MixerCommandType::SetTrackSend;
                                cmd.target_id = t + 1;
                                cmd.secondary_id = bus_reverb->id();
                                cmd.value1 = track_aux1_sends[t];
                                mixer.post_command(cmd);
                            }
                            ImGui::SameLine(0, 15);
                            ImGui::SetNextItemWidth(60);
                            if (ui::DrawRotaryKnob("Aux 2", &track_aux2_sends[t], 0.0f, 1.0f, "", 13.0f)) {
                                protocol::MixerCommand cmd{};
                                cmd.type = protocol::MixerCommandType::SetTrackSend;
                                cmd.target_id = t + 1;
                                cmd.secondary_id = bus_delay->id();
                                cmd.value1 = track_aux2_sends[t];
                                mixer.post_command(cmd);
                            }

                            ImGui::Separator();

                            // Console saturation selector
                            ImGui::SetNextItemWidth(120);
                            if (ImGui::Combo("Console", &track_consoles[t], console_types, 4)) {
                                protocol::MixerCommand cmd{};
                                cmd.type = protocol::MixerCommandType::SetTrackConsoleType;
                                cmd.target_id = t + 1;
                                cmd.flags = track_consoles[t];
                                mixer.post_command(cmd);
                            }

                            // 4 Insert Slots
                            ImGui::TextColored(ImVec4(0.35f, 0.40f, 0.48f, 1.0f), "Insert Slots (4x):");
                            auto* trk_ptr = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                            for (int s = 0; s < 4; ++s) {
                                ImGui::PushID(s);
                                ImGui::TextDisabled("S%d:", s + 1);
                                ImGui::SameLine();
                                const char* slot_name = "[Empty]";
                                bool is_by = false;
                                if (trk_ptr && trk_ptr->slot(s).processor()) {
                                    slot_name = trk_ptr->slot(s).processor()->name();
                                    is_by = trk_ptr->slot(s).is_bypassed();
                                }
                                char btn_label[48];
                                std::snprintf(btn_label, sizeof(btn_label), "%.14s", slot_name);
                                ImGui::Button(btn_label, ImVec2(100, 18));
                                if (ImGui::IsItemHovered() && trk_ptr && trk_ptr->slot(s).processor()) {
                                    ImGui::SetTooltip("%s", trk_ptr->slot(s).processor()->name());
                                }
                                ImGui::SameLine();
                                if (ImGui::Checkbox("By", &is_by)) {
                                    if (trk_ptr) {
                                        trk_ptr->slot(s).set_bypass(is_by);
                                    }
                                }
                                ImGui::PopID();
                            }

                            ImGui::Separator();

                            // Pan Rotary Dial
                            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 40.0f);
                            if (ui::DrawRotaryKnob("Pan", &track_pans[t], -1.0f, 1.0f, "", 18.0f)) {
                                protocol::MixerCommand cmd{};
                                cmd.type = protocol::MixerCommandType::SetTrackPan;
                                cmd.target_id = t + 1;
                                cmd.value1 = track_pans[t];
                                mixer.post_command(cmd);
                            }

                            // Mute, Solo, Solo-Safe Buttons
                            if (ImGui::Checkbox("M", &track_mutes[t])) {
                                protocol::MixerCommand cmd{};
                                cmd.type = protocol::MixerCommandType::SetTrackMute;
                                cmd.target_id = t + 1;
                                cmd.flags = track_mutes[t] ? 1 : 0;
                                mixer.post_command(cmd);
                            }
                            ImGui::SameLine();
                            if (ImGui::Checkbox("S", &track_solos[t])) {
                                protocol::MixerCommand cmd{};
                                cmd.type = protocol::MixerCommandType::SetTrackSolo;
                                cmd.target_id = t + 1;
                                cmd.flags = track_solos[t] ? 1 : 0;
                                mixer.post_command(cmd);
                            }
                            ImGui::SameLine();
                            if (ImGui::Checkbox("SS", &track_solo_safes[t])) {
                                protocol::MixerCommand cmd{};
                                cmd.type = protocol::MixerCommandType::SetTrackSoloSafe;
                                cmd.target_id = t + 1;
                                cmd.flags = track_solo_safes[t] ? 1 : 0;
                                mixer.post_command(cmd);
                            }

                            // Vertical Fader & Meter Bridge
                            ImGui::VSliderFloat("##fader", ImVec2(34, 110), &track_gains[t], 0.0f, 1.25f, "");

                            if (ImGui::IsItemEdited()) {
                                protocol::MixerCommand cmd{};
                                cmd.type = protocol::MixerCommandType::SetTrackGain;
                                cmd.target_id = t + 1;
                                cmd.value1 = track_gains[t];
                                mixer.post_command(cmd);
                            }

                            ImGui::SameLine();
                            ImVec2 meter_pos = ImGui::GetCursorScreenPos();
                            ui::DrawDbMeter(ImGui::GetWindowDrawList(), meter_pos, ImVec2(24, 110),
                                            telemetry.track_meters[t].peak_l, telemetry.track_meters[t].peak_r,
                                            telemetry.track_meters[t].rms_l, telemetry.track_meters[t].rms_r,
                                            telemetry.track_meters[t].peak_l >= 1.0f);
                            ImGui::Dummy(ImVec2(26, 110));

                            // Numerical dB read
                            float db = ui::linear_to_db(track_gains[t]);
                            ImGui::Text("%.1f dB", db);
                        }
                        ImGui::EndChild();
                        ImGui::SameLine();
                        ImGui::PopID();
                    }

                    // Render Submix Bus A: Drums Strip
                    ImGui::PushID(500);
                    ImGui::BeginChild("BusA_Strip", ImVec2(190, 0), true);
                    {
                        ImGui::TextColored(ImVec4(0.85f, 0.48f, 0.05f, 1.0f), "BUS A: DRUMS");
                        ImGui::Separator();
                        ImGui::TextDisabled("Target: MASTER");

                        ImGui::SetNextItemWidth(110);
                        if (ImGui::Combo("Console##b1", &bus_consoles[0], console_types, 4)) {
                            protocol::MixerCommand cmd{};
                            cmd.type = protocol::MixerCommandType::SetBusConsoleType;
                            cmd.target_id = bus_drums->id();
                            cmd.flags = bus_consoles[0];
                            mixer.post_command(cmd);
                        }

                        ImGui::Spacing();
                        ImGui::TextColored(ImVec4(0.35f, 0.40f, 0.48f, 1.0f), "[Glue Compressor]");

                        ImGui::Spacing();
                        if (ImGui::Checkbox("M##ba", &bus_mutes[0])) {
                            protocol::MixerCommand cmd{};
                            cmd.type = protocol::MixerCommandType::SetBusMute;
                            cmd.target_id = bus_drums->id();
                            cmd.flags = bus_mutes[0] ? 1 : 0;
                            mixer.post_command(cmd);
                        }
                        ImGui::SameLine();
                        if (ImGui::Checkbox("S##ba", &bus_solos[0])) {
                            protocol::MixerCommand cmd{};
                            cmd.type = protocol::MixerCommandType::SetBusSolo;
                            cmd.target_id = bus_drums->id();
                            cmd.flags = bus_solos[0] ? 1 : 0;
                            mixer.post_command(cmd);
                        }

                        ImGui::VSliderFloat("##bfaderA", ImVec2(34, 110), &bus_gains[0], 0.0f, 1.25f, "");
                        if (ImGui::IsItemEdited()) {
                            protocol::MixerCommand cmd{};
                            cmd.type = protocol::MixerCommandType::SetBusGain;
                            cmd.target_id = bus_drums->id();
                            cmd.value1 = bus_gains[0];
                            mixer.post_command(cmd);
                        }
                        ImGui::SameLine();
                        ImVec2 mpos = ImGui::GetCursorScreenPos();
                        ui::DrawDbMeter(ImGui::GetWindowDrawList(), mpos, ImVec2(24, 110),
                                        telemetry.bus_meters[0].peak_l, telemetry.bus_meters[0].peak_r,
                                        telemetry.bus_meters[0].rms_l, telemetry.bus_meters[0].rms_r,
                                        telemetry.bus_meters[0].peak_l >= 1.0f);
                        ImGui::Dummy(ImVec2(26, 110));
                        ImGui::Text("%.1f dB", ui::linear_to_db(bus_gains[0]));
                    }
                    ImGui::EndChild();
                    ImGui::SameLine();
                    ImGui::PopID();

                    // Render Submix Bus B: Music Strip
                    ImGui::PushID(501);
                    ImGui::BeginChild("BusB_Strip", ImVec2(190, 0), true);
                    {
                        ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "BUS B: MUSIC");
                        ImGui::Separator();
                        ImGui::TextDisabled("Target: MASTER");

                        ImGui::SetNextItemWidth(110);
                        if (ImGui::Combo("Console##b2", &bus_consoles[1], console_types, 4)) {
                            protocol::MixerCommand cmd{};
                            cmd.type = protocol::MixerCommandType::SetBusConsoleType;
                            cmd.target_id = bus_music->id();
                            cmd.flags = bus_consoles[1];
                            mixer.post_command(cmd);
                        }

                        ImGui::Spacing();
                        ImGui::TextColored(ImVec4(0.35f, 0.40f, 0.48f, 1.0f), "[Analog Summing]");

                        ImGui::Spacing();
                        if (ImGui::Checkbox("M##bb", &bus_mutes[1])) {
                            protocol::MixerCommand cmd{};
                            cmd.type = protocol::MixerCommandType::SetBusMute;
                            cmd.target_id = bus_music->id();
                            cmd.flags = bus_mutes[1] ? 1 : 0;
                            mixer.post_command(cmd);
                        }
                        ImGui::SameLine();
                        if (ImGui::Checkbox("S##bb", &bus_solos[1])) {
                            protocol::MixerCommand cmd{};
                            cmd.type = protocol::MixerCommandType::SetBusSolo;
                            cmd.target_id = bus_music->id();
                            cmd.flags = bus_solos[1] ? 1 : 0;
                            mixer.post_command(cmd);
                        }

                        ImGui::VSliderFloat("##bfaderB", ImVec2(34, 110), &bus_gains[1], 0.0f, 1.25f, "");
                        if (ImGui::IsItemEdited()) {
                            protocol::MixerCommand cmd{};
                            cmd.type = protocol::MixerCommandType::SetBusGain;
                            cmd.target_id = bus_music->id();
                            cmd.value1 = bus_gains[1];
                            mixer.post_command(cmd);
                        }
                        ImGui::SameLine();
                        ImVec2 mpos = ImGui::GetCursorScreenPos();
                        ui::DrawDbMeter(ImGui::GetWindowDrawList(), mpos, ImVec2(24, 110),
                                        telemetry.bus_meters[1].peak_l, telemetry.bus_meters[1].peak_r,
                                        telemetry.bus_meters[1].rms_l, telemetry.bus_meters[1].rms_r,
                                        telemetry.bus_meters[1].peak_l >= 1.0f);
                        ImGui::Dummy(ImVec2(26, 110));
                        ImGui::Text("%.1f dB", ui::linear_to_db(bus_gains[1]));
                    }
                    ImGui::EndChild();
                    ImGui::SameLine();
                    ImGui::PopID();

                    // Render Master Out Strip
                    ImGui::PushID(502);
                    ImGui::BeginChild("MasterOut_Strip", ImVec2(180, 0), true);
                    {
                        ImGui::TextColored(ImVec4(0.85f, 0.18f, 0.22f, 1.0f), "MASTER OUT");
                        ImGui::Separator();
                        ImGui::TextDisabled("Physical DAC");

                        ImGui::Spacing();
                        if (ImGui::Checkbox("LIMITER##mst", &master_limiter)) {
                            protocol::MixerCommand cmd{};
                            cmd.type = protocol::MixerCommandType::SetMasterLimiter;
                            cmd.flags = master_limiter ? 1 : 0;
                            mixer.post_command(cmd);
                        }

                        ImGui::Spacing();
                        ImGui::VSliderFloat("##mfader", ImVec2(34, 110), &master_gain, 0.0f, 1.25f, "");
                        if (ImGui::IsItemEdited()) {
                            protocol::MixerCommand cmd{};
                            cmd.type = protocol::MixerCommandType::SetMasterGain;
                            cmd.value1 = master_gain;
                            mixer.post_command(cmd);
                        }
                        ImGui::SameLine();
                        ImVec2 mpos = ImGui::GetCursorScreenPos();
                        ui::DrawDbMeter(ImGui::GetWindowDrawList(), mpos, ImVec2(24, 110),
                                        telemetry.master_meter.peak_l, telemetry.master_meter.peak_r,
                                        telemetry.master_meter.rms_l, telemetry.master_meter.rms_r,
                                        telemetry.master_meter.peak_l >= 1.0f);
                        ImGui::Dummy(ImVec2(26, 110));
                        ImGui::Text("%.1f dB", ui::linear_to_db(master_gain));
                    }
                    ImGui::EndChild();
                    ImGui::PopID();

                    ImGui::EndTabItem();
                }

                // ------------------------------------------------------------
                // TAB 2: SAMPLE EDITOR / SLICER
                // ------------------------------------------------------------
                if (ImGui::BeginTabItem("  SAMPLE EDITOR / SLICER  ")) {
                    // Track Selector Pills
                    ImGui::Text("Active Track:");
                    const char* trk_labels[4] = { "Track 1: Kick / 808", "Track 2: Acid 303", "Track 3: Vocal", "Track 4: Drums" };
                    for (int t = 0; t < 4; ++t) {
                        ImGui::SameLine();
                        if (t == selected_track) {
                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.38f, 0.85f, 1.0f));
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                        }
                        if (ImGui::Button(trk_labels[t])) {
                            selected_track = t;
                            sync_track_clip(selected_track, track_clips[selected_track]);
                        }
                        if (t == selected_track) {
                            ImGui::PopStyleColor(2);
                        }
                    }

                    auto cur_clip = track_clips[selected_track];

                    ImGui::SameLine(0, 20);
                    ImGui::TextColored(ImVec4(0.85f, 0.48f, 0.05f, 1.0f), "[STATUS]");
                    ImGui::SameLine();
                    ImGui::TextDisabled("%s", status_toast);

                    ImGui::Separator();

                    // Audio File I/O Bar (Drag & Drop or Manual Path)
                    ImGui::SetNextItemWidth(340);
                    ImGui::InputTextWithHint("##ManualWavPath", "Type / paste absolute .wav path...", manual_wav_path, sizeof(manual_wav_path));
                    ImGui::SameLine();
                    if (ImGui::Button("  LOAD WAV  ")) {
                        if (std::strlen(manual_wav_path) > 0) {
                            auto new_clip = std::make_shared<sampling::AudioClip>();
                            if (new_clip->load_from_wav(manual_wav_path)) {
                                track_clips_orig[selected_track] = new_clip;
                                sync_track_clip(selected_track, new_clip);
                                std::snprintf(status_toast, sizeof(status_toast), "LOADED: %s (%u Hz, %u frames)",
                                              new_clip->name().c_str(), new_clip->sample_rate(), new_clip->num_frames());
                            } else {
                                std::snprintf(status_toast, sizeof(status_toast), "ERROR: FAILED TO LOAD WAV: %s", manual_wav_path);
                            }
                        }
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("  EXPORT 24-BIT WAV  ")) {
                        if (cur_clip) {
                            std::string exp_path = "/tmp/" + cur_clip->name() + "_master.wav";
                            if (cur_clip->save_to_wav(exp_path, 24)) {
                                std::snprintf(status_toast, sizeof(status_toast), "SAVED 24-BIT MASTER TO: %s", exp_path.c_str());
                            }
                        }
                    }
                    ImGui::SameLine(0, 20);
                    if (cur_clip) {
                        float clip_dur = static_cast<float>(cur_clip->num_frames()) / cur_clip->sample_rate();
                        ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "%s", cur_clip->name().c_str());
                        ImGui::SameLine();
                        ImGui::TextDisabled("| %u Hz | %uch | %.2fs (%.1f Bars)",
                                            cur_clip->sample_rate(), cur_clip->num_channels(), clip_dur, clip_dur / 2.0f);
                    }

                    // High-Resolution Waveform Display
                    ImVec2 wf_pos = ImGui::GetCursorScreenPos();
                    ImVec2 wf_size(ImGui::GetContentRegionAvail().x, 100);
                    float play_ratio = playhead_seconds / loop_length_seconds;
                    ui::DrawWaveformDisplay(ImGui::GetWindowDrawList(), wf_pos, wf_size,
                                           sample_waveform.data(), sample_waveform.size(),
                                           play_ratio, slice_points, active_slice);
                    ImGui::Dummy(wf_size);

                    // Processing Control Groups (4 Panels)
                    float panel_w = (ImGui::GetContentRegionAvail().x - 36.0f) / 4.0f;

                    // Panel A: Mastering Normalization
                    ImGui::BeginChild("PanelNorm", ImVec2(panel_w, 140), true);
                    {
                        ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "MASTERING NORMALIZATION");
                        ImGui::Separator();
                        if (ImGui::Button("Peak Normalize (-0.1 dBFS)", ImVec2(-1, 24))) {
                            if (cur_clip) {
                                cur_clip->normalize_peak(0.98855f);
                                sync_track_clip(selected_track, cur_clip);
                                std::snprintf(status_toast, sizeof(status_toast), "PEAK NORMALIZED TO -0.1 dBFS");
                            }
                        }
                        if (ImGui::Button("RMS Normalize (-14 dBFS K-14)", ImVec2(-1, 24))) {
                            if (cur_clip) {
                                cur_clip->normalize_rms(-14.0f, 0.98855f);
                                sync_track_clip(selected_track, cur_clip);
                                std::snprintf(status_toast, sizeof(status_toast), "RMS NORMALIZED TO -14 dBFS (K-14)");
                            }
                        }
                        if (ImGui::Button("Remove DC Offset", ImVec2(-1, 24))) {
                            if (cur_clip) {
                                cur_clip->remove_dc_offset();
                                sync_track_clip(selected_track, cur_clip);
                                std::snprintf(status_toast, sizeof(status_toast), "DC OFFSET REMOVED (CENTERED AT 0.0)");
                            }
                        }
                    }
                    ImGui::EndChild();

                    ImGui::SameLine();

                    // Panel B: Sample Repair & Seam Inpainting
                    ImGui::BeginChild("PanelRepair", ImVec2(panel_w, 140), true);
                    {
                        ImGui::TextColored(ImVec4(0.85f, 0.48f, 0.05f, 1.0f), "SAMPLE REPAIR & ANTI-CLICK");
                        ImGui::Separator();
                        if (ImGui::Button("Scan Cuts / Discontinuities", ImVec2(-1, 24))) {
                            if (cur_clip) {
                                auto cuts = sampling::SampleRepairEngine::detect_discontinuities(*cur_clip, 0.20f);
                                std::snprintf(status_toast, sizeof(status_toast), "SCAN RESULT: %zu CUTS DETECTED", cuts.size());
                            }
                        }
                        if (ImGui::Button("Heal Cuts (Hermite C1 Inpaint)", ImVec2(-1, 24))) {
                            if (cur_clip) {
                                uint32_t count = sampling::SampleRepairEngine::heal_clip(*cur_clip, 0.20f, 24, creative_click_bypass);
                                sync_track_clip(selected_track, cur_clip);
                                std::snprintf(status_toast, sizeof(status_toast),
                                              creative_click_bypass ? "CREATIVE CLICK MODE ACTIVE: BYPASSED HEALING"
                                                                    : "HEALED %u MID-WAVE CUTS VIA HERMITE INPAINTING", count);
                            }
                        }
                        if (ImGui::Button("Snap Slices to Zero-Crossings", ImVec2(-1, 24))) {
                            if (cur_clip) {
                                sampling::SampleRepairEngine::snap_all_slices_to_zero_crossings(*cur_clip, 64);
                                sync_track_clip(selected_track, cur_clip);
                                std::snprintf(status_toast, sizeof(status_toast), "SNAPPED ALL SLICES TO ZERO CROSSINGS");
                            }
                        }
                        ImGui::Checkbox("Creative Click Mode (Bypass)", &creative_click_bypass);
                    }
                    ImGui::EndChild();

                    ImGui::SameLine();

                    // Panel C: Multi-Engine Pitch & Time Stretcher
                    ImGui::BeginChild("PanelStretch", ImVec2(panel_w, 140), true);
                    {
                        ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "PITCH & TIME STRETCH SUITE");
                        ImGui::Separator();
                        const char* algo_names[4] = {
                            "1: Vinyl Variclock",
                            "2: Vintage 12-Bit MPC",
                            "3: Rubberband WSOLA",
                            "4: Sovereign ODE Kinetic"
                        };
                        ImGui::SetNextItemWidth(panel_w * 0.60f);
                        ImGui::Combo("##Algo", &pitch_algo_mode, algo_names, 4);
                        ImGui::SameLine();
                        if (ImGui::Button("RESET##Orig", ImVec2(-1, 20))) {
                            if (track_clips_orig[selected_track]) {
                                sync_track_clip(selected_track, track_clips_orig[selected_track]);
                                sample_pitch_shift = 0.0f;
                                sample_time_stretch = 1.0f;
                                std::snprintf(status_toast, sizeof(status_toast), "REVERTED TO ORIGINAL AUDIO");
                            }
                        }

                        ImGui::SetNextItemWidth(panel_w * 0.45f);
                        ImGui::SliderFloat("Pitch", &sample_pitch_shift, -24.0f, 24.0f, "%.1f st");
                        ImGui::SameLine();
                        ImGui::SetNextItemWidth(panel_w * 0.45f);
                        ImGui::SliderFloat("Stretch", &sample_time_stretch, 0.25f, 4.0f, "%.2fx");

                        if (ImGui::Button("PROCESS & APPLY STRETCH", ImVec2(-1, 24))) {
                            auto orig = track_clips_orig[selected_track];
                            if (orig) {
                                auto algo = static_cast<dsp::PitchAlgorithm>(pitch_algo_mode);
                                auto stretched = dsp::PitchTimeStretcher::process(*orig, algo, sample_pitch_shift, sample_time_stretch);
                                if (stretched) {
                                    sync_track_clip(selected_track, stretched);
                                    std::snprintf(status_toast, sizeof(status_toast), "STRETCHED VIA %s (%u frames)",
                                                  algo_names[pitch_algo_mode], stretched->num_frames());
                                }
                            }
                        }
                    }
                    ImGui::EndChild();

                    ImGui::SameLine();

                    // Panel D: Airwindows DeRez2 Vintage Sampler Decimator
                    ImGui::BeginChild("PanelDeRez", ImVec2(panel_w, 140), true);
                    {
                        ImGui::TextColored(ImVec4(0.85f, 0.20f, 0.35f, 1.0f), "AIRWINDOWS DEREZ2 CRUNCH");
                        ImGui::Separator();
                        ImGui::SetNextItemWidth(panel_w * 0.45f);
                        ImGui::SliderFloat("Rate##DR", &derez_rate, 0.0f, 1.0f, "%.2f");
                        ImGui::SameLine();
                        ImGui::SetNextItemWidth(panel_w * 0.45f);
                        ImGui::SliderFloat("Bits##DR", &derez_res, 0.0f, 1.0f, "%.2f");

                        ImGui::SetNextItemWidth(panel_w * 0.45f);
                        ImGui::SliderFloat("Hard##DR", &derez_hard, 0.0f, 1.0f, "%.2f");
                        ImGui::SameLine();
                        ImGui::SetNextItemWidth(panel_w * 0.45f);
                        ImGui::SliderFloat("Wet##DR", &derez_wet, 0.0f, 1.0f, "%.2f");

                        float btn_w = (panel_w - 24.0f) / 3.0f;
                        if (ImGui::Button("SP-1200", ImVec2(btn_w, 20))) {
                            derez_rate = 0.85f; derez_res = 0.70f; derez_hard = 0.0f;
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("Mirage", ImVec2(btn_w, 20))) {
                            derez_rate = 0.70f; derez_res = 0.40f; derez_hard = 0.0f;
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("Lo-Fi 4b", ImVec2(btn_w, 20))) {
                            derez_rate = 0.50f; derez_res = 0.20f; derez_hard = 1.0f;
                        }

                        if (ImGui::Button("APPLY DEREZ2 CRUNCH", ImVec2(-1, 24))) {
                            if (cur_clip) {
                                dsp::DeRez proc;
                                proc.init(cur_clip->sample_rate());
                                proc.set_parameter(0, derez_rate);
                                proc.set_parameter(1, derez_res);
                                proc.set_parameter(2, derez_hard);
                                proc.set_parameter(3, derez_wet);
                                proc.process_stereo(cur_clip->channel(0), cur_clip->channel(1), cur_clip->num_frames());
                                sync_track_clip(selected_track, cur_clip);
                                std::snprintf(status_toast, sizeof(status_toast),
                                              "APPLIED DEREZ2 CRUNCH (Rate=%.2f, Res=%.2f, Hard=%.2f)",
                                              derez_rate, derez_res, derez_hard);
                            }
                        }
                    }
                    ImGui::EndChild();

                    ImGui::EndTabItem();
                }

                // ------------------------------------------------------------
                // TAB 3: ENVELOPES & AUTOMATION
                // ------------------------------------------------------------
                if (ImGui::BeginTabItem("  ENVELOPES & AUTOMATION  ")) {
                    ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f),
                                       "Liquid ODE Trapezoidal Modulation Envelope (A-Stable C^inf)");
                    ImGui::Separator();

                    ImVec2 env_pos = ImGui::GetCursorScreenPos();
                    ImVec2 env_size(ImGui::GetContentRegionAvail().x - 260.0f, 160.0f);
                    ui::DrawEnvelopeCurve(ImGui::GetWindowDrawList(), env_pos, env_size,
                                         env_attack, env_decay, env_sustain, env_release, env_tau);
                    ImGui::Dummy(env_size);

                    ImGui::SameLine(0, 20);
                    ImGui::BeginGroup();
                    {
                        ImGui::SetNextItemWidth(120);
                        ImGui::SliderFloat("Attack", &env_attack, 1.0f, 500.0f, "%.0f ms");
                        ImGui::SetNextItemWidth(120);
                        ImGui::SliderFloat("Decay", &env_decay, 10.0f, 1000.0f, "%.0f ms");
                        ImGui::SetNextItemWidth(120);
                        ImGui::SliderFloat("Sustain", &env_sustain, 0.0f, 1.0f, "%.2f");
                        ImGui::SetNextItemWidth(120);
                        ImGui::SliderFloat("Release", &env_release, 10.0f, 2000.0f, "%.0f ms");
                        ImGui::SetNextItemWidth(120);
                        ImGui::SliderFloat("ODE Tau (Curvature)", &env_tau, 0.1f, 5.0f, "%.2f");
                    }
                    ImGui::EndGroup();

                    ImGui::EndTabItem();
                }

                // ------------------------------------------------------------
                // TAB 4: WASM PLUGIN RACK & GAS WATCHDOG
                // ------------------------------------------------------------
                if (ImGui::BeginTabItem("  WASM PLUGIN RACK & GAS METER  ")) {
                    ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f),
                                       "Sovereign WebAssembly Host (Lock-Free RCU Hot-Swap & Gas Metering)");
                    ImGui::Separator();

                    ImGui::Text("Active Plugin: ");
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(0.85f, 0.48f, 0.05f, 1.0f), "saturator.wasm (936 Bytes)");

                    ImGui::Spacing();
                    ImVec2 gas_pos = ImGui::GetCursorScreenPos();
                    ui::DrawGasMeter(ImGui::GetWindowDrawList(), gas_pos, ImVec2(450, 24),
                                    wasm_gas_used, wasm_gas_limit, wasm_tripped);
                    ImGui::Dummy(ImVec2(450, 26));

                    ImGui::Spacing();
                    ImGui::Text("Introspected Parameters (sov_get_param_info):");
                    ImGui::SetNextItemWidth(200);
                    ImGui::SliderFloat("Drive / Harmonics", &wasm_param1, 0.0f, 1.0f, "%.2f");
                    ImGui::SetNextItemWidth(200);
                    ImGui::SliderFloat("Feedback / Asymmetry", &wasm_param2, 0.0f, 1.0f, "%.2f");

                    ImGui::Spacing();
                    if (ImGui::Button("Lock-Free Hot-Swap (Reload WASM)")) {
                        // Triggers RCU hot swap
                    }
                    ImGui::SameLine();
                    if (ImGui::Button(wasm_tripped ? "Reset Circuit Breaker" : "Simulate Runaway Loop")) {
                        wasm_tripped = !wasm_tripped;
                        wasm_gas_used = wasm_tripped ? 25.0f : 9.4f;
                    }

                    ImGui::EndTabItem();
                }

                ImGui::EndTabBar();
            }
        }
        ImGui::EndChild();

        ImGui::End();

        // 5. Render to OpenGL
        ImGui::Render();
        glViewport(0, 0, win_w, win_h);
        glClearColor(0.94f, 0.95f, 0.96f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());


        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    std::cout << "[Aethel Audio Desk] Terminated cleanly." << std::endl;
    return 0;
}
