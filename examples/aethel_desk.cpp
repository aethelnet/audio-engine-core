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
#include "audio_core/dsp/liquid_vactrol.hpp"
#include "audio_core/dsp/multihead_ode_compressor.hpp"
#include "audio_core/network/aoip_receiver.hpp"
#include "audio_core/serialization/session_serializer.hpp"
#include "audio_core/engine.hpp"
#include "audio_core/midi/hardware_midi_receiver.hpp"
#include "audio_core/midi/midi_sync.hpp"
#include "audio_core/midi/midi_learn_router.hpp"
#include "audio_core/routing/mseg_automation_bridge.hpp"
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
#include <filesystem>

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
    trk0->set_sequencer_send_a_bus(bus_reverb->id());
    trk0->set_sequencer_send_b_bus(bus_delay->id());
    trk1->set_sequencer_send_a_bus(bus_reverb->id());
    trk1->set_sequencer_send_b_bus(bus_delay->id());
    trk2->set_sequencer_send_a_bus(bus_reverb->id());
    trk2->set_sequencer_send_b_bus(bus_delay->id());
    trk3->set_sequencer_send_a_bus(bus_reverb->id());
    trk3->set_sequencer_send_b_bus(bus_delay->id());

    // Route tracks to Submix buses by default:
    // Track 1 (Kick/808) -> Bus A (Drums)
    // Track 2 (Acid 303) -> Bus B (Music)
    // Track 3 (Vocal)    -> Bus B (Music)
    // Track 4 (Drums)    -> Master
    trk0->set_target_bus(bus_drums->id());
    trk1->set_target_bus(bus_music->id());
    trk2->set_target_bus(bus_music->id());
    trk3->set_target_bus(0); // Master Out

    // Tracks 0-3 start completely clean and transparent (Zähl AM1 console baseline)
    // Inserts are user-assignable via Channel Strip slots S1..S4
    auto drum_bus_comp = std::make_shared<dsp::ButterComp2>();
    drum_bus_comp->init(kSampleRate);
    drum_bus_comp->set_parameter(0, 0.35f); // Optional drum bus glue
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

    // Initialize Track 3 (Percussion / Bus audio)
    auto perc_clip = std::make_shared<sampling::AudioClip>("Drums / Bus", kSampleRate, 2, static_cast<uint32_t>(sample_waveform.size()));
    for (size_t i = 0; i < sample_waveform.size(); ++i) {
        perc_clip->channel(0)[i] = sample_waveform[i] * 0.7f;
        perc_clip->channel(1)[i] = sample_waveform[i] * 0.7f;
    }
    trk3->set_clip(perc_clip, true);
    trk3->set_sync_to_transport(true);

    // Auto-slice clips into 8-slice grids for StepSequencers
    if (drum_clip->slices().empty()) drum_clip->slice_grid(8);
    if (acid_clip->slices().empty()) acid_clip->slice_grid(8);
    if (vocal_clip->slices().empty()) vocal_clip->slice_grid(8);
    if (perc_clip->slices().empty()) perc_clip->slice_grid(8);

    // Initialize StepSequencers for each track
    std::shared_ptr<sequencer::StepSequencer> track_seq[4];
    track_seq[0] = std::make_shared<sequencer::StepSequencer>(drum_clip);
    track_seq[1] = std::make_shared<sequencer::StepSequencer>(acid_clip);
    track_seq[2] = std::make_shared<sequencer::StepSequencer>(vocal_clip);
    track_seq[3] = std::make_shared<sequencer::StepSequencer>(perc_clip);

    trk0->set_sequencer(track_seq[0]);
    trk1->set_sequencer(track_seq[1]);
    trk2->set_sequencer(track_seq[2]);
    trk3->set_sequencer(track_seq[3]);

    trk0->enable_sequencer(false);
    trk1->enable_sequencer(false);
    trk2->enable_sequencer(false);
    trk3->enable_sequencer(false);

    // Populate standard melodic/drum patterns for each track
    // Track 0 (Drums):
    track_seq[0]->pattern(0).name = "Straight Beat";
    track_seq[0]->pattern(0).set_step(0, 0, 1.0f);
    track_seq[0]->pattern(0).set_step(4, 1, 0.9f);
    track_seq[0]->pattern(0).set_step(8, 0, 1.0f);
    track_seq[0]->pattern(0).set_step(12, 1, 0.95f);
    for (int st = 2; st < 16; st += 2) {
        if (!track_seq[0]->pattern(0).is_step_active(st))
            track_seq[0]->pattern(0).set_step(st, 2, 0.7f);
    }
    track_seq[0]->pattern(1).name = "Syncopated Break";
    track_seq[0]->pattern(1).set_step(0, 0, 1.0f);
    track_seq[0]->pattern(1).set_step(3, 2, 0.6f);
    track_seq[0]->pattern(1).set_step(4, 1, 0.9f);
    track_seq[0]->pattern(1).set_step(6, 0, 0.85f);
    track_seq[0]->pattern(1).set_step(10, 0, 0.95f);
    track_seq[0]->pattern(1).set_step(12, 1, 1.0f);
    track_seq[0]->pattern(1).set_step(14, 2, 0.75f);
    track_seq[0]->pattern(2).name = "Rapid Roll";
    for (int st = 0; st < 16; ++st) {
        track_seq[0]->pattern(2).set_step(st, st % 4, (st % 4 == 0) ? 1.0f : 0.65f);
    }

    // Track 1 (Acid 303 Lead):
    track_seq[1]->pattern(0).name = "Acid Groove";
    track_seq[1]->pattern(0).set_step(0, 0, 0.9f, 1.0f);
    track_seq[1]->pattern(0).set_step(2, 1, 0.8f, 1.0f);
    track_seq[1]->pattern(0).set_step(4, 0, 1.0f, 2.0f);
    track_seq[1]->pattern(0).set_step(6, 2, 0.85f, 1.0f);
    track_seq[1]->pattern(0).set_step(8, 0, 0.9f, 0.5f);
    track_seq[1]->pattern(0).set_step(10, 1, 0.8f, 1.0f);
    track_seq[1]->pattern(0).set_step(12, 3, 1.0f, 1.0f);
    track_seq[1]->pattern(0).set_step(14, 2, 0.7f, 1.5f);
    track_seq[1]->pattern(1).name = "Driving Arp";
    for (int st = 0; st < 16; ++st) {
        float pit = (st % 2 == 0) ? 1.0f : 2.0f;
        track_seq[1]->pattern(1).set_step(st, st % 3, 0.85f, pit);
    }
    track_seq[1]->pattern(2).name = "Acid Stabs";
    track_seq[1]->pattern(2).set_step(0, 0, 1.0f);
    track_seq[1]->pattern(2).set_step(6, 1, 0.95f);
    track_seq[1]->pattern(2).set_step(8, 0, 1.0f);
    track_seq[1]->pattern(2).set_step(14, 2, 0.9f);

    // Track 2 (Vocal Chops):
    track_seq[2]->pattern(0).name = "Hook Chop";
    track_seq[2]->pattern(0).set_step(0, 0, 0.95f);
    track_seq[2]->pattern(0).set_step(6, 1, 0.9f);
    track_seq[2]->pattern(0).set_step(10, 2, 1.0f);
    track_seq[2]->pattern(1).name = "Glitch Stutter";
    track_seq[2]->pattern(1).set_step(0, 0, 1.0f);
    track_seq[2]->pattern(1).set_step(1, 0, 0.8f);
    track_seq[2]->pattern(1).set_step(2, 0, 0.6f);
    track_seq[2]->pattern(1).set_step(8, 1, 0.95f);
    track_seq[2]->pattern(1).set_step(12, 2, 1.0f);
    track_seq[2]->pattern(2).name = "Ambient Echo";
    track_seq[2]->pattern(2).set_step(0, 2, 0.9f, 1.0f, 100, true);
    track_seq[2]->pattern(2).set_step(8, 1, 0.85f);

    // Track 3 (Drums/Perc):
    track_seq[3]->pattern(0).name = "HiHat Roll";
    for (int st = 0; st < 16; st += 2) {
        track_seq[3]->pattern(0).set_step(st, 2, 0.75f);
    }
    track_seq[3]->pattern(1).name = "Syncopated Perc";
    track_seq[3]->pattern(1).set_step(2, 3, 0.85f);
    track_seq[3]->pattern(1).set_step(5, 3, 0.75f);
    track_seq[3]->pattern(1).set_step(10, 3, 0.9f);
    track_seq[3]->pattern(2).name = "Full 16th Hats";
    for (int st = 0; st < 16; ++st) {
        track_seq[3]->pattern(2).set_step(st, 2, (st % 4 == 0) ? 0.95f : 0.6f);
    }

    // Initialize Clip Launcher slots for Tracks 0-3 (Hybrid Audio Loops & Sequencer Patterns)
    // Track 0 (Drums):
    trk0->clip_launcher().set_slot_hybrid(0, drum_clip, 0, sampling::PlaybackMode::BeatSyncTimeStretch, "Main Beat");
    trk0->clip_launcher().set_slot_hybrid(1, drum_clip, 1, sampling::PlaybackMode::BeatSyncTimeStretch, "Breakbeat");
    trk0->clip_launcher().set_slot_hybrid(2, drum_clip, 2, sampling::PlaybackMode::ReverseFree, "Glitch Rev");

    // Track 1 (Acid 303):
    trk1->clip_launcher().set_slot_hybrid(0, acid_clip, 0, sampling::PlaybackMode::BeatSyncTimeStretch, "Acid Groove");
    trk1->clip_launcher().set_slot_hybrid(1, acid_clip, 1, sampling::PlaybackMode::PitchShiftWsola, "Acid +7st");
    trk1->clip_launcher().slot(1).pitch_semitones = 7.0f;
    trk1->clip_launcher().set_slot_hybrid(2, acid_clip, 2, sampling::PlaybackMode::BeatSyncRepitch, "Acid Sub -12st");
    trk1->clip_launcher().slot(2).pitch_semitones = -12.0f;

    // Track 2 (Vocal Chops):
    trk2->clip_launcher().set_slot_hybrid(0, vocal_clip, 0, sampling::PlaybackMode::BeatSyncTimeStretch, "Vocal Hook");
    trk2->clip_launcher().set_slot_hybrid(1, vocal_clip, 1, sampling::PlaybackMode::BeatSyncTimeStretch, "Vocal Glitch");
    trk2->clip_launcher().set_slot_hybrid(2, vocal_clip, 2, sampling::PlaybackMode::PitchShiftWsola, "Vocal Pad -5st");
    trk2->clip_launcher().slot(2).pitch_semitones = -5.0f;

    // Track 3 (Percussion):
    trk3->clip_launcher().set_slot_hybrid(0, perc_clip, 0, sampling::PlaybackMode::BeatSyncTimeStretch, "HiHat 8th");
    trk3->clip_launcher().set_slot_hybrid(1, perc_clip, 1, sampling::PlaybackMode::BeatSyncTimeStretch, "Syncopated");
    trk3->clip_launcher().set_slot_hybrid(2, perc_clip, 2, sampling::PlaybackMode::BeatSyncTimeStretch, "Fast 16th");

    // Initialize Low-Latency AoIP Network Stream Receiver (Dante / AES67 / UDP:4848)
    network::AoipReceiver aoip_rx(4848);
    aoip_rx.bind_port(4848);
    aoip_rx.map_channel_pair(1, 0, 1);
    aoip_rx.map_channel_pair(2, 2, 3);
    aoip_rx.map_channel_pair(3, 4, 5);
    aoip_rx.map_channel_pair(4, 6, 7);
    aoip_rx.start();

    // Initialize Native PipeWire Audio Server Integration
    PipeWireBackend pw(mixer);
    pw.set_aoip_receiver(&aoip_rx);
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
    std::shared_ptr<sampling::AudioClip> track_clips[4] = { drum_clip, acid_clip, vocal_clip, perc_clip };
    std::shared_ptr<sampling::AudioClip> track_clips_orig[4] = { drum_clip, acid_clip, vocal_clip, perc_clip };

    // Sample Editor, Pitch & Repair state
    int pitch_algo_mode = 0; // 0=Vinyl, 1=Vintage MPC, 2=WSOLA, 3=Sovereign ODE
    float sample_pitch_shift = 0.0f;
    float sample_time_stretch = 1.0f;
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
        if (clip->slices().empty()) clip->slice_grid(8);
        if (track_seq[t]) track_seq[t]->set_clip(clip);
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

    // SampleTap Recorder & Quantized Bouncer State
    int tap_source_idx = 0; // 0=Master Out, 1=Trk 1 Post, 2=Trk 2 Post, 3=Trk 3 Post, 4=Trk 4 Post, 5=Bus 1 Drum Glue, 6=Trk 1 Pre, 7=Trk 2 Pre
    bool tap_auto_seamless = true;
    auto* tap0 = mixer.tap(0);
    if (tap0) {
        tap0->set_source(sampling::TapSourceType::MasterOutput, 0);
        tap0->set_rolling_mode();
    }

    auto update_tap_source = [&](int src_idx) {
        if (!tap0) return;
        tap_source_idx = src_idx;
        switch (src_idx) {
            case 0: tap0->set_source(sampling::TapSourceType::MasterOutput, 0); break;
            case 1: tap0->set_source(sampling::TapSourceType::TrackOutput, trk0->id()); break;
            case 2: tap0->set_source(sampling::TapSourceType::TrackOutput, trk1->id()); break;
            case 3: tap0->set_source(sampling::TapSourceType::TrackOutput, trk2->id()); break;
            case 4: tap0->set_source(sampling::TapSourceType::TrackOutput, trk3->id()); break;
            case 5: tap0->set_source(sampling::TapSourceType::BusOutput, 1); break;
            case 6: tap0->set_source(sampling::TapSourceType::TrackInput, trk0->id()); break;
            case 7: tap0->set_source(sampling::TapSourceType::TrackInput, trk1->id()); break;
            default: tap0->set_source(sampling::TapSourceType::MasterOutput, 0); break;
        }
        tap0->set_rolling_mode();
    };

    // Track UI state caches
    float track_gains[4] = { 0.85f, 0.70f, 0.80f, 0.90f };
    float track_pans[4] = { 0.0f, -0.25f, 0.30f, 0.0f };
    bool track_mutes[4] = { false, false, false, false };
    bool track_solos[4] = { false, false, false, false };
    bool track_solo_safes[4] = { false, false, false, false };
    int track_consoles[4] = { 1, 2, 3, 0 }; // Warm, Lush, etc.

    // Clip Launcher & Arranger state
    bool loop_region_active = true;
    int pattern_editor_pat_idx = 0; // 0..3 (Patterns 1..4)
    int pattern_editor_step_idx = 0; // 0..15

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

    // Modulator Lab & Meta-Modulation Engine state
    modulation::ModulationMatrix mod_matrix;
    mod_matrix.init(kSampleRate);
    mod_matrix.mseg1().preset_percussive_hihat();
    mod_matrix.mseg2().preset_plucked_synth();
    mod_matrix.lfo1().set_waveform(modulation::LfoWaveform::Sine);
    mod_matrix.lfo1().set_frequency_hz(3.0f);
    mod_matrix.lfo1().set_depth(0.85f);
    mod_matrix.lfo2().set_waveform(modulation::LfoWaveform::SmoothRandom);
    mod_matrix.lfo2().set_beat_sync(true);
    mod_matrix.lfo2().set_beats_per_cycle(1.0);
    mod_matrix.lfo2().set_depth(0.6f);

    // Initial Default Routes:
    // Route 0: LFO 1 -> MSEG 1 Attack (Modulating the Modulator! User's explicit request)
    mod_matrix.set_route(0, modulation::ModulationSource::LFO1, modulation::ModulationDestination::MSEG1_Attack, 1.5f, true);
    // Route 1: MSEG 1 -> Synth Cutoff
    mod_matrix.set_route(1, modulation::ModulationSource::MSEG1, modulation::ModulationDestination::SynthCutoff, 0.75f, true);
    // Route 2: Velocity -> MSEG 1 Level
    mod_matrix.set_route(2, modulation::ModulationSource::Velocity, modulation::ModulationDestination::MSEG1_Level, 0.5f, true);
    // Route 3: LFO 2 -> LFO 1 Rate (FM between modulators)
    mod_matrix.set_route(3, modulation::ModulationSource::LFO2, modulation::ModulationDestination::LFO1_Rate, 0.35f, true);

    // Hardware MIDI Ingestion & Auto-Routing Setup
    midi::HardwareMidiReceiver midi_rx;
    midi_rx.auto_connect();

    // Dynamic MIDI Learn Router
    midi::MidiLearnRouter midi_learn;
    midi_learn.bind(midi::MidiLearnRouter::kOmniChannel, 7, midi::MidiLearnTargetType::MasterVolume, 0, 0, 0, 0.0f, 1.25f, "Master Volume (CC 7)");
    midi_learn.bind(midi::MidiLearnRouter::kOmniChannel, 74, midi::MidiLearnTargetType::SynthParam, 0, 0, 0, 20.0f, 20000.0f, "PolySynth Cutoff (CC 74)");
    midi_learn.bind(midi::MidiLearnRouter::kOmniChannel, 71, midi::MidiLearnTargetType::SynthParam, 0, 0, 1, 0.5f, 15.0f, "PolySynth Res (CC 71)");

    // Auto-route Track 1 ("Acid 303 Lead" / Poly Synth) to PolyphonicSynth + ModulationMatrix
    trk1->set_name("Poly Synth / Lead");
    mixer.assign_track_poly_synth(trk1->id(), &mod_matrix.poly_synth(), &mod_matrix);

    int selected_modulator = 0; // 0=MSEG1 (Hi-Hat), 1=MSEG2 (Lead), 2=LFO1, 3=LFO2
    int selected_mseg_node = 0;
    float mod_preview_vel = 0.9f;

    // Universal Routing Matrix state: Pre-connect Track 1 Kick -> Track 2 Acid SC (120Hz Cytomic SVF)
    mixer.connect_sidechain(trk0->id(), trk1->id(), 0, 120.0f, routing::TapPoint::Input);
    uint32_t selected_patch_id = 1;
    int selected_curve_point = -1;
    std::vector<size_t> selected_curve_points;

    enum class LaneKind : uint8_t {
        TrackParam = 0,
        PluginParam = 1
    };

    struct StackedLaneDesc {
        LaneKind kind{LaneKind::TrackParam};
        routing::AutomationTarget track_target{routing::AutomationTarget::Gain};
        uint32_t slot_idx{0};
        uint32_t param_idx{0};
        bool collapsed{false};
    };

    int selected_auto_lane = 0; // 0=Gain, 1=Pan, 2=Aux1, 3=Aux2, 4+ = Slot S Param P
    int automation_view_mode = 0; // 0 = Focused Single Lane, 1 = Multi-Lane Stacked
    std::vector<StackedLaneDesc> stacked_lanes = {
        { LaneKind::TrackParam, routing::AutomationTarget::Gain, 0, 0, false },
        { LaneKind::TrackParam, routing::AutomationTarget::Pan, 0, 0, false }
    };
    int automation_context_mode = 0; // 0 = Track Timeline, 1 = Clip-Relative Envelope
    int selected_clip_lane = 0; // 0=Gain, 1=Pan, 2=Pitch

    protocol::MixerTelemetryFrame telemetry{};
    auto last_time = std::chrono::steady_clock::now();

    // Session Persistence & Offline Bounce State
    char session_file_path[256] = "session.json";
    char rack_preset_file_path[256] = "rack_preset.json";
    char bounce_wav_path[256] = "export_master.wav";
    int bounce_bars = 4;
    int bounce_bit_depth_idx = 1; // 0=16-bit, 1=24-bit, 2=32-bit Float
    bool bounce_normalize = true;
    float bounce_peak_target = -0.1f;
    std::string session_status_msg = "";
    auto session_status_time = std::chrono::steady_clock::now();

    bool open_save_session_modal = false;
    bool open_load_session_modal = false;
    bool open_bounce_modal = false;
    bool open_save_rack_modal = false;
    bool open_load_rack_modal = false;
    bool open_hardware_io_modal = false;

    auto sync_ui_from_mixer = [&]() {
        for (int i = 0; i < 4; ++i) {
            auto* t = mixer.get_track(i + 1);
            if (t) {
                track_gains[i] = t->gain();
                track_pans[i] = t->pan();
                track_mutes[i] = t->is_muted();
                track_solos[i] = t->is_solo();
                track_solo_safes[i] = t->is_solo_safe();
                track_consoles[i] = static_cast<int>(t->console_type());
                track_target_buses[i] = (t->target_bus() <= 0) ? 0 : t->target_bus();
            }
        }
        master_gain = mixer.master_volume();
        bpm = static_cast<float>(mixer.clock().bpm());
        for (int b = 0; b < 2; ++b) {
            auto* bus = mixer.get_bus(b + 1);
            if (bus) {
                bus_gains[b] = bus->gain();
                bus_mutes[b] = bus->is_muted();
                bus_solos[b] = bus->is_solo();
                bus_consoles[b] = static_cast<int>(bus->console_type());
            }
        }
    };

    auto do_offline_bounce = [&]() {
        Engine bounce_engine(kSampleRate, kBlockFrames);
        auto sess_data = serialization::SessionSerializer::extract_session(mixer, mixer.clock(), "Export Session");
        serialization::SessionSerializer::apply_session(bounce_engine.mixer(), bounce_engine.clock(), sess_data);

        BounceOptions b_opts{};
        b_opts.start_frame = 0;
        b_opts.total_frames = static_cast<uint64_t>(bounce_engine.clock().samples_per_bar() * bounce_bars);
        b_opts.tail_frames = static_cast<uint32_t>(kSampleRate * 1); // 1.0s decay tail
        b_opts.bits_per_sample = (bounce_bit_depth_idx == 0) ? 16 : ((bounce_bit_depth_idx == 1) ? 24 : 32);
        b_opts.apply_pdc_flush = true;
        b_opts.normalize = bounce_normalize;
        b_opts.target_peak_db = bounce_peak_target;

        auto b_res = bounce_engine.render_offline_wav(bounce_wav_path, b_opts);
        if (b_res.success) {
            char buf[256];
            std::snprintf(buf, sizeof(buf), "Exported %.1fs in %.3fs (%.1fx RT) -> %s",
                          b_res.duration_seconds, b_res.render_time_seconds, b_res.realtime_factor, bounce_wav_path);
            session_status_msg = buf;
        } else {
            session_status_msg = "Bounce error: " + b_res.error_message;
        }
        session_status_time = std::chrono::steady_clock::now();
    };

    // 4. Main Window Render Loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        auto now = std::chrono::steady_clock::now();
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
                mixer.clock().set_sample_position(static_cast<uint64_t>(playhead_seconds * 48000.0f));
            }
        } else if (!mixer.clock().is_scrubbing()) {
            playhead_seconds = static_cast<float>(mixer.clock().sample_position()) / static_cast<float>(kSampleRate);
        }

        // Drain incoming Hardware MIDI events through MidiLearnRouter into ModulationMatrix & PolyphonicSynth
        midi_rx.drain_to(midi_learn, mixer, &mod_matrix);
        midi_rx.sync_to_clock(mixer.clock());
        if (mixer.clock().authority() == clock::ClockAuthority::MidiClockSlave ||
            mixer.clock().authority() == clock::ClockAuthority::MtcSlave) {
            bpm = static_cast<float>(mixer.clock().bpm());
            is_playing = mixer.clock().is_playing();
            playhead_seconds = static_cast<float>(mixer.clock().sample_position()) / 48000.0f;
        }

        // Advance Modulator Matrix & Polyphonic Voice Pool at audio clock rate for real-time visual feedback
        uint32_t mod_sim_frames = std::clamp(static_cast<uint32_t>(dt * 48000.0f), 1u, 1024u);
        float synth_sim_l[1024];
        float synth_sim_r[1024];
        for (uint32_t s = 0; s < mod_sim_frames; ++s) {
            mod_matrix.evaluate_sample(bpm);
        }
        mod_matrix.process_synth_block(synth_sim_l, synth_sim_r, mod_sim_frames, bpm);

        // Master Clock Transmission (24 PPQN Beat Clock & MTC SMPTE Timecode):
        if (mixer.clock().authority() == clock::ClockAuthority::Master) {
            if (is_playing) {
                mixer.clock().set_sample_position(static_cast<uint64_t>(playhead_seconds * 48000.0f));
            }
            mixer.clock().set_playing(is_playing);
            mixer.clock().set_bpm(bpm);
            midi_rx.process_master_clock(is_playing ? mod_sim_frames : 0, mixer.clock());
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
                mixer.seek(0, true);
                playhead_seconds = 0.0f;
            }

            ImGui::SameLine();
            ImGui::SetNextItemWidth(100);
            if (mixer.clock().authority() == clock::ClockAuthority::MidiClockSlave) {
                ImGui::BeginDisabled();
                ImGui::SliderFloat("BPM", &bpm, 60.0f, 200.0f, "%.1f (MIDI)");
                ImGui::EndDisabled();
            } else if (ImGui::SliderFloat("BPM", &bpm, 60.0f, 200.0f, "%.1f")) {
                mixer.clock().set_bpm(bpm);
            }

            // Timecode & Musical Position Readout + Scrub Status
            auto tc = midi::MtcTimecode::from_seconds(playhead_seconds, midi_rx.clock_generator().mtc_framerate());
            auto pos = mixer.clock().position_snapshot();
            ImGui::SameLine(0, 15);
            ImGui::TextColored(mixer.clock().is_scrubbing() ? ImVec4(1.0f, 0.75f, 0.20f, 1.0f) : ImVec4(0.20f, 0.85f, 0.45f, 1.0f),
                               "[SMPTE %02d:%02d:%02d:%02d | BAR %u.%u.%u]",
                               tc.hours, tc.minutes, tc.seconds, tc.frames,
                               pos.bar_index + 1, pos.beat_within_bar + 1,
                               static_cast<uint32_t>(pos.beat_progress * 4.0) + 1);
            if (mixer.clock().is_scrubbing()) {
                ImGui::SameLine(0, 6);
                double vel = mixer.clock().scrub_velocity();
                if (std::abs(vel) < 0.05) {
                    ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.20f, 1.0f), "[SCRUB JOG]");
                } else if (vel > 0.0) {
                    ImGui::TextColored(ImVec4(0.20f, 0.90f, 0.40f, 1.0f), "[SCRUB FWD %+.1fx]", vel);
                } else {
                    ImGui::TextColored(ImVec4(0.95f, 0.35f, 0.25f, 1.0f), "[SCRUB REV %+.1fx]", vel);
                }
            }


            // Keyboard Shortcuts: Ctrl+S (Save), Ctrl+O (Load)
            if (ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S)) {
                if (serialization::SessionSerializer::save_session_file(session_file_path, mixer, mixer.clock(), "Aethel Project")) {
                    session_status_msg = "Saved: " + std::string(session_file_path);
                } else {
                    session_status_msg = "Error saving session!";
                }
                session_status_time = std::chrono::steady_clock::now();
            }
            if (ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_O)) {
                if (serialization::SessionSerializer::load_session_file(session_file_path, mixer, mixer.clock())) {
                    sync_ui_from_mixer();
                    session_status_msg = "Loaded: " + std::string(session_file_path);
                } else {
                    session_status_msg = "Error loading session!";
                }
                session_status_time = std::chrono::steady_clock::now();
            }

            ImGui::SameLine(0, 10);
            if (ImGui::Button("[ SESSION ]", ImVec2(85, 32))) {
                ImGui::OpenPopup("SessionMenuPopup");
            }
            if (ImGui::BeginPopup("SessionMenuPopup")) {
                if (ImGui::MenuItem("Save Session...", "Ctrl+S")) {
                    open_save_session_modal = true;
                }
                if (ImGui::MenuItem("Load Session...", "Ctrl+O")) {
                    open_load_session_modal = true;
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Export Master WAV (Offline Bounce)...")) {
                    open_bounce_modal = true;
                }
                ImGui::EndPopup();
            }

            ImGui::SameLine(0, 5);
            if (ImGui::Button("[ BOUNCE ]", ImVec2(80, 32))) {
                open_bounce_modal = true;
            }

            if (!session_status_msg.empty()) {
                float time_alive = std::chrono::duration<float>(now - session_status_time).count();
                if (time_alive < 5.0f) {
                    ImGui::SameLine(0, 8);
                    ImGui::TextColored(ImVec4(0.20f, 0.85f, 0.35f, 1.0f), "[%s]", session_status_msg.c_str());
                }
            }

            ImGui::SameLine(0, 15);
            if (pw_online) {
                auto sources = pw.get_available_sources();
                auto sinks = pw.get_available_sinks();
                if (ImGui::Button("[ HARDWARE I/O ]", ImVec2(125, 32))) {
                    pw.refresh_discovery();
                    open_hardware_io_modal = true;
                }
                ImGui::SameLine(0, 6);
                ImGui::TextColored(ImVec4(0.12f, 0.55f, 0.95f, 1.0f), "[HW: %zu IN / %zu OUT]", sources.size(), sinks.size());
            } else {
                if (ImGui::Button("[ HW: OFFLINE ]", ImVec2(115, 32))) {
                    open_hardware_io_modal = true;
                }
            }

            ImGui::SameLine(0, 12);
            const auto& aoip_stat = aoip_rx.stats();
            uint64_t pkts = aoip_stat.packets_received.load(std::memory_order_relaxed);
            uint64_t drops = aoip_stat.packets_dropped.load(std::memory_order_relaxed);
            ImGui::TextColored(ImVec4(0.20f, 0.70f, 0.85f, 1.0f), "[AOIP :4848 | PKTS: %lu | DROPS: %lu]",
                               static_cast<unsigned long>(pkts), static_cast<unsigned long>(drops));

            ImGui::SameLine(0, 8);
            auto ptp_src = static_cast<network::PtpTimestampSource>(aoip_stat.timestamp_source.load(std::memory_order_relaxed));
            int64_t avg_j = aoip_stat.avg_jitter_ns.load(std::memory_order_relaxed);
            if (aoip_stat.hardware_locked.load(std::memory_order_relaxed)) {
                ImGui::TextColored(ImVec4(0.25f, 0.85f, 0.35f, 1.0f), "[PTPv2: HW LOCKED (%ld ns)]", static_cast<long>(avg_j));
            } else if (ptp_src == network::PtpTimestampSource::KernelDriverStack) {
                ImGui::TextColored(ImVec4(0.20f, 0.75f, 0.90f, 1.0f), "[PTPv2: KERNEL (%ld ns)]", static_cast<long>(avg_j));
            } else {
                ImGui::TextColored(ImVec4(0.65f, 0.65f, 0.65f, 1.0f), "[PTPv2: USERSPACE]");
            }

            if (tap0) {
                auto tap_st = tap0->record_state();
                if (tap_st == sampling::RecordState::Armed) {
                    ImGui::SameLine(0, 8);
                    ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.15f, 1.0f), "[TAP: ARMED]");
                } else if (tap_st == sampling::RecordState::Recording) {
                    ImGui::SameLine(0, 8);
                    ImGui::TextColored(ImVec4(0.90f, 0.25f, 0.25f, 1.0f), "[TAP: REC %.0f%%]", tap0->progress() * 100.0f);
                } else if (tap_st == sampling::RecordState::Complete) {
                    ImGui::SameLine(0, 8);
                    ImGui::TextColored(ImVec4(0.25f, 0.85f, 0.35f, 1.0f), "[TAP: READY]");
                }
            }

            ImGui::SameLine(0, 20);
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
                            mixer.launch_scene(0, sequencer::LaunchQuantize::Bar);
                            std::snprintf(status_toast, sizeof(status_toast), "SCENE 1 (MAIN GROOVES) QUEUED FOR NEXT DOWNBEAT");
                        }
                        ImGui::SameLine();
                        if (ImGui::SmallButton("▶ S2")) {
                            mixer.launch_scene(1, sequencer::LaunchQuantize::Bar);
                            std::snprintf(status_toast, sizeof(status_toast), "SCENE 2 (WSOLA BREAK/MOD) QUEUED FOR NEXT DOWNBEAT");
                        }
                        ImGui::SameLine();
                        if (ImGui::SmallButton("▶ S3")) {
                            mixer.launch_scene(2, sequencer::LaunchQuantize::Bar);
                            std::snprintf(status_toast, sizeof(status_toast), "SCENE 3 (GLITCH/SUB/REV) QUEUED FOR NEXT DOWNBEAT");
                        }
                        ImGui::SameLine();
                        if (ImGui::SmallButton("■ ALL")) {
                            mixer.stop_all_clips(sequencer::LaunchQuantize::Bar);
                            std::snprintf(status_toast, sizeof(status_toast), "ALL TRACK CLIPS STOPPING AT NEXT BAR");
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

                                auto* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                                bool launcher_active = trk ? trk->is_clip_launcher_active() : false;

                                // Back to Arranger button
                                if (!launcher_active) {
                                    ImGui::TextColored(ImVec4(0.40f, 0.45f, 0.52f, 1.0f), "[ARR]");
                                } else {
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.48f, 0.05f, 0.85f));
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                    if (ImGui::SmallButton("⮌ ARR")) {
                                        if (trk) trk->clip_launcher().stop_immediate();
                                        std::snprintf(status_toast, sizeof(status_toast), "TRACK %d RETURNED TO ARRANGER", t + 1);
                                    }
                                    ImGui::PopStyleColor(2);
                                }

                                // 3 Clip Slots + Stop button
                                for (int s = 1; s <= 3; ++s) {
                                    ImGui::SameLine();
                                    const auto& slot_info = trk ? trk->clip_launcher().slot(s - 1) : sequencer::ClipSlot{};
                                    auto slot_state = slot_info.state.load(std::memory_order_relaxed);

                                    char slot_lbl[24];
                                    if (slot_state == sequencer::SlotPlayState::Playing) {
                                        std::snprintf(slot_lbl, sizeof(slot_lbl), "▶ %d##s", s);
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.38f, 0.85f, 1.0f));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                    } else if (slot_state == sequencer::SlotPlayState::QueuedPlay) {
                                        std::snprintf(slot_lbl, sizeof(slot_lbl), "⧗ %d##s", s);
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.48f, 0.05f, 1.0f));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                    } else if (slot_state == sequencer::SlotPlayState::QueuedStop) {
                                        std::snprintf(slot_lbl, sizeof(slot_lbl), "■ %d##s", s);
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.20f, 0.20f, 1.0f));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                    } else {
                                        std::snprintf(slot_lbl, sizeof(slot_lbl), "%d##s", s);
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.93f, 0.95f, 0.97f, 1.0f));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.25f, 0.30f, 0.40f, 1.0f));
                                    }

                                    if (ImGui::Button(slot_lbl, ImVec2(28, 20))) {
                                        mixer.launch_track_clip(t, s - 1, sequencer::LaunchQuantize::Bar, false);
                                        std::snprintf(status_toast, sizeof(status_toast), "TRACK %d CLIP %d (%s) QUEUED FOR NEXT DOWNBEAT",
                                                      t + 1, s, slot_info.name.c_str());
                                    }
                                    if (ImGui::IsItemHovered()) {
                                        ImGui::SetTooltip("Track %d Slot %d: %s\nMode: %s\nPitch: %+.1f st",
                                                          t + 1, s, slot_info.name.c_str(),
                                                          (slot_info.playback_mode == sampling::PlaybackMode::BeatSyncTimeStretch) ? "BeatSync WSOLA" :
                                                          ((slot_info.playback_mode == sampling::PlaybackMode::PitchShiftWsola) ? "PitchShift WSOLA" :
                                                          ((slot_info.playback_mode == sampling::PlaybackMode::ReverseFree) ? "Reverse" : "Repitch")),
                                                          slot_info.pitch_semitones);
                                    }
                                    ImGui::PopStyleColor(2);
                                }

                                ImGui::SameLine();
                                if (ImGui::SmallButton("■##stp")) {
                                    mixer.stop_track_clip(t, sequencer::LaunchQuantize::Bar);
                                    std::snprintf(status_toast, sizeof(status_toast), "TRACK %d STOPPING AT NEXT BAR", t + 1);
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
                    // RIGHT COLUMN: MULTITRACK ARRANGER TIMELINE & STEP PATTERN MATRIX
                    // ========================================================
                    ImGui::BeginGroup();
                    const float top_pane_h = std::max(avail_sz.y * 0.44f, 105.0f);
                    ImGui::BeginChild("ArrangerTimelinePane", ImVec2(0, top_pane_h), true, ImGuiWindowFlags_NoScrollbar);
                    {
                        ImDrawList* draw_list = ImGui::GetWindowDrawList();
                        ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
                        ImVec2 canvas_size = ImGui::GetContentRegionAvail();
                        canvas_size.y = std::max(canvas_size.y - 2.0f, 85.0f);

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
                            auto* trk_ptr = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                            if (trk_ptr && trk_ptr->is_clip_launcher_active()) {
                                float blk_x1 = canvas_pos.x + 4.0f;
                                float blk_x2 = canvas_pos.x + canvas_size.x - 4.0f;
                                draw_list->AddRectFilled(ImVec2(blk_x1, ly + 4.0f),
                                                         ImVec2(blk_x2, ly + lane_h - 4.0f),
                                                         ImColor(254, 243, 199, 180), 2.0f);
                                draw_list->AddRect(ImVec2(blk_x1, ly + 4.0f),
                                                   ImVec2(blk_x2, ly + lane_h - 4.0f),
                                                   ImColor(217, 123, 13, 220), 1.5f);
                                int cur_s = trk_ptr->clip_launcher().current_slot_idx();
                                int q_s = trk_ptr->clip_launcher().queued_slot_idx();
                                char act_txt[96];
                                if (cur_s >= 0) {
                                    std::snprintf(act_txt, sizeof(act_txt), "[ CLIP LAUNCHER: SLOT %d (%s) PLAYING%s ]",
                                                  cur_s + 1, trk_ptr->clip_launcher().slot(cur_s).name.c_str(),
                                                  (q_s >= 0) ? " // NEXT DOWNBEAT QUEUED" : "");
                                } else {
                                    std::snprintf(act_txt, sizeof(act_txt), "[ CLIP LAUNCHER: QUEUED FOR NEXT DOWNBEAT ]");
                                }
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
                        if (ImGui::IsItemActivated()) {
                            ImVec2 m = ImGui::GetIO().MousePos;
                            float ratio = std::clamp((m.x - canvas_pos.x) / canvas_size.x, 0.0f, 1.0f);
                            playhead_seconds = ratio * loop_length_seconds;
                            uint64_t target_sample = static_cast<uint64_t>(playhead_seconds * kSampleRate);
                            mixer.start_scrub(target_sample);
                        } else if (ImGui::IsItemActive()) {
                            ImVec2 m = ImGui::GetIO().MousePos;
                            float ratio = std::clamp((m.x - canvas_pos.x) / canvas_size.x, 0.0f, 1.0f);
                            float prev_sec = playhead_seconds;
                            playhead_seconds = ratio * loop_length_seconds;
                            float vel = (dt > 1e-4f) ? ((playhead_seconds - prev_sec) / dt) : 1.0f;
                            uint64_t target_sample = static_cast<uint64_t>(playhead_seconds * kSampleRate);
                            mixer.update_scrub(target_sample, static_cast<double>(vel));
                        } else if (ImGui::IsItemDeactivated()) {
                            ImVec2 m = ImGui::GetIO().MousePos;
                            float ratio = std::clamp((m.x - canvas_pos.x) / canvas_size.x, 0.0f, 1.0f);
                            playhead_seconds = ratio * loop_length_seconds;
                            uint64_t target_sample = static_cast<uint64_t>(playhead_seconds * kSampleRate);
                            mixer.end_scrub(target_sample);
                        }

                    }
                    ImGui::EndChild();

                    // ========================================================
                    // LOWER SECTION: STEP PATTERN MATRIX & MPC AUDITION PADS
                    // ========================================================
                    ImGui::Spacing();
                    ImGui::BeginChild("StepPatternMatrixPane", ImVec2(0, 0), true);
                    {
                        auto seq = track_seq[selected_track];
                        if (seq) {
                            auto& pat = seq->pattern(pattern_editor_pat_idx);

                            // Row 1: Header & Pattern Selector
                            ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "STEP PATTERN MATRIX // %s // PAT %d (%s):",
                                               track_names[selected_track], pattern_editor_pat_idx + 1, pat.name.c_str());
                            ImGui::SameLine();
                            for (int p = 0; p < 4; ++p) {
                                char p_lbl[32];
                                std::snprintf(p_lbl, sizeof(p_lbl), "PAT %d (S%d)##p", p + 1, p + 1);
                                bool is_active_p = (pattern_editor_pat_idx == p);
                                if (is_active_p) {
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.38f, 0.85f, 1.0f));
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                }
                                if (ImGui::SmallButton(p_lbl)) {
                                    pattern_editor_pat_idx = p;
                                }
                                if (is_active_p) ImGui::PopStyleColor(2);
                                ImGui::SameLine();
                            }

                            // Voice Mode: Monophonic (all-choke classic) vs Polyphonic 16-Voice with Choke Groups
                            bool is_poly = (seq->voice_mode() == sequencer::VoiceMode::Polyphonic);
                            if (is_poly) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.58f, 0.32f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            } else {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.45f, 0.10f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            }
                            if (ImGui::SmallButton(is_poly ? "VOICE: POLY (16)##vm" : "VOICE: MONO (CHOKE)##vm")) {
                                seq->set_voice_mode(is_poly ? sequencer::VoiceMode::Monophonic : sequencer::VoiceMode::Polyphonic);
                            }
                            ImGui::PopStyleColor(2);

                            ImGui::SameLine(0, 6);
                            ImGui::TextColored(ImVec4(0.55f, 0.60f, 0.70f, 1.0f), "[%u/16 VOICES]", seq->active_voice_count());

                            ImGui::SameLine(0, 12);
                            // Pattern Switch Mode (Quantized)
                            auto cur_mode = seq->switch_mode();
                            const char* mode_str = (cur_mode == sequencer::PatternSwitchMode::BarQuantized) ? "BAR-SYNC" :
                                                   ((cur_mode == sequencer::PatternSwitchMode::BeatQuantized) ? "BEAT-SYNC" : "IMMEDIATE");
                            ImGui::TextColored(ImVec4(0.85f, 0.48f, 0.05f, 1.0f), "[SYNC: %s]", mode_str);

                            ImGui::SameLine(0, 15);
                            if (ImGui::SmallButton("CLEAR##pat")) {
                                pat.clear();
                            }
                            ImGui::SameLine();
                            if (ImGui::SmallButton("FILL 1-8##pat")) {
                                pat.fill_linear_slices(8, 0.9f);
                            }
                            ImGui::SameLine();
                            if (ImGui::SmallButton("EUCLID 4##pat")) {
                                pat.clear();
                                for (int st = 0; st < 16; st += 4) pat.set_step(st, 0, 1.0f);
                            }
                            ImGui::SameLine();
                            if (ImGui::SmallButton("EUCLID 8##pat")) {
                                pat.clear();
                                for (int st = 0; st < 16; st += 2) pat.set_step(st, (st % 4 == 0) ? 0 : 2, 0.85f);
                            }

                            // Auto-Chop & Slice-to-MIDI Groove Action
                            ImGui::SameLine(0, 12);
                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.35f, 0.08f, 0.90f));
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            if (ImGui::SmallButton("⚡ AUTO-CHOP & GROOVE##pat")) {
                                seq->auto_chop_and_groove(0.5f, pattern_editor_pat_idx);
                            }
                            ImGui::PopStyleColor(2);

                            // Pattern Swing Slider
                            ImGui::SameLine(0, 12);
                            ImGui::SetNextItemWidth(75);
                            int swing_pct = static_cast<int>(std::round(pat.swing * 100.0f));
                            if (ImGui::SliderInt("Swing##pat", &swing_pct, 0, 100, "%d%%")) {
                                pat.swing = static_cast<float>(swing_pct) / 100.0f;
                            }

                            // Whole-Pattern Quick Quantize
                            ImGui::SameLine(0, 8);
                            if (ImGui::SmallButton("SNAP 100%##all")) {
                                pat.quantize_all(1.0f);
                            }
                            ImGui::SameLine();
                            if (ImGui::SmallButton("RAW 0%##all")) {
                                pat.quantize_all(0.0f);
                            }

                            // Parameter Automation Lane Selector (Elektron / Bitwig Style)
                            static int s_seq_auto_lane_target = 0; // 0=Vel, 1=Cutoff, 2=Decay, 3=Drive, 4=Rev A, 5=Dly B, 6=Pitch, 7=Pan, 8=Micro, 9=Prob
                            ImGui::Spacing();
                            ImGui::TextColored(ImVec4(0.35f, 0.40f, 0.48f, 1.0f), "AUTOMATION LANE:");
                            ImGui::SameLine(0, 8);
                            const char* auto_lane_names[] = { "Vel", "Cutoff", "Decay", "Drive", "Rev A", "Dly B", "Pitch", "Pan", "Micro", "Prob" };
                            for (int al = 0; al < 10; ++al) {
                                if (al > 0) ImGui::SameLine(0, 4);
                                bool is_al_sel = (s_seq_auto_lane_target == al);
                                if (is_al_sel) {
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.38f, 0.85f, 1.0f));
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                } else {
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.92f, 0.94f, 0.96f, 1.0f));
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.35f, 0.40f, 0.48f, 1.0f));
                                }
                                if (ImGui::SmallButton(auto_lane_names[al])) {
                                    s_seq_auto_lane_target = al;
                                }
                                ImGui::PopStyleColor(2);
                            }

                            // Row 2: 16-Step Button Grid with Live Step Playhead LED
                            uint32_t active_step = seq->current_step_index();
                            bool seq_running = is_playing && (trk0->is_sequencer_enabled() || trk1->is_sequencer_enabled() ||
                                                              trk2->is_sequencer_enabled() || trk3->is_sequencer_enabled());

                            ImGui::Spacing();
                            for (int st = 0; st < 16; ++st) {
                                if (st > 0 && (st % 4 == 0)) {
                                    ImGui::SameLine(0, 12); // Beat separator gap
                                } else if (st > 0) {
                                    ImGui::SameLine(0, 4);
                                }

                                ImGui::BeginGroup();
                                // Step Playhead LED
                                bool is_cur_step = (seq_running && active_step == static_cast<uint32_t>(st));
                                ImVec4 led_col = is_cur_step ? ImVec4(0.95f, 0.70f, 0.10f, 1.0f) : ImVec4(0.80f, 0.83f, 0.88f, 0.5f);
                                ImGui::PushStyleColor(ImGuiCol_Text, led_col);
                                ImGui::Text(is_cur_step ? " ● " : " · ");
                                ImGui::PopStyleColor();

                                // Step Button
                                bool st_active = pat.is_step_active(st);
                                char st_lbl[24];
                                if (st_active) {
                                    std::snprintf(st_lbl, sizeof(st_lbl), "%d##st%d", st + 1, st);
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.38f, 0.85f, 0.95f));
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                } else {
                                    std::snprintf(st_lbl, sizeof(st_lbl), "%d##st%d", st + 1, st);
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.93f, 0.95f, 0.97f, 1.0f));
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.45f, 0.50f, 0.58f, 1.0f));
                                }

                                if (ImGui::Button(st_lbl, ImVec2(34, 26))) {
                                    pat.toggle_step(st, 0, 0.9f);
                                    pattern_editor_step_idx = st;
                                }
                                ImGui::PopStyleColor(2);

                                // 16-Step Interactive Parameter Automation Bar (Elektron / Bitwig style)
                                ImDrawList* dlist = ImGui::GetWindowDrawList();
                                ImVec2 bar_sz(34.0f, 26.0f);
                                ImVec2 b_p0 = ImGui::GetCursorScreenPos();
                                ImVec2 b_p1 = ImVec2(b_p0.x + bar_sz.x, b_p0.y + bar_sz.y);

                                char bar_btn_id[32];
                                std::snprintf(bar_btn_id, sizeof(bar_btn_id), "##al_bar_%d", st);
                                ImGui::InvisibleButton(bar_btn_id, bar_sz);

                                bool b_hovered = ImGui::IsItemHovered();
                                bool b_active = ImGui::IsItemActive();

                                auto& sref = pat.steps[st];
                                if (b_active) {
                                    pattern_editor_step_idx = st;
                                    float my = ImGui::GetIO().MousePos.y;
                                    float drag_norm = std::clamp(1.0f - (my - b_p0.y) / bar_sz.y, 0.0f, 1.0f);

                                    if (s_seq_auto_lane_target == 0) { // Velocity
                                        sref.velocity = drag_norm;
                                        if (!sref.active && drag_norm > 0.05f) sref.active = true;
                                    } else if (s_seq_auto_lane_target == 1) { // Cutoff
                                        sref.filter_cutoff = (drag_norm >= 0.98f) ? 20000.0f : 40.0f * std::pow(500.0f, drag_norm);
                                    } else if (s_seq_auto_lane_target == 2) { // Decay
                                        sref.decay_ms = (drag_norm >= 0.98f) ? 0.0f : drag_norm * 1500.0f;
                                    } else if (s_seq_auto_lane_target == 3) { // Drive
                                        sref.drive = drag_norm;
                                    } else if (s_seq_auto_lane_target == 4) { // Rev A
                                        sref.send_a = drag_norm;
                                    } else if (s_seq_auto_lane_target == 5) { // Dly B
                                        sref.send_b = drag_norm;
                                    } else if (s_seq_auto_lane_target == 6) { // Pitch
                                        float bi = (drag_norm - 0.5f) * 2.0f;
                                        sref.pitch_ratio = std::pow(2.0f, bi);
                                    } else if (s_seq_auto_lane_target == 7) { // Pan
                                        sref.pan = (drag_norm - 0.5f) * 2.0f;
                                    } else if (s_seq_auto_lane_target == 8) { // Micro
                                        sref.micro_timing = (drag_norm - 0.5f);
                                    } else if (s_seq_auto_lane_target == 9) { // Prob
                                        sref.probability = static_cast<uint8_t>(std::round(drag_norm * 100.0f));
                                    }
                                }

                                // Background
                                dlist->AddRectFilled(b_p0, b_p1, IM_COL32(236, 239, 244, 255), 2.0f);

                                // Compute display values & color
                                float norm_fill = 0.0f;
                                float bi_fill = 0.0f;
                                bool is_bi = false;
                                ImU32 fill_col = IM_COL32(31, 97, 217, 220); // Default Blue
                                char val_txt[16] = "";

                                const auto& cur_s = pat.steps[st];
                                if (s_seq_auto_lane_target == 0) { // Velocity
                                    norm_fill = cur_s.active ? cur_s.velocity : 0.0f;
                                    fill_col = IM_COL32(31, 97, 217, 220);
                                    std::snprintf(val_txt, sizeof(val_txt), "%d", static_cast<int>(std::round(cur_s.velocity * 100.0f)));
                                } else if (s_seq_auto_lane_target == 1) { // Cutoff
                                    norm_fill = std::clamp(std::log(cur_s.filter_cutoff / 40.0f) / std::log(20000.0f / 40.0f), 0.0f, 1.0f);
                                    fill_col = IM_COL32(217, 123, 13, 220);
                                    if (cur_s.filter_cutoff >= 19900.0f) std::snprintf(val_txt, sizeof(val_txt), "BYP");
                                    else if (cur_s.filter_cutoff >= 1000.0f) std::snprintf(val_txt, sizeof(val_txt), "%.1fk", cur_s.filter_cutoff * 0.001f);
                                    else std::snprintf(val_txt, sizeof(val_txt), "%.0f", cur_s.filter_cutoff);
                                } else if (s_seq_auto_lane_target == 2) { // Decay
                                    norm_fill = (cur_s.decay_ms <= 0.01f) ? 1.0f : std::clamp(cur_s.decay_ms / 1500.0f, 0.0f, 1.0f);
                                    fill_col = IM_COL32(16, 163, 127, 220);
                                    if (cur_s.decay_ms <= 0.01f) std::snprintf(val_txt, sizeof(val_txt), "FULL");
                                    else std::snprintf(val_txt, sizeof(val_txt), "%.0fm", cur_s.decay_ms);
                                } else if (s_seq_auto_lane_target == 3) { // Drive
                                    norm_fill = cur_s.drive;
                                    fill_col = IM_COL32(220, 53, 69, 220);
                                    if (cur_s.drive < 0.01f) std::snprintf(val_txt, sizeof(val_txt), "--");
                                    else std::snprintf(val_txt, sizeof(val_txt), "%d%%", static_cast<int>(std::round(cur_s.drive * 100.0f)));
                                } else if (s_seq_auto_lane_target == 4) { // Rev A
                                    norm_fill = cur_s.send_a;
                                    fill_col = IM_COL32(138, 75, 232, 220);
                                    if (cur_s.send_a < 0.01f) std::snprintf(val_txt, sizeof(val_txt), "--");
                                    else std::snprintf(val_txt, sizeof(val_txt), "%d%%", static_cast<int>(std::round(cur_s.send_a * 100.0f)));
                                } else if (s_seq_auto_lane_target == 5) { // Dly B
                                    norm_fill = cur_s.send_b;
                                    fill_col = IM_COL32(14, 165, 233, 220);
                                    if (cur_s.send_b < 0.01f) std::snprintf(val_txt, sizeof(val_txt), "--");
                                    else std::snprintf(val_txt, sizeof(val_txt), "%d%%", static_cast<int>(std::round(cur_s.send_b * 100.0f)));
                                } else if (s_seq_auto_lane_target == 6) { // Pitch
                                    is_bi = true;
                                    bi_fill = std::clamp(std::log2(cur_s.pitch_ratio), -1.0f, 1.0f);
                                    fill_col = IM_COL32(99, 102, 241, 220);
                                    std::snprintf(val_txt, sizeof(val_txt), "%.2fx", cur_s.pitch_ratio);
                                } else if (s_seq_auto_lane_target == 7) { // Pan
                                    is_bi = true;
                                    bi_fill = cur_s.pan;
                                    fill_col = IM_COL32(249, 115, 22, 220);
                                    if (std::abs(cur_s.pan) < 0.05f) std::snprintf(val_txt, sizeof(val_txt), "C");
                                    else if (cur_s.pan < 0.0f) std::snprintf(val_txt, sizeof(val_txt), "L%.0f", -cur_s.pan * 100.0f);
                                    else std::snprintf(val_txt, sizeof(val_txt), "R%.0f", cur_s.pan * 100.0f);
                                } else if (s_seq_auto_lane_target == 8) { // Micro
                                    is_bi = true;
                                    bi_fill = cur_s.micro_timing * 2.0f;
                                    fill_col = IM_COL32(234, 88, 12, 220);
                                    std::snprintf(val_txt, sizeof(val_txt), "%+.0f%%", cur_s.micro_timing * 100.0f);
                                } else if (s_seq_auto_lane_target == 9) { // Prob
                                    norm_fill = static_cast<float>(cur_s.probability) / 100.0f;
                                    fill_col = IM_COL32(100, 116, 139, 220);
                                    std::snprintf(val_txt, sizeof(val_txt), "%d%%", cur_s.probability);
                                }

                                // Inactive step dimming
                                if (!cur_s.active) {
                                    fill_col = (fill_col & 0x00FFFFFF) | 0x40000000; // 25% alpha
                                }

                                // Draw bar geometry
                                if (is_bi) {
                                    float mid_y = b_p0.y + bar_sz.y * 0.5f;
                                    dlist->AddLine(ImVec2(b_p0.x, mid_y), ImVec2(b_p1.x, mid_y), IM_COL32(180, 185, 195, 255), 1.0f);
                                    float h = -bi_fill * (bar_sz.y * 0.45f);
                                    if (bi_fill > 0.01f) {
                                        dlist->AddRectFilled(ImVec2(b_p0.x + 2, mid_y + h), ImVec2(b_p1.x - 2, mid_y), fill_col, 1.0f);
                                    } else if (bi_fill < -0.01f) {
                                        dlist->AddRectFilled(ImVec2(b_p0.x + 2, mid_y), ImVec2(b_p1.x - 2, mid_y + h), fill_col, 1.0f);
                                    }
                                } else {
                                    float fill_h = norm_fill * (bar_sz.y - 2.0f);
                                    if (fill_h > 1.0f) {
                                        dlist->AddRectFilled(ImVec2(b_p0.x + 2, b_p1.y - 1.0f - fill_h), ImVec2(b_p1.x - 2, b_p1.y - 1.0f), fill_col, 1.0f);
                                    }
                                }

                                // Border highlight
                                ImU32 brd_col = b_hovered ? IM_COL32(31, 97, 217, 255) :
                                               (pattern_editor_step_idx == st ? IM_COL32(217, 123, 13, 255) : IM_COL32(200, 205, 215, 255));
                                dlist->AddRect(b_p0, b_p1, brd_col, 2.0f);

                                // Centered value text
                                ImVec2 txt_sz = ImGui::CalcTextSize(val_txt);
                                ImVec2 txt_pos(b_p0.x + (bar_sz.x - txt_sz.x) * 0.5f, b_p0.y + (bar_sz.y - txt_sz.y) * 0.5f);
                                ImU32 txt_col = cur_s.active ? IM_COL32(26, 30, 40, 255) : IM_COL32(140, 145, 155, 255);
                                dlist->AddText(txt_pos, txt_col, val_txt);

                                // Slice & micro info under step
                                if (cur_s.active) {
                                    ImGui::TextColored(ImVec4(0.20f, 0.45f, 0.85f, 0.9f), "S%u", cur_s.slice_id);
                                } else {
                                    ImGui::TextDisabled(" -- ");
                                }

                                ImGui::EndGroup();
                            }

                            // Row 3: Step Parameters Inspector & MPC Live Jam Trigger Pads
                            ImGui::Spacing();
                            ImGui::Separator();

                            // Left sub-pane: Comprehensive Note Inspector for pattern_editor_step_idx
                            ImGui::BeginGroup();
                            {
                                ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "NOTE INSPECTOR [Step %d]:", pattern_editor_step_idx + 1);
                                ImGui::SameLine();
                                auto& step_ref = pat.steps[pattern_editor_step_idx];
                                bool s_act = step_ref.active;
                                if (ImGui::Checkbox("Active##StepProp", &s_act)) {
                                    step_ref.active = s_act;
                                }
                                ImGui::SameLine(0, 10);
                                int s_slice = static_cast<int>(step_ref.slice_id);
                                ImGui::SetNextItemWidth(60);
                                if (ImGui::SliderInt("Slice##StepProp", &s_slice, 0, 15)) {
                                    step_ref.slice_id = static_cast<uint32_t>(s_slice);
                                }
                                ImGui::SameLine(0, 10);
                                ImGui::SetNextItemWidth(70);
                                ImGui::SliderFloat("Vel##StepProp", &step_ref.velocity, 0.0f, 1.0f, "%.2f");
                                ImGui::SameLine(0, 10);
                                ImGui::SetNextItemWidth(75);
                                char pan_buf[16];
                                if (std::abs(step_ref.pan) < 0.01f) {
                                    std::snprintf(pan_buf, sizeof(pan_buf), "C");
                                } else if (step_ref.pan < 0.0f) {
                                    std::snprintf(pan_buf, sizeof(pan_buf), "%.0f%% L", -step_ref.pan * 100.0f);
                                } else {
                                    std::snprintf(pan_buf, sizeof(pan_buf), "%.0f%% R", step_ref.pan * 100.0f);
                                }
                                ImGui::SliderFloat("Pan##StepProp", &step_ref.pan, -1.0f, 1.0f, pan_buf);
                                ImGui::SameLine(0, 10);
                                ImGui::SetNextItemWidth(70);
                                ImGui::SliderFloat("Pitch##StepProp", &step_ref.pitch_ratio, 0.25f, 2.0f, "%.2fx");
                                ImGui::SameLine(0, 10);
                                int prob = static_cast<int>(step_ref.probability);
                                ImGui::SetNextItemWidth(60);
                                if (ImGui::SliderInt("Prob%##StepProp", &prob, 0, 100)) {
                                    step_ref.probability = static_cast<uint8_t>(prob);
                                }
                                ImGui::SameLine(0, 10);
                                ImGui::Checkbox("Rev##StepProp", &step_ref.reverse);

                                // Second line in Inspector: Micro-Timing Offset & Per-Note Quantization
                                ImGui::Spacing();
                                ImGui::TextColored(ImVec4(0.40f, 0.45f, 0.55f, 1.0f), "Micro-Timing:");
                                ImGui::SameLine(0, 6);
                                ImGui::SetNextItemWidth(125);
                                int micro_int = static_cast<int>(std::round(step_ref.micro_timing * 100.0f));
                                char micro_str[32];
                                if (micro_int == 0) {
                                    std::snprintf(micro_str, sizeof(micro_str), "Grid (0%%)");
                                } else if (micro_int < 0) {
                                    std::snprintf(micro_str, sizeof(micro_str), "Rush (%d%%)", micro_int);
                                } else {
                                    std::snprintf(micro_str, sizeof(micro_str), "Late (+%d%%)", micro_int);
                                }
                                if (ImGui::SliderInt("##MicroSlider", &micro_int, -50, 50, micro_str)) {
                                    step_ref.micro_timing = static_cast<float>(micro_int) / 100.0f;
                                }

                                ImGui::SameLine(0, 15);
                                ImGui::TextColored(ImVec4(0.40f, 0.45f, 0.55f, 1.0f), "Quantize:");
                                ImGui::SameLine(0, 6);
                                ImGui::SetNextItemWidth(80);
                                int q_int = static_cast<int>(std::round(step_ref.quantize_pct * 100.0f));
                                if (ImGui::SliderInt("##QuantizePct", &q_int, 0, 100, "%d%%")) {
                                    step_ref.quantize_pct = static_cast<float>(q_int) / 100.0f;
                                }
                                ImGui::SameLine(0, 6);
                                if (ImGui::SmallButton("RAW 0%##nt")) {
                                    step_ref.quantize_pct = 0.0f;
                                }
                                ImGui::SameLine();
                                if (ImGui::SmallButton("50%##nt")) {
                                    step_ref.quantize_pct = 0.5f;
                                }
                                ImGui::SameLine();
                                if (ImGui::SmallButton("SNAP 100%##nt")) {
                                    step_ref.quantize_pct = 1.0f;
                                }

                                // Third line in Inspector: Per-Step Parameter Locks (Filter, Envelope, Saturation, Choke Group)
                                ImGui::Spacing();
                                ImGui::TextColored(ImVec4(0.95f, 0.65f, 0.15f, 1.0f), "P-LOCKS:");
                                ImGui::SameLine(0, 8);

                                // Choke Group selector: 0=Poly/Off, 1..4=Group 1..4
                                ImGui::SetNextItemWidth(88);
                                const char* choke_labels[] = { "Choke: Off", "Choke: G1", "Choke: G2", "Choke: G3", "Choke: G4" };
                                int cur_cg = static_cast<int>(step_ref.choke_group);
                                if (cur_cg > 4) cur_cg = 4;
                                if (ImGui::Combo("##ChokeGrp", &cur_cg, choke_labels, 5)) {
                                    step_ref.choke_group = static_cast<uint8_t>(cur_cg);
                                }

                                ImGui::SameLine(0, 8);
                                // Filter Cutoff Slider (20 Hz - 20000 Hz, logarithmic feel)
                                ImGui::SetNextItemWidth(95);
                                char cut_str[24];
                                if (step_ref.filter_cutoff >= 19900.0f) {
                                    std::snprintf(cut_str, sizeof(cut_str), "Cut: BYPASS");
                                } else {
                                    std::snprintf(cut_str, sizeof(cut_str), "Cut: %.0fHz", step_ref.filter_cutoff);
                                }
                                if (ImGui::SliderFloat("##CutoffLock", &step_ref.filter_cutoff, 40.0f, 20000.0f, cut_str, ImGuiSliderFlags_Logarithmic)) {
                                    // Cutoff updated
                                }

                                ImGui::SameLine(0, 6);
                                // Filter Resonance Q (0.1 - 8.0)
                                ImGui::SetNextItemWidth(65);
                                ImGui::SliderFloat("##ResLock", &step_ref.filter_res, 0.1f, 8.0f, "Q: %.2f");

                                ImGui::SameLine(0, 6);
                                // Filter Type (LP, HP, BP, Notch)
                                ImGui::SetNextItemWidth(65);
                                const char* ftype_names[] = { "LP", "HP", "BP", "Notch" };
                                int ftype_idx = static_cast<int>(step_ref.filter_type);
                                if (ftype_idx < 0 || ftype_idx > 3) ftype_idx = 0;
                                if (ImGui::Combo("##FilterTypeLock", &ftype_idx, ftype_names, 4)) {
                                    step_ref.filter_type = static_cast<dsp::FilterType>(ftype_idx);
                                }

                                ImGui::SameLine(0, 8);
                                // Decay Envelope Slider (0 ms = Full, up to 1500 ms)
                                ImGui::SetNextItemWidth(90);
                                char decay_str[24];
                                if (step_ref.decay_ms <= 0.01f) {
                                    std::snprintf(decay_str, sizeof(decay_str), "Dec: FULL");
                                } else {
                                    std::snprintf(decay_str, sizeof(decay_str), "Dec: %.0fms", step_ref.decay_ms);
                                }
                                if (ImGui::SliderFloat("##DecayLock", &step_ref.decay_ms, 0.0f, 1500.0f, decay_str)) {
                                    // Decay updated
                                }

                                ImGui::SameLine(0, 8);
                                // Analog Drive / Soft Saturation Slider (0% - 100%)
                                ImGui::SetNextItemWidth(80);
                                int drive_pct = static_cast<int>(std::round(step_ref.drive * 100.0f));
                                char drive_str[24];
                                if (drive_pct == 0) {
                                    std::snprintf(drive_str, sizeof(drive_str), "Sat: OFF");
                                } else {
                                    std::snprintf(drive_str, sizeof(drive_str), "Sat: %d%%", drive_pct);
                                }
                                if (ImGui::SliderInt("##DriveLock", &drive_pct, 0, 100, drive_str)) {
                                    step_ref.drive = static_cast<float>(drive_pct) / 100.0f;
                                }

                                ImGui::SameLine(0, 8);
                                // Aux Send A (Reverb) Slider (0% - 100%)
                                ImGui::SetNextItemWidth(75);
                                int send_a_pct = static_cast<int>(std::round(step_ref.send_a * 100.0f));
                                char send_a_str[24];
                                if (send_a_pct == 0) {
                                    std::snprintf(send_a_str, sizeof(send_a_str), "Rev: OFF");
                                } else {
                                    std::snprintf(send_a_str, sizeof(send_a_str), "Rev: %d%%", send_a_pct);
                                }
                                if (ImGui::SliderInt("##SendALock", &send_a_pct, 0, 100, send_a_str)) {
                                    step_ref.send_a = static_cast<float>(send_a_pct) / 100.0f;
                                }

                                ImGui::SameLine(0, 8);
                                // Aux Send B (Delay) Slider (0% - 100%)
                                ImGui::SetNextItemWidth(75);
                                int send_b_pct = static_cast<int>(std::round(step_ref.send_b * 100.0f));
                                char send_b_str[24];
                                if (send_b_pct == 0) {
                                    std::snprintf(send_b_str, sizeof(send_b_str), "Dly: OFF");
                                } else {
                                    std::snprintf(send_b_str, sizeof(send_b_str), "Dly: %d%%", send_b_pct);
                                }
                                if (ImGui::SliderInt("##SendBLock", &send_b_pct, 0, 100, send_b_str)) {
                                    step_ref.send_b = static_cast<float>(send_b_pct) / 100.0f;
                                }
                            }
                            ImGui::EndGroup();

                            ImGui::SameLine(0, 25);

                            // Right sub-pane: 4 Tactile MPC Trigger Pads for immediate auditioning
                            ImGui::BeginGroup();
                            {
                                ImGui::TextColored(ImVec4(0.85f, 0.48f, 0.05f, 1.0f), "MPC AUDITION PADS:");
                                ImGui::SameLine();
                                for (int p = 0; p < 4; ++p) {
                                    char pad_lbl[32];
                                    std::snprintf(pad_lbl, sizeof(pad_lbl), "PAD %d##mpc", p + 1);
                                    if (ImGui::Button(pad_lbl, ImVec2(58, 22))) {
                                        seq->trigger_slice(p, 1.0f);
                                    }
                                    ImGui::SameLine();
                                }
                            }
                            ImGui::EndGroup();
                        }
                    }
                    ImGui::EndChild();
                    ImGui::EndGroup();

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
                // TAB C: UNIVERSAL ROUTING MATRIX & SIGNAL CONDITIONER
                // ------------------------------------------------------------
                if (ImGui::BeginTabItem("  UNIVERSAL ROUTING MATRIX  ")) {
                    ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f),
                                       "Universal Lock-Free Matrix Grid & Cytomic SVF Inline Conditioner");
                    ImGui::Separator();

                    struct MatrixSource {
                        const char* name;
                        routing::RoutingSourceType type;
                        uint32_t id;
                    };

                    struct MatrixDest {
                        const char* name;
                        routing::RoutingDestType type;
                        uint32_t id;
                        uint32_t slot;
                    };

                    constexpr int kNumSources = 8;
                    constexpr int kNumDests = 10;

                    const MatrixSource sources[kNumSources] = {
                        { "Trk 1: Kick/808", routing::RoutingSourceType::TrackAudio, trk0->id() },
                        { "Trk 2: Acid 303", routing::RoutingSourceType::TrackAudio, trk1->id() },
                        { "Trk 3: Vocal",    routing::RoutingSourceType::TrackAudio, trk2->id() },
                        { "Trk 4: Drums",    routing::RoutingSourceType::TrackAudio, trk3->id() },
                        { "Dante Net Ch 1",  routing::RoutingSourceType::NetworkAoip, 0 },
                        { "Dante Net Ch 2",  routing::RoutingSourceType::NetworkAoip, 1 },
                        { "Dante Net Ch 3",  routing::RoutingSourceType::NetworkAoip, 2 },
                        { "Dante Net Ch 4",  routing::RoutingSourceType::NetworkAoip, 3 }
                    };

                    const MatrixDest dests[kNumDests] = {
                        { "Trk 1 Comp SC", routing::RoutingDestType::TrackSidechain, trk0->id(), 1 },
                        { "Trk 2 SC",      routing::RoutingDestType::TrackSidechain, trk1->id(), 0 },
                        { "Trk 3 SC",      routing::RoutingDestType::TrackSidechain, trk2->id(), 0 },
                        { "Trk 4 SC",      routing::RoutingDestType::TrackSidechain, trk3->id(), 0 },
                        { "Aux 1 Reverb",  routing::RoutingDestType::BusAuxInput, bus_reverb->id(), 0 },
                        { "Aux 2 Delay",   routing::RoutingDestType::BusAuxInput, bus_delay->id(), 0 },
                        { "Trk 1 AudioIn", routing::RoutingDestType::TrackAudioInput, trk0->id(), 0 },
                        { "Trk 2 AudioIn", routing::RoutingDestType::TrackAudioInput, trk1->id(), 0 },
                        { "Trk 4 DrumBus", routing::RoutingDestType::TrackAudioInput, trk3->id(), 0 },
                        { "AoIP Tx 1-2",   routing::RoutingDestType::NetworkAoipSink, 0, 0 }
                    };

                    float left_w = ImGui::GetContentRegionAvail().x - 330.0f;
                    if (left_w < 400.0f) left_w = 400.0f;

                    // Left Column: Interactive Pin Matrix Grid
                    ImGui::BeginChild("MatrixGridArea", ImVec2(left_w, 0), false);
                    {
                        // Quick Macro Buttons
                        if (ImGui::Button("+ Kick->Acid Duck (120Hz LP)")) {
                            int32_t pid = mixer.connect_sidechain(trk0->id(), trk1->id(), 0, 120.0f, routing::TapPoint::Input);
                            if (pid > 0) selected_patch_id = static_cast<uint32_t>(pid);
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("+ Vocal->Reverb")) {
                            int32_t pid = mixer.connect_aux_send(trk2->id(), bus_reverb->id(), 0.5f, routing::TapPoint::PostInsert);
                            if (pid > 0) selected_patch_id = static_cast<uint32_t>(pid);
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("+ Kick->DrumBus")) {
                            int32_t pid = mixer.connect_track_audio(trk0->id(), trk3->id(), 1.0f, routing::TapPoint::Input);
                            if (pid > 0) selected_patch_id = static_cast<uint32_t>(pid);
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("+ Dante Ch1->Trk3")) {
                            int32_t pid = mixer.connect_network_sidechain(0, trk2->id(), 0, 1.0f);
                            if (pid > 0) selected_patch_id = static_cast<uint32_t>(pid);
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("+ Master->AoIP Tx")) {
                            int32_t pid = mixer.connect_aoip_transmit(0, true, 0, routing::RouteChannel::StereoBoth);
                            if (pid > 0) selected_patch_id = static_cast<uint32_t>(pid);
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("Clear All")) {
                            mixer.clear_routes();
                            selected_patch_id = 0;
                        }

                        ImGui::Spacing();

                        if (ImGui::BeginTable("MatrixGridTable", kNumDests + 1, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchSame)) {
                            ImGui::TableSetupColumn("Source \\ Destination", ImGuiTableColumnFlags_WidthFixed, 140.0f);
                            for (int c = 0; c < kNumDests; ++c) {
                                ImGui::TableSetupColumn(dests[c].name, ImGuiTableColumnFlags_WidthStretch);
                            }
                            ImGui::TableHeadersRow();

                            for (int r = 0; r < kNumSources; ++r) {
                                ImGui::TableNextRow();
                                ImGui::TableSetColumnIndex(0);
                                ImGui::TextColored(ImVec4(0.2f, 0.25f, 0.35f, 1.0f), "%s", sources[r].name);

                                for (int c = 0; c < kNumDests; ++c) {
                                    ImGui::TableSetColumnIndex(c + 1);
                                    ImGui::PushID(r * 100 + c);

                                    // Find if patch is active
                                    uint32_t patch_id = 0;
                                    bool is_feedback = false;
                                    const auto& patches = mixer.routing_matrix().patches();
                                    for (size_t p = 0; p < routing::UniversalRoutingMatrix::kMaxRoutes; ++p) {
                                        if (patches[p].active &&
                                            patches[p].source_type == sources[r].type &&
                                            patches[p].source_id == sources[r].id &&
                                            patches[p].dest_type == dests[c].type &&
                                            patches[p].dest_id == dests[c].id &&
                                            patches[p].dest_slot == dests[c].slot) {
                                            patch_id = patches[p].id;
                                            is_feedback = patches[p].is_feedback;
                                            break;
                                        }
                                    }

                                    if (patch_id > 0) {
                                        bool is_selected = (selected_patch_id == patch_id);
                                        char btn_label[32];
                                        if (is_feedback) {
                                            std::snprintf(btn_label, sizeof(btn_label), "[ ⮌ Z⁻¹ ]##%u", patch_id);
                                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.35f, 0.1f, 1.0f));
                                        } else if (is_selected) {
                                            std::snprintf(btn_label, sizeof(btn_label), "[ ● SEL ]##%u", patch_id);
                                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.45f, 0.95f, 1.0f));
                                        } else {
                                            std::snprintf(btn_label, sizeof(btn_label), "[ ● ON ]##%u", patch_id);
                                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.25f, 0.55f, 0.85f, 0.8f));
                                        }

                                        if (ImGui::Button(btn_label, ImVec2(-22, 22))) {
                                            selected_patch_id = patch_id;
                                        }
                                        ImGui::PopStyleColor();

                                        ImGui::SameLine();
                                        if (ImGui::Button("x##del", ImVec2(18, 22))) {
                                            mixer.remove_route(patch_id);
                                            if (selected_patch_id == patch_id) selected_patch_id = 0;
                                        }
                                    } else {
                                        if (ImGui::Button("[ + ]", ImVec2(-1, 22))) {
                                            routing::RoutingPatch new_p{};
                                            new_p.source_type = sources[r].type;
                                            new_p.source_id = sources[r].id;
                                            new_p.dest_type = dests[c].type;
                                            new_p.dest_id = dests[c].id;
                                            new_p.dest_slot = dests[c].slot;
                                            new_p.tap_point = routing::TapPoint::Input;
                                            new_p.source_channel = routing::RouteChannel::MonoSum;
                                            new_p.dest_channel = routing::RouteChannel::StereoBoth;
                                            new_p.conditioning.gain = 1.0f;
                                            new_p.tag = std::string(sources[r].name) + " -> " + dests[c].name;
                                            int32_t pid = mixer.add_route(new_p);
                                            if (pid > 0) selected_patch_id = static_cast<uint32_t>(pid);
                                        }
                                    }

                                    ImGui::PopID();
                                }
                            }
                            ImGui::EndTable();
                        }
                    }
                    ImGui::EndChild();

                    // Right Column: Inline Signal Conditioner Inspector
                    ImGui::SameLine();
                    ImGui::BeginChild("PatchInspectorArea", ImVec2(0, 0), true);
                    {
                        routing::RoutingPatch* cur_patch = (selected_patch_id > 0)
                            ? mixer.routing_matrix().get_patch(selected_patch_id) : nullptr;

                        if (cur_patch && cur_patch->active) {
                            ImGui::TextColored(ImVec4(0.12f, 0.45f, 0.95f, 1.0f), "INLINE SIGNAL CONDITIONER");
                            ImGui::Separator();
                            ImGui::TextWrapped("Tag: %s", cur_patch->tag.c_str());

                            if (cur_patch->is_feedback) {
                                ImGui::TextColored(ImVec4(0.85f, 0.35f, 0.1f, 1.0f), "[ ⮌ CYCLIC FEEDBACK: Z⁻¹ DELAY ACTIVE ]");
                            } else {
                                ImGui::TextColored(ImVec4(0.2f, 0.7f, 0.3f, 1.0f), "[ ➔ FEEDFORWARD DIRECT COUPLING ]");
                            }
                            ImGui::Separator();

                            auto& cfg = cur_patch->conditioning;
                            bool cfg_changed = false;

                            // 1. Send Gain
                            float gain_db = 20.0f * std::log10(std::max(0.001f, cfg.gain));
                            ImGui::SetNextItemWidth(140);
                            if (ImGui::SliderFloat("Send Gain", &gain_db, -48.0f, 12.0f, "%.1f dB")) {
                                cfg.gain = std::pow(10.0f, gain_db / 20.0f);
                                cfg_changed = true;
                            }

                            // 2. Tap Point
                            const char* tap_names[5] = { "Input", "PreInsert", "PostInsert", "PreFader", "PostFader" };
                            int cur_tap = static_cast<int>(cur_patch->tap_point);
                            ImGui::SetNextItemWidth(140);
                            if (ImGui::Combo("Tap Point", &cur_tap, tap_names, 5)) {
                                cur_patch->tap_point = static_cast<routing::TapPoint>(cur_tap);
                            }

                            // 3. Cytomic SVF Filter
                            const char* filter_names[5] = { "Bypass", "Lowpass (SVF)", "Highpass", "Bandpass", "Notch" };
                            int cur_filter = static_cast<int>(cfg.filter_mode);
                            ImGui::SetNextItemWidth(140);
                            if (ImGui::Combo("Filter", &cur_filter, filter_names, 5)) {
                                cfg.filter_mode = static_cast<routing::ConditionerFilterMode>(cur_filter);
                                cfg_changed = true;
                            }

                            if (cfg.filter_mode != routing::ConditionerFilterMode::Bypass) {
                                ImGui::SetNextItemWidth(140);
                                if (ImGui::SliderFloat("Cutoff", &cfg.cutoff_hz, 20.0f, 18000.0f, "%.0f Hz", ImGuiSliderFlags_Logarithmic)) {
                                    cfg_changed = true;
                                }
                                ImGui::SetNextItemWidth(140);
                                if (ImGui::SliderFloat("Res Q", &cfg.q, 0.5f, 10.0f, "%.2f")) {
                                    cfg_changed = true;
                                }
                            }

                            // 4. Rectification
                            const char* rect_names[4] = { "Bipolar AC", "Half-Wave", "Full-Wave (|x|)", "Envelope" };
                            int cur_rect = static_cast<int>(cfg.rectify);
                            ImGui::SetNextItemWidth(140);
                            if (ImGui::Combo("Rectify", &cur_rect, rect_names, 4)) {
                                cfg.rectify = static_cast<routing::ConditionerRectifyMode>(cur_rect);
                                cfg_changed = true;
                            }

                            // 5. Invert Phase
                            if (ImGui::Checkbox("Invert Phase (180°)", &cfg.invert_phase)) {
                                cfg_changed = true;
                            }

                            if (cfg_changed) {
                                mixer.update_route_conditioning(selected_patch_id, cfg);
                            }

                            ImGui::Spacing();
                            if (ImGui::Button("DISCONNECT THIS ROUTE", ImVec2(-1, 24))) {
                                mixer.remove_route(selected_patch_id);
                                selected_patch_id = 0;
                            }
                        } else {
                            ImGui::TextDisabled("NO ROUTE SELECTED");
                            ImGui::Separator();
                            ImGui::TextWrapped("Click any [ + ] cell in the matrix grid to create a real-time virtual patch cable.");
                            ImGui::Spacing();
                            ImGui::TextWrapped("Features zero converter friction: each patch includes its own Cytomic SVF filter, rectification, and automatic cyclic feedback Z^-1 delay decoupling.");
                        }
                    }
                    ImGui::EndChild();

                    ImGui::EndTabItem();
                }

                // ------------------------------------------------------------
                // TAB D: PIPEWIRE & NETWORK AoIP BRIDGE
                // ------------------------------------------------------------
                if (ImGui::BeginTabItem("  PIPEWIRE & NETWORK AoIP BRIDGE  ")) {
                    ImGui::TextColored(ImVec4(0.12f, 0.45f, 0.95f, 1.0f),
                                       "Linux Audio Graph (PipeWire pw_filter) & Low-Latency AoIP Stream Receiver");
                    ImGui::Separator();

                    float half_w = ImGui::GetContentRegionAvail().x * 0.5f - 8.0f;
                    if (half_w < 350.0f) half_w = 350.0f;

                    // Left Column: PipeWire Graph & Linux App Discovery
                    ImGui::BeginChild("PipeWireDiscoveryPane", ImVec2(half_w, 0), true);
                    {
                        ImGui::TextColored(ImVec4(0.85f, 0.48f, 0.05f, 1.0f), "DISCOVERED PIPEWIRE STREAMS & PORTS");
                        ImGui::SameLine();
                        if (ImGui::SmallButton("RESCAN NOW")) {
                            pw.refresh_discovery();
                        }
                        ImGui::Separator();

                        auto sources = pw.get_available_sources();
                        auto sinks = pw.get_available_sinks();

                        ImGui::Text("Audio Capture & App Sources (%zu discovered):", sources.size());
                        if (sources.empty()) {
                            ImGui::TextDisabled("No external PipeWire sources found (or daemon offline).");
                        } else {
                            if (ImGui::BeginTable("SourcesTable", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                                ImGui::TableSetupColumn("Device / App", ImGuiTableColumnFlags_WidthStretch);
                                ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 70.0f);
                                ImGui::TableSetupColumn("Port L / R", ImGuiTableColumnFlags_WidthStretch);
                                ImGui::TableSetupColumn("Quick Patch", ImGuiTableColumnFlags_WidthFixed, 140.0f);
                                ImGui::TableHeadersRow();

                                for (size_t i = 0; i < sources.size(); ++i) {
                                    const auto& src = sources[i];
                                    ImGui::TableNextRow();
                                    ImGui::TableSetColumnIndex(0);
                                    ImGui::Text("%s", src.display_name.c_str());

                                    ImGui::TableSetColumnIndex(1);
                                    if (src.is_hardware_capture) {
                                        ImGui::TextColored(ImVec4(0.2f, 0.8f, 0.3f, 1.0f), "HARDWARE");
                                    } else if (src.is_monitor) {
                                        ImGui::TextColored(ImVec4(0.9f, 0.6f, 0.1f, 1.0f), "LOOPBACK");
                                    } else {
                                        ImGui::TextColored(ImVec4(0.3f, 0.6f, 0.9f, 1.0f), "APP STREAM");
                                    }

                                    ImGui::TableSetColumnIndex(2);
                                    ImGui::TextDisabled("%.22s", src.port_l.c_str());

                                    ImGui::TableSetColumnIndex(3);
                                    ImGui::PushID(static_cast<int>(i));
                                    for (uint32_t t = 1; t <= 4; ++t) {
                                        char trk_btn[16];
                                        std::snprintf(trk_btn, sizeof(trk_btn), "T%u", t);
                                        if (ImGui::SmallButton(trk_btn)) {
                                            pw.link_source_to_track(src, t);
                                        }
                                        if (t < 4) ImGui::SameLine();
                                    }
                                    ImGui::PopID();
                                }
                                ImGui::EndTable();
                            }
                        }

                        ImGui::Spacing();
                        ImGui::Text("Audio Playback Sinks (%zu discovered):", sinks.size());
                        if (sinks.empty()) {
                            ImGui::TextDisabled("No external PipeWire sinks found.");
                        } else {
                            for (const auto& snk : sinks) {
                                ImGui::BulletText("%s (%s)", snk.display_name.c_str(), snk.node_name.c_str());
                            }
                        }
                    }
                    ImGui::EndChild();

                    ImGui::SameLine();

                    // Right Column: Audio-over-IP (AoIP) Dante / AES67 Telemetry & Mapping
                    ImGui::BeginChild("AoipTelemetryPane", ImVec2(0, 0), true);
                    {
                        ImGui::TextColored(ImVec4(0.20f, 0.70f, 0.85f, 1.0f), "AoIP NETWORK STREAM RECEIVER (DANTE / AES67)");
                        ImGui::Separator();

                        const auto& stats = aoip_rx.stats();
                        uint64_t pkts = stats.packets_received.load(std::memory_order_relaxed);
                        uint64_t frames = stats.frames_received.load(std::memory_order_relaxed);
                        uint64_t drops = stats.packets_dropped.load(std::memory_order_relaxed);
                        uint64_t ooo = stats.out_of_order.load(std::memory_order_relaxed);
                        uint32_t sr = stats.sample_rate.load(std::memory_order_relaxed);
                        uint16_t ch = stats.channels.load(std::memory_order_relaxed);
                        int64_t instant_j = stats.jitter_ns.load(std::memory_order_relaxed);
                        int64_t avg_j = stats.avg_jitter_ns.load(std::memory_order_relaxed);
                        uint64_t max_j = stats.max_jitter_ns.load(std::memory_order_relaxed);
                        auto ptp_src = static_cast<network::PtpTimestampSource>(stats.timestamp_source.load(std::memory_order_relaxed));
                        bool hw_locked = stats.hardware_locked.load(std::memory_order_relaxed);

                        ImGui::Text("Socket Status: UDP Port 4848 (SO_TIMESTAMPING %s)",
                                    aoip_rx.ptp_engine().is_so_timestamping_active() ? "ACTIVE" : "INACTIVE");
                        ImGui::Text("Wire Protocol: RTP L24 Uncompressed / Dante Multicast");

                        if (hw_locked) {
                            ImGui::TextColored(ImVec4(0.25f, 0.85f, 0.35f, 1.0f),
                                               "PTPv2 Timestamping: HARDWARE NIC PHY (IEEE 1588-2008 Latch)");
                        } else if (ptp_src == network::PtpTimestampSource::KernelDriverStack) {
                            ImGui::TextColored(ImVec4(0.20f, 0.75f, 0.90f, 1.0f),
                                               "PTPv2 Timestamping: KERNEL DRIVER STACK (SOF_TIMESTAMPING_RX_SOFTWARE)");
                        } else {
                            ImGui::TextColored(ImVec4(0.70f, 0.70f, 0.70f, 1.0f),
                                               "PTPv2 Timestamping: USERSPACE MONOTONIC RAW (Fallback)");
                        }

                        ImGui::Separator();

                        ImGui::Columns(2, "AoipStatsColumns", false);
                        ImGui::Text("Packets Received:"); ImGui::NextColumn();
                        ImGui::TextColored(ImVec4(0.2f, 0.8f, 0.3f, 1.0f), "%lu", static_cast<unsigned long>(pkts)); ImGui::NextColumn();

                        ImGui::Text("Audio Frames Ingested:"); ImGui::NextColumn();
                        ImGui::Text("%lu", static_cast<unsigned long>(frames)); ImGui::NextColumn();

                        ImGui::Text("Packets Dropped / Burst Loss:"); ImGui::NextColumn();
                        if (drops > 0) {
                            ImGui::TextColored(ImVec4(0.9f, 0.2f, 0.2f, 1.0f), "%lu", static_cast<unsigned long>(drops));
                        } else {
                            ImGui::TextColored(ImVec4(0.4f, 0.8f, 0.4f, 1.0f), "0 (Zero Loss)");
                        }
                        ImGui::NextColumn();

                        ImGui::Text("Out of Order / Reordered:"); ImGui::NextColumn();
                        ImGui::Text("%lu", static_cast<unsigned long>(ooo)); ImGui::NextColumn();

                        ImGui::Text("Instant Transit Jitter:"); ImGui::NextColumn();
                        ImGui::TextColored(ImVec4(0.3f, 0.8f, 0.9f, 1.0f), "%ld ns (%.2f µs)",
                                           static_cast<long>(instant_j), static_cast<double>(instant_j) * 1e-3); ImGui::NextColumn();

                        ImGui::Text("Moving Average Jitter (ODE):"); ImGui::NextColumn();
                        ImGui::TextColored(ImVec4(0.3f, 0.8f, 0.9f, 1.0f), "%ld ns (%.2f µs)",
                                           static_cast<long>(avg_j), static_cast<double>(avg_j) * 1e-3); ImGui::NextColumn();

                        ImGui::Text("Peak Jitter Observed:"); ImGui::NextColumn();
                        ImGui::TextColored(ImVec4(0.85f, 0.5f, 0.1f, 1.0f), "%lu ns (%.2f µs)",
                                           static_cast<unsigned long>(max_j), static_cast<double>(max_j) * 1e-3); ImGui::NextColumn();

                        ImGui::Text("Stream Sample Rate:"); ImGui::NextColumn();
                        ImGui::Text("%u Hz", sr); ImGui::NextColumn();

                        ImGui::Text("Stream Channels:"); ImGui::NextColumn();
                        ImGui::Text("%u Channels (Interleaved/Planar)", ch); ImGui::NextColumn();
                        ImGui::Columns(1);

                        ImGui::Separator();
                        ImGui::TextColored(ImVec4(0.85f, 0.5f, 0.1f, 1.0f), "Network Track Mapping (Pre-allocated SPSC Jitter Buffers):");
                        ImGui::BulletText("Track 1 (Kick/808): Dante Network Ch 1 & 2");
                        ImGui::BulletText("Track 2 (Acid 303): Dante Network Ch 3 & 4");
                        ImGui::BulletText("Track 3 (Vocal):    Dante Network Ch 5 & 6");
                        ImGui::BulletText("Track 4 (Drums):    Dante Network Ch 7 & 8");

                        ImGui::Spacing();
                        ImGui::TextDisabled("Set channel strip input to [ AOIP ] to stream directly from Dante device into channel strip inserts.");
                    }
                    ImGui::EndChild();

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

                            // INPUT SELECTION & PRE-INSERT GAIN/PHASE STAGING
                            auto* trk_ptr = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                            TrackInputMode in_mode = trk_ptr ? trk_ptr->input_mode() : TrackInputMode::InternalClip;
                            const char* mode_labels[4] = { "CLIP", "PIPEWIRE", "MERGE", "AOIP" };
                            int cur_mode_idx = static_cast<int>(in_mode);

                            // Row 1: Input Mode & Phase Invert Button
                            ImGui::TextDisabled("In:");
                            ImGui::SameLine();
                            ImGui::SetNextItemWidth(90);
                            if (ImGui::Combo("##InMode", &cur_mode_idx, mode_labels, 4)) {
                                if (trk_ptr) {
                                    auto new_mode = static_cast<TrackInputMode>(cur_mode_idx);
                                    trk_ptr->set_input_mode(new_mode);
                                    if (new_mode == TrackInputMode::InternalClip) {
                                        pw.unlink_all_for_track(t + 1);
                                    }
                                }
                            }
                            ImGui::SameLine();
                            bool phase_inv = trk_ptr ? trk_ptr->input_phase_invert() : false;
                            if (phase_inv) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.45f, 0.10f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            }
                            if (ImGui::Button("Ø##Phase", ImVec2(26, 20))) {
                                if (trk_ptr) trk_ptr->set_input_phase_invert(!phase_inv);
                            }
                            if (phase_inv) ImGui::PopStyleColor(2);
                            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Pre-insert Phase Invert (180° Polarity Reversal)");

                            // Row 2: Input Trim (-24 dB .. +24 dB) & Pre-insert Input Peak LED
                            ImGui::TextDisabled("Trim:");
                            ImGui::SameLine();
                            float cur_gain_db = ui::linear_to_db(trk_ptr ? trk_ptr->input_gain() : 1.0f);
                            if (cur_gain_db < -24.0f) cur_gain_db = -24.0f;
                            if (cur_gain_db > 24.0f) cur_gain_db = 24.0f;
                            ImGui::SetNextItemWidth(100);
                            if (ImGui::SliderFloat("##Trim", &cur_gain_db, -24.0f, 24.0f, "%+.1f dB")) {
                                if (trk_ptr) trk_ptr->set_input_gain(ui::db_to_linear(cur_gain_db));
                            }
                            if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                                if (trk_ptr) trk_ptr->set_input_gain(1.0f);
                            }
                            ImGui::SameLine();
                            auto in_meters = trk_ptr ? trk_ptr->input_meter() : std::pair<float, float>{0.0f, 0.0f};
                            float in_peak = std::max(in_meters.first, in_meters.second);
                            ImVec4 led_col = (in_peak >= 0.999f) ? ImVec4(0.95f, 0.15f, 0.15f, 1.0f) :
                                             ((in_peak >= 0.005f) ? ImVec4(0.20f, 0.85f, 0.35f, 1.0f) : ImVec4(0.45f, 0.50f, 0.55f, 0.5f));
                            ImGui::TextColored(led_col, "●");
                            if (ImGui::IsItemHovered()) {
                                ImGui::SetTooltip("Pre-insert Input Level: %.1f dBFS\n(Double-click Trim to reset 0.0 dB)", ui::linear_to_db(in_peak));
                            }

                            // Row 3: Dynamic Contextual Hardware / AoIP Patching
                            if (in_mode == TrackInputMode::PipeWireStream || in_mode == TrackInputMode::MergeAll) {
                                auto linked_src = pw.get_track_source(t + 1);
                                if (linked_src.has_value()) {
                                    ImGui::TextColored(ImVec4(0.20f, 0.85f, 0.35f, 1.0f), "● [%s] %.16s",
                                                       linked_src->is_mono ? "1ch" : "2ch",
                                                       linked_src->display_name.c_str());
                                } else {
                                    ImGui::TextDisabled("○ External unlinked");
                                }

                                auto sources = pw.get_available_sources();
                                static int sel_pw_source[4] = { 0, 0, 0, 0 };
                                if (!sources.empty()) {
                                    if (sel_pw_source[t] >= static_cast<int>(sources.size())) sel_pw_source[t] = 0;
                                    ImGui::SetNextItemWidth(110);
                                    char combo_preview[64];
                                    std::snprintf(combo_preview, sizeof(combo_preview), "[%s] %.12s",
                                                  sources[sel_pw_source[t]].is_hardware_capture ? (sources[sel_pw_source[t]].is_mono ? "HW1" : "HW2") : "APP",
                                                  sources[sel_pw_source[t]].display_name.c_str());
                                    if (ImGui::BeginCombo("##PWSrc", combo_preview)) {
                                        for (size_t s_idx = 0; s_idx < sources.size(); ++s_idx) {
                                            bool is_selected = (sel_pw_source[t] == static_cast<int>(s_idx));
                                            char item_name[128];
                                            std::snprintf(item_name, sizeof(item_name), "[%s] %s",
                                                          sources[s_idx].is_hardware_capture ? (sources[s_idx].is_mono ? "HW 1ch" : "HW 2ch") : "APP",
                                                          sources[s_idx].display_name.c_str());
                                            if (ImGui::Selectable(item_name, is_selected)) {
                                                sel_pw_source[t] = static_cast<int>(s_idx);
                                            }
                                        }
                                        ImGui::EndCombo();
                                    }
                                    ImGui::SameLine();
                                    if (ImGui::SmallButton("PATCH")) {
                                        pw.link_source_to_track(sources[sel_pw_source[t]], t + 1);
                                    }
                                    ImGui::SameLine();
                                    if (ImGui::SmallButton("DEL")) {
                                        pw.unlink_all_for_track(t + 1);
                                    }
                                } else {
                                    ImGui::TextDisabled("No PW streams found");
                                }
                            } else if (in_mode == TrackInputMode::NetworkAoip) {
                                ImGui::TextColored(ImVec4(0.20f, 0.70f, 0.85f, 1.0f), "Dante Ch %d/%d (UDP:4848)", t * 2 + 1, t * 2 + 2);
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

                            // 4 Insert Slots (Zähl AM1 Modular Inserts)
                            ImGui::TextColored(ImVec4(0.35f, 0.40f, 0.48f, 1.0f), "Insert Slots (4x):");
                            for (int s = 0; s < 4; ++s) {
                                ImGui::PushID(s);
                                ImGui::TextDisabled("S%d:", s + 1);
                                ImGui::SameLine();
                                const char* slot_name = "[Empty]";
                                bool is_by = false;
                                bool has_proc = (trk_ptr && trk_ptr->slot(s).processor() != nullptr);
                                if (has_proc) {
                                    slot_name = trk_ptr->slot(s).processor()->name();
                                    is_by = trk_ptr->slot(s).is_bypassed();
                                }
                                char btn_label[48];
                                std::snprintf(btn_label, sizeof(btn_label), "%.14s", slot_name);

                                if (has_proc) {
                                    if (is_by) {
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.55f, 0.55f, 0.60f, 1.0f));
                                    } else {
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.12f, 0.45f, 0.95f, 1.0f));
                                    }
                                } else {
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.40f, 0.45f, 0.50f, 0.7f));
                                }

                                if (ImGui::Button(btn_label, ImVec2(96, 18))) {
                                    ImGui::OpenPopup("InsertSelectMenu");
                                }
                                ImGui::PopStyleColor();

                                if (ImGui::IsItemHovered() && has_proc) {
                                    ImGui::SetTooltip("%s (Click to change/clear)", trk_ptr->slot(s).processor()->name());
                                } else if (ImGui::IsItemHovered()) {
                                    ImGui::SetTooltip("Click to load DSP module into Slot %d", s + 1);
                                }

                                if (ImGui::BeginPopup("InsertSelectMenu")) {
                                    ImGui::TextColored(ImVec4(0.12f, 0.45f, 0.95f, 1.0f), "TRACK %d - INSERT SLOT %d", t + 1, s + 1);
                                    ImGui::Separator();
                                    if (ImGui::MenuItem("None / Clear Slot", nullptr, !has_proc)) {
                                        if (trk_ptr) trk_ptr->slot(s).set_processor(nullptr);
                                    }
                                    ImGui::Separator();
                                    if (ImGui::MenuItem("Airwindows Baxandall EQ")) {
                                        if (trk_ptr) {
                                            auto p = std::make_shared<dsp::Baxandall>();
                                            p->init(kSampleRate);
                                            trk_ptr->slot(s).set_processor(p);
                                        }
                                    }
                                    if (ImGui::MenuItem("Airwindows ButterComp2")) {
                                        if (trk_ptr) {
                                            auto p = std::make_shared<dsp::ButterComp2>();
                                            p->init(kSampleRate);
                                            trk_ptr->slot(s).set_processor(p);
                                        }
                                    }
                                    if (ImGui::MenuItem("Airwindows PurestDrive")) {
                                        if (trk_ptr) {
                                            auto p = std::make_shared<dsp::PurestDrive>();
                                            p->init(kSampleRate);
                                            trk_ptr->slot(s).set_processor(p);
                                        }
                                    }
                                    if (ImGui::MenuItem("Airwindows DeRez2 Crunch")) {
                                        if (trk_ptr) {
                                            auto p = std::make_shared<dsp::DeRez>();
                                            p->init(kSampleRate);
                                            p->set_parameter(0, 0.90f);
                                            p->set_parameter(1, 0.80f);
                                            p->set_parameter(2, 0.0f); // mu-law
                                            p->set_parameter(3, 1.0f);
                                            trk_ptr->slot(s).set_processor(p);
                                        }
                                    }
                                    ImGui::Separator();
                                    if (ImGui::MenuItem("Liquid Vactrol Leveler (LA-2A Opto)")) {
                                        if (trk_ptr) {
                                            auto p = std::make_shared<dsp::LiquidVactrolProcessor>(kSampleRate);
                                            p->init(kSampleRate);
                                            p->set_parameter(0, 0.60f); // Peak Reduction
                                            p->set_parameter(1, 0.0f);  // Makeup
                                            p->set_parameter(2, 0.0f);  // OptoCompressor
                                            p->set_parameter(3, 0.75f); // Memory depth
                                            p->set_parameter(4, 0.50f); // HF Emphasis (R37)
                                            trk_ptr->slot(s).set_processor(p);
                                        }
                                    }
                                    if (ImGui::MenuItem("Buchla 292 LPG (Vactrol Low-Pass Gate)")) {
                                        if (trk_ptr) {
                                            auto p = std::make_shared<dsp::LiquidVactrolProcessor>(kSampleRate);
                                            p->init(kSampleRate);
                                            p->set_parameter(0, 0.75f); // Sensitivity
                                            p->set_parameter(1, 0.0f);
                                            p->set_parameter(2, 2.0f);  // BuchlaLPG
                                            p->set_parameter(6, 0.35f); // LPG Resonance
                                            trk_ptr->slot(s).set_processor(p);
                                        }
                                    }
                                    if (ImGui::MenuItem("Sovereign MultiHead ODE Compressor")) {
                                        if (trk_ptr) {
                                            auto p = std::make_shared<dsp::MultiHeadOdeProcessor>(kSampleRate, 4);
                                            p->init(kSampleRate);
                                            trk_ptr->slot(s).set_processor(p);
                                        }
                                    }
                                    ImGui::EndPopup();
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
                            if (trk_ptr) {
                                track_gains[t] = trk_ptr->gain();
                            }
                            ImGui::VSliderFloat("##fader", ImVec2(34, 110), &track_gains[t], 0.0f, 1.25f, "");

                            if (ImGui::IsItemEdited()) {
                                protocol::MixerCommand cmd{};
                                cmd.type = protocol::MixerCommandType::SetTrackGain;
                                cmd.target_id = t + 1;
                                cmd.value1 = track_gains[t];
                                mixer.post_command(cmd);
                            }

                            if (ImGui::BeginPopupContextItem("FaderMidiCtx")) {
                                uint8_t b_ch = 0, b_cc = 0;
                                midi::MidiLearnTarget tgt{midi::MidiLearnTargetType::TrackGain, static_cast<uint32_t>(t + 1), 0, 0};
                                bool is_bound = midi_learn.is_target_bound(tgt, &b_ch, &b_cc);
                                if (is_bound) {
                                    ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.4f, 1.0f), "MIDI Bound: CC %u (Ch %u)", b_cc, b_ch);
                                    if (ImGui::MenuItem("Unbind MIDI CC")) {
                                        midi_learn.unbind_target(tgt);
                                    }
                                } else {
                                    if (ImGui::MenuItem("Learn MIDI CC")) {
                                        std::string lbl = "Track " + std::to_string(t + 1) + " Gain";
                                        midi_learn.arm_learn(tgt, 0.0f, 1.25f, lbl);
                                    }
                                }
                                ImGui::EndPopup();
                            }

                            ImGui::SameLine();
                            ImVec2 meter_pos = ImGui::GetCursorScreenPos();
                            ui::DrawDbMeter(ImGui::GetWindowDrawList(), meter_pos, ImVec2(24, 110),
                                            telemetry.track_meters[t].peak_l, telemetry.track_meters[t].peak_r,
                                            telemetry.track_meters[t].rms_l, telemetry.track_meters[t].rms_r,
                                            telemetry.track_meters[t].peak_l >= 1.0f);
                            ImGui::Dummy(ImVec2(26, 110));

                            // Numerical dB read + MIDI CC Badge
                            float db = ui::linear_to_db(track_gains[t]);
                            ImGui::Text("%.1f dB", db);
                            uint8_t b_ch = 0, b_cc = 0;
                            midi::MidiLearnTarget tgt{midi::MidiLearnTargetType::TrackGain, static_cast<uint32_t>(t + 1), 0, 0};
                            if (midi_learn.is_target_bound(tgt, &b_ch, &b_cc)) {
                                ImGui::SameLine();
                                ImGui::TextColored(ImVec4(0.95f, 0.65f, 0.15f, 1.0f), "[CC%u]", b_cc);
                            } else if (midi_learn.is_learning() && midi_learn.learn_target() == tgt) {
                                ImGui::SameLine();
                                ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "[LRN]");
                            }
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

                        master_gain = mixer.master_volume();
                        ImGui::VSliderFloat("##mfader", ImVec2(34, 110), &master_gain, 0.0f, 1.25f, "");
                        if (ImGui::IsItemEdited()) {
                            protocol::MixerCommand cmd{};
                            cmd.type = protocol::MixerCommandType::SetMasterGain;
                            cmd.value1 = master_gain;
                            mixer.post_command(cmd);
                        }
                        if (ImGui::BeginPopupContextItem("MasterMidiCtx")) {
                            uint8_t b_ch = 0, b_cc = 0;
                            midi::MidiLearnTarget tgt{midi::MidiLearnTargetType::MasterVolume, 0, 0, 0};
                            bool is_bound = midi_learn.is_target_bound(tgt, &b_ch, &b_cc);
                            if (is_bound) {
                                ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.4f, 1.0f), "MIDI Bound: CC %u (Ch %u)", b_cc, b_ch);
                                if (ImGui::MenuItem("Unbind MIDI CC")) {
                                    midi_learn.unbind_target(tgt);
                                }
                            } else {
                                if (ImGui::MenuItem("Learn MIDI CC")) {
                                    midi_learn.arm_learn(tgt, 0.0f, 1.25f, "Master Volume");
                                }
                            }
                            ImGui::EndPopup();
                        }
                        ImGui::SameLine();
                        ImVec2 mpos = ImGui::GetCursorScreenPos();
                        ui::DrawDbMeter(ImGui::GetWindowDrawList(), mpos, ImVec2(24, 110),
                                        telemetry.master_meter.peak_l, telemetry.master_meter.peak_r,
                                        telemetry.master_meter.rms_l, telemetry.master_meter.rms_r,
                                        telemetry.master_meter.peak_l >= 1.0f);
                        ImGui::Dummy(ImVec2(26, 110));
                        ImGui::Text("%.1f dB", ui::linear_to_db(master_gain));
                        uint8_t mb_ch = 0, mb_cc = 0;
                        midi::MidiLearnTarget mtgt{midi::MidiLearnTargetType::MasterVolume, 0, 0, 0};
                        if (midi_learn.is_target_bound(mtgt, &mb_ch, &mb_cc)) {
                            ImGui::SameLine();
                            ImGui::TextColored(ImVec4(0.95f, 0.65f, 0.15f, 1.0f), "[CC%u]", mb_cc);
                        } else if (midi_learn.is_learning() && midi_learn.learn_target() == mtgt) {
                            ImGui::SameLine();
                            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "[LRN]");
                        }
                    }
                    ImGui::EndChild();
                    ImGui::PopID();

                    // Render Selected Track Modular DSP Rack Inspector (Eurorack / 500-Series Style)
                    ImGui::SameLine();
                    ImGui::PushID(900);
                    ImGui::BeginChild("TrackDspRack_Inspector", ImVec2(340, 0), true);
                    {
                        // Header
                        const char* cur_trk_name = track_names[selected_track];
                        ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "DSP RACK: TRACK %d", selected_track + 1);
                        ImGui::SameLine();
                        ImGui::TextDisabled("(%s)", cur_trk_name);

                        auto* sel_trk = (selected_track == 0) ? trk0 : ((selected_track == 1) ? trk1 : ((selected_track == 2) ? trk2 : trk3));

                        ImGui::SameLine(ImGui::GetContentRegionAvail().x - 90);
                        if (ImGui::SmallButton("SAVE##rk")) {
                            std::snprintf(rack_preset_file_path, sizeof(rack_preset_file_path), "rack_track_%d.json", selected_track + 1);
                            open_save_rack_modal = true;
                        }
                        ImGui::SameLine();
                        if (ImGui::SmallButton("LOAD##rk")) {
                            std::snprintf(rack_preset_file_path, sizeof(rack_preset_file_path), "rack_track_%d.json", selected_track + 1);
                            open_load_rack_modal = true;
                        }
                        ImGui::Separator();

                        // 4 Modular Rack Units
                        for (int s = 0; s < 4; ++s) {
                            ImGui::PushID(s);
                            bool has_p = (sel_trk && sel_trk->slot(s).processor() != nullptr);
                            auto* proc = has_p ? sel_trk->slot(s).processor() : nullptr;
                            bool is_by = has_p ? sel_trk->slot(s).is_bypassed() : true;

                            ImVec4 frame_bg = has_p ? (is_by ? ImVec4(0.94f, 0.94f, 0.96f, 1.0f) : ImVec4(0.91f, 0.94f, 0.98f, 1.0f))
                                                    : ImVec4(0.96f, 0.96f, 0.97f, 1.0f);
                            ImGui::PushStyleColor(ImGuiCol_ChildBg, frame_bg);
                            char frame_id[32];
                            std::snprintf(frame_id, sizeof(frame_id), "RackSlot_%d", s + 1);
                            ImGui::BeginChild(frame_id, ImVec2(0, 100), true);
                            {
                                // Unit Faceplate Top Bar
                                ImGui::TextColored(has_p ? ImVec4(0.12f, 0.38f, 0.85f, 1.0f) : ImVec4(0.50f, 0.55f, 0.60f, 1.0f),
                                                   "S%d: %s", s + 1, has_p ? proc->name() : "[EMPTY]");

                                if (has_p) {
                                    ImGui::SameLine(ImGui::GetContentRegionAvail().x - 70);
                                    if (ImGui::SmallButton(is_by ? "OFF##by" : "ON##by")) {
                                        sel_trk->slot(s).set_bypass(!is_by);
                                    }
                                    ImGui::SameLine();
                                    if (ImGui::SmallButton("X##ej")) {
                                        sel_trk->slot(s).set_processor(nullptr);
                                    }
                                    ImGui::Separator();

                                    // Detailed DSP Controls based on Processor Name
                                    std::string pname = proc->name();
                                    if (pname.find("Baxandall") != std::string::npos) {
                                        float bass = proc->get_parameter(0);
                                        float treble = proc->get_parameter(1);
                                        ImGui::SetNextItemWidth(120);
                                        if (ImGui::SliderFloat("Bass##bax", &bass, -12.0f, 12.0f, "%.1f dB")) {
                                            proc->set_parameter(0, bass);
                                        }
                                        ImGui::SameLine(0, 10);
                                        ImGui::SetNextItemWidth(120);
                                        if (ImGui::SliderFloat("Treble##bax", &treble, -12.0f, 12.0f, "%.1f dB")) {
                                            proc->set_parameter(1, treble);
                                        }
                                        ImGui::TextDisabled("C^inf Smooth Baxandall Tone Stack");
                                    } else if (pname.find("ButterComp2") != std::string::npos) {
                                        float comp = proc->get_parameter(0);
                                        float out = proc->get_parameter(1);
                                        ImGui::SetNextItemWidth(120);
                                        if (ImGui::SliderFloat("Comp##bc", &comp, 0.0f, 1.0f, "%.2f")) {
                                            proc->set_parameter(0, comp);
                                        }
                                        ImGui::SameLine(0, 10);
                                        ImGui::SetNextItemWidth(120);
                                        if (ImGui::SliderFloat("Out##bc", &out, 0.0f, 1.0f, "%.2f")) {
                                            proc->set_parameter(1, out);
                                        }
                                        float est_gr = comp * 0.7f;
                                        ImGui::ProgressBar(est_gr, ImVec2(-1, 6), "");
                                        ImGui::TextDisabled("Butterworth Dynamics & Master Glue");
                                    } else if (pname.find("PurestDrive") != std::string::npos) {
                                        float drv = proc->get_parameter(0);
                                        ImGui::SetNextItemWidth(180);
                                        if (ImGui::SliderFloat("Drive##pdrv", &drv, 0.0f, 1.0f, "%.2f")) {
                                            proc->set_parameter(0, drv);
                                        }
                                        ImGui::SameLine();
                                        ImGui::TextColored(ImVec4(0.85f, 0.35f, 0.10f, 1.0f), "%.0f%% Sat", drv * 100.0f);
                                        ImGui::TextDisabled("Pure Non-Linear Console Saturation");
                                    } else if (pname.find("DeRez") != std::string::npos) {
                                        float rate = proc->get_parameter(0);
                                        float res = proc->get_parameter(1);
                                        ImGui::SetNextItemWidth(120);
                                        if (ImGui::SliderFloat("Rate##drz", &rate, 0.0f, 1.0f, "%.2f")) {
                                            proc->set_parameter(0, rate);
                                        }
                                        ImGui::SameLine(0, 10);
                                        ImGui::SetNextItemWidth(120);
                                        if (ImGui::SliderFloat("Res##drz", &res, 0.0f, 1.0f, "%.2f")) {
                                            proc->set_parameter(1, res);
                                        }
                                        ImGui::TextDisabled("Vintage Sampler Variable-Clock & Mu-Law");
                                    } else if (pname.find("Vactrol") != std::string::npos || pname.find("LA-2A") != std::string::npos || pname.find("Buchla") != std::string::npos) {
                                        float pr = proc->get_parameter(0);
                                        float mk = proc->get_parameter(1);
                                        float hf = proc->get_parameter(4);
                                        ImGui::SetNextItemWidth(100);
                                        if (ImGui::SliderFloat("Red##vac", &pr, 0.0f, 1.0f, "%.2f")) {
                                            proc->set_parameter(0, pr);
                                        }
                                        ImGui::SameLine(0, 8);
                                        ImGui::SetNextItemWidth(90);
                                        if (ImGui::SliderFloat("Gain##vac", &mk, -6.0f, 18.0f, "%.1fdB")) {
                                            proc->set_parameter(1, mk);
                                        }
                                        ImGui::SameLine(0, 8);
                                        ImGui::SetNextItemWidth(65);
                                        if (ImGui::SliderFloat("R37##vac", &hf, 0.0f, 1.0f, "%.2f")) {
                                            proc->set_parameter(4, hf);
                                        }
                                        float opto_gr = pr * 0.85f;
                                        ImGui::ProgressBar(opto_gr, ImVec2(-1, 6), "");
                                        ImGui::TextDisabled("Optical CdS Dark Memory Photocell Leveler");
                                    } else {
                                        float p0 = proc->get_parameter(0);
                                        float p1 = proc->get_parameter(1);
                                        ImGui::SetNextItemWidth(120);
                                        if (ImGui::SliderFloat("Param 1##gen", &p0, 0.0f, 1.0f, "%.2f")) {
                                            proc->set_parameter(0, p0);
                                        }
                                        ImGui::SameLine(0, 10);
                                        ImGui::SetNextItemWidth(120);
                                        if (ImGui::SliderFloat("Param 2##gen", &p1, 0.0f, 1.0f, "%.2f")) {
                                            proc->set_parameter(1, p1);
                                        }
                                        ImGui::TextDisabled("Real-Time IProcessor DSP Module");
                                    }
                                } else {
                                    ImGui::Separator();
                                    ImGui::Spacing();
                                    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 30);
                                    if (ImGui::Button("+ LOAD DSP MODULE...", ImVec2(240, 26))) {
                                        ImGui::OpenPopup("RackSlotLoadPopup");
                                    }
                                    if (ImGui::BeginPopup("RackSlotLoadPopup")) {
                                        ImGui::TextColored(ImVec4(0.12f, 0.45f, 0.95f, 1.0f), "LOAD MODULE INTO SLOT %d", s + 1);
                                        ImGui::Separator();
                                        if (ImGui::MenuItem("Airwindows Baxandall EQ")) {
                                            auto p = std::make_shared<dsp::Baxandall>();
                                            p->init(kSampleRate);
                                            sel_trk->slot(s).set_processor(p);
                                        }
                                        if (ImGui::MenuItem("Airwindows ButterComp2")) {
                                            auto p = std::make_shared<dsp::ButterComp2>();
                                            p->init(kSampleRate);
                                            sel_trk->slot(s).set_processor(p);
                                        }
                                        if (ImGui::MenuItem("Airwindows PurestDrive")) {
                                            auto p = std::make_shared<dsp::PurestDrive>();
                                            p->init(kSampleRate);
                                            sel_trk->slot(s).set_processor(p);
                                        }
                                        if (ImGui::MenuItem("Airwindows DeRez2 Crunch")) {
                                            auto p = std::make_shared<dsp::DeRez>();
                                            p->init(kSampleRate);
                                            p->set_parameter(0, 0.90f);
                                            p->set_parameter(1, 0.80f);
                                            p->set_parameter(2, 0.0f);
                                            p->set_parameter(3, 1.0f);
                                            sel_trk->slot(s).set_processor(p);
                                        }
                                        if (ImGui::MenuItem("Liquid Vactrol Leveler (LA-2A Opto)")) {
                                            auto p = std::make_shared<dsp::LiquidVactrolProcessor>(kSampleRate);
                                            p->init(kSampleRate);
                                            p->set_parameter(0, 0.60f);
                                            p->set_parameter(1, 0.0f);
                                            p->set_parameter(2, 0.0f);
                                            p->set_parameter(3, 0.75f);
                                            p->set_parameter(4, 0.50f);
                                            sel_trk->slot(s).set_processor(p);
                                        }
                                        if (ImGui::MenuItem("Buchla 292 LPG (Vactrol Gate)")) {
                                            auto p = std::make_shared<dsp::LiquidVactrolProcessor>(kSampleRate);
                                            p->init(kSampleRate);
                                            p->set_parameter(0, 0.75f);
                                            p->set_parameter(1, 0.0f);
                                            p->set_parameter(2, 2.0f);
                                            p->set_parameter(6, 0.35f);
                                            sel_trk->slot(s).set_processor(p);
                                        }
                                        if (ImGui::MenuItem("Sovereign MultiHead ODE Compressor")) {
                                            auto p = std::make_shared<dsp::MultiHeadOdeProcessor>(kSampleRate, 4);
                                            p->init(kSampleRate);
                                            sel_trk->slot(s).set_processor(p);
                                        }
                                        ImGui::EndPopup();
                                    }
                                }
                            }
                            ImGui::EndChild();
                            ImGui::PopStyleColor();
                            ImGui::PopID();
                        }
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

                    // High-Resolution Waveform Display with Multi-Resolution Peak Mipmapping
                    ImVec2 wf_pos = ImGui::GetCursorScreenPos();
                    ImVec2 wf_size(ImGui::GetContentRegionAvail().x, 100);
                    float play_ratio = playhead_seconds / loop_length_seconds;
                    if (cur_clip && cur_clip->overview()) {
                        ui::DrawWaveformDisplay(ImGui::GetWindowDrawList(), wf_pos, wf_size,
                                               cur_clip->overview().get(), 0, 0, cur_clip->num_frames(),
                                               play_ratio, slice_points, active_slice);
                    } else {
                        ui::DrawWaveformDisplay(ImGui::GetWindowDrawList(), wf_pos, wf_size,
                                               sample_waveform.data(), sample_waveform.size(),
                                               play_ratio, slice_points, active_slice);
                    }
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
                        const char* algo_names[5] = {
                            "1: Vinyl Variclock",
                            "2: Vintage 12-Bit MPC",
                            "3: Rubberband WSOLA",
                            "4: Sovereign ODE Kinetic",
                            "5: DeRez Sampler (SP/Mirage)"
                        };
                        ImGui::SetNextItemWidth(panel_w * 0.60f);
                        ImGui::Combo("##Algo", &pitch_algo_mode, algo_names, 5);
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

                    ImGui::Spacing();
                    ImGui::Separator();

                    // Panel Vari-Speed: Continuous Vari-Speed Resampler, Beat-Sync & Tape Ballistics
                    ImGui::BeginChild("PanelVariSpeed", ImVec2(0, 115), true);
                    {
                        auto* cur_trk = (selected_track == 0) ? trk0 :
                                        (selected_track == 1) ? trk1 :
                                        (selected_track == 2) ? trk2 : trk3;

                        ImGui::TextColored(ImVec4(0.95f, 0.55f, 0.15f, 1.0f),
                                           "LIVE VARI-SPEED RESAMPLER & BEAT-SYNC ENGINE (ANALOG TAPE CAPSTAN MOTOR)");
                        ImGui::SameLine(0, 20);
                        if (cur_trk) {
                            auto m_state = cur_trk->streamer().motor_state();
                            const char* state_str = (m_state == sampling::TapeMotorState::Running) ? "RUNNING" :
                                                    (m_state == sampling::TapeMotorState::Stopping) ? "STOPPING (BRAKE)" :
                                                    (m_state == sampling::TapeMotorState::Stopped) ? "STOPPED" : "STARTING (TORQUE)";
                            ImVec4 state_col = (m_state == sampling::TapeMotorState::Running) ? ImVec4(0.2f, 0.85f, 0.3f, 1.0f) :
                                               (m_state == sampling::TapeMotorState::Stopping) ? ImVec4(0.9f, 0.4f, 0.1f, 1.0f) :
                                               (m_state == sampling::TapeMotorState::Stopped) ? ImVec4(0.85f, 0.2f, 0.2f, 1.0f) :
                                               ImVec4(0.2f, 0.6f, 0.9f, 1.0f);
                            ImGui::TextColored(state_col, "[MOTOR: %s | SPEED: %.2fx | PH: %.1f]",
                                               state_str, cur_trk->streamer().effective_playback_ratio(), cur_trk->clip_playhead_f());
                        }
                        ImGui::Separator();

                        if (cur_trk) {
                            // Row 1: Mode Combo, Pitch Slider, Speed Slider, Reverse Toggle
                            ImGui::Text("Mode:");
                            ImGui::SameLine();
                            const char* mode_names[6] = {
                                "Free (VariSpeed Repitch)",
                                "Beat-Sync Repitch (Tape Lock)",
                                "Transport Phase-Lock (Hard Sync)",
                                "Reverse Free",
                                "Beat-Sync WSOLA (Pitch Locked)",
                                "Pitch-Shift WSOLA (Tempo Locked)"
                            };
                            int cur_mode_idx = static_cast<int>(cur_trk->playback_mode());
                            ImGui::SetNextItemWidth(250);
                            if (ImGui::Combo("##TrkMode", &cur_mode_idx, mode_names, 6)) {
                                cur_trk->set_playback_mode(static_cast<sampling::PlaybackMode>(cur_mode_idx));
                            }

                            ImGui::SameLine(0, 15);
                            float p_st = cur_trk->pitch_semitones();
                            ImGui::SetNextItemWidth(160);
                            if (ImGui::SliderFloat("Pitch##TrkP", &p_st, -24.0f, 24.0f, "%.1f st")) {
                                cur_trk->set_pitch_semitones(p_st);
                            }

                            ImGui::SameLine(0, 15);
                            float s_ratio = cur_trk->speed_ratio();
                            ImGui::SetNextItemWidth(140);
                            if (ImGui::SliderFloat("Speed##TrkS", &s_ratio, 0.25f, 4.0f, "%.2fx")) {
                                cur_trk->set_speed_ratio(s_ratio);
                            }

                            ImGui::SameLine(0, 15);
                            bool rev = cur_trk->is_reverse();
                            if (rev) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.2f, 0.2f, 1.0f));
                            if (ImGui::Button(rev ? " REV [ON] " : " REV [OFF] ")) {
                                cur_trk->set_reverse(!rev);
                            }
                            if (rev) ImGui::PopStyleColor();

                            // Row 2: Capstan Inertia, Bar Length, Tape Stop & Tape Start
                            ImGui::Spacing();
                            float inertia = cur_trk->capstan_inertia_ms();
                            ImGui::SetNextItemWidth(180);
                            if (ImGui::SliderFloat("Capstan Inertia##TrkInertia", &inertia, 0.0f, 250.0f, "%.1f ms")) {
                                cur_trk->set_capstan_inertia_ms(inertia);
                            }

                            ImGui::SameLine(0, 20);
                            float bars = cur_trk->clip_bar_length();
                            ImGui::SetNextItemWidth(120);
                            if (ImGui::SliderFloat("Bars (Sync)##TrkBars", &bars, 0.0f, 16.0f, "%.0f bars")) {
                                cur_trk->set_clip_bar_length(bars);
                            }

                            ImGui::SameLine(0, 25);
                            if (ImGui::Button("  TAPE STOP (0.5s)  ")) {
                                cur_trk->trigger_tape_stop(0.5f);
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("  TAPE START (0.3s)  ")) {
                                cur_trk->trigger_tape_start(0.3f);
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("RESET##TrkVari")) {
                                cur_trk->set_pitch_semitones(0.0f);
                                cur_trk->set_speed_ratio(1.0f);
                                cur_trk->set_reverse(false);
                                cur_trk->set_capstan_inertia_ms(15.0f);
                                cur_trk->trigger_tape_start(0.05f);
                            }
                        }
                    }
                    ImGui::EndChild();

                    ImGui::Spacing();
                    ImGui::Separator();

                    // Panel E: Live SampleTap Recorder & Downbeat-Quantized Bouncer
                    ImGui::BeginChild("PanelSampleTap", ImVec2(0, 0), true);
                    {
                        ImGui::TextColored(ImVec4(0.20f, 0.70f, 0.85f, 1.0f),
                                           "SAMPLETAP LIVE RECORDER & DOWNBEAT-QUANTIZED BOUNCER (ZERO-CLICK SEAMLESS LOOPING)");
                        ImGui::Separator();

                        // Row 1: Source & Current Status
                        ImGui::Text("Tap Source:");
                        ImGui::SameLine();
                        const char* tap_source_names[8] = {
                            "Master Out (Full Mix)",
                            "Track 1 Post-FX (Kick / 808)",
                            "Track 2 Post-FX (Acid 303)",
                            "Track 3 Post-FX (Vocal)",
                            "Track 4 Post-FX (Drums)",
                            "Submix Bus 1 (Drum Glue Comp)",
                            "Track 1 Pre-FX (PipeWire / AoIP)",
                            "Track 2 Pre-FX (PipeWire / AoIP)"
                        };
                        ImGui::SetNextItemWidth(260);
                        if (ImGui::Combo("##TapSrc", &tap_source_idx, tap_source_names, 8)) {
                            update_tap_source(tap_source_idx);
                        }

                        ImGui::SameLine(0, 20);
                        if (tap0) {
                            auto t_state = tap0->record_state();
                            if (t_state == sampling::RecordState::Armed) {
                                ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.15f, 1.0f), "[ ARMED // WAITING FOR DOWNBEAT ]");
                                ImGui::SameLine();
                                if (ImGui::Button("CANCEL##Arm")) {
                                    tap0->dismiss_bounce_to_rolling();
                                }
                            } else if (t_state == sampling::RecordState::Recording) {
                                ImGui::TextColored(ImVec4(0.90f, 0.25f, 0.25f, 1.0f), "[ RECORDING QUANTIZED BOUNCE ]");
                                ImGui::SameLine();
                                ImGui::ProgressBar(tap0->progress(), ImVec2(160, 20));
                                ImGui::SameLine();
                                if (ImGui::Button("CANCEL##Rec")) {
                                    tap0->dismiss_bounce_to_rolling();
                                }
                            } else if (t_state == sampling::RecordState::Complete) {
                                auto b_clip = tap0->get_quantized_clip();
                                if (b_clip) {
                                    ImGui::TextColored(ImVec4(0.25f, 0.85f, 0.35f, 1.0f),
                                                       "[ BOUNCE READY: %s (%u frames, %.2fs) ]",
                                                       b_clip->name().c_str(), b_clip->num_frames(),
                                                       static_cast<float>(b_clip->num_frames()) / b_clip->sample_rate());
                                }
                            } else {
                                ImGui::TextColored(ImVec4(0.40f, 0.75f, 0.40f, 1.0f),
                                                   "[ ROLLING BUFFER ACTIVE (10s Ring Buffer) ]");
                            }
                        }

                        // Row 2: Quantized Downbeat Bounce Triggers
                        ImGui::Text("Quantized Downbeat Bounce (BarSync):");
                        ImGui::SameLine();
                        if (ImGui::Button("ARM 1 BAR")) {
                            if (tap0) {
                                std::string name = "Bounce_1Bar_Trk" + std::to_string(selected_track + 1);
                                tap0->arm_bar_bounce(mixer.clock(), 1, name, tap_auto_seamless);
                                std::snprintf(status_toast, sizeof(status_toast), "ARMED 1-BAR BOUNCE ON NEXT DOWNBEAT");
                            }
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("ARM 2 BARS")) {
                            if (tap0) {
                                std::string name = "Bounce_2Bars_Trk" + std::to_string(selected_track + 1);
                                tap0->arm_bar_bounce(mixer.clock(), 2, name, tap_auto_seamless);
                                std::snprintf(status_toast, sizeof(status_toast), "ARMED 2-BAR BOUNCE ON NEXT DOWNBEAT");
                            }
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("ARM 4 BARS")) {
                            if (tap0) {
                                std::string name = "Bounce_4Bars_Trk" + std::to_string(selected_track + 1);
                                tap0->arm_bar_bounce(mixer.clock(), 4, name, tap_auto_seamless);
                                std::snprintf(status_toast, sizeof(status_toast), "ARMED 4-BAR BOUNCE ON NEXT DOWNBEAT");
                            }
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("ARM 8 BARS")) {
                            if (tap0) {
                                std::string name = "Bounce_8Bars_Trk" + std::to_string(selected_track + 1);
                                tap0->arm_bar_bounce(mixer.clock(), 8, name, tap_auto_seamless);
                                std::snprintf(status_toast, sizeof(status_toast), "ARMED 8-BAR BOUNCE ON NEXT DOWNBEAT");
                            }
                        }
                        ImGui::SameLine(0, 15);
                        ImGui::Checkbox("Equal-Power Seamless Crossfade (128 samples)", &tap_auto_seamless);

                        // Row 3: Bounce Commit Actions (when Complete)
                        if (tap0 && tap0->record_state() == sampling::RecordState::Complete) {
                            auto b_clip = tap0->get_quantized_clip();
                            if (b_clip) {
                                ImGui::Separator();
                                ImGui::TextColored(ImVec4(0.85f, 0.5f, 0.1f, 1.0f), "Commit Bounce to Track / Storage:");
                                ImGui::SameLine();
                                if (ImGui::Button("  COMMIT TO CURRENT TRACK  ")) {
                                    track_clips_orig[selected_track] = b_clip;
                                    sync_track_clip(selected_track, b_clip);
                                    std::snprintf(status_toast, sizeof(status_toast),
                                                  "COMMITTED BOUNCE TO TRACK %d (%s)",
                                                  selected_track + 1, b_clip->name().c_str());
                                    tap0->dismiss_bounce_to_rolling();
                                }
                                ImGui::SameLine();
                                for (int tr = 0; tr < 4; ++tr) {
                                    char t_label[32];
                                    std::snprintf(t_label, sizeof(t_label), "-> Trk %d", tr + 1);
                                    if (ImGui::Button(t_label)) {
                                        track_clips_orig[tr] = b_clip;
                                        sync_track_clip(tr, b_clip);
                                        std::snprintf(status_toast, sizeof(status_toast),
                                                      "COMMITTED BOUNCE TO TRACK %d (%s)",
                                                      tr + 1, b_clip->name().c_str());
                                        tap0->dismiss_bounce_to_rolling();
                                    }
                                    ImGui::SameLine();
                                }
                                if (ImGui::Button("EXPORT 24-BIT WAV##Bounce")) {
                                    std::filesystem::create_directories("renders");
                                    std::string path = "renders/" + b_clip->name() + ".wav";
                                    if (b_clip->save_to_wav(path, 24)) {
                                        std::snprintf(status_toast, sizeof(status_toast),
                                                      "SAVED 24-BIT BOUNCE TO: %s", path.c_str());
                                    }
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("DISMISS##Bounce")) {
                                    tap0->dismiss_bounce_to_rolling();
                                }
                            }
                        }

                        // Row 4: Retroactive Jam Grab (Capture last N bars from rolling circular buffer)
                        ImGui::Separator();
                        ImGui::Text("Retroactive Jam Grab (Capture Last N Bars):");
                        ImGui::SameLine();
                        if (ImGui::Button("GRAB LAST 1 BAR")) {
                            if (tap0) {
                                uint32_t frames = static_cast<uint32_t>(std::round(mixer.clock().samples_for_bars(1)));
                                auto c = tap0->capture_retroactive(frames, "Retro_1Bar_Trk" + std::to_string(selected_track + 1), tap_auto_seamless);
                                if (c) {
                                    track_clips_orig[selected_track] = c;
                                    sync_track_clip(selected_track, c);
                                    std::snprintf(status_toast, sizeof(status_toast),
                                                  "RETROACTIVE CAPTURE: 1 BAR (%.2fs) COMMITTED TO TRACK %d",
                                                  static_cast<float>(frames) / mixer.sample_rate(), selected_track + 1);
                                }
                            }
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("GRAB LAST 2 BARS")) {
                            if (tap0) {
                                uint32_t frames = static_cast<uint32_t>(std::round(mixer.clock().samples_for_bars(2)));
                                auto c = tap0->capture_retroactive(frames, "Retro_2Bars_Trk" + std::to_string(selected_track + 1), tap_auto_seamless);
                                if (c) {
                                    track_clips_orig[selected_track] = c;
                                    sync_track_clip(selected_track, c);
                                    std::snprintf(status_toast, sizeof(status_toast),
                                                  "RETROACTIVE CAPTURE: 2 BARS (%.2fs) COMMITTED TO TRACK %d",
                                                  static_cast<float>(frames) / mixer.sample_rate(), selected_track + 1);
                                }
                            }
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("GRAB LAST 4 BARS")) {
                            if (tap0) {
                                uint32_t frames = static_cast<uint32_t>(std::round(mixer.clock().samples_for_bars(4)));
                                auto c = tap0->capture_retroactive(frames, "Retro_4Bars_Trk" + std::to_string(selected_track + 1), tap_auto_seamless);
                                if (c) {
                                    track_clips_orig[selected_track] = c;
                                    sync_track_clip(selected_track, c);
                                    std::snprintf(status_toast, sizeof(status_toast),
                                                  "RETROACTIVE CAPTURE: 4 BARS (%.2fs) COMMITTED TO TRACK %d",
                                                  static_cast<float>(frames) / mixer.sample_rate(), selected_track + 1);
                                }
                            }
                        }
                    }
                    ImGui::EndChild();

                    ImGui::EndTabItem();
                }

                // ------------------------------------------------------------
                // ------------------------------------------------------------
                // TAB 3: ENVELOPES & AUTOMATION
                // ------------------------------------------------------------
                if (ImGui::BeginTabItem("  ENVELOPES & AUTOMATION  ")) {
                    Track* auto_trk = (selected_track == 0) ? trk0 : ((selected_track == 1) ? trk1 : ((selected_track == 2) ? trk2 : trk3));

                    const double spb = std::max(1.0, mixer.clock().samples_per_beat());
                    const double cur_play_beat = is_playing ? std::fmod(static_cast<double>(mixer.clock().sample_position()) / spb, 16.0) : -1.0;

                    ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f),
                                       "Orderly Architect: FontLab-Inspired Track Automation Curve Editor");
                    ImGui::SameLine();
                    ImGui::TextDisabled("| C^1 Hermite Smooth Splines, Direct Curvature Tension Dots & Zero-Allocation RCU");
                    ImGui::Separator();

                    // Scope Selector Buttons
                    ImGui::Text("Editing Scope:");
                    ImGui::SameLine();
                    if (ImGui::RadioButton("Timeline Automation (Track-Wide)", automation_context_mode == 0)) {
                        automation_context_mode = 0;
                        selected_curve_point = -1;
                        selected_curve_points.clear();
                    }
                    ImGui::SameLine(0, 24);
                    if (ImGui::RadioButton("Clip Envelope (Loop-Relative)", automation_context_mode == 1)) {
                        automation_context_mode = 1;
                        selected_curve_point = -1;
                        selected_curve_points.clear();
                    }
                    ImGui::Separator();

                    // Track Selector Buttons
                    ImGui::Text("Target Channel:");
                    ImGui::SameLine();
                    const char* trk_short_names[4] = { "Track 1: Kick", "Track 2: Acid", "Track 3: Vocal", "Track 4: Drums" };
                    for (int t = 0; t < 4; ++t) {
                        if (t > 0) ImGui::SameLine();
                        bool is_sel = (selected_track == t);
                        if (is_sel) {
                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.38f, 0.85f, 0.9f));
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                        }
                        if (ImGui::Button(trk_short_names[t], ImVec2(120, 24))) {
                            selected_track = t;
                            selected_curve_point = -1;
                            selected_curve_points.clear();
                        }
                        if (is_sel) {
                            ImGui::PopStyleColor(2);
                        }
                    }

                    ImGui::SameLine(0, 28);
                    ImGui::Text("View Mode:");
                    ImGui::SameLine();
                    if (ImGui::RadioButton("Focused Single Lane", automation_view_mode == 0)) {
                        automation_view_mode = 0;
                    }
                    ImGui::SameLine(0, 16);
                    if (ImGui::RadioButton("Multi-Lane Stacked", automation_view_mode == 1)) {
                        automation_view_mode = 1;
                    }

                    ImGui::Spacing();

                    routing::AutomationCurve* editor_curve = nullptr;
                    routing::AutomationTarget editor_target = routing::AutomationTarget::Gain;
                    float editor_custom_min = 0.0f;
                    float editor_custom_max = 1.0f;
                    const char* editor_custom_unit = nullptr;
                    double editor_total_beats = 16.0;
                    double editor_play_beat = -1.0;
                    const char* editor_id = "TrackAutomationEditor";

                    if (automation_context_mode == 0) {
                        if (automation_view_mode == 1) {
                            // Scope 0, View 1: Multi-Lane Stacked View
                            ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "Stacked Automation Lanes (%zu Active):", stacked_lanes.size());
                            ImGui::SameLine(0, 16);
                            if (ImGui::Button("[ + ADD AUTOMATION LANE ]")) {
                                ImGui::OpenPopup("AddLanePopup");
                            }

                            if (ImGui::BeginPopup("AddLanePopup")) {
                                ImGui::TextDisabled("Select Parameter to Add:");
                                ImGui::Separator();
                                if (ImGui::Selectable("Track Gain")) {
                                    stacked_lanes.push_back({ LaneKind::TrackParam, routing::AutomationTarget::Gain, 0, 0, false });
                                }
                                if (ImGui::Selectable("Track Pan")) {
                                    stacked_lanes.push_back({ LaneKind::TrackParam, routing::AutomationTarget::Pan, 0, 0, false });
                                }
                                if (ImGui::Selectable("Track Aux 1 (Reverb)")) {
                                    stacked_lanes.push_back({ LaneKind::TrackParam, routing::AutomationTarget::Aux1, 0, 0, false });
                                }
                                if (ImGui::Selectable("Track Aux 2 (Delay)")) {
                                    stacked_lanes.push_back({ LaneKind::TrackParam, routing::AutomationTarget::Aux2, 0, 0, false });
                                }
                                for (size_t s = 0; s < 4; ++s) {
                                    auto* proc = auto_trk->slot(s).processor();
                                    if (proc) {
                                        ImGui::Separator();
                                        uint32_t pcount = std::min<uint32_t>(4, proc->parameter_count());
                                        for (uint32_t p = 0; p < pcount; ++p) {
                                            char item_name[128];
                                            std::snprintf(item_name, sizeof(item_name), "Slot %zu: %s -> %s", s + 1, proc->name(), proc->parameter_name(p));
                                            if (ImGui::Selectable(item_name)) {
                                                stacked_lanes.push_back({ LaneKind::PluginParam, routing::AutomationTarget::PluginParam, static_cast<uint32_t>(s), p, false });
                                            }
                                        }
                                    }
                                }
                                ImGui::EndPopup();
                            }

                            ImGui::SameLine();
                            if (ImGui::Button("Expand All")) {
                                for (auto& ln : stacked_lanes) ln.collapsed = false;
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("Collapse All")) {
                                for (auto& ln : stacked_lanes) ln.collapsed = true;
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("Reset to Default (Gain & Pan)")) {
                                stacked_lanes = {
                                    { LaneKind::TrackParam, routing::AutomationTarget::Gain, 0, 0, false },
                                    { LaneKind::TrackParam, routing::AutomationTarget::Pan, 0, 0, false }
                                };
                            }

                            ImGui::Separator();

                            int lane_to_remove = -1;
                            for (size_t i = 0; i < stacked_lanes.size(); ++i) {
                                auto& ln = stacked_lanes[i];
                                ImGui::PushID(static_cast<int>(i));

                                routing::AutomationCurve* cur_lane_curve = nullptr;
                                routing::AutomationTarget cur_lane_target = routing::AutomationTarget::Gain;
                                bool is_cur_lane_en = false;
                                float cur_min = 0.0f;
                                float cur_max = 1.0f;
                                const char* cur_unit = nullptr;
                                char lane_title[128];

                                if (ln.kind == LaneKind::TrackParam) {
                                    cur_lane_target = ln.track_target;
                                    cur_lane_curve = &auto_trk->automation_curve(cur_lane_target);
                                    is_cur_lane_en = auto_trk->is_automation_enabled(cur_lane_target);
                                    if (cur_lane_target == routing::AutomationTarget::Gain) std::snprintf(lane_title, sizeof(lane_title), "LANE %zu: TRACK GAIN", i + 1);
                                    else if (cur_lane_target == routing::AutomationTarget::Pan) std::snprintf(lane_title, sizeof(lane_title), "LANE %zu: TRACK PAN", i + 1);
                                    else if (cur_lane_target == routing::AutomationTarget::Aux1) std::snprintf(lane_title, sizeof(lane_title), "LANE %zu: AUX 1 (REVERB)", i + 1);
                                    else if (cur_lane_target == routing::AutomationTarget::Aux2) std::snprintf(lane_title, sizeof(lane_title), "LANE %zu: AUX 2 (DELAY)", i + 1);
                                    else std::snprintf(lane_title, sizeof(lane_title), "LANE %zu: TRACK PARAM", i + 1);
                                } else {
                                    cur_lane_target = routing::AutomationTarget::PluginParam;
                                    cur_lane_curve = &auto_trk->slot_automation_curve(ln.slot_idx, ln.param_idx);
                                    is_cur_lane_en = auto_trk->is_slot_automation_enabled(ln.slot_idx, ln.param_idx);
                                    auto* proc = auto_trk->slot(ln.slot_idx).processor();
                                    if (proc) {
                                        cur_min = proc->parameter_min(ln.param_idx);
                                        cur_max = proc->parameter_max(ln.param_idx);
                                        cur_unit = proc->parameter_name(ln.param_idx);
                                        std::snprintf(lane_title, sizeof(lane_title), "LANE %zu: SLOT %u [%s] -> %s",
                                                      i + 1, ln.slot_idx + 1, proc->name(), proc->parameter_name(ln.param_idx));
                                    } else {
                                        std::snprintf(lane_title, sizeof(lane_title), "LANE %zu: SLOT %u -> PARAM %u",
                                                      i + 1, ln.slot_idx + 1, ln.param_idx + 1);
                                    }
                                }

                                // Header Line
                                ImGui::TextColored(ImVec4(0.85f, 0.45f, 0.10f, 1.0f), "%s", lane_title);
                                ImGui::SameLine(0, 16);
                                if (is_cur_lane_en) {
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.16f, 0.62f, 0.30f, 1.0f));
                                    if (ImGui::Button("[ ENGAGED ]##btn", ImVec2(100, 20))) {
                                        if (ln.kind == LaneKind::TrackParam) auto_trk->set_automation_enabled(cur_lane_target, false);
                                        else auto_trk->set_slot_automation_enabled(ln.slot_idx, ln.param_idx, false);
                                    }
                                    ImGui::PopStyleColor();
                                } else {
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.32f, 0.35f, 0.40f, 0.8f));
                                    if (ImGui::Button("[ BYPASSED ]##btn", ImVec2(100, 20))) {
                                        if (ln.kind == LaneKind::TrackParam) auto_trk->set_automation_enabled(cur_lane_target, true);
                                        else auto_trk->set_slot_automation_enabled(ln.slot_idx, ln.param_idx, true);
                                    }
                                    ImGui::PopStyleColor();
                                }

                                ImGui::SameLine(0, 8);
                                if (ImGui::Button(ln.collapsed ? "[ + Expand ]" : "[ - Fold ]", ImVec2(90, 20))) {
                                    ln.collapsed = !ln.collapsed;
                                }

                                ImGui::SameLine(0, 8);
                                if (ImGui::Button("[ X ]", ImVec2(32, 20))) {
                                    lane_to_remove = static_cast<int>(i);
                                }

                                if (!ln.collapsed && cur_lane_curve) {
                                    char editor_uid[64];
                                    std::snprintf(editor_uid, sizeof(editor_uid), "StackedEditor_%zu", i);
                                    ui::DrawAutomationCurveEditor(editor_uid,
                                                                  *cur_lane_curve,
                                                                  ImVec2(0, 115),
                                                                  16.0,
                                                                  cur_play_beat,
                                                                  nullptr,
                                                                  cur_lane_target,
                                                                  nullptr,
                                                                  cur_min,
                                                                  cur_max,
                                                                  cur_unit,
                                                                  true);
                                }

                                ImGui::PopID();
                                ImGui::Spacing();
                            }

                            if (lane_to_remove >= 0 && lane_to_remove < static_cast<int>(stacked_lanes.size())) {
                                stacked_lanes.erase(stacked_lanes.begin() + lane_to_remove);
                            }
                        } else {
                            // Scope 0, View 0: Focused Single Lane View
                            routing::AutomationTarget current_target = (selected_auto_lane < 4)
                                ? static_cast<routing::AutomationTarget>(selected_auto_lane)
                                : routing::AutomationTarget::PluginParam;

                            routing::AutomationCurve* active_curve_ptr = nullptr;
                            bool is_auto_on = false;
                            float cur_pmin = 0.0f;
                            float cur_pmax = 1.0f;
                            const char* cur_punit = nullptr;

                            if (selected_auto_lane < 4) {
                                active_curve_ptr = &auto_trk->automation_curve(current_target);
                                is_auto_on = auto_trk->is_automation_enabled(current_target);
                            } else {
                                size_t s_idx = (selected_auto_lane - 4) / 4;
                                size_t p_idx = (selected_auto_lane - 4) % 4;
                                active_curve_ptr = &auto_trk->slot_automation_curve(s_idx, p_idx);
                                is_auto_on = auto_trk->is_slot_automation_enabled(s_idx, p_idx);
                                auto* proc = auto_trk->slot(s_idx).processor();
                                if (proc) {
                                    cur_pmin = proc->parameter_min(p_idx);
                                    cur_pmax = proc->parameter_max(p_idx);
                                    cur_punit = proc->parameter_name(p_idx);
                                }
                            }
                            routing::AutomationCurve& active_curve = *active_curve_ptr;

                            ImGui::Text("Track Parameters:");
                            ImGui::SameLine();
                            const char* lane_names[4] = { "GAIN", "PAN", "AUX 1 (REVERB)", "AUX 2 (DELAY)" };
                            for (int l = 0; l < 4; ++l) {
                                if (l > 0) ImGui::SameLine();
                                bool is_sel = (selected_auto_lane == l);
                                if (is_sel) {
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.45f, 0.10f, 0.9f));
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                }
                                if (ImGui::Button(lane_names[l], ImVec2(130, 24))) {
                                    selected_auto_lane = l;
                                    selected_curve_point = -1;
                                    selected_curve_points.clear();
                                }
                                if (is_sel) {
                                    ImGui::PopStyleColor(2);
                                }
                            }

                            // Plugin Insert Slot Parameters
                            bool has_plugins = false;
                            for (size_t s = 0; s < 4; ++s) {
                                if (auto_trk->slot(s).processor()) { has_plugins = true; break; }
                            }
                            if (has_plugins) {
                                ImGui::Spacing();
                                ImGui::Text("Plugin Insert Slot Parameters:");
                                for (size_t s = 0; s < 4; ++s) {
                                    auto* proc = auto_trk->slot(s).processor();
                                    if (!proc) continue;
                                    ImGui::TextDisabled("Slot %zu [%s]:", s + 1, proc->name());
                                    ImGui::SameLine();
                                    uint32_t pcount = std::min<uint32_t>(4, proc->parameter_count());
                                    for (uint32_t p = 0; p < pcount; ++p) {
                                        if (p > 0) ImGui::SameLine();
                                        int p_id = 4 + static_cast<int>(s * 4 + p);
                                        bool is_sel = (selected_auto_lane == p_id);
                                        if (is_sel) {
                                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.45f, 0.10f, 0.9f));
                                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                        }
                                        char pbtn_lbl[64];
                                        std::snprintf(pbtn_lbl, sizeof(pbtn_lbl), "%s##s%zup%u", proc->parameter_name(p), s, p);
                                        if (ImGui::Button(pbtn_lbl)) {
                                            selected_auto_lane = p_id;
                                            selected_curve_point = -1;
                                            selected_curve_points.clear();
                                        }
                                        if (is_sel) {
                                            ImGui::PopStyleColor(2);
                                        }
                                    }
                                }
                            }

                            ImGui::Spacing();
                            ImGui::Text("Automation Control:");
                            ImGui::SameLine();
                            if (is_auto_on) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.16f, 0.62f, 0.30f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.20f, 0.72f, 0.35f, 1.0f));
                                if (ImGui::Button("[ AUTOMATION ENGAGED ]", ImVec2(180, 24))) {
                                    if (selected_auto_lane < 4) auto_trk->set_automation_enabled(current_target, false);
                                    else auto_trk->set_slot_automation_enabled((selected_auto_lane - 4) / 4, (selected_auto_lane - 4) % 4, false);
                                }
                                ImGui::PopStyleColor(2);
                            } else {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.32f, 0.35f, 0.40f, 0.8f));
                                if (ImGui::Button("[ AUTOMATION BYPASSED ]", ImVec2(180, 24))) {
                                    if (selected_auto_lane < 4) auto_trk->set_automation_enabled(current_target, true);
                                    else auto_trk->set_slot_automation_enabled((selected_auto_lane - 4) / 4, (selected_auto_lane - 4) % 4, true);
                                }
                                ImGui::PopStyleColor(1);
                            }

                            ImGui::SameLine(0, 16);
                            ImGui::TextDisabled("Presets:");
                            ImGui::SameLine();
                            if (current_target == routing::AutomationTarget::Gain) {
                                if (ImGui::Button("Fade In")) {
                                    active_curve.preset_fade_in(0.0, 16.0);
                                    auto_trk->set_automation_enabled(current_target, true);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Fade Out")) {
                                    active_curve.preset_fade_out(0.0, 16.0);
                                    auto_trk->set_automation_enabled(current_target, true);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("4-Beat Pump")) {
                                    active_curve.preset_sidechain_pump(16.0);
                                    auto_trk->set_automation_enabled(current_target, true);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Reset 0 dB")) {
                                    active_curve.preset_reset_unity(16.0);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Clear")) {
                                    active_curve.clear(1.0f);
                                }
                            } else if (current_target == routing::AutomationTarget::Pan) {
                                if (ImGui::Button("Auto-Pan Sine")) {
                                    active_curve.preset_sine_pan(16.0, 4.0);
                                    auto_trk->set_automation_enabled(current_target, true);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Reset Center")) {
                                    active_curve.clear(0.0f);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Clear")) {
                                    active_curve.clear(0.0f);
                                }
                            } else if (current_target == routing::AutomationTarget::Aux1 || current_target == routing::AutomationTarget::Aux2) {
                                if (ImGui::Button("Reverb/FX Swell")) {
                                    active_curve.preset_reverb_swell(12.0, 16.0, 0.80f);
                                    auto_trk->set_automation_enabled(current_target, true);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Delay Throw")) {
                                    active_curve.preset_delay_throw(4, 0.75f);
                                    auto_trk->set_automation_enabled(current_target, true);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Reset Off")) {
                                    active_curve.clear(0.0f);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Clear")) {
                                    active_curve.clear(0.0f);
                                }
                            } else {
                                // PluginParam presets
                                size_t s_idx = (selected_auto_lane - 4) / 4;
                                size_t p_idx = (selected_auto_lane - 4) % 4;
                                auto* proc = auto_trk->slot(s_idx).processor();
                                float def_v = proc ? proc->parameter_default(p_idx) : 0.5f;
                                if (ImGui::Button("Ramp Up")) {
                                    active_curve.clear(cur_pmin);
                                    active_curve.add_point(0.0, cur_pmin, routing::NodeMode::Smooth, 0.0f);
                                    active_curve.add_point(16.0, cur_pmax, routing::NodeMode::Smooth, 0.0f);
                                    auto_trk->set_slot_automation_enabled(s_idx, p_idx, true);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Ramp Down")) {
                                    active_curve.clear(cur_pmax);
                                    active_curve.add_point(0.0, cur_pmax, routing::NodeMode::Smooth, 0.0f);
                                    active_curve.add_point(16.0, cur_pmin, routing::NodeMode::Smooth, 0.0f);
                                    auto_trk->set_slot_automation_enabled(s_idx, p_idx, true);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Reset Default")) {
                                    active_curve.clear(def_v);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Clear (Min)")) {
                                    active_curve.clear(cur_pmin);
                                }
                            }

                            ImGui::Spacing();
                            ImGui::TextColored(ImVec4(0.95f, 0.65f, 0.15f, 1.0f), "MSEG Spline Bridge:");
                            ImGui::SameLine();
                            if (ImGui::Button("[ ⤓ Stamp MSEG 1 (Voice Env) ]")) {
                                routing::MsegAutomationBridge::bake_to_curve(
                                    mod_matrix.mseg1(), active_curve, 0.0, 16.0, 4,
                                    cur_pmin, cur_pmax, routing::MsegAutomationBridge::FitMode::FitDuration,
                                    true, mixer.clock().bpm());
                                if (selected_auto_lane < 4) auto_trk->set_automation_enabled(current_target, true);
                                else auto_trk->set_slot_automation_enabled((selected_auto_lane - 4) / 4, (selected_auto_lane - 4) % 4, true);
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("[ ⤓ Stamp MSEG 2 (Mod Env) ]")) {
                                routing::MsegAutomationBridge::bake_to_curve(
                                    mod_matrix.mseg2(), active_curve, 0.0, 16.0, 4,
                                    cur_pmin, cur_pmax, routing::MsegAutomationBridge::FitMode::FitDuration,
                                    true, mixer.clock().bpm());
                                if (selected_auto_lane < 4) auto_trk->set_automation_enabled(current_target, true);
                                else auto_trk->set_slot_automation_enabled((selected_auto_lane - 4) / 4, (selected_auto_lane - 4) % 4, true);
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("[ ⤒ Extract Lane to MSEG 2 ]")) {
                                routing::MsegAutomationBridge::extract_to_mseg(
                                    active_curve, mod_matrix.mseg2(), 0.0, 16.0, modulation::MsegTimeMode::BeatSync,
                                    cur_pmin, cur_pmax, mixer.clock().bpm());
                            }

                            editor_curve = &active_curve;
                            editor_target = current_target;
                            editor_custom_min = cur_pmin;
                            editor_custom_max = cur_pmax;
                            editor_custom_unit = cur_punit;
                            editor_total_beats = 16.0;
                            editor_play_beat = cur_play_beat;
                            editor_id = "TrackAutomationEditor";
                        }
                    } else {
                        // Scope 1: Clip-Relative Loop Envelope
                        auto cur_clip = auto_trk ? auto_trk->clip() : nullptr;
                        if (!cur_clip && track_clips[selected_track]) {
                            cur_clip = track_clips[selected_track];
                        }

                        if (!cur_clip) {
                            ImGui::Spacing();
                            ImGui::TextColored(ImVec4(0.85f, 0.45f, 0.10f, 1.0f),
                                               "No AudioClip assigned to Track %d.", selected_track + 1);
                            ImGui::TextDisabled("Load an audio sample in TAB 2 (SAMPLE SLICING & RESAMPLING) or click below to assign default clip.");
                            if (ImGui::Button("ASSIGN TRACK DEFAULT CLIP")) {
                                sync_track_clip(selected_track, track_clips[selected_track]);
                            }
                        } else {
                            ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "Clip: \"%s\"", cur_clip->name().c_str());
                            const float dur_s = static_cast<float>(cur_clip->num_frames()) / static_cast<float>(std::max(1u, cur_clip->sample_rate()));
                            ImGui::TextDisabled("| Length: %.2f s (%.2f Beats) | Rate: %u Hz | %u Frames",
                                                dur_s, cur_clip->envelope_length_beats(),
                                                cur_clip->sample_rate(), cur_clip->num_frames());

                            ImGui::Spacing();

                            sampling::ClipEnvelopeTarget clip_tgt = static_cast<sampling::ClipEnvelopeTarget>(selected_clip_lane);
                            routing::AutomationTarget auto_tgt = (selected_clip_lane == 0) ? routing::AutomationTarget::Gain :
                                                                 ((selected_clip_lane == 1) ? routing::AutomationTarget::Pan : routing::AutomationTarget::Pitch);
                            routing::AutomationCurve& clip_curve = cur_clip->envelope(clip_tgt);

                            ImGui::Text("Envelope Lane:");
                            ImGui::SameLine();
                            const char* clip_lane_names[3] = { "CLIP GAIN", "CLIP PAN", "CLIP PITCH" };
                            for (int l = 0; l < 3; ++l) {
                                if (l > 0) ImGui::SameLine();
                                bool is_sel = (selected_clip_lane == l);
                                if (is_sel) {
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.45f, 0.10f, 0.9f));
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                }
                                if (ImGui::Button(clip_lane_names[l], ImVec2(130, 24))) {
                                    selected_clip_lane = l;
                                    selected_curve_point = -1;
                                    selected_curve_points.clear();
                                }
                                if (is_sel) {
                                    ImGui::PopStyleColor(2);
                                }
                            }

                            ImGui::SameLine(0, 16);
                            bool is_env_on = cur_clip->is_envelope_enabled(clip_tgt);
                            if (is_env_on) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.16f, 0.62f, 0.30f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.20f, 0.72f, 0.35f, 1.0f));
                                if (ImGui::Button("[ ENVELOPE ENGAGED ]", ImVec2(180, 24))) {
                                    cur_clip->set_envelope_enabled(clip_tgt, false);
                                }
                                ImGui::PopStyleColor(2);
                            } else {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.32f, 0.35f, 0.40f, 0.8f));
                                if (ImGui::Button("[ ENVELOPE BYPASSED ]", ImVec2(180, 24))) {
                                    cur_clip->set_envelope_enabled(clip_tgt, true);
                                }
                                ImGui::PopStyleColor(1);
                            }

                            ImGui::SameLine(0, 16);
                            ImGui::TextDisabled("Presets:");
                            ImGui::SameLine();
                            const double env_beats = cur_clip->envelope_length_beats();
                            if (clip_tgt == sampling::ClipEnvelopeTarget::Gain) {
                                if (ImGui::Button("4-Beat Pump")) {
                                    clip_curve.preset_clip_sidechain_pump(env_beats);
                                    cur_clip->set_envelope_enabled(clip_tgt, true);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("16th Gate")) {
                                    clip_curve.preset_clip_gate_trance(env_beats);
                                    cur_clip->set_envelope_enabled(clip_tgt, true);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Fade In/Out")) {
                                    clip_curve.preset_clip_fade_in_out(env_beats);
                                    cur_clip->set_envelope_enabled(clip_tgt, true);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Reset Unity")) {
                                    clip_curve.preset_reset_unity(env_beats);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Clear")) {
                                    clip_curve.clear(1.0f);
                                }
                            } else if (clip_tgt == sampling::ClipEnvelopeTarget::Pan) {
                                if (ImGui::Button("Ping-Pong")) {
                                    clip_curve.preset_clip_ping_pong_pan(env_beats, 1.0);
                                    cur_clip->set_envelope_enabled(clip_tgt, true);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Sine Pan")) {
                                    clip_curve.preset_sine_pan(env_beats, 2.0);
                                    cur_clip->set_envelope_enabled(clip_tgt, true);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Reset Center")) {
                                    clip_curve.clear(0.0f);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Clear")) {
                                    clip_curve.clear(0.0f);
                                }
                            } else { // Pitch
                                if (ImGui::Button("Octave Drop (-12st)")) {
                                    clip_curve.preset_clip_pitch_drop(env_beats, -12.0f);
                                    cur_clip->set_envelope_enabled(clip_tgt, true);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Double Drop (-24st)")) {
                                    clip_curve.preset_clip_pitch_drop(env_beats, -24.0f);
                                    cur_clip->set_envelope_enabled(clip_tgt, true);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Riser (+12st)")) {
                                    clip_curve.clear(0.0f);
                                    clip_curve.add_point(0.0, 0.0f, routing::NodeMode::Smooth, 0.0f);
                                    clip_curve.add_point(env_beats, 12.0f, routing::NodeMode::Smooth, 0.0f);
                                    cur_clip->set_envelope_enabled(clip_tgt, true);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Reset 0 st")) {
                                    clip_curve.clear(0.0f);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Clear")) {
                                    clip_curve.clear(0.0f);
                                }
                            }

                            editor_curve = &clip_curve;
                            editor_target = auto_tgt;
                            editor_total_beats = env_beats;
                            editor_play_beat = is_playing ? cur_clip->frame_to_envelope_beat(auto_trk->clip_playhead_f()) : -1.0;
                            editor_id = "ClipEnvelopeEditor";
                        }
                    }

                    if (editor_curve != nullptr) {
                        ImGui::Spacing();

                        // Canvas
                        ui::DrawAutomationCurveEditor(editor_id,
                                                      *editor_curve,
                                                      ImVec2(0, 200),
                                                      editor_total_beats,
                                                      editor_play_beat,
                                                      &selected_curve_point,
                                                      editor_target,
                                                      &selected_curve_points,
                                                      editor_custom_min,
                                                      editor_custom_max,
                                                      editor_custom_unit);

                        // Curve Inspector & Ergonomics Legend
                        auto pts = editor_curve->get_points();
                        ImGui::Spacing();
                        ImGui::TextColored(ImVec4(0.40f, 0.45f, 0.52f, 1.0f),
                            "Ergonomics: Click Canvas = Marquee Box | Shift+Click = Multi-select | Ctrl+A = Select All | Esc = Clear Selection");
                        ImGui::TextColored(ImVec4(0.40f, 0.45f, 0.52f, 1.0f),
                            "            Drag Handles = Time-Stretch / Scale Y | Drag Nodes = Move (1/16th Snap; Shift=Free) | Arrow Keys = Nudge (Alt=Fine)");
                        ImGui::TextColored(ImVec4(0.40f, 0.45f, 0.52f, 1.0f),
                            "            Click Curve = Split & Add Node | Drag Tension Dot = Curvature tau | Double-Click = Mode | Del = Remove | Ctrl+D = Dup");

                        // Multi-Node Batch Action Bar
                        if (selected_curve_points.size() > 1) {
                            ImGui::Spacing();
                            ImGui::TextColored(ImVec4(0.85f, 0.45f, 0.10f, 1.0f), "Batch Selection (%zu Nodes):", selected_curve_points.size());
                            ImGui::SameLine();
                            if (ImGui::Button("Smooth All (S)")) {
                                editor_curve->set_nodes_mode(selected_curve_points, routing::NodeMode::Smooth);
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("Corner All (C)")) {
                                editor_curve->set_nodes_mode(selected_curve_points, routing::NodeMode::Corner);
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("Hold All (H)")) {
                                editor_curve->set_nodes_mode(selected_curve_points, routing::NodeMode::Hold);
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("Invert Y")) {
                                float mid_v = 0.5f;
                                float min_v = 0.0f;
                                float max_v = 1.25f;
                                if (editor_target == routing::AutomationTarget::Pan) {
                                    mid_v = 0.0f; min_v = -1.0f; max_v = 1.0f;
                                } else if (editor_target == routing::AutomationTarget::Pitch) {
                                    mid_v = 0.0f; min_v = -24.0f; max_v = 24.0f;
                                } else if (editor_target == routing::AutomationTarget::PluginParam) {
                                    min_v = editor_custom_min; max_v = editor_custom_max; mid_v = (min_v + max_v) * 0.5f;
                                }
                                editor_curve->invert_points_value(selected_curve_points, mid_v, min_v, max_v);
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("Stretch 2x")) {
                                double min_t = 1e9;
                                for (auto idx : selected_curve_points) {
                                    if (idx < pts.size()) min_t = std::min(min_t, pts[idx].time_beats);
                                }
                                editor_curve->scale_points_time(selected_curve_points, min_t, 2.0);
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("Compress 0.5x")) {
                                double min_t = 1e9;
                                for (auto idx : selected_curve_points) {
                                    if (idx < pts.size()) min_t = std::min(min_t, pts[idx].time_beats);
                                }
                                editor_curve->scale_points_time(selected_curve_points, min_t, 0.5);
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("Duplicate (Ctrl+D)")) {
                                selected_curve_points = editor_curve->duplicate_points(selected_curve_points, 1.0);
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("Delete (Del)")) {
                                editor_curve->remove_points(selected_curve_points);
                                selected_curve_points.clear();
                                selected_curve_point = -1;
                            }
                        } else if (selected_curve_point >= 0 && selected_curve_point < static_cast<int>(pts.size())) {
                            auto pt = pts[selected_curve_point];
                            ImGui::Spacing();
                            ImGui::Text("Node #%d Selected:", selected_curve_point + 1);
                            ImGui::SameLine();
                            if (editor_target == routing::AutomationTarget::Pan) {
                                if (std::abs(pt.value) < 0.01f) {
                                    ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "Beat: %.2f | Center [0.00]", pt.time_beats);
                                } else if (pt.value < 0.0f) {
                                    ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "Beat: %.2f | Left %.0f%% (%.2f)", pt.time_beats, -pt.value * 100.0f, pt.value);
                                } else {
                                    ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "Beat: %.2f | Right %.0f%% (+%.2f)", pt.time_beats, pt.value * 100.0f, pt.value);
                                }
                            } else if (editor_target == routing::AutomationTarget::Pitch) {
                                ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "Beat: %.2f | Pitch: %+.1f st (%.2f st)", pt.time_beats, pt.value, pt.value);
                            } else if (editor_target == routing::AutomationTarget::Aux1 || editor_target == routing::AutomationTarget::Aux2) {
                                float db_val = (pt.value > 1e-4f) ? 20.0f * std::log10(pt.value) : -96.0f;
                                ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "Beat: %.2f | Send: %.0f%% (%.1f dB)",
                                                   pt.time_beats, pt.value * 100.0f, db_val);
                            } else if (editor_target == routing::AutomationTarget::PluginParam) {
                                if (editor_custom_unit) {
                                    ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "Beat: %.2f | Value: %.2f %s", pt.time_beats, pt.value, editor_custom_unit);
                                } else {
                                    ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "Beat: %.2f | Value: %.2f", pt.time_beats, pt.value);
                                }
                            } else {
                                float db_val = (pt.value > 1e-4f) ? 20.0f * std::log10(pt.value) : -96.0f;
                                ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "Beat: %.2f | Linear: %.3f (%+.1f dB)",
                                                   pt.time_beats, pt.value, db_val);
                            }
                            ImGui::SameLine(0, 16);
                            const char* m_str = (pt.node_mode == routing::NodeMode::Smooth) ? "Mode: Smooth (Circle)" :
                                                (pt.node_mode == routing::NodeMode::Corner) ? "Mode: Corner (Diamond)" : "Mode: Hold (Box)";
                            if (ImGui::Button(m_str)) {
                                editor_curve->toggle_node_mode(selected_curve_point);
                            }
                            if (selected_curve_point < static_cast<int>(pts.size()) - 1) {
                                ImGui::SameLine(0, 16);
                                ImGui::SetNextItemWidth(140);
                                float cur_t = pt.tension;
                                if (ImGui::SliderFloat("Tension", &cur_t, -1.0f, 1.0f, "%.2f")) {
                                    editor_curve->set_segment_tension(selected_curve_point, cur_t);
                                }
                            }
                        }
                    }

                    ImGui::Spacing();
                    ImGui::Separator();
                    ImGui::Spacing();

                    // Liquid ODE Envelope Section
                    ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f),
                                       "Liquid ODE Trapezoidal Modulation Envelope (A-Stable C^inf)");
                    ImGui::Spacing();

                    ImVec2 env_pos = ImGui::GetCursorScreenPos();
                    ImVec2 env_size(ImGui::GetContentRegionAvail().x - 260.0f, 130.0f);
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
                        ImGui::SliderFloat("ODE Tau", &env_tau, 0.1f, 5.0f, "%.2f");
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

                // ------------------------------------------------------------
                // TAB 5: MODULATOR LAB // MSEG & LFO MATRIX
                // ------------------------------------------------------------
                if (ImGui::BeginTabItem("  MODULATOR LAB // MSEG & LFO MATRIX  ")) {
                    ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f),
                                       "Orderly Architect: Multi-Stage Envelope (MSEG) & Meta-Modulation Engine");
                    ImGui::SameLine();
                    ImGui::TextDisabled("| Bitwig Dragless Modulation Rings • Buchla/Maths Function Generator • Zero-Allocation RCU");
                    ImGui::Separator();

                    // Hardware MIDI & Automatic Poly Track Routing Status Banner
                    {
                        bool is_conn = midi_rx.is_connected();
                        bool is_mock = midi_rx.is_mock();
                        bool has_act = midi_rx.has_activity_and_clear();
                        static float midi_activity_timer = 0.0f;
                        if (has_act) midi_activity_timer = 0.30f;
                        if (midi_activity_timer > 0.0f) midi_activity_timer -= dt;

                        ImVec4 port_col = is_conn ? (is_mock ? ImVec4(0.90f, 0.75f, 0.20f, 1.0f) : ImVec4(0.20f, 0.90f, 0.40f, 1.0f))
                                                  : ImVec4(0.80f, 0.25f, 0.25f, 1.0f);
                        ImGui::TextColored(port_col, is_conn ? (is_mock ? "[ ● VIRTUAL/MOCK MIDI ]" : "[ ● ALSA USB MIDI ]") : "[ ○ NO MIDI PORT ]");
                        ImGui::SameLine();
                        ImGui::TextDisabled("Interface: %s", midi_rx.current_device_path().c_str());

                        ImGui::SameLine();
                        ImVec4 led_col = (midi_activity_timer > 0.0f) ? ImVec4(0.20f, 1.0f, 0.40f, 1.0f) : ImVec4(0.30f, 0.30f, 0.30f, 1.0f);
                        ImGui::TextColored(led_col, " [RX LED]");

                        ImGui::SameLine();
                        uint8_t st = midi_rx.last_status();
                        uint8_t d1 = midi_rx.last_note();
                        uint8_t d2 = midi_rx.last_velocity();
                        if (st != 0) {
                            ImGui::TextDisabled("| Last Event: 0x%02X (d1:%u, d2:%u)", st, d1, d2);
                        } else {
                            ImGui::TextDisabled("| Last Event: Idle");
                        }

                        ImGui::SameLine();
                        ImGui::TextColored(ImVec4(0.25f, 0.70f, 1.0f, 1.0f), "➔ Auto-Routed: Track 2 (\"Poly Synth / Lead\") -> Bus B (Music)");

                        ImGui::SameLine();
                        size_t n_subs = midi_rx.subscription_count(false);
                        ImGui::TextColored(ImVec4(0.35f, 0.85f, 1.0f, 1.0f), "[Subs: %zu]", n_subs);
                        if (ImGui::IsItemHovered()) {
                            auto subs = midi_rx.active_subscriptions();
                            ImGui::BeginTooltip();
                            ImGui::TextUnformatted("Active ALSA Sequencer Subscriptions:");
                            if (subs.empty()) {
                                ImGui::TextDisabled("No active subscriptions");
                            } else {
                                for (const auto& s : subs) {
                                    if (s.is_system_announce) {
                                        ImGui::TextDisabled("• [%d:%d] Kernel Hotplug Announce", s.client_id, s.port_id);
                                    } else {
                                        ImGui::Text("• [%d:%d] %s - %s", s.client_id, s.port_id, s.client_name.c_str(), s.port_name.c_str());
                                    }
                                }
                            }
                            ImGui::EndTooltip();
                        }

                        ImGui::SameLine();
                        if (ImGui::SmallButton(" ⟳ Auto-Subscribe ")) {
                            midi_rx.auto_subscribe_all(true);
                        }

                        ImGui::SameLine();
                        if (ImGui::SmallButton(" ⟳ Re-probe MIDI ")) {
                            midi_rx.auto_connect();
                        }

                        // MIDI Sync Telemetry & Clock Authority Banner
                        const auto& tracker = midi_rx.sync_tracker();
                        auto sync_tel = tracker.telemetry(4);
                        auto mtc_tc = tracker.mtc_timecode();
                        auto auth = mixer.clock().authority();

                        ImGui::Spacing();
                        ImGui::Text("Sync Authority:");
                        ImGui::SameLine();
                        const char* auth_names[] = { "Master (Internal)", "Ableton Link Follower", "Isolated", "MIDI Clock Slave (24 PPQN)", "MTC Slave (SMPTE Timecode)" };
                        int current_auth_idx = static_cast<int>(auth);
                        ImGui::SetNextItemWidth(210);
                        if (ImGui::Combo("##SyncAuthCombo", &current_auth_idx, auth_names, IM_ARRAYSIZE(auth_names))) {
                            mixer.clock().set_authority(static_cast<clock::ClockAuthority>(current_auth_idx));
                        }

                        ImGui::SameLine();
                        if (sync_tel.is_locked) {
                            ImGui::TextColored(ImVec4(0.20f, 1.0f, 0.40f, 1.0f), "[ ● MIDI CLOCK LOCKED: %.1f BPM ]", sync_tel.estimated_bpm);
                        } else {
                            ImGui::TextColored(ImVec4(0.50f, 0.50f, 0.50f, 1.0f), "[ ○ MIDI CLOCK: NO LOCK ]");
                        }

                        ImGui::SameLine();
                        ImGui::TextDisabled("| Ticks: %llu | SPP: %u (Bar %u.%u) | Jitter: %.2f ms",
                            static_cast<unsigned long long>(sync_tel.tick_count),
                            sync_tel.song_position_spp,
                            sync_tel.bar_index + 1,
                            sync_tel.beat_within_bar + 1,
                            sync_tel.jitter_ms);

                        ImGui::SameLine();
                        if (mtc_tc.is_valid) {
                            ImGui::TextColored(ImVec4(0.20f, 0.90f, 1.0f, 1.0f), "| [ MTC: %s ]", mtc_tc.to_string().c_str());
                        } else {
                            ImGui::TextDisabled("| [ MTC: Offline ]");
                        }

                        // Master Clock Output Transmission Bar (Visible when Master authority is active)
                        if (auth == clock::ClockAuthority::Master) {
                            ImGui::Spacing();
                            ImGui::TextColored(ImVec4(0.20f, 0.85f, 0.45f, 1.0f), "Master Output TX:");
                            ImGui::SameLine();
                            bool tx_beat = midi_rx.clock_generator().is_beat_clock_enabled();
                            if (ImGui::Checkbox("Beat Clock (24 PPQN)", &tx_beat)) {
                                midi_rx.clock_generator().set_beat_clock_enabled(tx_beat);
                            }
                            ImGui::SameLine();
                            bool tx_mtc = midi_rx.clock_generator().is_mtc_enabled();
                            if (ImGui::Checkbox("MTC (SMPTE Timecode)", &tx_mtc)) {
                                midi_rx.clock_generator().set_mtc_enabled(tx_mtc);
                            }
                            ImGui::SameLine();
                            const char* fps_labels[] = { "24 fps (Film)", "25 fps (PAL)", "29.97 df (NTSC)", "30 fps (HD)" };
                            int fps_idx = static_cast<int>(midi_rx.clock_generator().mtc_framerate());
                            ImGui::SetNextItemWidth(140);
                            if (ImGui::Combo("##MtcFpsCombo", &fps_idx, fps_labels, 4)) {
                                midi_rx.clock_generator().set_mtc_framerate(static_cast<midi::MtcFrameRate>(fps_idx));
                            }
                            ImGui::SameLine();
                            ImGui::TextDisabled("| Out: %llu Clocks | %llu QFrames | %llu SPP | %llu SysEx FF",
                                static_cast<unsigned long long>(midi_rx.clock_generator().tick_count()),
                                static_cast<unsigned long long>(midi_rx.clock_generator().qframe_count()),
                                static_cast<unsigned long long>(midi_rx.spp_tx_count()),
                                static_cast<unsigned long long>(midi_rx.mtc_full_frame_tx_count()));
                            ImGui::SameLine();
                            if (ImGui::SmallButton(" ⤓ Locate Full Frame SysEx ")) {
                                midi_rx.broadcast_seek_position(mixer.clock());
                            }

                        }
                    }

                    // Dynamic MIDI Learn & Hardware CC Mapping Table
                    {
                        static bool show_midi_learn_table = true;
                        ImGui::Spacing();
                        ImGui::TextColored(ImVec4(0.20f, 0.85f, 0.95f, 1.0f), "MIDI Learn & CC Mappings:");
                        ImGui::SameLine();
                        if (ImGui::SmallButton(show_midi_learn_table ? " [-] Hide Table " : " [+] Show Table ")) {
                            show_midi_learn_table = !show_midi_learn_table;
                        }
                        ImGui::SameLine();
                        if (ImGui::SmallButton(" + Learn Master Vol ")) {
                            midi_learn.arm_learn(midi::MidiLearnTargetType::MasterVolume, 0, 0, 0, 0.0f, 1.25f, "Master Volume");
                        }
                        ImGui::SameLine();
                        if (ImGui::SmallButton(" + Learn Trk 1 Gain ")) {
                            midi_learn.arm_learn(midi::MidiLearnTargetType::TrackGain, 0, 0, 0, 0.0f, 1.25f, "Track 1 Gain");
                        }
                        ImGui::SameLine();
                        if (ImGui::SmallButton(" + Learn Trk 2 Pan ")) {
                            midi_learn.arm_learn(midi::MidiLearnTargetType::TrackPan, 1, 0, 0, -1.0f, 1.0f, "Track 2 Pan");
                        }
                        ImGui::SameLine();
                        if (ImGui::SmallButton(" + Learn Synth Cutoff ")) {
                            midi_learn.arm_learn(midi::MidiLearnTargetType::SynthParam, 0, 0, 0, 20.0f, 20000.0f, "PolySynth Cutoff");
                        }
                        ImGui::SameLine();
                        if (ImGui::SmallButton(" Clear All ")) {
                            midi_learn.clear_all_bindings();
                        }

                        if (midi_learn.is_learning()) {
                            ImGui::SameLine();
                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.50f, 0.10f, 1.0f));
                            if (ImGui::SmallButton(" ⚠ CANCEL LEARN ")) {
                                midi_learn.cancel_learn();
                            }
                            ImGui::PopStyleColor();
                            ImGui::SameLine();
                            ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.20f, 1.0f), "● Waiting for CC input...");
                        }

                        if (show_midi_learn_table) {
                            auto bindings = midi_learn.get_bindings();
                            if (bindings.empty()) {
                                ImGui::TextDisabled("   (No active MIDI CC bindings. Click '+ Learn' or right-click any track fader)");
                            } else {
                                if (ImGui::BeginTable("MidiLearnBindingsTable", 7, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
                                    ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed, 35.0f);
                                    ImGui::TableSetupColumn("Label / Parameter", ImGuiTableColumnFlags_WidthStretch);
                                    ImGui::TableSetupColumn("Channel", ImGuiTableColumnFlags_WidthFixed, 60.0f);
                                    ImGui::TableSetupColumn("CC #", ImGuiTableColumnFlags_WidthFixed, 55.0f);
                                    ImGui::TableSetupColumn("Range [Min, Max]", ImGuiTableColumnFlags_WidthFixed, 130.0f);
                                    ImGui::TableSetupColumn("Last Value", ImGuiTableColumnFlags_WidthFixed, 80.0f);
                                    ImGui::TableSetupColumn("Actions", ImGuiTableColumnFlags_WidthFixed, 70.0f);
                                    ImGui::TableHeadersRow();

                                    for (const auto& b : bindings) {
                                        ImGui::TableNextRow();
                                        ImGui::TableSetColumnIndex(0);
                                        ImGui::Text("%u", b.id);

                                        ImGui::TableSetColumnIndex(1);
                                        if (!b.custom_label.empty()) {
                                            ImGui::Text("%s", b.custom_label.c_str());
                                        } else {
                                            ImGui::Text("%s", midi::midi_learn_target_type_name(b.target.type));
                                        }

                                        ImGui::TableSetColumnIndex(2);
                                        if (b.channel == midi::MidiLearnRouter::kOmniChannel) {
                                            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Omni");
                                        } else {
                                            ImGui::Text("Ch %u", b.channel + 1);
                                        }

                                        ImGui::TableSetColumnIndex(3);
                                        ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "CC %u", b.cc_number);

                                        ImGui::TableSetColumnIndex(4);
                                        ImGui::Text("[%.2f, %.2f]", b.min_val, b.max_val);

                                        ImGui::TableSetColumnIndex(5);
                                        ImGui::Text("%.2f", b.last_value);

                                        ImGui::TableSetColumnIndex(6);
                                        ImGui::PushID(static_cast<int>(b.id));
                                        if (ImGui::SmallButton("Unbind")) {
                                            midi_learn.unbind_by_id(b.id);
                                        }
                                        ImGui::PopID();
                                    }
                                    ImGui::EndTable();
                                }
                            }
                        }
                    }
                    ImGui::Separator();

                    // Header Status & Trigger Bar: Polyphonic Chords & Voice Audition
                    ImGui::Text("Poly Chords & Audition:");
                    ImGui::SameLine();
                    if (ImGui::Button(" ▶ C MIN 9TH ", ImVec2(115, 24))) {
                        mod_matrix.poly_all_notes_off();
                        mod_matrix.poly_note_on(48, 0.90f); // C3
                        mod_matrix.poly_note_on(51, 0.85f); // Eb3
                        mod_matrix.poly_note_on(55, 0.85f); // G3
                        mod_matrix.poly_note_on(58, 0.80f); // Bb3
                        mod_matrix.poly_note_on(62, 0.75f); // D4
                    }
                    ImGui::SameLine();
                    if (ImGui::Button(" ▶ F MAJ 7TH ", ImVec2(115, 24))) {
                        mod_matrix.poly_all_notes_off();
                        mod_matrix.poly_note_on(53, 0.90f); // F3
                        mod_matrix.poly_note_on(57, 0.85f); // A3
                        mod_matrix.poly_note_on(60, 0.85f); // C4
                        mod_matrix.poly_note_on(64, 0.80f); // E4
                    }
                    ImGui::SameLine();
                    if (ImGui::Button(" ▶ D DORIAN LEAD ", ImVec2(130, 24))) {
                        mod_matrix.poly_all_notes_off();
                        mod_matrix.poly_note_on(50, 0.95f); // D3
                        mod_matrix.poly_note_on(53, 0.85f); // F3
                        mod_matrix.poly_note_on(57, 0.85f); // A3
                    }
                    ImGui::SameLine();
                    if (ImGui::Button(" ▶ UNISON 4X ", ImVec2(110, 24))) {
                        mod_matrix.poly_synth().set_play_mode(modulation::PolyphonyPlayMode::Unison4x);
                        mod_matrix.poly_note_on(48, 0.95f);
                    }
                    ImGui::SameLine();
                    if (ImGui::Button(" ■ RELEASE ALL ", ImVec2(115, 24))) {
                        mod_matrix.poly_all_notes_off();
                    }
                    ImGui::SameLine();
                    if (ImGui::Button(" ⟳ HI-HAT ", ImVec2(90, 24))) {
                        mod_matrix.mseg1_voice().trigger(mod_preview_vel);
                    }

                    ImGui::SameLine(0, 15);
                    size_t act_voices = mod_matrix.poly_synth().active_voice_count();
                    ImVec4 v_col = (act_voices > 0) ? ImVec4(0.12f, 0.65f, 0.35f, 1.0f) : ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
                    ImGui::TextColored(v_col, "[VOICES: %zu / %zu ACTIVE]", act_voices, mod_matrix.poly_synth().polyphony_limit());

                    ImGui::Separator();

                    // Modulator Selector Buttons (5 Tabs)
                    const char* mod_tabs[5] = {
                        "MSEG 1: Hi-Hat Percussion",
                        "MSEG 2: Plucked Lead / Mod",
                        "LFO 1: Primary Oscillator",
                        "LFO 2: BeatSync Slew Random",
                        "POLY SYNTH // VOICE POOL (16v)"
                    };
                    for (int m = 0; m < 5; ++m) {
                        if (m > 0) ImGui::SameLine();
                        bool is_sel = (selected_modulator == m);
                        if (is_sel) {
                            ImVec4 btn_col = (m == 4) ? ImVec4(0.16f, 0.52f, 0.32f, 0.95f) : ImVec4(0.12f, 0.38f, 0.85f, 0.9f);
                            ImGui::PushStyleColor(ImGuiCol_Button, btn_col);
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                        }
                        const float tab_w = (m == 4) ? 230.0f : 185.0f;
                        if (ImGui::Button(mod_tabs[m], ImVec2(tab_w, 26))) {
                            selected_modulator = m;
                        }
                        if (is_sel) {
                            ImGui::PopStyleColor(2);
                        }
                    }

                    ImGui::Spacing();

                    // Split into Top (Inspector & Curve Editor) and Bottom (Modulation Matrix Table)
                    const float content_h = ImGui::GetContentRegionAvail().y;
                    const float top_h = std::max(220.0f, content_h * 0.52f);
                    const float bot_h = std::max(160.0f, content_h - top_h - 10.0f);

                    ImGui::BeginChild("ModulatorTopSection", ImVec2(0, top_h), false);
                    {
                        // Left column: Modulator Controls (Inspector)
                        ImGui::BeginChild("ModulatorInspector", ImVec2(320, 0), true);
                        {
                            if (selected_modulator == 0 || selected_modulator == 1) {
                                auto& cur_mseg = (selected_modulator == 0) ? mod_matrix.mseg1() : mod_matrix.mseg2();

                                ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "%s PARAMETERS",
                                                   (selected_modulator == 0) ? "MSEG 1 (PERCUSSIVE)" : "MSEG 2 (LEAD)");
                                ImGui::Separator();

                                // Presets
                                ImGui::Text("Curated Presets:");
                                if (ImGui::Button("Hi-Hat (75ms)")) {
                                    cur_mseg.preset_percussive_hihat();
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Plucked")) {
                                    cur_mseg.preset_plucked_synth();
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Pad Swell")) {
                                    cur_mseg.preset_pad_swell();
                                }
                                if (ImGui::Button("Wobble LFO")) {
                                    cur_mseg.preset_wobble_lfo();
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("Buchla Maths")) {
                                    cur_mseg.preset_buchla_maths();
                                }

                                ImGui::Spacing();
                                ImGui::Text("Domain / Timing:");
                                bool is_beats = (cur_mseg.time_mode() == modulation::MsegTimeMode::BeatSync);
                                if (ImGui::RadioButton("Milliseconds", !is_beats)) {
                                    auto pts = cur_mseg.get_points();
                                    cur_mseg.set_points(pts, modulation::MsegTimeMode::Milliseconds, cur_mseg.loop_mode(), cur_mseg.sustain_index());
                                }
                                ImGui::SameLine();
                                if (ImGui::RadioButton("Beat-Sync", is_beats)) {
                                    auto pts = cur_mseg.get_points();
                                    cur_mseg.set_points(pts, modulation::MsegTimeMode::BeatSync, cur_mseg.loop_mode(), cur_mseg.sustain_index());
                                }

                                ImGui::Text("Loop Topology:");
                                int loop_m = static_cast<int>(cur_mseg.loop_mode());
                                if (ImGui::RadioButton("OneShot", loop_m == 0)) {
                                    auto pts = cur_mseg.get_points();
                                    cur_mseg.set_points(pts, cur_mseg.time_mode(), modulation::MsegLoopMode::OneShot, cur_mseg.sustain_index());
                                }
                                ImGui::SameLine();
                                if (ImGui::RadioButton("Sustain", loop_m == 1)) {
                                    auto pts = cur_mseg.get_points();
                                    cur_mseg.set_points(pts, cur_mseg.time_mode(), modulation::MsegLoopMode::SustainLoop, cur_mseg.sustain_index());
                                }
                                ImGui::SameLine();
                                if (ImGui::RadioButton("FreeRun", loop_m == 2)) {
                                    auto pts = cur_mseg.get_points();
                                    cur_mseg.set_points(pts, cur_mseg.time_mode(), modulation::MsegLoopMode::FreeRunLoop, cur_mseg.sustain_index());
                                }

                                ImGui::Spacing();
                                ImGui::TextColored(ImVec4(0.85f, 0.48f, 0.05f, 1.0f), "Meta-Modulation Inputs (Modulated Macros):");

                                // Base Attack Scale
                                float base_atk = cur_mseg.base_attack_scale();
                                float eff_atk = cur_mseg.effective_attack_scale();
                                float mod_atk = eff_atk - base_atk;
                                const char* atk_badge = (selected_modulator == 0) ? "LFO 1" : "Mod";
                                if (ui::DrawModulatedSlider("Attack Scale", &base_atk, 0.05f, 5.0f, mod_atk, "%.2fx", atk_badge, 280.0f)) {
                                    cur_mseg.set_base_attack_scale(base_atk);
                                }

                                // Base Decay Scale
                                float base_dec = cur_mseg.base_decay_scale();
                                float eff_dec = cur_mseg.effective_decay_scale();
                                float mod_dec = eff_dec - base_dec;
                                if (ui::DrawModulatedSlider("Decay Scale", &base_dec, 0.05f, 5.0f, mod_dec, "%.2fx", nullptr, 280.0f)) {
                                    cur_mseg.set_base_decay_scale(base_dec);
                                }

                                // Base Speed / TimeScale
                                float base_spd = cur_mseg.base_time_scale();
                                float eff_spd = cur_mseg.effective_time_scale();
                                float mod_spd = eff_spd - base_spd;
                                if (ui::DrawModulatedSlider("Time Scale (Speed)", &base_spd, 0.1f, 5.0f, mod_spd, "%.2fx", nullptr, 280.0f)) {
                                    cur_mseg.set_base_time_scale(base_spd);
                                }

                                // Level Scale
                                float base_lvl = cur_mseg.base_level_scale();
                                float eff_lvl = cur_mseg.effective_level_scale();
                                float mod_lvl = eff_lvl - base_lvl;
                                if (ui::DrawModulatedSlider("Level Scale", &base_lvl, 0.0f, 2.0f, mod_lvl, "%.2f", nullptr, 280.0f)) {
                                    cur_mseg.set_base_level_scale(base_lvl);
                                }

                                // Curvature Tension Offset
                                float base_tens = cur_mseg.base_tension_offset();
                                float eff_tens = cur_mseg.effective_tension_offset();
                                float mod_tens = eff_tens - base_tens;
                                if (ui::DrawModulatedSlider("Tension Bias", &base_tens, -1.0f, 1.0f, mod_tens, "%+.2f", nullptr, 280.0f)) {
                                    cur_mseg.set_base_tension_offset(base_tens);
                                }
                            } else if (selected_modulator == 2 || selected_modulator == 3) {
                                // LFO 1 or LFO 2
                                auto& cur_lfo = (selected_modulator == 2) ? mod_matrix.lfo1() : mod_matrix.lfo2();
                                ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "%s PARAMETERS",
                                                   (selected_modulator == 2) ? "LFO 1 (PRIMARY)" : "LFO 2 (SECONDARY)");
                                ImGui::Separator();

                                const char* wf_labels[] = { "Sine", "Triangle", "Saw Up", "Saw Down", "Square", "Sample & Hold", "Smooth Random Walk" };
                                int cur_wf = static_cast<int>(cur_lfo.waveform());
                                ImGui::SetNextItemWidth(260);
                                if (ImGui::Combo("Waveform", &cur_wf, wf_labels, 7)) {
                                    cur_lfo.set_waveform(static_cast<modulation::LfoWaveform>(cur_wf));
                                }

                                bool sync = cur_lfo.is_beat_sync();
                                if (ImGui::Checkbox("Beat Sync", &sync)) {
                                    cur_lfo.set_beat_sync(sync);
                                }

                                if (sync) {
                                    const char* beat_rates[] = { "1/16 (0.25)", "1/8 (0.50)", "1/4 (1.00)", "1/2 (2.00)", "1 Bar (4.00)", "2 Bars (8.00)" };
                                    const double beat_vals[] = { 0.25, 0.50, 1.00, 2.00, 4.00, 8.00 };
                                    int b_idx = 2;
                                    double cur_b = cur_lfo.beats_per_cycle();
                                    for (int i = 0; i < 6; ++i) {
                                        if (std::abs(cur_b - beat_vals[i]) < 0.05) b_idx = i;
                                    }
                                    ImGui::SetNextItemWidth(260);
                                    if (ImGui::Combo("Beats / Cycle", &b_idx, beat_rates, 6)) {
                                        cur_lfo.set_beats_per_cycle(beat_vals[b_idx]);
                                    }
                                } else {
                                    float hz = cur_lfo.frequency_hz();
                                    float eff_hz = cur_lfo.effective_rate_hz();
                                    float mod_hz = eff_hz - hz;
                                    if (ui::DrawModulatedSlider("Rate (Hz)", &hz, 0.05f, 50.0f, mod_hz, "%.2f Hz", "FM", 260.0f)) {
                                        cur_lfo.set_frequency_hz(hz);
                                    }
                                }

                                float depth = cur_lfo.depth();
                                float eff_depth = cur_lfo.effective_depth();
                                float mod_depth = eff_depth - depth;
                                if (ui::DrawModulatedSlider("Depth", &depth, 0.0f, 1.0f, mod_depth, "%.2f", "AM", 260.0f)) {
                                    cur_lfo.set_depth(depth);
                                }

                                bool bp = cur_lfo.is_bipolar();
                                if (ImGui::Checkbox("Bipolar [-1.0 .. +1.0]", &bp)) {
                                    cur_lfo.set_bipolar(bp);
                                }

                                ImGui::Spacing();
                                ImGui::Text("Real-Time Output: ");
                                ImGui::SameLine();
                                ImGui::TextColored(ImVec4(0.85f, 0.48f, 0.05f, 1.0f), "%+.3f", cur_lfo.current_value());
                            } else if (selected_modulator == 4) {
                                // Polyphonic Synth Voice Pool Parameters
                                ImGui::TextColored(ImVec4(0.16f, 0.65f, 0.38f, 1.0f), "POLYPHONIC SYNTH PARAMETERS");
                                ImGui::Separator();

                                const char* play_mode_names[] = { "Polyphonic (16-Voice)", "Mono Legato (Glide)", "Unison 4x (Detuned)" };
                                int cur_mode = static_cast<int>(mod_matrix.poly_synth().play_mode());
                                ImGui::SetNextItemWidth(260);
                                if (ImGui::Combo("Play Mode", &cur_mode, play_mode_names, 3)) {
                                    mod_matrix.poly_synth().set_play_mode(static_cast<modulation::PolyphonyPlayMode>(cur_mode));
                                }

                                int poly_lim = static_cast<int>(mod_matrix.poly_synth().polyphony_limit());
                                ImGui::SetNextItemWidth(260);
                                if (ImGui::SliderInt("Voice Limit", &poly_lim, 1, 16)) {
                                    mod_matrix.poly_synth().set_polyphony_limit(static_cast<size_t>(poly_lim));
                                }

                                float glide = mod_matrix.poly_synth().glide_time_ms();
                                ImGui::SetNextItemWidth(260);
                                if (ImGui::SliderFloat("Portamento Glide", &glide, 0.0f, 300.0f, "%.1f ms")) {
                                    mod_matrix.poly_synth().set_glide_time_ms(glide);
                                }

                                float pan_spr = mod_matrix.poly_synth().voice_pan_spread();
                                ImGui::SetNextItemWidth(260);
                                if (ImGui::SliderFloat("Stereo Pan Spread", &pan_spr, 0.0f, 1.0f, "%.2f")) {
                                    mod_matrix.poly_synth().set_voice_pan_spread(pan_spr);
                                }

                                float m_lvl = mod_matrix.poly_synth().master_level();
                                ImGui::SetNextItemWidth(260);
                                if (ImGui::SliderFloat("Master Level", &m_lvl, 0.0f, 1.5f, "%.2f")) {
                                    mod_matrix.poly_synth().set_master_level(m_lvl);
                                }

                                ImGui::Spacing();
                                ImGui::TextColored(ImVec4(0.85f, 0.65f, 0.15f, 1.0f), "DUAL OSCILLATORS (polyBLEP)");
                                ImGui::Separator();

                                const char* osc_wf_names[] = { "Sine", "Triangle", "Saw", "Square", "Noise" };
                                int osc1_wf = static_cast<int>(mod_matrix.poly_synth().osc1_waveform());
                                ImGui::SetNextItemWidth(260);
                                if (ImGui::Combo("Osc 1 Wave", &osc1_wf, osc_wf_names, 5)) {
                                    mod_matrix.poly_synth().set_osc1_waveform(static_cast<dsp::Waveform>(osc1_wf));
                                }

                                int osc2_wf = static_cast<int>(mod_matrix.poly_synth().osc2_waveform());
                                ImGui::SetNextItemWidth(260);
                                if (ImGui::Combo("Osc 2 Wave", &osc2_wf, osc_wf_names, 5)) {
                                    mod_matrix.poly_synth().set_osc2_waveform(static_cast<dsp::Waveform>(osc2_wf));
                                }

                                float osc_mix = mod_matrix.poly_synth().osc_mix();
                                ImGui::SetNextItemWidth(260);
                                if (ImGui::SliderFloat("Osc Mix (1 vs 2)", &osc_mix, 0.0f, 1.0f, "%.2f")) {
                                    mod_matrix.poly_synth().set_osc_mix(osc_mix);
                                }

                                float osc2_det = mod_matrix.poly_synth().osc2_detune_cents();
                                ImGui::SetNextItemWidth(260);
                                if (ImGui::SliderFloat("Osc 2 Detune", &osc2_det, -50.0f, 50.0f, "%+.1f cents")) {
                                    mod_matrix.poly_synth().set_osc2_detune_cents(osc2_det);
                                }

                                int osc2_oct = mod_matrix.poly_synth().osc2_octave_offset();
                                ImGui::SetNextItemWidth(260);
                                if (ImGui::SliderInt("Osc 2 Octave", &osc2_oct, -2, 2)) {
                                    mod_matrix.poly_synth().set_osc2_octave_offset(osc2_oct);
                                }

                                ImGui::Spacing();
                                ImGui::TextColored(ImVec4(0.85f, 0.48f, 0.05f, 1.0f), "PER-VOICE BIQUAD FILTER");
                                ImGui::Separator();

                                const char* flt_names[] = { "LowPass", "HighPass", "BandPass", "Notch" };
                                int flt_type = static_cast<int>(mod_matrix.poly_synth().filter_type());
                                ImGui::SetNextItemWidth(260);
                                if (ImGui::Combo("Filter Type", &flt_type, flt_names, 4)) {
                                    mod_matrix.poly_synth().set_filter_type(static_cast<dsp::FilterType>(flt_type));
                                }

                                float f_cut = mod_matrix.poly_synth().base_cutoff();
                                ImGui::SetNextItemWidth(260);
                                if (ImGui::SliderFloat("Cutoff (Hz)", &f_cut, 40.0f, 18000.0f, "%.0f Hz", ImGuiSliderFlags_Logarithmic)) {
                                    mod_matrix.poly_synth().set_base_cutoff(f_cut);
                                }

                                float f_res = mod_matrix.poly_synth().resonance_q();
                                ImGui::SetNextItemWidth(260);
                                if (ImGui::SliderFloat("Resonance (Q)", &f_res, 0.5f, 18.0f, "%.2f")) {
                                    mod_matrix.poly_synth().set_resonance_q(f_res);
                                }

                                float f_env = mod_matrix.poly_synth().filter_env_amount();
                                ImGui::SetNextItemWidth(260);
                                if (ImGui::SliderFloat("Filter Env Amt", &f_env, -8000.0f, 8000.0f, "%+.0f Hz")) {
                                    mod_matrix.poly_synth().set_filter_env_amount(f_env);
                                }

                                float f_kt = mod_matrix.poly_synth().keytrack_amount();
                                ImGui::SetNextItemWidth(260);
                                if (ImGui::SliderFloat("Key Tracking", &f_kt, 0.0f, 1.0f, "%.2f")) {
                                    mod_matrix.poly_synth().set_keytrack_amount(f_kt);
                                }
                            }
                        }
                        ImGui::EndChild();

                        ImGui::SameLine();

                        // Right column: Canvas (MSEG Spline Canvas or LFO Scope or Polyphonic Telemetry Grid)
                        ImGui::BeginChild("ModulatorCanvasPanel", ImVec2(0, 0), true);
                        {
                            if (selected_modulator == 0 || selected_modulator == 1) {
                                auto& cur_mseg = (selected_modulator == 0) ? mod_matrix.mseg1() : mod_matrix.mseg2();
                                auto& cur_voice = (selected_modulator == 0) ? mod_matrix.mseg1_voice() : mod_matrix.mseg2_voice();

                                ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f),
                                                   "BREAKPOINT SPLINE EDITOR // %s (Drag Nodes, Drag Tension Dots, Double-Click Add, Right-Click Delete)",
                                                   (selected_modulator == 0) ? "MSEG 1 (PERCUSSIVE VOICE)" : "MSEG 2 (LEAD VOICE)");

                                const float canvas_h = top_h - 78.0f;
                                ui::DrawMsegCurveEditor("##MsegCanvas", cur_mseg, &cur_voice, ImVec2(0, canvas_h), &selected_mseg_node, true);

                                // Enhanced Node Inspector Footer: Precision Numerical Controls & Shape Presets
                                auto pts = cur_mseg.get_points();
                                if (selected_mseg_node >= 0 && selected_mseg_node < static_cast<int>(pts.size())) {
                                    auto pt = pts[selected_mseg_node];
                                    bool pts_changed = false;
                                    const bool is_sync = (cur_mseg.time_mode() == modulation::MsegTimeMode::BeatSync);
                                    const char* unit_str = is_sync ? "beats" : "ms";

                                    // Row 1: Node Identification & Numerical Parameter Drags
                                    ImVec4 mode_col = (pt.node_mode == routing::NodeMode::Smooth) ? ImVec4(0.2f, 0.6f, 1.0f, 1.0f) :
                                                      ((pt.node_mode == routing::NodeMode::Corner) ? ImVec4(1.0f, 0.65f, 0.2f, 1.0f) : ImVec4(0.7f, 0.7f, 0.7f, 1.0f));
                                    ImGui::TextColored(mode_col, "Node #%d [%s]", selected_mseg_node,
                                                       (pt.node_mode == routing::NodeMode::Smooth) ? "Smooth" :
                                                       ((pt.node_mode == routing::NodeMode::Corner) ? "Corner" : "Hold"));

                                    if (cur_mseg.sustain_index() == selected_mseg_node) {
                                        ImGui::SameLine();
                                        ImGui::TextColored(ImVec4(0.85f, 0.48f, 0.05f, 1.0f), "[SUSTAIN]");
                                    }

                                    ImGui::SameLine(0, 15);
                                    ImGui::SetNextItemWidth(80);
                                    float f_time = static_cast<float>(pt.time);
                                    float t_min = (selected_mseg_node == 0) ? 0.0f : static_cast<float>(pts[selected_mseg_node - 1].time + (is_sync ? 0.02f : 0.5f));
                                    float t_max = (selected_mseg_node == static_cast<int>(pts.size() - 1)) ? 10000.0f : static_cast<float>(pts[selected_mseg_node + 1].time - (is_sync ? 0.02f : 0.5f));
                                    if (selected_mseg_node > 0) {
                                        if (ImGui::DragFloat("##NodeTime", &f_time, is_sync ? 0.02f : 1.0f, t_min, t_max, "%.2f")) {
                                            pt.time = std::clamp(static_cast<double>(f_time), static_cast<double>(t_min), static_cast<double>(t_max));
                                            pts_changed = true;
                                        }
                                        ImGui::SameLine(0, 3);
                                        ImGui::TextDisabled("%s", unit_str);
                                    } else {
                                        ImGui::TextDisabled("Time: 0.00 %s", unit_str);
                                    }

                                    ImGui::SameLine(0, 12);
                                    ImGui::SetNextItemWidth(75);
                                    float f_val = pt.value;
                                    if (ImGui::DragFloat("Val##NodeVal", &f_val, 0.01f, 0.0f, 1.0f, "%.2f")) {
                                        pt.value = std::clamp(f_val, 0.0f, 1.0f);
                                        pts_changed = true;
                                    }

                                    ImGui::SameLine(0, 12);
                                    ImGui::SetNextItemWidth(75);
                                    float f_tens = pt.tension;
                                    if (ImGui::DragFloat("τ##NodeTens", &f_tens, 0.02f, -1.0f, 1.0f, "%+.2f")) {
                                        pt.tension = std::clamp(f_tens, -1.0f, 1.0f);
                                        pts_changed = true;
                                    }

                                    ImGui::SameLine(0, 12);
                                    if (ImGui::SmallButton(" Mode ")) {
                                        if (pt.node_mode == routing::NodeMode::Smooth) pt.node_mode = routing::NodeMode::Corner;
                                        else if (pt.node_mode == routing::NodeMode::Corner) pt.node_mode = routing::NodeMode::Hold;
                                        else pt.node_mode = routing::NodeMode::Smooth;
                                        pts_changed = true;
                                    }

                                    ImGui::SameLine();
                                    if (ImGui::SmallButton(" Set Sustain ")) {
                                        cur_mseg.set_points(pts, cur_mseg.time_mode(), cur_mseg.loop_mode(), selected_mseg_node);
                                    }

                                    // Row 2: Segment Shape Presets & Node Insertion/Deletion
                                    ImGui::TextDisabled("Shape Presets:");
                                    ImGui::SameLine();
                                    if (ImGui::SmallButton(" Linear ")) {
                                        pt.tension = 0.0f;
                                        pt.node_mode = routing::NodeMode::Corner;
                                        pts_changed = true;
                                    }
                                    ImGui::SameLine();
                                    if (ImGui::SmallButton(" Smooth S ")) {
                                        pt.tension = 0.0f;
                                        pt.node_mode = routing::NodeMode::Smooth;
                                        pts_changed = true;
                                    }
                                    ImGui::SameLine();
                                    if (ImGui::SmallButton(" Exp (+) ")) {
                                        pt.tension = 0.65f;
                                        pts_changed = true;
                                    }
                                    ImGui::SameLine();
                                    if (ImGui::SmallButton(" Log (-) ")) {
                                        pt.tension = -0.65f;
                                        pts_changed = true;
                                    }
                                    ImGui::SameLine();
                                    if (ImGui::SmallButton(" Step/Hold ")) {
                                        pt.node_mode = routing::NodeMode::Hold;
                                        pts_changed = true;
                                    }

                                    ImGui::SameLine(0, 15);
                                    if (ImGui::SmallButton(" + Insert Node ")) {
                                        double next_t = (selected_mseg_node + 1 < static_cast<int>(pts.size())) ?
                                            0.5 * (pt.time + pts[selected_mseg_node + 1].time) : (pt.time + (is_sync ? 1.0 : 50.0));
                                        float next_v = 0.5f * pt.value;
                                        pts.insert(pts.begin() + selected_mseg_node + 1,
                                                   modulation::MsegPoint{next_t, next_v, routing::NodeMode::Smooth, 0.0f});
                                        cur_mseg.set_points(pts, cur_mseg.time_mode(), cur_mseg.loop_mode(), cur_mseg.sustain_index());
                                        selected_mseg_node++;
                                        pts_changed = false;
                                    }

                                    if (selected_mseg_node > 0 && selected_mseg_node < static_cast<int>(pts.size() - 1)) {
                                        ImGui::SameLine();
                                        if (ImGui::SmallButton(" ✕ Delete Node ")) {
                                            pts.erase(pts.begin() + selected_mseg_node);
                                            cur_mseg.set_points(pts, cur_mseg.time_mode(), cur_mseg.loop_mode(), cur_mseg.sustain_index());
                                            selected_mseg_node = std::min(selected_mseg_node, static_cast<int>(pts.size() - 1));
                                            pts_changed = false;
                                        }
                                    }

                                    if (pts_changed) {
                                        pts[selected_mseg_node] = pt;
                                        cur_mseg.set_points(pts, cur_mseg.time_mode(), cur_mseg.loop_mode(), cur_mseg.sustain_index());
                                    }
                                }
                            } else if (selected_modulator == 2 || selected_modulator == 3) {
                                // LFO Visualizer
                                auto& cur_lfo = (selected_modulator == 2) ? mod_matrix.lfo1() : mod_matrix.lfo2();
                                ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f),
                                                   "LFO OSCILLOSCOPE // %s",
                                                   (selected_modulator == 2) ? "LFO 1 (PRIMARY)" : "LFO 2 (SECONDARY)");
                                const float scope_h = top_h - 40.0f;
                                ui::DrawLfoOscilloscope("##LfoOsc", cur_lfo, ImVec2(0, scope_h));
                            } else if (selected_modulator == 4) {
                                // 16-Voice Runtime Telemetry Grid
                                ImGui::TextColored(ImVec4(0.16f, 0.65f, 0.38f, 1.0f),
                                                   "16-VOICE RUNTIME TELEMETRY GRID // Click-Free Voice Stealing Pool");
                                ImGui::SameLine();
                                ImGui::TextDisabled("| Dual MSEG Dynamic Levels • Stereo Pan Tracking • Zero-Allocation POD");

                                std::array<modulation::PolyVoiceTelemetry, 16> poly_telem;
                                mod_matrix.poly_synth().get_telemetry(poly_telem);
                                ui::DrawPolyVoiceTelemetryGrid("##VoicePoolGrid", poly_telem, ImVec2(0, 0), mod_matrix.poly_synth().polyphony_limit());
                            }
                        }
                        ImGui::EndChild();
                    }
                    ImGui::EndChild();

                    ImGui::Spacing();

                    // Bottom section: 16-Route Modulation Matrix Table
                    ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "BITWIG MODULATION MATRIX // 16 MODULATION ROUTES");
                    ImGui::SameLine();
                    ImGui::TextDisabled("| Direct Modulator-to-Modulator Routing (Meta-Modulation) & Synth Targets");

                    ImGui::BeginChild("ModulationTableChild", ImVec2(0, bot_h), true);
                    {
                        struct ModSourceChoice {
                            modulation::ModulationSource src;
                            const char* name;
                        };
                        static const ModSourceChoice kSrcChoices[10] = {
                            { modulation::ModulationSource::None,      "-- None --" },
                            { modulation::ModulationSource::LFO1,      "LFO 1 (Free Osc)" },
                            { modulation::ModulationSource::LFO2,      "LFO 2 (BeatSync / Slew)" },
                            { modulation::ModulationSource::MSEG1,     "MSEG 1 (Voice Env)" },
                            { modulation::ModulationSource::MSEG2,     "MSEG 2 (Mod Env)" },
                            { modulation::ModulationSource::Velocity,  "Velocity" },
                            { modulation::ModulationSource::KeyTrack,  "Key Tracking" },
                            { modulation::ModulationSource::ModWheel,  "Mod Wheel (CC 1)" },
                            { modulation::ModulationSource::PitchBend, "Pitch Bend" },
                            { modulation::ModulationSource::RandomSH,  "Random S&H" }
                        };

                        struct ModDestChoice {
                            modulation::ModulationDestination dst;
                            const char* name;
                        };
                        static const ModDestChoice kDstChoices[21] = {
                            { modulation::ModulationDestination::None,              "-- None --" },
                            { modulation::ModulationDestination::MSEG1_Attack,      "MSEG 1: Attack Time (Meta-Mod)" },
                            { modulation::ModulationDestination::MSEG1_Decay,       "MSEG 1: Decay Time (Meta-Mod)" },
                            { modulation::ModulationDestination::MSEG1_TimeScale,   "MSEG 1: Speed / TimeScale" },
                            { modulation::ModulationDestination::MSEG1_Level,       "MSEG 1: Level / Amplitude" },
                            { modulation::ModulationDestination::MSEG1_Tension,     "MSEG 1: Tension / Curvature" },
                            { modulation::ModulationDestination::MSEG2_Attack,      "MSEG 2: Attack Time (Meta-Mod)" },
                            { modulation::ModulationDestination::MSEG2_Decay,       "MSEG 2: Decay Time (Meta-Mod)" },
                            { modulation::ModulationDestination::MSEG2_TimeScale,   "MSEG 2: Speed / TimeScale" },
                            { modulation::ModulationDestination::MSEG2_Level,       "MSEG 2: Level / Amplitude" },
                            { modulation::ModulationDestination::MSEG2_Tension,     "MSEG 2: Tension / Curvature" },
                            { modulation::ModulationDestination::LFO1_Rate,         "LFO 1: Frequency / Rate (FM)" },
                            { modulation::ModulationDestination::LFO1_Depth,        "LFO 1: Depth (AM)" },
                            { modulation::ModulationDestination::LFO2_Rate,         "LFO 2: Frequency / Rate (FM)" },
                            { modulation::ModulationDestination::LFO2_Depth,        "LFO 2: Depth (AM)" },
                            { modulation::ModulationDestination::SynthCutoff,       "Synth: Filter Cutoff" },
                            { modulation::ModulationDestination::SynthResonance,    "Synth: Filter Resonance" },
                            { modulation::ModulationDestination::SynthPitch,        "Synth: Pitch / Detune" },
                            { modulation::ModulationDestination::SynthAmp,          "Synth: Amplifier Level" },
                            { modulation::ModulationDestination::TrackSlot0_Param0, "Insert Slot 0: Param 0" },
                            { modulation::ModulationDestination::TrackSlot0_Param1, "Insert Slot 0: Param 1" }
                        };

                        if (ImGui::BeginTable("ModMatrixTable", 6, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
                            ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, 30.0f);
                            ImGui::TableSetupColumn("Active", ImGuiTableColumnFlags_WidthFixed, 45.0f);
                            ImGui::TableSetupColumn("Source", ImGuiTableColumnFlags_WidthFixed, 180.0f);
                            ImGui::TableSetupColumn("Destination", ImGuiTableColumnFlags_WidthFixed, 230.0f);
                            ImGui::TableSetupColumn("Amount", ImGuiTableColumnFlags_WidthStretch);
                            ImGui::TableSetupColumn("Live Signal", ImGuiTableColumnFlags_WidthFixed, 140.0f);
                            ImGui::TableHeadersRow();

                            const char* src_names[10];
                            for (int i = 0; i < 10; ++i) src_names[i] = kSrcChoices[i].name;

                            const char* dst_names[21];
                            for (int i = 0; i < 21; ++i) dst_names[i] = kDstChoices[i].name;

                            auto& routes = mod_matrix.routes();
                            for (size_t r = 0; r < modulation::ModulationMatrix::kMaxRoutes; ++r) {
                                auto& route = routes[r];
                                ImGui::TableNextRow();

                                // Column 0: Index
                                ImGui::TableSetColumnIndex(0);
                                ImGui::Text("%zu", r + 1);

                                // Column 1: Active
                                ImGui::TableSetColumnIndex(1);
                                ImGui::PushID(static_cast<int>(r));
                                ImGui::Checkbox("##act", &route.active);

                                // Column 2: Source
                                ImGui::TableSetColumnIndex(2);
                                int cur_src_idx = 0;
                                for (int s = 0; s < 10; ++s) {
                                    if (kSrcChoices[s].src == route.source) cur_src_idx = s;
                                }
                                ImGui::SetNextItemWidth(-1);
                                if (ImGui::Combo("##src", &cur_src_idx, src_names, 10)) {
                                    route.source = kSrcChoices[cur_src_idx].src;
                                }

                                // Column 3: Destination
                                ImGui::TableSetColumnIndex(3);
                                int cur_dst_idx = 0;
                                for (int d = 0; d < 21; ++d) {
                                    if (kDstChoices[d].dst == route.destination) cur_dst_idx = d;
                                }
                                ImGui::SetNextItemWidth(-1);
                                if (ImGui::Combo("##dst", &cur_dst_idx, dst_names, 21)) {
                                    route.destination = kDstChoices[cur_dst_idx].dst;
                                }

                                // Column 4: Amount
                                ImGui::TableSetColumnIndex(4);
                                ImGui::SetNextItemWidth(-1);
                                ImGui::SliderFloat("##amt", &route.amount, -1.0f, 1.0f, "%+.2f");

                                // Column 5: Live Signal Meter
                                ImGui::TableSetColumnIndex(5);
                                float src_sig = 0.0f;
                                switch (route.source) {
                                    case modulation::ModulationSource::LFO1: src_sig = mod_matrix.lfo1().current_value(); break;
                                    case modulation::ModulationSource::LFO2: src_sig = mod_matrix.lfo2().current_value(); break;
                                    case modulation::ModulationSource::MSEG1: src_sig = mod_matrix.mseg1_voice().current_value(); break;
                                    case modulation::ModulationSource::MSEG2: src_sig = mod_matrix.mseg2_voice().current_value(); break;
                                    case modulation::ModulationSource::Velocity: src_sig = mod_preview_vel; break;
                                    case modulation::ModulationSource::KeyTrack: src_sig = 0.5f; break;
                                    default: src_sig = 0.0f; break;
                                }
                                float live_delta = route.active ? (src_sig * route.amount) : 0.0f;

                                // Draw miniature bi-directional meter
                                ImDrawList* dl = ImGui::GetWindowDrawList();
                                ImVec2 m_pos = ImGui::GetCursorScreenPos();
                                const float m_w = 120.0f;
                                const float m_h = 16.0f;
                                dl->AddRectFilled(m_pos, ImVec2(m_pos.x + m_w, m_pos.y + m_h), ImColor(230, 234, 240, 255), 1.0f);
                                const float center_x = m_pos.x + m_w * 0.5f;
                                dl->AddLine(ImVec2(center_x, m_pos.y), ImVec2(center_x, m_pos.y + m_h), ImColor(180, 185, 195, 255), 1.0f);

                                float bar_x = center_x + std::clamp(live_delta, -1.0f, 1.0f) * (m_w * 0.5f);
                                ImU32 bar_col = (route.destination <= modulation::ModulationDestination::LFO2_Depth) ?
                                                ImColor(217, 123, 13, 230) : ImColor(31, 97, 217, 230);
                                if (std::abs(bar_x - center_x) > 1.0f) {
                                    dl->AddRectFilled(ImVec2(std::min(center_x, bar_x), m_pos.y + 2.0f),
                                                      ImVec2(std::max(center_x, bar_x), m_pos.y + m_h - 2.0f),
                                                      bar_col, 1.0f);
                                }
                                char num_tag[16];
                                std::snprintf(num_tag, sizeof(num_tag), "%+.2f", live_delta);
                                dl->AddText(ImVec2(m_pos.x + 4.0f, m_pos.y + 1.0f), ImColor(26, 30, 40, 200), num_tag);

                                ImGui::Dummy(ImVec2(m_w, m_h));
                                ImGui::PopID();
                            }
                            ImGui::EndTable();
                        }
                    }
                    ImGui::EndChild();

                    ImGui::EndTabItem();
                }

                ImGui::EndTabBar();
            }
        }
        // ====================================================================
        // SESSION PERSISTENCE & OFFLINE BOUNCE MODAL DIALOGS
        // ====================================================================
        if (open_save_session_modal) {
            ImGui::OpenPopup("Save Project As");
            open_save_session_modal = false;
        }
        if (ImGui::BeginPopupModal("Save Project As", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "SAVE PROJECT SESSION (.JSON)");
            ImGui::Separator();
            ImGui::Spacing();

            ImGui::InputText("File Path", session_file_path, sizeof(session_file_path));
            ImGui::Spacing();

            if (ImGui::Button("SAVE PROJECT", ImVec2(130, 30))) {
                if (serialization::SessionSerializer::save_session_file(session_file_path, mixer, mixer.clock(), "Aethel Project")) {
                    session_status_msg = "Saved: " + std::string(session_file_path);
                } else {
                    session_status_msg = "Error saving session!";
                }
                session_status_time = std::chrono::steady_clock::now();
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("CANCEL", ImVec2(90, 30))) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        if (open_load_session_modal) {
            ImGui::OpenPopup("Load Project");
            open_load_session_modal = false;
        }
        if (ImGui::BeginPopupModal("Load Project", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "LOAD PROJECT SESSION (.JSON)");
            ImGui::Separator();
            ImGui::Spacing();

            ImGui::InputText("File Path", session_file_path, sizeof(session_file_path));
            ImGui::Spacing();

            if (ImGui::Button("LOAD PROJECT", ImVec2(130, 30))) {
                if (serialization::SessionSerializer::load_session_file(session_file_path, mixer, mixer.clock())) {
                    sync_ui_from_mixer();
                    session_status_msg = "Loaded: " + std::string(session_file_path);
                } else {
                    session_status_msg = "Error loading session!";
                }
                session_status_time = std::chrono::steady_clock::now();
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("CANCEL", ImVec2(90, 30))) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        if (open_bounce_modal) {
            ImGui::OpenPopup("Bounce Master to WAV");
            open_bounce_modal = false;
        }
        if (ImGui::BeginPopupModal("Bounce Master to WAV", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextColored(ImVec4(0.85f, 0.45f, 0.10f, 1.0f), "FASTER-THAN-REALTIME OFFLINE MASTER BOUNCE");
            ImGui::TextDisabled("Pure C++20 zero-hardware accelerated export with PDC latency flush");
            ImGui::Separator();
            ImGui::Spacing();

            ImGui::InputText("Output WAV Path", bounce_wav_path, sizeof(bounce_wav_path));
            ImGui::SliderInt("Length", &bounce_bars, 1, 32, "%d Bars");
            const char* bit_depths[] = { "16-Bit PCM", "24-Bit PCM (Studio Reference)", "32-Bit IEEE Float" };
            ImGui::Combo("Bit Depth", &bounce_bit_depth_idx, bit_depths, IM_ARRAYSIZE(bit_depths));
            ImGui::Checkbox("Peak Normalize", &bounce_normalize);
            if (bounce_normalize) {
                ImGui::SameLine();
                ImGui::SliderFloat("Target Peak", &bounce_peak_target, -6.0f, 0.0f, "%.1f dBFS");
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            if (ImGui::Button("EXPORT NOW", ImVec2(130, 32))) {
                do_offline_bounce();
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("CANCEL", ImVec2(90, 32))) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        if (open_save_rack_modal) {
            ImGui::OpenPopup("Save Rack Preset");
            open_save_rack_modal = false;
        }
        if (ImGui::BeginPopupModal("Save Rack Preset", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "SAVE DSP RACK PRESET (TRACK %d)", selected_track + 1);
            ImGui::Separator();
            ImGui::Spacing();

            ImGui::InputText("Preset File", rack_preset_file_path, sizeof(rack_preset_file_path));
            ImGui::Spacing();

            if (ImGui::Button("SAVE RACK", ImVec2(120, 28))) {
                auto* sel_trk = (selected_track == 0) ? trk0 : ((selected_track == 1) ? trk1 : ((selected_track == 2) ? trk2 : trk3));
                if (sel_trk && serialization::SessionSerializer::save_rack_preset_file(rack_preset_file_path, *sel_trk, sel_trk->name())) {
                    session_status_msg = "Rack saved: " + std::string(rack_preset_file_path);
                } else {
                    session_status_msg = "Error saving rack!";
                }
                session_status_time = std::chrono::steady_clock::now();
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("CANCEL", ImVec2(80, 28))) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        if (open_load_rack_modal) {
            ImGui::OpenPopup("Load Rack Preset");
            open_load_rack_modal = false;
        }
        if (ImGui::BeginPopupModal("Load Rack Preset", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "LOAD DSP RACK PRESET (TRACK %d)", selected_track + 1);
            ImGui::Separator();
            ImGui::Spacing();

            ImGui::InputText("Preset File", rack_preset_file_path, sizeof(rack_preset_file_path));
            ImGui::Spacing();

            if (ImGui::Button("LOAD RACK", ImVec2(120, 28))) {
                auto* sel_trk = (selected_track == 0) ? trk0 : ((selected_track == 1) ? trk1 : ((selected_track == 2) ? trk2 : trk3));
                if (sel_trk && serialization::SessionSerializer::load_rack_preset_file(rack_preset_file_path, *sel_trk, kSampleRate)) {
                    session_status_msg = "Rack loaded: " + std::string(rack_preset_file_path);
                } else {
                    session_status_msg = "Error loading rack!";
                }
                session_status_time = std::chrono::steady_clock::now();
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("CANCEL", ImVec2(80, 28))) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        // ------------------------------------------------------------
        // HARDWARE AUDIO I/O MANAGER MODAL DIALOG
        // ------------------------------------------------------------
        if (open_hardware_io_modal) {
            ImGui::OpenPopup("Hardware Audio I/O Manager");
            open_hardware_io_modal = false;
        }

        ImGui::SetNextWindowSize(ImVec2(860, 640), ImGuiCond_FirstUseEver);
        if (ImGui::BeginPopupModal("Hardware Audio I/O Manager", nullptr, ImGuiWindowFlags_None)) {
            ImGui::TextColored(ImVec4(0.12f, 0.45f, 0.95f, 1.0f), "HARDWARE AUDIO I/O MATRIX & DEVICE ROUTER");
            ImGui::SameLine();
            if (pw_online) {
                ImGui::TextColored(ImVec4(0.20f, 0.85f, 0.35f, 1.0f), "[ PIPEWIRE ACTIVE (48 kHz / 32-bit Float) ]");
            } else {
                ImGui::TextColored(ImVec4(0.90f, 0.25f, 0.25f, 1.0f), "[ PIPEWIRE OFFLINE ]");
            }
            ImGui::Separator();
            ImGui::Spacing();

            // Top Action Bar
            if (ImGui::Button("REFRESH DISCOVERY", ImVec2(160, 26))) {
                pw.refresh_discovery();
            }
            ImGui::SameLine(0, 15);
            auto hw_inputs = pw.get_hardware_inputs();
            auto app_sources = pw.get_app_sources();
            auto sinks = pw.get_available_sinks();
            ImGui::TextDisabled("Found: %zu HW Inputs (%zu App Streams) | %zu Output Sinks (DACs)",
                                hw_inputs.size(), app_sources.size(), sinks.size());

            ImGui::SameLine(ImGui::GetWindowWidth() - 90);
            if (ImGui::Button("CLOSE", ImVec2(75, 26))) {
                ImGui::CloseCurrentPopup();
            }

            ImGui::Spacing();
            ImGui::Separator();

            // SECTION 1: MASTER AUDIO OUTPUT (DAC ROUTING)
            ImGui::TextColored(ImVec4(0.85f, 0.48f, 0.05f, 1.0f), "1. MASTER AUDIO OUTPUT (DAC / SINK ROUTING)");
            std::string active_sink_disp = pw.active_master_sink_display_name();
            std::string active_sink_node = pw.active_master_sink_node_name();
            if (active_sink_disp.empty()) {
                ImGui::Text("Active Master DAC: [None / Disconnected]");
            } else {
                ImGui::Text("Active Master DAC: ");
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.12f, 0.70f, 0.35f, 1.0f), "%s", active_sink_disp.c_str());
                ImGui::SameLine();
                ImGui::TextDisabled("(%s)", active_sink_node.c_str());
            }

            static int sel_sink_idx = 0;
            if (!sinks.empty()) {
                if (sel_sink_idx >= static_cast<int>(sinks.size())) sel_sink_idx = 0;
                ImGui::SetNextItemWidth(380);
                if (ImGui::BeginCombo("##MasterSinkCombo", sinks[sel_sink_idx].display_name.c_str())) {
                    for (size_t i = 0; i < sinks.size(); ++i) {
                        bool is_sel = (sel_sink_idx == static_cast<int>(i));
                        char sink_item[256];
                        std::snprintf(sink_item, sizeof(sink_item), "%s (Node %u)", sinks[i].display_name.c_str(), sinks[i].node_id);
                        if (ImGui::Selectable(sink_item, is_sel)) {
                            sel_sink_idx = static_cast<int>(i);
                        }
                    }
                    ImGui::EndCombo();
                }
                ImGui::SameLine();
                if (ImGui::Button("SWITCH MASTER DAC", ImVec2(160, 24))) {
                    pw.connect_master_to_sink(sinks[sel_sink_idx].node_name);
                }
                ImGui::SameLine();
                if (ImGui::Button("DISCONNECT DAC", ImVec2(130, 24))) {
                    pw.disconnect_master_output();
                }
            } else {
                ImGui::TextDisabled("No output audio sinks discovered.");
            }

            ImGui::Spacing();
            ImGui::Separator();

            // SECTION 2: MULTI-CHANNEL HARDWARE INPUT ROUTING MATRIX
            ImGui::TextColored(ImVec4(0.85f, 0.48f, 0.05f, 1.0f), "2. HARDWARE & APPLICATION INPUT ROUTING MATRIX");

            if (ImGui::BeginTabBar("HardwareIoTabBar")) {
                if (ImGui::BeginTabItem("  PHYSICAL HARDWARE INPUTS (ADCs)  ")) {
                    if (hw_inputs.empty()) {
                        ImGui::Spacing();
                        ImGui::TextDisabled("No physical hardware capture devices detected.");
                    } else {
                        if (ImGui::BeginTable("HwInputsTable", 6, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY, ImVec2(0, 180))) {
                            ImGui::TableSetupColumn("Device / Port", ImGuiTableColumnFlags_WidthStretch);
                            ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 65.0f);
                            ImGui::TableSetupColumn("Node ID", ImGuiTableColumnFlags_WidthFixed, 60.0f);
                            ImGui::TableSetupColumn("Physical Port", ImGuiTableColumnFlags_WidthFixed, 140.0f);
                            ImGui::TableSetupColumn("Direct Track Patch", ImGuiTableColumnFlags_WidthFixed, 220.0f);
                            ImGui::TableSetupColumn("Action", ImGuiTableColumnFlags_WidthFixed, 60.0f);
                            ImGui::TableHeadersRow();

                            Track* track_arr[4] = { trk0, trk1, trk2, trk3 };

                            for (size_t i = 0; i < hw_inputs.size(); ++i) {
                                const auto& src = hw_inputs[i];
                                ImGui::TableNextRow();
                                ImGui::PushID(static_cast<int>(i));

                                // Col 0: Device Name
                                ImGui::TableSetColumnIndex(0);
                                ImGui::TextUnformatted(src.display_name.c_str());

                                // Col 1: Type
                                ImGui::TableSetColumnIndex(1);
                                if (src.is_mono) {
                                    ImGui::TextColored(ImVec4(0.85f, 0.65f, 0.15f, 1.0f), "MONO");
                                } else {
                                    ImGui::TextColored(ImVec4(0.20f, 0.70f, 0.85f, 1.0f), "STEREO");
                                }

                                // Col 2: Node ID
                                ImGui::TableSetColumnIndex(2);
                                ImGui::Text("%u", src.node_id);

                                // Col 3: Physical Port
                                ImGui::TableSetColumnIndex(3);
                                ImGui::TextDisabled("%s", src.physical_port_name.empty() ? src.port_l.c_str() : src.physical_port_name.c_str());

                                // Col 4: Direct Track Patch Buttons [-> T1] [-> T2] [-> T3] [-> T4]
                                ImGui::TableSetColumnIndex(4);
                                for (uint32_t t = 1; t <= 4; ++t) {
                                    auto linked_src = pw.get_track_source(t);
                                    bool is_active = linked_src.has_value() &&
                                                     linked_src->node_name == src.node_name &&
                                                     linked_src->port_l == src.port_l;
                                    char btn_lbl[32];
                                    std::snprintf(btn_lbl, sizeof(btn_lbl), "T%u##hw_%zu", t, i);
                                    if (is_active) {
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.65f, 0.30f, 1.0f));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                    }
                                    if (ImGui::Button(btn_lbl, ImVec2(46, 20))) {
                                        if (is_active) {
                                            pw.unlink_all_for_track(t);
                                        } else {
                                            if (track_arr[t - 1]) track_arr[t - 1]->set_input_mode(TrackInputMode::PipeWireStream);
                                            pw.link_source_to_track(src, t);
                                        }
                                    }
                                    if (is_active) ImGui::PopStyleColor(2);
                                    if (ImGui::IsItemHovered()) {
                                        ImGui::SetTooltip("%s Track %u to %s", is_active ? "Unlink" : "Patch", t, src.display_name.c_str());
                                    }
                                    if (t < 4) ImGui::SameLine();
                                }

                                // Col 5: Clear / Unlink
                                ImGui::TableSetColumnIndex(5);
                                if (ImGui::SmallButton("UNLINK")) {
                                    for (uint32_t t = 1; t <= 4; ++t) {
                                        auto linked_src = pw.get_track_source(t);
                                        if (linked_src.has_value() &&
                                            linked_src->node_name == src.node_name &&
                                            linked_src->port_l == src.port_l) {
                                            pw.unlink_all_for_track(t);
                                        }
                                    }
                                }

                                ImGui::PopID();
                            }
                            ImGui::EndTable();
                        }
                    }
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("  APPLICATION STREAMS (JACK / PIPEWIRE)  ")) {
                    if (app_sources.empty()) {
                        ImGui::Spacing();
                        ImGui::TextDisabled("No external audio applications detected.");
                    } else {
                        if (ImGui::BeginTable("AppSourcesTable", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY, ImVec2(0, 180))) {
                            ImGui::TableSetupColumn("Application / Client", ImGuiTableColumnFlags_WidthStretch);
                            ImGui::TableSetupColumn("Node ID", ImGuiTableColumnFlags_WidthFixed, 60.0f);
                            ImGui::TableSetupColumn("Ports (L/R)", ImGuiTableColumnFlags_WidthFixed, 140.0f);
                            ImGui::TableSetupColumn("Direct Track Patch", ImGuiTableColumnFlags_WidthFixed, 220.0f);
                            ImGui::TableSetupColumn("Action", ImGuiTableColumnFlags_WidthFixed, 60.0f);
                            ImGui::TableHeadersRow();

                            Track* track_arr[4] = { trk0, trk1, trk2, trk3 };

                            for (size_t i = 0; i < app_sources.size(); ++i) {
                                const auto& src = app_sources[i];
                                ImGui::TableNextRow();
                                ImGui::PushID(static_cast<int>(1000 + i));

                                // Col 0: App Name
                                ImGui::TableSetColumnIndex(0);
                                ImGui::TextUnformatted(src.display_name.c_str());

                                // Col 1: Node ID
                                ImGui::TableSetColumnIndex(1);
                                ImGui::Text("%u", src.node_id);

                                // Col 2: Ports
                                ImGui::TableSetColumnIndex(2);
                                ImGui::TextDisabled("%s / %s", src.port_l.c_str(), src.port_r.c_str());

                                // Col 3: Direct Track Patch Buttons
                                ImGui::TableSetColumnIndex(3);
                                for (uint32_t t = 1; t <= 4; ++t) {
                                    auto linked_src = pw.get_track_source(t);
                                    bool is_active = linked_src.has_value() &&
                                                     linked_src->node_name == src.node_name &&
                                                     linked_src->port_l == src.port_l;
                                    char btn_lbl[32];
                                    std::snprintf(btn_lbl, sizeof(btn_lbl), "T%u##app_%zu", t, i);
                                    if (is_active) {
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.65f, 0.30f, 1.0f));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                    }
                                    if (ImGui::Button(btn_lbl, ImVec2(46, 20))) {
                                        if (is_active) {
                                            pw.unlink_all_for_track(t);
                                        } else {
                                            if (track_arr[t - 1]) track_arr[t - 1]->set_input_mode(TrackInputMode::PipeWireStream);
                                            pw.link_source_to_track(src, t);
                                        }
                                    }
                                    if (is_active) ImGui::PopStyleColor(2);
                                    if (ImGui::IsItemHovered()) {
                                        ImGui::SetTooltip("%s Track %u to %s", is_active ? "Unlink" : "Patch", t, src.display_name.c_str());
                                    }
                                    if (t < 4) ImGui::SameLine();
                                }

                                // Col 4: Clear / Unlink
                                ImGui::TableSetColumnIndex(4);
                                if (ImGui::SmallButton("UNLINK")) {
                                    for (uint32_t t = 1; t <= 4; ++t) {
                                        auto linked_src = pw.get_track_source(t);
                                        if (linked_src.has_value() &&
                                            linked_src->node_name == src.node_name &&
                                            linked_src->port_l == src.port_l) {
                                            pw.unlink_all_for_track(t);
                                        }
                                    }
                                }

                                ImGui::PopID();
                            }
                            ImGui::EndTable();
                        }
                    }
                    ImGui::EndTabItem();
                }
                ImGui::EndTabBar();
            }

            ImGui::Spacing();
            ImGui::Separator();

            // SECTION 3: CHANNEL STRIP INPUT STAGING & METERING OVERVIEW
            ImGui::TextColored(ImVec4(0.85f, 0.48f, 0.05f, 1.0f), "3. CHANNEL STRIP INPUT STAGING & REAL-TIME METERING");
            if (ImGui::BeginTable("TrackInputOverviewTable", 7, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg, ImVec2(0, 140))) {
                ImGui::TableSetupColumn("Track", ImGuiTableColumnFlags_WidthFixed, 100.0f);
                ImGui::TableSetupColumn("Input Mode", ImGuiTableColumnFlags_WidthFixed, 90.0f);
                ImGui::TableSetupColumn("Source Routing", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Trim Gain", ImGuiTableColumnFlags_WidthFixed, 120.0f);
                ImGui::TableSetupColumn("Phase", ImGuiTableColumnFlags_WidthFixed, 45.0f);
                ImGui::TableSetupColumn("Input Peak", ImGuiTableColumnFlags_WidthFixed, 80.0f);
                ImGui::TableSetupColumn("Reset", ImGuiTableColumnFlags_WidthFixed, 55.0f);
                ImGui::TableHeadersRow();

                Track* track_arr[4] = { trk0, trk1, trk2, trk3 };
                const char* t_names[4] = { "1: Kick / 808", "2: Acid 303", "3: Vocal", "4: Drums" };
                const char* m_labels[4] = { "CLIP", "PIPEWIRE", "MERGE", "AOIP" };

                for (uint32_t t = 0; t < 4; ++t) {
                    Track* trk = track_arr[t];
                    if (!trk) continue;

                    ImGui::TableNextRow();
                    ImGui::PushID(static_cast<int>(5000 + t));

                    // Col 0: Track
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%s", t_names[t]);

                    // Col 1: Mode
                    ImGui::TableSetColumnIndex(1);
                    int cur_m = static_cast<int>(trk->input_mode());
                    ImGui::SetNextItemWidth(80);
                    if (ImGui::Combo("##TblMode", &cur_m, m_labels, 4)) {
                        trk->set_input_mode(static_cast<TrackInputMode>(cur_m));
                        if (static_cast<TrackInputMode>(cur_m) == TrackInputMode::InternalClip) {
                            pw.unlink_all_for_track(t + 1);
                        }
                    }

                    // Col 2: Source Routing
                    ImGui::TableSetColumnIndex(2);
                    auto linked_src = pw.get_track_source(t + 1);
                    if (linked_src.has_value()) {
                        ImGui::TextColored(ImVec4(0.20f, 0.85f, 0.35f, 1.0f), "● [%s] %s",
                                           linked_src->is_hardware_capture ? (linked_src->is_mono ? "HW-1ch" : "HW-2ch") : "APP",
                                           linked_src->display_name.c_str());
                    } else if (trk->input_mode() == TrackInputMode::NetworkAoip) {
                        ImGui::TextColored(ImVec4(0.20f, 0.70f, 0.85f, 1.0f), "Dante Ch %d/%d (UDP:4848)", t * 2 + 1, t * 2 + 2);
                    } else if (trk->input_mode() == TrackInputMode::InternalClip) {
                        ImGui::TextDisabled("Internal Clip Engine");
                    } else {
                        ImGui::TextDisabled("— Unlinked —");
                    }

                    // Col 3: Trim Gain
                    ImGui::TableSetColumnIndex(3);
                    float cur_trim_db = ui::linear_to_db(trk->input_gain());
                    if (cur_trim_db < -24.0f) cur_trim_db = -24.0f;
                    if (cur_trim_db > 24.0f) cur_trim_db = 24.0f;
                    ImGui::SetNextItemWidth(100);
                    if (ImGui::SliderFloat("##TblTrim", &cur_trim_db, -24.0f, 24.0f, "%+.1f dB")) {
                        trk->set_input_gain(ui::db_to_linear(cur_trim_db));
                    }
                    if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                        trk->set_input_gain(1.0f);
                    }

                    // Col 4: Phase
                    ImGui::TableSetColumnIndex(4);
                    bool ph = trk->input_phase_invert();
                    if (ph) {
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.45f, 0.10f, 1.0f));
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                    }
                    if (ImGui::Button("Ø##TblPh", ImVec2(28, 20))) {
                        trk->set_input_phase_invert(!ph);
                    }
                    if (ph) ImGui::PopStyleColor(2);
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Pre-insert Phase Invert");

                    // Col 5: Input Peak Meter
                    ImGui::TableSetColumnIndex(5);
                    auto in_m = trk->input_meter();
                    float pk = std::max(in_m.first, in_m.second);
                    float pk_db = ui::linear_to_db(pk);
                    ImVec4 pk_col = (pk >= 0.999f) ? ImVec4(0.95f, 0.15f, 0.15f, 1.0f) :
                                    ((pk >= 0.005f) ? ImVec4(0.20f, 0.85f, 0.35f, 1.0f) : ImVec4(0.45f, 0.50f, 0.55f, 0.6f));
                    ImGui::TextColored(pk_col, "%+.1f dB", pk_db);

                    // Col 6: Reset / Unlink
                    ImGui::TableSetColumnIndex(6);
                    if (ImGui::SmallButton("RESET")) {
                        trk->set_input_gain(1.0f);
                        trk->set_input_phase_invert(false);
                        pw.unlink_all_for_track(t + 1);
                    }

                    ImGui::PopID();
                }
                ImGui::EndTable();
            }

            ImGui::Spacing();
            if (ImGui::Button("DONE", ImVec2(100, 28))) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

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
