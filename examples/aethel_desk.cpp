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
#include "audio_core/dsp/lookahead_dnl.hpp"
#include "audio_core/dsp/transient_shaper.hpp"
#include "audio_core/network/aoip_receiver.hpp"
#include "audio_core/serialization/session_serializer.hpp"
#include "audio_core/engine.hpp"
#include "audio_core/midi/hardware_midi_receiver.hpp"
#include "audio_core/midi/midi_sync.hpp"
#include "audio_core/midi/midi_learn_router.hpp"
#include "audio_core/routing/mseg_automation_bridge.hpp"
#include "backends/pipewire/pipewire_backend.hpp"
#include "backends/desktop/desktop_backend.hpp"
#include "audio_core/threading/realtime_scheduler.hpp"
#include "audio_core/sampling/sample_asset_manager.hpp"
#include "audio_core/undo/undo_manager.hpp"
#include "audio_core/sequencer/tracker_timing.hpp"
#include "audio_core/sequencer/instrument_phrase.hpp"
#include "audio_core/tuning/scales_and_tuning.hpp"
#include <mutex>


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

// Static thread-safe state for GLFW Drag & Drop (multi-file audio & .aethel project bundles)
static std::vector<std::string> g_dropped_paths;
static std::mutex g_drop_mutex;
static bool g_has_dropped_files = false;

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

    // Query monitor work area for adaptive window sizing
    GLFWmonitor* primary_mon = glfwGetPrimaryMonitor();
    int work_x = 0, work_y = 0, work_w = 1440, work_h = 900;
    if (primary_mon) {
        glfwGetMonitorWorkarea(primary_mon, &work_x, &work_y, &work_w, &work_h);
    }
    int win_target_w = std::min(1440, work_w - 24);
    int win_target_h = std::min(900, work_h - 48);
    if (win_target_w < 1024) win_target_w = work_w;
    if (win_target_h < 640) win_target_h = work_h;

    GLFWwindow* window = glfwCreateWindow(win_target_w, win_target_h, "AETHEL AUDIO DESK // SOVEREIGN ENGINE [RT 48kHz]", nullptr, nullptr);
    if (!window) {
        std::cerr << "[Error] Failed to create GLFW window!" << std::endl;
        glfwTerminate();
        return 1;
    }
    if (primary_mon) {
        glfwSetWindowPos(window, work_x + (work_w - win_target_w) / 2, work_y + (work_h - win_target_h) / 2);
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable V-Sync (60/120 Hz refresh)

    // Install GLFW Drag & Drop Callback for Audio Import (.wav) & Project Bundles (.aethel / .json)
    glfwSetDropCallback(window, [](GLFWwindow*, int count, const char** paths) {
        if (count > 0 && paths) {
            std::lock_guard<std::mutex> lock(g_drop_mutex);
            g_dropped_paths.clear();
            for (int i = 0; i < count; ++i) {
                if (paths[i]) {
                    g_dropped_paths.emplace_back(paths[i]);
                }
            }
            g_has_dropped_files = true;
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
    auto* trk3 = mixer.allocate_track("Percussion / Hats");

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

    // Route tracks to Submix buses by default (Clean Zähl AM1 Summing Topology):
    // Track 1 (Kick / 808 Sub)    -> Bus A (Drums)
    // Track 2 (Acid 303 Lead)     -> Bus B (Music)
    // Track 3 (Vocal Chops)       -> Bus B (Music)
    // Track 4 (Percussion / Hats) -> Bus A (Drums)
    trk0->set_target_bus(bus_drums->id());
    trk1->set_target_bus(bus_music->id());
    trk2->set_target_bus(bus_music->id());
    trk3->set_target_bus(bus_drums->id());

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

    // Load Acid Loop AudioClip into Track 1
    std::vector<float> acid_waveform = generate_synthetic_acid_loop(48000 * 4);
    auto acid_clip = std::make_shared<sampling::AudioClip>("Acid 303", kSampleRate, 2, static_cast<uint32_t>(acid_waveform.size()));
    for (size_t i = 0; i < acid_waveform.size(); ++i) {
        acid_clip->channel(0)[i] = acid_waveform[i];
        acid_clip->channel(1)[i] = acid_waveform[i];
    }

    // Load Vocal Chops AudioClip into Track 2
    std::vector<float> vocal_waveform = generate_synthetic_vocal_chops(48000 * 4);
    auto vocal_clip = std::make_shared<sampling::AudioClip>("Vocal Chops", kSampleRate, 2, static_cast<uint32_t>(vocal_waveform.size()));
    for (size_t i = 0; i < vocal_waveform.size(); ++i) {
        vocal_clip->channel(0)[i] = vocal_waveform[i];
        vocal_clip->channel(1)[i] = vocal_waveform[i];
    }

    // Initialize Track 3 (Percussion / Bus audio)
    auto perc_clip = std::make_shared<sampling::AudioClip>("Drums / Bus", kSampleRate, 2, static_cast<uint32_t>(sample_waveform.size()));
    for (size_t i = 0; i < sample_waveform.size(); ++i) {
        perc_clip->channel(0)[i] = sample_waveform[i] * 0.7f;
        perc_clip->channel(1)[i] = sample_waveform[i] * 0.7f;
    }

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
            std::cout << "[AudioDriver] Primary driver active: Native PipeWire 0.3 stream running!" << std::endl;
        }
    }

    // Resilient Fallback to DesktopBackend (Miniaudio + ALSA + RTKit SCHED_FIFO)
    std::unique_ptr<DesktopBackend> desktop_backend;
    bool desktop_online = false;
    AudioBuffer desktop_render_buf(2, 2048);

    if (!pw_online) {
        std::cout << "[AudioDriver] PipeWire offline or unavailable. Initiating resilient fallback to DesktopBackend (ALSA + RTKit)..." << std::endl;
        desktop_backend = std::make_unique<DesktopBackend>();
        desktop_backend->set_callback([&mixer, &desktop_render_buf](Sample* output, uint32_t frames, uint32_t channels) {
            uint32_t safe_frames = std::min(frames, desktop_render_buf.num_frames());
            auto view = desktop_render_buf.view();
            mixer.render(view);
            for (uint32_t f = 0; f < safe_frames; ++f) {
                for (uint32_t ch = 0; ch < channels; ++ch) {
                    output[f * channels + ch] = (ch < 2) ? view.channel(ch)[f] : 0.0f;
                }
            }
        });

        if (desktop_backend->init(kSampleRate, 2, kBlockFrames) && desktop_backend->start()) {
            desktop_online = true;
            std::cout << "[AudioDriver] Resilient fallback SUCCESS: DesktopBackend active with RTKit (SCHED_FIFO Prio 20)!" << std::endl;
        } else {
            std::cerr << "[AudioDriver] CRITICAL WARNING: Neither PipeWire nor DesktopBackend could be started!" << std::endl;
        }
    }


    // Workspace UI & Transport State
    bool is_playing = false;
    float bpm = 126.0f;
    float playhead_seconds = 0.0f;
    bool loop_active = true;
    float loop_start_bar = 0.0f;
    float loop_bars = 4.0f;

    // Punch-In / Punch-Out Overdub Engine State
    static bool g_punch_enabled = false;
    static bool g_punch_link_to_loop = true;
    static float g_punch_in_bar = 0.0f;
    static float g_punch_out_bar = 4.0f;
    static bool g_punch_is_recording = false;
    static uint32_t g_punch_take_counter = 1;

    auto get_seconds_per_bar = [](float current_bpm) noexcept -> float {
        return (60.0f / std::max(20.0f, current_bpm)) * 4.0f;
    };
    auto get_loop_length_seconds = [&](float current_bpm) noexcept -> float {
        return loop_bars * get_seconds_per_bar(current_bpm);
    };
    auto get_loop_start_seconds = [&](float current_bpm) noexcept -> float {
        return loop_start_bar * get_seconds_per_bar(current_bpm);
    };
    int selected_track = 0;
    int active_slice = 0;

    // Multi-Track Clip Pool & Original Backups
    std::shared_ptr<sampling::AudioClip> track_clips[4] = { drum_clip, acid_clip, vocal_clip, perc_clip };
    std::shared_ptr<sampling::AudioClip> track_clips_orig[4] = { drum_clip, acid_clip, vocal_clip, perc_clip };

    // Register initial synthetic clips into Sononym SampleAssetPool
    sampling::SampleAssetPool::instance().register_asset(drum_clip, "kick_808_sub.wav");
    sampling::SampleAssetPool::instance().register_asset(acid_clip, "acid_303_lead.wav");
    sampling::SampleAssetPool::instance().register_asset(vocal_clip, "vocal_chops.wav");
    sampling::SampleAssetPool::instance().register_asset(perc_clip, "percussion_loop.wav");

    // Arranger Multi-Clip Track Lanes & Tool State
    static std::vector<sequencer::ArrangerClipInstance> track_arranger_clips[4];
    static int g_selected_clip_idx = 0;
    static bool g_arranger_initialized = false;
    static int g_switch_to_main_tab = -1; // -1 = none, 0 = Arranger, 2 = Tracker

    enum class ArrangerTool {
        Select = 0,
        Razor = 1,
        Slip = 2,
        Stretch = 3,
        Warp = 4
    };
    static ArrangerTool g_arranger_tool = ArrangerTool::Select;

    enum class ArrangerDragMode {
        None,
        MoveClip,
        TrimStart,
        TrimEnd,
        FadeIn,
        FadeOut,
        SlipEdit,
        ScrubTimeline,
        StretchStart,
        StretchEnd,
        WarpPinDrag
    };
    static ArrangerDragMode g_arranger_drag_mode = ArrangerDragMode::None;
    enum class ArrangerRulerDragTarget {
        None,
        LoopLeft,
        LoopRight,
        LoopBody,
        PunchIn,
        PunchOut,
        Scrub
    };
    static ArrangerRulerDragTarget g_ruler_drag_target = ArrangerRulerDragTarget::None;
    static float g_ruler_drag_orig_start = 0.0f;
    static float g_ruler_drag_orig_len = 4.0f;
    static float g_ruler_drag_orig_punch_in = 0.0f;
    static float g_ruler_drag_orig_punch_out = 4.0f;
    static int g_arranger_drag_track = -1;
    static int g_arranger_drag_clip_idx = -1;
    static int g_arranger_drag_hover_track = -1;
    static uint32_t g_arranger_drag_warp_pin_id = 0;
    static float g_arranger_drag_orig_pinned_bar = 0.0f;
    static float g_arranger_drag_start_mouse_x = 0.0f;
    static float g_arranger_drag_orig_start_bar = 0.0f;
    static float g_arranger_drag_orig_len_bars = 4.0f;
    static float g_arranger_drag_orig_offset_bars = 0.0f;
    static float g_arranger_drag_orig_fade_in = 0.0f;
    static float g_arranger_drag_orig_fade_out = 0.0f;
    static undo::UndoManager g_undo_mgr(128);
    static std::vector<sequencer::ArrangerClipInstance> g_pre_drag_clips[4];

    // Renoise Tracker Timing State
    static sequencer::TrackerTiming g_tracker_timing;
    static int g_tracker_lpb_idx = 0; // 0=4, 1=8, 2=12, 3=16, 4=32
    static const uint16_t g_lpb_options[] = { 4, 8, 12, 16, 32 };
    static int g_tracker_tpl_idx = 1; // 12
    static const uint16_t g_tpl_options[] = { 8, 12, 16, 24 };

    // Scale & Tuning Engine State
    static tuning::ScaleType g_selected_scale = tuning::ScaleType::Major;
    static int g_scale_type_idx = 1; // 1 = Major
    static int g_scale_root = 0; // 0=C, 1=C#, 2=D, 3=D#, 4=E, 5=F, 6=F#, 7=G, 8=G#, 9=A, 10=A#, 11=B
    static tuning::SnapMode g_scale_snap_mode = tuning::SnapMode::Nearest;
    static int g_scale_snap_mode_idx = 0;
    static bool g_scale_snap_enabled = false;
    static tuning::TuningTable g_tuning_table;
    static int g_tuning_preset_idx = 0;
    static char g_scl_import_buffer[2048] = "";

    // Instrument Phrases & Multi-Mode Arpeggiator State
    static sequencer::InstrumentPhraseBank g_phrase_bank;
    static sequencer::InstrumentPhrasePlayer g_phrase_player;
    static int g_active_phrase_idx = 0;
    static bool g_phrases_initialized = false;
    if (!g_phrases_initialized) {
        g_phrases_initialized = true;
        auto& p0 = g_phrase_bank.phrases[0];
        p0.name = "Acid Triad Arp";
        p0.num_lines = 16;
        p0.resize(16);
        p0.playback_mode = sequencer::PhraseArpMode::ArpUp;
        p0.build_arpeggiator_pattern({0, 3, 7, 10, 12, 10, 7, 3}, 110);

        auto& p1 = g_phrase_bank.add_phrase("Jungle Break Drill", 16);
        p1.playback_mode = sequencer::PhraseArpMode::Keymap;
        p1.key_trigger_min = 48;
        p1.key_trigger_max = 48;
        p1.build_drum_break_variation(0, 1, 2, true);

        auto& p2 = g_phrase_bank.add_phrase("Sub Bass Groove", 16);
        p2.playback_mode = sequencer::PhraseArpMode::Transposed;
        p2.set_note(0, 0, -12, 120, 0, 10);
        p2.set_note(3, 0, -12, 90, 6, 6);
        p2.set_note(6, 0, -5, 100, 0, 8);
        p2.set_note(8, 0, -12, 115, 0, 10);
        p2.set_note(11, 0, -7, 95, 0, 8);
        p2.set_note(14, 0, -10, 105, 3, 6);
    }

    // Realtime Track Phrase / Arpeggiator Routing State
    static bool g_track_phrase_arp_enabled[4] = { false, true, false, false }; // Default Track 2 (Acid Lead) armed with phrase arp
    static int g_track_phrase_assigned[4] = { 0, 0, 1, 0 }; // Phrase index mapped to track

    // Unified Pattern Studio Mode (FL Studio Channel Rack vs Renoise Multi-Tracker vs GridPie Live Matrix)
    enum class SequencerViewMode : uint8_t {
        FlChannelRack = 0,       // Horizontal Multi-Track Step Rack (FL Studio style)
        RenoiseMultiTracker = 1, // Vertical Multi-Track Pattern Matrix (Renoise style)
        GridPieLiveMatrix = 2    // Renoise GridPie / Ableton-style Live Remix Matrix
    };
    static SequencerViewMode g_seq_view_mode = SequencerViewMode::FlChannelRack;
    static bool g_seq_show_phrase_lab = false;
    static bool g_tracker_auto_scroll = true;
    static int g_tracker_last_scrolled_line = -1;
    static int g_tracker_lines_preset = 16;

    // Renoise GridPie Scene Presets & Live Remix State
    struct GridPieScene {
        char name[32];
        uint32_t track_patterns[4];
    };
    static std::array<GridPieScene, 4> g_gridpie_scenes = {{
        { "Intro Groove", {0, 0, 0, 0} },
        { "Main Drop",    {1, 1, 1, 1} },
        { "Half-Time",    {2, 2, 2, 2} },
        { "Breakdown",    {3, 3, 3, 3} }
    }};
    static int g_gridpie_active_scene = 0;

    auto sync_step_to_phrase = [](const sequencer::Pattern& pat, sequencer::InstrumentPhrase& phr, uint32_t tpl) {
        phr.num_lines = std::clamp(pat.num_steps, 16u, 64u);
        phr.resize(phr.num_lines);
        for (size_t i = 0; i < phr.num_lines && i < pat.kMaxSteps; ++i) {
            const auto& step = pat.steps[i];
            auto& col = phr.lines[i].columns[0];
            col.active = step.active;
            if (step.active) {
                col.slice_id = static_cast<uint16_t>(step.slice_id);
                col.velocity = static_cast<uint8_t>(std::clamp(std::round(step.velocity * 127.0f), 1.0f, 127.0f));
                col.probability = step.probability;
                col.pan = static_cast<int8_t>(std::clamp(std::round(step.pan * 63.0f), -64.0f, 63.0f));
                float norm_delay = std::clamp(step.micro_timing + 0.5f, 0.0f, 0.999f);
                col.delay_ticks = static_cast<uint8_t>(std::floor(norm_delay * static_cast<float>(tpl)));
                col.cutoff_offset = (step.filter_cutoff < 19999.0f) ? std::clamp((step.filter_cutoff - 10000.0f) / 10000.0f, -1.0f, 1.0f) : 0.0f;
                col.res_offset = std::clamp((step.filter_res - 0.707f) / 5.0f, -1.0f, 1.0f);
                if (step.pitch_ratio > 0.01f && std::abs(step.pitch_ratio - 1.0f) > 1e-3f) {
                    col.note_offset = static_cast<int8_t>(std::clamp(std::round(12.0f * std::log2(step.pitch_ratio)), -60.0f, 60.0f));
                } else {
                    col.note_offset = 0;
                }
                col.is_slice = true;
            }
        }
    };

    auto sync_phrase_to_step = [](const sequencer::InstrumentPhrase& phr, sequencer::Pattern& pat, uint32_t tpl) {
        pat.num_steps = std::clamp(phr.num_lines, 16u, static_cast<uint32_t>(pat.kMaxSteps));
        for (size_t i = 0; i < pat.num_steps && i < phr.lines.size(); ++i) {
            const auto& col = phr.lines[i].columns[0];
            auto& step = pat.steps[i];
            step.active = col.active;
            if (col.active) {
                step.slice_id = col.slice_id;
                step.velocity = static_cast<float>(col.velocity) / 127.0f;
                step.probability = col.probability;
                step.pan = static_cast<float>(col.pan) / 63.0f;
                step.micro_timing = (static_cast<float>(col.delay_ticks) / static_cast<float>(std::max(1u, tpl))) - 0.5f;
                if (col.cutoff_offset != 0.0f) {
                    step.filter_cutoff = std::clamp(10000.0f + col.cutoff_offset * 10000.0f, 20.0f, 20000.0f);
                }
                if (col.res_offset != 0.0f) {
                    step.filter_res = std::clamp(0.707f + col.res_offset * 5.0f, 0.1f, 10.0f);
                }
                if (col.note_offset != 0) {
                    step.pitch_ratio = std::pow(2.0f, static_cast<float>(col.note_offset) / 12.0f);
                }
            }
        }
    };

    // Arranger Viewport Navigation State (Continuous Zoom & Pan)
    static float g_arranger_visible_bars = 16.0f; // Default 16 visible bars
    static float g_arranger_scroll_bar = 0.0f;    // Leftmost visible bar
    constexpr float g_arranger_max_bars = 128.0f; // Max horizontal timeline extent
    constexpr float g_arranger_min_bars = 2.0f;   // Max zoom in (2 bars across canvas)

    // Self-Contained Project Bundle & Sononym Library UI State
    bool open_save_bundle_modal = false;
    bool open_load_bundle_modal = false;
    char bundle_dir_path[512] = "/tmp/aethel_session.aethel";
    char asset_search_filter[128] = "";
    int asset_cat_filter = 0; // 0=All, 1=Percussion, 2=Bass/Sub, 3=Melodic/Hook

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

    auto sync_arranger_to_mixer = [&](int t) {
        if (t < 0 || t >= 4) return;
        Track* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
        if (!trk) return;
        trk->arranger().clear_clips();
        for (const auto& c : track_arranger_clips[t]) {
            trk->arranger().add_clip(c);
        }
    };

    if (!g_arranger_initialized) {
        g_arranger_initialized = true;
        // Track 0 (Drums): Audio Clip (Bars 0-4) + Pattern Clip (Bars 4-8)
        track_arranger_clips[0].push_back(sequencer::ArrangerClipInstance{
            .id = 1, .name = "Kick / 808 Sub", .clip = drum_clip,
            .type = sequencer::ArrangerClipType::Audio,
            .start_bar = 0.0f, .len_bars = 4.0f, .offset_bars = 0.0f
        });
        track_arranger_clips[0].push_back(sequencer::ArrangerClipInstance{
            .id = 5, .name = "PAT 1: Straight Beat", .clip = drum_clip,
            .type = sequencer::ArrangerClipType::Pattern,
            .pattern_index = 0,
            .start_bar = 4.0f, .len_bars = 4.0f, .offset_bars = 0.0f
        });

        // Track 1 (Bass): Tracker Pattern Clip (Bars 2-8)
        track_arranger_clips[1].push_back(sequencer::ArrangerClipInstance{
            .id = 2, .name = "PAT 1: Acid Groove", .clip = acid_clip,
            .type = sequencer::ArrangerClipType::Pattern,
            .pattern_index = 0,
            .start_bar = 2.0f, .len_bars = 6.0f, .offset_bars = 0.0f
        });

        // Track 2 (Vocals): Audio Vocal Chop Take (Bars 4-8)
        track_arranger_clips[2].push_back(sequencer::ArrangerClipInstance{
            .id = 3, .name = "Vocal Chops", .clip = vocal_clip,
            .type = sequencer::ArrangerClipType::Audio,
            .start_bar = 4.0f, .len_bars = 4.0f, .offset_bars = 0.0f
        });

        // Track 3 (Perc): Tracker Pattern Clip (Bars 0-8)
        track_arranger_clips[3].push_back(sequencer::ArrangerClipInstance{
            .id = 4, .name = "PAT 1: HiHat Roll", .clip = perc_clip,
            .type = sequencer::ArrangerClipType::Pattern,
            .pattern_index = 0,
            .start_bar = 0.0f, .len_bars = 8.0f, .offset_bars = 0.0f
        });

        for (int t = 0; t < 4; ++t) {
            sync_arranger_to_mixer(t);
        }
    }

    auto sync_track_clip = [&](int t, std::shared_ptr<sampling::AudioClip> clip) {
        if (t < 0 || t >= 4 || !clip) return;
        track_clips[t] = clip;
        if (clip->slices().empty()) clip->slice_grid(8);
        if (track_seq[t]) track_seq[t]->set_clip(clip);

        if (!track_arranger_clips[t].empty()) {
            int sel_idx = std::clamp(g_selected_clip_idx, 0, static_cast<int>(track_arranger_clips[t].size() - 1));
            track_arranger_clips[t][sel_idx].clip = clip;
            track_arranger_clips[t][sel_idx].name = clip->name();
            sync_arranger_to_mixer(t);
        }

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

    auto stamp_pattern_to_arranger = [&](int t, int p_idx) {
        if (t < 0 || t >= 4) return;
        auto seq = track_seq[t];
        if (!seq || p_idx < 0 || p_idx >= 4) return;
        float cur_p_bar = playhead_seconds / get_seconds_per_bar(bpm);
        float snap_bar = std::floor(cur_p_bar * 4.0f) * 0.25f;
        Track* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
        if (!trk) return;

        auto before = trk->arranger().clips();
        sequencer::ArrangerClipInstance pat_inst;
        pat_inst.type = sequencer::ArrangerClipType::Pattern;
        pat_inst.pattern_index = static_cast<uint32_t>(p_idx);
        pat_inst.clip = seq->clip() ? seq->clip() : track_clips[t];
        pat_inst.name = "PAT " + std::to_string(p_idx + 1) + ": " + seq->pattern(p_idx).name;
        pat_inst.start_bar = snap_bar;
        float dur_bars = static_cast<float>(seq->pattern(p_idx).num_steps) / 16.0f;
        if (seq->pattern(p_idx).subdivision == sequencer::StepSubdivision::ThirtySecond)
            dur_bars = static_cast<float>(seq->pattern(p_idx).num_steps) / 32.0f;
        else if (seq->pattern(p_idx).subdivision == sequencer::StepSubdivision::Eighth)
            dur_bars = static_cast<float>(seq->pattern(p_idx).num_steps) / 8.0f;
        pat_inst.len_bars = std::max(dur_bars, 0.25f);
        pat_inst.base_len_bars = pat_inst.len_bars;
        pat_inst.orig_len_bars = pat_inst.len_bars;

        trk->arranger().add_clip(pat_inst);
        track_arranger_clips[t] = trk->arranger().clips();
        g_selected_clip_idx = static_cast<int>(track_arranger_clips[t].size() - 1);
        g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
            trk, before, track_arranger_clips[t], "Stamp Pattern Clip '" + pat_inst.name + "'"
        ));
        std::snprintf(status_toast, sizeof(status_toast), "STAMPED PATTERN %d TO TRACK %d AT BAR %.2f",
                      p_idx + 1, t + 1, snap_bar + 1.0f);
    };

    auto bake_gridpie_scene_to_arranger = [&]() {
        float cur_p_bar = playhead_seconds / get_seconds_per_bar(bpm);
        float snap_bar = std::floor(cur_p_bar * 4.0f) * 0.25f;
        float max_len_bars = 1.0f;
        int baked_count = 0;

        for (int t = 0; t < 4; ++t) {
            auto seq = track_seq[t];
            Track* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
            if (!seq || !trk || !trk->is_sequencer_enabled()) continue;

            uint32_t p_idx = seq->current_pattern_index();
            auto before = trk->arranger().clips();
            sequencer::ArrangerClipInstance pat_inst;
            pat_inst.type = sequencer::ArrangerClipType::Pattern;
            pat_inst.pattern_index = p_idx;
            pat_inst.clip = seq->clip() ? seq->clip() : track_clips[t];
            pat_inst.name = "PAT " + std::to_string(p_idx + 1) + ": " + seq->pattern(p_idx).name;
            pat_inst.start_bar = snap_bar;
            float dur_bars = static_cast<float>(seq->pattern(p_idx).num_steps) / 16.0f;
            if (seq->pattern(p_idx).subdivision == sequencer::StepSubdivision::ThirtySecond)
                dur_bars = static_cast<float>(seq->pattern(p_idx).num_steps) / 32.0f;
            else if (seq->pattern(p_idx).subdivision == sequencer::StepSubdivision::Eighth)
                dur_bars = static_cast<float>(seq->pattern(p_idx).num_steps) / 8.0f;
            pat_inst.len_bars = std::max(dur_bars, 0.25f);
            pat_inst.base_len_bars = pat_inst.len_bars;
            pat_inst.orig_len_bars = pat_inst.len_bars;
            max_len_bars = std::max(max_len_bars, pat_inst.len_bars);

            trk->arranger().add_clip(pat_inst);
            track_arranger_clips[t] = trk->arranger().clips();
            g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
                trk, before, track_arranger_clips[t], "Bake GridPie Clip '" + pat_inst.name + "'"
            ));
            baked_count++;
        }

        if (baked_count > 0) {
            // Auto-advance playhead by baked duration so successive Ctrl+B presses stitch contiguous scenes
            playhead_seconds = (snap_bar + max_len_bars) * get_seconds_per_bar(bpm);
            std::snprintf(status_toast, sizeof(status_toast), "GRIDPIE BAKED %d TRACKS TO ARRANGER AT BAR %.2f (ADVANCED +%.1fb)",
                          baked_count, snap_bar + 1.0f, max_len_bars);
        } else {
            std::snprintf(status_toast, sizeof(status_toast), "GRIDPIE BAKE: NO TRACK SEQUENCERS ENABLED (CLICK 'ON' FIRST)");
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

        float loop_st_sec = get_loop_start_seconds(bpm);
        float loop_len_sec = get_loop_length_seconds(bpm);
        float loop_end_sec = loop_st_sec + loop_len_sec;

        if (is_playing) {
            if (pw_online) {
                playhead_seconds = static_cast<float>(mixer.clock().sample_position()) / static_cast<float>(kSampleRate);
                if (loop_active && playhead_seconds >= loop_end_sec) {
                    playhead_seconds = loop_st_sec + std::fmod(std::max(0.0f, playhead_seconds - loop_st_sec), loop_len_sec);
                    mixer.seek(static_cast<uint64_t>(playhead_seconds * kSampleRate));
                }
            } else {
                playhead_seconds += dt * (bpm / 120.0f);
                if (loop_active && playhead_seconds >= loop_end_sec) {
                    playhead_seconds = loop_st_sec + std::fmod(std::max(0.0f, playhead_seconds - loop_st_sec), loop_len_sec);
                }
                mixer.clock().set_sample_position(static_cast<uint64_t>(playhead_seconds * 48000.0f));
            }
        } else if (!mixer.clock().is_scrubbing()) {
            playhead_seconds = static_cast<float>(mixer.clock().sample_position()) / static_cast<float>(kSampleRate);
        }

        // Punch-In / Punch-Out Automation Engine
        float sec_per_bar = get_seconds_per_bar(bpm);
        float current_bar = playhead_seconds / sec_per_bar;
        float punch_in_b = g_punch_link_to_loop ? loop_start_bar : g_punch_in_bar;
        float punch_out_b = g_punch_link_to_loop ? (loop_start_bar + loop_bars) : g_punch_out_bar;

        if (is_playing && g_punch_enabled) {
            if (!g_punch_is_recording) {
                // Check if playhead has crossed into punch-in window
                if (current_bar >= punch_in_b && current_bar < punch_out_b) {
                    g_punch_is_recording = true;
                    if (tap0) {
                        float punch_dur_bars = punch_out_b - punch_in_b;
                        uint32_t target_f = static_cast<uint32_t>(std::max(128.0f, punch_dur_bars * sec_per_bar * static_cast<float>(kSampleRate)));
                        std::string take_name = "Take" + std::to_string(g_punch_take_counter++) + "_Trk" + std::to_string(selected_track + 1);
                        tap0->arm_quantized_bounce(target_f, take_name, sampling::QuantizeSyncMode::Immediate, true);
                        std::snprintf(status_toast, sizeof(status_toast), "⏺ PUNCH-IN: RECORDING TAKE %u (BARS %.1f-%.1f)",
                                      g_punch_take_counter - 1, punch_in_b + 1.0f, punch_out_b + 1.0f);
                    }
                }
            } else {
                // Currently recording: check if playhead passed punch-out or wrapped
                if (current_bar >= punch_out_b || current_bar < punch_in_b) {
                    g_punch_is_recording = false;
                    if (tap0) {
                        auto take_clip = tap0->get_quantized_clip();
                        if (!take_clip && tap0->recorded_frames() >= 256) {
                            take_clip = tap0->capture_retroactive(tap0->recorded_frames(),
                                                                  "Take" + std::to_string(g_punch_take_counter - 1) + "_Trk" + std::to_string(selected_track + 1),
                                                                  true);
                        }
                        if (take_clip) {
                            int t = selected_track;
                            Track* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                            if (trk) {
                                auto before_clips = trk->arranger().clips();
                                sequencer::ArrangerClipInstance new_clip;
                                new_clip.id = 0;
                                new_clip.name = take_clip->name();
                                new_clip.clip = take_clip;
                                new_clip.source_clip = take_clip;
                                new_clip.start_bar = punch_in_b;
                                new_clip.len_bars = punch_out_b - punch_in_b;
                                new_clip.base_len_bars = new_clip.len_bars;
                                new_clip.orig_len_bars = new_clip.len_bars;
                                new_clip.stretch_ratio = 1.0f;
                                new_clip.fade_in_bars = 0.04f;
                                new_clip.fade_out_bars = 0.04f;

                                sampling::SampleAssetPool::instance().register_asset(take_clip, new_clip.name + ".wav");
                                trk->arranger().add_clip(new_clip);
                                track_arranger_clips[t] = trk->arranger().clips();
                                g_selected_clip_idx = static_cast<int>(track_arranger_clips[t].size() - 1);
                                g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
                                    trk, before_clips, track_arranger_clips[t], "Punch Record Take '" + new_clip.name + "'"
                                ));
                                std::snprintf(status_toast, sizeof(status_toast), "⏺ PUNCH-OUT: COMMITTED TAKE '%s' (BARS %.1f-%.1f)",
                                              new_clip.name.c_str(), punch_in_b + 1.0f, punch_out_b + 1.0f);
                            }
                            tap0->dismiss_bounce_to_rolling();
                        }
                    }
                }
            }
        } else if (!is_playing && g_punch_is_recording) {
            // Stopped playback while recording take: gracefully commit partial take
            g_punch_is_recording = false;
            if (tap0) {
                auto take_clip = tap0->get_quantized_clip();
                if (!take_clip && tap0->recorded_frames() >= 256) {
                    take_clip = tap0->capture_retroactive(tap0->recorded_frames(),
                                                          "Take" + std::to_string(g_punch_take_counter - 1) + "_Trk" + std::to_string(selected_track + 1),
                                                          true);
                }
                if (take_clip) {
                    int t = selected_track;
                    Track* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                    if (trk) {
                        auto before_clips = trk->arranger().clips();
                        sequencer::ArrangerClipInstance new_clip;
                        new_clip.id = 0;
                        new_clip.name = take_clip->name();
                        new_clip.clip = take_clip;
                        new_clip.source_clip = take_clip;
                        new_clip.start_bar = punch_in_b;
                        float actual_dur = std::max(0.25f, (playhead_seconds - punch_in_b * sec_per_bar) / sec_per_bar);
                        new_clip.len_bars = std::min(actual_dur, punch_out_b - punch_in_b);
                        new_clip.base_len_bars = new_clip.len_bars;
                        new_clip.orig_len_bars = new_clip.len_bars;
                        new_clip.stretch_ratio = 1.0f;
                        new_clip.fade_in_bars = 0.04f;
                        new_clip.fade_out_bars = 0.04f;

                        sampling::SampleAssetPool::instance().register_asset(take_clip, new_clip.name + ".wav");
                        trk->arranger().add_clip(new_clip);
                        track_arranger_clips[t] = trk->arranger().clips();
                        g_selected_clip_idx = static_cast<int>(track_arranger_clips[t].size() - 1);
                        g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
                            trk, before_clips, track_arranger_clips[t], "Punch Record Take '" + new_clip.name + "'"
                        ));
                        std::snprintf(status_toast, sizeof(status_toast), "⏺ PUNCH STOPPED: COMMITTED TAKE '%s'", new_clip.name.c_str());
                    }
                    tap0->dismiss_bounce_to_rolling();
                }
            }
        }

        // Determine if selected track has phrase arpeggiator enabled
        int cur_trk_idx = std::clamp(selected_track, 0, 3);
        bool cur_track_arp_on = g_track_phrase_arp_enabled[cur_trk_idx];
        if (cur_track_arp_on) {
            auto* p = g_phrase_bank.get_phrase(static_cast<size_t>(g_track_phrase_assigned[cur_trk_idx]));
            if (p && g_phrase_player.phrase() != p) {
                g_phrase_player.set_phrase(p);
            }
        }

        // Drain incoming Hardware MIDI events through MidiLearnRouter into ModulationMatrix & PolyphonicSynth or PhrasePlayer
        midi_rx.drain_to(midi_learn, mixer, &mod_matrix, cur_track_arp_on ? &g_phrase_player : nullptr);
        midi_rx.sync_to_clock(mixer.clock());
        if (mixer.clock().authority() == clock::ClockAuthority::MidiClockSlave ||
            mixer.clock().authority() == clock::ClockAuthority::MtcSlave) {
            bpm = static_cast<float>(mixer.clock().bpm());
            is_playing = mixer.clock().is_playing();
            playhead_seconds = static_cast<float>(mixer.clock().sample_position()) / 48000.0f;
        }

        // Advance Modulator Matrix & Polyphonic Voice Pool at audio clock rate for real-time visual feedback
        uint32_t mod_sim_frames = std::clamp(static_cast<uint32_t>(dt * 48000.0f), 1u, 1024u);

        // Advance Instrument Phrase Player & Arpeggiator if active
        if (g_phrase_player.is_playing() || cur_track_arp_on) {
            constexpr size_t kMaxPhraseEvs = 64;
            sequencer::PhraseOutputEvent phrase_evs[kMaxPhraseEvs];
            size_t num_phrase_evs = 0;
            g_phrase_player.process_block(mod_sim_frames, g_tracker_timing, kSampleRate,
                                          phrase_evs, kMaxPhraseEvs, num_phrase_evs);
            for (size_t pe = 0; pe < num_phrase_evs; ++pe) {
                const auto& pev = phrase_evs[pe];
                if (pev.is_note_on) {
                    uint8_t note_to_play = pev.note;
                    if (g_scale_snap_enabled) {
                        note_to_play = tuning::ScaleDictionary::snap_to_scale(note_to_play, g_scale_root, g_selected_scale, g_scale_snap_mode);
                    }
                    mod_matrix.poly_note_on(note_to_play, pev.velocity);
                } else {
                    mod_matrix.poly_note_off(pev.note);
                }
            }
        }

        float synth_sim_l[1024];
        float synth_sim_r[1024];
        for (uint32_t s = 0; s < mod_sim_frames; ++s) {
            mod_matrix.evaluate_sample(bpm);
        }
        mod_matrix.process_synth_block(synth_sim_l, synth_sim_r, mod_sim_frames, bpm);

        if (!pw_online && !desktop_online && is_playing && tap0 && tap0->is_active()) {
            tap0->record(synth_sim_l, synth_sim_r, mod_sim_frames);
        }

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

        // Process any Drag & Drop Audio File or Project Bundle Import from GLFW
        if (g_has_dropped_files) {
            std::vector<std::string> paths_to_process;
            {
                std::lock_guard<std::mutex> lock(g_drop_mutex);
                paths_to_process = std::move(g_dropped_paths);
                g_has_dropped_files = false;
            }

            int imported_audio_count = 0;
            std::string last_sononym_toast;

            for (size_t p_idx = 0; p_idx < paths_to_process.size(); ++p_idx) {
                const auto& fpath = paths_to_process[p_idx];
                namespace fs = std::filesystem;
                std::error_code ec;

                // Check if dropped item is an .aethel bundle directory or .json session file
                bool is_dir = fs::is_directory(fpath, ec);
                bool is_json = (fpath.size() >= 5 && fpath.substr(fpath.size() - 5) == ".json");
                bool is_aethel = (fpath.size() >= 7 && fpath.substr(fpath.size() - 7) == ".aethel") ||
                                 (is_dir && fs::exists(fs::path(fpath) / "project.json", ec));

                if (is_aethel || (is_dir && !is_json)) {
                    std::vector<std::shared_ptr<sampling::AudioClip>> loaded_bundle_clips;
                    if (sampling::ProjectBundleManager::load_bundle(fpath, mixer, mixer.clock(), loaded_bundle_clips, &mod_matrix, &midi_learn)) {
                        for (size_t t = 0; t < std::min<size_t>(4, loaded_bundle_clips.size()); ++t) {
                            track_clips_orig[t] = loaded_bundle_clips[t];
                            sync_track_clip(static_cast<int>(t), loaded_bundle_clips[t]);
                        }
                        sync_ui_from_mixer();
                        std::snprintf(status_toast, sizeof(status_toast),
                                      "[BUNDLE LOADED] %s | %zu Clips Linked | BPM: %.1f",
                                      fs::path(fpath).filename().string().c_str(),
                                      loaded_bundle_clips.size(), mixer.clock().bpm());
                        break;
                    }
                } else if (is_json) {
                    if (serialization::SessionSerializer::load_session_file(fpath, mixer, mixer.clock(), &mod_matrix, &midi_learn)) {
                        sync_ui_from_mixer();
                        std::snprintf(status_toast, sizeof(status_toast),
                                      "[SESSION LOADED] %s | BPM: %.1f",
                                      fs::path(fpath).filename().string().c_str(), mixer.clock().bpm());
                        break;
                    }
                } else {
                    // Audio file (.wav)
                    auto imported = std::make_shared<sampling::AudioClip>();
                    if (imported->load_from_wav(fpath)) {
                        int target_track = (paths_to_process.size() == 1) ? selected_track : static_cast<int>((selected_track + imported_audio_count) % 4);
                        track_clips_orig[target_track] = imported;
                        sync_track_clip(target_track, imported);

                        // Sononym Perceptual Analysis & Content Hashing
                        auto desc = sampling::SampleAnalyzer::analyze(*imported, fs::path(fpath).filename().string());
                        sampling::SampleAssetPool::instance().register_asset(imported, desc.file_name);

                        char toast_buf[256];
                        std::snprintf(toast_buf, sizeof(toast_buf),
                                      "[SONONYM] #%d \"%s\" | Key: %s (%.1fHz) | %.1f BPM | RMS: %.1f dB | Peak: %.1f dB | Crest: %.1f dB | %s",
                                      target_track + 1, desc.file_name.c_str(), desc.musical_key.c_str(),
                                      desc.fundamental_hz, desc.estimated_bpm, desc.rms_db, desc.peak_db,
                                      desc.crest_factor_db, desc.perceptual_category.c_str());
                        last_sononym_toast = toast_buf;
                        imported_audio_count++;
                    }
                }
            }

            if (!last_sononym_toast.empty()) {
                if (imported_audio_count > 1) {
                    std::snprintf(status_toast, sizeof(status_toast),
                                  "[SONONYM MULTI-DROP] %d audio files registered into Asset Pool! Last: %s",
                                  imported_audio_count, last_sononym_toast.c_str());
                } else {
                    std::snprintf(status_toast, sizeof(status_toast), "%s", last_sononym_toast.c_str());
                }
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Workstation Fullscreen Dock Window
        int win_w, win_h;
        glfwGetFramebufferSize(window, &win_w, &win_h);
        static float g_ui_scale = (win_target_h <= 800) ? 0.78f : 1.0f;
        static float s_zone_split_ratio = 0.48f;
        ImGuiIO& io = ImGui::GetIO();
        io.FontGlobalScale = g_ui_scale;

        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(static_cast<float>(win_w), static_cast<float>(win_h)));
        ImGui::Begin("AethelAudioDeskRoot", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus);

        auto sync_all_gui_track_state = [&]() {
            Track* trks[4] = {trk0, trk1, trk2, trk3};
            for (int i = 0; i < 4; ++i) {
                if (trks[i]) {
                    track_arranger_clips[i] = trks[i]->arranger().clips();
                    track_gains[i] = trks[i]->gain();
                    track_pans[i] = trks[i]->pan();
                    track_mutes[i] = trks[i]->is_muted();
                    track_solos[i] = trks[i]->is_solo();
                    int32_t b = trks[i]->target_bus();
                    if (bus_drums && b == static_cast<int32_t>(bus_drums->id())) track_target_buses[i] = 1;
                    else if (bus_music && b == static_cast<int32_t>(bus_music->id())) track_target_buses[i] = 2;
                    else track_target_buses[i] = 0;
                }
            }
        };

        auto perform_undo = [&]() {
            if (g_undo_mgr.can_undo()) {
                std::string desc = g_undo_mgr.undo_description();
                g_undo_mgr.undo();
                sync_all_gui_track_state();
                std::snprintf(status_toast, sizeof(status_toast), "UNDO: %s", desc.c_str());
            } else {
                std::snprintf(status_toast, sizeof(status_toast), "NOTHING TO UNDO");
            }
        };

        auto perform_redo = [&]() {
            if (g_undo_mgr.can_redo()) {
                std::string desc = g_undo_mgr.redo_description();
                g_undo_mgr.redo();
                sync_all_gui_track_state();
                std::snprintf(status_toast, sizeof(status_toast), "REDO: %s", desc.c_str());
            } else {
                std::snprintf(status_toast, sizeof(status_toast), "NOTHING TO REDO");
            }
        };

        // ====================================================================
        // ZONE 1: TOP GLOBAL TRANSPORT & HUD BAR
        // ====================================================================
        ImGui::BeginChild("TopTransportBar", ImVec2(0, 54), true, ImGuiWindowFlags_NoScrollbar);
        {
            // Global Transport Key Shortcuts (Space = Play/Pause, Home = Return to Zero, L = Loop)
            if (!io.WantTextInput) {
                if (ImGui::IsKeyPressed(ImGuiKey_Space)) {
                    is_playing = !is_playing;
                    mixer.clock().set_playing(is_playing);
                }
                if (ImGui::IsKeyPressed(ImGuiKey_Home)) {
                    mixer.seek(0, true);
                    playhead_seconds = 0.0f;
                }
                if (ImGui::IsKeyPressed(ImGuiKey_L)) {
                    loop_active = !loop_active;
                    std::snprintf(status_toast, sizeof(status_toast), "TIMELINE LOOP: %s", loop_active ? "ENABLED" : "BYPASSED");
                }
                if (ImGui::IsKeyPressed(ImGuiKey_R)) {
                    g_punch_enabled = !g_punch_enabled;
                    std::snprintf(status_toast, sizeof(status_toast), "PUNCH OVERDUB: %s", g_punch_enabled ? "ARMED" : "BYPASSED");
                }
            }

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
            if (loop_active) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.45f, 0.95f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
            }
            if (ImGui::Button("[ ⟳ LOOP ]", ImVec2(78, 32))) {
                loop_active = !loop_active;
                std::snprintf(status_toast, sizeof(status_toast), "TIMELINE LOOP: %s", loop_active ? "ENABLED" : "BYPASSED");
            }
            if (loop_active) ImGui::PopStyleColor(2);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Toggle Timeline Loop (Shortcut: L)");

            ImGui::SameLine(0, 4);
            if (g_punch_enabled) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.20f, 0.20f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
            }
            if (ImGui::Button("[ ⏺ PUNCH ]", ImVec2(84, 32))) {
                g_punch_enabled = !g_punch_enabled;
                std::snprintf(status_toast, sizeof(status_toast), "PUNCH OVERDUB: %s", g_punch_enabled ? "ARMED" : "BYPASSED");
            }
            if (g_punch_enabled) ImGui::PopStyleColor(2);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Toggle Punch-In / Punch-Out Overdub Engine (Shortcut: R)");

            // Layout Presets & UI Scaling
            ImGui::SameLine(0, 10);
            ImGui::SetNextItemWidth(62);
            if (ImGui::SliderFloat("Scale", &g_ui_scale, 0.70f, 1.30f, "%.2fx")) {
                io.FontGlobalScale = g_ui_scale;
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Global UI & Font Scale (70%% - 130%%)");

            ImGui::SameLine(0, 6);
            if (ImGui::SmallButton("ARR")) { s_zone_split_ratio = 0.82f; }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Arranger Focus Layout");
            ImGui::SameLine(0, 3);
            if (ImGui::SmallButton("50/50")) { s_zone_split_ratio = 0.48f; }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Balanced Split Layout");
            ImGui::SameLine(0, 3);
            if (ImGui::SmallButton("MIX")) { s_zone_split_ratio = 0.16f; }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Mixer Focus Layout");

            ImGui::SameLine(0, 10);
            ImGui::SetNextItemWidth(90);
            if (mixer.clock().authority() == clock::ClockAuthority::MidiClockSlave) {
                ImGui::BeginDisabled();
                ImGui::SliderFloat("BPM", &bpm, 60.0f, 200.0f, "%.1f (MIDI)");
                ImGui::EndDisabled();
            } else if (ImGui::SliderFloat("BPM", &bpm, 60.0f, 200.0f, "%.1f")) {
                mixer.clock().set_bpm(bpm);
            }

            // Renoise Granular Timing (LPB & TPL) + Scale Quantize toggle
            ImGui::SameLine(0, 10);
            ImGui::SetNextItemWidth(65);
            const char* lpb_labels[5] = { "LPB:4", "LPB:8", "LPB:12", "LPB:16", "LPB:32" };
            if (ImGui::Combo("##LPBCombo", &g_tracker_lpb_idx, lpb_labels, 5)) {
                g_tracker_timing.lpb = g_lpb_options[g_tracker_lpb_idx];
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Renoise Lines Per Beat (LPB: 4=16ths, 8=32nds, 12=triplets)");

            ImGui::SameLine(0, 4);
            ImGui::SetNextItemWidth(65);
            const char* tpl_labels[4] = { "TPL:8", "TPL:12", "TPL:16", "TPL:24" };
            if (ImGui::Combo("##TPLCombo", &g_tracker_tpl_idx, tpl_labels, 4)) {
                g_tracker_timing.tpl = g_tpl_options[g_tracker_tpl_idx];
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Renoise Ticks Per Line (TPL: 12 or 16 micro-timing subdivisions)");

            ImGui::SameLine(0, 6);
            if (ImGui::Button(g_scale_snap_enabled ? "[SCALE: ON]" : "[SCALE: OFF]")) {
                g_scale_snap_enabled = !g_scale_snap_enabled;
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Toggle Scale Snapping & Quantization across Arranger/Instruments");

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

            // Arranger Tool Shortcuts (1=Select, 2=Razor, 3=Slip, 4=Stretch, [/]=Pitch)
            if (!ImGui::GetIO().WantTextInput) {
                if (ImGui::IsKeyPressed(ImGuiKey_1)) {
                    g_arranger_tool = ArrangerTool::Select;
                    std::snprintf(status_toast, sizeof(status_toast), "ARRANGER TOOL: SELECT / MOVE");
                } else if (ImGui::IsKeyPressed(ImGuiKey_2)) {
                    g_arranger_tool = ArrangerTool::Razor;
                    std::snprintf(status_toast, sizeof(status_toast), "ARRANGER TOOL: RAZOR / SPLIT (CLICK CLIP TO CUT)");
                } else if (ImGui::IsKeyPressed(ImGuiKey_3)) {
                    g_arranger_tool = ArrangerTool::Slip;
                    std::snprintf(status_toast, sizeof(status_toast), "ARRANGER TOOL: SLIP-EDIT (ALT+DRAG)");
                } else if (ImGui::IsKeyPressed(ImGuiKey_4)) {
                    g_arranger_tool = ArrangerTool::Stretch;
                    std::snprintf(status_toast, sizeof(status_toast), "ARRANGER TOOL: TIME-STRETCH (WSOLA EDGE-DRAG)");
                } else if (ImGui::IsKeyPressed(ImGuiKey_5)) {
                    g_arranger_tool = ArrangerTool::Warp;
                    std::snprintf(status_toast, sizeof(status_toast), "ARRANGER TOOL: WARP-PINS & GROOVE (CLICK TRANSIENT TO PIN)");
                }
            }

            // Keyboard Shortcuts: Ctrl+Z (Undo / Redo) & Ctrl+Y (Redo)
            if (!ImGui::GetIO().WantTextInput) {
                if (ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z)) {
                    if (ImGui::GetIO().KeyShift) {
                        perform_redo();
                    } else {
                        perform_undo();
                    }
                } else if (ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Y)) {
                    perform_redo();
                } else if (ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_L)) {
                    int cur_t = selected_track;
                    if (cur_t >= 0 && cur_t < 4 && !track_arranger_clips[cur_t].empty()) {
                        int c_idx = std::clamp(g_selected_clip_idx, 0, static_cast<int>(track_arranger_clips[cur_t].size() - 1));
                        const auto& c = track_arranger_clips[cur_t][c_idx];
                        loop_start_bar = c.start_bar;
                        loop_bars = c.len_bars;
                        loop_active = true;
                        if (g_punch_link_to_loop) {
                            g_punch_in_bar = loop_start_bar;
                            g_punch_out_bar = loop_start_bar + loop_bars;
                        }
                        std::snprintf(status_toast, sizeof(status_toast), "LOOP FIT TO CLIP '%s' (BARS %.1f-%.1f)",
                                      c.name.c_str(), loop_start_bar + 1.0f, loop_start_bar + loop_bars + 1.0f);
                    }
                } else if (ImGui::GetIO().KeyCtrl && (ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter))) {
                    stamp_pattern_to_arranger(selected_track, pattern_editor_pat_idx);
                } else if (ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_B)) {
                    bake_gridpie_scene_to_arranger();
                }
            }

            // [ and ]: Semitone Pitch Shift
            if (!ImGui::GetIO().WantTextInput) {
                if (ImGui::IsKeyPressed(ImGuiKey_LeftBracket) || ImGui::IsKeyPressed(ImGuiKey_RightBracket)) {
                    float delta = ImGui::IsKeyPressed(ImGuiKey_RightBracket) ? +1.0f : -1.0f;
                    int t = selected_track;
                    if (t >= 0 && t < 4 && !track_arranger_clips[t].empty()) {
                        int c_idx = std::clamp(g_selected_clip_idx, 0, static_cast<int>(track_arranger_clips[t].size() - 1));
                        auto& c = track_arranger_clips[t][c_idx];
                        Track* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                        if (trk) {
                            auto before = trk->arranger().clips();
                            trk->arranger().set_clip_pitch(c.id, c.pitch_semitones + delta);
                            track_arranger_clips[t] = trk->arranger().clips();
                            g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
                                trk, before, track_arranger_clips[t], "Pitch Shift '" + c.name + "'"
                            ));
                            const auto* sc = trk->arranger().find_clip(c.id);
                            if (sc) {
                                std::snprintf(status_toast, sizeof(status_toast), "CLIP '%s' PITCH: %+.1f SEMITONES",
                                              sc->name.c_str(), sc->pitch_semitones);
                            }
                        }
                    }
                }
            }

            // Ctrl+E: Split selected clip at playhead
            if (ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_E)) {
                float playhead_bar = playhead_seconds / get_seconds_per_bar(bpm);
                int t = selected_track;
                if (t >= 0 && t < 4 && !track_arranger_clips[t].empty()) {
                    int c_idx = std::clamp(g_selected_clip_idx, 0, static_cast<int>(track_arranger_clips[t].size() - 1));
                    auto& c = track_arranger_clips[t][c_idx];
                    if (playhead_bar > c.start_bar + 0.05f && playhead_bar < c.end_bar() - 0.05f) {
                        Track* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                        if (trk) {
                            auto before = trk->arranger().clips();
                            trk->arranger().split_clip_at_bar(c.id, playhead_bar, bpm, kSampleRate);
                            track_arranger_clips[t] = trk->arranger().clips();
                            g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
                                trk, before, track_arranger_clips[t], "Split Clip '" + c.name + "'"
                            ));
                            std::snprintf(status_toast, sizeof(status_toast), "SPLIT CLIP '%s' AT BAR %.2f (CTRL+E)", c.name.c_str(), playhead_bar + 1.0f);
                        }
                    } else {
                        std::snprintf(status_toast, sizeof(status_toast), "CANNOT SPLIT: PLAYHEAD (%.2fb) OUTSIDE SELECTED CLIP (%.2f-%.2fb)",
                                      playhead_bar + 1.0f, c.start_bar + 1.0f, c.end_bar() + 1.0f);
                    }
                }
            }

            // Delete / Backspace: Remove selected clip
            if (!ImGui::GetIO().WantTextInput && (ImGui::IsKeyPressed(ImGuiKey_Delete) || ImGui::IsKeyPressed(ImGuiKey_Backspace))) {
                int t = selected_track;
                if (t >= 0 && t < 4 && !track_arranger_clips[t].empty()) {
                    int c_idx = std::clamp(g_selected_clip_idx, 0, static_cast<int>(track_arranger_clips[t].size() - 1));
                    uint32_t cid = track_arranger_clips[t][c_idx].id;
                    std::string cname = track_arranger_clips[t][c_idx].name;
                    Track* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                    if (trk) {
                        auto before = trk->arranger().clips();
                        trk->arranger().remove_clip(cid);
                        track_arranger_clips[t] = trk->arranger().clips();
                        g_selected_clip_idx = std::max(0, c_idx - 1);
                        g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
                            trk, before, track_arranger_clips[t], "Delete Clip '" + cname + "'"
                        ));
                        std::snprintf(status_toast, sizeof(status_toast), "DELETED CLIP (TRACK %d)", t + 1);
                    }
                }
            }

            // Ctrl+D: Duplicate selected clip
            if (ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_D)) {
                int t = selected_track;
                if (t >= 0 && t < 4 && !track_arranger_clips[t].empty()) {
                    int c_idx = std::clamp(g_selected_clip_idx, 0, static_cast<int>(track_arranger_clips[t].size() - 1));
                    auto orig = track_arranger_clips[t][c_idx];
                    orig.id = 0;
                    orig.start_bar = orig.end_bar();
                    orig.name += " (Copy)";
                    Track* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                    if (trk) {
                        auto before = trk->arranger().clips();
                        trk->arranger().add_clip(orig);
                        track_arranger_clips[t] = trk->arranger().clips();
                        g_selected_clip_idx = static_cast<int>(track_arranger_clips[t].size() - 1);
                        g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
                            trk, before, track_arranger_clips[t], "Duplicate Clip '" + orig.name + "'"
                        ));
                        std::snprintf(status_toast, sizeof(status_toast), "DUPLICATED CLIP '%s' TO BAR %.1f", orig.name.c_str(), orig.start_bar + 1.0f);
                    }
                }
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
                if (ImGui::MenuItem("Save Self-Contained Bundle (.aethel)...")) {
                    open_save_bundle_modal = true;
                }
                if (ImGui::MenuItem("Load Self-Contained Bundle (.aethel)...")) {
                    open_load_bundle_modal = true;
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Export Master WAV (Offline Bounce)...")) {
                    open_bounce_modal = true;
                }
                ImGui::EndPopup();
            }

            ImGui::SameLine(0, 5);
            if (ImGui::Button("[ BUNDLE ]", ImVec2(80, 32))) {
                open_save_bundle_modal = true;
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
                ImGui::TextColored(ImVec4(0.12f, 0.55f, 0.95f, 1.0f), "[PIPEWIRE: %zu IN / %zu OUT]", sources.size(), sinks.size());
            } else if (desktop_online) {
                ImGui::TextColored(ImVec4(0.20f, 0.85f, 0.45f, 1.0f), "[DRIVER: ALSA / RTKit (SCHED_FIFO)]");
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
        const float available_h = static_cast<float>(win_h) - 72.0f;
        const float min_z2 = 40.0f;
        const float min_z3 = 120.0f;
        float zone2_h = std::clamp(available_h * s_zone_split_ratio, min_z2, std::max(min_z2, available_h - min_z3 - 10.0f));
        float zone3_h = std::max(min_z3, available_h - zone2_h - 10.0f);

        // ====================================================================
        // ZONE 2: CENTER WORKSPACE (TIMELINE ARRANGER vs. ROUTING MATRIX)
        // ====================================================================
        ImGui::BeginChild("CenterWorkspace", ImVec2(0, zone2_h), true);
        {
            const char* track_names[4] = { "Track 1: Kick / 808", "Track 2: Acid 303 Lead", "Track 3: Vocal Chops", "Track 4: Percussion / Hats" };

            struct TrackRoutingBadge {
                const char* bus_name;
                ImColor bg_color;
                ImColor text_color;
            };
            const TrackRoutingBadge k_track_routing_badges[4] = {
                { "-> BUS A (DRUMS)", ImColor(217, 123, 13, 230), ImColor(255, 255, 255, 255) },
                { "-> BUS B (MUSIC)", ImColor(14, 116, 144, 230), ImColor(255, 255, 255, 255) },
                { "-> BUS B (MUSIC)", ImColor(14, 116, 144, 230), ImColor(255, 255, 255, 255) },
                { "-> BUS A (DRUMS)", ImColor(217, 123, 13, 230), ImColor(255, 255, 255, 255) },
            };

            if (ImGui::BeginTabBar("MainWorkspaceTabs", ImGuiTabBarFlags_None)) {
                // ------------------------------------------------------------
                // TAB 1: TIMELINE ARRANGER (CLEAN ZÄHL AM1 CANVAS)
                // ------------------------------------------------------------
                ImGuiTabItemFlags t1_flags = (g_switch_to_main_tab == 0) ? ImGuiTabItemFlags_SetSelected : 0;
                if (ImGui::BeginTabItem("  TIMELINE ARRANGER  ", nullptr, t1_flags)) {
                    if (g_switch_to_main_tab == 0) g_switch_to_main_tab = -1;
                    ImVec2 avail_sz = ImGui::GetContentRegionAvail();
                    avail_sz.y = std::max(avail_sz.y - 4.0f, 130.0f);

                    // Arranger Toolbar: Tools (Select, Razor, Slip), Actions (Split @ Playhead, Dup, Del) & Snap Info
                    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 3.0f);
                    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(5, 4));

                    bool is_sel_tool = (g_arranger_tool == ArrangerTool::Select);
                    if (is_sel_tool) {
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.38f, 0.85f, 1.0f));
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                    }
                    if (ImGui::SmallButton("1: SELECT")) {
                        g_arranger_tool = ArrangerTool::Select;
                        std::snprintf(status_toast, sizeof(status_toast), "ARRANGER TOOL: SELECT / MOVE");
                    }
                    if (is_sel_tool) ImGui::PopStyleColor(2);

                    ImGui::SameLine();
                    bool is_razor_tool = (g_arranger_tool == ArrangerTool::Razor);
                    if (is_razor_tool) {
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.25f, 0.20f, 1.0f));
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                    }
                    if (ImGui::SmallButton("2: RAZOR")) {
                        g_arranger_tool = ArrangerTool::Razor;
                        std::snprintf(status_toast, sizeof(status_toast), "ARRANGER TOOL: RAZOR / SPLIT (CLICK CLIP TO CUT)");
                    }
                    if (is_razor_tool) ImGui::PopStyleColor(2);

                    ImGui::SameLine();
                    bool is_slip_tool = (g_arranger_tool == ArrangerTool::Slip);
                    if (is_slip_tool) {
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.65f, 0.35f, 1.0f));
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                    }
                    if (ImGui::SmallButton("3: SLIP")) {
                        g_arranger_tool = ArrangerTool::Slip;
                        std::snprintf(status_toast, sizeof(status_toast), "ARRANGER TOOL: SLIP-EDIT (DRAG AUDIO OFFSET)");
                    }
                    if (is_slip_tool) ImGui::PopStyleColor(2);

                    ImGui::SameLine();
                    bool is_stretch_tool = (g_arranger_tool == ArrangerTool::Stretch);
                    if (is_stretch_tool) {
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.45f, 0.10f, 1.0f));
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                    }
                    if (ImGui::SmallButton("4: STRETCH")) {
                        g_arranger_tool = ArrangerTool::Stretch;
                        std::snprintf(status_toast, sizeof(status_toast), "ARRANGER TOOL: TIME-STRETCH (DRAG EDGE TO STRETCH WSOLA)");
                    }
                    if (is_stretch_tool) ImGui::PopStyleColor(2);

                    ImGui::SameLine();
                    bool is_warp_tool = (g_arranger_tool == ArrangerTool::Warp);
                    if (is_warp_tool) {
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.20f, 0.65f, 1.0f));
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                    }
                    if (ImGui::SmallButton("5: WARP")) {
                        g_arranger_tool = ArrangerTool::Warp;
                        std::snprintf(status_toast, sizeof(status_toast), "ARRANGER TOOL: WARP-PINS & GROOVE (CLICK TRANSIENT TO PIN)");
                    }
                    if (is_warp_tool) ImGui::PopStyleColor(2);

                    ImGui::SameLine();
                    ImGui::TextDisabled("|");
                    ImGui::SameLine();

                    if (ImGui::SmallButton("SPLIT (Ctrl+E)")) {
                        float playhead_bar = playhead_seconds / get_seconds_per_bar(bpm);
                        int t = selected_track;
                        if (t >= 0 && t < 4 && !track_arranger_clips[t].empty()) {
                            int c_idx = std::clamp(g_selected_clip_idx, 0, static_cast<int>(track_arranger_clips[t].size() - 1));
                            auto& c = track_arranger_clips[t][c_idx];
                            if (playhead_bar > c.start_bar + 0.05f && playhead_bar < c.end_bar() - 0.05f) {
                                Track* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                                if (trk) {
                                    trk->arranger().split_clip_at_bar(c.id, playhead_bar, bpm, kSampleRate);
                                    track_arranger_clips[t] = trk->arranger().clips();
                                    std::snprintf(status_toast, sizeof(status_toast), "SPLIT CLIP '%s' AT BAR %.2f (CTRL+E)", c.name.c_str(), playhead_bar + 1.0f);
                                }
                            } else {
                                std::snprintf(status_toast, sizeof(status_toast), "CANNOT SPLIT: PLAYHEAD (%.2fb) OUTSIDE SELECTED CLIP (%.2f-%.2fb)",
                                              playhead_bar + 1.0f, c.start_bar + 1.0f, c.end_bar() + 1.0f);
                            }
                        }
                    }

                    ImGui::SameLine();
                    if (ImGui::SmallButton("+ DUP (Ctrl+D)")) {
                        int t = selected_track;
                        if (t >= 0 && t < 4 && !track_arranger_clips[t].empty()) {
                            int c_idx = std::clamp(g_selected_clip_idx, 0, static_cast<int>(track_arranger_clips[t].size() - 1));
                            auto orig = track_arranger_clips[t][c_idx];
                            orig.id = 0;
                            orig.start_bar = orig.end_bar();
                            orig.name += " (Copy)";
                            Track* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                            if (trk) {
                                auto before = trk->arranger().clips();
                                trk->arranger().add_clip(orig);
                                track_arranger_clips[t] = trk->arranger().clips();
                                g_selected_clip_idx = static_cast<int>(track_arranger_clips[t].size() - 1);
                                g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
                                    trk, before, track_arranger_clips[t], "Duplicate Clip '" + orig.name + "'"
                                ));
                                std::snprintf(status_toast, sizeof(status_toast), "DUPLICATED CLIP '%s' TO BAR %.1f", orig.name.c_str(), orig.start_bar + 1.0f);
                            }
                        }
                    }

                    ImGui::SameLine();
                    if (ImGui::SmallButton("DEL (Del)")) {
                        int t = selected_track;
                        if (t >= 0 && t < 4 && !track_arranger_clips[t].empty()) {
                            int c_idx = std::clamp(g_selected_clip_idx, 0, static_cast<int>(track_arranger_clips[t].size() - 1));
                            uint32_t cid = track_arranger_clips[t][c_idx].id;
                            std::string cname = track_arranger_clips[t][c_idx].name;
                            Track* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                            if (trk) {
                                auto before = trk->arranger().clips();
                                trk->arranger().remove_clip(cid);
                                track_arranger_clips[t] = trk->arranger().clips();
                                g_selected_clip_idx = std::max(0, c_idx - 1);
                                g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
                                    trk, before, track_arranger_clips[t], "Delete Clip '" + cname + "'"
                                ));
                                std::snprintf(status_toast, sizeof(status_toast), "DELETED CLIP (TRACK %d)", t + 1);
                            }
                        }
                    }

                    ImGui::SameLine();
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.45f, 0.10f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                    if (ImGui::SmallButton("+ PAT CLIP")) {
                        ImGui::OpenPopup("AddPatternClipPopup");
                    }
                    ImGui::PopStyleColor(2);
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Insert Tracker Pattern as Arranger Clip at playhead.");

                    if (ImGui::BeginPopup("AddPatternClipPopup")) {
                        ImGui::TextColored(ImVec4(0.95f, 0.60f, 0.20f, 1.0f), "INSERT TRACKER PATTERN CLIP (TRK %d):", selected_track + 1);
                        ImGui::Separator();
                        auto seq = track_seq[selected_track];
                        if (seq) {
                            for (int p = 0; p < 4; ++p) {
                                char pat_btn[64];
                                std::snprintf(pat_btn, sizeof(pat_btn), "PAT %d: %s (%d Steps)",
                                              p + 1, seq->pattern(p).name.c_str(), seq->pattern(p).num_steps);
                                if (ImGui::Selectable(pat_btn)) {
                                    stamp_pattern_to_arranger(selected_track, p);
                                }
                            }
                        }
                        ImGui::EndPopup();
                    }

                    ImGui::SameLine();
                    ImGui::TextDisabled("|");
                    ImGui::SameLine();

                    bool can_u = g_undo_mgr.can_undo();
                    if (!can_u) ImGui::BeginDisabled();
                    if (ImGui::SmallButton("UNDO (Ctrl+Z)")) {
                        perform_undo();
                    }
                    if (!can_u) ImGui::EndDisabled();
                    else if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("Undo: %s", g_undo_mgr.undo_description().c_str());
                    }

                    ImGui::SameLine();
                    bool can_r = g_undo_mgr.can_redo();
                    if (!can_r) ImGui::BeginDisabled();
                    if (ImGui::SmallButton("REDO (Ctrl+Y)")) {
                        perform_redo();
                    }
                    if (!can_r) ImGui::EndDisabled();
                    else if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("Redo: %s", g_undo_mgr.redo_description().c_str());
                    }

                    ImGui::SameLine();
                    ImGui::TextDisabled("|");
                    ImGui::SameLine();
                    ImGui::TextDisabled("SNAP: 1/4 BAR");

                    ImGui::SameLine();
                    ImGui::TextDisabled("|");
                    ImGui::SameLine();
                    if (loop_active) {
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.45f, 0.95f, 1.0f));
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                    }
                    if (ImGui::SmallButton(loop_active ? "🔁 LOOP ON" : "🔁 LOOP OFF")) {
                        loop_active = !loop_active;
                    }
                    if (loop_active) ImGui::PopStyleColor(2);
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Toggle Timeline Loop (Double-click ruler or press L).\nCtrl+L: Fit loop to selected clip.");

                    ImGui::SameLine();
                    if (g_punch_enabled) {
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.20f, 0.20f, 1.0f));
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                    }
                    if (ImGui::SmallButton(g_punch_enabled ? "⏺ PUNCH ON" : "⏺ PUNCH OFF")) {
                        g_punch_enabled = !g_punch_enabled;
                    }
                    if (g_punch_enabled) ImGui::PopStyleColor(2);
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Toggle Punch-In / Punch-Out Overdub Engine (Shortcut: R).\nRecords takes between IN and OUT flags directly into active track lane.");

                    ImGui::SameLine();
                    if (ImGui::SmallButton(g_punch_link_to_loop ? "🔗 PUNCH=LOOP" : "🔓 PUNCH FREE")) {
                        g_punch_link_to_loop = !g_punch_link_to_loop;
                        if (g_punch_link_to_loop) {
                            g_punch_in_bar = loop_start_bar;
                            g_punch_out_bar = loop_start_bar + loop_bars;
                        }
                    }
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Link Punch markers directly to Loop brackets (or unlock to position independently).");

                    ImGui::SameLine();
                    ImGui::TextDisabled("|");
                    ImGui::SameLine();
                    if (ImGui::SmallButton(" - ##arr_zm_out")) {
                        float new_v = std::min(g_arranger_max_bars, g_arranger_visible_bars * 1.25f);
                        g_arranger_visible_bars = new_v;
                        g_arranger_scroll_bar = std::clamp(g_arranger_scroll_bar, 0.0f, std::max(0.0f, g_arranger_max_bars - g_arranger_visible_bars));
                    }
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Zoom Out Timeline (Ctrl+Wheel Down)");

                    ImGui::SameLine();
                    if (ImGui::SmallButton(" + ##arr_zm_in")) {
                        float new_v = std::max(g_arranger_min_bars, g_arranger_visible_bars * 0.8f);
                        g_arranger_visible_bars = new_v;
                        g_arranger_scroll_bar = std::clamp(g_arranger_scroll_bar, 0.0f, std::max(0.0f, g_arranger_max_bars - g_arranger_visible_bars));
                    }
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Zoom In Timeline (Ctrl+Wheel Up)");

                    ImGui::SameLine();
                    if (ImGui::SmallButton("FIT 16B")) {
                        g_arranger_visible_bars = 16.0f;
                        g_arranger_scroll_bar = 0.0f;
                    }
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Reset Viewport to 16 Bars");

                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(0.12f, 0.45f, 0.95f, 1.0f), "VIEW: %.0fb (%.1f-%.1fb)",
                                       g_arranger_visible_bars, g_arranger_scroll_bar + 1.0f, g_arranger_scroll_bar + g_arranger_visible_bars + 1.0f);
                    ImGui::PopStyleVar(2);

                    const float top_pane_h = std::max(avail_sz.y - 52.0f, 130.0f);
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

                        // Dynamic Viewport Coordinate Transforms (Zoom & Horizontal Pan)
                        const float bar_w = canvas_size.x / g_arranger_visible_bars;
                        auto bar_to_x = [&](float bar) -> float {
                            return canvas_pos.x + (bar - g_arranger_scroll_bar) * bar_w;
                        };
                        auto x_to_bar = [&](float px) -> float {
                            return g_arranger_scroll_bar + (px - canvas_pos.x) / bar_w;
                        };

                        draw_list->PushClipRect(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), true);

                        // Dynamic Bar Grid Lines
                        int b_start = static_cast<int>(std::floor(g_arranger_scroll_bar));
                        int b_end = static_cast<int>(std::ceil(g_arranger_scroll_bar + g_arranger_visible_bars));
                        for (int b = b_start; b <= b_end; ++b) {
                            float bx = bar_to_x(static_cast<float>(b));
                            if (bx >= canvas_pos.x - 1.0f && bx <= canvas_pos.x + canvas_size.x + 1.0f) {
                                draw_list->AddLine(ImVec2(bx, canvas_pos.y),
                                                   ImVec2(bx, canvas_pos.y + canvas_size.y),
                                                   ImColor(230, 235, 242, 255), 1.0f);
                                char b_txt[16];
                                std::snprintf(b_txt, sizeof(b_txt), "%d.1", b + 1);
                                draw_list->AddText(ImVec2(bx + 4.0f, canvas_pos.y + 2.0f),
                                                   ImColor(100, 110, 125, 255), b_txt);
                            }
                            if (bar_w >= 50.0f) {
                                for (int q = 1; q <= 3; ++q) {
                                    float qx = bar_to_x(static_cast<float>(b) + q * 0.25f);
                                    if (qx >= canvas_pos.x && qx <= canvas_pos.x + canvas_size.x) {
                                        draw_list->AddLine(ImVec2(qx, canvas_pos.y + 14.0f),
                                                           ImVec2(qx, canvas_pos.y + canvas_size.y),
                                                           ImColor(240, 243, 248, 180), 1.0f);
                                    }
                                }
                            }
                        }

                        // Interactive Loop Region & Track Lane Tint
                        float loop_x1 = bar_to_x(loop_start_bar);
                        float loop_x2 = bar_to_x(loop_start_bar + loop_bars);
                        float draw_lx1 = std::max(canvas_pos.x, loop_x1);
                        float draw_lx2 = std::min(canvas_pos.x + canvas_size.x, loop_x2);

                        if (draw_lx2 > draw_lx1) {
                            // Subtle Loop Lane Tint across all 4 Tracks
                            ImU32 track_tint = loop_active ? ImColor(31, 97, 217, 18) : ImColor(140, 145, 160, 10);
                            draw_list->AddRectFilled(ImVec2(draw_lx1, canvas_pos.y + 20.0f),
                                                     ImVec2(draw_lx2, canvas_pos.y + canvas_size.y),
                                                     track_tint);

                            // Loop Body Strip in Ruler
                            ImU32 loop_bar_bg = loop_active ? ImColor(31, 97, 217, 65) : ImColor(120, 130, 145, 40);
                            ImU32 loop_bar_border = loop_active ? ImColor(31, 97, 217, 230) : ImColor(120, 130, 145, 160);

                            draw_list->AddRectFilled(ImVec2(draw_lx1, canvas_pos.y + 1.0f),
                                                     ImVec2(draw_lx2, canvas_pos.y + 19.0f),
                                                     loop_bar_bg, 2.0f);
                            draw_list->AddLine(ImVec2(draw_lx1, canvas_pos.y + 19.0f),
                                               ImVec2(draw_lx2, canvas_pos.y + 19.0f),
                                               loop_bar_border, 2.0f);
                            draw_list->AddLine(ImVec2(draw_lx1, canvas_pos.y + 1.0f),
                                               ImVec2(draw_lx2, canvas_pos.y + 1.0f),
                                               loop_bar_border, 1.5f);

                            char loop_txt[64];
                            std::snprintf(loop_txt, sizeof(loop_txt), "[ 🔁 LOOP: %.2f-%.2f (%.1fb) %s ]",
                                          loop_start_bar + 1.0f, loop_start_bar + loop_bars + 1.0f, loop_bars,
                                          loop_active ? "ON" : "OFF");
                            draw_list->AddText(ImVec2(draw_lx1 + 10.0f, canvas_pos.y + 2.0f),
                                               loop_active ? ImColor(31, 97, 217, 255) : ImColor(100, 110, 125, 220), loop_txt);
                        }

                        // Left Bracket Handle [L]
                        ImU32 handle_col = loop_active ? ImColor(37, 99, 235, 255) : ImColor(100, 110, 125, 255);
                        if (loop_x1 >= canvas_pos.x - 8.0f && loop_x1 <= canvas_pos.x + canvas_size.x + 8.0f) {
                            draw_list->AddRectFilled(ImVec2(loop_x1 - 2.0f, canvas_pos.y + 1.0f),
                                                     ImVec2(loop_x1 + 3.0f, canvas_pos.y + 19.0f),
                                                     handle_col, 1.0f);
                            draw_list->AddTriangleFilled(ImVec2(loop_x1 + 3.0f, canvas_pos.y + 3.0f),
                                                         ImVec2(loop_x1 + 8.0f, canvas_pos.y + 7.0f),
                                                         ImVec2(loop_x1 + 3.0f, canvas_pos.y + 11.0f),
                                                         handle_col);
                            draw_list->AddText(ImVec2(loop_x1 - 9.0f, canvas_pos.y + 2.0f), handle_col, "L");
                        }

                        // Right Bracket Handle [R]
                        if (loop_x2 >= canvas_pos.x - 8.0f && loop_x2 <= canvas_pos.x + canvas_size.x + 8.0f) {
                            draw_list->AddRectFilled(ImVec2(loop_x2 - 3.0f, canvas_pos.y + 1.0f),
                                                     ImVec2(loop_x2 + 2.0f, canvas_pos.y + 19.0f),
                                                     handle_col, 1.0f);
                            draw_list->AddTriangleFilled(ImVec2(loop_x2 - 3.0f, canvas_pos.y + 3.0f),
                                                         ImVec2(loop_x2 - 8.0f, canvas_pos.y + 7.0f),
                                                         ImVec2(loop_x2 - 3.0f, canvas_pos.y + 11.0f),
                                                         handle_col);
                            draw_list->AddText(ImVec2(loop_x2 + 4.0f, canvas_pos.y + 2.0f), handle_col, "R");
                        }

                        // Punch-In / Punch-Out Region Highlighting & Crimson Ruler Flags
                        if (g_punch_enabled) {
                            float p_in = g_punch_link_to_loop ? loop_start_bar : g_punch_in_bar;
                            float p_out = g_punch_link_to_loop ? (loop_start_bar + loop_bars) : g_punch_out_bar;
                            float p_x1 = bar_to_x(p_in);
                            float p_x2 = bar_to_x(p_out);
                            float draw_px1 = std::max(canvas_pos.x, p_x1);
                            float draw_px2 = std::min(canvas_pos.x + canvas_size.x, p_x2);

                            ImU32 punch_flag_col = g_punch_is_recording ? ImColor(239, 68, 68, 255) : ImColor(220, 38, 38, 240);
                            if (draw_px2 > draw_px1) {
                                draw_list->AddLine(ImVec2(draw_px1, canvas_pos.y), ImVec2(draw_px2, canvas_pos.y), punch_flag_col, 2.5f);
                            }

                            // Punch-In Flag [▶| IN] in Ruler
                            if (p_x1 >= canvas_pos.x - 10.0f && p_x1 <= canvas_pos.x + canvas_size.x + 10.0f) {
                                draw_list->AddLine(ImVec2(p_x1, canvas_pos.y), ImVec2(p_x1, canvas_pos.y + 20.0f), punch_flag_col, 2.0f);
                                draw_list->AddTriangleFilled(ImVec2(p_x1, canvas_pos.y + 10.0f),
                                                             ImVec2(p_x1 + 7.0f, canvas_pos.y + 14.0f),
                                                             ImVec2(p_x1, canvas_pos.y + 18.0f), punch_flag_col);
                                draw_list->AddText(ImVec2(p_x1 + 3.0f, canvas_pos.y + 1.0f), punch_flag_col, "IN");
                            }

                            // Punch-Out Flag [|◀ OUT] in Ruler
                            if (p_x2 >= canvas_pos.x - 10.0f && p_x2 <= canvas_pos.x + canvas_size.x + 10.0f) {
                                draw_list->AddLine(ImVec2(p_x2, canvas_pos.y), ImVec2(p_x2, canvas_pos.y + 20.0f), punch_flag_col, 2.0f);
                                draw_list->AddTriangleFilled(ImVec2(p_x2, canvas_pos.y + 10.0f),
                                                             ImVec2(p_x2 - 7.0f, canvas_pos.y + 14.0f),
                                                             ImVec2(p_x2, canvas_pos.y + 18.0f), punch_flag_col);
                                draw_list->AddText(ImVec2(p_x2 - 24.0f, canvas_pos.y + 1.0f), punch_flag_col, "OUT");
                            }
                        }

                        // 4 Track Lanes
                        const float lane_h = (canvas_size.y - 20.0f) / 4.0f;
                        for (int t = 0; t < 4; ++t) {
                            float ly = canvas_pos.y + 20.0f + t * lane_h;

                            // Track Header & Bus Destination Badge (Zähl AM1 Routing Clarity)
                            {
                                char trk_hdr_str[64];
                                std::snprintf(trk_hdr_str, sizeof(trk_hdr_str), "%s", track_names[t]);
                                draw_list->AddText(ImVec2(canvas_pos.x + 8.0f, ly + 4.0f),
                                                   ImColor(60, 75, 95, 230), trk_hdr_str);

                                const auto& badge = k_track_routing_badges[t];
                                ImVec2 b_size = ImGui::CalcTextSize(badge.bus_name);
                                float b_x = canvas_pos.x + 8.0f + ImGui::CalcTextSize(trk_hdr_str).x + 10.0f;
                                float b_y = ly + 3.0f;
                                draw_list->AddRectFilled(ImVec2(b_x - 4.0f, b_y), ImVec2(b_x + b_size.x + 4.0f, b_y + b_size.y + 2.0f),
                                                         badge.bg_color, 3.0f);
                                draw_list->AddText(ImVec2(b_x, b_y + 1.0f), badge.text_color, badge.bus_name);
                            }
                            draw_list->AddLine(ImVec2(canvas_pos.x, ly),
                                               ImVec2(canvas_pos.x + canvas_size.x, ly),
                                               ImColor(230, 235, 242, 255), 1.0f);

                            if (selected_track == t) {
                                draw_list->AddRectFilled(ImVec2(canvas_pos.x, ly),
                                                         ImVec2(canvas_pos.x + canvas_size.x, ly + lane_h),
                                                         ImColor(31, 97, 217, 18));
                            }

                            // Punch Zone Tint on Selected Track Lane
                            if (g_punch_enabled && selected_track == t) {
                                float p_in = g_punch_link_to_loop ? loop_start_bar : g_punch_in_bar;
                                float p_out = g_punch_link_to_loop ? (loop_start_bar + loop_bars) : g_punch_out_bar;
                                float px1 = std::max(canvas_pos.x, bar_to_x(p_in));
                                float px2 = std::min(canvas_pos.x + canvas_size.x, bar_to_x(p_out));
                                if (px2 > px1) {
                                    ImU32 p_bg = g_punch_is_recording ? ImColor(220, 38, 38, 55) : ImColor(220, 38, 38, 20);
                                    ImU32 p_border = g_punch_is_recording ? ImColor(239, 68, 68, 240) : ImColor(220, 38, 38, 120);
                                    draw_list->AddRectFilled(ImVec2(px1, ly + 1.0f), ImVec2(px2, ly + lane_h - 1.0f), p_bg);
                                    draw_list->AddRect(ImVec2(px1, ly + 1.0f), ImVec2(px2, ly + lane_h - 1.0f), p_border, 2.0f);
                                    char p_zone_lbl[64];
                                    std::snprintf(p_zone_lbl, sizeof(p_zone_lbl), "[ %s ]",
                                                  g_punch_is_recording ? "⏺ RECORDING OVERDUB TAKE..." : "⏺ PUNCH OVERDUB ZONE");
                                    draw_list->AddText(ImVec2(px1 + 8.0f, ly + lane_h - 18.0f), ImColor(220, 38, 38, 240), p_zone_lbl);
                                }
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
                                // Multi-Clip Timeline Lanes
                                for (size_t c_idx = 0; c_idx < track_arranger_clips[t].size(); ++c_idx) {
                                    auto& c = track_arranger_clips[t][c_idx];
                                    float clip_x1 = bar_to_x(c.start_bar);
                                    float clip_x2 = bar_to_x(c.end_bar());
                                    // Viewport Culling: Skip clip if completely off-screen
                                    if (clip_x2 < canvas_pos.x - 2.0f || clip_x1 > canvas_pos.x + canvas_size.x + 2.0f) {
                                        continue;
                                    }
                                    bool is_sel = (selected_track == t && static_cast<int>(c_idx) == g_selected_clip_idx);

                                    bool is_pat_clip = (c.type == sequencer::ArrangerClipType::Pattern);

                                    if (is_pat_clip) {
                                        // Pattern Clip Box (Warm Amber)
                                        draw_list->AddRectFilled(ImVec2(clip_x1 + 1.0f, ly + 2.0f),
                                                                 ImVec2(clip_x2 - 1.0f, ly + lane_h - 2.0f),
                                                                 is_sel ? ImColor(254, 243, 199, 250) : ImColor(255, 251, 235, 235),
                                                                 3.0f);
                                        draw_list->AddRect(ImVec2(clip_x1 + 1.0f, ly + 2.0f),
                                                           ImVec2(clip_x2 - 1.0f, ly + lane_h - 2.0f),
                                                           is_sel ? ImColor(217, 119, 6, 255) : ImColor(245, 158, 11, 200),
                                                           3.0f, 0, is_sel ? 2.0f : 1.0f);

                                        // Pattern Header Ribbon
                                        draw_list->AddRectFilled(ImVec2(clip_x1 + 1.0f, ly + 2.0f),
                                                                 ImVec2(clip_x2 - 1.0f, ly + 14.0f),
                                                                 is_sel ? ImColor(245, 158, 11, 140) : ImColor(217, 119, 6, 70),
                                                                 3.0f, ImDrawFlags_RoundCornersTop);

                                        // Tracker Step Matrix
                                        if (track_seq[t] && c.pattern_index < sequencer::StepSequencer::kMaxPatterns) {
                                            const auto& pat = track_seq[t]->pattern(c.pattern_index);
                                            uint32_t num_st = (pat.num_steps > 0) ? pat.num_steps : 16;
                                            float step_dur_bars = (pat.subdivision == sequencer::StepSubdivision::ThirtySecond) ? (1.0f / 32.0f) :
                                                                  ((pat.subdivision == sequencer::StepSubdivision::Eighth) ? (1.0f / 8.0f) : (1.0f / 16.0f));
                                            float pat_dur_bars = static_cast<float>(num_st) * step_dur_bars;
                                            if (pat_dur_bars <= 0.0001f) pat_dur_bars = 1.0f;

                                            float vis_x1 = std::max(clip_x1 + 2.0f, canvas_pos.x);
                                            float vis_x2 = std::min(clip_x2 - 2.0f, canvas_pos.x + canvas_size.x);

                                            float grid_top = ly + 16.0f;
                                            float grid_bottom = ly + lane_h - 4.0f;
                                            float grid_h = grid_bottom - grid_top;

                                            float cur_rel_b = 0.0f;
                                            while (cur_rel_b < c.len_bars - 0.0001f) {
                                                float pat_time = std::fmod(cur_rel_b + c.offset_bars, pat_dur_bars);
                                                if (pat_time < 0.0f) pat_time += pat_dur_bars;
                                                uint32_t st = static_cast<uint32_t>(pat_time / step_dur_bars);
                                                if (st >= num_st) st = num_st - 1;

                                                float next_step_time = (static_cast<float>(st) + 1.0f) * step_dur_bars;
                                                float step_rem_in_pat = next_step_time - pat_time;
                                                float cur_step_len = std::min(step_rem_in_pat, c.len_bars - cur_rel_b);
                                                cur_step_len = std::max(cur_step_len, 0.0005f);

                                                float sx1 = bar_to_x(c.start_bar + cur_rel_b);
                                                float sx2 = bar_to_x(c.start_bar + cur_rel_b + cur_step_len);

                                                if (sx2 >= vis_x1 && sx1 <= vis_x2) {
                                                    bool is_beat = (st % 4 == 0);
                                                    draw_list->AddLine(ImVec2(sx1, grid_top), ImVec2(sx1, grid_bottom),
                                                                       is_beat ? ImColor(217, 119, 6, 80) : ImColor(217, 119, 6, 30),
                                                                       is_beat ? 1.0f : 0.5f);

                                                    const auto& step_trig = pat.steps[st];
                                                    if (step_trig.active) {
                                                        float vel = std::clamp(step_trig.velocity, 0.1f, 1.0f);
                                                        float bar_h = vel * (grid_h - 4.0f);
                                                        float bx1 = sx1 + 1.0f;
                                                        float bx2 = std::max(bx1 + 2.0f, sx2 - 1.0f);
                                                        float by2 = grid_bottom - 2.0f;
                                                        float by1 = by2 - bar_h;

                                                        ImColor trig_col;
                                                        if (t == 0) {
                                                            if (step_trig.slice_id == 0) trig_col = ImColor(239, 68, 68, 230);
                                                            else if (step_trig.slice_id == 1) trig_col = ImColor(245, 158, 11, 230);
                                                            else trig_col = ImColor(14, 165, 233, 230);
                                                        } else if (t == 1) {
                                                            trig_col = ImColor(16, 185, 129, 230);
                                                        } else if (t == 2) {
                                                            trig_col = ImColor(168, 85, 247, 230);
                                                        } else {
                                                            trig_col = ImColor(249, 115, 22, 230);
                                                        }

                                                        draw_list->AddRectFilled(ImVec2(bx1, by1), ImVec2(bx2, by2), trig_col, 1.5f);
                                                        draw_list->AddLine(ImVec2(bx1, by1), ImVec2(bx2, by1), ImColor(255, 255, 255, 220), 1.0f);
                                                    }

                                                    float playhead_bar = playhead_seconds / get_seconds_per_bar(bpm);
                                                    if (playhead_bar >= (c.start_bar + cur_rel_b) && playhead_bar < (c.start_bar + cur_rel_b + cur_step_len)) {
                                                        draw_list->AddRectFilled(ImVec2(sx1, grid_top), ImVec2(sx2, grid_bottom), ImColor(245, 158, 11, 55));
                                                        draw_list->AddRect(ImVec2(sx1, grid_top), ImVec2(sx2, grid_bottom), ImColor(251, 191, 36, 220), 1.0f);
                                                    }
                                                }

                                                cur_rel_b += cur_step_len;
                                            }
                                        }
                                    } else {
                                        // Main Clip Body Box (Audio pastel blue)
                                        draw_list->AddRectFilled(ImVec2(clip_x1 + 1.0f, ly + 2.0f),
                                                                 ImVec2(clip_x2 - 1.0f, ly + lane_h - 2.0f),
                                                                 is_sel ? ImColor(220, 235, 255, 245) : ImColor(240, 245, 252, 230),
                                                                 3.0f);
                                        draw_list->AddRect(ImVec2(clip_x1 + 1.0f, ly + 2.0f),
                                                           ImVec2(clip_x2 - 1.0f, ly + lane_h - 2.0f),
                                                           is_sel ? ImColor(31, 97, 217, 255) : ImColor(160, 180, 210, 200),
                                                           3.0f, 0, is_sel ? 2.0f : 1.0f);

                                        // Mini Waveform preview inside clip with Slip-Editing Sliding Window
                                        auto clip_ptr = c.clip ? c.clip : track_clips[t];
                                        if (clip_ptr && clip_ptr->num_frames() > 0 && clip_ptr->channel(0)) {
                                            const float* ch0 = clip_ptr->channel(0);
                                            const uint32_t total_f = clip_ptr->num_frames();
                                            float wf_mid_y = ly + lane_h * 0.58f;
                                            float max_h = (lane_h - 18.0f) * 0.42f;
                                            float vis_x1 = std::max(clip_x1 + 3.0f, canvas_pos.x);
                                            float vis_x2 = std::min(clip_x2 - 3.0f, canvas_pos.x + canvas_size.x);
                                            const float* ch1 = (clip_ptr->num_channels() > 1 && clip_ptr->channel(1)) ? clip_ptr->channel(1) : nullptr;

                                            if (vis_x1 < vis_x2) {
                                                constexpr float step_px = 2.0f;
                                                for (float px = vis_x1; px < vis_x2; px += step_px) {
                                                    float bar_s = x_to_bar(px);
                                                    float bar_e = x_to_bar(px + step_px);
                                                    float rel_b_s = std::clamp(bar_s - c.start_bar, 0.0f, c.len_bars);
                                                    float rel_b_e = std::clamp(bar_e - c.start_bar, 0.0f, c.len_bars);
                                                    if (rel_b_e <= rel_b_s) continue;

                                                    double raw_s = c.evaluate_warped_frame(rel_b_s, total_f);
                                                    double raw_e = c.evaluate_warped_frame(rel_b_e, total_f);
                                                    double frame_s = std::fmod(raw_s, static_cast<double>(total_f));
                                                    if (frame_s < 0.0) frame_s += total_f;
                                                    double frame_e = std::fmod(raw_e, static_cast<double>(total_f));
                                                    if (frame_e < 0.0) frame_e += total_f;

                                                    uint32_t f_start = static_cast<uint32_t>(std::clamp(frame_s, 0.0, static_cast<double>(total_f - 1)));
                                                    uint32_t f_end = static_cast<uint32_t>(std::clamp(frame_e, 0.0, static_cast<double>(total_f)));
                                                    if (f_end <= f_start) f_end = total_f;

                                                    float peak_val = 0.0f;
                                                    uint32_t span = f_end - f_start;
                                                    uint32_t stride = std::max(1u, span / 32u);
                                                    for (uint32_t fi = f_start; fi < f_end; fi += stride) {
                                                        float s = std::abs(ch0[fi]);
                                                        if (ch1) s = std::max(s, std::abs(ch1[fi]));
                                                        if (s > peak_val) peak_val = s;
                                                    }

                                                    float col_bar = 0.5f * (bar_s + bar_e);
                                                    peak_val *= c.evaluate_gain_at_bar(col_bar);

                                                    float h = std::clamp(peak_val * max_h, 1.0f, max_h);
                                                    draw_list->AddLine(ImVec2(px, wf_mid_y - h),
                                                                       ImVec2(px, wf_mid_y + h),
                                                                       is_sel ? ImColor(31, 97, 217, 160) : ImColor(100, 130, 170, 130), 1.5f);
                                                }
                                            }

                                            // Transient Markers (Unpinned ticks)
                                            for (const auto& span : c.detected_transients) {
                                                float rel_b = static_cast<float>(c.frame_to_bar(span.peak_frame, total_f));
                                                float tx = bar_to_x(c.start_bar + rel_b);
                                                if (tx >= clip_x1 + 3.0f && tx <= clip_x2 - 3.0f) {
                                                    bool is_pinned = false;
                                                    for (const auto& pin : c.warp_pins) {
                                                        if (pin.source_frame >= span.start_frame && pin.source_frame <= span.decay_end_frame) {
                                                            is_pinned = true;
                                                            break;
                                                        }
                                                    }
                                                    if (!is_pinned) {
                                                        ImColor tick_col = (span.band == analysis::TransientBand::SubBass) ? ImColor(239, 68, 68, 160) :
                                                                           ((span.band == analysis::TransientBand::MidPunch) ? ImColor(59, 130, 246, 170) :
                                                                           ImColor(16, 185, 129, 170));
                                                        draw_list->AddLine(ImVec2(tx, wf_mid_y - max_h * 0.75f),
                                                                           ImVec2(tx, wf_mid_y + max_h * 0.75f),
                                                                           tick_col, 1.0f);
                                                        if (g_arranger_tool == ArrangerTool::Warp || is_sel) {
                                                            draw_list->AddTriangleFilled(ImVec2(tx - 3.0f, ly + 2.0f),
                                                                                         ImVec2(tx + 3.0f, ly + 2.0f),
                                                                                         ImVec2(tx, ly + 6.0f),
                                                                                         tick_col);
                                                        }
                                                    }
                                                }
                                            }

                                            // Pinned Warp-Pins (prominent amber diamond flags)
                                            for (const auto& pin : c.warp_pins) {
                                                float px = bar_to_x(c.start_bar + pin.pinned_bar);
                                                if (px >= clip_x1 && px <= clip_x2) {
                                                    bool is_dragged = (g_arranger_drag_mode == ArrangerDragMode::WarpPinDrag &&
                                                                      g_arranger_drag_warp_pin_id == pin.id);
                                                    ImColor pin_col = is_dragged ? ImColor(251, 191, 36, 255) : ImColor(245, 158, 11, 230);
                                                    draw_list->AddLine(ImVec2(px, ly + 2.0f), ImVec2(px, ly + lane_h - 2.0f),
                                                                      pin_col, is_dragged ? 2.5f : 1.8f);
                                                    draw_list->AddQuadFilled(ImVec2(px, ly + 2.0f),
                                                                            ImVec2(px + 4.5f, ly + 7.0f),
                                                                            ImVec2(px, ly + 12.0f),
                                                                            ImVec2(px - 4.5f, ly + 7.0f),
                                                                            pin_col);
                                                    draw_list->AddQuad(ImVec2(px, ly + 2.0f),
                                                                      ImVec2(px + 4.5f, ly + 7.0f),
                                                                      ImVec2(px, ly + 12.0f),
                                                                      ImVec2(px - 4.5f, ly + 7.0f),
                                                                      ImColor(255, 255, 255, 230), 1.0f);
                                                }
                                            }
                                        }
                                    }

                                    if (!is_pat_clip) {
                                        constexpr float kPiOver2 = 1.5707963267948966f;

                                        // Fade In Shaded Polygon & Grab Handle
                                        float fin_b = c.fade_in_bars;
                                        if (fin_b > 0.01f) {
                                            float fin_w = fin_b * bar_w;
                                            fin_w = std::min(fin_w, (clip_x2 - clip_x1) * 0.95f);
                                            float hx = clip_x1 + fin_w;

                                            constexpr int kRampSteps = 16;
                                            ImVec2 poly_pts[kRampSteps + 2];
                                            poly_pts[0] = ImVec2(clip_x1 + 1.0f, ly + 2.0f);
                                            for (int si = 0; si <= kRampSteps; ++si) {
                                                float u = static_cast<float>(si) / static_cast<float>(kRampSteps);
                                                float px = clip_x1 + 1.0f + u * fin_w;
                                                float gain = u;
                                                switch (c.fade_in_shape) {
                                                    case sampling::FadeShape::Linear:      gain = u; break;
                                                    case sampling::FadeShape::Exponential: gain = std::pow(u, 2.8f); break;
                                                    case sampling::FadeShape::Logarithmic: gain = 1.0f - std::pow(1.0f - u, 2.8f); break;
                                                    case sampling::FadeShape::SCurve:      gain = u * u * (3.0f - 2.0f * u); break;
                                                    case sampling::FadeShape::EqualPower:  gain = std::sin(kPiOver2 * u); break;
                                                }
                                                float py = ly + 2.0f + (1.0f - gain) * (lane_h - 4.0f);
                                                poly_pts[si + 1] = ImVec2(px, py);
                                            }
                                            ImColor poly_col = c.is_auto_crossfade_in ? ImColor(168, 85, 247, 55) : ImColor(245, 158, 11, 45);
                                            ImColor line_col = c.is_auto_crossfade_in ? ImColor(168, 85, 247, 220) : ImColor(217, 119, 6, 220);
                                            draw_list->AddConvexPolyFilled(poly_pts, kRampSteps + 2, poly_col);

                                            for (int si = 0; si < kRampSteps; ++si) {
                                                draw_list->AddLine(poly_pts[si + 1], poly_pts[si + 2], line_col, 1.8f);
                                            }

                                            draw_list->AddTriangleFilled(ImVec2(hx - 4.0f, ly + 2.0f),
                                                                         ImVec2(hx + 4.0f, ly + 2.0f),
                                                                         ImVec2(hx, ly + 9.0f),
                                                                         line_col);
                                        } else {
                                            draw_list->AddTriangleFilled(ImVec2(clip_x1 + 1.0f, ly + 2.0f),
                                                                         ImVec2(clip_x1 + 8.0f, ly + 2.0f),
                                                                         ImVec2(clip_x1 + 1.0f, ly + 9.0f),
                                                                         ImColor(180, 190, 205, 180));
                                        }

                                        // Fade Out Shaded Polygon & Grab Handle
                                        float fout_b = c.fade_out_bars;
                                        if (fout_b > 0.01f) {
                                            float fout_w = fout_b * bar_w;
                                            fout_w = std::min(fout_w, (clip_x2 - clip_x1) * 0.95f);
                                            float hx = clip_x2 - fout_w;

                                            constexpr int kRampSteps = 16;
                                            ImVec2 poly_pts[kRampSteps + 2];
                                            poly_pts[0] = ImVec2(clip_x2 - 1.0f, ly + 2.0f);
                                            for (int si = 0; si <= kRampSteps; ++si) {
                                                float u = static_cast<float>(si) / static_cast<float>(kRampSteps);
                                                float px = clip_x2 - 1.0f - (1.0f - u) * fout_w;
                                                float gain = 1.0f - u;
                                                switch (c.fade_out_shape) {
                                                    case sampling::FadeShape::Linear:      gain = 1.0f - u; break;
                                                    case sampling::FadeShape::Exponential: gain = std::pow(1.0f - u, 2.8f); break;
                                                    case sampling::FadeShape::Logarithmic: gain = 1.0f - (1.0f - std::pow(u, 2.8f)); break;
                                                    case sampling::FadeShape::SCurve:      gain = 1.0f - (u * u * (3.0f - 2.0f * u)); break;
                                                    case sampling::FadeShape::EqualPower:  gain = std::sin(kPiOver2 * (1.0f - u)); break;
                                                }
                                                float py = ly + 2.0f + (1.0f - gain) * (lane_h - 4.0f);
                                                poly_pts[si + 1] = ImVec2(px, py);
                                            }
                                            ImColor poly_col = c.is_auto_crossfade_out ? ImColor(168, 85, 247, 55) : ImColor(245, 158, 11, 45);
                                            ImColor line_col = c.is_auto_crossfade_out ? ImColor(168, 85, 247, 220) : ImColor(217, 119, 6, 220);
                                            draw_list->AddConvexPolyFilled(poly_pts, kRampSteps + 2, poly_col);

                                            for (int si = 0; si < kRampSteps; ++si) {
                                                draw_list->AddLine(poly_pts[si + 1], poly_pts[si + 2], line_col, 1.8f);
                                            }

                                            draw_list->AddTriangleFilled(ImVec2(hx - 4.0f, ly + 2.0f),
                                                                         ImVec2(hx + 4.0f, ly + 2.0f),
                                                                         ImVec2(hx, ly + 9.0f),
                                                                         line_col);
                                        } else {
                                            draw_list->AddTriangleFilled(ImVec2(clip_x2 - 8.0f, ly + 2.0f),
                                                                         ImVec2(clip_x2 - 1.0f, ly + 2.0f),
                                                                         ImVec2(clip_x2 - 1.0f, ly + 9.0f),
                                                                         ImColor(180, 190, 205, 180));
                                        }
                                    }

                                    // Clip Title, Sononym Key & Slip/Fade Info
                                    char clip_label[160];
                                    if (is_pat_clip) {
                                        std::string pat_name = "Pattern";
                                        if (track_seq[t] && c.pattern_index < sequencer::StepSequencer::kMaxPatterns) {
                                            pat_name = track_seq[t]->pattern(c.pattern_index).name;
                                        }
                                        char extra_meta[80] = "";
                                        if (c.offset_bars > 0.05f) {
                                            std::snprintf(extra_meta, sizeof(extra_meta), " [slip: +%.1fb]", c.offset_bars);
                                        }
                                        std::snprintf(clip_label, sizeof(clip_label), "[ %s ] #%d%s (%.1f-%.1fb)",
                                                      pat_name.c_str(), c.pattern_index + 1, extra_meta, c.start_bar + 1.0f, c.end_bar() + 1.0f);
                                        draw_list->AddText(ImVec2(clip_x1 + 6.0f, ly + 3.0f),
                                                           is_sel ? ImColor(120, 53, 15, 255) : ImColor(180, 83, 9, 255),
                                                           clip_label);
                                    } else {
                                        auto clip_ptr = c.clip ? c.clip : track_clips[t];
                                        auto desc = sampling::SampleAssetPool::instance().get_descriptor(
                                            clip_ptr ? sampling::SampleAnalyzer::analyze(*clip_ptr).asset_id : "");
                                        const char* key_str = (desc && !desc->musical_key.empty() && desc->musical_key != "--") ? desc->musical_key.c_str() : "";

                                        char extra_meta[80] = "";
                                        if (std::abs(c.stretch_ratio - 1.0f) > 0.01f || std::abs(c.pitch_semitones) > 0.01f) {
                                            const char* algo_str = (c.stretch_algo == dsp::PitchAlgorithm::TransientWarpWsola) ? "T-WARP" :
                                                                   ((c.stretch_algo == dsp::PitchAlgorithm::RubberbandWsola) ? "WSOLA" :
                                                                   ((c.stretch_algo == dsp::PitchAlgorithm::SovereignOde) ? "ODE" :
                                                                   ((c.stretch_algo == dsp::PitchAlgorithm::VintageMpc) ? "MPC12" :
                                                                   ((c.stretch_algo == dsp::PitchAlgorithm::DeRezSampler) ? "DEREZ" : "VINYL"))));
                                            if (std::abs(c.pitch_semitones) > 0.01f) {
                                                std::snprintf(extra_meta, sizeof(extra_meta), " [%.2fx %s %+.1fst]", c.stretch_ratio, algo_str, c.pitch_semitones);
                                            } else {
                                                std::snprintf(extra_meta, sizeof(extra_meta), " [%.2fx %s]", c.stretch_ratio, algo_str);
                                            }
                                        } else if (c.offset_bars > 0.05f) {
                                            std::snprintf(extra_meta, sizeof(extra_meta), " [slip: +%.1fb]", c.offset_bars);
                                        }

                                        if (std::strlen(key_str) > 0) {
                                            std::snprintf(clip_label, sizeof(clip_label), "%s [%s]%s (%.1f-%.1fb)",
                                                          c.name.c_str(), key_str, extra_meta, c.start_bar + 1.0f, c.end_bar() + 1.0f);
                                        } else {
                                            std::snprintf(clip_label, sizeof(clip_label), "%s%s (%.1f-%.1fb)",
                                                          c.name.c_str(), extra_meta, c.start_bar + 1.0f, c.end_bar() + 1.0f);
                                        }
                                        draw_list->AddText(ImVec2(clip_x1 + 6.0f, ly + 3.0f),
                                                           is_sel ? ImColor(10, 30, 80, 255) : ImColor(60, 75, 100, 255),
                                                           clip_label);
                                    }

                                    // Trim / Stretch Handles visual indicators on edges
                                    ImColor edge_col = (g_arranger_tool == ArrangerTool::Stretch) ? ImColor(245, 158, 11, 230) :
                                                       (is_sel ? (is_pat_clip ? ImColor(217, 119, 6, 220) : ImColor(31, 97, 217, 200)) :
                                                                 (is_pat_clip ? ImColor(245, 158, 11, 160) : ImColor(160, 180, 205, 160)));
                                    draw_list->AddLine(ImVec2(clip_x1 + 4.0f, ly + 5.0f), ImVec2(clip_x1 + 4.0f, ly + lane_h - 5.0f),
                                                       edge_col, 2.0f);
                                    draw_list->AddLine(ImVec2(clip_x2 - 4.0f, ly + 5.0f), ImVec2(clip_x2 - 4.0f, ly + lane_h - 5.0f),
                                                       edge_col, 2.0f);

                                    // Auto-Crossfade Overlap Box with Adjacent Clip
                                    if (!is_pat_clip && c_idx + 1 < track_arranger_clips[t].size()) {
                                        const auto& next_c = track_arranger_clips[t][c_idx + 1];
                                        if (c.end_bar() > next_c.start_bar + 0.005f) {
                                            float xf_x1 = bar_to_x(next_c.start_bar);
                                            float xf_x2 = bar_to_x(std::min(c.end_bar(), next_c.end_bar()));

                                            if (xf_x2 >= canvas_pos.x && xf_x1 <= canvas_pos.x + canvas_size.x) {
                                                draw_list->AddRectFilled(ImVec2(xf_x1, ly + 2.0f), ImVec2(xf_x2, ly + lane_h - 2.0f),
                                                                         ImColor(147, 51, 234, 45), 2.0f);
                                                draw_list->AddRect(ImVec2(xf_x1, ly + 2.0f), ImVec2(xf_x2, ly + lane_h - 2.0f),
                                                                   ImColor(168, 85, 247, 180), 2.0f, 0, 1.0f);

                                                draw_list->AddLine(ImVec2(xf_x1, ly + 4.0f), ImVec2(xf_x2, ly + lane_h - 4.0f),
                                                                   ImColor(168, 85, 247, 220), 1.5f);
                                                draw_list->AddLine(ImVec2(xf_x1, ly + lane_h - 4.0f), ImVec2(xf_x2, ly + 4.0f),
                                                                   ImColor(168, 85, 247, 220), 1.5f);

                                                draw_list->AddText(ImVec2(xf_x1 + 3.0f, ly + lane_h * 0.5f - 6.0f),
                                                                   ImColor(126, 34, 206, 255), "[X-FADE]");
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        // Ghost Target Lane Rendering during Vertical Cross-Track Drag
                        if (g_arranger_drag_mode == ArrangerDragMode::MoveClip && g_arranger_drag_track >= 0 &&
                            g_arranger_drag_hover_track >= 0 && g_arranger_drag_hover_track != g_arranger_drag_track &&
                            g_arranger_drag_clip_idx >= 0 && g_arranger_drag_clip_idx < static_cast<int>(track_arranger_clips[g_arranger_drag_track].size())) {
                            const auto& c = track_arranger_clips[g_arranger_drag_track][g_arranger_drag_clip_idx];
                            float gh_ly = canvas_pos.y + 20.0f + g_arranger_drag_hover_track * lane_h;
                            float gh_x1 = bar_to_x(c.start_bar);
                            float gh_x2 = bar_to_x(c.end_bar());

                            draw_list->AddRectFilled(ImVec2(gh_x1, gh_ly + 2.0f),
                                                     ImVec2(gh_x2, gh_ly + lane_h - 2.0f),
                                                     ImColor(59, 130, 246, 50), 3.0f);
                            draw_list->AddRect(ImVec2(gh_x1, gh_ly + 2.0f),
                                               ImVec2(gh_x2, gh_ly + lane_h - 2.0f),
                                               ImColor(59, 130, 246, 220), 3.0f, 0, 2.0f);

                            char gh_txt[64];
                            std::snprintf(gh_txt, sizeof(gh_txt), "MOVE TO TRACK %d (%s)",
                                          g_arranger_drag_hover_track + 1, track_names[g_arranger_drag_hover_track]);
                            draw_list->AddText(ImVec2(gh_x1 + 8.0f, gh_ly + 6.0f),
                                               ImColor(29, 78, 216, 255), gh_txt);
                        }

                        // Live Stretch Preview Badge during Edge Drag
                        if ((g_arranger_drag_mode == ArrangerDragMode::StretchStart || g_arranger_drag_mode == ArrangerDragMode::StretchEnd) &&
                            g_arranger_drag_track >= 0 && g_arranger_drag_clip_idx >= 0 &&
                            g_arranger_drag_clip_idx < static_cast<int>(track_arranger_clips[g_arranger_drag_track].size())) {
                            const auto& c = track_arranger_clips[g_arranger_drag_track][g_arranger_drag_clip_idx];
                            float base_l = (c.base_len_bars > 0.01f) ? c.base_len_bars : g_arranger_drag_orig_len_bars;
                            float ratio = c.len_bars / base_l;
                            float b_x = bar_to_x(c.start_bar) + 6.0f;
                            float b_y = canvas_pos.y + 20.0f + g_arranger_drag_track * lane_h + lane_h * 0.5f - 8.0f;
                            char badge_txt[64];
                            const char* badge_algo = (c.stretch_algo == dsp::PitchAlgorithm::TransientWarpWsola) ? "T-WARP" :
                                                     ((c.stretch_algo == dsp::PitchAlgorithm::RubberbandWsola) ? "WSOLA" :
                                                     ((c.stretch_algo == dsp::PitchAlgorithm::SovereignOde) ? "ODE" :
                                                     ((c.stretch_algo == dsp::PitchAlgorithm::VintageMpc) ? "MPC12" : "VINYL")));
                            std::snprintf(badge_txt, sizeof(badge_txt), "[STRETCH: %.2fx %s]", ratio, badge_algo);
                            draw_list->AddRectFilled(ImVec2(b_x - 3.0f, b_y - 2.0f), ImVec2(b_x + 140.0f, b_y + 16.0f), ImColor(245, 158, 11, 230), 3.0f);
                            draw_list->AddText(ImVec2(b_x, b_y), ImColor(0, 0, 0, 255), badge_txt);
                        }

                        // Live Warp Pin Preview Badge during Drag
                        if (g_arranger_drag_mode == ArrangerDragMode::WarpPinDrag &&
                            g_arranger_drag_track >= 0 && g_arranger_drag_clip_idx >= 0 &&
                            g_arranger_drag_clip_idx < static_cast<int>(track_arranger_clips[g_arranger_drag_track].size())) {
                            const auto& c = track_arranger_clips[g_arranger_drag_track][g_arranger_drag_clip_idx];
                            const auto* p = c.find_warp_pin(g_arranger_drag_warp_pin_id);
                            if (p) {
                                float px = bar_to_x(c.start_bar + p->pinned_bar);
                                float py = canvas_pos.y + 20.0f + g_arranger_drag_track * lane_h + lane_h * 0.5f - 8.0f;
                                char badge_txt[64];
                                std::snprintf(badge_txt, sizeof(badge_txt), "[WARP PIN: BAR %.2f]", c.start_bar + p->pinned_bar + 1.0f);
                                draw_list->AddRectFilled(ImVec2(px - 3.0f, py - 2.0f), ImVec2(px + 145.0f, py + 16.0f), ImColor(245, 158, 11, 235), 3.0f);
                                draw_list->AddText(ImVec2(px, py), ImColor(0, 0, 0, 255), badge_txt);
                            }
                        }

                        // Playhead Needle
                        float playhead_bar = playhead_seconds / get_seconds_per_bar(bpm);
                        float playhead_x = bar_to_x(playhead_bar);
                        if (playhead_x >= canvas_pos.x - 2.0f && playhead_x <= canvas_pos.x + canvas_size.x + 2.0f) {
                            draw_list->AddLine(ImVec2(playhead_x, canvas_pos.y),
                                               ImVec2(playhead_x, canvas_pos.y + canvas_size.y),
                                               ImColor(20, 25, 35, 255), 2.0f);
                            draw_list->AddTriangleFilled(ImVec2(playhead_x - 6.0f, canvas_pos.y),
                                                         ImVec2(playhead_x + 6.0f, canvas_pos.y),
                                                         ImVec2(playhead_x, canvas_pos.y + 10.0f),
                                                         ImColor(20, 25, 35, 255));
                        }

                        // Interactive Arranger Controls: Clip Drag, Trim, Fade, Slip-Edit & Scrubbing
                        ImGui::SetCursorScreenPos(canvas_pos);
                        ImGui::InvisibleButton("ArrangerCanvasInteractionBtn", canvas_size);
                        const bool is_hovered = ImGui::IsItemHovered();
                        const bool is_activated = ImGui::IsItemActivated();
                        const bool is_active = ImGui::IsItemActive();
                        const bool is_deactivated = ImGui::IsItemDeactivated();
                        ImVec2 m = ImGui::GetIO().MousePos;

                        // Zoom & Pan Interactions (Ctrl+Wheel, Shift+Wheel, Middle-Mouse Drag)
                        if (is_hovered) {
                            float wheel = ImGui::GetIO().MouseWheel;
                            if (ImGui::GetIO().KeyCtrl && std::abs(wheel) > 0.0f) {
                                float mouse_bar = x_to_bar(m.x);
                                float zoom_factor = (wheel > 0.0f) ? 0.85f : 1.176f;
                                float new_visible = std::clamp(g_arranger_visible_bars * zoom_factor, g_arranger_min_bars, g_arranger_max_bars);
                                float new_bar_w = canvas_size.x / new_visible;
                                float new_scroll = mouse_bar - (m.x - canvas_pos.x) / new_bar_w;
                                float max_scroll = std::max(0.0f, g_arranger_max_bars - new_visible);
                                g_arranger_visible_bars = new_visible;
                                g_arranger_scroll_bar = std::clamp(new_scroll, 0.0f, max_scroll);
                            } else if (ImGui::GetIO().KeyShift && std::abs(wheel) > 0.0f) {
                                float pan_bars = wheel * (g_arranger_visible_bars * 0.125f);
                                float max_scroll = std::max(0.0f, g_arranger_max_bars - g_arranger_visible_bars);
                                g_arranger_scroll_bar = std::clamp(g_arranger_scroll_bar - pan_bars, 0.0f, max_scroll);
                            }

                            if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
                                float dx = ImGui::GetIO().MouseDelta.x;
                                float d_bar = (dx / bar_w);
                                float max_scroll = std::max(0.0f, g_arranger_max_bars - g_arranger_visible_bars);
                                g_arranger_scroll_bar = std::clamp(g_arranger_scroll_bar - d_bar, 0.0f, max_scroll);
                            }
                        }

                        // Cursor styling based on hover location
                        if (is_hovered && g_arranger_drag_mode == ArrangerDragMode::None && g_ruler_drag_target == ArrangerRulerDragTarget::None) {
                            if (m.y < canvas_pos.y + 20.0f) {
                                float loop_x1 = bar_to_x(loop_start_bar);
                                float loop_x2 = bar_to_x(loop_start_bar + loop_bars);
                                float p_in = g_punch_link_to_loop ? loop_start_bar : g_punch_in_bar;
                                float p_out = g_punch_link_to_loop ? (loop_start_bar + loop_bars) : g_punch_out_bar;
                                float px1 = bar_to_x(p_in);
                                float px2 = bar_to_x(p_out);

                                if (g_punch_enabled && !g_punch_link_to_loop && std::abs(m.x - px1) <= 7.0f) {
                                    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
                                } else if (g_punch_enabled && !g_punch_link_to_loop && std::abs(m.x - px2) <= 7.0f) {
                                    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
                                } else if (std::abs(m.x - loop_x1) <= 7.0f) {
                                    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
                                } else if (std::abs(m.x - loop_x2) <= 7.0f) {
                                    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
                                } else if (m.x > loop_x1 + 7.0f && m.x < loop_x2 - 7.0f) {
                                    ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                                } else {
                                    ImGui::SetMouseCursor(ImGuiMouseCursor_Arrow);
                                }
                            } else {
                                int hov_t = static_cast<int>((m.y - (canvas_pos.y + 20.0f)) / lane_h);
                                if (hov_t >= 0 && hov_t < 4) {
                                    for (size_t i = 0; i < track_arranger_clips[hov_t].size(); ++i) {
                                        const auto& c = track_arranger_clips[hov_t][i];
                                        float hov_x1 = bar_to_x(c.start_bar);
                                        float hov_x2 = bar_to_x(c.end_bar());
                                        if (m.x >= hov_x1 && m.x <= hov_x2) {
                                            if (g_arranger_tool == ArrangerTool::Warp) {
                                                bool on_pin = false;
                                                for (const auto& pin : c.warp_pins) {
                                                    float px = bar_to_x(c.start_bar + pin.pinned_bar);
                                                    if (std::abs(m.x - px) < 8.0f) {
                                                        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
                                                        on_pin = true;
                                                        break;
                                                    }
                                                }
                                                if (!on_pin) {
                                                    uint32_t nf = c.clip ? c.clip->num_frames() : 0;
                                                    if (nf > 0) {
                                                        for (const auto& span : c.detected_transients) {
                                                            float rel_b = static_cast<float>(c.frame_to_bar(span.peak_frame, nf));
                                                            float tx = bar_to_x(c.start_bar + rel_b);
                                                            if (std::abs(m.x - tx) < 8.0f) {
                                                                ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                                                                on_pin = true;
                                                                break;
                                                            }
                                                        }
                                                    }
                                                }
                                                if (!on_pin) break;
                                            }

                                            if (g_arranger_tool == ArrangerTool::Razor) {
                                                ImGui::SetMouseCursor(ImGuiMouseCursor_TextInput);
                                                float rly = canvas_pos.y + 20.0f + hov_t * lane_h;
                                                float cut_bar = x_to_bar(m.x);
                                                if (!ImGui::GetIO().KeyShift) cut_bar = std::round(cut_bar * 4.0f) / 4.0f;
                                                float cut_x = bar_to_x(cut_bar);
                                                draw_list->AddLine(ImVec2(cut_x, rly + 1.0f), ImVec2(cut_x, rly + lane_h - 1.0f),
                                                                   ImColor(239, 68, 68, 240), 2.0f);
                                                char cut_lbl[32];
                                                std::snprintf(cut_lbl, sizeof(cut_lbl), "CUT %.2fb", cut_bar + 1.0f);
                                                draw_list->AddText(ImVec2(cut_x + 3.0f, rly + 3.0f), ImColor(220, 38, 38, 255), cut_lbl);
                                            } else {
                                                float fin_hx = hov_x1 + std::max(8.0f, c.fade_in_bars * bar_w);
                                                float fout_hx = hov_x2 - std::max(8.0f, c.fade_out_bars * bar_w);
                                                float top_y = canvas_pos.y + 20.0f + hov_t * lane_h;

                                                if (m.y <= top_y + 12.0f && std::abs(m.x - fin_hx) < 8.0f) {
                                                    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
                                                } else if (m.y <= top_y + 12.0f && std::abs(m.x - fout_hx) < 8.0f) {
                                                    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
                                                } else if (std::abs(m.x - hov_x1) < 8.0f || std::abs(m.x - hov_x2) < 8.0f) {
                                                    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
                                                } else if (ImGui::GetIO().KeyAlt || g_arranger_tool == ArrangerTool::Slip) {
                                                    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
                                                } else {
                                                    ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
                                                }
                                            }
                                            break;
                                        }
                                    }
                                }
                            }
                        }

                        if (is_activated) {
                            Track* all_trks_snap[4] = { trk0, trk1, trk2, trk3 };
                            for (int ti = 0; ti < 4; ++ti) {
                                if (all_trks_snap[ti]) {
                                    g_pre_drag_clips[ti] = all_trks_snap[ti]->arranger().clips();
                                } else {
                                    g_pre_drag_clips[ti] = track_arranger_clips[ti];
                                }
                            }

                            if (m.y < canvas_pos.y + 20.0f) {
                                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                                    loop_active = !loop_active;
                                    std::snprintf(status_toast, sizeof(status_toast), "LOOP %s (BARS %.1f-%.1f)",
                                                  loop_active ? "ENABLED" : "BYPASSED", loop_start_bar + 1.0f, loop_start_bar + loop_bars + 1.0f);
                                } else {
                                    g_ruler_drag_orig_start = loop_start_bar;
                                    g_ruler_drag_orig_len = loop_bars;
                                    g_ruler_drag_orig_punch_in = g_punch_in_bar;
                                    g_ruler_drag_orig_punch_out = g_punch_out_bar;
                                    g_arranger_drag_start_mouse_x = m.x;

                                    float loop_x1 = bar_to_x(loop_start_bar);
                                    float loop_x2 = bar_to_x(loop_start_bar + loop_bars);
                                    float p_in = g_punch_link_to_loop ? loop_start_bar : g_punch_in_bar;
                                    float p_out = g_punch_link_to_loop ? (loop_start_bar + loop_bars) : g_punch_out_bar;
                                    float px1 = bar_to_x(p_in);
                                    float px2 = bar_to_x(p_out);

                                    if (g_punch_enabled && !g_punch_link_to_loop && std::abs(m.x - px1) <= 7.0f) {
                                        g_ruler_drag_target = ArrangerRulerDragTarget::PunchIn;
                                    } else if (g_punch_enabled && !g_punch_link_to_loop && std::abs(m.x - px2) <= 7.0f) {
                                        g_ruler_drag_target = ArrangerRulerDragTarget::PunchOut;
                                    } else if (std::abs(m.x - loop_x1) <= 7.0f) {
                                        g_ruler_drag_target = ArrangerRulerDragTarget::LoopLeft;
                                    } else if (std::abs(m.x - loop_x2) <= 7.0f) {
                                        g_ruler_drag_target = ArrangerRulerDragTarget::LoopRight;
                                    } else if (m.x > loop_x1 + 7.0f && m.x < loop_x2 - 7.0f) {
                                        g_ruler_drag_target = ArrangerRulerDragTarget::LoopBody;
                                    } else {
                                        g_ruler_drag_target = ArrangerRulerDragTarget::Scrub;
                                        g_arranger_drag_mode = ArrangerDragMode::ScrubTimeline;
                                        float scrub_bar = std::clamp(x_to_bar(m.x), 0.0f, g_arranger_max_bars);
                                        playhead_seconds = scrub_bar * get_seconds_per_bar(bpm);
                                        uint64_t target_sample = static_cast<uint64_t>(playhead_seconds * kSampleRate);
                                        mixer.start_scrub(target_sample);
                                    }
                                }
                            } else {
                                int clicked_lane = static_cast<int>((m.y - (canvas_pos.y + 20.0f)) / lane_h);
                                clicked_lane = std::clamp(clicked_lane, 0, 3);
                                selected_track = clicked_lane;

                                // Find which clip in this lane was clicked
                                int clicked_clip_idx = -1;
                                for (size_t i = 0; i < track_arranger_clips[clicked_lane].size(); ++i) {
                                    float cx1 = bar_to_x(track_arranger_clips[clicked_lane][i].start_bar);
                                    float cx2 = bar_to_x(track_arranger_clips[clicked_lane][i].end_bar());
                                    if (m.x >= cx1 && m.x <= cx2) {
                                        clicked_clip_idx = static_cast<int>(i);
                                        break;
                                    }
                                }

                                if (clicked_clip_idx >= 0) {
                                    g_selected_clip_idx = clicked_clip_idx;
                                    auto& c = track_arranger_clips[clicked_lane][clicked_clip_idx];
                                    if (c.clip) {
                                        sync_track_clip(selected_track, c.clip);
                                    }

                                    if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                                        if (c.type == sequencer::ArrangerClipType::Pattern) {
                                            selected_track = clicked_lane;
                                            pattern_editor_pat_idx = static_cast<int>(c.pattern_index);
                                            g_switch_to_main_tab = 2; // Jump to Tab 3 (PATTERN SEQUENCER // STEP GRID & TRACKER)
                                            std::string p_name = (track_seq[clicked_lane]) ? track_seq[clicked_lane]->pattern(c.pattern_index).name : "Pattern";
                                            std::snprintf(status_toast, sizeof(status_toast), "OPENED PATTERN %d '%s' IN TRACKER EDITOR",
                                                          c.pattern_index + 1, p_name.c_str());
                                        }
                                    } else if (g_arranger_tool == ArrangerTool::Razor) {
                                        // Razor split at mouse position
                                        float click_bar = x_to_bar(m.x);
                                        if (!ImGui::GetIO().KeyShift) {
                                            click_bar = std::round(click_bar * 4.0f) / 4.0f; // 1-beat snap
                                        }
                                        if (click_bar > c.start_bar + 0.05f && click_bar < c.end_bar() - 0.05f) {
                                            Track* trk = (clicked_lane == 0) ? trk0 : ((clicked_lane == 1) ? trk1 : ((clicked_lane == 2) ? trk2 : trk3));
                                            if (trk) {
                                                auto before_clips = trk->arranger().clips();
                                                trk->arranger().split_clip_at_bar(c.id, click_bar, bpm, kSampleRate);
                                                track_arranger_clips[clicked_lane] = trk->arranger().clips();
                                                g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
                                                    trk, before_clips, track_arranger_clips[clicked_lane], "Razor Split '" + c.name + "'"
                                                ));
                                                std::snprintf(status_toast, sizeof(status_toast), "RAZOR SPLIT '%s' AT BAR %.2f", c.name.c_str(), click_bar + 1.0f);
                                            }
                                        } else {
                                            std::snprintf(status_toast, sizeof(status_toast), "CANNOT SPLIT AT BAR %.2f (TOO CLOSE TO CLIP BOUNDARY)", click_bar + 1.0f);
                                        }
                                    } else {
                                        // Select / Move / Trim / Fade / Slip
                                        g_arranger_drag_track = clicked_lane;
                                        g_arranger_drag_clip_idx = clicked_clip_idx;
                                        g_arranger_drag_hover_track = clicked_lane;
                                        g_arranger_drag_start_mouse_x = m.x;
                                        g_arranger_drag_orig_start_bar = c.start_bar;
                                        g_arranger_drag_orig_len_bars = c.len_bars;
                                        g_arranger_drag_orig_offset_bars = c.offset_bars;
                                        g_arranger_drag_orig_fade_in = c.fade_in_bars;
                                        g_arranger_drag_orig_fade_out = c.fade_out_bars;

                                        float cx1 = bar_to_x(c.start_bar);
                                        float cx2 = bar_to_x(c.end_bar());
                                        float top_y = canvas_pos.y + 20.0f + clicked_lane * lane_h;
                                        float fin_hx = cx1 + std::max(8.0f, c.fade_in_bars * bar_w);
                                        float fout_hx = cx2 - std::max(8.0f, c.fade_out_bars * bar_w);

                                        // Right-click on fade handle cycles fade curvature shape (0..4)
                                        bool handled_shape_click = false;
                                        if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
                                            Track* trk = (clicked_lane == 0) ? trk0 : ((clicked_lane == 1) ? trk1 : ((clicked_lane == 2) ? trk2 : trk3));
                                            if (m.y <= top_y + 14.0f && std::abs(m.x - fin_hx) < 10.0f) {
                                                auto before_clips = trk ? trk->arranger().clips() : track_arranger_clips[clicked_lane];
                                                c.fade_in_shape = static_cast<sampling::FadeShape>((static_cast<int>(c.fade_in_shape) + 1) % 5);
                                                sync_arranger_to_mixer(clicked_lane);
                                                if (trk) {
                                                    g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
                                                        trk, before_clips, trk->arranger().clips(), "Change Fade-In Shape"
                                                    ));
                                                }
                                                std::snprintf(status_toast, sizeof(status_toast), "CLIP '%s' FADE-IN SHAPE: %d", c.name.c_str(), static_cast<int>(c.fade_in_shape));
                                                handled_shape_click = true;
                                            } else if (m.y <= top_y + 14.0f && std::abs(m.x - fout_hx) < 10.0f) {
                                                auto before_clips = trk ? trk->arranger().clips() : track_arranger_clips[clicked_lane];
                                                c.fade_out_shape = static_cast<sampling::FadeShape>((static_cast<int>(c.fade_out_shape) + 1) % 5);
                                                sync_arranger_to_mixer(clicked_lane);
                                                if (trk) {
                                                    g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
                                                        trk, before_clips, trk->arranger().clips(), "Change Fade-Out Shape"
                                                    ));
                                                }
                                                std::snprintf(status_toast, sizeof(status_toast), "CLIP '%s' FADE-OUT SHAPE: %d", c.name.c_str(), static_cast<int>(c.fade_out_shape));
                                                handled_shape_click = true;
                                            } else {
                                                ImGui::OpenPopup("ArrangerClipContextMenu");
                                                handled_shape_click = true;
                                            }
                                        }

                                        if (!handled_shape_click) {
                                            bool handled_warp_click = false;
                                            Track* trk = (clicked_lane == 0) ? trk0 : ((clicked_lane == 1) ? trk1 : ((clicked_lane == 2) ? trk2 : trk3));

                                            // 1. Existing Warp Pins
                                            for (const auto& pin : c.warp_pins) {
                                                float px = bar_to_x(c.start_bar + pin.pinned_bar);
                                                if (std::abs(m.x - px) < 8.0f) {
                                                    if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                                                        if (trk) {
                                                            auto before_clips = trk->arranger().clips();
                                                            trk->arranger().remove_clip_warp_pin(c.id, pin.id);
                                                            track_arranger_clips[clicked_lane] = trk->arranger().clips();
                                                            g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
                                                                trk, before_clips, track_arranger_clips[clicked_lane], "Remove Warp Pin"
                                                            ));
                                                            std::snprintf(status_toast, sizeof(status_toast), "REMOVED WARP PIN #%u", pin.id);
                                                        }
                                                    } else {
                                                        g_arranger_drag_mode = ArrangerDragMode::WarpPinDrag;
                                                        g_arranger_drag_track = clicked_lane;
                                                        g_arranger_drag_clip_idx = clicked_clip_idx;
                                                        g_arranger_drag_warp_pin_id = pin.id;
                                                        g_arranger_drag_orig_pinned_bar = pin.pinned_bar;
                                                        g_arranger_drag_start_mouse_x = m.x;
                                                        std::snprintf(status_toast, sizeof(status_toast), "DRAGGING WARP PIN #%u (HOLD SHIFT TO FREE FLOAT)", pin.id);
                                                    }
                                                    handled_warp_click = true;
                                                    break;
                                                }
                                            }

                                            // 2. Unpinned Transients (if Warp tool active or double-clicked)
                                            if (!handled_warp_click && (g_arranger_tool == ArrangerTool::Warp || ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))) {
                                                uint32_t num_f = c.clip ? c.clip->num_frames() : 0;
                                                if (num_f > 0) {
                                                    for (const auto& span : c.detected_transients) {
                                                        float rel_b = static_cast<float>(c.frame_to_bar(span.peak_frame, num_f));
                                                        float tx = bar_to_x(c.start_bar + rel_b);
                                                        if (std::abs(m.x - tx) < 8.0f) {
                                                            if (trk) {
                                                                auto before_clips = trk->arranger().clips();
                                                                uint32_t new_pid = trk->arranger().add_clip_warp_pin(c.id, span.peak_frame, rel_b);
                                                                track_arranger_clips[clicked_lane] = trk->arranger().clips();
                                                                g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
                                                                    trk, before_clips, track_arranger_clips[clicked_lane], "Add Warp Pin"
                                                                ));
                                                                g_arranger_drag_mode = ArrangerDragMode::WarpPinDrag;
                                                                g_arranger_drag_track = clicked_lane;
                                                                g_arranger_drag_clip_idx = clicked_clip_idx;
                                                                g_arranger_drag_warp_pin_id = new_pid;
                                                                g_arranger_drag_orig_pinned_bar = rel_b;
                                                                g_arranger_drag_start_mouse_x = m.x;
                                                                std::snprintf(status_toast, sizeof(status_toast), "PINNED TRANSIENT TO WARP PIN #%u (DRAG TO WARP)", new_pid);
                                                            }
                                                            handled_warp_click = true;
                                                            break;
                                                        }
                                                    }
                                                }
                                            }

                                            if (!handled_warp_click) {
                                                if (m.y <= top_y + 14.0f && std::abs(m.x - fin_hx) < 10.0f) {
                                                    g_arranger_drag_mode = ArrangerDragMode::FadeIn;
                                                } else if (m.y <= top_y + 14.0f && std::abs(m.x - fout_hx) < 10.0f) {
                                                    g_arranger_drag_mode = ArrangerDragMode::FadeOut;
                                                } else if (std::abs(m.x - cx1) < 8.0f) {
                                                    if (g_arranger_tool == ArrangerTool::Stretch || (g_arranger_tool == ArrangerTool::Select && ImGui::GetIO().KeyShift)) {
                                                        g_arranger_drag_mode = ArrangerDragMode::StretchStart;
                                                    } else {
                                                        g_arranger_drag_mode = ArrangerDragMode::TrimStart;
                                                    }
                                                } else if (std::abs(m.x - cx2) < 8.0f) {
                                                    if (g_arranger_tool == ArrangerTool::Stretch || (g_arranger_tool == ArrangerTool::Select && ImGui::GetIO().KeyShift)) {
                                                        g_arranger_drag_mode = ArrangerDragMode::StretchEnd;
                                                    } else {
                                                        g_arranger_drag_mode = ArrangerDragMode::TrimEnd;
                                                    }
                                                } else if (ImGui::GetIO().KeyAlt || g_arranger_tool == ArrangerTool::Slip) {
                                                    g_arranger_drag_mode = ArrangerDragMode::SlipEdit;
                                                } else {
                                                    g_arranger_drag_mode = ArrangerDragMode::MoveClip;
                                                }
                                            }
                                        }
                                    }
                                } else {
                                    // Clicked outside clip on lane -> Seek playhead
                                    g_arranger_drag_mode = ArrangerDragMode::ScrubTimeline;
                                    float scrub_bar = std::clamp(x_to_bar(m.x), 0.0f, g_arranger_max_bars);
                                playhead_seconds = scrub_bar * get_seconds_per_bar(bpm);
                                    uint64_t target_sample = static_cast<uint64_t>(playhead_seconds * kSampleRate);
                                    mixer.start_scrub(target_sample);
                                }
                            }
                        } else if (is_active) {
                            if (g_ruler_drag_target != ArrangerRulerDragTarget::None) {
                                float delta_bars = (m.x - g_arranger_drag_start_mouse_x) / bar_w;
                                if (g_ruler_drag_target == ArrangerRulerDragTarget::LoopLeft) {
                                    float new_start = g_ruler_drag_orig_start + delta_bars;
                                    if (!ImGui::GetIO().KeyShift) new_start = std::round(new_start * 4.0f) / 4.0f;
                                    float end_b = g_ruler_drag_orig_start + g_ruler_drag_orig_len;
                                    new_start = std::clamp(new_start, 0.0f, end_b - 0.25f);
                                    loop_start_bar = new_start;
                                    loop_bars = end_b - new_start;
                                    if (g_punch_link_to_loop) {
                                        g_punch_in_bar = loop_start_bar;
                                        g_punch_out_bar = loop_start_bar + loop_bars;
                                    }
                                } else if (g_ruler_drag_target == ArrangerRulerDragTarget::LoopRight) {
                                    float new_len = g_ruler_drag_orig_len + delta_bars;
                                    if (!ImGui::GetIO().KeyShift) new_len = std::round(new_len * 4.0f) / 4.0f;
                                    loop_bars = std::clamp(new_len, 0.25f, g_arranger_max_bars - loop_start_bar);
                                    if (g_punch_link_to_loop) {
                                        g_punch_in_bar = loop_start_bar;
                                        g_punch_out_bar = loop_start_bar + loop_bars;
                                    }
                                } else if (g_ruler_drag_target == ArrangerRulerDragTarget::LoopBody) {
                                    float new_start = g_ruler_drag_orig_start + delta_bars;
                                    if (!ImGui::GetIO().KeyShift) new_start = std::round(new_start * 4.0f) / 4.0f;
                                    loop_start_bar = std::clamp(new_start, 0.0f, g_arranger_max_bars - loop_bars);
                                    if (g_punch_link_to_loop) {
                                        g_punch_in_bar = loop_start_bar;
                                        g_punch_out_bar = loop_start_bar + loop_bars;
                                    }
                                } else if (g_ruler_drag_target == ArrangerRulerDragTarget::PunchIn) {
                                    float new_in = g_ruler_drag_orig_punch_in + delta_bars;
                                    if (!ImGui::GetIO().KeyShift) new_in = std::round(new_in * 4.0f) / 4.0f;
                                    g_punch_in_bar = std::clamp(new_in, 0.0f, g_punch_out_bar - 0.25f);
                                } else if (g_ruler_drag_target == ArrangerRulerDragTarget::PunchOut) {
                                    float new_out = g_ruler_drag_orig_punch_out + delta_bars;
                                    if (!ImGui::GetIO().KeyShift) new_out = std::round(new_out * 4.0f) / 4.0f;
                                    g_punch_out_bar = std::clamp(new_out, g_punch_in_bar + 0.25f, g_arranger_max_bars);
                                } else if (g_ruler_drag_target == ArrangerRulerDragTarget::Scrub) {
                                    float scrub_bar = std::clamp(x_to_bar(m.x), 0.0f, g_arranger_max_bars);
                                    float prev_sec = playhead_seconds;
                                    playhead_seconds = scrub_bar * get_seconds_per_bar(bpm);
                                    float vel = (dt > 1e-4f) ? ((playhead_seconds - prev_sec) / dt) : 1.0f;
                                    vel = std::clamp(vel, -4.0f, 4.0f);
                                    uint64_t target_sample = static_cast<uint64_t>(playhead_seconds * kSampleRate);
                                    mixer.update_scrub(target_sample, static_cast<double>(vel));
                                }
                            } else if (g_arranger_drag_track >= 0 && g_arranger_drag_clip_idx >= 0 &&
                                       g_arranger_drag_clip_idx < static_cast<int>(track_arranger_clips[g_arranger_drag_track].size())) {
                                auto& c = track_arranger_clips[g_arranger_drag_track][g_arranger_drag_clip_idx];

                                if (g_arranger_drag_mode == ArrangerDragMode::MoveClip) {
                                    float delta_bars = (m.x - g_arranger_drag_start_mouse_x) / bar_w;
                                    float new_start = g_arranger_drag_orig_start_bar + delta_bars;
                                    if (!ImGui::GetIO().KeyShift) {
                                        new_start = std::round(new_start * 4.0f) / 4.0f; // 1 beat snap (1/4 bar)
                                    }
                                    float max_start = g_arranger_max_bars - c.len_bars;
                                    new_start = std::clamp(new_start, 0.0f, max_start);
                                    c.start_bar = new_start;
                                    sync_arranger_to_mixer(g_arranger_drag_track);

                                    // Update hover track lane for cross-track dragging
                                    int cur_lane = static_cast<int>((m.y - (canvas_pos.y + 20.0f)) / lane_h);
                                    g_arranger_drag_hover_track = std::clamp(cur_lane, 0, 3);
                                } else if (g_arranger_drag_mode == ArrangerDragMode::SlipEdit) {
                                    float delta_bars = (m.x - g_arranger_drag_start_mouse_x) / bar_w;
                                    float new_offset = std::max(0.0f, g_arranger_drag_orig_offset_bars - delta_bars);
                                    if (!ImGui::GetIO().KeyShift) {
                                        new_offset = std::round(new_offset * 16.0f) / 16.0f; // 1/16th bar snap
                                    }
                                    c.offset_bars = new_offset;
                                    double clip_bpm = (c.clip && c.clip->bpm() > 10.0) ? c.clip->bpm() : 120.0;
                                    uint32_t clip_sr = c.clip ? c.clip->sample_rate() : 48000;
                                    double dur_sec = (new_offset * 4.0 / clip_bpm) * 60.0;
                                    c.start_offset_frames = static_cast<uint32_t>(dur_sec * static_cast<double>(clip_sr));
                                    if (c.clip && c.clip->num_frames() > 0) {
                                        c.start_offset_frames %= c.clip->num_frames();
                                    }
                                    sync_arranger_to_mixer(g_arranger_drag_track);
                                } else if (g_arranger_drag_mode == ArrangerDragMode::FadeIn) {
                                    float delta_bars = (m.x - g_arranger_drag_start_mouse_x) / bar_w;
                                    float new_fade_in = std::clamp(g_arranger_drag_orig_fade_in + delta_bars, 0.0f,
                                                                   c.len_bars - c.fade_out_bars);
                                    if (!ImGui::GetIO().KeyShift) {
                                        new_fade_in = std::round(new_fade_in * 16.0f) / 16.0f;
                                    }
                                    c.fade_in_bars = new_fade_in;
                                    sync_arranger_to_mixer(g_arranger_drag_track);
                                } else if (g_arranger_drag_mode == ArrangerDragMode::FadeOut) {
                                    float delta_bars = (m.x - g_arranger_drag_start_mouse_x) / bar_w;
                                    float new_fade_out = std::clamp(g_arranger_drag_orig_fade_out - delta_bars, 0.0f,
                                                                    c.len_bars - c.fade_in_bars);
                                    if (!ImGui::GetIO().KeyShift) {
                                        new_fade_out = std::round(new_fade_out * 16.0f) / 16.0f;
                                    }
                                    c.fade_out_bars = new_fade_out;
                                    sync_arranger_to_mixer(g_arranger_drag_track);
                                } else if (g_arranger_drag_mode == ArrangerDragMode::TrimStart) {
                                    float delta_bars = (m.x - g_arranger_drag_start_mouse_x) / bar_w;
                                    float new_start = g_arranger_drag_orig_start_bar + delta_bars;
                                    if (!ImGui::GetIO().KeyShift) {
                                        new_start = std::round(new_start * 4.0f) / 4.0f;
                                    }
                                    float clip_end_bar = g_arranger_drag_orig_start_bar + g_arranger_drag_orig_len_bars;
                                    new_start = std::clamp(new_start, 0.0f, clip_end_bar - 0.25f);
                                    c.start_bar = new_start;
                                    c.len_bars = clip_end_bar - new_start;
                                    sync_arranger_to_mixer(g_arranger_drag_track);
                                } else if (g_arranger_drag_mode == ArrangerDragMode::TrimEnd) {
                                    float delta_bars = (m.x - g_arranger_drag_start_mouse_x) / bar_w;
                                    float new_len = g_arranger_drag_orig_len_bars + delta_bars;
                                    if (!ImGui::GetIO().KeyShift) {
                                        new_len = std::round(new_len * 4.0f) / 4.0f;
                                    }
                                    float max_len = g_arranger_max_bars - c.start_bar;
                                    new_len = std::clamp(new_len, 0.25f, max_len);
                                    c.len_bars = new_len;
                                    sync_arranger_to_mixer(g_arranger_drag_track);
                                } else if (g_arranger_drag_mode == ArrangerDragMode::StretchStart) {
                                    float delta_bars = (m.x - g_arranger_drag_start_mouse_x) / bar_w;
                                    float new_start = g_arranger_drag_orig_start_bar + delta_bars;
                                    if (!ImGui::GetIO().KeyShift) {
                                        new_start = std::round(new_start * 4.0f) / 4.0f;
                                    }
                                    float clip_end_bar = g_arranger_drag_orig_start_bar + g_arranger_drag_orig_len_bars;
                                    new_start = std::clamp(new_start, 0.0f, clip_end_bar - 0.25f);
                                    c.start_bar = new_start;
                                    c.len_bars = clip_end_bar - new_start;
                                } else if (g_arranger_drag_mode == ArrangerDragMode::StretchEnd) {
                                    float delta_bars = (m.x - g_arranger_drag_start_mouse_x) / bar_w;
                                    float new_len = g_arranger_drag_orig_len_bars + delta_bars;
                                    if (!ImGui::GetIO().KeyShift) {
                                        new_len = std::round(new_len * 4.0f) / 4.0f;
                                    }
                                    float max_len = g_arranger_max_bars - c.start_bar;
                                    new_len = std::clamp(new_len, 0.25f, max_len);
                                    c.len_bars = new_len;
                                } else if (g_arranger_drag_mode == ArrangerDragMode::WarpPinDrag) {
                                    float delta_bars = (m.x - g_arranger_drag_start_mouse_x) / bar_w;
                                    float new_bar = g_arranger_drag_orig_pinned_bar + delta_bars;
                                    if (!ImGui::GetIO().KeyShift) {
                                        new_bar = std::round(new_bar * 16.0f) / 16.0f; // 1/16th bar snap
                                    }
                                    Track* trk = (g_arranger_drag_track == 0) ? trk0 : ((g_arranger_drag_track == 1) ? trk1 : ((g_arranger_drag_track == 2) ? trk2 : trk3));
                                    if (trk) {
                                        trk->arranger().move_clip_warp_pin(c.id, g_arranger_drag_warp_pin_id, new_bar);
                                        track_arranger_clips[g_arranger_drag_track] = trk->arranger().clips();
                                    }
                                }
                            } else if (g_arranger_drag_mode == ArrangerDragMode::ScrubTimeline) {
                                float scrub_bar = std::clamp(x_to_bar(m.x), 0.0f, g_arranger_max_bars);
                                float prev_sec = playhead_seconds;
                                playhead_seconds = scrub_bar * get_seconds_per_bar(bpm);
                                float vel = (dt > 1e-4f) ? ((playhead_seconds - prev_sec) / dt) : 1.0f;
                                vel = std::clamp(vel, -4.0f, 4.0f);
                                uint64_t target_sample = static_cast<uint64_t>(playhead_seconds * kSampleRate);
                                mixer.update_scrub(target_sample, static_cast<double>(vel));
                            }
                        } else if (is_deactivated) {
                            g_ruler_drag_target = ArrangerRulerDragTarget::None;
                            ArrangerDragMode finished_mode = g_arranger_drag_mode;

                            if (g_arranger_drag_mode == ArrangerDragMode::WarpPinDrag) {
                                g_arranger_drag_mode = ArrangerDragMode::None;
                                g_arranger_drag_warp_pin_id = 0;
                            } else if (g_arranger_drag_mode == ArrangerDragMode::ScrubTimeline) {
                                float scrub_bar = std::clamp(x_to_bar(m.x), 0.0f, g_arranger_max_bars);
                                playhead_seconds = scrub_bar * get_seconds_per_bar(bpm);
                                uint64_t target_sample = static_cast<uint64_t>(playhead_seconds * kSampleRate);
                                mixer.end_scrub(target_sample);
                                if (is_playing) mixer.seek(target_sample);
                            } else if (g_arranger_drag_mode == ArrangerDragMode::MoveClip && g_arranger_drag_track >= 0) {
                                int src_t = g_arranger_drag_track;
                                int dst_t = g_arranger_drag_hover_track;
                                if (dst_t >= 0 && dst_t != src_t && g_arranger_drag_clip_idx >= 0 &&
                                    g_arranger_drag_clip_idx < static_cast<int>(track_arranger_clips[src_t].size())) {
                                    // Move clip to new track
                                    auto clip_to_move = track_arranger_clips[src_t][g_arranger_drag_clip_idx];
                                    track_arranger_clips[src_t].erase(track_arranger_clips[src_t].begin() + g_arranger_drag_clip_idx);
                                    track_arranger_clips[dst_t].push_back(clip_to_move);
                                    sync_arranger_to_mixer(src_t);
                                    sync_arranger_to_mixer(dst_t);
                                    Track* dst_trk = (dst_t == 0) ? trk0 : ((dst_t == 1) ? trk1 : ((dst_t == 2) ? trk2 : trk3));
                                    if (dst_trk) {
                                        track_arranger_clips[dst_t] = dst_trk->arranger().clips();
                                    }
                                    selected_track = dst_t;
                                    g_selected_clip_idx = static_cast<int>(track_arranger_clips[dst_t].size() - 1);
                                    std::snprintf(status_toast, sizeof(status_toast), "MOVED CLIP '%s' TO TRACK %d", clip_to_move.name.c_str(), dst_t + 1);
                                } else {
                                    sync_arranger_to_mixer(src_t);
                                    Track* src_trk = (src_t == 0) ? trk0 : ((src_t == 1) ? trk1 : ((src_t == 2) ? trk2 : trk3));
                                    if (src_trk) {
                                        track_arranger_clips[src_t] = src_trk->arranger().clips();
                                    }
                                }
                            } else if ((g_arranger_drag_mode == ArrangerDragMode::StretchStart || g_arranger_drag_mode == ArrangerDragMode::StretchEnd) && g_arranger_drag_track >= 0) {
                                int src_t = g_arranger_drag_track;
                                if (src_t >= 0 && g_arranger_drag_clip_idx >= 0 &&
                                    g_arranger_drag_clip_idx < static_cast<int>(track_arranger_clips[src_t].size())) {
                                    Track* trk = (src_t == 0) ? trk0 : ((src_t == 1) ? trk1 : ((src_t == 2) ? trk2 : trk3));
                                    if (trk) {
                                        auto& c = track_arranger_clips[src_t][g_arranger_drag_clip_idx];
                                        trk->arranger().stretch_clip(c.id, c.len_bars, c.stretch_algo, c.pitch_semitones);
                                        track_arranger_clips[src_t] = trk->arranger().clips();
                                        const auto* sc = trk->arranger().find_clip(c.id);
                                        if (sc) {
                                            std::snprintf(status_toast, sizeof(status_toast), "STRETCHED '%s' TO %.2fb (%.2fx WSOLA)",
                                                          sc->name.c_str(), sc->len_bars, sc->stretch_ratio);
                                        }
                                    }
                                }
                            }

                            if (finished_mode != ArrangerDragMode::None && finished_mode != ArrangerDragMode::ScrubTimeline) {
                                Track* all_trks_post[4] = { trk0, trk1, trk2, trk3 };
                                std::vector<undo::ArrangerMultiTrackAction::TrackState> diff_states;
                                for (int ti = 0; ti < 4; ++ti) {
                                    if (all_trks_post[ti]) {
                                        auto current_clips = all_trks_post[ti]->arranger().clips();
                                        if (!(current_clips == g_pre_drag_clips[ti])) {
                                            diff_states.push_back({
                                                .track = all_trks_post[ti],
                                                .old_clips = g_pre_drag_clips[ti],
                                                .new_clips = current_clips
                                            });
                                        }
                                    }
                                }
                                if (!diff_states.empty()) {
                                    const char* d_name = "Arranger Edit";
                                    if (finished_mode == ArrangerDragMode::MoveClip) d_name = "Move Clip";
                                    else if (finished_mode == ArrangerDragMode::TrimStart || finished_mode == ArrangerDragMode::TrimEnd) d_name = "Trim Clip";
                                    else if (finished_mode == ArrangerDragMode::FadeIn || finished_mode == ArrangerDragMode::FadeOut) d_name = "Fade Clip";
                                    else if (finished_mode == ArrangerDragMode::SlipEdit) d_name = "Slip Edit Clip";
                                    else if (finished_mode == ArrangerDragMode::StretchStart || finished_mode == ArrangerDragMode::StretchEnd) d_name = "Stretch Clip";
                                    else if (finished_mode == ArrangerDragMode::WarpPinDrag) d_name = "Warp Pin Edit";

                                    g_undo_mgr.push_action(std::make_unique<undo::ArrangerMultiTrackAction>(
                                        std::move(diff_states), d_name
                                    ));
                                }
                            }

                            g_arranger_drag_mode = ArrangerDragMode::None;
                            g_arranger_drag_track = -1;
                            g_arranger_drag_clip_idx = -1;
                            g_arranger_drag_hover_track = -1;
                        }

                        // Right-Click Context Menu for Selected Arranger Clip
                        if (ImGui::BeginPopup("ArrangerClipContextMenu")) {
                            int t = selected_track;
                            if (t >= 0 && t < 4 && !track_arranger_clips[t].empty()) {
                                int c_idx = std::clamp(g_selected_clip_idx, 0, static_cast<int>(track_arranger_clips[t].size() - 1));
                                auto& c = track_arranger_clips[t][c_idx];
                                Track* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));

                                ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "CLIP: %s (%.1f-%.1fb)", c.name.c_str(), c.start_bar + 1.0f, c.end_bar() + 1.0f);
                                ImGui::Separator();

                                ImGui::TextDisabled("STRETCH ALGORITHM:");
                                auto algo_item = [&](dsp::PitchAlgorithm algo, const char* label) {
                                    bool is_cur = (c.stretch_algo == algo);
                                    if (ImGui::MenuItem(label, nullptr, is_cur)) {
                                        if (trk) {
                                            auto before_clips = trk->arranger().clips();
                                            trk->arranger().set_clip_stretch_algo(c.id, algo);
                                            track_arranger_clips[t] = trk->arranger().clips();
                                            g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
                                                trk, before_clips, track_arranger_clips[t], std::string("Set Stretch Algo: ") + label
                                            ));
                                            std::snprintf(status_toast, sizeof(status_toast), "CLIP '%s' ALGO: %s", c.name.c_str(), label);
                                        }
                                    }
                                };

                                algo_item(dsp::PitchAlgorithm::TransientWarpWsola, "Transient-Warp WSOLA (Punch Preserved)");
                                algo_item(dsp::PitchAlgorithm::RubberbandWsola,    "Granular WSOLA (Rubberband)");
                                algo_item(dsp::PitchAlgorithm::SovereignOde,       "Sovereign Kinetic ODE (Continuous)");
                                algo_item(dsp::PitchAlgorithm::VinylRepitch,       "Vinyl / Tape Repitch (Variclock)");
                                algo_item(dsp::PitchAlgorithm::VintageMpc,         "Vintage MPC 12-Bit (Multiplying DAC)");
                                algo_item(dsp::PitchAlgorithm::DeRezSampler,       "Airwindows DeRez2 (SP-1200 / Mirage)");

                                ImGui::Separator();
                                ImGui::TextDisabled("DECOUPLED PITCH SHIFT:");
                                auto pitch_item = [&](float st, const char* label) {
                                    if (ImGui::MenuItem(label)) {
                                        if (trk) {
                                            auto before_clips = trk->arranger().clips();
                                            trk->arranger().set_clip_pitch(c.id, st);
                                            track_arranger_clips[t] = trk->arranger().clips();
                                            g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
                                                trk, before_clips, track_arranger_clips[t], std::string("Pitch Shift: ") + label
                                            ));
                                            std::snprintf(status_toast, sizeof(status_toast), "CLIP '%s' PITCH: %+.1f st", c.name.c_str(), st);
                                        }
                                    }
                                };

                                pitch_item(c.pitch_semitones - 12.0f, "-12 st (Octave Down)");
                                pitch_item(c.pitch_semitones - 1.0f,  "-1 st (Semitone Down)");
                                pitch_item(0.0f,                      "Reset Pitch (0 st)");
                                pitch_item(c.pitch_semitones + 1.0f,  "+1 st (Semitone Up)");
                                pitch_item(c.pitch_semitones + 12.0f, "+12 st (Octave Up)");

                                ImGui::Separator();
                                if (ImGui::MenuItem("Reset Length (1.00x)")) {
                                    if (trk) {
                                        auto before_clips = trk->arranger().clips();
                                        trk->arranger().stretch_clip(c.id, c.base_len_bars, c.stretch_algo, c.pitch_semitones);
                                        track_arranger_clips[t] = trk->arranger().clips();
                                        g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
                                            trk, before_clips, track_arranger_clips[t], "Reset Clip Length"
                                        ));
                                        std::snprintf(status_toast, sizeof(status_toast), "RESET STRETCH FOR '%s'", c.name.c_str());
                                    }
                                }

                                ImGui::Separator();
                                ImGui::TextDisabled("ELASTIC AUDIO / WARP PINS:");
                                if (ImGui::MenuItem("Quantize Transients (16th Grid)")) {
                                    if (trk) {
                                        auto before_clips = trk->arranger().clips();
                                        trk->arranger().quantize_clip_transients(c.id, 0.0625f);
                                        track_arranger_clips[t] = trk->arranger().clips();
                                        g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
                                            trk, before_clips, track_arranger_clips[t], "Quantize Transients (16th Grid)"
                                        ));
                                        std::snprintf(status_toast, sizeof(status_toast), "QUANTIZED TRANSIENTS (16TH) ON '%s'", c.name.c_str());
                                    }
                                }
                                if (ImGui::MenuItem("Re-Detect Multiband Transients")) {
                                    if (trk) {
                                        auto before_clips = trk->arranger().clips();
                                        trk->arranger().detect_clip_transients(c.id, 0.5f);
                                        track_arranger_clips[t] = trk->arranger().clips();
                                        g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
                                            trk, before_clips, track_arranger_clips[t], "Re-Detect Transients"
                                        ));
                                        std::snprintf(status_toast, sizeof(status_toast), "DETECTED TRANSIENTS ON '%s'", c.name.c_str());
                                    }
                                }
                                if (ImGui::MenuItem("Clear All Warp Pins")) {
                                    if (trk) {
                                        auto before_clips = trk->arranger().clips();
                                        trk->arranger().clear_clip_warp_pins(c.id);
                                        track_arranger_clips[t] = trk->arranger().clips();
                                        g_undo_mgr.push_action(std::make_unique<undo::ArrangerSingleTrackAction>(
                                            trk, before_clips, track_arranger_clips[t], "Clear Warp Pins"
                                        ));
                                        std::snprintf(status_toast, sizeof(status_toast), "CLEARED WARP PINS ON '%s'", c.name.c_str());
                                    }
                                }
                            }
                            ImGui::EndPopup();
                        }
                        draw_list->PopClipRect();
                    }
                    ImGui::EndChild();

                    // Horizontal Arranger Overview Scrollbar
                    float max_scroll = std::max(0.0f, g_arranger_max_bars - g_arranger_visible_bars);
                    if (max_scroll > 0.001f) {
                        ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.92f, 0.94f, 0.98f, 1.0f));
                        ImGui::PushStyleColor(ImGuiCol_SliderGrab, ImVec4(0.12f, 0.38f, 0.85f, 0.85f));
                        ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, ImVec4(0.08f, 0.30f, 0.75f, 1.0f));
                        ImGui::SetNextItemWidth(-1.0f);
                        char scroll_fmt[64];
                        std::snprintf(scroll_fmt, sizeof(scroll_fmt), "PAN: Bar %.1f - %.1f (Total: %.0f Bars)",
                                      g_arranger_scroll_bar + 1.0f, g_arranger_scroll_bar + g_arranger_visible_bars + 1.0f, g_arranger_max_bars);
                        if (ImGui::SliderFloat("##ArrangerTimelineScrollbar", &g_arranger_scroll_bar, 0.0f, max_scroll, scroll_fmt)) {
                            // clamped automatically
                        }
                        ImGui::PopStyleColor(3);
                    }

                    // ========================================================
                    ImGui::EndTabItem();
                }

                // ------------------------------------------------------------
                // TAB 2: SESSION CLIP LAUNCHER (BITWIG / ABLETON LIVE STYLE)
                // ------------------------------------------------------------
                if (ImGui::BeginTabItem("  SESSION CLIP LAUNCHER  ")) {
                    ImVec2 avail_sz = ImGui::GetContentRegionAvail();
                    avail_sz.y = std::max(avail_sz.y - 4.0f, 130.0f);

                    ImGui::BeginChild("SessionClipLauncherPane", ImVec2(0, 0), true);
                    {
                        // Master Scene Launcher Row
                        ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "MASTER SCENES:");
                        ImGui::SameLine();
                        if (ImGui::Button("▶ SCENE 1 (MAIN)", ImVec2(160, 28))) {
                            mixer.launch_scene(0, sequencer::LaunchQuantize::Bar);
                            std::snprintf(status_toast, sizeof(status_toast), "SCENE 1 (MAIN GROOVES) QUEUED FOR NEXT DOWNBEAT");
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("▶ SCENE 2 (WSOLA BREAK)", ImVec2(185, 28))) {
                            mixer.launch_scene(1, sequencer::LaunchQuantize::Bar);
                            std::snprintf(status_toast, sizeof(status_toast), "SCENE 2 (WSOLA BREAK/MOD) QUEUED FOR NEXT DOWNBEAT");
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("▶ SCENE 3 (GLITCH/SUB)", ImVec2(175, 28))) {
                            mixer.launch_scene(2, sequencer::LaunchQuantize::Bar);
                            std::snprintf(status_toast, sizeof(status_toast), "SCENE 3 (GLITCH/SUB/REV) QUEUED FOR NEXT DOWNBEAT");
                        }
                        ImGui::SameLine(0, 20);
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.25f, 0.20f, 0.90f));
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                        if (ImGui::Button("■ STOP ALL CLIPS", ImVec2(150, 28))) {
                            mixer.stop_all_clips(sequencer::LaunchQuantize::Bar);
                            std::snprintf(status_toast, sizeof(status_toast), "ALL TRACK CLIPS STOPPING AT NEXT BAR");
                        }
                        ImGui::PopStyleColor(2);

                        ImGui::Separator();
                        ImGui::Spacing();

                        // 4 Track Rows
                        for (int t = 0; t < 4; ++t) {
                            ImGui::PushID(800 + t);
                            ImGui::BeginGroup();
                            {
                                bool is_sel = (selected_track == t);
                                if (ImGui::Selectable(track_names[t], is_sel, 0, ImVec2(180, 24))) {
                                    selected_track = t;
                                }
                                ImGui::SameLine();

                                const auto& badge = k_track_routing_badges[t];
                                ImGui::TextColored(badge.bg_color, "[%s]", badge.bus_name);
                                ImGui::SameLine(0, 15);

                                auto* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                                bool launcher_active = trk ? trk->is_clip_launcher_active() : false;

                                if (!launcher_active) {
                                    ImGui::TextColored(ImVec4(0.40f, 0.45f, 0.52f, 1.0f), "[STATUS: PLAYING TIMELINE ARRANGER]");
                                } else {
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.48f, 0.05f, 0.85f));
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                    if (ImGui::Button("⮌ RETURN TO ARRANGER", ImVec2(180, 24))) {
                                        if (trk) trk->clip_launcher().stop_immediate();
                                        std::snprintf(status_toast, sizeof(status_toast), "TRACK %d RETURNED TO ARRANGER", t + 1);
                                    }
                                    ImGui::PopStyleColor(2);
                                }

                                ImGui::SameLine(0, 15);
                                bool arp_on = g_track_phrase_arp_enabled[t];
                                if (arp_on) {
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.92f, 0.45f, 0.05f, 1.0f));
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                } else {
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.90f, 0.92f, 0.95f, 1.0f));
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.40f, 0.45f, 0.52f, 1.0f));
                                }
                                char arp_lbl[32];
                                std::snprintf(arp_lbl, sizeof(arp_lbl), arp_on ? "ARP: ONLINE##%d" : "ARP: BYPASS##%d", t);
                                if (ImGui::Button(arp_lbl, ImVec2(95, 24))) {
                                    g_track_phrase_arp_enabled[t] = !g_track_phrase_arp_enabled[t];
                                    if (!g_track_phrase_arp_enabled[t]) {
                                        g_phrase_player.all_notes_off();
                                        mod_matrix.poly_all_notes_off();
                                    }
                                    std::snprintf(status_toast, sizeof(status_toast), "TRACK %d: INSTRUMENT PHRASE ARPEGGIATOR %s",
                                                  t + 1, g_track_phrase_arp_enabled[t] ? "ARMED (ONLINE)" : "BYPASSED");
                                }
                                ImGui::PopStyleColor(2);

                                ImGui::Spacing();

                                for (int s = 1; s <= 3; ++s) {
                                    if (s > 1) ImGui::SameLine(0, 12);
                                    const auto& slot_info = trk ? trk->clip_launcher().slot(s - 1) : sequencer::ClipSlot{};
                                    auto slot_state = slot_info.state.load(std::memory_order_relaxed);

                                    ImGui::BeginGroup();
                                    {
                                        char slot_btn_lbl[64];
                                        if (slot_state == sequencer::SlotPlayState::Playing) {
                                            std::snprintf(slot_btn_lbl, sizeof(slot_btn_lbl), "▶ SLOT %d: PLAYING##t%ds%d", s, t, s);
                                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.38f, 0.85f, 1.0f));
                                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                        } else if (slot_state == sequencer::SlotPlayState::QueuedPlay) {
                                            std::snprintf(slot_btn_lbl, sizeof(slot_btn_lbl), "⧗ SLOT %d: QUEUED##t%ds%d", s, t, s);
                                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.48f, 0.05f, 1.0f));
                                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                        } else if (slot_state == sequencer::SlotPlayState::QueuedStop) {
                                            std::snprintf(slot_btn_lbl, sizeof(slot_btn_lbl), "■ SLOT %d: STOPPING##t%ds%d", s, t, s);
                                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.20f, 0.20f, 1.0f));
                                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                        } else {
                                            std::snprintf(slot_btn_lbl, sizeof(slot_btn_lbl), "SLOT %d##t%ds%d", s, t, s);
                                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.93f, 0.95f, 0.97f, 1.0f));
                                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.25f, 0.30f, 0.40f, 1.0f));
                                        }

                                        if (ImGui::Button(slot_btn_lbl, ImVec2(150, 32))) {
                                            uint32_t target_trk_id = trk ? trk->id() : static_cast<uint32_t>(t + 1);
                                            mixer.launch_track_clip(target_trk_id, s - 1, sequencer::LaunchQuantize::Bar, false);
                                            std::snprintf(status_toast, sizeof(status_toast), "TRACK %d CLIP %d (%s) QUEUED FOR NEXT DOWNBEAT",
                                                          t + 1, s, slot_info.name.c_str());
                                        }
                                        ImGui::PopStyleColor(2);

                                        ImGui::TextColored(ImVec4(0.25f, 0.30f, 0.40f, 1.0f), "%s", slot_info.name.c_str());
                                        const char* mode_name = (slot_info.playback_mode == sampling::PlaybackMode::BeatSyncTimeStretch) ? "BeatSync" :
                                                                ((slot_info.playback_mode == sampling::PlaybackMode::PitchShiftWsola) ? "PitchShift" :
                                                                ((slot_info.playback_mode == sampling::PlaybackMode::ReverseFree) ? "Reverse" : "Repitch"));
                                        ImGui::TextDisabled("%s | %+.1fst", mode_name, slot_info.pitch_semitones);
                                    }
                                    ImGui::EndGroup();
                                }

                                ImGui::SameLine(0, 15);
                                ImGui::BeginGroup();
                                {
                                    if (ImGui::Button("■ STOP TRACK##stpt", ImVec2(100, 32))) {
                                        uint32_t target_trk_id = trk ? trk->id() : static_cast<uint32_t>(t + 1);
                                        mixer.stop_track_clip(target_trk_id, sequencer::LaunchQuantize::Bar);
                                        std::snprintf(status_toast, sizeof(status_toast), "TRACK %d STOPPING AT NEXT BAR", t + 1);
                                    }
                                    ImGui::TextDisabled("Bar-Quantized");
                                }
                                ImGui::EndGroup();
                            }
                            ImGui::EndGroup();
                            if (t < 3) {
                                ImGui::Spacing();
                                ImGui::Separator();
                                ImGui::Spacing();
                            }
                            ImGui::PopID();
                        }
                    }
                    ImGui::EndChild();

                    ImGui::EndTabItem();
                }

                // ------------------------------------------------------------
                // TAB 3: PATTERN SEQUENCER // STEP GRID & TRACKER STUDIO
                // ------------------------------------------------------------
                ImGuiTabItemFlags t3_flags = (g_switch_to_main_tab == 2) ? ImGuiTabItemFlags_SetSelected : 0;
                if (ImGui::BeginTabItem("  PATTERN SEQUENCER // STEP GRID & TRACKER  ", nullptr, t3_flags)) {
                    if (g_switch_to_main_tab == 2) g_switch_to_main_tab = -1;
                    ImVec2 avail_sz = ImGui::GetContentRegionAvail();
                    avail_sz.y = std::max(avail_sz.y - 4.0f, 130.0f);
                    const char* rack_trk_names[4] = { "TRK 1 (DRUMS)", "TRK 2 (BASS)", "TRK 3 (LEAD)", "TRK 4 (PERC)" };

                    // Reusable Phrase & Scales Lab Drawer Renderer
                    auto render_instrument_phrase_and_scales_lab = [&](ImVec2 lab_avail) {
                        float left_col_w = 420.0f;
                        float right_col_w = std::max(lab_avail.x - left_col_w - 20.0f, 400.0f);

                        // ========================================================
                        // LEFT COLUMN: TRACKER TIMING & SCALES / TUNING ENGINE
                        // ========================================================
                        ImGui::BeginChild("LeftTimingAndScalesCol", ImVec2(left_col_w, 460.0f), true);
                        {
                            // 1. Renoise Tracker Timing Engine
                            ImGui::TextColored(ImVec4(0.12f, 0.45f, 0.95f, 1.0f), "RENOISE TRACKER TIMING ENGINE");
                            ImGui::Separator();
                            ImGui::Spacing();

                            ImGui::TextDisabled("Lines Per Beat (LPB):");
                            ImGui::SameLine();
                            const char* lpb_names[] = { "4 (16th notes)", "8 (32nd notes)", "12 (triplets)", "16 (64ths)", "32 (high-res)" };
                            ImGui::SetNextItemWidth(160);
                            if (ImGui::Combo("##TimingLPB", &g_tracker_lpb_idx, lpb_names, 5)) {
                                g_tracker_timing.lpb = g_lpb_options[g_tracker_lpb_idx];
                            }

                            ImGui::TextDisabled("Ticks Per Line (TPL):");
                            ImGui::SameLine();
                            const char* tpl_names[] = { "8 Ticks", "12 Ticks", "16 Ticks", "24 Ticks" };
                            ImGui::SetNextItemWidth(160);
                            if (ImGui::Combo("##TimingTPL", &g_tracker_tpl_idx, tpl_names, 4)) {
                                g_tracker_timing.tpl = g_tpl_options[g_tracker_tpl_idx];
                            }

                            int lines_pat = static_cast<int>(g_tracker_timing.lines_per_pattern);
                            ImGui::TextDisabled("Pattern Length (Lines):");
                            ImGui::SameLine();
                            ImGui::SetNextItemWidth(150);
                            if (ImGui::SliderInt("##PatLen", &lines_pat, 16, 256, "%d Lines")) {
                                g_tracker_timing.lines_per_pattern = static_cast<uint32_t>(lines_pat);
                            }

                            // Realtime Timing Metrics
                            g_tracker_timing.bpm = bpm;
                            double line_ms = g_tracker_timing.line_duration_seconds() * 1000.0;
                            double tick_ms = g_tracker_timing.tick_duration_seconds() * 1000.0;
                            double spl = g_tracker_timing.samples_per_line(kSampleRate);
                            double spt = g_tracker_timing.samples_per_tick(kSampleRate);

                            uint64_t cur_sample = mixer.clock().sample_position();
                            auto trk_pos = g_tracker_timing.position_at_sample(cur_sample, kSampleRate);

                            ImGui::Spacing();
                            ImGui::TextColored(ImVec4(0.20f, 0.85f, 0.45f, 1.0f),
                                               "LINE: %.2f ms (%.0f smp) | TICK: %.2f ms (%.0f smp)",
                                               line_ms, spl, tick_ms, spt);
                            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.20f, 1.0f),
                                               "TRACKER POS: PAT %02u | LINE %02u | TICK %02u (+%.2f)",
                                               trk_pos.pattern_index, trk_pos.line_in_pattern,
                                               trk_pos.tick_in_line, trk_pos.tick_fraction);

                            ImGui::Spacing();
                            ImGui::Separator();
                            ImGui::Spacing();

                            // 2. Musical Scale Catalog & Snap
                            ImGui::TextColored(ImVec4(0.95f, 0.55f, 0.15f, 1.0f), "MUSICAL SCALE & QUANTIZE ENGINE");
                            ImGui::Separator();
                            ImGui::Spacing();

                            const char* root_names[] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
                            ImGui::TextDisabled("Root Note:");
                            ImGui::SameLine();
                            ImGui::SetNextItemWidth(70);
                            ImGui::Combo("##ScaleRoot", &g_scale_root, root_names, 12);

                            ImGui::SameLine();
                            ImGui::TextDisabled("Mode:");
                            ImGui::SameLine();
                            const char* scale_names[] = {
                                "Chromatic", "Major (Ionian)", "Natural Minor", "Harmonic Minor",
                                "Melodic Minor", "Dorian", "Phrygian", "Lydian", "Mixolydian",
                                "Locrian", "Pentatonic Major", "Pentatonic Minor", "Blues",
                                "Arabic Hijaz", "Japanese Insen", "Whole Tone",
                                "Diminished (W-H)", "Diminished (H-W)", "Hungarian Minor", "Bhairav"
                            };
                            ImGui::SetNextItemWidth(170);
                            if (ImGui::Combo("##ScaleType", &g_scale_type_idx, scale_names, 20)) {
                                g_selected_scale = static_cast<tuning::ScaleType>(g_scale_type_idx);
                            }

                            const char* snap_names[] = { "Nearest", "Floor (Down)", "Ceil (Up)" };
                            ImGui::TextDisabled("Snap Mode:");
                            ImGui::SameLine();
                            ImGui::SetNextItemWidth(130);
                            if (ImGui::Combo("##SnapMode", &g_scale_snap_mode_idx, snap_names, 3)) {
                                g_scale_snap_mode = static_cast<tuning::SnapMode>(g_scale_snap_mode_idx);
                            }

                            ImGui::SameLine();
                            if (ImGui::Checkbox("Snap Active", &g_scale_snap_enabled)) {
                                if (g_scale_snap_enabled) {
                                    std::snprintf(status_toast, sizeof(status_toast), "SCALE SNAP: %s %s (%s)",
                                                  root_names[g_scale_root], scale_names[g_scale_type_idx], snap_names[g_scale_snap_mode_idx]);
                                }
                            }
                            if (g_scale_snap_enabled) {
                                uint8_t test_snap = tuning::ScaleDictionary::snap_to_scale(
                                    61, static_cast<uint8_t>(60 + g_scale_root), g_selected_scale, g_scale_snap_mode);
                                ImGui::TextDisabled("Preview: C#4 -> %s", tuning::ScaleDictionary::note_name(test_snap).c_str());
                            }

                            // Visual 12-Tone Scale Degree Display
                            ImGui::Spacing();
                            ImGui::TextDisabled("Active Scale Tones:");
                            for (int k = 0; k < 12; ++k) {
                                uint8_t note_cand = static_cast<uint8_t>(60 + k);
                                bool in_scale = tuning::ScaleDictionary::is_note_in_scale(note_cand, static_cast<uint8_t>(60 + g_scale_root), g_selected_scale);
                                int deg = tuning::ScaleDictionary::get_degree(note_cand, static_cast<uint8_t>(60 + g_scale_root), g_selected_scale);

                                ImVec4 col = in_scale ? ImVec4(0.20f, 0.90f, 0.45f, 1.0f) : ImVec4(0.25f, 0.28f, 0.35f, 0.6f);
                                ImGui::PushStyleColor(ImGuiCol_Button, col);
                                char k_lbl[16];
                                if (in_scale) {
                                    std::snprintf(k_lbl, sizeof(k_lbl), "%s%d", root_names[k], deg + 1);
                                } else {
                                    std::snprintf(k_lbl, sizeof(k_lbl), "%s", root_names[k]);
                                }
                                ImGui::Button(k_lbl, ImVec2(28, 24));
                                ImGui::PopStyleColor();
                                if (k < 11) ImGui::SameLine(0, 3);
                            }

                            ImGui::Spacing();
                            ImGui::Separator();
                            ImGui::Spacing();

                            // 3. Microtonal Tuning & Scala Parser
                            ImGui::TextColored(ImVec4(0.85f, 0.35f, 0.95f, 1.0f), "MICROTONAL TUNING & SCALA (.SCL)");
                            ImGui::Separator();
                            ImGui::Spacing();

                            const char* tuning_presets[] = {
                                "12-TET Standard (Concert A4=440Hz)",
                                "Just Intonation (5-Limit)",
                                "24-EDO (Quarter-Tone Scale)",
                                "Pythagorean (Pure 3:2 Fifths)",
                                "Werckmeister III (Baroque Well-Temp)",
                                "Bohlen-Pierce (3:1 Tritave, 13 steps)",
                                "Arabic Maqam Rast (Neutral 3rd/7th)"
                            };
                            ImGui::TextDisabled("Preset:");
                            ImGui::SetNextItemWidth(left_col_w - 30);
                            if (ImGui::Combo("##TuningPreset", &g_tuning_preset_idx, tuning_presets, 7)) {
                                switch (g_tuning_preset_idx) {
                                    case 0: g_tuning_table.make_12tet(440.0); break;
                                    case 1: g_tuning_table.make_just_intonation(static_cast<uint8_t>(60 + g_scale_root), 440.0); break;
                                    case 2: g_tuning_table.make_24edo(440.0); break;
                                    case 3: g_tuning_table.make_pythagorean(static_cast<uint8_t>(60 + g_scale_root), 440.0); break;
                                    case 4: g_tuning_table.make_werckmeister_iii(static_cast<uint8_t>(60 + g_scale_root), 440.0); break;
                                    case 5: g_tuning_table.make_bohlen_pierce(220.0); break;
                                    case 6: g_tuning_table.make_arabic_rast(static_cast<uint8_t>(60 + g_scale_root), 440.0); break;
                                }
                                std::snprintf(status_toast, sizeof(status_toast), "TUNING APPLIED: %s", g_tuning_table.name.c_str());
                            }

                            ImGui::TextColored(ImVec4(0.35f, 0.85f, 0.95f, 1.0f), "Active: %s", g_tuning_table.name.c_str());
                            ImGui::TextDisabled("Ref: %s (Note %u) = %.2f Hz",
                                                tuning::ScaleDictionary::note_name(g_tuning_table.reference_note).c_str(),
                                                g_tuning_table.reference_note, g_tuning_table.reference_frequency);

                            ImGui::Spacing();
                            ImGui::TextDisabled("Import Huygens-Fokker Scala (.scl):");
                            ImGui::InputTextMultiline("##SclImport", g_scl_import_buffer, sizeof(g_scl_import_buffer),
                                                       ImVec2(left_col_w - 30, 75));
                            if (ImGui::Button("PARSE SCALA STRING")) {
                                if (g_scl_import_buffer[0] != 0) {
                                    if (g_tuning_table.parse_scl(g_scl_import_buffer, static_cast<uint8_t>(60 + g_scale_root), 261.625565)) {
                                        std::snprintf(status_toast, sizeof(status_toast), "SCALA IMPORTED: %s", g_tuning_table.name.c_str());
                                    } else {
                                        std::snprintf(status_toast, sizeof(status_toast), "ERROR: Failed to parse Scala (.scl) format!");
                                    }
                                }
                            }
                        }
                        ImGui::EndChild();

                        ImGui::SameLine();

                        // ========================================================
                        // RIGHT COLUMN: INSTRUMENT PHRASES & ARPEGGIATOR
                        // ========================================================
                        ImGui::BeginChild("RightPhraseArpCol", ImVec2(right_col_w, 460.0f), true);
                        {
                            ImGui::TextColored(ImVec4(0.95f, 0.35f, 0.45f, 1.0f), "INSTRUMENT-OWNED PHRASES & ARPEGGIATOR");
                            ImGui::Separator();
                            ImGui::Spacing();

                            // Realtime Arp Track Target & Status
                            ImGui::TextDisabled("Target Track:");
                            ImGui::SameLine();
                            const char* trk_targets[] = { "Track 1 (TRK 1)", "Track 2 (TRK 2)", "Track 3 (TRK 3)", "Track 4 (TRK 4)" };
                            ImGui::SetNextItemWidth(140);
                            if (ImGui::Combo("##PhraseTargetTrackCombo", &selected_track, trk_targets, 4)) {
                                g_active_phrase_idx = g_track_phrase_assigned[selected_track];
                            }
                            ImGui::SameLine();
                            bool cur_arp = g_track_phrase_arp_enabled[selected_track];
                            if (cur_arp) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.45f, 0.05f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            } else {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.25f, 0.35f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.65f, 0.70f, 0.80f, 1.0f));
                            }
                            char arp_btn_lbl[48];
                            std::snprintf(arp_btn_lbl, sizeof(arp_btn_lbl), "ARP: %s##Tab7Arp", cur_arp ? "ACTIVE [ON]" : "OFF");
                            if (ImGui::Button(arp_btn_lbl)) {
                                g_track_phrase_arp_enabled[selected_track] = !cur_arp;
                                if (g_track_phrase_arp_enabled[selected_track]) {
                                    g_track_phrase_assigned[selected_track] = g_active_phrase_idx;
                                    std::snprintf(status_toast, sizeof(status_toast), "TRACK %d: LIVE PHRASE ARP ROUTING ENABLED", selected_track + 1);
                                } else {
                                    g_phrase_player.all_notes_off();
                                    for (int v = 0; v < 16; ++v) mod_matrix.poly_note_off(v);
                                    std::snprintf(status_toast, sizeof(status_toast), "TRACK %d: ARP ROUTING DISABLED", selected_track + 1);
                                }
                            }
                            ImGui::PopStyleColor(2);
                            if (ImGui::IsItemHovered()) {
                                ImGui::SetTooltip("When ACTIVE, incoming MIDI notes on Track %d trigger tracker-timed phrase playback through PolySynth.", selected_track + 1);
                            }

                            ImGui::Spacing();

                            // Phrase Selector & Header Controls
                            ImGui::TextDisabled("Active Phrase:");
                            ImGui::SameLine();
                            ImGui::SetNextItemWidth(180);
                            std::vector<std::string> phrase_names;
                            for (size_t p = 0; p < g_phrase_bank.phrases.size(); ++p) {
                                phrase_names.push_back(std::to_string(p + 1) + ": " + g_phrase_bank.phrases[p].name);
                            }
                            std::vector<const char*> phrase_ptrs;
                            for (const auto& nm : phrase_names) phrase_ptrs.push_back(nm.c_str());

                            if (ImGui::Combo("##ActivePhraseCombo", &g_active_phrase_idx, phrase_ptrs.data(), static_cast<int>(phrase_ptrs.size()))) {
                                g_phrase_bank.active_phrase_index = static_cast<size_t>(g_active_phrase_idx);
                                g_track_phrase_assigned[selected_track] = g_active_phrase_idx;
                            }

                            ImGui::SameLine();
                            if (ImGui::Button("+ ADD PHRASE")) {
                                g_phrase_bank.add_phrase("New Phrase", 16);
                                g_active_phrase_idx = static_cast<int>(g_phrase_bank.phrases.size() - 1);
                                g_track_phrase_assigned[selected_track] = g_active_phrase_idx;
                            }

                            auto* phrase = g_phrase_bank.get_phrase(static_cast<size_t>(g_active_phrase_idx));
                            if (phrase) {
                                g_phrase_player.set_phrase(phrase);

                                // Mode Selector
                                ImGui::Spacing();
                                const char* mode_labels[] = {
                                    "Transposed (Melodic Root)",
                                    "Direct (Absolute Notes)",
                                    "Keymap (Drum Kit / Slices)",
                                    "Arp Up (Ascending Chord)",
                                    "Arp Down (Descending)",
                                    "Arp Up-Down (Ping-Pong)",
                                    "Arp Random (Generative)",
                                    "Arp Chord (Simultaneous)"
                                };
                                int cur_mode = static_cast<int>(phrase->playback_mode);
                                ImGui::TextDisabled("Playback Mode:");
                                ImGui::SameLine();
                                ImGui::SetNextItemWidth(200);
                                if (ImGui::Combo("##PhraseMode", &cur_mode, mode_labels, 8)) {
                                    phrase->playback_mode = static_cast<sequencer::PhraseArpMode>(cur_mode);
                                }

                                ImGui::SameLine();
                                int p_lines = static_cast<int>(phrase->num_lines);
                                ImGui::TextDisabled("Lines:");
                                ImGui::SameLine();
                                ImGui::SetNextItemWidth(90);
                                if (ImGui::SliderInt("##PhraseLines", &p_lines, 4, 64)) {
                                    phrase->resize(static_cast<uint32_t>(p_lines));
                                }

                                ImGui::SameLine();
                                ImGui::Checkbox("Loop", &phrase->loop);

                                // Quick Generator Buttons
                                ImGui::Spacing();
                                ImGui::TextDisabled("Quick Builders:");
                                ImGui::SameLine();
                                if (ImGui::SmallButton("Triad Arp")) {
                                    phrase->playback_mode = sequencer::PhraseArpMode::ArpUp;
                                    phrase->build_arpeggiator_pattern({0, 4, 7, 12, 16, 12, 7, 4}, 115);
                                }
                                ImGui::SameLine();
                                if (ImGui::SmallButton("Minor 9th Arp")) {
                                    phrase->playback_mode = sequencer::PhraseArpMode::ArpUp;
                                    phrase->build_arpeggiator_pattern({0, 3, 7, 10, 14, 12, 7, 3}, 110);
                                }
                                ImGui::SameLine();
                                if (ImGui::SmallButton("Jungle Break (Ratchet)")) {
                                    phrase->playback_mode = sequencer::PhraseArpMode::Keymap;
                                    phrase->build_drum_break_variation(0, 1, 2, true);
                                }
                                ImGui::SameLine();
                                if (ImGui::SmallButton("Clear")) {
                                    phrase->clear();
                                }

                                // Interactive Audition Buttons
                                ImGui::SameLine(0, 20);
                                if (ImGui::Button("[ AUDITION C-4 ]")) {
                                    g_phrase_player.all_notes_off();
                                    g_phrase_player.note_on(60, 110);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("[ AUDITION CHORD ]")) {
                                    g_phrase_player.all_notes_off();
                                    g_phrase_player.note_on(60, 110);
                                    g_phrase_player.note_on(64, 105);
                                    g_phrase_player.note_on(67, 100);
                                }
                                ImGui::SameLine();
                                if (ImGui::Button("[ STOP ]")) {
                                    g_phrase_player.all_notes_off();
                                    mod_matrix.poly_all_notes_off();
                                }

                                // Matrix Table
                                ImGui::Spacing();
                                ImGui::Separator();
                                ImGui::Spacing();

                                if (ImGui::BeginTable("PhraseLinesTable", 8,
                                                      ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                                      ImGuiTableFlags_ScrollY, ImVec2(0, 250))) {
                                    ImGui::TableSetupColumn("LN", ImGuiTableColumnFlags_WidthFixed, 32);
                                    ImGui::TableSetupColumn("Trig / Type", ImGuiTableColumnFlags_WidthFixed, 85);
                                    ImGui::TableSetupColumn("Note / Slice", ImGuiTableColumnFlags_WidthFixed, 100);
                                    ImGui::TableSetupColumn("Vel", ImGuiTableColumnFlags_WidthFixed, 55);
                                    ImGui::TableSetupColumn("Delay (Ticks)", ImGuiTableColumnFlags_WidthFixed, 75);
                                    ImGui::TableSetupColumn("Gate (Ticks)", ImGuiTableColumnFlags_WidthFixed, 75);
                                    ImGui::TableSetupColumn("Ratchet (Roll)", ImGuiTableColumnFlags_WidthFixed, 85);
                                    ImGui::TableSetupColumn("Prob %", ImGuiTableColumnFlags_WidthFixed, 60);
                                    ImGui::TableHeadersRow();

                                    const uint32_t cur_line = g_phrase_player.current_line();
                                    for (uint32_t l = 0; l < phrase->num_lines && l < phrase->lines.size(); ++l) {
                                        auto& line = phrase->lines[l];
                                        auto& note_col = line.columns[0];

                                        ImGui::TableNextRow();
                                        bool is_current = (g_phrase_player.is_playing() && cur_line == l);
                                        if (is_current) {
                                            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, ImGui::GetColorU32(ImVec4(0.85f, 0.45f, 0.05f, 0.35f)));
                                        }

                                        // Col 0: Line #
                                        ImGui::TableSetColumnIndex(0);
                                        ImGui::Text("%02u", l);

                                        // Col 1: Trig / Type
                                        ImGui::TableSetColumnIndex(1);
                                        ImGui::PushID(static_cast<int>(l));
                                        if (ImGui::Checkbox("##Act", &note_col.active)) {
                                            if (note_col.active && note_col.velocity == 0) note_col.velocity = 100;
                                        }
                                        ImGui::SameLine();
                                        if (note_col.active) {
                                            if (ImGui::SmallButton(note_col.is_slice ? "SLICE" : "NOTE")) {
                                                note_col.is_slice = !note_col.is_slice;
                                            }
                                        } else {
                                            ImGui::TextDisabled("--");
                                        }

                                        // Col 2: Note / Slice
                                        ImGui::TableSetColumnIndex(2);
                                        if (note_col.active) {
                                            if (note_col.is_slice) {
                                                int s_id = static_cast<int>(note_col.slice_id);
                                                ImGui::SetNextItemWidth(90);
                                                if (ImGui::InputInt("##SliceId", &s_id, 1, 4)) {
                                                    note_col.slice_id = static_cast<uint16_t>(std::clamp(s_id, 0, 65535));
                                                }
                                            } else {
                                                int n_off = static_cast<int>(note_col.note_offset);
                                                ImGui::SetNextItemWidth(90);
                                                if (ImGui::InputInt("##NoteOff", &n_off, 1, 12)) {
                                                    note_col.note_offset = static_cast<int8_t>(std::clamp(n_off, -60, 60));
                                                }
                                            }
                                        } else {
                                            ImGui::TextDisabled("...");
                                        }

                                        // Col 3: Velocity
                                        ImGui::TableSetColumnIndex(3);
                                        if (note_col.active) {
                                            int vel = static_cast<int>(note_col.velocity);
                                            ImGui::SetNextItemWidth(50);
                                            if (ImGui::SliderInt("##Vel", &vel, 1, 127)) {
                                                note_col.velocity = static_cast<uint8_t>(vel);
                                            }
                                        } else {
                                            ImGui::TextDisabled("--");
                                        }

                                        // Col 4: Delay Ticks
                                        ImGui::TableSetColumnIndex(4);
                                        if (note_col.active) {
                                            int del = static_cast<int>(note_col.delay_ticks);
                                            ImGui::SetNextItemWidth(65);
                                            if (ImGui::SliderInt("##Del", &del, 0, 15)) {
                                                note_col.delay_ticks = static_cast<uint8_t>(del);
                                            }
                                        } else {
                                            ImGui::TextDisabled("--");
                                        }

                                        // Col 5: Gate Ticks
                                        ImGui::TableSetColumnIndex(5);
                                        if (note_col.active) {
                                            int gate = static_cast<int>(note_col.gate_ticks);
                                            ImGui::SetNextItemWidth(65);
                                            if (ImGui::SliderInt("##Gate", &gate, 1, 48)) {
                                                note_col.gate_ticks = static_cast<uint16_t>(gate);
                                            }
                                        } else {
                                            ImGui::TextDisabled("--");
                                        }

                                        // Col 6: Ratchet Drill (Retrigger)
                                        ImGui::TableSetColumnIndex(6);
                                        if (note_col.active) {
                                            int ratch = static_cast<int>(note_col.retrigger_ticks);
                                            ImGui::SetNextItemWidth(75);
                                            if (ImGui::SliderInt("##Ratch", &ratch, 0, 8, ratch == 0 ? "OFF" : "Every %d")) {
                                                note_col.retrigger_ticks = static_cast<uint8_t>(ratch);
                                            }
                                        } else {
                                            ImGui::TextDisabled("--");
                                        }

                                        // Col 7: Probability
                                        ImGui::TableSetColumnIndex(7);
                                        if (note_col.active) {
                                            int prob = static_cast<int>(note_col.probability);
                                            ImGui::SetNextItemWidth(55);
                                            if (ImGui::SliderInt("##Prob", &prob, 0, 100)) {
                                                note_col.probability = static_cast<uint8_t>(prob);
                                            }
                                        } else {
                                            ImGui::TextDisabled("--");
                                        }

                                        ImGui::PopID();
                                    }
                                    ImGui::EndTable();
                                }
                            }
                        }
                        ImGui::EndChild();
                    };

                    ImGui::BeginChild("UnifiedPatternStudioPane", ImVec2(0, 0), true);
                    {
                        auto seq = track_seq[selected_track];
                        if (seq) {
                            auto& pat = seq->pattern(pattern_editor_pat_idx);

                            // Row 1: Dual Perspective Selector & Unified Studio Header
                            bool is_fl_view = (g_seq_view_mode == SequencerViewMode::FlChannelRack);
                            if (is_fl_view) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.45f, 0.95f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            } else {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.22f, 0.28f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.65f, 0.70f, 0.80f, 1.0f));
                            }
                            if (ImGui::SmallButton(" [■] FL CHANNEL RACK (HORIZONTAL) ")) {
                                g_seq_view_mode = SequencerViewMode::FlChannelRack;
                            }
                            ImGui::PopStyleColor(2);

                            ImGui::SameLine(0, 4);
                            bool is_trk_view = (g_seq_view_mode == SequencerViewMode::RenoiseMultiTracker);
                            if (is_trk_view) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.45f, 0.05f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            } else {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.22f, 0.28f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.65f, 0.70f, 0.80f, 1.0f));
                            }
                            if (ImGui::SmallButton(" [≡] RENOISE MULTI-TRACKER (VERTICAL) ")) {
                                g_seq_view_mode = SequencerViewMode::RenoiseMultiTracker;
                            }
                            ImGui::PopStyleColor(2);

                            ImGui::SameLine(0, 4);
                            bool is_gp_view = (g_seq_view_mode == SequencerViewMode::GridPieLiveMatrix);
                            if (is_gp_view) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.65f, 0.35f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            } else {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.22f, 0.28f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.65f, 0.70f, 0.80f, 1.0f));
                            }
                            if (ImGui::SmallButton(" [⊞] GRIDPIE LIVE MATRIX (REMIX) ")) {
                                g_seq_view_mode = SequencerViewMode::GridPieLiveMatrix;
                            }
                            ImGui::PopStyleColor(2);

                            ImGui::SameLine(0, 10);
                            if (g_seq_show_phrase_lab) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.35f, 0.45f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            } else {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.22f, 0.25f, 0.32f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.70f, 0.75f, 0.85f, 1.0f));
                            }
                            if (ImGui::SmallButton(g_seq_show_phrase_lab ? "[ ♫ PHRASE & SCALES LAB: OPEN ]" : "[ ♫ PHRASE & SCALES LAB ]")) {
                                g_seq_show_phrase_lab = !g_seq_show_phrase_lab;
                            }
                            ImGui::PopStyleColor(2);

                            ImGui::SameLine(0, 16);
                            // Pattern Selectors
                            for (int p = 0; p < 4; ++p) {
                                char p_lbl[32];
                                std::snprintf(p_lbl, sizeof(p_lbl), "PAT %d##p", p + 1);
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

                            // Voice Mode: Monophonic vs Polyphonic
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

                            ImGui::SameLine(0, 10);
                            auto cur_mode = seq->switch_mode();
                            const char* mode_str = (cur_mode == sequencer::PatternSwitchMode::BarQuantized) ? "BAR-SYNC" :
                                                   ((cur_mode == sequencer::PatternSwitchMode::BeatQuantized) ? "BEAT-SYNC" : "IMMEDIATE");
                            ImGui::TextColored(ImVec4(0.85f, 0.48f, 0.05f, 1.0f), "[SYNC: %s]", mode_str);

                            ImGui::SameLine(0, 12);
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
                            ImGui::SameLine(0, 10);
                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.35f, 0.08f, 0.90f));
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            if (ImGui::SmallButton("⚡ AUTO-CHOP & GROOVE##pat")) {
                                seq->auto_chop_and_groove(0.5f, pattern_editor_pat_idx);
                            }
                            ImGui::PopStyleColor(2);

                            ImGui::SameLine(0, 10);
                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.45f, 0.10f, 1.0f));
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            if (ImGui::SmallButton("⤓ STAMP TO ARRANGER (Ctrl+Enter)##pat_stamp")) {
                                stamp_pattern_to_arranger(selected_track, pattern_editor_pat_idx);
                            }
                            ImGui::PopStyleColor(2);
                            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Instantiate current pattern into Timeline Arranger at playhead.");

                            ImGui::SameLine(0, 6);
                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.60f, 0.35f, 1.0f));
                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            if (ImGui::SmallButton("⤓ BAKE SCENE (Ctrl+B)##gp_bake_hdr")) {
                                bake_gridpie_scene_to_arranger();
                            }
                            ImGui::PopStyleColor(2);
                            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Bakes the active pattern combination of all 4 tracks to Timeline Arranger at playhead.");

                            // Pattern Swing Slider
                            ImGui::SameLine(0, 10);
                            ImGui::SetNextItemWidth(70);
                            int swing_pct = static_cast<int>(std::round(pat.swing * 100.0f));
                            if (ImGui::SliderInt("Swing##pat", &swing_pct, 0, 100, "%d%%")) {
                                pat.swing = static_cast<float>(swing_pct) / 100.0f;
                            }

                            // Whole-Pattern Quick Quantize
                            ImGui::SameLine(0, 6);
                            if (ImGui::SmallButton("SNAP 100%##all")) {
                                pat.quantize_all(1.0f);
                            }
                            ImGui::SameLine();
                            if (ImGui::SmallButton("RAW 0%##all")) {
                                pat.quantize_all(0.0f);
                            }

                            bool seq_running = is_playing && (trk0->is_sequencer_enabled() || trk1->is_sequencer_enabled() ||
                                                              trk2->is_sequencer_enabled() || trk3->is_sequencer_enabled());

                            // ========================================================
                            // PERSPECTIVE 1: FL STUDIO CHANNEL RACK (HORIZONTAL MATRIX)
                            // ========================================================
                            if (g_seq_view_mode == SequencerViewMode::FlChannelRack) {
                                ImGui::Spacing();
                                ImGui::TextColored(ImVec4(0.12f, 0.45f, 0.95f, 1.0f), "FL STUDIO MULTI-TRACK CHANNEL RACK // 4 TRACKS x 16 STEPS:");
                                ImGui::Separator();
                                ImGui::Spacing();

                                for (int t = 0; t < 4; ++t) {
                                    ImGui::PushID(t);
                                    auto t_seq = track_seq[t];
                                    Track* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                                    if (!t_seq || !trk) {
                                        ImGui::PopID();
                                        continue;
                                    }
                                    auto& t_pat = t_seq->pattern(pattern_editor_pat_idx);
                                    bool is_sel_trk = (selected_track == t);

                                    // Track Header Controls
                                    ImGui::BeginGroup();
                                    if (is_sel_trk) {
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.45f, 0.95f, 0.95f));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                    } else {
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.22f, 0.28f, 0.90f));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.70f, 0.75f, 0.85f, 1.0f));
                                    }
                                    if (ImGui::Button(rack_trk_names[t], ImVec2(102, 26))) {
                                        selected_track = t;
                                    }
                                    ImGui::PopStyleColor(2);

                                    ImGui::SameLine(0, 3);
                                    bool is_muted = trk->is_muted();
                                    if (is_muted) {
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.25f, 0.25f, 1.0f));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                    } else {
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.22f, 0.25f, 0.32f, 0.8f));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.60f, 0.65f, 0.75f, 1.0f));
                                    }
                                    if (ImGui::Button(is_muted ? "M##m" : "M##m", ImVec2(22, 26))) {
                                        trk->set_mute(!is_muted);
                                    }
                                    ImGui::PopStyleColor(2);

                                    ImGui::SameLine(0, 2);
                                    bool is_solo = trk->is_solo();
                                    if (is_solo) {
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.95f, 0.75f, 0.15f, 1.0f));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.1f, 0.1f, 0.1f, 1.0f));
                                    } else {
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.22f, 0.25f, 0.32f, 0.8f));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.60f, 0.65f, 0.75f, 1.0f));
                                    }
                                    if (ImGui::Button(is_solo ? "S##s" : "S##s", ImVec2(22, 26))) {
                                        trk->set_solo(!is_solo);
                                    }
                                    ImGui::PopStyleColor(2);

                                    ImGui::SameLine(0, 2);
                                    bool seq_on = trk->is_sequencer_enabled();
                                    if (seq_on) {
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.58f, 0.32f, 1.0f));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                    } else {
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.22f, 0.25f, 0.32f, 0.8f));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.50f, 0.55f, 0.65f, 1.0f));
                                    }
                                    if (ImGui::Button(seq_on ? "ON##seq" : "--##seq", ImVec2(28, 26))) {
                                        trk->enable_sequencer(!seq_on);
                                    }
                                    ImGui::PopStyleColor(2);
                                    ImGui::EndGroup();

                                    // 16 Step Buttons for Track t
                                    uint32_t t_cur_step = t_seq->current_step_index();
                                    for (int st = 0; st < 16; ++st) {
                                        if (st > 0 && (st % 4 == 0)) {
                                            ImGui::SameLine(0, 10); // Beat separator gap
                                        } else {
                                            ImGui::SameLine(0, 3);
                                        }

                                        bool is_cur = (seq_running && t_cur_step == static_cast<uint32_t>(st));
                                        bool st_act = t_pat.is_step_active(st);
                                        int beat_idx = st / 4;
                                        bool is_beat_even = (beat_idx % 2 == 0);

                                        char st_lbl[24];
                                        if (st_act) {
                                            if (t_pat.steps[st].ratchet_count > 1) {
                                                std::snprintf(st_lbl, sizeof(st_lbl), "S%u:R%u##%d_%d", t_pat.steps[st].slice_id, t_pat.steps[st].ratchet_count, t, st);
                                            } else {
                                                std::snprintf(st_lbl, sizeof(st_lbl), "S%u##%d_%d", t_pat.steps[st].slice_id, t, st);
                                            }
                                            ImVec4 active_col;
                                            if (t == 0) active_col = ImVec4(0.15f, 0.45f, 0.90f, 0.95f);      // Drum Blue
                                            else if (t == 1) active_col = ImVec4(0.12f, 0.65f, 0.45f, 0.95f); // Acid Green
                                            else if (t == 2) active_col = ImVec4(0.85f, 0.40f, 0.10f, 0.95f); // Vocal Orange
                                            else active_col = ImVec4(0.70f, 0.30f, 0.85f, 0.95f);             // Perc Purple

                                            if (is_cur) {
                                                active_col = ImVec4(1.0f, 0.85f, 0.20f, 1.0f); // Playhead hit
                                            }
                                            ImGui::PushStyleColor(ImGuiCol_Button, active_col);
                                            ImGui::PushStyleColor(ImGuiCol_Text, is_cur ? ImVec4(0.1f, 0.1f, 0.1f, 1.0f) : ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                        } else {
                                            std::snprintf(st_lbl, sizeof(st_lbl), "%d##%d_%d", st + 1, t, st);
                                            ImVec4 inactive_col = is_beat_even ? ImVec4(0.88f, 0.90f, 0.93f, 1.0f) : ImVec4(0.94f, 0.95f, 0.97f, 1.0f);
                                            if (is_cur) {
                                                inactive_col = ImVec4(0.95f, 0.80f, 0.40f, 1.0f);
                                            }
                                            ImGui::PushStyleColor(ImGuiCol_Button, inactive_col);
                                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.50f, 0.55f, 0.62f, 1.0f));
                                        }

                                        if (ImGui::Button(st_lbl, ImVec2(34, 26))) {
                                            t_pat.toggle_step(st, 0, 0.9f);
                                            pattern_editor_step_idx = st;
                                            selected_track = t;
                                        }
                                        ImGui::PopStyleColor(2);
                                    }
                                    ImGui::PopID();
                                }

                                // Interactive Parameter Automation Drag Bars (for selected_track)
                                static int s_seq_auto_lane_target = 0; // 0=Vel, 1=Cutoff, 2=Decay, 3=Drive, 4=Rev A, 5=Dly B, 6=Pitch, 7=Pan, 8=Micro, 9=Prob
                                ImGui::Spacing();
                                ImGui::TextColored(ImVec4(0.35f, 0.40f, 0.48f, 1.0f), "AUTOMATION LANE (%s):", rack_trk_names[selected_track]);
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

                                ImGui::Spacing();
                                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 185.0f);
                                for (int st = 0; st < 16; ++st) {
                                    if (st > 0 && (st % 4 == 0)) {
                                        ImGui::SameLine(0, 10);
                                    } else if (st > 0) {
                                        ImGui::SameLine(0, 3);
                                    }

                                    ImGui::BeginGroup();
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

                                        if (s_seq_auto_lane_target == 0) {
                                            sref.velocity = drag_norm;
                                            if (!sref.active && drag_norm > 0.05f) sref.active = true;
                                        } else if (s_seq_auto_lane_target == 1) {
                                            sref.filter_cutoff = (drag_norm >= 0.98f) ? 20000.0f : 40.0f * std::pow(500.0f, drag_norm);
                                        } else if (s_seq_auto_lane_target == 2) {
                                            sref.decay_ms = (drag_norm >= 0.98f) ? 0.0f : drag_norm * 1500.0f;
                                        } else if (s_seq_auto_lane_target == 3) {
                                            sref.drive = drag_norm;
                                        } else if (s_seq_auto_lane_target == 4) {
                                            sref.send_a = drag_norm;
                                        } else if (s_seq_auto_lane_target == 5) {
                                            sref.send_b = drag_norm;
                                        } else if (s_seq_auto_lane_target == 6) {
                                            float bi = (drag_norm - 0.5f) * 2.0f;
                                            sref.pitch_ratio = std::pow(2.0f, bi);
                                        } else if (s_seq_auto_lane_target == 7) {
                                            sref.pan = (drag_norm - 0.5f) * 2.0f;
                                        } else if (s_seq_auto_lane_target == 8) {
                                            sref.micro_timing = (drag_norm - 0.5f);
                                        } else if (s_seq_auto_lane_target == 9) {
                                            sref.probability = static_cast<uint8_t>(std::round(drag_norm * 100.0f));
                                        }
                                    }

                                    dlist->AddRectFilled(b_p0, b_p1, IM_COL32(236, 239, 244, 255), 2.0f);

                                    float norm_fill = 0.0f;
                                    float bi_fill = 0.0f;
                                    bool is_bi = false;
                                    ImU32 fill_col = IM_COL32(31, 97, 217, 220);
                                    char val_txt[16] = "";

                                    const auto& cur_s = pat.steps[st];
                                    if (s_seq_auto_lane_target == 0) {
                                        norm_fill = cur_s.active ? cur_s.velocity : 0.0f;
                                        fill_col = IM_COL32(31, 97, 217, 220);
                                        std::snprintf(val_txt, sizeof(val_txt), "%d", static_cast<int>(std::round(cur_s.velocity * 100.0f)));
                                    } else if (s_seq_auto_lane_target == 1) {
                                        norm_fill = std::clamp(std::log(cur_s.filter_cutoff / 40.0f) / std::log(20000.0f / 40.0f), 0.0f, 1.0f);
                                        fill_col = IM_COL32(217, 123, 13, 220);
                                        if (cur_s.filter_cutoff >= 19900.0f) std::snprintf(val_txt, sizeof(val_txt), "BYP");
                                        else if (cur_s.filter_cutoff >= 1000.0f) std::snprintf(val_txt, sizeof(val_txt), "%.1fk", cur_s.filter_cutoff * 0.001f);
                                        else std::snprintf(val_txt, sizeof(val_txt), "%.0f", cur_s.filter_cutoff);
                                    } else if (s_seq_auto_lane_target == 2) {
                                        norm_fill = (cur_s.decay_ms <= 0.01f) ? 1.0f : std::clamp(cur_s.decay_ms / 1500.0f, 0.0f, 1.0f);
                                        fill_col = IM_COL32(16, 163, 127, 220);
                                        if (cur_s.decay_ms <= 0.01f) std::snprintf(val_txt, sizeof(val_txt), "FULL");
                                        else std::snprintf(val_txt, sizeof(val_txt), "%.0fm", cur_s.decay_ms);
                                    } else if (s_seq_auto_lane_target == 3) {
                                        norm_fill = cur_s.drive;
                                        fill_col = IM_COL32(220, 53, 69, 220);
                                        if (cur_s.drive < 0.01f) std::snprintf(val_txt, sizeof(val_txt), "--");
                                        else std::snprintf(val_txt, sizeof(val_txt), "%d%%", static_cast<int>(std::round(cur_s.drive * 100.0f)));
                                    } else if (s_seq_auto_lane_target == 4) {
                                        norm_fill = cur_s.send_a;
                                        fill_col = IM_COL32(138, 75, 232, 220);
                                        if (cur_s.send_a < 0.01f) std::snprintf(val_txt, sizeof(val_txt), "--");
                                        else std::snprintf(val_txt, sizeof(val_txt), "%d%%", static_cast<int>(std::round(cur_s.send_a * 100.0f)));
                                    } else if (s_seq_auto_lane_target == 5) {
                                        norm_fill = cur_s.send_b;
                                        fill_col = IM_COL32(14, 165, 233, 220);
                                        if (cur_s.send_b < 0.01f) std::snprintf(val_txt, sizeof(val_txt), "--");
                                        else std::snprintf(val_txt, sizeof(val_txt), "%d%%", static_cast<int>(std::round(cur_s.send_b * 100.0f)));
                                    } else if (s_seq_auto_lane_target == 6) {
                                        is_bi = true;
                                        bi_fill = std::clamp(std::log2(cur_s.pitch_ratio), -1.0f, 1.0f);
                                        fill_col = IM_COL32(99, 102, 241, 220);
                                        std::snprintf(val_txt, sizeof(val_txt), "%.2fx", cur_s.pitch_ratio);
                                    } else if (s_seq_auto_lane_target == 7) {
                                        is_bi = true;
                                        bi_fill = cur_s.pan;
                                        fill_col = IM_COL32(249, 115, 22, 220);
                                        if (std::abs(cur_s.pan) < 0.05f) std::snprintf(val_txt, sizeof(val_txt), "C");
                                        else if (cur_s.pan < 0.0f) std::snprintf(val_txt, sizeof(val_txt), "L%.0f", -cur_s.pan * 100.0f);
                                        else std::snprintf(val_txt, sizeof(val_txt), "R%.0f", cur_s.pan * 100.0f);
                                    } else if (s_seq_auto_lane_target == 8) {
                                        is_bi = true;
                                        bi_fill = cur_s.micro_timing * 2.0f;
                                        fill_col = IM_COL32(234, 88, 12, 220);
                                        std::snprintf(val_txt, sizeof(val_txt), "%+.0f%%", cur_s.micro_timing * 100.0f);
                                    } else if (s_seq_auto_lane_target == 9) {
                                        norm_fill = static_cast<float>(cur_s.probability) / 100.0f;
                                        fill_col = IM_COL32(100, 116, 139, 220);
                                        std::snprintf(val_txt, sizeof(val_txt), "%d%%", cur_s.probability);
                                    }

                                    if (!cur_s.active) {
                                        fill_col = (fill_col & 0x00FFFFFF) | 0x40000000;
                                    }

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

                                    ImU32 brd_col = b_hovered ? IM_COL32(31, 97, 217, 255) :
                                                   (pattern_editor_step_idx == st ? IM_COL32(217, 123, 13, 255) : IM_COL32(200, 205, 215, 255));
                                    dlist->AddRect(b_p0, b_p1, brd_col, 2.0f);

                                    ImVec2 txt_sz = ImGui::CalcTextSize(val_txt);
                                    ImVec2 txt_pos(b_p0.x + (bar_sz.x - txt_sz.x) * 0.5f, b_p0.y + (bar_sz.y - txt_sz.y) * 0.5f);
                                    ImU32 txt_col = cur_s.active ? IM_COL32(26, 30, 40, 255) : IM_COL32(140, 145, 155, 255);
                                    dlist->AddText(txt_pos, txt_col, val_txt);

                                    if (cur_s.active) {
                                        ImGui::TextColored(ImVec4(0.20f, 0.45f, 0.85f, 0.9f), "S%u", cur_s.slice_id);
                                    } else {
                                        ImGui::TextDisabled(" -- ");
                                    }
                                    ImGui::EndGroup();
                                }
                            }

                            // ========================================================
                            // PERSPECTIVE 2: RENOISE MULTI-TRACKER (VERTICAL MATRIX)
                            // ========================================================
                            if (g_seq_view_mode == SequencerViewMode::RenoiseMultiTracker) {
                                ImGui::Spacing();
                                ImGui::TextColored(ImVec4(0.85f, 0.45f, 0.05f, 1.0f), "RENOISE MULTI-TRACKER // 4-TRACK VERTICAL MATRIX:");
                                ImGui::SameLine(0, 15);
                                if (g_tracker_auto_scroll) {
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.55f, 0.35f, 1.0f));
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                    if (ImGui::SmallButton("↕ AUTO-SCROLL: CTR##tr_scr")) {
                                        g_tracker_auto_scroll = false;
                                    }
                                    ImGui::PopStyleColor(2);
                                } else {
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.35f, 0.38f, 0.45f, 0.8f));
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.9f, 0.9f, 0.9f, 1.0f));
                                    if (ImGui::SmallButton("↕ SCROLL: MANUAL##tr_scr")) {
                                        g_tracker_auto_scroll = true;
                                        g_tracker_last_scrolled_line = -1;
                                    }
                                    ImGui::PopStyleColor(2);
                                }
                                if (ImGui::IsItemHovered()) {
                                    ImGui::SetTooltip("Continuous Center-Scroll (Renoise Tracker behavior):\nWhen enabled, keeps the active playhead row centered vertically.\nScrolling the mouse wheel inside the tracker temporarily releases to MANUAL.");
                                }

                                ImGui::SameLine(0, 10);
                                ImGui::TextDisabled("LINES:");
                                ImGui::SameLine(0, 4);
                                const int line_counts[3] = { 16, 32, 64 };
                                for (int lc : line_counts) {
                                    char lc_lbl[16];
                                    std::snprintf(lc_lbl, sizeof(lc_lbl), "%dL##pat", lc);
                                    bool is_cur_lc = (g_tracker_lines_preset == lc);
                                    if (is_cur_lc) {
                                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.38f, 0.85f, 1.0f));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                    }
                                    if (ImGui::SmallButton(lc_lbl)) {
                                        g_tracker_lines_preset = lc;
                                        for (int t = 0; t < 4; ++t) {
                                            if (track_seq[t]) {
                                                track_seq[t]->pattern(pattern_editor_pat_idx).num_steps = static_cast<uint32_t>(lc);
                                            }
                                        }
                                        std::snprintf(status_toast, sizeof(status_toast), "TRACKER PATTERN LENGTH: %d LINES", lc);
                                    }
                                    if (is_cur_lc) ImGui::PopStyleColor(2);
                                    ImGui::SameLine(0, 3);
                                }

                                ImGui::SameLine(0, 15);
                                ImGui::TextDisabled("[Left-click select/toggle | Right-click clear]");

                                uint32_t num_tracker_lines = static_cast<uint32_t>(g_tracker_lines_preset);
                                for (int t = 0; t < 4; ++t) {
                                    if (track_seq[t]) {
                                        num_tracker_lines = std::max(num_tracker_lines, track_seq[t]->pattern(pattern_editor_pat_idx).num_steps);
                                    }
                                }
                                num_tracker_lines = std::clamp(num_tracker_lines, 16u, 64u);

                                if (ImGui::BeginTable("RenoiseMultiTrackerTable", 5,
                                                      ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                                      ImGuiTableFlags_ScrollY, ImVec2(0, 270))) {
                                    if (ImGui::IsWindowHovered() && std::abs(ImGui::GetIO().MouseWheel) > 0.01f) {
                                        g_tracker_auto_scroll = false;
                                    }

                                    ImGui::TableSetupColumn("LN", ImGuiTableColumnFlags_WidthFixed, 36);
                                    ImGui::TableSetupColumn("TRK 1 (DRUMS)", ImGuiTableColumnFlags_WidthStretch);
                                    ImGui::TableSetupColumn("TRK 2 (BASS)", ImGuiTableColumnFlags_WidthStretch);
                                    ImGui::TableSetupColumn("TRK 3 (LEAD)", ImGuiTableColumnFlags_WidthStretch);
                                    ImGui::TableSetupColumn("TRK 4 (PERC)", ImGuiTableColumnFlags_WidthStretch);
                                    ImGui::TableHeadersRow();

                                    uint32_t active_step = seq->current_step_index();

                                    if (!seq_running) {
                                        g_tracker_last_scrolled_line = -1;
                                    }

                                    for (uint32_t l = 0; l < num_tracker_lines; ++l) {
                                        ImGui::TableNextRow();
                                        bool is_cur_line = (seq_running && active_step == l);
                                        bool is_downbeat = (l % 4 == 0);

                                        if (g_tracker_auto_scroll && is_cur_line && g_tracker_last_scrolled_line != static_cast<int>(l)) {
                                            ImGui::SetScrollHereY(0.5f);
                                            g_tracker_last_scrolled_line = static_cast<int>(l);
                                        }

                                        if (is_cur_line) {
                                            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, ImGui::GetColorU32(ImVec4(0.95f, 0.70f, 0.10f, 0.35f)));
                                        } else if (is_downbeat) {
                                            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, ImGui::GetColorU32(ImVec4(0.15f, 0.22f, 0.35f, 0.30f)));
                                        }

                                        // Col 0: Line Number
                                        ImGui::TableSetColumnIndex(0);
                                        if (is_downbeat) {
                                            ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.20f, 1.0f), "%02u", l);
                                        } else {
                                            ImGui::TextDisabled("%02u", l);
                                        }

                                        // Cols 1..4: Track Step Cells
                                        for (int t = 0; t < 4; ++t) {
                                            ImGui::TableSetColumnIndex(t + 1);
                                            ImGui::PushID(static_cast<int>(l * 10 + t));
                                            auto t_seq = track_seq[t];
                                            if (!t_seq) {
                                                ImGui::TextDisabled("---");
                                                ImGui::PopID();
                                                continue;
                                            }
                                            auto& t_pat = t_seq->pattern(pattern_editor_pat_idx);
                                            auto& step = t_pat.steps[l];
                                            bool is_sel_cell = (selected_track == t && pattern_editor_step_idx == static_cast<int>(l));

                                            if (step.active) {
                                                char cell_txt[48];
                                                int hex_vel = static_cast<int>(std::round(step.velocity * 127.0f));
                                                char r_sfx[12] = "";
                                                if (step.ratchet_count > 1) {
                                                    std::snprintf(r_sfx, sizeof(r_sfx), " R%u", step.ratchet_count);
                                                }
                                                if (step.pitch_ratio > 0.01f && std::abs(step.pitch_ratio - 1.0f) > 1e-3f) {
                                                    int semi = static_cast<int>(std::round(12.0f * std::log2(step.pitch_ratio)));
                                                    std::snprintf(cell_txt, sizeof(cell_txt), "S%02u %+2d %02X%s##c", step.slice_id, semi, hex_vel, r_sfx);
                                                } else {
                                                    std::snprintf(cell_txt, sizeof(cell_txt), "S%02u -- %02X%s##c", step.slice_id, hex_vel, r_sfx);
                                                }

                                                ImVec4 cell_col;
                                                if (is_sel_cell) {
                                                    cell_col = ImVec4(0.12f, 0.45f, 0.95f, 1.0f);
                                                } else {
                                                    if (t == 0) cell_col = ImVec4(0.15f, 0.25f, 0.45f, 0.9f);
                                                    else if (t == 1) cell_col = ImVec4(0.12f, 0.35f, 0.30f, 0.9f);
                                                    else if (t == 2) cell_col = ImVec4(0.40f, 0.25f, 0.15f, 0.9f);
                                                    else cell_col = ImVec4(0.35f, 0.18f, 0.40f, 0.9f);
                                                }
                                                ImGui::PushStyleColor(ImGuiCol_Button, cell_col);
                                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.95f, 0.98f, 1.0f));
                                                if (ImGui::Button(cell_txt, ImVec2(-1, 20))) {
                                                    selected_track = t;
                                                    pattern_editor_step_idx = static_cast<int>(l);
                                                }
                                                ImGui::PopStyleColor(2);

                                                if (ImGui::IsItemClicked(ImGuiMouseButton_Right)) {
                                                    step.active = false;
                                                }
                                            } else {
                                                char empty_lbl[24];
                                                std::snprintf(empty_lbl, sizeof(empty_lbl), "· ··· ··##e%u_%d", l, t);
                                                ImVec4 cell_col = is_sel_cell ? ImVec4(0.12f, 0.45f, 0.95f, 0.35f) : ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
                                                ImGui::PushStyleColor(ImGuiCol_Button, cell_col);
                                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.40f, 0.44f, 0.52f, 0.7f));
                                                if (ImGui::Button(empty_lbl, ImVec2(-1, 20))) {
                                                    t_pat.set_step(l, 0, 0.9f);
                                                    selected_track = t;
                                                    pattern_editor_step_idx = static_cast<int>(l);
                                                }
                                                ImGui::PopStyleColor(2);
                                            }
                                            ImGui::PopID();
                                        }
                                    }
                                    ImGui::EndTable();
                                }

                                // Renoise Tracker Keyboard Navigation & Step Editing
                                if (!ImGui::GetIO().WantTextInput) {
                                    if (ImGui::IsKeyPressed(ImGuiKey_UpArrow)) {
                                        if (pattern_editor_step_idx > 0) pattern_editor_step_idx--;
                                        else pattern_editor_step_idx = static_cast<int>(num_tracker_lines - 1);
                                    } else if (ImGui::IsKeyPressed(ImGuiKey_DownArrow)) {
                                        if (pattern_editor_step_idx < static_cast<int>(num_tracker_lines - 1)) pattern_editor_step_idx++;
                                        else pattern_editor_step_idx = 0;
                                    } else if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow)) {
                                        if (selected_track > 0) selected_track--;
                                    } else if (ImGui::IsKeyPressed(ImGuiKey_RightArrow)) {
                                        if (selected_track < 3) selected_track++;
                                    } else if (ImGui::IsKeyPressed(ImGuiKey_Space)) {
                                        auto t_seq = track_seq[selected_track];
                                        if (t_seq) {
                                            auto& t_pat = t_seq->pattern(pattern_editor_pat_idx);
                                            t_pat.toggle_step(pattern_editor_step_idx, 0, 0.9f);
                                        }
                                    } else if (ImGui::IsKeyPressed(ImGuiKey_Delete) || ImGui::IsKeyPressed(ImGuiKey_Backspace)) {
                                        auto t_seq = track_seq[selected_track];
                                        if (t_seq) {
                                            t_seq->pattern(pattern_editor_pat_idx).steps[pattern_editor_step_idx].active = false;
                                        }
                                    } else if (ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter)) {
                                        if (!ImGui::GetIO().KeyCtrl) {
                                            auto t_seq = track_seq[selected_track];
                                            if (t_seq) {
                                                auto& st_ref = t_seq->pattern(pattern_editor_pat_idx).steps[pattern_editor_step_idx];
                                                t_seq->trigger_slice(st_ref.active ? st_ref.slice_id : 0, st_ref.active ? st_ref.velocity : 0.9f);
                                            }
                                        }
                                    } else if (ImGui::IsKeyPressed(ImGuiKey_R)) {
                                        auto t_seq = track_seq[selected_track];
                                        if (t_seq) {
                                            auto& st_ref = t_seq->pattern(pattern_editor_pat_idx).steps[pattern_editor_step_idx];
                                            if (!st_ref.active) {
                                                st_ref.active = true;
                                                st_ref.ratchet_count = 2;
                                            } else {
                                                st_ref.ratchet_count = (st_ref.ratchet_count >= 4) ? 1 : (st_ref.ratchet_count + 1);
                                            }
                                            std::snprintf(status_toast, sizeof(status_toast), "TRK %d LINE %02d: RATCHET %dx",
                                                          selected_track + 1, pattern_editor_step_idx, st_ref.ratchet_count);
                                        }
                                    } else {
                                        for (int k = 0; k < 8; ++k) {
                                            if (ImGui::IsKeyPressed(static_cast<ImGuiKey>(ImGuiKey_1 + k))) {
                                                auto t_seq = track_seq[selected_track];
                                                if (t_seq) {
                                                    auto& st_ref = t_seq->pattern(pattern_editor_pat_idx).steps[pattern_editor_step_idx];
                                                    st_ref.active = true;
                                                    st_ref.slice_id = static_cast<uint32_t>(k);
                                                    std::snprintf(status_toast, sizeof(status_toast), "TRK %d LINE %02d: SLICE %d",
                                                                  selected_track + 1, pattern_editor_step_idx, k);
                                                }
                                                break;
                                            }
                                        }
                                    }
                                }
                            }

                            // ========================================================
                            // PERSPECTIVE 3: RENOISE GRIDPIE LIVE MATRIX (SCENE REMIX)
                            // ========================================================
                            if (g_seq_view_mode == SequencerViewMode::GridPieLiveMatrix) {
                                ImGui::Spacing();
                                ImGui::TextColored(ImVec4(0.18f, 0.75f, 0.40f, 1.0f), "RENOISE GRIDPIE // 4-TRACK PATTERN MATRIX & LIVE SCENE REMIX:");
                                ImGui::SameLine(0, 15);
                                ImGui::TextDisabled("Quantization: BAR-SYNC");
                                ImGui::SameLine(0, 15);
                                ImGui::TextDisabled("[Left-click Launch/Queue | Right-click Edit Pattern]");

                                // Grid Matrix: 4 Tracks (rows) x 4 Patterns (cols)
                                if (ImGui::BeginTable("GridPieMatrixTable", 6, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg, ImVec2(0, 150))) {
                                    ImGui::TableSetupColumn("TRACK", ImGuiTableColumnFlags_WidthFixed, 130);
                                    ImGui::TableSetupColumn("PATTERN 1", ImGuiTableColumnFlags_WidthStretch);
                                    ImGui::TableSetupColumn("PATTERN 2", ImGuiTableColumnFlags_WidthStretch);
                                    ImGui::TableSetupColumn("PATTERN 3", ImGuiTableColumnFlags_WidthStretch);
                                    ImGui::TableSetupColumn("PATTERN 4", ImGuiTableColumnFlags_WidthStretch);
                                    ImGui::TableSetupColumn("STATUS", ImGuiTableColumnFlags_WidthFixed, 85);
                                    ImGui::TableHeadersRow();

                                    for (int t = 0; t < 4; ++t) {
                                        ImGui::TableNextRow();
                                        auto t_seq = track_seq[t];
                                        Track* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                                        if (!t_seq || !trk) continue;

                                        uint32_t active_p = t_seq->current_pattern_index();
                                        bool has_queued = t_seq->has_queued_pattern();
                                        uint32_t queued_p = t_seq->queued_pattern_index();

                                        // Col 0: Track Header & Controls
                                        ImGui::TableSetColumnIndex(0);
                                        ImGui::PushID(t * 100);
                                        bool is_sel_trk = (selected_track == t);
                                        if (is_sel_trk) {
                                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.45f, 0.95f, 0.95f));
                                        } else {
                                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.22f, 0.28f, 0.90f));
                                        }
                                        if (ImGui::Button(rack_trk_names[t], ImVec2(80, 24))) {
                                            selected_track = t;
                                        }
                                        ImGui::PopStyleColor();

                                        ImGui::SameLine(0, 4);
                                        bool seq_on = trk->is_sequencer_enabled();
                                        if (seq_on) {
                                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.58f, 0.32f, 1.0f));
                                        } else {
                                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.25f, 0.28f, 0.35f, 0.8f));
                                        }
                                        if (ImGui::Button(seq_on ? "ON##gp" : "--##gp", ImVec2(28, 24))) {
                                            trk->enable_sequencer(!seq_on);
                                        }
                                        ImGui::PopStyleColor();

                                        // Cols 1..4: Pattern slots P1..P4
                                        for (int p = 0; p < 4; ++p) {
                                            ImGui::TableSetColumnIndex(p + 1);
                                            char slot_lbl[64];
                                            const auto& pat_info = t_seq->pattern(p);
                                            std::snprintf(slot_lbl, sizeof(slot_lbl), "P%d: %s (%d st)##gp_%d_%d",
                                                          p + 1, pat_info.name.c_str(), pat_info.num_steps, t, p);

                                            bool is_active_playing = (active_p == static_cast<uint32_t>(p));
                                            bool is_slot_queued = (has_queued && queued_p == static_cast<uint32_t>(p));

                                            ImVec4 btn_col;
                                            if (is_slot_queued) {
                                                btn_col = ImVec4(0.95f, 0.65f, 0.10f, 1.0f);
                                            } else if (is_active_playing) {
                                                btn_col = ImVec4(0.15f, 0.65f, 0.35f, 1.0f);
                                            } else {
                                                btn_col = ImVec4(0.22f, 0.25f, 0.32f, 0.85f);
                                            }

                                            ImGui::PushStyleColor(ImGuiCol_Button, btn_col);
                                            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                            if (ImGui::Button(slot_lbl, ImVec2(-1, 24))) {
                                                if (is_playing) {
                                                    t_seq->queue_pattern_switch(p, sequencer::PatternSwitchMode::BarQuantized);
                                                } else {
                                                    t_seq->switch_pattern_immediate(p);
                                                }
                                                if (!trk->is_sequencer_enabled()) trk->enable_sequencer(true);
                                            }
                                            ImGui::PopStyleColor(2);

                                            if (ImGui::IsItemClicked(ImGuiMouseButton_Right)) {
                                                pattern_editor_pat_idx = p;
                                                selected_track = t;
                                                std::snprintf(status_toast, sizeof(status_toast), "EDITING TRACK %d PATTERN %d", t + 1, p + 1);
                                            }
                                        }

                                        // Col 5: Status
                                        ImGui::TableSetColumnIndex(5);
                                        if (has_queued) {
                                            ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.10f, 1.0f), ">> P%u (BAR)", queued_p + 1);
                                        } else if (seq_on) {
                                            ImGui::TextColored(ImVec4(0.25f, 0.85f, 0.45f, 1.0f), "PLAY P%u", active_p + 1);
                                        } else {
                                            ImGui::TextDisabled("MUTED");
                                        }

                                        ImGui::PopID();
                                    }
                                    ImGui::EndTable();
                                }

                                // GridPie Scene Launchers
                                ImGui::Spacing();
                                ImGui::BeginGroup();
                                {
                                    ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.20f, 1.0f), "SCENE LAUNCHERS:");
                                    ImGui::SameLine(0, 10);
                                    for (int sc = 0; sc < 4; ++sc) {
                                        char sc_lbl[48];
                                        std::snprintf(sc_lbl, sizeof(sc_lbl), "► %s##sc%d", g_gridpie_scenes[sc].name, sc);
                                        bool is_cur_sc = (g_gridpie_active_scene == sc);
                                        if (is_cur_sc) {
                                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.65f, 0.35f, 1.0f));
                                        } else {
                                            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.25f, 0.28f, 0.35f, 0.9f));
                                        }
                                        if (ImGui::Button(sc_lbl, ImVec2(125, 24))) {
                                            g_gridpie_active_scene = sc;
                                            for (int t = 0; t < 4; ++t) {
                                                if (track_seq[t]) {
                                                    uint32_t tp = g_gridpie_scenes[sc].track_patterns[t];
                                                    if (is_playing) track_seq[t]->queue_pattern_switch(tp, sequencer::PatternSwitchMode::BarQuantized);
                                                    else track_seq[t]->switch_pattern_immediate(tp);
                                                }
                                            }
                                            std::snprintf(status_toast, sizeof(status_toast), "LAUNCHED SCENE: %s", g_gridpie_scenes[sc].name);
                                        }
                                        ImGui::PopStyleColor();
                                        ImGui::SameLine(0, 6);
                                    }

                                    if (ImGui::SmallButton("CAPTURE AS SCENE##gp_cap")) {
                                        for (int t = 0; t < 4; ++t) {
                                            if (track_seq[t]) g_gridpie_scenes[g_gridpie_active_scene].track_patterns[t] = track_seq[t]->current_pattern_index();
                                        }
                                        std::snprintf(status_toast, sizeof(status_toast), "CAPTURED CURRENT REMIX INTO SCENE %d", g_gridpie_active_scene + 1);
                                    }
                                }
                                ImGui::EndGroup();

                                // The GridPie Clou: Active Loop / Remix Row
                                ImGui::Spacing();
                                ImGui::Separator();
                                ImGui::Spacing();
                                ImGui::BeginGroup();
                                {
                                    ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.20f, 1.0f), "ACTIVE REMIX LOOP (GRIDPIE BUFFER):");
                                    ImGui::SameLine(0, 15);
                                    for (int t = 0; t < 4; ++t) {
                                        uint32_t p = track_seq[t] ? track_seq[t]->current_pattern_index() : 0;
                                        ImGui::TextColored(ImVec4(0.20f, 0.75f, 0.95f, 1.0f), "[TRK %d: P%u]", t + 1, p + 1);
                                        ImGui::SameLine(0, 6);
                                    }

                                    ImGui::SameLine(0, 20);
                                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.45f, 0.10f, 1.0f));
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                    if (ImGui::Button("⤓ BAKE TO TIMELINE / ARRANGER (Ctrl+B)##gp_bake", ImVec2(280, 26))) {
                                        bake_gridpie_scene_to_arranger();
                                    }
                                    ImGui::PopStyleColor(2);
                                    if (ImGui::IsItemHovered()) {
                                        ImGui::SetTooltip("Renoise GridPie 1-Key Action:\nStamps the entire active remix scene across all 4 tracks to the Arranger\nand auto-advances the playhead so pressing Ctrl+B again arranges consecutive bars!");
                                    }

                                    ImGui::SameLine(0, 10);
                                    if (ImGui::Button("⇧ SHIFT REMIX UP TO PATTERN##gp_shift", ImVec2(230, 26))) {
                                        for (int t = 0; t < 4; ++t) {
                                            if (track_seq[t]) {
                                                uint32_t cur_p = track_seq[t]->current_pattern_index();
                                                if (cur_p != static_cast<uint32_t>(pattern_editor_pat_idx)) {
                                                    track_seq[t]->pattern(pattern_editor_pat_idx) = track_seq[t]->pattern(cur_p);
                                                }
                                            }
                                        }
                                        std::snprintf(status_toast, sizeof(status_toast), "SHIFTED ACTIVE REMIX UP INTO PATTERN %d FOR ALL TRACKS", pattern_editor_pat_idx + 1);
                                    }
                                    if (ImGui::IsItemHovered()) {
                                        ImGui::SetTooltip("Copies each track's currently playing remix pattern into Pattern %d so you can edit and preserve the remix as a unified pattern!", pattern_editor_pat_idx + 1);
                                    }
                                }
                                ImGui::EndGroup();
                            }

                            // Shared Note Inspector & MPC Audition Pads (for selected_track and pattern_editor_step_idx)
                            ImGui::Spacing();
                            ImGui::Separator();

                            ImGui::BeginGroup();
                            {
                                ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "NOTE INSPECTOR [%s // Step %d]:",
                                                   rack_trk_names[selected_track], pattern_editor_step_idx + 1);
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
                                ImGui::SameLine(0, 10);
                                int r_count = static_cast<int>(step_ref.ratchet_count);
                                ImGui::SetNextItemWidth(65);
                                if (ImGui::SliderInt("Ratchet##StepProp", &r_count, 1, 8, "%dx")) {
                                    step_ref.ratchet_count = static_cast<uint8_t>(r_count);
                                }
                                if (step_ref.ratchet_count > 1) {
                                    ImGui::SameLine(0, 8);
                                    ImGui::SetNextItemWidth(70);
                                    ImGui::SliderFloat("R-Decay##StepProp", &step_ref.ratchet_decay, 0.2f, 1.5f, "%.2fx");
                                }

                                // Second line: Micro-Timing & Quantize
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

                                // Third line: Parameter Locks
                                ImGui::Spacing();
                                ImGui::TextColored(ImVec4(0.95f, 0.65f, 0.15f, 1.0f), "P-LOCKS:");
                                ImGui::SameLine(0, 8);

                                ImGui::SetNextItemWidth(88);
                                const char* choke_labels[] = { "Choke: Off", "Choke: G1", "Choke: G2", "Choke: G3", "Choke: G4" };
                                int cur_cg = static_cast<int>(step_ref.choke_group);
                                if (cur_cg > 4) cur_cg = 4;
                                if (ImGui::Combo("##ChokeGrp", &cur_cg, choke_labels, 5)) {
                                    step_ref.choke_group = static_cast<uint8_t>(cur_cg);
                                }

                                ImGui::SameLine(0, 8);
                                ImGui::SetNextItemWidth(95);
                                char cut_str[24];
                                if (step_ref.filter_cutoff >= 19900.0f) {
                                    std::snprintf(cut_str, sizeof(cut_str), "Cut: BYPASS");
                                } else {
                                    std::snprintf(cut_str, sizeof(cut_str), "Cut: %.0fHz", step_ref.filter_cutoff);
                                }
                                if (ImGui::SliderFloat("##CutoffLock", &step_ref.filter_cutoff, 40.0f, 20000.0f, cut_str, ImGuiSliderFlags_Logarithmic)) {
                                }

                                ImGui::SameLine(0, 6);
                                ImGui::SetNextItemWidth(65);
                                ImGui::SliderFloat("##ResLock", &step_ref.filter_res, 0.1f, 8.0f, "Q: %.2f");

                                ImGui::SameLine(0, 6);
                                ImGui::SetNextItemWidth(65);
                                const char* ftype_names[] = { "LP", "HP", "BP", "Notch" };
                                int ftype_idx = static_cast<int>(step_ref.filter_type);
                                if (ftype_idx < 0 || ftype_idx > 3) ftype_idx = 0;
                                if (ImGui::Combo("##FilterTypeLock", &ftype_idx, ftype_names, 4)) {
                                    step_ref.filter_type = static_cast<dsp::FilterType>(ftype_idx);
                                }

                                ImGui::SameLine(0, 8);
                                ImGui::SetNextItemWidth(90);
                                char decay_str[24];
                                if (step_ref.decay_ms <= 0.01f) {
                                    std::snprintf(decay_str, sizeof(decay_str), "Dec: FULL");
                                } else {
                                    std::snprintf(decay_str, sizeof(decay_str), "Dec: %.0fms", step_ref.decay_ms);
                                }
                                if (ImGui::SliderFloat("##DecayLock", &step_ref.decay_ms, 0.0f, 1500.0f, decay_str)) {
                                }

                                ImGui::SameLine(0, 8);
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

                            // Right sub-pane: MPC Audition Pads for selected_track
                            ImGui::BeginGroup();
                            {
                                ImGui::TextColored(ImVec4(0.85f, 0.48f, 0.05f, 1.0f), "MPC AUDITION PADS (%s):", rack_trk_names[selected_track]);
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

                            // ========================================================
                            // COLLAPSIBLE DRAWER: PHRASE & SCALES LAB
                            // ========================================================
                            if (g_seq_show_phrase_lab) {
                                ImGui::Spacing();
                                ImGui::Separator();
                                ImGui::Spacing();

                                ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.15f, 1.0f), "[ ♫ INSTRUMENT PHRASE & ARPEGGIATOR LAB // SCALES & TUNING ]");
                                ImGui::SameLine(0, 15);
                                ImGui::TextColored(ImVec4(0.20f, 0.75f, 0.95f, 1.0f), "BIDIRECTIONAL SYNC (TRK %d <-> PHRASE %d):",
                                                   selected_track + 1, g_active_phrase_idx + 1);
                                ImGui::SameLine(0, 8);
                                auto* cur_phrase = g_phrase_bank.get_phrase(static_cast<size_t>(g_active_phrase_idx));
                                if (ImGui::SmallButton("⇄ PUSH TRK -> PHRASE##SyncPush")) {
                                    if (cur_phrase) {
                                        sync_step_to_phrase(pat, *cur_phrase, g_tracker_timing.tpl);
                                        std::snprintf(status_toast, sizeof(status_toast), "SYNC: Pushed Track %d Pat %d -> Phrase %d (%s)",
                                                      selected_track + 1, pattern_editor_pat_idx + 1, g_active_phrase_idx + 1, cur_phrase->name.c_str());
                                    }
                                }
                                if (ImGui::IsItemHovered()) {
                                    ImGui::SetTooltip("Copies steps, velocity, micro-timing, and filter locks from Track %d to the active instrument phrase arpeggiator.", selected_track + 1);
                                }
                                ImGui::SameLine(0, 6);
                                if (ImGui::SmallButton("⇄ PULL PHRASE -> TRK##SyncPull")) {
                                    if (cur_phrase) {
                                        sync_phrase_to_step(*cur_phrase, pat, g_tracker_timing.tpl);
                                        std::snprintf(status_toast, sizeof(status_toast), "SYNC: Pulled Phrase %d (%s) -> Track %d Pat %d",
                                                      g_active_phrase_idx + 1, cur_phrase->name.c_str(), selected_track + 1, pattern_editor_pat_idx + 1);
                                    }
                                }
                                if (ImGui::IsItemHovered()) {
                                    ImGui::SetTooltip("Copies instrument phrase lines and notes into Track %d step pattern.", selected_track + 1);
                                }

                                ImGui::Spacing();
                                render_instrument_phrase_and_scales_lab(ImGui::GetContentRegionAvail());
                            }
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

        // --------------------------------------------------------------------
        // RESIZABLE ZONE SPLITTER HANDLE (DRAG NS TO ADJUST ARRANGER / MIXER)
        // --------------------------------------------------------------------
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.22f, 0.28f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.35f, 0.55f, 0.85f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.20f, 0.70f, 1.0f, 1.0f));
        ImGui::Button("##ZoneSplitter", ImVec2(-1, 6));
        if (ImGui::IsItemHovered() || ImGui::IsItemActive()) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
        }
        if (ImGui::IsItemActive()) {
            float mouse_delta_y = io.MouseDelta.y;
            if (available_h > 100.0f) {
                s_zone_split_ratio = std::clamp(s_zone_split_ratio + (mouse_delta_y / available_h), 0.08f, 0.92f);
            }
        }
        ImGui::PopStyleColor(3);

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
                    ImGui::BeginChild("MixerHorizontalScrollRegion", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);
                    const char* track_names[4] = { "1: Kick / 808", "2: Acid 303", "3: Vocal", "4: Drums" };
                    const char* console_types[4] = { "Digital", "Warm Analog", "Lush Console", "Direct Clean" };

                    for (int t = 0; t < 4; ++t) {
                        ImGui::PushID(t);
                        ImGui::BeginChild(track_names[t], ImVec2(240, 0), true);
                        {
                            // Track header with instant A/B DSP Audition toggle
                            auto* trk_ptr = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                            bool is_sel = (selected_track == t);
                            if (ImGui::Selectable(track_names[t], is_sel, 0, ImVec2(120, 22))) {
                                selected_track = t;
                            }
                            ImGui::SameLine();
                            bool dsp_bypassed = trk_ptr ? trk_ptr->is_dsp_bypassed() : false;
                            if (dsp_bypassed) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.92f, 0.45f, 0.05f, 1.0f)); // Bright Orange DRY
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                if (ImGui::Button("DRY (RAW)##DspBypass", ImVec2(95, 22))) {
                                    if (trk_ptr) trk_ptr->set_dsp_bypass(false);
                                }
                            } else {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.38f, 0.85f, 1.0f)); // Sapphire Blue DSP ON
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                if (ImGui::Button("DSP ACTIVE##DspBypass", ImVec2(95, 22))) {
                                    if (trk_ptr) trk_ptr->set_dsp_bypass(true);
                                }
                            }
                            ImGui::PopStyleColor(2);
                            if (ImGui::IsItemHovered()) {
                                ImGui::SetTooltip("Instant A/B DSP Auditioning:\n[DSP ACTIVE] = Inserts 1-4 + Console Harmonic Saturation enabled\n[DRY (RAW)] = Full Insert & Console bypass to hear pure source recording");
                            }
                            ImGui::Separator();

                            // TACTILE SOURCE SELECTION: [ TIMELINE ] [ LIVE IN ] [ MERGE ]
                            TrackInputMode in_mode = trk_ptr ? trk_ptr->input_mode() : TrackInputMode::InternalClip;
                            bool is_timeline = (in_mode == TrackInputMode::InternalClip);
                            bool is_live = (in_mode == TrackInputMode::PipeWireStream);
                            bool is_merge = (in_mode == TrackInputMode::MergeAll);

                            // Segmented Source Selection Row
                            if (is_timeline) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.45f, 0.75f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            } else {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.90f, 0.92f, 0.95f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.35f, 0.40f, 0.48f, 1.0f));
                            }
                            if (ImGui::Button("TIMELINE##src", ImVec2(68, 22))) {
                                if (trk_ptr) {
                                    trk_ptr->set_input_mode(TrackInputMode::InternalClip);
                                    pw.unlink_all_for_track(t + 1);
                                }
                            }
                            ImGui::PopStyleColor(2);
                            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Hear ONLY Timeline Arranger recordings.\n(Silence outside clips, zero microphone/line leakage)");

                            ImGui::SameLine(0, 3);

                            if (is_live) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.65f, 0.40f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            } else {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.90f, 0.92f, 0.95f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.35f, 0.40f, 0.48f, 1.0f));
                            }
                            if (ImGui::Button("LIVE IN##src", ImVec2(60, 22))) {
                                if (trk_ptr) trk_ptr->set_input_mode(TrackInputMode::PipeWireStream);
                            }
                            ImGui::PopStyleColor(2);
                            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Hear ONLY Live External Input (Microphone / Audio Interface stream)");

                            ImGui::SameLine(0, 3);

                            if (is_merge) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.48f, 0.05f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            } else {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.90f, 0.92f, 0.95f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.35f, 0.40f, 0.48f, 1.0f));
                            }
                            if (ImGui::Button("MERGE##src", ImVec2(52, 22))) {
                                if (trk_ptr) trk_ptr->set_input_mode(TrackInputMode::MergeAll);
                            }
                            ImGui::PopStyleColor(2);
                            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Sum Timeline recordings + Live external microphone input together");

                            ImGui::SameLine(0, 4);

                            // Phase Invert Button
                            bool phase_inv = trk_ptr ? trk_ptr->input_phase_invert() : false;
                            if (phase_inv) {
                                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.45f, 0.10f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            }
                            if (ImGui::Button("Ø##Phase", ImVec2(24, 22))) {
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

                            auto set_track_bus = [&](int new_bus_id) {
                                int old_bus_id = track_target_buses[t];
                                if (old_bus_id == new_bus_id) return;
                                track_target_buses[t] = new_bus_id;
                                protocol::MixerCommand cmd{};
                                cmd.type = protocol::MixerCommandType::SetTrackTargetBus;
                                cmd.target_id = t + 1; // 1-based track ID
                                cmd.secondary_id = (new_bus_id == 1) ? bus_drums->id() : ((new_bus_id == 2) ? bus_music->id() : 0);
                                mixer.post_command(cmd);

                                uint32_t bd_id = bus_drums ? bus_drums->id() : 0;
                                uint32_t bm_id = bus_music ? bus_music->id() : 0;
                                std::string bname = (new_bus_id == 1) ? "Bus A (Drums)" : ((new_bus_id == 2) ? "Bus B (Music)" : "Master Out");
                                g_undo_mgr.push_action(std::make_unique<undo::CustomLambdaAction>(
                                    "Route Track " + std::to_string(t + 1) + " -> " + bname,
                                    [t, old_bus_id, bd_id, bm_id, &mixer, &track_target_buses]() {
                                        track_target_buses[t] = old_bus_id;
                                        protocol::MixerCommand c{};
                                        c.type = protocol::MixerCommandType::SetTrackTargetBus;
                                        c.target_id = t + 1;
                                        c.secondary_id = (old_bus_id == 1) ? bd_id : ((old_bus_id == 2) ? bm_id : 0);
                                        mixer.post_command(c);
                                    },
                                    [t, new_bus_id, bd_id, bm_id, &mixer, &track_target_buses]() {
                                        track_target_buses[t] = new_bus_id;
                                        protocol::MixerCommand c{};
                                        c.type = protocol::MixerCommandType::SetTrackTargetBus;
                                        c.target_id = t + 1;
                                        c.secondary_id = (new_bus_id == 1) ? bd_id : ((new_bus_id == 2) ? bm_id : 0);
                                        mixer.post_command(c);
                                    }
                                ));
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

                            // 500-Series Modular Insert Strip (Zähl AM1 & API 500 Style)
                            static bool s_slot_unfolded[4][4] = {
                                {true, false, false, false},
                                {true, false, false, false},
                                {true, false, false, false},
                                {true, false, false, false}
                            };

                            ImGui::TextColored(ImVec4(0.35f, 0.40f, 0.48f, 1.0f), "500-Series Inserts:");
                            for (int s = 0; s < 4; ++s) {
                                ImGui::PushID(s);
                                bool has_proc = (trk_ptr && trk_ptr->slot(s).processor() != nullptr);
                                auto* proc = has_proc ? trk_ptr->slot(s).processor() : nullptr;
                                bool is_by = has_proc ? trk_ptr->slot(s).is_bypassed() : false;

                                // Header row: Accordion toggle, Slot name/menu button, Bypass
                                if (has_proc) {
                                    if (ImGui::SmallButton(s_slot_unfolded[t][s] ? "v" : ">")) {
                                        s_slot_unfolded[t][s] = !s_slot_unfolded[t][s];
                                    }
                                    ImGui::SameLine(0, 3);
                                } else {
                                    ImGui::TextDisabled("S%d:", s + 1);
                                    ImGui::SameLine(0, 4);
                                }

                                const char* slot_name = has_proc ? proc->name() : "[Empty]";
                                char btn_label[48];
                                std::snprintf(btn_label, sizeof(btn_label), "%.13s", slot_name);

                                if (has_proc) {
                                    if (is_by) {
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.55f, 0.55f, 0.60f, 1.0f));
                                    } else {
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.12f, 0.45f, 0.95f, 1.0f));
                                    }
                                } else {
                                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.40f, 0.45f, 0.50f, 0.7f));
                                }

                                if (ImGui::Button(btn_label, ImVec2(has_proc ? 88 : 105, 18))) {
                                    ImGui::OpenPopup("InsertSelectMenu");
                                }
                                ImGui::PopStyleColor();

                                if (ImGui::IsItemHovered() && has_proc) {
                                    ImGui::SetTooltip("%s\nClick to change/clear module", proc->name());
                                } else if (ImGui::IsItemHovered()) {
                                    ImGui::SetTooltip("Click to load 500-series module into Slot %d", s + 1);
                                }

                                if (ImGui::BeginPopup("InsertSelectMenu")) {
                                    ImGui::TextColored(ImVec4(0.12f, 0.45f, 0.95f, 1.0f), "TRACK %d - INSERT SLOT %d", t + 1, s + 1);
                                    ImGui::Separator();

                                    auto assign_slot_proc = [&](std::shared_ptr<IProcessor> new_p, const std::string& desc) {
                                        if (!trk_ptr) return;
                                        auto old_p = trk_ptr->slot(s).shared_processor();
                                        trk_ptr->slot(s).set_processor(new_p);
                                        s_slot_unfolded[t][s] = (new_p != nullptr);
                                        g_undo_mgr.push_action(std::make_unique<undo::InsertSlotAction>(
                                            &trk_ptr->slot(s), old_p, new_p, desc
                                        ));
                                    };

                                    if (ImGui::MenuItem("None / Clear Slot", nullptr, !has_proc)) {
                                        assign_slot_proc(nullptr, "Clear Slot " + std::to_string(s + 1) + " (Track " + std::to_string(t + 1) + ")");
                                    }
                                    ImGui::Separator();
                                    if (ImGui::MenuItem("Airwindows Baxandall EQ")) {
                                        if (trk_ptr) {
                                            auto p = std::make_shared<dsp::Baxandall>();
                                            p->init(kSampleRate);
                                            assign_slot_proc(p, "Insert Baxandall EQ (Track " + std::to_string(t + 1) + ")");
                                        }
                                    }
                                    if (ImGui::MenuItem("Airwindows ButterComp2")) {
                                        if (trk_ptr) {
                                            auto p = std::make_shared<dsp::ButterComp2>();
                                            p->init(kSampleRate);
                                            assign_slot_proc(p, "Insert ButterComp2 (Track " + std::to_string(t + 1) + ")");
                                        }
                                    }
                                    if (ImGui::MenuItem("Airwindows PurestDrive")) {
                                        if (trk_ptr) {
                                            auto p = std::make_shared<dsp::PurestDrive>();
                                            p->init(kSampleRate);
                                            assign_slot_proc(p, "Insert PurestDrive (Track " + std::to_string(t + 1) + ")");
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
                                            assign_slot_proc(p, "Insert DeRez2 Crunch (Track " + std::to_string(t + 1) + ")");
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
                                            assign_slot_proc(p, "Insert Liquid Vactrol (Track " + std::to_string(t + 1) + ")");
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
                                            assign_slot_proc(p, "Insert Buchla 292 LPG (Track " + std::to_string(t + 1) + ")");
                                        }
                                    }
                                    if (ImGui::MenuItem("Sovereign MultiHead ODE Compressor")) {
                                        if (trk_ptr) {
                                            auto p = std::make_shared<dsp::MultiHeadOdeProcessor>(kSampleRate, 4);
                                            p->init(kSampleRate);
                                            assign_slot_proc(p, "Insert MultiHead ODE (Track " + std::to_string(t + 1) + ")");
                                        }
                                    }
                                    if (ImGui::MenuItem("Sovereign Real-Time Transient Shaper")) {
                                        if (trk_ptr) {
                                            auto p = std::make_shared<dsp::TransientShaper>();
                                            p->init(kSampleRate);
                                            assign_slot_proc(p, "Insert Transient Shaper (Track " + std::to_string(t + 1) + ")");
                                        }
                                    }
                                    ImGui::EndPopup();
                                }

                                ImGui::SameLine();
                                if (ImGui::Checkbox("By", &is_by)) {
                                    if (trk_ptr) {
                                        trk_ptr->slot(s).set_bypass(is_by);
                                        InsertSlot* slot_ptr = &trk_ptr->slot(s);
                                        std::string s_desc = std::string(is_by ? "Bypass" : "Enable") + " Slot " + std::to_string(s + 1) + " (Track " + std::to_string(t + 1) + ")";
                                        g_undo_mgr.push_action(std::make_unique<undo::CustomLambdaAction>(
                                            s_desc,
                                            [slot_ptr, is_by]() { slot_ptr->set_bypass(!is_by); },
                                            [slot_ptr, is_by]() { slot_ptr->set_bypass(is_by); }
                                        ));
                                    }
                                }

                                if (has_proc && s_slot_unfolded[t][s]) {
                                    // Embedded 500-Series Faceplate Module with Rotary Controls
                                    ImGui::PushStyleColor(ImGuiCol_ChildBg, is_by ? ImVec4(0.95f, 0.95f, 0.96f, 0.7f) : ImVec4(0.91f, 0.94f, 0.98f, 0.9f));
                                    char mod_child_id[32];
                                    std::snprintf(mod_child_id, sizeof(mod_child_id), "ModStrip_%d_%d", t, s);
                                    ImGui::BeginChild(mod_child_id, ImVec2(0, 56), true, ImGuiWindowFlags_NoScrollbar);
                                    {
                                        std::string pname = proc->name();
                                        if (pname.find("Baxandall") != std::string::npos) {
                                            float bass = proc->get_parameter(0);
                                            float treb = proc->get_parameter(1);
                                            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 10.0f);
                                            if (ui::DrawRotaryKnob("Bass", &bass, -12.0f, 12.0f, "dB", 12.0f)) {
                                                proc->set_parameter(0, bass);
                                            }
                                            ImGui::SameLine(0, 16);
                                            if (ui::DrawRotaryKnob("Treb", &treb, -12.0f, 12.0f, "dB", 12.0f)) {
                                                proc->set_parameter(1, treb);
                                            }
                                        } else if (pname.find("ButterComp2") != std::string::npos) {
                                            float comp = proc->get_parameter(0);
                                            float out  = proc->get_parameter(1);
                                            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 10.0f);
                                            if (ui::DrawRotaryKnob("Comp", &comp, 0.0f, 1.0f, "", 12.0f)) {
                                                proc->set_parameter(0, comp);
                                            }
                                            ImGui::SameLine(0, 16);
                                            if (ui::DrawRotaryKnob("Out", &out, 0.0f, 2.0f, "", 12.0f)) {
                                                proc->set_parameter(1, out);
                                            }
                                        } else if (pname.find("PurestDrive") != std::string::npos) {
                                            float drv = proc->get_parameter(0);
                                            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 26.0f);
                                            if (ui::DrawRotaryKnob("Drive", &drv, 0.0f, 1.0f, "", 12.0f)) {
                                                proc->set_parameter(0, drv);
                                            }
                                            ImGui::SameLine(0, 8);
                                            ImGui::TextColored(ImVec4(0.85f, 0.35f, 0.10f, 1.0f), "%.0f%%", drv * 100.0f);
                                        } else if (pname.find("DeRez") != std::string::npos) {
                                            float rate = proc->get_parameter(0);
                                            float res  = proc->get_parameter(1);
                                            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 10.0f);
                                            if (ui::DrawRotaryKnob("Rate", &rate, 0.0f, 1.0f, "", 12.0f)) {
                                                proc->set_parameter(0, rate);
                                            }
                                            ImGui::SameLine(0, 16);
                                            if (ui::DrawRotaryKnob("Res", &res, 0.0f, 1.0f, "", 12.0f)) {
                                                proc->set_parameter(1, res);
                                            }
                                        } else if (pname.find("Vactrol") != std::string::npos || pname.find("LA-2A") != std::string::npos || pname.find("Buchla") != std::string::npos) {
                                            float red = proc->get_parameter(0);
                                            float gain = proc->get_parameter(1);
                                            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 10.0f);
                                            if (ui::DrawRotaryKnob("Red", &red, 0.0f, 1.0f, "", 12.0f)) {
                                                proc->set_parameter(0, red);
                                            }
                                            ImGui::SameLine(0, 16);
                                            if (ui::DrawRotaryKnob("Gain", &gain, -6.0f, 18.0f, "dB", 12.0f)) {
                                                proc->set_parameter(1, gain);
                                            }
                                        } else if (pname.find("Transient") != std::string::npos || pname.find("Shaper") != std::string::npos) {
                                            float att = proc->get_parameter(0);
                                            float sus = proc->get_parameter(1);
                                            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 10.0f);
                                            if (ui::DrawRotaryKnob("Att", &att, -15.0f, 15.0f, "dB", 12.0f)) {
                                                proc->set_parameter(0, att);
                                            }
                                            ImGui::SameLine(0, 16);
                                            if (ui::DrawRotaryKnob("Sus", &sus, -15.0f, 15.0f, "dB", 12.0f)) {
                                                proc->set_parameter(1, sus);
                                            }
                                        } else {
                                            uint32_t pcount = proc->parameter_count();
                                            if (pcount >= 2) {
                                                float p0 = proc->get_parameter(0);
                                                float p1 = proc->get_parameter(1);
                                                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 10.0f);
                                                if (ui::DrawRotaryKnob(proc->parameter_name(0), &p0, proc->parameter_min(0), proc->parameter_max(0), "", 12.0f)) {
                                                    proc->set_parameter(0, p0);
                                                }
                                                ImGui::SameLine(0, 16);
                                                if (ui::DrawRotaryKnob(proc->parameter_name(1), &p1, proc->parameter_min(1), proc->parameter_max(1), "", 12.0f)) {
                                                    proc->set_parameter(1, p1);
                                                }
                                            } else if (pcount == 1) {
                                                float p0 = proc->get_parameter(0);
                                                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 26.0f);
                                                if (ui::DrawRotaryKnob(proc->parameter_name(0), &p0, proc->parameter_min(0), proc->parameter_max(0), "", 12.0f)) {
                                                    proc->set_parameter(0, p0);
                                                }
                                            }
                                        }
                                    }
                                    ImGui::EndChild();
                                    ImGui::PopStyleColor();
                                }

                                ImGui::PopID();
                            }

                            ImGui::Separator();

                            // Pan Rotary Dial
                            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 40.0f);
                            static float s_pre_pan[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
                            if (ui::DrawRotaryKnob("Pan", &track_pans[t], -1.0f, 1.0f, "", 18.0f)) {
                                protocol::MixerCommand cmd{};
                                cmd.type = protocol::MixerCommandType::SetTrackPan;
                                cmd.target_id = t + 1;
                                cmd.value1 = track_pans[t];
                                mixer.post_command(cmd);
                            }
                            if (ImGui::IsItemDeactivatedAfterEdit()) {
                                float old_p = s_pre_pan[t];
                                float new_p = track_pans[t];
                                if (std::abs(old_p - new_p) > 1e-4f) {
                                    Track* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                                    g_undo_mgr.push_action(std::make_unique<undo::TrackGainPanAction>(
                                        trk, undo::TrackGainPanAction::TargetProperty::Pan, old_p, new_p, track_names[t]
                                    ));
                                }
                            }
                            if (!ImGui::IsItemActive()) {
                                s_pre_pan[t] = track_pans[t];
                            }

                            // Mute, Solo, Solo-Safe Buttons
                            if (ImGui::Checkbox("M", &track_mutes[t])) {
                                protocol::MixerCommand cmd{};
                                cmd.type = protocol::MixerCommandType::SetTrackMute;
                                cmd.target_id = t + 1;
                                cmd.flags = track_mutes[t] ? 1 : 0;
                                mixer.post_command(cmd);
                                Track* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                                g_undo_mgr.push_action(std::make_unique<undo::TrackMuteSoloAction>(
                                    trk, undo::TrackMuteSoloAction::TargetProperty::Mute, !track_mutes[t], track_mutes[t], track_names[t]
                                ));
                            }
                            ImGui::SameLine();
                            if (ImGui::Checkbox("S", &track_solos[t])) {
                                protocol::MixerCommand cmd{};
                                cmd.type = protocol::MixerCommandType::SetTrackSolo;
                                cmd.target_id = t + 1;
                                cmd.flags = track_solos[t] ? 1 : 0;
                                mixer.post_command(cmd);
                                Track* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                                g_undo_mgr.push_action(std::make_unique<undo::TrackMuteSoloAction>(
                                    trk, undo::TrackMuteSoloAction::TargetProperty::Solo, !track_solos[t], track_solos[t], track_names[t]
                                ));
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
                            static float s_pre_gain[4] = { 0.85f, 0.70f, 0.80f, 0.90f };
                            if (trk_ptr && !ImGui::IsItemActive()) {
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
                            if (ImGui::IsItemDeactivatedAfterEdit()) {
                                float old_g = s_pre_gain[t];
                                float new_g = track_gains[t];
                                if (std::abs(old_g - new_g) > 1e-4f) {
                                    Track* trk = (t == 0) ? trk0 : ((t == 1) ? trk1 : ((t == 2) ? trk2 : trk3));
                                    g_undo_mgr.push_action(std::make_unique<undo::TrackGainPanAction>(
                                        trk, undo::TrackGainPanAction::TargetProperty::Gain, old_g, new_g, track_names[t]
                                    ));
                                }
                            }
                            if (!ImGui::IsItemActive()) {
                                s_pre_gain[t] = track_gains[t];
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
                        auto* bprocA = bus_drums ? bus_drums->slot(0).processor() : nullptr;
                        if (bprocA) {
                            ImGui::TextColored(ImVec4(0.85f, 0.48f, 0.05f, 1.0f), "500: %s", bprocA->name());
                            float comp = bprocA->get_parameter(0);
                            float out  = bprocA->get_parameter(1);
                            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 10.0f);
                            if (ui::DrawRotaryKnob("Comp##ba", &comp, 0.0f, 1.0f, "", 12.0f)) {
                                bprocA->set_parameter(0, comp);
                            }
                            ImGui::SameLine(0, 14);
                            if (ui::DrawRotaryKnob("Out##ba", &out, 0.0f, 2.0f, "", 12.0f)) {
                                bprocA->set_parameter(1, out);
                            }
                        } else {
                            ImGui::TextColored(ImVec4(0.35f, 0.40f, 0.48f, 1.0f), "[Glue Compressor]");
                        }

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
                        auto* bprocB = bus_music ? bus_music->slot(0).processor() : nullptr;
                        if (bprocB) {
                            ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "500: %s", bprocB->name());
                            float p0 = bprocB->get_parameter(0);
                            float p1 = bprocB->get_parameter(1);
                            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 10.0f);
                            if (ui::DrawRotaryKnob(bprocB->parameter_name(0), &p0, bprocB->parameter_min(0), bprocB->parameter_max(0), "", 12.0f)) {
                                bprocB->set_parameter(0, p0);
                            }
                            ImGui::SameLine(0, 14);
                            if (ui::DrawRotaryKnob(bprocB->parameter_name(1), &p1, bprocB->parameter_min(1), bprocB->parameter_max(1), "", 12.0f)) {
                                bprocB->set_parameter(1, p1);
                            }
                        } else {
                            ImGui::TextColored(ImVec4(0.35f, 0.40f, 0.48f, 1.0f), "[Analog Summing]");
                        }

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
                        auto* mproc = mixer.master_bus().slot(0).processor();
                        if (mproc) {
                            ImGui::TextColored(ImVec4(0.85f, 0.18f, 0.22f, 1.0f), "500: %s", mproc->name());
                            float mbass = mproc->get_parameter(0);
                            float mtreb = mproc->get_parameter(1);
                            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 10.0f);
                            if (ui::DrawRotaryKnob("Bass##mst", &mbass, -12.0f, 12.0f, "dB", 12.0f)) {
                                mproc->set_parameter(0, mbass);
                            }
                            ImGui::SameLine(0, 14);
                            if (ui::DrawRotaryKnob("Treb##mst", &mtreb, -12.0f, 12.0f, "dB", 12.0f)) {
                                mproc->set_parameter(1, mtreb);
                            }
                        }

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
                                    } else if (pname.find("Lookahead") != std::string::npos || pname.find("DNL") != std::string::npos) {
                                        float thresh = proc->get_parameter(0);
                                        float red = proc->get_parameter(1);
                                        float la = proc->get_parameter(2);

                                        ImGui::SetNextItemWidth(80);
                                        if (ImGui::SliderFloat("Thr##dnl", &thresh, -60.0f, -10.0f, "%.0fdB")) {
                                            proc->set_parameter(0, thresh);
                                        }
                                        ImGui::SameLine(0, 6);
                                        ImGui::SetNextItemWidth(75);
                                        if (ImGui::SliderFloat("Red##dnl", &red, 0.0f, 36.0f, "%.0fdB")) {
                                            proc->set_parameter(1, red);
                                        }
                                        ImGui::SameLine(0, 6);
                                        ImGui::SetNextItemWidth(65);
                                        if (ImGui::SliderFloat("LA##dnl", &la, 0.0f, 64.0f, "%.0fsm")) {
                                            proc->set_parameter(2, la);
                                        }

                                        auto* dnl_proc = dynamic_cast<dsp::LookaheadDnlProcessor*>(proc);
                                        float air_atten = dnl_proc ? dnl_proc->current_air_gain_reduction_db() : 0.0f;
                                        float norm_gr = std::clamp(-air_atten / (red > 0.0f ? red : 18.0f), 0.0f, 1.0f);

                                        char atten_txt[64];
                                        std::snprintf(atten_txt, sizeof(atten_txt), "Air Hiss Atten: %.1f dB", air_atten);
                                        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.85f, 0.25f, 0.15f, 1.0f));
                                        ImGui::ProgressBar(norm_gr, ImVec2(-1, 8), atten_txt);
                                        ImGui::PopStyleColor();
                                        ImGui::TextDisabled("4-Band LR4 Crossover + 32-Sample Lookahead Noise Gate");
                                    } else if (pname.find("Transient") != std::string::npos || pname.find("Shaper") != std::string::npos) {
                                        float att = proc->get_parameter(0);
                                        float sus = proc->get_parameter(1);
                                        float len = proc->get_parameter(2);
                                        float trim = proc->get_parameter(3);

                                        ImGui::SetNextItemWidth(75);
                                        if (ImGui::SliderFloat("Attack##ts", &att, -15.0f, 15.0f, "%+.1fdB")) {
                                            proc->set_parameter(0, att);
                                        }
                                        ImGui::SameLine(0, 6);
                                        ImGui::SetNextItemWidth(75);
                                        if (ImGui::SliderFloat("Sustain##ts", &sus, -15.0f, 15.0f, "%+.1fdB")) {
                                            proc->set_parameter(1, sus);
                                        }
                                        ImGui::SameLine(0, 6);
                                        ImGui::SetNextItemWidth(65);
                                        if (ImGui::SliderFloat("Window##ts", &len, 1.0f, 25.0f, "%.0fms")) {
                                            proc->set_parameter(2, len);
                                        }
                                        ImGui::SameLine(0, 6);
                                        ImGui::SetNextItemWidth(65);
                                        if (ImGui::SliderFloat("Trim##ts", &trim, -15.0f, 15.0f, "%+.1fdB")) {
                                            proc->set_parameter(3, trim);
                                        }

                                        auto* ts = dynamic_cast<dsp::TransientShaper*>(proc);
                                        float t_int = ts ? ts->last_transient_intensity() : 0.0f;
                                        float s_int = ts ? ts->last_sustain_intensity() : 0.0f;
                                        ImGui::Text("Detection: Attack");
                                        ImGui::SameLine();
                                        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.85f, 0.20f, 0.15f, 1.0f));
                                        ImGui::ProgressBar(t_int, ImVec2(90, 6), "");
                                        ImGui::PopStyleColor();
                                        ImGui::SameLine(0, 10);
                                        ImGui::Text("Sustain");
                                        ImGui::SameLine();
                                        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.20f, 0.55f, 0.85f, 1.0f));
                                        ImGui::ProgressBar(s_int, ImVec2(90, 6), "");
                                        ImGui::PopStyleColor();
                                        ImGui::TextDisabled("Dual-Envelope Differential Transient & Sustain Shaper (Zero Latency)");
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
                                        if (ImGui::MenuItem("Sovereign Lookahead Multiband DNL")) {
                                            auto p = std::make_shared<dsp::LookaheadDnlProcessor>(kSampleRate);
                                            p->init(kSampleRate);
                                            p->set_parameter(0, -36.0f);
                                            p->set_parameter(1, 18.0f);
                                            p->set_parameter(2, 32.0f);
                                            p->set_parameter(3, 25.0f);
                                            sel_trk->slot(s).set_processor(p);
                                        }
                                        if (ImGui::MenuItem("Sovereign Real-Time Transient Shaper")) {
                                            auto p = std::make_shared<dsp::TransientShaper>();
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

                    ImGui::EndChild(); // MixerHorizontalScrollRegion
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
                    float cur_loop_len = get_loop_length_seconds(bpm);
                    float play_ratio = (cur_loop_len > 0.001f) ? (std::fmod(playhead_seconds, cur_loop_len) / cur_loop_len) : 0.0f;
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

                // ============================================================
                // TAB: SONONYM ASSET POOL & PROJECT BUNDLE MANAGER (.AETHEL)
                // ============================================================
                if (ImGui::BeginTabItem("  SONONYM ASSET POOL & BUNDLE (.AETHEL)  ")) {
                    // TOP SECTION: Self-Contained Project Packaging (.aethel)
                    ImGui::BeginChild("BundlePackagingPane", ImVec2(0, 160), true);
                    {
                        ImGui::TextColored(ImVec4(0.12f, 0.45f, 0.90f, 1.0f),
                                           "SELF-CONTAINED AETHEL PROJECT BUNDLE MANAGER (.AETHEL)");
                        ImGui::SameLine();
                        ImGui::TextDisabled("| Zero Broken Links & Bit-Exact Portability Across Machines");
                        ImGui::Separator();

                        ImGui::TextWrapped("Packages session routing, automation, plugins, and writes all active audio clips into "
                                           "an 'assets/' folder named by deterministic FNV-1a 64-bit content hashes. "
                                           "Drag & drop any .aethel folder directly onto the window to restore your entire session.");
                        ImGui::Spacing();

                        ImGui::SetNextItemWidth(450);
                        ImGui::InputTextWithHint("##BundlePathInput", "Bundle directory path (e.g. /tmp/my_track.aethel)",
                                                 bundle_dir_path, sizeof(bundle_dir_path));
                        ImGui::SameLine();

                        if (ImGui::Button("  SAVE .AETHEL BUNDLE  ", ImVec2(180, 26))) {
                            std::vector<std::shared_ptr<sampling::AudioClip>> active_clips;
                            for (int t = 0; t < 4; ++t) {
                                if (track_clips[t]) active_clips.push_back(track_clips[t]);
                            }
                            if (sampling::ProjectBundleManager::save_bundle(bundle_dir_path, mixer, mixer.clock(),
                                                                           active_clips, "Aethel Desk Session",
                                                                           &mod_matrix, &midi_learn)) {
                                session_status_msg = "Saved Bundle: " + std::string(bundle_dir_path);
                                std::snprintf(status_toast, sizeof(status_toast),
                                              "[BUNDLE SAVED] %s | %zu Clips Packaged",
                                              bundle_dir_path, active_clips.size());
                            } else {
                                session_status_msg = "Error saving bundle to: " + std::string(bundle_dir_path);
                            }
                            session_status_time = std::chrono::steady_clock::now();
                        }
                        ImGui::SameLine();

                        if (ImGui::Button("  LOAD .AETHEL BUNDLE  ", ImVec2(180, 26))) {
                            std::vector<std::shared_ptr<sampling::AudioClip>> loaded_clips;
                            if (sampling::ProjectBundleManager::load_bundle(bundle_dir_path, mixer, mixer.clock(),
                                                                           loaded_clips, &mod_matrix, &midi_learn)) {
                                for (size_t t = 0; t < std::min<size_t>(4, loaded_clips.size()); ++t) {
                                    track_clips_orig[t] = loaded_clips[t];
                                    sync_track_clip(static_cast<int>(t), loaded_clips[t]);
                                }
                                sync_ui_from_mixer();
                                session_status_msg = "Loaded Bundle: " + std::string(bundle_dir_path);
                                std::snprintf(status_toast, sizeof(status_toast),
                                              "[BUNDLE LOADED] %s | %zu Clips Linked | BPM: %.1f",
                                              bundle_dir_path, loaded_clips.size(), mixer.clock().bpm());
                            } else {
                                session_status_msg = "Error loading bundle from: " + std::string(bundle_dir_path);
                            }
                            session_status_time = std::chrono::steady_clock::now();
                        }

                        ImGui::SameLine();
                        if (ImGui::Button("CLEAR POOL", ImVec2(100, 26))) {
                            sampling::SampleAssetPool::instance().clear();
                            // Re-register active tracks
                            for (int t = 0; t < 4; ++t) {
                                if (track_clips[t]) {
                                    sampling::SampleAssetPool::instance().register_asset(track_clips[t], track_clips[t]->name());
                                }
                            }
                        }
                    }
                    ImGui::EndChild();

                    ImGui::Spacing();

                    // BOTTOM SECTION: Sononym Perceptual Sample Asset Table
                    ImGui::BeginChild("SononymAssetPoolPane", ImVec2(0, 0), true);
                    {
                        ImGui::TextColored(ImVec4(0.85f, 0.45f, 0.05f, 1.0f),
                                           "SONONYM PERCEPTUAL ASSET LIBRARY (%zu SAMPLES REGISTERED)",
                                           sampling::SampleAssetPool::instance().size());
                        ImGui::SameLine();
                        ImGui::TextDisabled("| FNV-1a Content-Addressable Fingerprints & Pitch/Dynamics Extraction");
                        ImGui::Separator();

                        // Search & Filter Bar
                        ImGui::SetNextItemWidth(260);
                        ImGui::InputTextWithHint("##AssetSearch", "Search by name, key, or tag...",
                                                 asset_search_filter, sizeof(asset_search_filter));
                        ImGui::SameLine();

                        const char* cat_filters[] = { "All Categories", "Percussive / Drums", "Bass / Sub", "Melodic / Hooks", "Full Stems" };
                        ImGui::SetNextItemWidth(170);
                        ImGui::Combo("##CatFilter", &asset_cat_filter, cat_filters, 5);

                        ImGui::SameLine(0, 20);
                        ImGui::TextColored(ImVec4(0.20f, 0.70f, 0.90f, 1.0f), "[DRAG & DROP READY]");
                        ImGui::SameLine();
                        ImGui::TextDisabled("Drop multiple .wav files anywhere to auto-analyze and register into this pool");

                        // Asset Table
                        static ImGuiTableFlags tbl_flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                                           ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY;
                        if (ImGui::BeginTable("SononymTable", 9, tbl_flags, ImVec2(0, -1))) {
                            ImGui::TableSetupColumn("Asset ID (FNV-1a)", ImGuiTableColumnFlags_WidthFixed, 100.0f);
                            ImGui::TableSetupColumn("Sample Name", ImGuiTableColumnFlags_WidthStretch, 1.4f);
                            ImGui::TableSetupColumn("Key (F0 Pitch)", ImGuiTableColumnFlags_WidthFixed, 110.0f);
                            ImGui::TableSetupColumn("Loop BPM", ImGuiTableColumnFlags_WidthFixed, 75.0f);
                            ImGui::TableSetupColumn("RMS / Peak", ImGuiTableColumnFlags_WidthFixed, 115.0f);
                            ImGui::TableSetupColumn("Crest Factor", ImGuiTableColumnFlags_WidthFixed, 85.0f);
                            ImGui::TableSetupColumn("Perceptual Category", ImGuiTableColumnFlags_WidthStretch, 1.2f);
                            ImGui::TableSetupColumn("Route to Track", ImGuiTableColumnFlags_WidthFixed, 140.0f);
                            ImGui::TableSetupColumn("Inspect", ImGuiTableColumnFlags_WidthFixed, 65.0f);
                            ImGui::TableHeadersRow();

                            auto descs = sampling::SampleAssetPool::instance().all_descriptors();
                            for (size_t row = 0; row < descs.size(); ++row) {
                                const auto& desc = descs[row];

                                // Filtering
                                if (std::strlen(asset_search_filter) > 0) {
                                    std::string query = asset_search_filter;
                                    std::string fname = desc.file_name;
                                    std::string mkey = desc.musical_key;
                                    std::string cat = desc.perceptual_category;
                                    auto to_lower = [](std::string s) {
                                        std::transform(s.begin(), s.end(), s.begin(), ::tolower);
                                        return s;
                                    };
                                    query = to_lower(query);
                                    if (to_lower(fname).find(query) == std::string::npos &&
                                        to_lower(mkey).find(query) == std::string::npos &&
                                        to_lower(cat).find(query) == std::string::npos) {
                                        continue;
                                    }
                                }

                                if (asset_cat_filter == 1 && desc.perceptual_category.find("Percussive") == std::string::npos && desc.perceptual_category.find("Snare") == std::string::npos) continue;
                                if (asset_cat_filter == 2 && desc.perceptual_category.find("Bass") == std::string::npos && desc.perceptual_category.find("Sub") == std::string::npos && desc.perceptual_category.find("Kick") == std::string::npos) continue;
                                if (asset_cat_filter == 3 && desc.perceptual_category.find("Melodic") == std::string::npos && desc.perceptual_category.find("Pad") == std::string::npos && desc.perceptual_category.find("Vocal") == std::string::npos) continue;
                                if (asset_cat_filter == 4 && desc.perceptual_category.find("Stem") == std::string::npos) continue;

                                ImGui::TableNextRow();
                                ImGui::PushID(static_cast<int>(row));

                                // Col 0: Asset ID preview (first 8 chars)
                                ImGui::TableSetColumnIndex(0);
                                ImGui::TextColored(ImVec4(0.55f, 0.60f, 0.70f, 1.0f), "%.8s...", desc.asset_id.c_str());

                                // Col 1: Sample Name
                                ImGui::TableSetColumnIndex(1);
                                ImGui::TextUnformatted(desc.file_name.c_str());

                                // Col 2: Musical Key & F0
                                ImGui::TableSetColumnIndex(2);
                                if (!desc.musical_key.empty() && desc.musical_key != "--") {
                                    ImGui::TextColored(ImVec4(0.12f, 0.45f, 0.90f, 1.0f), "%s (%.1f Hz)",
                                                       desc.musical_key.c_str(), desc.fundamental_hz);
                                } else {
                                    ImGui::TextDisabled("Percussive (--)");
                                }

                                // Col 3: BPM
                                ImGui::TableSetColumnIndex(3);
                                if (desc.estimated_bpm > 10.0f) {
                                    ImGui::Text("%.1f", desc.estimated_bpm);
                                } else {
                                    ImGui::TextDisabled("--");
                                }

                                // Col 4: RMS / Peak dBFS
                                ImGui::TableSetColumnIndex(4);
                                ImGui::Text("%.1f / %.1f dB", desc.rms_db, desc.peak_db);

                                // Col 5: Crest Factor
                                ImGui::TableSetColumnIndex(5);
                                ImGui::TextColored(desc.crest_factor_db > 12.0f ? ImVec4(0.85f, 0.45f, 0.05f, 1.0f) : ImVec4(0.35f, 0.40f, 0.50f, 1.0f),
                                                   "%.1f dB", desc.crest_factor_db);

                                // Col 6: Perceptual Category
                                ImGui::TableSetColumnIndex(6);
                                ImGui::TextUnformatted(desc.perceptual_category.c_str());

                                // Col 7: Route to Track 1..4
                                ImGui::TableSetColumnIndex(7);
                                for (int tr = 0; tr < 4; ++tr) {
                                    char btn_lbl[32];
                                    std::snprintf(btn_lbl, sizeof(btn_lbl), "T%d##%zu_%d", tr + 1, row, tr);
                                    if (ImGui::SmallButton(btn_lbl)) {
                                        auto clip = sampling::SampleAssetPool::instance().find_by_id(desc.asset_id);
                                        if (clip) {
                                            track_clips_orig[tr] = clip;
                                            sync_track_clip(tr, clip);
                                            std::snprintf(status_toast, sizeof(status_toast),
                                                          "[ROUTED] Assigned \"%s\" to Track %d",
                                                          desc.file_name.c_str(), tr + 1);
                                        }
                                    }
                                    if (tr < 3) ImGui::SameLine();
                                }

                                // Col 8: Inspect
                                ImGui::TableSetColumnIndex(8);
                                if (ImGui::SmallButton("View##insp")) {
                                    auto clip = sampling::SampleAssetPool::instance().find_by_id(desc.asset_id);
                                    if (clip) {
                                        selected_track = 0;
                                        track_clips_orig[0] = clip;
                                        sync_track_clip(0, clip);
                                    }
                                }

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
        ImGui::EndChild();

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

        if (open_save_bundle_modal) {
            ImGui::OpenPopup("Save Self-Contained Project Bundle");
            open_save_bundle_modal = false;
        }
        if (ImGui::BeginPopupModal("Save Self-Contained Project Bundle", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextColored(ImVec4(0.12f, 0.45f, 0.90f, 1.0f), "SAVE SELF-CONTAINED PROJECT BUNDLE (.AETHEL)");
            ImGui::Separator();
            ImGui::Spacing();
            ImGui::TextWrapped("Packages session routing, automation, plugins, and writes all audio clips into 'assets/' with content-hashed filenames.");
            ImGui::Spacing();
            ImGui::InputText("Bundle Directory", bundle_dir_path, sizeof(bundle_dir_path));
            ImGui::Spacing();

            if (ImGui::Button("SAVE BUNDLE", ImVec2(130, 30))) {
                std::vector<std::shared_ptr<sampling::AudioClip>> active_clips;
                for (int t = 0; t < 4; ++t) {
                    if (track_clips[t]) active_clips.push_back(track_clips[t]);
                }
                if (sampling::ProjectBundleManager::save_bundle(bundle_dir_path, mixer, mixer.clock(), active_clips, "Aethel Desk Session", &mod_matrix, &midi_learn)) {
                    session_status_msg = "Saved Bundle: " + std::string(bundle_dir_path);
                    std::snprintf(status_toast, sizeof(status_toast), "[BUNDLE SAVED] %s (%zu Clips)", bundle_dir_path, active_clips.size());
                } else {
                    session_status_msg = "Error saving bundle: " + std::string(bundle_dir_path);
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

        if (open_load_bundle_modal) {
            ImGui::OpenPopup("Load Self-Contained Project Bundle");
            open_load_bundle_modal = false;
        }
        if (ImGui::BeginPopupModal("Load Self-Contained Project Bundle", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextColored(ImVec4(0.12f, 0.45f, 0.90f, 1.0f), "LOAD SELF-CONTAINED PROJECT BUNDLE (.AETHEL)");
            ImGui::Separator();
            ImGui::Spacing();
            ImGui::TextWrapped("Select an .aethel bundle directory to restore session topology, plugins, and re-link all WAV assets.");
            ImGui::Spacing();
            ImGui::InputText("Bundle Directory", bundle_dir_path, sizeof(bundle_dir_path));
            ImGui::Spacing();

            if (ImGui::Button("LOAD BUNDLE", ImVec2(130, 30))) {
                std::vector<std::shared_ptr<sampling::AudioClip>> loaded_clips;
                if (sampling::ProjectBundleManager::load_bundle(bundle_dir_path, mixer, mixer.clock(), loaded_clips, &mod_matrix, &midi_learn)) {
                    for (size_t t = 0; t < std::min<size_t>(4, loaded_clips.size()); ++t) {
                        track_clips_orig[t] = loaded_clips[t];
                        sync_track_clip(static_cast<int>(t), loaded_clips[t]);
                    }
                    sync_ui_from_mixer();
                    session_status_msg = "Loaded Bundle: " + std::string(bundle_dir_path);
                    std::snprintf(status_toast, sizeof(status_toast), "[BUNDLE LOADED] %s (%zu Clips Linked)", bundle_dir_path, loaded_clips.size());
                } else {
                    session_status_msg = "Error loading bundle: " + std::string(bundle_dir_path);
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
