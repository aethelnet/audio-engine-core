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

// Mock audio clip sample buffer for the sample editor / waveform slicer
static std::vector<float> generate_synthetic_drum_loop(size_t num_samples) {
    std::vector<float> buf(num_samples, 0.0f);
    // Generate 4 bars of transients (kick, snare, hi-hats)
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
    constexpr uint32_t kBlockFrames = 256;
    constexpr uint32_t kSampleRate = 48000;
    MixerGraph mixer(kBlockFrames);

    auto* trk0 = mixer.allocate_track("Kick / 808 Sub");
    auto* trk1 = mixer.allocate_track("Acid 303 Lead");
    auto* trk2 = mixer.allocate_track("Vocal Chops");
    auto* trk3 = mixer.allocate_track("Drums / Bus");
    (void)trk2;
    (void)trk3;


    // Pre-insert standard Airwindows DSPs
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

    auto master_clipper = std::make_shared<dsp::ClipOnly2>();
    master_clipper->init(kSampleRate);
    mixer.master_bus().slot(0).set_processor(master_clipper);

    // Audio test loop waveform data (4 seconds @ 48kHz)
    std::vector<float> sample_waveform = generate_synthetic_drum_loop(48000 * 4);
    std::vector<float> slice_points = { 0.0f, 0.125f, 0.25f, 0.375f, 0.5f, 0.625f, 0.75f, 0.875f };

    // Workspace UI State
    bool is_playing = false;
    float bpm = 126.0f;
    float playhead_seconds = 0.0f;
    const float loop_length_seconds = 8.0f; // 4 bars at 120bpm
    int selected_track = 0;
    int active_slice = 0;

    // Track UI state caches
    float track_gains[4] = { 0.85f, 0.70f, 0.80f, 0.90f };
    float track_pans[4] = { 0.0f, -0.25f, 0.30f, 0.0f };
    bool track_mutes[4] = { false, false, false, false };
    bool track_solos[4] = { false, false, false, false };
    bool track_solo_safes[4] = { false, false, false, false };
    int track_consoles[4] = { 1, 2, 3, 0 }; // Warm, Lush, etc.

    // Sample Editor State
    float sample_pitch_shift = 0.0f; // Semitones
    bool sample_reverse = false;
    bool sample_choke = true;

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
            playhead_seconds += dt * (bpm / 120.0f);
            if (playhead_seconds >= loop_length_seconds) {
                playhead_seconds = std::fmod(playhead_seconds, loop_length_seconds);
            }
        }

        // Lock-free telemetry query
        mixer.capture_telemetry_snapshot(telemetry);

        // Synthesize visual meters for active animation if playing
        if (is_playing) {
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
            }
            ImGui::SameLine();
            if (ImGui::Button("[ [] STOP ]", ImVec2(80, 32))) {
                is_playing = false;
                playhead_seconds = 0.0f;
            }

            ImGui::SameLine();
            ImGui::SetNextItemWidth(100);
            if (ImGui::SliderFloat("BPM", &bpm, 60.0f, 200.0f, "%.1f")) {
                // sample-accurate clock tempo adjustment
            }

            ImGui::SameLine(0, 20);
            ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f), "[PIPEWIRE RT]");
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.40f, 0.45f, 0.52f, 1.0f), "| 48.0 kHz | 256s (5.3ms) | CPU: 3.8%%");

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
            static float master_gain = 0.90f;
            ImGui::SetNextItemWidth(120);
            if (ImGui::SliderFloat("Vol##Master", &master_gain, 0.0f, 1.25f, "%.2f")) {
                protocol::MixerCommand cmd{};
                cmd.type = protocol::MixerCommandType::SetMasterGain;
                cmd.value1 = master_gain;
                mixer.post_command(cmd);
            }

            ImGui::SameLine(0, 15);
            static bool master_limiter = true;
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
                // TAB A: TIMELINE & BAR GRID
                // ------------------------------------------------------------
                if (ImGui::BeginTabItem("  TIMELINE / ARRANGER & BAR GRID  ")) {
                    ImDrawList* draw_list = ImGui::GetWindowDrawList();
                    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
                    ImVec2 canvas_size = ImGui::GetContentRegionAvail();
                    canvas_size.y = std::max(canvas_size.y - 4.0f, 120.0f);

                    // Draw Timeline Background (Pure Vellum White with Graphite Border)
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

                    // 4 Track Lanes
                    const float lane_h = (canvas_size.y - 20.0f) / 4.0f;
                    const char* track_names[4] = { "Track 1: Kick & 808", "Track 2: Acid 303 Lead", "Track 3: Vocal Slices", "Track 4: Drum Bus" };
                    for (int t = 0; t < 4; ++t) {
                        float ly = canvas_pos.y + 20.0f + t * lane_h;
                        draw_list->AddLine(ImVec2(canvas_pos.x, ly),
                                           ImVec2(canvas_pos.x + canvas_size.x, ly),
                                           ImColor(230, 235, 242, 255), 1.0f);

                        // Highlight selected lane (Subtle blueprint tint)
                        if (selected_track == t) {
                            draw_list->AddRectFilled(ImVec2(canvas_pos.x, ly),
                                                     ImVec2(canvas_pos.x + canvas_size.x, ly + lane_h),
                                                     ImColor(31, 97, 217, 18));
                        }

                        // Clip Blocks (Blueprint Blue Vellum Cards)
                        float clip_x1 = canvas_pos.x + (t * 2.0f) * bar_w;
                        float clip_x2 = clip_x1 + (4.0f) * bar_w;
                        draw_list->AddRectFilled(ImVec2(clip_x1 + 2.0f, ly + 4.0f),
                                                 ImVec2(clip_x2 - 2.0f, ly + lane_h - 4.0f),
                                                 (selected_track == t) ? ImColor(215, 230, 255, 240) : ImColor(235, 242, 255, 220),
                                                 2.0f);
                        draw_list->AddRect(ImVec2(clip_x1 + 2.0f, ly + 4.0f),
                                           ImVec2(clip_x2 - 2.0f, ly + lane_h - 4.0f),
                                           ImColor(31, 97, 217, 220), 2.0f);
                        draw_list->AddText(ImVec2(clip_x1 + 8.0f, ly + 8.0f),
                                           ImColor(15, 30, 70, 255), track_names[t]);
                    }

                    // Precision Playhead Needle (Drafting Black)
                    float play_ratio = playhead_seconds / loop_length_seconds;
                    float playhead_x = canvas_pos.x + play_ratio * canvas_size.x;
                    draw_list->AddLine(ImVec2(playhead_x, canvas_pos.y),
                                       ImVec2(playhead_x, canvas_pos.y + canvas_size.y),
                                       ImColor(20, 25, 35, 255), 2.0f);
                    draw_list->AddTriangleFilled(ImVec2(playhead_x - 6.0f, canvas_pos.y),
                                                 ImVec2(playhead_x + 6.0f, canvas_pos.y),
                                                 ImVec2(playhead_x, canvas_pos.y + 10.0f),
                                                 ImColor(20, 25, 35, 255));


                    // Click detection to select tracks
                    ImGui::InvisibleButton("TimelineCanvasBtn", canvas_size);
                    if (ImGui::IsItemClicked()) {
                        ImVec2 m = ImGui::GetIO().MousePos;
                        if (m.y >= canvas_pos.y + 20.0f) {
                            int clicked_trk = static_cast<int>((m.y - (canvas_pos.y + 20.0f)) / lane_h);
                            if (clicked_trk >= 0 && clicked_trk < 4) {
                                selected_track = clicked_trk;
                            }
                        }
                    }

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

                            // Console saturation selector
                            ImGui::SetNextItemWidth(120);
                            if (ImGui::Combo("Console", &track_consoles[t], console_types, 4)) {
                                protocol::MixerCommand cmd{};
                                cmd.type = protocol::MixerCommandType::SetTrackConsoleType;
                                cmd.target_id = t;
                                cmd.flags = track_consoles[t];
                                mixer.post_command(cmd);
                            }

                            // 4 Insert Slots
                            ImGui::TextColored(ImVec4(0.35f, 0.40f, 0.48f, 1.0f), "Insert Slots (4x):");
                            const char* slot_labels[4] = {

                                (t == 0) ? "[Baxandall EQ]" : ((t == 1) ? "[PurestDrive]" : "[Empty]"),
                                (t == 0) ? "[ButterComp2]" : ((t == 1) ? "[WASM Sat]" : "[Empty]"),
                                "[Empty]",
                                "[Empty]"
                            };
                            for (int s = 0; s < 4; ++s) {
                                ImGui::PushID(s);
                                ImGui::TextDisabled("S%d:", s + 1);
                                ImGui::SameLine();
                                ImGui::Button(slot_labels[s], ImVec2(100, 18));
                                ImGui::SameLine();
                                static bool bypass[4][4] = {};
                                ImGui::Checkbox("By", &bypass[t][s]);
                                ImGui::PopID();
                            }

                            ImGui::Separator();

                            // Pan Rotary Dial
                            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 40.0f);
                            if (ui::DrawRotaryKnob("Pan", &track_pans[t], -1.0f, 1.0f, "", 18.0f)) {
                                protocol::MixerCommand cmd{};
                                cmd.type = protocol::MixerCommandType::SetTrackPan;
                                cmd.target_id = t;
                                cmd.value1 = track_pans[t];
                                mixer.post_command(cmd);
                            }

                            // Mute, Solo, Solo-Safe Buttons
                            if (ImGui::Checkbox("M", &track_mutes[t])) {
                                protocol::MixerCommand cmd{};
                                cmd.type = protocol::MixerCommandType::SetTrackMute;
                                cmd.target_id = t;
                                cmd.flags = track_mutes[t] ? 1 : 0;
                                mixer.post_command(cmd);
                            }
                            ImGui::SameLine();
                            if (ImGui::Checkbox("S", &track_solos[t])) {
                                protocol::MixerCommand cmd{};
                                cmd.type = protocol::MixerCommandType::SetTrackSolo;
                                cmd.target_id = t;
                                cmd.flags = track_solos[t] ? 1 : 0;
                                mixer.post_command(cmd);
                            }
                            ImGui::SameLine();
                            if (ImGui::Checkbox("SS", &track_solo_safes[t])) {
                                protocol::MixerCommand cmd{};
                                cmd.type = protocol::MixerCommandType::SetTrackSoloSafe;
                                cmd.target_id = t;
                                cmd.flags = track_solo_safes[t] ? 1 : 0;
                                mixer.post_command(cmd);
                            }

                            // Vertical Fader & Meter Bridge
                            ImGui::VSliderFloat("##fader", ImVec2(34, 110), &track_gains[t], 0.0f, 1.25f, "");

                            if (ImGui::IsItemEdited()) {
                                protocol::MixerCommand cmd{};
                                cmd.type = protocol::MixerCommandType::SetTrackGain;
                                cmd.target_id = t;
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
                    ImGui::EndTabItem();
                }

                // ------------------------------------------------------------
                // TAB 2: SAMPLE EDITOR / SLICER
                // ------------------------------------------------------------
                if (ImGui::BeginTabItem("  SAMPLE EDITOR / SLICER  ")) {
                    ImGui::TextColored(ImVec4(0.12f, 0.38f, 0.85f, 1.0f),
                                       "Transient Slicer & Hermite Resampler [Selected Track: %d]", selected_track + 1);
                    ImGui::Separator();

                    // Waveform Display
                    ImVec2 wf_pos = ImGui::GetCursorScreenPos();
                    ImVec2 wf_size(ImGui::GetContentRegionAvail().x, 150);
                    float play_ratio = playhead_seconds / loop_length_seconds;
                    ui::DrawWaveformDisplay(ImGui::GetWindowDrawList(), wf_pos, wf_size,
                                           sample_waveform.data(), sample_waveform.size(),
                                           play_ratio, slice_points, active_slice);
                    ImGui::Dummy(wf_size);

                    // Slicer Tools
                    ImGui::Spacing();
                    ImGui::SetNextItemWidth(180);
                    ImGui::SliderFloat("Pitch Shift (Semitones)", &sample_pitch_shift, -24.0f, 24.0f, "%.1f st");
                    ImGui::SameLine(0, 20);
                    ImGui::Checkbox("Reverse", &sample_reverse);
                    ImGui::SameLine(0, 20);
                    ImGui::Checkbox("Micro-Fade Choke (Anti-Click)", &sample_choke);
                    ImGui::SameLine(0, 30);
                    if (ImGui::Button("Detect Transients")) {
                        // Re-run transient detector
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Export Slices to Pads")) {
                        // Slices mapped to sampler
                    }

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
