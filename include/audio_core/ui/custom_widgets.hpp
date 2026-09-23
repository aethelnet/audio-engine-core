#pragma once

#include "imgui.h"
#include "imgui_internal.h"
#include "audio_core/protocol/telemetry_packet.hpp"
#include "audio_core/sampling/waveform_overview.hpp"
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <unordered_map>
#include <cstdio>

namespace audio_core::ui {

inline float linear_to_db(float lin) noexcept {
    if (lin <= 0.00001f) return -96.0f;
    return 20.0f * std::log10(lin);
}

inline float db_to_linear(float db) noexcept {
    if (db <= -96.0f) return 0.0f;
    return std::pow(10.0f, db / 20.0f);
}

// Map dB [-60 dB .. +6 dB] to normalized [0.0 .. 1.0]
inline float db_to_normalized(float db) noexcept {
    constexpr float kMinDb = -60.0f;
    constexpr float kMaxDb = 6.0f;
    if (db <= kMinDb) return 0.0f;
    if (db >= kMaxDb) return 1.0f;
    return (db - kMinDb) / (kMaxDb - kMinDb);
}

// ============================================================================
// Orderly Architect's Desk: Vertical Stereo/Mono dB Peak & RMS Meter
// Recessed Aluminum Slot, Blueprint Cobalt, Drafting Amber, Crimson Clip LED
// ============================================================================
inline void DrawDbMeter(ImDrawList* draw_list, ImVec2 pos, ImVec2 size,
                        float peak_l, float peak_r, float rms_l, float rms_r,
                        bool is_clipping, const char* label = nullptr) {
    const float x = pos.x;
    const float y = pos.y;
    const float w = size.x;
    const float h = size.y;

    // 1. Recessed Aluminum Meter Well
    const ImU32 col_bg = ImColor(232, 235, 240, 255);       // Pale matte gray slot
    const ImU32 col_border = ImColor(190, 196, 206, 255);   // Graphite pencil line
    draw_list->AddRectFilled(pos, ImVec2(x + w, y + h), col_bg, 2.0f);
    draw_list->AddRect(pos, ImVec2(x + w, y + h), col_border, 2.0f);

    // 2. Channel Split: Left and Right bars
    const float bar_w = (w - 6.0f) * 0.5f;
    const float clip_h = 6.0f;
    const float meter_h = h - clip_h - 4.0f;
    const float meter_y_bottom = y + h - 2.0f;

    // dB values
    const float db_peak_l = linear_to_db(peak_l);
    const float db_peak_r = linear_to_db(peak_r);
    const float db_rms_l  = linear_to_db(rms_l);
    const float db_rms_r  = linear_to_db(rms_r);

    const float norm_peak_l = db_to_normalized(db_peak_l);
    const float norm_peak_r = db_to_normalized(db_peak_r);
    const float norm_rms_l  = db_to_normalized(db_rms_l);
    const float norm_rms_r  = db_to_normalized(db_rms_r);

    auto draw_single_bar = [&](float bar_x, float norm_rms, float norm_peak) {
        if (norm_rms > 0.001f) {
            float bar_top = meter_y_bottom - (norm_rms * meter_h);
            ImU32 col_rms = (norm_rms > 0.909f) ? ImColor(220, 38, 38, 240) :   // Crimson > 0dB
                            (norm_rms > 0.727f) ? ImColor(217, 119, 6, 240) :   // Drafting Amber > -12dB
                                                  ImColor(31, 97, 217, 240);    // Blueprint Cobalt
            draw_list->AddRectFilled(ImVec2(bar_x, bar_top), ImVec2(bar_x + bar_w, meter_y_bottom), col_rms, 1.0f);
        }

        if (norm_peak > 0.001f) {
            float peak_y = meter_y_bottom - (norm_peak * meter_h);
            ImU32 col_pk = (norm_peak >= 0.909f) ? ImColor(180, 20, 20, 255) :
                           (norm_peak > 0.727f)  ? ImColor(180, 90, 5, 255) :
                                                   ImColor(15, 45, 120, 255);
            draw_list->AddLine(ImVec2(bar_x, peak_y), ImVec2(bar_x + bar_w, peak_y), col_pk, 2.0f);
        }
    };

    draw_single_bar(x + 2.0f, norm_rms_l, norm_peak_l);
    draw_single_bar(x + 4.0f + bar_w, norm_rms_r, norm_peak_r);

    // 3. Clip LED indicators at the top
    const ImU32 col_clip_l = (peak_l >= 1.0f || is_clipping) ? ImColor(220, 38, 38, 255) : ImColor(210, 215, 222, 200);
    const ImU32 col_clip_r = (peak_r >= 1.0f || is_clipping) ? ImColor(220, 38, 38, 255) : ImColor(210, 215, 222, 200);
    draw_list->AddRectFilled(ImVec2(x + 2.0f, y + 2.0f), ImVec2(x + 2.0f + bar_w, y + 2.0f + clip_h), col_clip_l, 1.0f);
    draw_list->AddRectFilled(ImVec2(x + 4.0f + bar_w, y + 2.0f), ImVec2(x + w - 2.0f, y + 2.0f + clip_h), col_clip_r, 1.0f);

    // 4. 0 dB / -12 dB ruler ticks
    const float y_0db = meter_y_bottom - (db_to_normalized(0.0f) * meter_h);
    const float y_12db = meter_y_bottom - (db_to_normalized(-12.0f) * meter_h);
    draw_list->AddLine(ImVec2(x, y_0db), ImVec2(x + w, y_0db), ImColor(220, 38, 38, 180), 1.0f);
    draw_list->AddLine(ImVec2(x, y_12db), ImVec2(x + w, y_12db), ImColor(150, 160, 175, 160), 1.0f);

    if (label && label[0] != '\0') {
        draw_list->AddText(ImVec2(x + 2.0f, y + h + 2.0f), ImColor(80, 90, 105, 255), label);
    }
}

// ============================================================================
// Orderly Architect's Desk: Precision Machined Aluminum / Ceramic Dial Knob
// White/Alabaster Face, Blueprint Blue Indicator Arc, Deep Drafting Ink Pointer
// ============================================================================
inline bool DrawRotaryKnob(const char* label, float* value, float v_min, float v_max,
                           const char* unit = "", float radius = 20.0f) {
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    if (window->SkipItems) return false;

    ImGuiContext& g = *GImGui;
    const ImGuiStyle& style = g.Style;
    const ImGuiID id = window->GetID(label);

    const ImVec2 label_size = ImGui::CalcTextSize(label, nullptr, true);
    const ImVec2 size(radius * 2.0f, radius * 2.0f + label_size.y + 4.0f);
    const ImRect bb(window->DC.CursorPos, ImVec2(window->DC.CursorPos.x + size.x, window->DC.CursorPos.y + size.y));

    ImGui::ItemSize(bb, style.FramePadding.y);
    if (!ImGui::ItemAdd(bb, id)) return false;

    bool hovered, held;
    ImGui::ButtonBehavior(bb, id, &hovered, &held);

    bool changed = false;
    if (held) {
        const float drag_speed = 0.005f * (v_max - v_min);
        float delta = -g.IO.MouseDelta.y * drag_speed;
        if (g.IO.KeyShift) delta *= 0.2f;
        if (delta != 0.0f) {
            *value = std::clamp(*value + delta, v_min, v_max);
            changed = true;
        }
    }

    const ImVec2 center = ImVec2(bb.Min.x + radius, bb.Min.y + radius);
    ImDrawList* draw_list = window->DrawList;

    const float norm = std::clamp((*value - v_min) / (v_max - v_min), 0.0f, 1.0f);
    constexpr float kAngleMin = -3.14159265f * 0.75f; // -135 deg
    constexpr float kAngleMax =  3.14159265f * 0.75f; // +135 deg
    const float current_angle = kAngleMin + norm * (kAngleMax - kAngleMin);

    // Clean matte ceramic dial face
    const ImU32 col_bg = hovered ? ImColor(245, 247, 250, 255) : ImColor(255, 255, 255, 255);
    const ImU32 col_border = held ? ImColor(31, 97, 217, 255) : (hovered ? ImColor(100, 130, 180, 220) : ImColor(190, 196, 206, 255));
    draw_list->AddCircleFilled(center, radius, col_bg, 32);
    draw_list->AddCircle(center, radius, col_border, 32, 1.5f);

    // Active blueprint blue arc
    constexpr int kArcSegments = 24;
    for (int i = 0; i < kArcSegments; ++i) {
        float a1 = kAngleMin + (float)i / kArcSegments * (current_angle - kAngleMin);
        float a2 = kAngleMin + (float)(i + 1) / kArcSegments * (current_angle - kAngleMin);
        if (a1 >= current_angle) break;
        if (a2 > current_angle) a2 = current_angle;

        ImVec2 p1(center.x + std::sin(a1) * (radius - 3.0f), center.y - std::cos(a1) * (radius - 3.0f));
        ImVec2 p2(center.x + std::sin(a2) * (radius - 3.0f), center.y - std::cos(a2) * (radius - 3.0f));
        draw_list->AddLine(p1, p2, ImColor(31, 97, 217, 240), 2.5f);
    }

    // Pointer line (Drafting ink)
    const ImVec2 pointer_end(center.x + std::sin(current_angle) * (radius - 2.0f),
                             center.y - std::cos(current_angle) * (radius - 2.0f));
    const ImVec2 pointer_start(center.x + std::sin(current_angle) * (radius * 0.35f),
                               center.y - std::cos(current_angle) * (radius * 0.35f));
    draw_list->AddLine(pointer_start, pointer_end, ImColor(25, 30, 40, 255), 2.0f);

    // Clean technical value below
    char val_buf[32];
    std::snprintf(val_buf, sizeof(val_buf), "%.1f%s", *value, unit);
    ImVec2 val_size = ImGui::CalcTextSize(val_buf);
    draw_list->AddText(ImVec2(center.x - val_size.x * 0.5f, bb.Min.y + radius * 2.0f + 2.0f),
                       hovered ? ImColor(31, 97, 217, 255) : ImColor(80, 90, 105, 255), val_buf);

    return changed;
}

// ============================================================================
// Orderly Architect's Desk: Technical Waveform & Slicer Display
// Crisp Vellum Canvas, Blueprint Ink Columns, Millimeter Guideline Axis
// ============================================================================
inline void DrawWaveformDisplay(ImDrawList* draw_list, ImVec2 pos, ImVec2 size,
                                const float* samples, size_t sample_count,
                                float playhead_ratio,
                                const std::vector<float>& slice_points_ratio,
                                int active_slice_idx = -1) {
    const float x = pos.x;
    const float y = pos.y;
    const float w = size.x;
    const float h = size.y;
    const float mid_y = y + h * 0.5f;

    // 1. Pure White Vellum Background Canvas
    draw_list->AddRectFilled(pos, ImVec2(x + w, y + h), ImColor(255, 255, 255, 255), 2.0f);
    draw_list->AddRect(pos, ImVec2(x + w, y + h), ImColor(190, 196, 206, 255), 2.0f);

    // Subtle technical grid lines (Drafting millimeter paper)
    for (float gx = x + 40.0f; gx < x + w; gx += 40.0f) {
        draw_list->AddLine(ImVec2(gx, y), ImVec2(gx, y + h), ImColor(240, 243, 248, 255), 1.0f);
    }

    // Center zero graphite line
    draw_list->AddLine(ImVec2(x, mid_y), ImVec2(x + w, mid_y), ImColor(200, 208, 220, 255), 1.0f);

    if (!samples || sample_count == 0) {
        const char* msg = "[ NO AUDIO SAMPLE LOADED ]";
        ImVec2 msg_size = ImGui::CalcTextSize(msg);
        draw_list->AddText(ImVec2(x + (w - msg_size.x) * 0.5f, mid_y - msg_size.y * 0.5f),
                           ImColor(130, 140, 155, 255), msg);
        return;
    }

    // 2. Render Peak Columns with Blueprint Cobalt Ink
    const int num_columns = static_cast<int>(w);
    const float samples_per_pixel = static_cast<float>(sample_count) / w;

    for (int px = 0; px < num_columns; ++px) {
        size_t start_s = static_cast<size_t>(px * samples_per_pixel);
        size_t end_s   = static_cast<size_t>((px + 1) * samples_per_pixel);
        if (end_s > sample_count) end_s = sample_count;

        float min_val = 0.0f;
        float max_val = 0.0f;
        for (size_t s = start_s; s < end_s; ++s) {
            float v = samples[s];
            if (v < min_val) min_val = v;
            if (v > max_val) max_val = v;
        }

        float y_top = mid_y - (max_val * (h * 0.46f));
        float y_bot = mid_y - (min_val * (h * 0.46f));
        if (y_bot - y_top < 1.0f) y_bot = y_top + 1.0f;

        // Architectural Blueprint Cobalt wave
        draw_list->AddLine(ImVec2(x + px, y_top), ImVec2(x + px, y_bot), ImColor(31, 97, 217, 230), 1.0f);
    }

    // 3. Render Slicing Markers (Drafting Ochre / Amber)
    for (size_t i = 0; i < slice_points_ratio.size(); ++i) {
        float slice_x = x + (slice_points_ratio[i] * w);
        bool is_active = (static_cast<int>(i) == active_slice_idx);
        ImU32 col_slice = is_active ? ImColor(217, 119, 6, 255) : ImColor(217, 119, 6, 180);

        draw_list->AddLine(ImVec2(slice_x, y), ImVec2(slice_x, y + h), col_slice, is_active ? 2.0f : 1.0f);

        // Marker tag at top
        draw_list->AddTriangleFilled(ImVec2(slice_x - 4.0f, y),
                                     ImVec2(slice_x + 4.0f, y),
                                     ImVec2(slice_x, y + 7.0f), col_slice);
    }

    // 4. Precision Playhead Needle (Drafting Black Needle)
    if (playhead_ratio >= 0.0f && playhead_ratio <= 1.0f) {
        float play_x = x + (playhead_ratio * w);
        draw_list->AddLine(ImVec2(play_x, y), ImVec2(play_x, y + h), ImColor(20, 25, 35, 255), 1.5f);
        draw_list->AddTriangleFilled(ImVec2(play_x - 5.0f, y),
                                     ImVec2(play_x + 5.0f, y),
                                     ImVec2(play_x, y + 8.0f), ImColor(20, 25, 35, 255));
    }
}

// ============================================================================
// Multi-Resolution Waveform Peak Mipmap Display
// O(W) Constant-time peak & RMS rendering for multi-minute stems
// ============================================================================
inline void DrawWaveformDisplay(ImDrawList* draw_list, ImVec2 pos, ImVec2 size,
                                const sampling::WaveformOverview* overview,
                                uint32_t channel,
                                uint64_t start_frame, uint64_t end_frame,
                                float playhead_ratio,
                                const std::vector<float>& slice_points_ratio,
                                int active_slice_idx = -1) {
    const float x = pos.x;
    const float y = pos.y;
    const float w = size.x;
    const float h = size.y;
    const float mid_y = y + h * 0.5f;

    // 1. Pure White Vellum Background Canvas
    draw_list->AddRectFilled(pos, ImVec2(x + w, y + h), ImColor(255, 255, 255, 255), 2.0f);
    draw_list->AddRect(pos, ImVec2(x + w, y + h), ImColor(190, 196, 206, 255), 2.0f);

    // Subtle technical grid lines (Drafting millimeter paper)
    for (float gx = x + 40.0f; gx < x + w; gx += 40.0f) {
        draw_list->AddLine(ImVec2(gx, y), ImVec2(gx, y + h), ImColor(240, 243, 248, 255), 1.0f);
    }

    // Center zero graphite line
    draw_list->AddLine(ImVec2(x, mid_y), ImVec2(x + w, mid_y), ImColor(200, 208, 220, 255), 1.0f);

    if (!overview || overview->total_frames() == 0) {
        const char* msg = "[ NO AUDIO SAMPLE LOADED ]";
        ImVec2 msg_size = ImGui::CalcTextSize(msg);
        draw_list->AddText(ImVec2(x + (w - msg_size.x) * 0.5f, mid_y - msg_size.y * 0.5f),
                           ImColor(130, 140, 155, 255), msg);
        return;
    }

    if (!overview->is_ready()) {
        char msg[64];
        std::snprintf(msg, sizeof(msg), "[ ANALYZING STEM PEAKS: %3.0f%% ]", overview->progress() * 100.0f);
        ImVec2 msg_size = ImGui::CalcTextSize(msg);
        draw_list->AddText(ImVec2(x + (w - msg_size.x) * 0.5f, mid_y - msg_size.y * 0.5f - 8.0f),
                           ImColor(31, 97, 217, 255), msg);

        float bar_w = std::min(w * 0.6f, 200.0f);
        float bar_x = x + (w - bar_w) * 0.5f;
        float bar_y = mid_y + 10.0f;
        draw_list->AddRect(ImVec2(bar_x, bar_y), ImVec2(bar_x + bar_w, bar_y + 6.0f), ImColor(190, 196, 206, 255));
        draw_list->AddRectFilled(ImVec2(bar_x, bar_y), ImVec2(bar_x + bar_w * overview->progress(), bar_y + 6.0f), ImColor(31, 97, 217, 255));
        return;
    }

    // 2. Query O(W) Mipmap Peaks across screen columns
    const int num_columns = static_cast<int>(w);
    if (num_columns <= 0) return;

    static thread_local std::vector<sampling::ViewportPeak> s_peaks;
    overview->query_peaks(channel, start_frame, end_frame, static_cast<size_t>(num_columns), s_peaks);

    // 3. Render Peak Columns with Blueprint Cobalt Ink & RMS Solid Core
    for (int px = 0; px < num_columns; ++px) {
        const auto& pk = s_peaks[px];

        float y_top = mid_y - (pk.max_val * (h * 0.46f));
        float y_bot = mid_y - (pk.min_val * (h * 0.46f));
        if (y_bot - y_top < 1.0f) y_bot = y_top + 1.0f;

        // Architectural Blueprint Cobalt outer peak wave
        draw_list->AddLine(ImVec2(x + px, y_top), ImVec2(x + px, y_bot), ImColor(31, 97, 217, 180), 1.0f);

        // Deep Navy RMS musical energy inner core
        if (pk.rms > 0.001f) {
            float rms_top = mid_y - (pk.rms * (h * 0.46f));
            float rms_bot = mid_y + (pk.rms * (h * 0.46f));
            draw_list->AddLine(ImVec2(x + px, rms_top), ImVec2(x + px, rms_bot), ImColor(15, 45, 110, 240), 1.0f);
        }
    }

    // 4. Render Slicing Markers (Drafting Ochre / Amber)
    for (size_t i = 0; i < slice_points_ratio.size(); ++i) {
        float slice_x = x + (slice_points_ratio[i] * w);
        bool is_active = (static_cast<int>(i) == active_slice_idx);
        ImU32 col_slice = is_active ? ImColor(217, 119, 6, 255) : ImColor(217, 119, 6, 180);

        draw_list->AddLine(ImVec2(slice_x, y), ImVec2(slice_x, y + h), col_slice, is_active ? 2.0f : 1.0f);

        // Marker tag at top
        draw_list->AddTriangleFilled(ImVec2(slice_x - 4.0f, y),
                                     ImVec2(slice_x + 4.0f, y),
                                     ImVec2(slice_x, y + 7.0f), col_slice);
    }

    // 5. Precision Playhead Needle (Drafting Black Needle)
    if (playhead_ratio >= 0.0f && playhead_ratio <= 1.0f) {
        float play_x = x + (playhead_ratio * w);
        draw_list->AddLine(ImVec2(play_x, y), ImVec2(play_x, y + h), ImColor(20, 25, 35, 255), 1.5f);
        draw_list->AddTriangleFilled(ImVec2(play_x - 5.0f, y),
                                     ImVec2(play_x + 5.0f, y),
                                     ImVec2(play_x, y + 8.0f), ImColor(20, 25, 35, 255));
    }
}

// ============================================================================
// Orderly Architect's Desk: Liquid ODE Envelope Curve Plotter
// ============================================================================
inline void DrawEnvelopeCurve(ImDrawList* draw_list, ImVec2 pos, ImVec2 size,
                              float attack, float decay, float sustain, float release, float tau) {
    (void)tau;
    const float x = pos.x;
    const float y = pos.y;
    const float w = size.x;
    const float h = size.y;
    const float bottom_y = y + h - 4.0f;
    const float top_y    = y + 4.0f;
    const float height_usable = bottom_y - top_y;

    draw_list->AddRectFilled(pos, ImVec2(x + w, y + h), ImColor(255, 255, 255, 255), 2.0f);
    draw_list->AddRect(pos, ImVec2(x + w, y + h), ImColor(190, 196, 206, 255), 2.0f);

    const float total_time = attack + decay + 200.0f + release;
    if (total_time <= 0.0f) return;

    const float w_att = (attack / total_time) * w;
    const float w_dec = (decay / total_time) * w;
    const float w_sus = (200.0f / total_time) * w;
    const float w_rel = (release / total_time) * w;

    const float sus_y = bottom_y - (sustain * height_usable);

    ImVec2 p0(x, bottom_y);
    ImVec2 p1(x + w_att, top_y);
    ImVec2 p2(x + w_att + w_dec, sus_y);
    ImVec2 p3(x + w_att + w_dec + w_sus, sus_y);
    ImVec2 p4(x + w_att + w_dec + w_sus + w_rel, bottom_y);

    // Blueprint Cobalt line
    draw_list->AddLine(p0, p1, ImColor(31, 97, 217, 255), 2.0f);
    draw_list->AddLine(p1, p2, ImColor(31, 97, 217, 255), 2.0f);
    draw_list->AddLine(p2, p3, ImColor(217, 119, 6, 255), 2.0f);
    draw_list->AddLine(p3, p4, ImColor(31, 97, 217, 255), 2.0f);

    // Light blueprint water-wash fill under curve
    ImVec2 fill_pts[7] = {
        p0, p1, p2, p3, p4,
        ImVec2(p4.x, bottom_y), ImVec2(p0.x, bottom_y)
    };
    draw_list->AddConvexPolyFilled(fill_pts, 7, ImColor(31, 97, 217, 30));

    // Marker knots (Technical points)
    draw_list->AddCircleFilled(p1, 3.5f, ImColor(25, 30, 40, 255));
    draw_list->AddCircleFilled(p2, 3.5f, ImColor(217, 119, 6, 255));
    draw_list->AddCircleFilled(p3, 3.5f, ImColor(217, 119, 6, 255));
}

// ============================================================================
// Orderly Architect's Desk: Precision Gas Watchdog Gauge
// ============================================================================
inline void DrawGasMeter(ImDrawList* draw_list, ImVec2 pos, ImVec2 size,
                         float gas_used, float gas_limit, bool is_tripped) {
    const float x = pos.x;
    const float y = pos.y;
    const float w = size.x;
    const float h = size.y;

    draw_list->AddRectFilled(pos, ImVec2(x + w, y + h), ImColor(235, 238, 242, 255), 2.0f);
    draw_list->AddRect(pos, ImVec2(x + w, y + h), is_tripped ? ImColor(220, 38, 38, 255) : ImColor(190, 196, 206, 255), 2.0f);

    const float ratio = std::clamp(gas_used / gas_limit, 0.0f, 1.0f);
    const float bar_w = (w - 4.0f) * ratio;

    ImU32 col_bar = is_tripped ? ImColor(220, 38, 38, 220) :
                    (ratio > 0.8f) ? ImColor(217, 119, 6, 220) :
                                     ImColor(31, 97, 217, 220);

    if (bar_w > 0.5f) {
        draw_list->AddRectFilled(ImVec2(x + 2.0f, y + 2.0f), ImVec2(x + 2.0f + bar_w, y + h - 2.0f), col_bar, 1.0f);
    }

    char text[64];
    if (is_tripped) {
        std::snprintf(text, sizeof(text), "CIRCUIT BREAKER TRIPPED! [AUTO-BYPASSED]");
    } else {
        std::snprintf(text, sizeof(text), "GAS: %.1f / %.1f (%.0f%%)", gas_used, gas_limit, ratio * 100.0f);
    }
    ImVec2 txt_sz = ImGui::CalcTextSize(text);
    draw_list->AddText(ImVec2(x + (w - txt_sz.x) * 0.5f, y + (h - txt_sz.y) * 0.5f),
                       is_tripped ? ImColor(255, 255, 255, 255) : ImColor(30, 35, 45, 255), text);
}

// ============================================================================
// Orderly Architect's Desk: Kinetic ODE & Airwindows Hit Record Meter
// - Upper: 2D Poincaré Phase-Space Plot (x vs dx/dt) with Blueprint Cobalt Attractor
// - Lower: Tri-Color Airwindows Bars (Green Authority, Blue Power, Red Detail)
// - Dynamic Hit Profile Diagnostic Ribbon
// ============================================================================
inline void DrawKineticHitMeter(ImDrawList* draw_list, ImVec2 pos, ImVec2 size,
                                const protocol::KineticTelemetryData& data) {
    const float x = pos.x;
    const float y = pos.y;
    const float w = size.x;
    const float h = size.y;

    // 1. Drafting Vellum Background Canvas
    draw_list->AddRectFilled(pos, ImVec2(x + w, y + h), ImColor(252, 253, 254, 255), 3.0f);
    draw_list->AddRect(pos, ImVec2(x + w, y + h), ImColor(199, 204, 214, 255), 3.0f, 0, 1.0f);

    // Layout Split: Upper Phase Attractor (h - 76px), Lower Tri-Color Bars (76px)
    const float split_y = y + h - 74.0f;
    const float plot_cx = x + w * 0.5f;
    const float plot_cy = y + (split_y - y) * 0.5f;
    const float plot_radius = std::max(20.0f, std::min(w * 0.5f - 16.0f, (split_y - y) * 0.5f - 10.0f));

    // Millimeter Grid in Upper Plot
    const ImU32 col_axis = ImColor(210, 215, 224, 255);

    // Crosshairs
    draw_list->AddLine(ImVec2(plot_cx - plot_radius, plot_cy), ImVec2(plot_cx + plot_radius, plot_cy), col_axis, 1.0f);
    draw_list->AddLine(ImVec2(plot_cx, plot_cy - plot_radius), ImVec2(plot_cx, plot_cy + plot_radius), col_axis, 1.0f);

    // Reference Target Circles: Golden Ratio Hit Zone (0.65) and 0dBFS boundary (0.95)
    draw_list->AddCircle(ImVec2(plot_cx, plot_cy), plot_radius * 0.65f, ImColor(31, 97, 217, 45), 32, 1.0f);
    draw_list->AddCircle(ImVec2(plot_cx, plot_cy), plot_radius * 0.95f, ImColor(217, 46, 56, 40), 32, 1.0f);

    // 2. Render 2D Phase Space Point Cloud (x vs dx/dt)
    for (size_t i = 0; i < protocol::KineticTelemetryData::kPhasePoints; ++i) {
        float px = data.phase_x[i];
        float py = data.phase_y[i];

        if (std::abs(px) < 0.0001f && std::abs(py) < 0.0001f) continue;

        float screen_x = plot_cx + px * plot_radius;
        float screen_y = plot_cy - py * plot_radius;

        // Fading Blueprint dot: Recent points are solid, older points decay
        float alpha = 0.25f + 0.75f * (static_cast<float>(i) / static_cast<float>(protocol::KineticTelemetryData::kPhasePoints));
        
        // Edge clipping indicator
        bool is_edge = (std::abs(px) >= 0.92f || std::abs(py) >= 0.92f);
        ImU32 pt_col = is_edge ? ImColor(217, 46, 56, static_cast<int>(alpha * 255.0f))
                               : ImColor(31, 97, 217, static_cast<int>(alpha * 220.0f));

        draw_list->AddCircleFilled(ImVec2(screen_x, screen_y), is_edge ? 2.5f : 1.8f, pt_col);
    }

    // Axis Legend
    draw_list->AddText(ImVec2(plot_cx + plot_radius - 20.0f, plot_cy + 2.0f), ImColor(160, 168, 180, 255), "+X");
    draw_list->AddText(ImVec2(plot_cx + 4.0f, plot_cy - plot_radius), ImColor(160, 168, 180, 255), "+dX");

    // Divider Line between Plot and Meters
    draw_list->AddLine(ImVec2(x + 8.0f, split_y), ImVec2(x + w - 8.0f, split_y), ImColor(218, 222, 230, 255), 1.0f);

    // 3. Lower Section: Airwindows Tri-Color Horizontal Meters
    const float meter_start_y = split_y + 8.0f;
    const float bar_h = 10.0f;
    const float label_w = 80.0f;
    const float bar_w = std::max(40.0f, w - label_w - 24.0f);

    auto draw_meter_row = [&](int row, const char* name, float val, ImU32 bar_color) {
        float row_y = meter_start_y + row * 18.0f;
        draw_list->AddText(ImVec2(x + 10.0f, row_y - 1.0f), ImColor(80, 90, 105, 255), name);

        // Recessed slot
        float bx = x + label_w + 10.0f;
        draw_list->AddRectFilled(ImVec2(bx, row_y), ImVec2(bx + bar_w, row_y + bar_h), ImColor(235, 238, 244, 255), 2.0f);
        draw_list->AddRect(ImVec2(bx, row_y), ImVec2(bx + bar_w, row_y + bar_h), ImColor(205, 210, 220, 255), 2.0f);

        // Fill
        float fill_w = bar_w * std::clamp(val, 0.0f, 1.0f);
        if (fill_w > 1.0f) {
            draw_list->AddRectFilled(ImVec2(bx + 1.0f, row_y + 1.0f), ImVec2(bx + fill_w, row_y + bar_h - 1.0f), bar_color, 1.0f);
        }
    };

    draw_meter_row(0, "AUTHORITY", data.authority, ImColor(34, 197, 94, 220)); // Emerald Green
    draw_meter_row(1, "POWER",     data.power,     ImColor(31, 97, 217, 220)); // Blueprint Cobalt
    draw_meter_row(2, "DETAIL",    data.detail,    ImColor(217, 46, 56, 220)); // Crimson Red

    // 4. Dynamic Diagnostic Pill / Badge
    const char* diag_text = "IDLE // WAITING FOR SIGNAL";
    ImVec4 diag_bg = ImVec4(0.92f, 0.94f, 0.96f, 1.0f);
    ImVec4 diag_fg = ImVec4(0.35f, 0.40f, 0.48f, 1.0f);

    switch (data.diagnostic_id) {
        case 1:
            diag_text = "HIT PROFILE: BALANCED SONORITY (BLUE CLOUD OPTIMAL)";
            diag_bg = ImVec4(0.88f, 0.94f, 1.00f, 1.0f);
            diag_fg = ImVec4(0.12f, 0.38f, 0.85f, 1.0f); // Blueprint Cobalt
            break;
        case 2:
            diag_text = "WARNING: EXCESS SLEW / HARSH (REDUCE RED HIGHS)";
            diag_bg = ImVec4(1.00f, 0.90f, 0.90f, 1.0f);
            diag_fg = ImVec4(0.85f, 0.18f, 0.22f, 1.0f); // Crimson
            break;
        case 3:
            diag_text = "WARNING: LACKS AUTHORITY (SUB-BASS TOO THIN)";
            diag_bg = ImVec4(1.00f, 0.95f, 0.85f, 1.0f);
            diag_fg = ImVec4(0.85f, 0.48f, 0.05f, 1.0f); // Ruler Ochre
            break;
        case 4:
            diag_text = "WARNING: EXCESS ZERO-CROSS (MUDDY / OVERBOOSTED SUB)";
            diag_bg = ImVec4(0.90f, 0.97f, 0.92f, 1.0f);
            diag_fg = ImVec4(0.13f, 0.65f, 0.35f, 1.0f); // Forest Green
            break;
        case 5:
            diag_text = "CRITICAL: BRICKWALL CLIPPED (COLLAPSED ORBIT)";
            diag_bg = ImVec4(1.00f, 0.85f, 0.85f, 1.0f);
            diag_fg = ImVec4(0.85f, 0.10f, 0.15f, 1.0f); // Bright Red
            break;
        default:
            break;
    }

    // Render Pill Banner at top of the widget
    ImVec2 txt_sz = ImGui::CalcTextSize(diag_text);
    float pill_w = txt_sz.x + 16.0f;
    float pill_h = txt_sz.y + 6.0f;
    ImVec2 pill_pos(x + 10.0f, y + 8.0f);

    draw_list->AddRectFilled(pill_pos, ImVec2(pill_pos.x + pill_w, pill_pos.y + pill_h), ImColor(diag_bg), 3.0f);
    draw_list->AddRect(pill_pos, ImVec2(pill_pos.x + pill_w, pill_pos.y + pill_h), ImColor(diag_fg.x, diag_fg.y, diag_fg.z, 0.35f), 3.0f);
    draw_list->AddText(ImVec2(pill_pos.x + 8.0f, pill_pos.y + 3.0f), ImColor(diag_fg), diag_text);
}

// ============================================================================
// Orderly Architect's Desk: FontLab-Inspired Automation Curve Editor
// Technical Vellum Canvas, Blueprint Cobalt Splines, FontLab Smooth/Corner Nodes,
// Tension Dots (Rapid Tool curvature bending), Ghost Splitting & Live Needle
// ============================================================================
inline bool DrawAutomationCurveEditor(const char* str_id,
                                      routing::AutomationCurve& curve,
                                      ImVec2 size = ImVec2(0, 160),
                                      double total_beats = 16.0,
                                      double current_playhead_beat = -1.0,
                                      int* selected_point_out = nullptr,
                                      routing::AutomationTarget target = routing::AutomationTarget::Gain,
                                      std::vector<size_t>* selected_points_out = nullptr,
                                      float custom_min = 0.0f,
                                      float custom_max = 1.0f,
                                      const char* custom_unit = nullptr,
                                      bool compact_mode = false) {
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    if (window->SkipItems) return false;

    ImVec2 pos = ImGui::GetCursorScreenPos();
    if (size.x <= 0.0f) size.x = ImGui::GetContentRegionAvail().x;
    ImRect bb(pos, ImVec2(pos.x + size.x, pos.y + size.y));
    ImGui::ItemSize(bb);
    ImGuiID id = window->GetID(str_id);
    if (!ImGui::ItemAdd(bb, id)) return false;

    ImDrawList* draw_list = window->DrawList;
    const float x = pos.x;
    const float y = pos.y;
    const float w = size.x;
    const float h = size.y;
    const float pad_top = compact_mode ? 8.0f : 18.0f;
    const float pad_bot = compact_mode ? 8.0f : 18.0f;
    const float usable_h = h - pad_top - pad_bot;
    const float bot_y = y + h - pad_bot;

    float min_val = 0.0f;
    float max_val = 1.25f; // Headroom up to +2 dB for Gain
    if (target == routing::AutomationTarget::Pan) {
        min_val = -1.0f;
        max_val = 1.0f;
    } else if (target == routing::AutomationTarget::Aux1 || target == routing::AutomationTarget::Aux2) {
        min_val = 0.0f;
        max_val = 1.0f;
    } else if (target == routing::AutomationTarget::Pitch) {
        min_val = -24.0f;
        max_val = 24.0f;
    } else if (target == routing::AutomationTarget::PluginParam) {
        min_val = custom_min;
        max_val = (custom_max > custom_min) ? custom_max : (custom_min + 1.0f);
    }

    auto beat_to_x = [&](double b) -> float {
        return x + static_cast<float>(std::clamp(b / total_beats, 0.0, 1.0)) * w;
    };
    auto x_to_beat = [&](float px) -> double {
        return std::clamp(static_cast<double>((px - x) / w) * total_beats, 0.0, total_beats);
    };
    auto val_to_y = [&](float v) -> float {
        float norm = std::clamp((v - min_val) / (max_val - min_val), 0.0f, 1.0f);
        return bot_y - (norm * usable_h);
    };
    auto y_to_val = [&](float py) -> float {
        float norm = std::clamp((bot_y - py) / usable_h, 0.0f, 1.0f);
        return min_val + norm * (max_val - min_val);
    };

    struct CurveEditorState {
        std::vector<size_t> selected_indices;
        int active_drag_idx = -1;
        int active_tension_idx = -1;
        bool is_dragging_nodes = false;
        bool is_dragging_tension = false;
        bool is_marquee_selecting = false;
        ImVec2 marquee_start{0, 0};
        ImVec2 marquee_end{0, 0};

        enum class TransformHandle { None, Left, Right, Top, Bottom };
        TransformHandle active_handle = TransformHandle::None;

        struct InitialPoint {
            size_t idx;
            double t;
            float v;
        };
        std::vector<InitialPoint> drag_start_pts;
        double drag_start_mouse_beat = 0.0;
        float drag_start_mouse_val = 0.0f;
        double bbox_min_t = 0.0;
        double bbox_max_t = 0.0;
        float bbox_min_v = 0.0f;
        float bbox_max_v = 0.0f;
    };

    uint64_t state_key = (static_cast<uint64_t>(id) << 32) ^ static_cast<uint64_t>(reinterpret_cast<uintptr_t>(&curve));
    static std::unordered_map<uint64_t, CurveEditorState> s_editor_states;
    CurveEditorState& state = s_editor_states[state_key];

    ImGuiIO& io = ImGui::GetIO();
    ImVec2 mouse = io.MousePos;
    bool is_hovered = bb.Contains(mouse);

    auto points = curve.get_points();
    auto snap = curve.snapshot();

    // Sanitize selection indices
    std::vector<size_t> valid_sel;
    for (size_t s : state.selected_indices) {
        if (s < points.size() && std::find(valid_sel.begin(), valid_sel.end(), s) == valid_sel.end()) {
            valid_sel.push_back(s);
        }
    }
    state.selected_indices = std::move(valid_sel);

    auto snap_val = [&](float v) -> float {
        if (io.KeyShift) return v;
        if (target == routing::AutomationTarget::Pitch) {
            return std::round(v); // Snap to semitone
        } else if (target == routing::AutomationTarget::Pan) {
            if (std::abs(v) < 0.05f) return 0.0f; // Snap to Center
            if (std::abs(v - 1.0f) < 0.04f) return 1.0f;
            if (std::abs(v + 1.0f) < 0.04f) return -1.0f;
        } else if (target == routing::AutomationTarget::Gain) {
            if (std::abs(v - 1.0f) < 0.04f) return 1.0f; // Snap to 0 dB
            if (std::abs(v - 0.5f) < 0.03f) return 0.5f; // Snap to -6 dB
            if (v < 0.03f) return 0.0f; // Snap to silence
        } else if (target == routing::AutomationTarget::PluginParam) {
            float span = max_val - min_val;
            if (span > 1e-4f) {
                float norm = (v - min_val) / span;
                if (std::abs(norm - 0.5f) < 0.03f) return min_val + 0.5f * span;
                if (norm < 0.03f) return min_val;
                if (norm > 0.97f) return max_val;
            }
        } else {
            if (v < 0.03f) return 0.0f;
            if (std::abs(v - 0.5f) < 0.03f) return 0.5f;
            if (std::abs(v - 1.0f) < 0.04f) return 1.0f;
        }
        return v;
    };

    auto snap_beat = [&](double b) -> double {
        if (io.KeyShift) return b;
        return std::round(b * 4.0) / 4.0; // Snap to 1/16th beat
    };

    // Calculate Bounding Box of selection
    bool has_multi_selection = state.selected_indices.size() >= 2;
    double sel_min_t = 1e9, sel_max_t = -1e9;
    float sel_min_v = 1e9f, sel_max_v = -1e9f;
    if (has_multi_selection) {
        for (size_t s : state.selected_indices) {
            if (s < points.size()) {
                sel_min_t = std::min(sel_min_t, points[s].time_beats);
                sel_max_t = std::max(sel_max_t, points[s].time_beats);
                sel_min_v = std::min(sel_min_v, points[s].value);
                sel_max_v = std::max(sel_max_v, points[s].value);
            }
        }
    }

    float bbox_x1 = beat_to_x(sel_min_t) - 8.0f;
    float bbox_x2 = beat_to_x(sel_max_t) + 8.0f;
    float bbox_y_top = val_to_y(sel_max_v) - 8.0f;
    float bbox_y_bot = val_to_y(sel_min_v) + 8.0f;

    // 1. Hit Testing: Transform Handles, Points, Tension Handles, and Curve Line
    int hovered_pt = -1;
    int hovered_tension = -1;
    CurveEditorState::TransformHandle hovered_handle = CurveEditorState::TransformHandle::None;

    if (has_multi_selection && !state.is_dragging_nodes && !state.is_marquee_selecting) {
        ImVec2 h_l(bbox_x1, 0.5f * (bbox_y_top + bbox_y_bot));
        ImVec2 h_r(bbox_x2, 0.5f * (bbox_y_top + bbox_y_bot));
        ImVec2 h_t(0.5f * (bbox_x1 + bbox_x2), bbox_y_top);
        ImVec2 h_b(0.5f * (bbox_x1 + bbox_x2), bbox_y_bot);

        if (std::hypot(mouse.x - h_l.x, mouse.y - h_l.y) <= 8.0f) {
            hovered_handle = CurveEditorState::TransformHandle::Left;
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        } else if (std::hypot(mouse.x - h_r.x, mouse.y - h_r.y) <= 8.0f) {
            hovered_handle = CurveEditorState::TransformHandle::Right;
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        } else if (std::hypot(mouse.x - h_t.x, mouse.y - h_t.y) <= 8.0f) {
            hovered_handle = CurveEditorState::TransformHandle::Top;
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
        } else if (std::hypot(mouse.x - h_b.x, mouse.y - h_b.y) <= 8.0f) {
            hovered_handle = CurveEditorState::TransformHandle::Bottom;
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
        }
    }

    if (hovered_handle == CurveEditorState::TransformHandle::None) {
        for (size_t i = 0; i < points.size(); ++i) {
            float px = beat_to_x(points[i].time_beats);
            float py = val_to_y(points[i].value);
            if (std::hypot(mouse.x - px, mouse.y - py) <= 9.0f) {
                hovered_pt = static_cast<int>(i);
                break;
            }
        }
    }

    if (hovered_handle == CurveEditorState::TransformHandle::None && hovered_pt < 0) {
        for (size_t i = 0; i + 1 < points.size(); ++i) {
            if (points[i].node_mode == routing::NodeMode::Hold) continue;
            double mid_t = 0.5 * (points[i].time_beats + points[i + 1].time_beats);
            float mid_v = snap ? snap->evaluate(mid_t) : 0.5f * (points[i].value + points[i + 1].value);
            float tx = beat_to_x(mid_t);
            float ty = val_to_y(mid_v);
            if (std::hypot(mouse.x - tx, mouse.y - ty) <= 8.0f) {
                hovered_tension = static_cast<int>(i);
                break;
            }
        }
    }

    // Check if mouse is directly on the curve (Ghost node splitting)
    bool is_mouse_on_curve = false;
    double mouse_beat = x_to_beat(mouse.x);
    float curve_val_at_mouse = snap ? snap->evaluate(mouse_beat) : 0.0f;
    float curve_y_at_mouse = val_to_y(curve_val_at_mouse);
    if (is_hovered && hovered_handle == CurveEditorState::TransformHandle::None && hovered_pt < 0 && hovered_tension < 0) {
        if (std::abs(mouse.y - curve_y_at_mouse) <= 10.0f && mouse.x >= x && mouse.x <= x + w) {
            is_mouse_on_curve = true;
        }
    }

    // Dynamic mouse cursor styling
    if (hovered_pt >= 0) {
        bool is_sel = std::find(state.selected_indices.begin(), state.selected_indices.end(), static_cast<size_t>(hovered_pt)) != state.selected_indices.end();
        ImGui::SetMouseCursor(is_sel ? ImGuiMouseCursor_ResizeAll : ImGuiMouseCursor_Hand);
    } else if (is_mouse_on_curve) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
    }

    // 2. Mouse Actions & Gestures
    bool modified = false;

    if (is_hovered && ImGui::IsMouseClicked(0)) {
        if (hovered_handle != CurveEditorState::TransformHandle::None) {
            state.active_handle = hovered_handle;
            state.drag_start_pts.clear();
            for (size_t s : state.selected_indices) {
                if (s < points.size()) {
                    state.drag_start_pts.push_back({s, points[s].time_beats, points[s].value});
                }
            }
            state.drag_start_mouse_beat = x_to_beat(mouse.x);
            state.drag_start_mouse_val = y_to_val(mouse.y);
            state.bbox_min_t = sel_min_t;
            state.bbox_max_t = sel_max_t;
            state.bbox_min_v = sel_min_v;
            state.bbox_max_v = sel_max_v;
        } else if (hovered_pt >= 0) {
            size_t h_idx = static_cast<size_t>(hovered_pt);
            if (io.MouseDoubleClicked[0]) {
                // FontLab Double-Click: Toggle Smooth <-> Corner <-> Hold
                if (std::find(state.selected_indices.begin(), state.selected_indices.end(), h_idx) != state.selected_indices.end()) {
                    for (size_t s : state.selected_indices) {
                        curve.toggle_node_mode(s);
                    }
                } else {
                    curve.toggle_node_mode(h_idx);
                }
                modified = true;
            } else {
                if (io.KeyShift) {
                    auto it = std::find(state.selected_indices.begin(), state.selected_indices.end(), h_idx);
                    if (it != state.selected_indices.end()) {
                        state.selected_indices.erase(it);
                    } else {
                        state.selected_indices.push_back(h_idx);
                    }
                } else {
                    auto it = std::find(state.selected_indices.begin(), state.selected_indices.end(), h_idx);
                    if (it == state.selected_indices.end()) {
                        state.selected_indices = { h_idx };
                    }
                }

                state.is_dragging_nodes = true;
                state.active_drag_idx = hovered_pt;
                state.drag_start_mouse_beat = x_to_beat(mouse.x);
                state.drag_start_mouse_val = y_to_val(mouse.y);
                state.drag_start_pts.clear();
                for (size_t s : state.selected_indices) {
                    if (s < points.size()) {
                        state.drag_start_pts.push_back({s, points[s].time_beats, points[s].value});
                    }
                }
            }
        } else if (hovered_tension >= 0) {
            state.active_tension_idx = hovered_tension;
            state.is_dragging_tension = true;
        } else if (is_mouse_on_curve) {
            // Click on curve line: Split curve & insert new node (Ghost node becomes real)
            double nb = snap_beat(mouse_beat);
            float nv = snap_val(curve_val_at_mouse);
            size_t new_idx = curve.add_point(nb, nv, routing::NodeMode::Smooth, 0.0f);
            state.selected_indices = { new_idx };
            state.is_dragging_nodes = true;
            state.active_drag_idx = static_cast<int>(new_idx);
            state.drag_start_mouse_beat = nb;
            state.drag_start_mouse_val = nv;
            state.drag_start_pts = { {new_idx, nb, nv} };
            modified = true;
        } else {
            // Click in empty canvas: Marquee Selection Box
            if (!io.KeyShift) {
                state.selected_indices.clear();
            }
            state.is_marquee_selecting = true;
            state.marquee_start = mouse;
            state.marquee_end = mouse;
        }
    }

    if (is_hovered && ImGui::IsMouseClicked(1)) {
        if (hovered_pt >= 0) {
            size_t h_idx = static_cast<size_t>(hovered_pt);
            bool in_sel = std::find(state.selected_indices.begin(), state.selected_indices.end(), h_idx) != state.selected_indices.end();
            if (in_sel && state.selected_indices.size() > 1) {
                curve.remove_points(state.selected_indices);
                state.selected_indices.clear();
            } else {
                curve.remove_point(h_idx);
                auto it = std::find(state.selected_indices.begin(), state.selected_indices.end(), h_idx);
                if (it != state.selected_indices.end()) state.selected_indices.erase(it);
            }
            modified = true;
        }
    }

    // Dragging
    if (io.MouseDown[0]) {
        if (state.active_handle != CurveEditorState::TransformHandle::None) {
            // Bounding box transformation
            double cur_b = x_to_beat(mouse.x);
            float cur_v = y_to_val(mouse.y);

            if (state.active_handle == CurveEditorState::TransformHandle::Right) {
                double orig_span = state.bbox_max_t - state.bbox_min_t;
                if (orig_span > 1e-4) {
                    double target_b = snap_beat(cur_b);
                    double scale_t = std::max(0.05, (target_b - state.bbox_min_t) / orig_span);
                    for (const auto& pt : state.drag_start_pts) {
                        double rel = pt.t - state.bbox_min_t;
                        double new_t = std::clamp(state.bbox_min_t + rel * scale_t, 0.0, total_beats);
                        curve.update_point(pt.idx, new_t, pt.v);
                    }
                    modified = true;
                }
            } else if (state.active_handle == CurveEditorState::TransformHandle::Left) {
                double orig_span = state.bbox_max_t - state.bbox_min_t;
                if (orig_span > 1e-4) {
                    double target_b = snap_beat(cur_b);
                    double scale_t = std::max(0.05, (state.bbox_max_t - target_b) / orig_span);
                    for (const auto& pt : state.drag_start_pts) {
                        double rel = state.bbox_max_t - pt.t;
                        double new_t = std::clamp(state.bbox_max_t - rel * scale_t, 0.0, total_beats);
                        curve.update_point(pt.idx, new_t, pt.v);
                    }
                    modified = true;
                }
            } else if (state.active_handle == CurveEditorState::TransformHandle::Top) {
                float orig_h = state.bbox_max_v - state.bbox_min_v;
                if (orig_h > 1e-4f) {
                    float target_v = snap_val(cur_v);
                    float scale_v = (target_v - state.bbox_min_v) / orig_h;
                    for (const auto& pt : state.drag_start_pts) {
                        float rel = pt.v - state.bbox_min_v;
                        float new_v = std::clamp(state.bbox_min_v + rel * scale_v, min_val, max_val);
                        curve.update_point(pt.idx, pt.t, new_v);
                    }
                    modified = true;
                }
            } else if (state.active_handle == CurveEditorState::TransformHandle::Bottom) {
                float orig_h = state.bbox_max_v - state.bbox_min_v;
                if (orig_h > 1e-4f) {
                    float target_v = snap_val(cur_v);
                    float scale_v = (state.bbox_max_v - target_v) / orig_h;
                    for (const auto& pt : state.drag_start_pts) {
                        float rel = state.bbox_max_v - pt.v;
                        float new_v = std::clamp(state.bbox_max_v - rel * scale_v, min_val, max_val);
                        curve.update_point(pt.idx, pt.t, new_v);
                    }
                    modified = true;
                }
            }
        } else if (state.is_dragging_nodes && !state.drag_start_pts.empty()) {
            double cur_b = x_to_beat(mouse.x);
            float cur_v = y_to_val(mouse.y);
            double delta_b = cur_b - state.drag_start_mouse_beat;
            float delta_v = cur_v - state.drag_start_mouse_val;

            if (!io.KeyShift) {
                delta_b = std::round(delta_b * 4.0) / 4.0;
            }

            for (const auto& pt : state.drag_start_pts) {
                double new_t = std::clamp(pt.t + delta_b, 0.0, total_beats);
                float new_v = snap_val(std::clamp(pt.v + delta_v, min_val, max_val));
                curve.update_point(pt.idx, new_t, new_v);
            }
            modified = true;
        } else if (state.is_dragging_tension && state.active_tension_idx >= 0 &&
                   static_cast<size_t>(state.active_tension_idx) < points.size()) {
            float dy = -io.MouseDelta.y * 0.035f;
            float cur_tau = points[state.active_tension_idx].tension + dy;
            curve.set_segment_tension(static_cast<size_t>(state.active_tension_idx), cur_tau);
            modified = true;
        } else if (state.is_marquee_selecting) {
            state.marquee_end = mouse;

            float mx1 = std::min(state.marquee_start.x, state.marquee_end.x);
            float mx2 = std::max(state.marquee_start.x, state.marquee_end.x);
            float my1 = std::min(state.marquee_start.y, state.marquee_end.y);
            float my2 = std::max(state.marquee_start.y, state.marquee_end.y);

            for (size_t i = 0; i < points.size(); ++i) {
                float px = beat_to_x(points[i].time_beats);
                float py = val_to_y(points[i].value);
                if (px >= mx1 && px <= mx2 && py >= my1 && py <= my2) {
                    if (std::find(state.selected_indices.begin(), state.selected_indices.end(), i) == state.selected_indices.end()) {
                        state.selected_indices.push_back(i);
                    }
                }
            }
        }
    } else {
        if (state.is_dragging_nodes || state.active_handle != CurveEditorState::TransformHandle::None) {
            // Re-sync selected_indices to match newly sorted points by (time_beats, value)
            auto cur_pts = curve.get_points();
            std::vector<size_t> new_sel;
            for (const auto& sp : state.drag_start_pts) {
                size_t best_idx = 0;
                double best_dist = 1e9;
                for (size_t i = 0; i < cur_pts.size(); ++i) {
                    double d = std::abs(cur_pts[i].time_beats - sp.t) + std::abs(cur_pts[i].value - sp.v);
                    if (d < best_dist) {
                        best_dist = d;
                        best_idx = i;
                    }
                }
                if (best_dist < 0.1 && std::find(new_sel.begin(), new_sel.end(), best_idx) == new_sel.end()) {
                    new_sel.push_back(best_idx);
                }
            }
            if (!new_sel.empty()) state.selected_indices = new_sel;
        }
        state.is_dragging_nodes = false;
        state.is_dragging_tension = false;
        state.is_marquee_selecting = false;
        state.active_handle = CurveEditorState::TransformHandle::None;
        state.active_drag_idx = -1;
        state.active_tension_idx = -1;
    }

    // Keyboard Shortcuts & Fine Nudging
    if (is_hovered) {
        if ((ImGui::IsKeyPressed(ImGuiKey_Delete) || ImGui::IsKeyPressed(ImGuiKey_Backspace)) && !state.selected_indices.empty()) {
            curve.remove_points(state.selected_indices);
            state.selected_indices.clear();
            modified = true;
        }
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_A)) {
            state.selected_indices.clear();
            for (size_t i = 0; i < points.size(); ++i) state.selected_indices.push_back(i);
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
            state.selected_indices.clear();
        }
        if (!state.selected_indices.empty()) {
            double nudge_b = 0.0;
            float nudge_v = 0.0f;
            if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow))  nudge_b -= (io.KeyAlt ? 0.0625 : 0.25);
            if (ImGui::IsKeyPressed(ImGuiKey_RightArrow)) nudge_b += (io.KeyAlt ? 0.0625 : 0.25);
            if (ImGui::IsKeyPressed(ImGuiKey_UpArrow)) {
                if (target == routing::AutomationTarget::Pitch) nudge_v += (io.KeyAlt ? 0.1f : 1.0f);
                else if (target == routing::AutomationTarget::PluginParam) nudge_v += (io.KeyAlt ? 0.005f : 0.02f) * (max_val - min_val);
                else nudge_v += (target == routing::AutomationTarget::Pan ? 0.05f : 0.02f);
            }
            if (ImGui::IsKeyPressed(ImGuiKey_DownArrow)) {
                if (target == routing::AutomationTarget::Pitch) nudge_v -= (io.KeyAlt ? 0.1f : 1.0f);
                else if (target == routing::AutomationTarget::PluginParam) nudge_v -= (io.KeyAlt ? 0.005f : 0.02f) * (max_val - min_val);
                else nudge_v -= (target == routing::AutomationTarget::Pan ? 0.05f : 0.02f);
            }

            if (nudge_b != 0.0 || nudge_v != 0.0f) {
                curve.move_points(state.selected_indices, nudge_b, nudge_v, min_val, max_val);
                modified = true;
            }
        }
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_D) && !state.selected_indices.empty()) {
            state.selected_indices = curve.duplicate_points(state.selected_indices, 1.0);
            modified = true;
        }
    }

    if (selected_point_out) {
        *selected_point_out = state.selected_indices.empty() ? -1 : static_cast<int>(state.selected_indices.back());
    }
    if (selected_points_out) {
        *selected_points_out = state.selected_indices;
    }

    // Re-fetch points after potential edits
    points = curve.get_points();
    snap = curve.snapshot();

    // 3. Render Canvas & Guidelines (Drafting Millimeter Aesthetic)
    draw_list->AddRectFilled(pos, ImVec2(x + w, y + h), ImColor(255, 255, 255, 255), 2.0f);
    draw_list->AddRect(pos, ImVec2(x + w, y + h), ImColor(190, 196, 206, 255), 2.0f);

    if (target == routing::AutomationTarget::Pan) {
        // Center line [C]
        float y_c = val_to_y(0.0f);
        draw_list->AddLine(ImVec2(x, y_c), ImVec2(x + w, y_c), ImColor(140, 150, 168, 255), 1.5f);
        draw_list->AddText(ImVec2(x + 6.0f, y_c - 13.0f), ImColor(120, 130, 145, 220), "Center [C]");

        // Right line [+1.0]
        float y_r = val_to_y(1.0f);
        draw_list->AddLine(ImVec2(x, y_r), ImVec2(x + w, y_r), ImColor(225, 230, 238, 255), 1.0f);
        draw_list->AddText(ImVec2(x + 6.0f, y_r + 2.0f), ImColor(160, 170, 185, 200), "Right [+1.0]");

        // Left line [-1.0]
        draw_list->AddLine(ImVec2(x, bot_y), ImVec2(x + w, bot_y), ImColor(200, 208, 220, 255), 1.0f);
        draw_list->AddText(ImVec2(x + 6.0f, bot_y - 13.0f), ImColor(160, 170, 185, 200), "Left [-1.0]");
    } else if (target == routing::AutomationTarget::Aux1 || target == routing::AutomationTarget::Aux2) {
        float y_100 = val_to_y(1.0f);
        draw_list->AddLine(ImVec2(x, y_100), ImVec2(x + w, y_100), ImColor(140, 150, 168, 255), 1.5f);
        draw_list->AddText(ImVec2(x + 6.0f, y_100 + 2.0f), ImColor(120, 130, 145, 220), "100% [Full Send]");

        float y_50 = val_to_y(0.5f);
        draw_list->AddLine(ImVec2(x, y_50), ImVec2(x + w, y_50), ImColor(225, 230, 238, 255), 1.0f);
        draw_list->AddText(ImVec2(x + 6.0f, y_50 - 13.0f), ImColor(160, 170, 185, 200), "50% [-6 dB]");

        draw_list->AddLine(ImVec2(x, bot_y), ImVec2(x + w, bot_y), ImColor(200, 208, 220, 255), 1.0f);
        draw_list->AddText(ImVec2(x + 6.0f, bot_y - 13.0f), ImColor(160, 170, 185, 200), "0% [Off]");
    } else if (target == routing::AutomationTarget::Pitch) {
        float y_0 = val_to_y(0.0f);
        draw_list->AddLine(ImVec2(x, y_0), ImVec2(x + w, y_0), ImColor(140, 150, 168, 255), 1.5f);
        draw_list->AddText(ImVec2(x + 6.0f, y_0 - 13.0f), ImColor(120, 130, 145, 220), "0 st [Original Pitch]");

        float y_12 = val_to_y(12.0f);
        draw_list->AddLine(ImVec2(x, y_12), ImVec2(x + w, y_12), ImColor(225, 230, 238, 255), 1.0f);
        draw_list->AddText(ImVec2(x + 6.0f, y_12 - 13.0f), ImColor(160, 170, 185, 200), "+12 st (+1 Oct)");

        float y_m12 = val_to_y(-12.0f);
        draw_list->AddLine(ImVec2(x, y_m12), ImVec2(x + w, y_m12), ImColor(225, 230, 238, 255), 1.0f);
        draw_list->AddText(ImVec2(x + 6.0f, y_m12 - 13.0f), ImColor(160, 170, 185, 200), "-12 st (-1 Oct)");

        draw_list->AddLine(ImVec2(x, bot_y), ImVec2(x + w, bot_y), ImColor(200, 208, 220, 255), 1.0f);
        draw_list->AddText(ImVec2(x + 6.0f, bot_y - 13.0f), ImColor(160, 170, 185, 200), "-24 st (-2 Oct)");
    } else if (target == routing::AutomationTarget::PluginParam) {
        float y_max = val_to_y(max_val);
        draw_list->AddLine(ImVec2(x, y_max), ImVec2(x + w, y_max), ImColor(140, 150, 168, 255), 1.0f);
        char lbl_max[32];
        if (custom_unit) std::snprintf(lbl_max, sizeof(lbl_max), "%.2f %s", max_val, custom_unit);
        else std::snprintf(lbl_max, sizeof(lbl_max), "%.2f", max_val);
        draw_list->AddText(ImVec2(x + 6.0f, y_max + 2.0f), ImColor(120, 130, 145, 220), lbl_max);

        float mid_val = (min_val + max_val) * 0.5f;
        float y_mid = val_to_y(mid_val);
        draw_list->AddLine(ImVec2(x, y_mid), ImVec2(x + w, y_mid), ImColor(225, 230, 238, 255), 1.0f);
        char lbl_mid[32];
        if (custom_unit) std::snprintf(lbl_mid, sizeof(lbl_mid), "%.2f %s", mid_val, custom_unit);
        else std::snprintf(lbl_mid, sizeof(lbl_mid), "%.2f", mid_val);
        draw_list->AddText(ImVec2(x + 6.0f, y_mid - 13.0f), ImColor(160, 170, 185, 200), lbl_mid);

        draw_list->AddLine(ImVec2(x, bot_y), ImVec2(x + w, bot_y), ImColor(200, 208, 220, 255), 1.0f);
        char lbl_min[32];
        if (custom_unit) std::snprintf(lbl_min, sizeof(lbl_min), "%.2f %s", min_val, custom_unit);
        else std::snprintf(lbl_min, sizeof(lbl_min), "%.2f", min_val);
        draw_list->AddText(ImVec2(x + 6.0f, bot_y - 13.0f), ImColor(160, 170, 185, 200), lbl_min);
    } else {
        // Gain: 0 dB guideline (solid graphite)
        float y_0db = val_to_y(1.0f);
        draw_list->AddLine(ImVec2(x, y_0db), ImVec2(x + w, y_0db), ImColor(140, 150, 168, 255), 1.5f);
        draw_list->AddText(ImVec2(x + 6.0f, y_0db - 13.0f), ImColor(120, 130, 145, 220), "0 dB [Unity]");

        // -6 dB guideline (dashed)
        float y_6db = val_to_y(0.5f);
        draw_list->AddLine(ImVec2(x, y_6db), ImVec2(x + w, y_6db), ImColor(225, 230, 238, 255), 1.0f);
        draw_list->AddText(ImVec2(x + 6.0f, y_6db - 13.0f), ImColor(160, 170, 185, 200), "-6 dB [0.5]");

        // Baseline (-inf)
        draw_list->AddLine(ImVec2(x, bot_y), ImVec2(x + w, bot_y), ImColor(200, 208, 220, 255), 1.0f);
        draw_list->AddText(ImVec2(x + 6.0f, bot_y - 13.0f), ImColor(160, 170, 185, 200), "-inf [Silence]");
    }

    // Vertical Bars & Beats
    for (double b = 0.0; b <= total_beats; b += 1.0) {
        float bx = beat_to_x(b);
        bool is_bar = (std::fmod(b, 4.0) == 0.0);
        if (is_bar) {
            draw_list->AddLine(ImVec2(bx, y), ImVec2(bx, y + h), ImColor(215, 222, 232, 255), 1.0f);
            char bar_txt[16];
            std::snprintf(bar_txt, sizeof(bar_txt), "Bar %d", static_cast<int>(b / 4.0) + 1);
            draw_list->AddText(ImVec2(bx + 4.0f, y + 2.0f), ImColor(140, 150, 168, 200), bar_txt);
        } else {
            draw_list->AddLine(ImVec2(bx, y + 16.0f), ImVec2(bx, y + h), ImColor(245, 247, 250, 255), 1.0f);
        }
    }

    // 4. Sampled Curve Polyline & Translucent Wash
    const int num_steps = static_cast<int>(std::clamp(w * 0.45f, 60.0f, 600.0f));
    std::vector<ImVec2> poly_pts;
    poly_pts.reserve(num_steps + 3);
    poly_pts.push_back(ImVec2(x, bot_y));

    for (int s = 0; s <= num_steps; ++s) {
        double b = (static_cast<double>(s) / static_cast<double>(num_steps)) * total_beats;
        float v = snap ? snap->evaluate(b) : 1.0f;
        poly_pts.push_back(ImVec2(beat_to_x(b), val_to_y(v)));
    }
    poly_pts.push_back(ImVec2(x + w, bot_y));

    // Semi-transparent Cobalt wash below curve
    draw_list->AddConvexPolyFilled(poly_pts.data(), static_cast<int>(poly_pts.size()), ImColor(31, 97, 217, 30));
    // Polyline stroke
    draw_list->AddPolyline(poly_pts.data() + 1, num_steps + 1, ImColor(31, 97, 217, 240), 0, 2.5f);

    // 5. Tension Dots (FontLab Curvature Handles)
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        if (points[i].node_mode == routing::NodeMode::Hold) continue;

        double mid_t = 0.5 * (points[i].time_beats + points[i + 1].time_beats);
        float mid_v = snap ? snap->evaluate(mid_t) : 0.5f * (points[i].value + points[i + 1].value);
        float tx = beat_to_x(mid_t);
        float ty = val_to_y(mid_v);
        bool is_t_hov = (hovered_tension == static_cast<int>(i) || (state.is_dragging_tension && state.active_tension_idx == static_cast<int>(i)));
        ImU32 col_t = is_t_hov ? ImColor(217, 119, 6, 255) : ImColor(217, 119, 6, 170);

        draw_list->AddCircleFilled(ImVec2(tx, ty), is_t_hov ? 5.0f : 3.5f, col_t);
        draw_list->AddCircle(ImVec2(tx, ty), is_t_hov ? 5.0f : 3.5f, ImColor(255, 255, 255, 255), 0, 1.0f);
    }

    // 6. Breakpoint Nodes (Smooth = Circle, Corner = Diamond, Hold = Step Box)
    for (size_t i = 0; i < points.size(); ++i) {
        float px = beat_to_x(points[i].time_beats);
        float py = val_to_y(points[i].value);
        bool is_sel = (std::find(state.selected_indices.begin(), state.selected_indices.end(), i) != state.selected_indices.end());
        bool is_hov = (hovered_pt == static_cast<int>(i));

        // Selected halo
        if (is_sel) {
            draw_list->AddCircle(ImVec2(px, py), 10.5f, ImColor(220, 38, 38, 170), 0, 1.5f);
        }

        if (points[i].node_mode == routing::NodeMode::Smooth) {
            float r = (is_sel || is_hov) ? 6.5f : 4.5f;
            draw_list->AddCircleFilled(ImVec2(px, py), r, ImColor(255, 255, 255, 255));
            draw_list->AddCircle(ImVec2(px, py), r, is_sel ? ImColor(220, 38, 38, 255) : ImColor(31, 97, 217, 255), 0, 2.0f);
        } else if (points[i].node_mode == routing::NodeMode::Corner) {
            float d = (is_sel || is_hov) ? 7.0f : 5.0f;
            ImVec2 p_top(px, py - d), p_right(px + d, py), p_bot(px, py + d), p_left(px - d, py);
            draw_list->AddQuadFilled(p_top, p_right, p_bot, p_left, ImColor(255, 255, 255, 255));
            draw_list->AddQuad(p_top, p_right, p_bot, p_left, is_sel ? ImColor(220, 38, 38, 255) : ImColor(217, 119, 6, 255), 2.0f);
        } else {
            float s = (is_sel || is_hov) ? 6.0f : 4.0f;
            draw_list->AddRectFilled(ImVec2(px - s, py - s), ImVec2(px + s, py + s), ImColor(255, 255, 255, 255));
            draw_list->AddRect(ImVec2(px - s, py - s), ImVec2(px + s, py + s), is_sel ? ImColor(220, 38, 38, 255) : ImColor(20, 25, 35, 255), 0.0f, 0, 2.0f);
        }
    }

    // 7. Marquee Selection Box
    if (state.is_marquee_selecting) {
        float mx1 = std::min(state.marquee_start.x, state.marquee_end.x);
        float mx2 = std::max(state.marquee_start.x, state.marquee_end.x);
        float my1 = std::min(state.marquee_start.y, state.marquee_end.y);
        float my2 = std::max(state.marquee_start.y, state.marquee_end.y);
        draw_list->AddRectFilled(ImVec2(mx1, my1), ImVec2(mx2, my2), ImColor(31, 97, 217, 35), 1.0f);
        draw_list->AddRect(ImVec2(mx1, my1), ImVec2(mx2, my2), ImColor(31, 97, 217, 220), 1.0f, 0, 1.5f);
    }

    // 8. Transform Bounding Box & Scale Handles
    if (has_multi_selection && !state.is_marquee_selecting) {
        sel_min_t = 1e9; sel_max_t = -1e9;
        sel_min_v = 1e9f; sel_max_v = -1e9f;
        for (size_t s : state.selected_indices) {
            if (s < points.size()) {
                sel_min_t = std::min(sel_min_t, points[s].time_beats);
                sel_max_t = std::max(sel_max_t, points[s].time_beats);
                sel_min_v = std::min(sel_min_v, points[s].value);
                sel_max_v = std::max(sel_max_v, points[s].value);
            }
        }
        float bx1 = beat_to_x(sel_min_t) - 8.0f;
        float bx2 = beat_to_x(sel_max_t) + 8.0f;
        float by_t = val_to_y(sel_max_v) - 8.0f;
        float by_b = val_to_y(sel_min_v) + 8.0f;

        draw_list->AddRect(ImVec2(bx1, by_t), ImVec2(bx2, by_b), ImColor(31, 97, 217, 130), 2.0f, 0, 1.0f);

        auto draw_handle = [&](ImVec2 h_pos, bool is_h_hov) {
            float hr = is_h_hov ? 5.5f : 4.0f;
            draw_list->AddRectFilled(ImVec2(h_pos.x - hr, h_pos.y - hr), ImVec2(h_pos.x + hr, h_pos.y + hr),
                                     is_h_hov ? ImColor(220, 38, 38, 255) : ImColor(255, 255, 255, 255), 1.0f);
            draw_list->AddRect(ImVec2(h_pos.x - hr, h_pos.y - hr), ImVec2(h_pos.x + hr, h_pos.y + hr),
                               ImColor(31, 97, 217, 255), 1.0f, 0, 1.5f);
        };
        draw_handle(ImVec2(bx1, 0.5f * (by_t + by_b)), hovered_handle == CurveEditorState::TransformHandle::Left);
        draw_handle(ImVec2(bx2, 0.5f * (by_t + by_b)), hovered_handle == CurveEditorState::TransformHandle::Right);
        draw_handle(ImVec2(0.5f * (bx1 + bx2), by_t), hovered_handle == CurveEditorState::TransformHandle::Top);
        draw_handle(ImVec2(0.5f * (bx1 + bx2), by_b), hovered_handle == CurveEditorState::TransformHandle::Bottom);

        char bbox_hdr[64];
        std::snprintf(bbox_hdr, sizeof(bbox_hdr), "%zu Nodes | Span: %.2f Beats", state.selected_indices.size(), sel_max_t - sel_min_t);
        draw_list->AddText(ImVec2(bx1, by_t - 14.0f), ImColor(31, 97, 217, 220), bbox_hdr);
    }

    // 9. Ghost Node & Tooltip (Hover Preview)
    if (is_mouse_on_curve && hovered_pt < 0 && hovered_tension < 0 && !state.is_dragging_nodes && !state.is_marquee_selecting) {
        double hb = snap_beat(mouse_beat);
        float hv = snap ? snap->evaluate(hb) : 1.0f;
        float gx = beat_to_x(hb);
        float gy = val_to_y(hv);

        draw_list->AddCircle(ImVec2(gx, gy), 6.0f, ImColor(31, 97, 217, 140), 0, 1.5f);
        char tip[64];
        if (target == routing::AutomationTarget::Pan) {
            if (std::abs(hv) < 0.01f) {
                std::snprintf(tip, sizeof(tip), "Bar %.2f | Center", (hb / 4.0) + 1.0);
            } else if (hv < 0.0f) {
                std::snprintf(tip, sizeof(tip), "Bar %.2f | L %.0f%%", (hb / 4.0) + 1.0, -hv * 100.0f);
            } else {
                std::snprintf(tip, sizeof(tip), "Bar %.2f | R %.0f%%", (hb / 4.0) + 1.0, hv * 100.0f);
            }
        } else if (target == routing::AutomationTarget::Aux1 || target == routing::AutomationTarget::Aux2) {
            std::snprintf(tip, sizeof(tip), "Bar %.2f | %.0f%% Send", (hb / 4.0) + 1.0, hv * 100.0f);
        } else if (target == routing::AutomationTarget::Pitch) {
            std::snprintf(tip, sizeof(tip), "Beat %.2f | %+.1f st", hb, hv);
        } else if (target == routing::AutomationTarget::PluginParam) {
            if (custom_unit) {
                std::snprintf(tip, sizeof(tip), "Bar %.2f | %.2f %s", (hb / 4.0) + 1.0, hv, custom_unit);
            } else {
                std::snprintf(tip, sizeof(tip), "Bar %.2f | %.2f", (hb / 4.0) + 1.0, hv);
            }
        } else {
            float db = linear_to_db(hv);
            std::snprintf(tip, sizeof(tip), "Bar %.2f | %.1f dB", (hb / 4.0) + 1.0, db);
        }
        draw_list->AddText(ImVec2(gx + 10.0f, gy - 16.0f), ImColor(31, 97, 217, 220), tip);
    }

    // 10. Live Transport Playhead Needle
    if (current_playhead_beat >= 0.0 && current_playhead_beat <= total_beats) {
        float hx = beat_to_x(current_playhead_beat);
        draw_list->AddLine(ImVec2(hx, y), ImVec2(hx, y + h), ImColor(20, 25, 35, 240), 1.5f);
        draw_list->AddTriangleFilled(ImVec2(hx - 5.0f, y), ImVec2(hx + 5.0f, y), ImVec2(hx, y + 8.0f), ImColor(20, 25, 35, 240));

        float cur_v = snap ? snap->evaluate(current_playhead_beat) : 1.0f;
        draw_list->AddCircleFilled(ImVec2(hx, val_to_y(cur_v)), 4.5f, ImColor(220, 38, 38, 255));
    }

    return modified;
}

} // namespace audio_core::ui
