#pragma once

#include "imgui.h"
#include "imgui_internal.h"
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <cstdio>

namespace audio_core::ui {

inline float linear_to_db(float lin) noexcept {
    if (lin <= 0.00001f) return -96.0f;
    return 20.0f * std::log10(lin);
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

} // namespace audio_core::ui
