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
// Professional Vertical Stereo/Mono dB Peak & RMS Meter
// Segmented gradient: Green (-60 to -12), Amber (-12 to 0), Red (> 0 clip)
// ============================================================================
inline void DrawDbMeter(ImDrawList* draw_list, ImVec2 pos, ImVec2 size,
                        float peak_l, float peak_r, float rms_l, float rms_r,
                        bool is_clipping, const char* label = nullptr) {
    const float x = pos.x;
    const float y = pos.y;
    const float w = size.x;
    const float h = size.y;

    // 1. Background dark well
    const ImU32 col_bg = ImColor(15, 18, 24, 255);
    const ImU32 col_border = ImColor(45, 52, 65, 255);
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
            ImU32 col_rms = (norm_rms > 0.909f) ? ImColor(255, 50, 80, 240) :
                            (norm_rms > 0.727f) ? ImColor(255, 179, 0, 240) :
                                                  ImColor(0, 229, 255, 240);
            draw_list->AddRectFilled(ImVec2(bar_x, bar_top), ImVec2(bar_x + bar_w, meter_y_bottom), col_rms, 1.0f);
        }

        if (norm_peak > 0.001f) {
            float peak_y = meter_y_bottom - (norm_peak * meter_h);
            ImU32 col_pk = (norm_peak >= 0.909f) ? ImColor(255, 80, 100, 255) :
                           (norm_peak > 0.727f)  ? ImColor(255, 220, 100, 255) :
                                                   ImColor(150, 245, 255, 255);
            draw_list->AddLine(ImVec2(bar_x, peak_y), ImVec2(bar_x + bar_w, peak_y), col_pk, 2.0f);
        }
    };

    draw_single_bar(x + 2.0f, norm_rms_l, norm_peak_l);
    draw_single_bar(x + 4.0f + bar_w, norm_rms_r, norm_peak_r);

    // 3. Clip LED indicators at the top
    const ImU32 col_clip_l = (peak_l >= 1.0f || is_clipping) ? ImColor(255, 30, 60, 255) : ImColor(60, 20, 25, 200);
    const ImU32 col_clip_r = (peak_r >= 1.0f || is_clipping) ? ImColor(255, 30, 60, 255) : ImColor(60, 20, 25, 200);
    draw_list->AddRectFilled(ImVec2(x + 2.0f, y + 2.0f), ImVec2(x + 2.0f + bar_w, y + 2.0f + clip_h), col_clip_l, 1.0f);
    draw_list->AddRectFilled(ImVec2(x + 4.0f + bar_w, y + 2.0f), ImVec2(x + w - 2.0f, y + 2.0f + clip_h), col_clip_r, 1.0f);

    // 4. 0 dB / -12 dB / -24 dB grid ticks
    const float y_0db = meter_y_bottom - (db_to_normalized(0.0f) * meter_h);
    const float y_12db = meter_y_bottom - (db_to_normalized(-12.0f) * meter_h);
    draw_list->AddLine(ImVec2(x, y_0db), ImVec2(x + w, y_0db), ImColor(180, 40, 40, 160), 1.0f);
    draw_list->AddLine(ImVec2(x, y_12db), ImVec2(x + w, y_12db), ImColor(100, 120, 140, 100), 1.0f);

    if (label && label[0] != '\0') {
        draw_list->AddText(ImVec2(x + 2.0f, y + h + 2.0f), ImColor(160, 170, 185, 255), label);
    }
}

// ============================================================================
// Hardware-Style Rotary Dial / Knob
// Circular arc indicator with mouse-drag sensitivity and center value text
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
        if (g.IO.KeyShift) delta *= 0.2f; // Fine tuning with shift key
        if (delta != 0.0f) {
            *value = std::clamp(*value + delta, v_min, v_max);
            changed = true;
        }
    }

    const ImVec2 center = ImVec2(bb.Min.x + radius, bb.Min.y + radius);
    ImDrawList* draw_list = window->DrawList;

    // Normalized parameter [0..1]
    const float norm = std::clamp((*value - v_min) / (v_max - v_min), 0.0f, 1.0f);
    constexpr float kAngleMin = -3.14159265f * 0.75f; // -135 deg
    constexpr float kAngleMax =  3.14159265f * 0.75f; // +135 deg
    const float current_angle = kAngleMin + norm * (kAngleMax - kAngleMin);

    // Background circle
    const ImU32 col_bg = hovered ? ImColor(35, 42, 54, 255) : ImColor(22, 26, 34, 255);
    const ImU32 col_border = held ? ImColor(0, 229, 255, 255) : (hovered ? ImColor(0, 180, 210, 200) : ImColor(50, 60, 75, 255));
    draw_list->AddCircleFilled(center, radius, col_bg, 32);
    draw_list->AddCircle(center, radius, col_border, 32, 1.5f);

    // Active arc
    constexpr int kArcSegments = 24;
    for (int i = 0; i < kArcSegments; ++i) {
        float a1 = kAngleMin + (float)i / kArcSegments * (current_angle - kAngleMin);
        float a2 = kAngleMin + (float)(i + 1) / kArcSegments * (current_angle - kAngleMin);
        if (a1 >= current_angle) break;
        if (a2 > current_angle) a2 = current_angle;

        ImVec2 p1(center.x + std::sin(a1) * (radius - 3.0f), center.y - std::cos(a1) * (radius - 3.0f));
        ImVec2 p2(center.x + std::sin(a2) * (radius - 3.0f), center.y - std::cos(a2) * (radius - 3.0f));
        draw_list->AddLine(p1, p2, ImColor(0, 229, 255, 220), 2.5f);
    }

    // Pointer indicator line
    const ImVec2 pointer_end(center.x + std::sin(current_angle) * (radius - 2.0f),
                             center.y - std::cos(current_angle) * (radius - 2.0f));
    const ImVec2 pointer_start(center.x + std::sin(current_angle) * (radius * 0.35f),
                               center.y - std::cos(current_angle) * (radius * 0.35f));
    draw_list->AddLine(pointer_start, pointer_end, ImColor(255, 255, 255, 255), 2.0f);

    // Label below
    char val_buf[32];
    std::snprintf(val_buf, sizeof(val_buf), "%.1f%s", *value, unit);
    ImVec2 val_size = ImGui::CalcTextSize(val_buf);
    draw_list->AddText(ImVec2(center.x - val_size.x * 0.5f, bb.Min.y + radius * 2.0f + 2.0f),
                       hovered ? ImColor(0, 229, 255, 255) : ImColor(160, 175, 195, 255), val_buf);

    return changed;
}

// ============================================================================
// Interactive High-Resolution Waveform & Slicer Display
// Renders min/max sample peak bars, playhead marker, and slice boundary lines
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

    // 1. Background grid well
    draw_list->AddRectFilled(pos, ImVec2(x + w, y + h), ImColor(12, 14, 18, 255), 2.0f);
    draw_list->AddRect(pos, ImVec2(x + w, y + h), ImColor(38, 44, 56, 255), 2.0f);

    // Center zero axis
    draw_list->AddLine(ImVec2(x, mid_y), ImVec2(x + w, mid_y), ImColor(45, 55, 70, 180), 1.0f);

    if (!samples || sample_count == 0) {
        const char* msg = "[ NO AUDIO SAMPLE LOADED ]";
        ImVec2 msg_size = ImGui::CalcTextSize(msg);
        draw_list->AddText(ImVec2(x + (w - msg_size.x) * 0.5f, mid_y - msg_size.y * 0.5f),
                           ImColor(80, 95, 115, 255), msg);
        return;
    }

    // 2. Render Min/Max Peak Columns across the display width
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

        draw_list->AddLine(ImVec2(x + px, y_top), ImVec2(x + px, y_bot), ImColor(0, 210, 240, 200), 1.0f);
    }

    // 3. Render Slicing Markers
    for (size_t i = 0; i < slice_points_ratio.size(); ++i) {
        float slice_x = x + (slice_points_ratio[i] * w);
        bool is_active = (static_cast<int>(i) == active_slice_idx);
        ImU32 col_slice = is_active ? ImColor(255, 180, 0, 255) : ImColor(255, 180, 0, 160);

        draw_list->AddLine(ImVec2(slice_x, y), ImVec2(slice_x, y + h), col_slice, is_active ? 2.0f : 1.0f);

        // Marker tag at top
        draw_list->AddTriangleFilled(ImVec2(slice_x - 4.0f, y),
                                     ImVec2(slice_x + 4.0f, y),
                                     ImVec2(slice_x, y + 7.0f), col_slice);
    }

    // 4. Playhead cursor
    if (playhead_ratio >= 0.0f && playhead_ratio <= 1.0f) {
        float play_x = x + (playhead_ratio * w);
        draw_list->AddLine(ImVec2(play_x, y), ImVec2(play_x, y + h), ImColor(255, 255, 255, 240), 1.5f);
        draw_list->AddTriangleFilled(ImVec2(play_x - 5.0f, y),
                                     ImVec2(play_x + 5.0f, y),
                                     ImVec2(play_x, y + 8.0f), ImColor(255, 255, 255, 255));
    }
}

// ============================================================================
// Trapezoidal Liquid ODE & ADSR Envelope Curve Visualizer
// ============================================================================
inline void DrawEnvelopeCurve(ImDrawList* draw_list, ImVec2 pos, ImVec2 size,
                              float attack, float decay, float sustain, float release, float tau) {
    const float x = pos.x;
    const float y = pos.y;
    const float w = size.x;
    const float h = size.y;
    const float bottom_y = y + h - 4.0f;
    const float top_y    = y + 4.0f;
    const float height_usable = bottom_y - top_y;

    draw_list->AddRectFilled(pos, ImVec2(x + w, y + h), ImColor(15, 17, 22, 255), 2.0f);
    draw_list->AddRect(pos, ImVec2(x + w, y + h), ImColor(38, 44, 56, 255), 2.0f);

    const float total_time = attack + decay + 200.0f + release; // fixed 200ms sustain stage
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

    // Curve rendering
    draw_list->AddLine(p0, p1, ImColor(0, 229, 255, 255), 2.0f);
    draw_list->AddLine(p1, p2, ImColor(0, 229, 255, 255), 2.0f);
    draw_list->AddLine(p2, p3, ImColor(255, 180, 0, 255), 2.0f);
    draw_list->AddLine(p3, p4, ImColor(0, 229, 255, 255), 2.0f);

    // Fill under curve
    ImVec2 fill_pts[7] = {
        p0, p1, p2, p3, p4,
        ImVec2(p4.x, bottom_y), ImVec2(p0.x, bottom_y)
    };
    draw_list->AddConvexPolyFilled(fill_pts, 7, ImColor(0, 229, 255, 35));

    // Marker knots
    draw_list->AddCircleFilled(p1, 3.5f, ImColor(255, 255, 255, 255));
    draw_list->AddCircleFilled(p2, 3.5f, ImColor(255, 180, 0, 255));
    draw_list->AddCircleFilled(p3, 3.5f, ImColor(255, 180, 0, 255));
}

// ============================================================================
// Real-Time Gas Watchdog & Circuit-Breaker Gauge
// ============================================================================
inline void DrawGasMeter(ImDrawList* draw_list, ImVec2 pos, ImVec2 size,
                         float gas_used, float gas_limit, bool is_tripped) {
    const float x = pos.x;
    const float y = pos.y;
    const float w = size.x;
    const float h = size.y;

    draw_list->AddRectFilled(pos, ImVec2(x + w, y + h), ImColor(16, 20, 26, 255), 2.0f);
    draw_list->AddRect(pos, ImVec2(x + w, y + h), is_tripped ? ImColor(255, 30, 60, 255) : ImColor(45, 55, 70, 255), 2.0f);

    const float ratio = std::clamp(gas_used / gas_limit, 0.0f, 1.0f);
    const float bar_w = (w - 4.0f) * ratio;

    ImU32 col_bar = is_tripped ? ImColor(255, 30, 60, 220) :
                    (ratio > 0.8f) ? ImColor(255, 180, 0, 220) :
                                     ImColor(0, 229, 255, 220);

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
                       is_tripped ? ImColor(255, 255, 255, 255) : ImColor(220, 230, 245, 255), text);
}

} // namespace audio_core::ui
