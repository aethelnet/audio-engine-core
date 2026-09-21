#pragma once

#include "imgui.h"

namespace audio_core::ui {

// ============================================================================
// Cyberpunk / Brutalist High-Contrast Audio Workstation Theme
// Matte Carbon Charcoal Backgrounds, Electric Cyan, Warm Amber, Signal Red
// ============================================================================
inline void apply_brutalist_theme() {
    ImGuiStyle& style = ImGui::GetStyle();

    // Geometric, razor-sharp brutalism: low roundings, crisp borders
    style.WindowRounding    = 2.0f;
    style.ChildRounding     = 2.0f;
    style.FrameRounding     = 2.0f;
    style.PopupRounding     = 2.0f;
    style.ScrollbarRounding = 2.0f;
    style.GrabRounding      = 2.0f;
    style.TabRounding       = 2.0f;

    style.WindowBorderSize  = 1.0f;
    style.ChildBorderSize   = 1.0f;
    style.PopupBorderSize   = 1.0f;
    style.FrameBorderSize   = 1.0f;
    style.TabBorderSize     = 1.0f;

    style.WindowPadding     = ImVec2(10.0f, 10.0f);
    style.FramePadding      = ImVec2(6.0f, 4.0f);
    style.ItemSpacing       = ImVec2(8.0f, 6.0f);
    style.ItemInnerSpacing  = ImVec2(6.0f, 4.0f);
    style.ScrollbarSize     = 10.0f;
    style.GrabMinSize       = 10.0f;

    ImVec4* colors = style.Colors;

    // Palette Definition
    const ImVec4 kBgDeepBlack      = ImVec4(0.04f, 0.05f, 0.07f, 1.00f); // #0A0D12
    const ImVec4 kBgPanel          = ImVec4(0.08f, 0.09f, 0.12f, 1.00f); // #14171F
    const ImVec4 kBgElement        = ImVec4(0.12f, 0.14f, 0.18f, 1.00f); // #1F242E
    const ImVec4 kBgHovered        = ImVec4(0.17f, 0.20f, 0.26f, 1.00f);
    const ImVec4 kBgActive         = ImVec4(0.22f, 0.26f, 0.33f, 1.00f);

    const ImVec4 kBorderMuted      = ImVec4(0.20f, 0.23f, 0.29f, 1.00f); // Crisp panel edges
    const ImVec4 kBorderAccent     = ImVec4(0.00f, 0.90f, 1.00f, 0.80f); // Neon Cyan border
    (void)kBorderAccent;

    const ImVec4 kCyanAccent       = ImVec4(0.00f, 0.90f, 1.00f, 1.00f); // #00E5FF
    const ImVec4 kCyanDimmed       = ImVec4(0.00f, 0.55f, 0.65f, 1.00f);
    const ImVec4 kAmberAccent      = ImVec4(1.00f, 0.70f, 0.00f, 1.00f); // #FFB300
    const ImVec4 kRedSignal        = ImVec4(1.00f, 0.20f, 0.35f, 1.00f); // #FF3359

    const ImVec4 kTextPrimary      = ImVec4(0.92f, 0.94f, 0.96f, 1.00f);
    const ImVec4 kTextMuted        = ImVec4(0.50f, 0.55f, 0.62f, 1.00f);

    colors[ImGuiCol_Text]                  = kTextPrimary;
    colors[ImGuiCol_TextDisabled]          = kTextMuted;
    colors[ImGuiCol_WindowBg]              = kBgDeepBlack;
    colors[ImGuiCol_ChildBg]               = kBgPanel;
    colors[ImGuiCol_PopupBg]               = kBgPanel;
    colors[ImGuiCol_Border]                = kBorderMuted;
    colors[ImGuiCol_BorderShadow]          = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);

    colors[ImGuiCol_FrameBg]               = kBgElement;
    colors[ImGuiCol_FrameBgHovered]        = kBgHovered;
    colors[ImGuiCol_FrameBgActive]         = kBgActive;

    colors[ImGuiCol_TitleBg]               = kBgPanel;
    colors[ImGuiCol_TitleBgActive]         = ImVec4(0.06f, 0.10f, 0.14f, 1.00f);
    colors[ImGuiCol_TitleBgCollapsed]      = kBgDeepBlack;

    colors[ImGuiCol_MenuBarBg]             = kBgPanel;
    colors[ImGuiCol_ScrollbarBg]           = kBgDeepBlack;
    colors[ImGuiCol_ScrollbarGrab]         = kBgElement;
    colors[ImGuiCol_ScrollbarGrabHovered]  = kBgHovered;
    colors[ImGuiCol_ScrollbarGrabActive]   = kCyanDimmed;

    colors[ImGuiCol_CheckMark]             = kCyanAccent;
    colors[ImGuiCol_SliderGrab]            = kCyanAccent;
    colors[ImGuiCol_SliderGrabActive]      = ImVec4(0.30f, 1.00f, 1.00f, 1.00f);

    colors[ImGuiCol_Button]                = kBgElement;
    colors[ImGuiCol_ButtonHovered]         = kBgHovered;
    colors[ImGuiCol_ButtonActive]          = kCyanDimmed;

    colors[ImGuiCol_Header]                = kBgHovered;
    colors[ImGuiCol_HeaderHovered]         = kCyanDimmed;
    colors[ImGuiCol_HeaderActive]          = kCyanAccent;

    colors[ImGuiCol_Separator]             = kBorderMuted;
    colors[ImGuiCol_SeparatorHovered]      = kCyanDimmed;
    colors[ImGuiCol_SeparatorActive]       = kCyanAccent;

    colors[ImGuiCol_ResizeGrip]            = kBgElement;
    colors[ImGuiCol_ResizeGripHovered]     = kCyanDimmed;
    colors[ImGuiCol_ResizeGripActive]      = kCyanAccent;

    colors[ImGuiCol_Tab]                   = kBgElement;
    colors[ImGuiCol_TabHovered]            = kBgHovered;
    colors[ImGuiCol_TabActive]             = ImVec4(0.08f, 0.16f, 0.22f, 1.00f);
    colors[ImGuiCol_TabUnfocused]          = kBgPanel;
    colors[ImGuiCol_TabUnfocusedActive]    = kBgElement;

    colors[ImGuiCol_PlotLines]             = kCyanAccent;
    colors[ImGuiCol_PlotLinesHovered]      = kAmberAccent;
    colors[ImGuiCol_PlotHistogram]         = kAmberAccent;
    colors[ImGuiCol_PlotHistogramHovered]  = kRedSignal;

    colors[ImGuiCol_TableHeaderBg]         = kBgElement;
    colors[ImGuiCol_TableBorderStrong]     = kBorderMuted;
    colors[ImGuiCol_TableBorderLight]      = ImVec4(0.14f, 0.16f, 0.20f, 1.00f);
    colors[ImGuiCol_TableRowBg]            = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_TableRowBgAlt]         = ImVec4(1.00f, 1.00f, 1.00f, 0.02f);
}

} // namespace audio_core::ui
