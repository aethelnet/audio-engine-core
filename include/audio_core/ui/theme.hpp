#pragma once

#include "imgui.h"

namespace audio_core::ui {

// ============================================================================
// Orderly Architect's Drafting Desk Theme
// Warm Alabaster Drafting Table, Crisp Vellum Cards, Technical Graphite Lines,
// Blueprint Cobalt Accents, Drafting Amber, and Architectural Precision.
// ============================================================================
inline void apply_architect_desk_theme() {
    ImGuiStyle& style = ImGui::GetStyle();

    // Architectural Drafting Precision: crisp edges, subtle 2px rounding
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
    style.FramePadding      = ImVec2(6.0f, 5.0f);
    style.ItemSpacing       = ImVec2(8.0f, 6.0f);
    style.ItemInnerSpacing  = ImVec2(6.0f, 4.0f);
    style.ScrollbarSize     = 10.0f;
    style.GrabMinSize       = 10.0f;

    ImVec4* colors = style.Colors;

    // 1. Drafting Table Palette
    const ImVec4 kDeskSurface       = ImVec4(0.94f, 0.95f, 0.96f, 1.00f); // #F0F2F5 (Drafting mat)
    const ImVec4 kVellumWhite       = ImVec4(0.99f, 0.99f, 1.00f, 1.00f); // #FCFDFE (Vellum paper card)
    const ImVec4 kElementRecessed   = ImVec4(0.91f, 0.92f, 0.94f, 1.00f); // #E8EBEE (Recessed slot)
    const ImVec4 kElementHovered    = ImVec4(0.85f, 0.88f, 0.92f, 1.00f); // #D9E0EB
    const ImVec4 kElementActive     = ImVec4(0.78f, 0.82f, 0.88f, 1.00f); // #C7D1E0

    // 2. Graphite Pencil & Technical Line Borders
    const ImVec4 kBorderGraphite    = ImVec4(0.78f, 0.80f, 0.84f, 1.00f); // #C7CCD6 (Pencil line)
    const ImVec4 kBorderSubtle      = ImVec4(0.86f, 0.88f, 0.90f, 1.00f); // #DCE0E6

    // 3. Technical Ink & Drafting Pens
    const ImVec4 kDraftingInkDark   = ImVec4(0.10f, 0.12f, 0.16f, 1.00f); // #1A1E28 (Deep drafting ink)
    const ImVec4 kDraftingInkMuted  = ImVec4(0.42f, 0.46f, 0.52f, 1.00f); // #6B7584 (Technical notation)
    const ImVec4 kBlueprintCobalt   = ImVec4(0.12f, 0.38f, 0.85f, 1.00f); // #1F61D9 (Classic blueprint blue)
    const ImVec4 kBlueprintLight    = ImVec4(0.24f, 0.50f, 0.95f, 1.00f);
    const ImVec4 kDraftingAmber     = ImVec4(0.85f, 0.48f, 0.05f, 1.00f); // #D97B0D (Ruler ochre)
    const ImVec4 kDraftingCrimson   = ImVec4(0.85f, 0.18f, 0.22f, 1.00f); // #D92E38 (Red pen)
    (void)kDraftingAmber;
    (void)kDraftingCrimson;

    // Assign to ImGui Colors
    colors[ImGuiCol_Text]                  = kDraftingInkDark;
    colors[ImGuiCol_TextDisabled]          = kDraftingInkMuted;
    colors[ImGuiCol_WindowBg]              = kDeskSurface;
    colors[ImGuiCol_ChildBg]               = kVellumWhite;
    colors[ImGuiCol_PopupBg]               = kVellumWhite;
    colors[ImGuiCol_Border]                = kBorderGraphite;
    colors[ImGuiCol_BorderShadow]          = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);

    colors[ImGuiCol_FrameBg]               = kElementRecessed;
    colors[ImGuiCol_FrameBgHovered]        = kElementHovered;
    colors[ImGuiCol_FrameBgActive]         = kElementActive;

    colors[ImGuiCol_TitleBg]               = kDeskSurface;
    colors[ImGuiCol_TitleBgActive]         = kDeskSurface;
    colors[ImGuiCol_TitleBgCollapsed]      = kDeskSurface;

    colors[ImGuiCol_MenuBarBg]             = kVellumWhite;
    colors[ImGuiCol_ScrollbarBg]           = kDeskSurface;
    colors[ImGuiCol_ScrollbarGrab]         = kBorderGraphite;
    colors[ImGuiCol_ScrollbarGrabHovered]  = kDraftingInkMuted;
    colors[ImGuiCol_ScrollbarGrabActive]   = kBlueprintCobalt;

    colors[ImGuiCol_CheckMark]             = kBlueprintCobalt;
    colors[ImGuiCol_SliderGrab]            = kBlueprintCobalt;
    colors[ImGuiCol_SliderGrabActive]      = kBlueprintLight;

    colors[ImGuiCol_Button]                = kElementRecessed;
    colors[ImGuiCol_ButtonHovered]         = kElementHovered;
    colors[ImGuiCol_ButtonActive]          = kElementActive;

    colors[ImGuiCol_Header]                = kElementHovered;
    colors[ImGuiCol_HeaderHovered]         = kElementActive;
    colors[ImGuiCol_HeaderActive]          = kBlueprintCobalt;

    colors[ImGuiCol_Separator]             = kBorderSubtle;
    colors[ImGuiCol_SeparatorHovered]      = kBorderGraphite;
    colors[ImGuiCol_SeparatorActive]       = kBlueprintCobalt;

    colors[ImGuiCol_ResizeGrip]            = kBorderGraphite;
    colors[ImGuiCol_ResizeGripHovered]     = kBlueprintCobalt;
    colors[ImGuiCol_ResizeGripActive]      = kBlueprintLight;

    // Clean drafting tabs
    colors[ImGuiCol_Tab]                   = kElementRecessed;
    colors[ImGuiCol_TabHovered]            = kElementHovered;
    colors[ImGuiCol_TabActive]             = kVellumWhite;
    colors[ImGuiCol_TabUnfocused]          = kElementRecessed;
    colors[ImGuiCol_TabUnfocusedActive]    = kVellumWhite;

    colors[ImGuiCol_PlotLines]             = kBlueprintCobalt;
    colors[ImGuiCol_PlotLinesHovered]      = kDraftingAmber;
    colors[ImGuiCol_PlotHistogram]         = kBlueprintCobalt;
    colors[ImGuiCol_PlotHistogramHovered]  = kDraftingCrimson;

    colors[ImGuiCol_TableHeaderBg]         = kElementRecessed;
    colors[ImGuiCol_TableBorderStrong]     = kBorderGraphite;
    colors[ImGuiCol_TableBorderLight]      = kBorderSubtle;
    colors[ImGuiCol_TableRowBg]            = kVellumWhite;
    colors[ImGuiCol_TableRowBgAlt]         = ImVec4(0.96f, 0.97f, 0.98f, 1.00f);
}

} // namespace audio_core::ui
