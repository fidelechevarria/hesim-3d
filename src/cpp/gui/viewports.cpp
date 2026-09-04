#include "gui_app.h"
#include <imgui.h>
#include <implot.h>
#include "icons_material_design.h"
#include <iostream>
#include <algorithm>

namespace hesim3d {

void GuiApp::render_multi_viewport_grid() {
    ImGuiViewport* vp = ImGui::GetMainViewport();
    float total_w = vp->Size.x;
    float total_h = vp->Size.y;

    float header_h = 30.0f;
    float upper_h = total_h - header_h - timeline_height_px_;
    float upper_y = header_h;

    float split_x = total_w * split_ratio_x_;
    float split_y = upper_h * split_ratio_y_;

    ImGuiIO& io = ImGui::GetIO();
    ImVec2 mouse_pos = io.MousePos;

    static bool s_dragging_col = false;
    static bool s_dragging_row = false;

    // Handle Splitter Drag State
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        if (active_layout_ == MultiViewLayout::VIEW_4_GRID || active_layout_ == MultiViewLayout::VIEW_2_SPLIT || active_layout_ == MultiViewLayout::VIEW_3_SPLIT) {
            if (std::abs(mouse_pos.x - split_x) < 8.0f && mouse_pos.y >= upper_y && mouse_pos.y <= upper_y + upper_h) {
                s_dragging_col = true;
            }
        }
        if (active_layout_ == MultiViewLayout::VIEW_4_GRID || active_layout_ == MultiViewLayout::VIEW_3_SPLIT) {
            if (std::abs(mouse_pos.y - (upper_y + split_y)) < 8.0f && mouse_pos.x >= 0.0f && mouse_pos.x <= total_w) {
                s_dragging_row = true;
            }
        }
    }

    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        s_dragging_col = false;
        s_dragging_row = false;
    }

    if (s_dragging_col) {
        split_ratio_x_ = std::clamp(mouse_pos.x / total_w, 0.15f, 0.85f);
        split_x = total_w * split_ratio_x_;
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    } else if ((active_layout_ == MultiViewLayout::VIEW_4_GRID || active_layout_ == MultiViewLayout::VIEW_2_SPLIT || active_layout_ == MultiViewLayout::VIEW_3_SPLIT) &&
               std::abs(mouse_pos.x - split_x) < 8.0f && mouse_pos.y >= upper_y && mouse_pos.y <= upper_y + upper_h) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    }

    if (s_dragging_row) {
        split_ratio_y_ = std::clamp((mouse_pos.y - upper_y) / upper_h, 0.15f, 0.85f);
        split_y = upper_h * split_ratio_y_;
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
    } else if ((active_layout_ == MultiViewLayout::VIEW_4_GRID || active_layout_ == MultiViewLayout::VIEW_3_SPLIT) &&
               std::abs(mouse_pos.y - (upper_y + split_y)) < 8.0f && mouse_pos.x >= 0.0f && mouse_pos.x <= total_w) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
    }

    float gap = 2.0f;

    // Render Quadrants according to active_layout_
    if (active_layout_ == MultiViewLayout::VIEW_1_SINGLE) {
        ImGui::SetNextWindowPos(ImVec2(0, upper_y), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(total_w, upper_h), ImGuiCond_Always);
        render_single_viewport(0, "Main Viewport (Maximized)", viewport_views_[0]);
    } else if (active_layout_ == MultiViewLayout::VIEW_2_SPLIT) {
        // Left Column (Full height)
        ImGui::SetNextWindowPos(ImVec2(0, upper_y), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(split_x - gap, upper_h), ImGuiCond_Always);
        render_single_viewport(0, "Left Viewport", viewport_views_[0]);

        // Right Column (Full height)
        ImGui::SetNextWindowPos(ImVec2(split_x + gap, upper_y), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(total_w - split_x - gap, upper_h), ImGuiCond_Always);
        render_single_viewport(1, "Right Viewport", viewport_views_[1]);
    } else if (active_layout_ == MultiViewLayout::VIEW_3_SPLIT) {
        // Left Column (Full height)
        ImGui::SetNextWindowPos(ImVec2(0, upper_y), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(split_x - gap, upper_h), ImGuiCond_Always);
        render_single_viewport(0, "Main Viewport", viewport_views_[0]);

        // Right Top
        ImGui::SetNextWindowPos(ImVec2(split_x + gap, upper_y), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(total_w - split_x - gap, split_y - gap), ImGuiCond_Always);
        render_single_viewport(1, "Top-Right Viewport", viewport_views_[1]);

        // Right Bottom
        ImGui::SetNextWindowPos(ImVec2(split_x + gap, upper_y + split_y + gap), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(total_w - split_x - gap, upper_h - split_y - gap), ImGuiCond_Always);
        render_single_viewport(2, "Bottom-Right Viewport", viewport_views_[2]);
    } else { // MultiViewLayout::VIEW_4_GRID (2x2 Grid)
        // Top-Left (Quad 0)
        ImGui::SetNextWindowPos(ImVec2(0, upper_y), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(split_x - gap, split_y - gap), ImGuiCond_Always);
        render_single_viewport(0, "Quadrant 1", viewport_views_[0]);

        // Top-Right (Quad 1)
        ImGui::SetNextWindowPos(ImVec2(split_x + gap, upper_y), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(total_w - split_x - gap, split_y - gap), ImGuiCond_Always);
        render_single_viewport(1, "Quadrant 2", viewport_views_[1]);

        // Bottom-Left (Quad 2)
        ImGui::SetNextWindowPos(ImVec2(0, upper_y + split_y + gap), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(split_x - gap, upper_h - split_y - gap), ImGuiCond_Always);
        render_single_viewport(2, "Quadrant 3", viewport_views_[2]);

        // Bottom-Right (Quad 3)
        ImGui::SetNextWindowPos(ImVec2(split_x + gap, upper_y + split_y + gap), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(total_w - split_x - gap, upper_h - split_y - gap), ImGuiCond_Always);
        render_single_viewport(3, "Quadrant 4", viewport_views_[3]);
    }

    // Draw clean visual splitter divider lines (hidden during simulation to avoid overlapping modals)
    if (!is_simulating_) {
        ImDrawList* bg = ImGui::GetBackgroundDrawList();
        if (active_layout_ == MultiViewLayout::VIEW_4_GRID || active_layout_ == MultiViewLayout::VIEW_2_SPLIT || active_layout_ == MultiViewLayout::VIEW_3_SPLIT) {
            ImU32 col_c = s_dragging_col ? IM_COL32(0, 190, 255, 255) : IM_COL32(45, 48, 56, 255);
            bg->AddLine(ImVec2(split_x, upper_y), ImVec2(split_x, upper_y + upper_h), col_c, s_dragging_col ? 3.0f : 2.0f);
        }
        if (active_layout_ == MultiViewLayout::VIEW_4_GRID || active_layout_ == MultiViewLayout::VIEW_3_SPLIT) {
            ImU32 row_c = s_dragging_row ? IM_COL32(0, 190, 255, 255) : IM_COL32(45, 48, 56, 255);
            bg->AddLine(ImVec2(0, upper_y + split_y), ImVec2(total_w, upper_y + split_y), row_c, s_dragging_row ? 3.0f : 2.0f);
        }
    }

    if (layout_settle_frames_ > 0) {
        --layout_settle_frames_;
    }
}

void GuiApp::render_single_viewport(int quad_idx, const std::string& name, ViewportContent content) {
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
                            ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoResize |
                            ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
    std::string win_id = "##ViewportWindow_" + std::to_string(quad_idx);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(4.0f, 4.0f));
    bool open = ImGui::Begin(win_id.c_str(), nullptr, flags);
    ImGui::PopStyleVar();

    if (open) {
        // Filter available views based on current application mode
        struct ViewportOption {
            ViewportContent content;
            const char* name;
        };

        std::vector<ViewportOption> available_views;
        if (current_mode_ == AppMode::TRAJECTORY_STUDIO) {
            available_views = {
                { ViewportContent::CAMERA_CLEAN,  ICON_MDI_CAMERA " Camera (Clean Look-Through)" },
                { ViewportContent::TOP_ORTHO,      ICON_MDI_VIEW_GRID " Top (Top Ortho X-Z)" },
                { ViewportContent::FRONT_ORTHO,    ICON_MDI_VIEW_AGENDA " Front (Front Ortho X-Y)" },
                { ViewportContent::SIDE_ORTHO,     ICON_MDI_VIEW_WEEK " Side (Side Ortho Z-Y)" },
                { ViewportContent::WORLD_3D_ORBIT, ICON_MDI_ROTATE_ORBIT " 3D Perspective" },
                { ViewportContent::IMU_TELEMETRY,  ICON_MDI_CHART_BELL_CURVE " IMU Telemetry (Kinematics)" }
            };
        } else {
            available_views = {
                { ViewportContent::CAMERA_CLEAN,         ICON_MDI_CAMERA " Camera (Ground Truth Ref)" },
                { ViewportContent::SIMULATED_APS_SENSOR, ICON_MDI_IMAGE " Simulated APS (Blur + Noise)" },
                { ViewportContent::EVS_ACCUMULATION,     ICON_MDI_LIGHTNING_BOLT " EVS Events (Accumulation)" },
                { ViewportContent::IMU_TELEMETRY,        ICON_MDI_CHART_BELL_CURVE " Sensor IMU (With Noise)" }
            };
        }

        int cur_view_idx = 0;
        bool found_view = false;
        for (size_t i = 0; i < available_views.size(); ++i) {
            if (available_views[i].content == viewport_views_[quad_idx]) {
                cur_view_idx = static_cast<int>(i);
                found_view = true;
                break;
            }
        }
        if (!found_view && !available_views.empty()) {
            viewport_views_[quad_idx] = available_views[0].content;
            cur_view_idx = 0;
        }

        std::vector<const char*> view_names;
        view_names.reserve(available_views.size());
        for (const auto& opt : available_views) {
            view_names.push_back(opt.name);
        }

        ImGui::SetNextItemWidth(220);
        std::string combo_id = "##ViewCombo_" + std::to_string(quad_idx);
        if (ImGui::Combo(combo_id.c_str(), &cur_view_idx, view_names.data(), static_cast<int>(view_names.size()))) {
            viewport_views_[quad_idx] = available_views[cur_view_idx].content;
            reset_viewport_resolutions();
        }

        ImGui::SameLine();
        if (viewport_views_[quad_idx] == ViewportContent::CAMERA_CLEAN) {
            std::string btn_id = ICON_MDI_FIT_TO_SCREEN " Frame##cam" + std::to_string(quad_idx);
            if (ImGui::SmallButton(btn_id.c_str())) compute_optimal_initial_camera();
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Reset camera view to initial framing (Shortcut: 'F')");
            ImGui::SameLine();
            if (current_mode_ == AppMode::TRAJECTORY_STUDIO) {
                ImGui::TextColored(ImVec4(0.45f, 0.85f, 1.0f, 1.0f), ICON_MDI_AXIS_ARROW " [Look-Through | Left: Pan | Right: Dolly | Mid: Orbit | Q/E: Roll]");
            } else {
                ImGui::TextColored(ImVec4(0.45f, 0.85f, 1.0f, 1.0f), ICON_MDI_CHECK_CIRCLE_OUTLINE " [Ground Truth Reference (Filament)]");
            }
        } else if (viewport_views_[quad_idx] == ViewportContent::SIMULATED_APS_SENSOR) {
            ImGui::TextColored(ImVec4(1.0f, 0.70f, 0.30f, 1.0f), ICON_MDI_CHIP " [Physical Sensor APS | Blur + Poisson Noise + CFA]");
        } else if (viewport_views_[quad_idx] == ViewportContent::EVS_ACCUMULATION) {
            ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.45f, 1.0f), ICON_MDI_LIGHTNING_BOLT " [EVS Event Slice | Red: ON (+1) | Blue: OFF (-1)]");
        } else if (viewport_views_[quad_idx] == ViewportContent::TOP_ORTHO) {
            ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.35f, 1.0f), ICON_MDI_VIEW_GRID " [Top Ortho | X-Z]");
            ImGui::SameLine();
            std::string btn_id = ICON_MDI_FIT_TO_SCREEN " Frame##top" + std::to_string(quad_idx);
            if (ImGui::SmallButton(btn_id.c_str())) frame_ortho_view(0);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Frame scene, camera and frustum (Shortcut: 'F')");
            ImGui::SameLine();
            ImGui::TextDisabled("| Pan: Drag | Zoom: Wheel | Edit: Drag KF");
        } else if (viewport_views_[quad_idx] == ViewportContent::SIDE_ORTHO) {
            ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.35f, 1.0f), ICON_MDI_VIEW_WEEK " [Side Ortho | Z-Y]");
            ImGui::SameLine();
            std::string btn_id = ICON_MDI_FIT_TO_SCREEN " Frame##side" + std::to_string(quad_idx);
            if (ImGui::SmallButton(btn_id.c_str())) frame_ortho_view(2);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Frame scene, camera and frustum (Shortcut: 'F')");
            ImGui::SameLine();
            ImGui::TextDisabled("| Pan: Drag | Zoom: Wheel | Edit: Drag KF");
        } else if (viewport_views_[quad_idx] == ViewportContent::FRONT_ORTHO) {
            ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.35f, 1.0f), ICON_MDI_VIEW_AGENDA " [Front Ortho | X-Y]");
            ImGui::SameLine();
            std::string btn_id = ICON_MDI_FIT_TO_SCREEN " Frame##front" + std::to_string(quad_idx);
            if (ImGui::SmallButton(btn_id.c_str())) frame_ortho_view(1);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Frame scene, camera and frustum (Shortcut: 'F')");
            ImGui::SameLine();
            ImGui::TextDisabled("| Pan: Drag | Zoom: Wheel | Edit: Drag KF");
        } else if (viewport_views_[quad_idx] == ViewportContent::IMU_TELEMETRY) {
            ImGui::TextColored(ImVec4(0.5f, 0.9f, 0.7f, 1.0f), ICON_MDI_CHART_BELL_CURVE " [IMU Telemetry | Gyroscope (rad/s) + Accelerometer (m/s²)]");
        }

        ImGui::Separator();

        ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
        ImVec2 avail_sz = ImGui::GetContentRegionAvail();
        ImVec2 canvas_p1 = ImVec2(canvas_p0.x + avail_sz.x, canvas_p0.y + avail_sz.y);
        ImDrawList* draw_list = ImGui::GetWindowDrawList();

        auto render_aspect_image = [&](uint32_t tex_id, uint32_t tex_w, uint32_t tex_h) {
            if (tex_id != 0 && avail_sz.x > 10 && avail_sz.y > 10) {
                float tex_aspect = static_cast<float>(tex_w) / static_cast<float>(tex_h);
                float avail_aspect = avail_sz.x / avail_sz.y;
                ImVec2 draw_size = avail_sz;
                if (avail_aspect > tex_aspect) {
                    draw_size.x = avail_sz.y * tex_aspect;
                } else {
                    draw_size.y = avail_sz.x / tex_aspect;
                }
                float offset_x = (avail_sz.x - draw_size.x) * 0.5f;
                float offset_y = (avail_sz.y - draw_size.y) * 0.5f;
                if (offset_x > 0.0f) ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset_x);
                if (offset_y > 0.0f) ImGui::SetCursorPosY(ImGui::GetCursorPosY() + offset_y);
                ImVec2 img_p0 = ImGui::GetCursorScreenPos();
                ImGui::Image((ImTextureID)(intptr_t)tex_id, draw_size);

                // Draw central target reticle
                draw_list->PushClipRect(canvas_p0, canvas_p1, true);
                float cx = img_p0.x + draw_size.x * 0.5f;
                float cy = img_p0.y + draw_size.y * 0.5f;
                draw_list->AddCircle(ImVec2(cx, cy), 14.0f, IM_COL32(95, 120, 240, 180), 16, 1.5f);
                draw_list->AddLine(ImVec2(cx - 20, cy), ImVec2(cx - 5, cy), IM_COL32(95, 120, 240, 180), 1.5f);
                draw_list->AddLine(ImVec2(cx + 5, cy), ImVec2(cx + 20, cy), IM_COL32(95, 120, 240, 180), 1.5f);
                draw_list->AddLine(ImVec2(cx, cy - 20), ImVec2(cx, cy - 5), IM_COL32(95, 120, 240, 180), 1.5f);
                draw_list->AddLine(ImVec2(cx, cy + 5), ImVec2(cx, cy + 20), IM_COL32(95, 120, 240, 180), 1.5f);
                draw_list->PopClipRect();
            }
        };

        // Render viewport contents
        switch (viewport_views_[quad_idx]) {
            case ViewportContent::CAMERA_CLEAN: {
                if (current_mode_ == AppMode::SENSOR_SIMULATION) {
                    render_aspect_image(sensor_texture_id_, sensor_tex_w_, sensor_tex_h_);
                    break;
                }

                if (avail_sz.x > 10.0f && avail_sz.y > 10.0f) {
                    // Full-bleed viewport render (Google Earth Studio / Blender Passepartout style)
                    uint32_t tw = (static_cast<uint32_t>(avail_sz.x) + 3) & ~3u;
                    uint32_t th = (static_cast<uint32_t>(avail_sz.y) + 1) & ~1u;

                    int thresh = (layout_settle_frames_ > 0) ? 0 : 8;
                    if (std::abs(static_cast<int>(tw) - static_cast<int>(camera_render_w_)) > thresh ||
                        std::abs(static_cast<int>(th) - static_cast<int>(camera_render_h_)) > thresh) {
                        resize_camera_render(tw, th);
                    }

                    if (sensor_texture_id_ != 0) {
                        // 1. Render 3D scene across full viewport canvas
                        ImGui::SetCursorScreenPos(canvas_p0);
                        ImGui::Image((ImTextureID)(intptr_t)sensor_texture_id_, avail_sz);

                        // 2. Compute physical sensor frame bounding rectangle
                        float sensor_aspect = static_cast<float>(sensor_tex_w_) / std::max(1.0f, static_cast<float>(sensor_tex_h_));
                        float avail_aspect = avail_sz.x / avail_sz.y;
                        float frame_w = avail_sz.x;
                        float frame_h = avail_sz.y;
                        float frame_x0 = canvas_p0.x;
                        float frame_y0 = canvas_p0.y;

                        if (avail_aspect > sensor_aspect) {
                            // Viewport is wider than sensor: pillarbox bands on left & right
                            frame_w = avail_sz.y * sensor_aspect;
                            frame_h = avail_sz.y;
                            frame_x0 = canvas_p0.x + (avail_sz.x - frame_w) * 0.5f;
                            frame_y0 = canvas_p0.y;
                        } else {
                            // Viewport is taller than sensor: letterbox bands on top & bottom
                            frame_w = avail_sz.x;
                            frame_h = avail_sz.x / sensor_aspect;
                            frame_x0 = canvas_p0.x;
                            frame_y0 = canvas_p0.y + (avail_sz.y - frame_h) * 0.5f;
                        }
                        float frame_x1 = frame_x0 + frame_w;
                        float frame_y1 = frame_y0 + frame_h;

                        draw_list->PushClipRect(canvas_p0, canvas_p1, true);

                        // 3. Passepartout shaded overlay for inactive sensor bands (Google Earth Studio style)
                        const ImU32 shade_col = IM_COL32(10, 12, 16, 175); // ~68% dark scrim
                        if (avail_aspect > sensor_aspect) {
                            if (frame_x0 > canvas_p0.x) {
                                draw_list->AddRectFilled(canvas_p0, ImVec2(frame_x0, canvas_p1.y), shade_col);
                            }
                            if (canvas_p1.x > frame_x1) {
                                draw_list->AddRectFilled(ImVec2(frame_x1, canvas_p0.y), canvas_p1, shade_col);
                            }
                        } else {
                            if (frame_y0 > canvas_p0.y) {
                                draw_list->AddRectFilled(canvas_p0, ImVec2(canvas_p1.x, frame_y0), shade_col);
                            }
                            if (canvas_p1.y > frame_y1) {
                                draw_list->AddRectFilled(ImVec2(canvas_p0.x, frame_y1), canvas_p1, shade_col);
                            }
                        }

                        // 4. Sensor Frame Boundary (crisp subtle border)
                        const ImU32 frame_border_col = IM_COL32(230, 240, 255, 160);
                        draw_list->AddRect(ImVec2(frame_x0, frame_y0), ImVec2(frame_x1, frame_y1), frame_border_col, 0.0f, 0, 1.5f);

                        // Corner brackets for high-end studio camera feel
                        float corner_len = std::min(18.0f, std::min(frame_w, frame_h) * 0.12f);
                        const ImU32 corner_col = IM_COL32(0, 190, 255, 220);
                        // Top-Left
                        draw_list->AddLine(ImVec2(frame_x0, frame_y0), ImVec2(frame_x0 + corner_len, frame_y0), corner_col, 2.5f);
                        draw_list->AddLine(ImVec2(frame_x0, frame_y0), ImVec2(frame_x0, frame_y0 + corner_len), corner_col, 2.5f);
                        // Top-Right
                        draw_list->AddLine(ImVec2(frame_x1, frame_y0), ImVec2(frame_x1 - corner_len, frame_y0), corner_col, 2.5f);
                        draw_list->AddLine(ImVec2(frame_x1, frame_y0), ImVec2(frame_x1, frame_y0 + corner_len), corner_col, 2.5f);
                        // Bottom-Left
                        draw_list->AddLine(ImVec2(frame_x0, frame_y1), ImVec2(frame_x0 + corner_len, frame_y1), corner_col, 2.5f);
                        draw_list->AddLine(ImVec2(frame_x0, frame_y1), ImVec2(frame_x0, frame_y1 - corner_len), corner_col, 2.5f);
                        // Bottom-Right
                        draw_list->AddLine(ImVec2(frame_x1, frame_y1), ImVec2(frame_x1 - corner_len, frame_y1), corner_col, 2.5f);
                        draw_list->AddLine(ImVec2(frame_x1, frame_y1), ImVec2(frame_x1, frame_y1 - corner_len), corner_col, 2.5f);

                        // 5. Subtle Sensor Info Badge
                        char badge_buf[64];
                        std::snprintf(badge_buf, sizeof(badge_buf), "%u x %u (Sensor Gate)", sensor_tex_w_, sensor_tex_h_);
                        ImVec2 badge_sz = ImGui::CalcTextSize(badge_buf);
                        ImVec2 badge_pos = ImVec2(frame_x1 - badge_sz.x - 8.0f, frame_y1 - badge_sz.y - 6.0f);
                        if (badge_pos.x > frame_x0 + 10.0f && badge_pos.y > frame_y0 + 10.0f) {
                            draw_list->AddRectFilled(
                                ImVec2(badge_pos.x - 4.0f, badge_pos.y - 2.0f),
                                ImVec2(badge_pos.x + badge_sz.x + 4.0f, badge_pos.y + badge_sz.y + 2.0f),
                                IM_COL32(15, 18, 24, 180), 3.0f
                            );
                            draw_list->AddText(badge_pos, IM_COL32(180, 210, 255, 210), badge_buf);
                        }

                        // 6. Center target reticle
                        float cx = frame_x0 + frame_w * 0.5f;
                        float cy = frame_y0 + frame_h * 0.5f;
                        draw_list->AddCircle(ImVec2(cx, cy), 14.0f, IM_COL32(95, 120, 240, 180), 16, 1.5f);
                        draw_list->AddLine(ImVec2(cx - 20, cy), ImVec2(cx - 5, cy), IM_COL32(95, 120, 240, 180), 1.5f);
                        draw_list->AddLine(ImVec2(cx + 5, cy), ImVec2(cx + 20, cy), IM_COL32(95, 120, 240, 180), 1.5f);
                        draw_list->AddLine(ImVec2(cx, cy - 20), ImVec2(cx, cy - 5), IM_COL32(95, 120, 240, 180), 1.5f);
                        draw_list->AddLine(ImVec2(cx, cy + 5), ImVec2(cx, cy + 20), IM_COL32(95, 120, 240, 180), 1.5f);

                        draw_list->PopClipRect();
                    }
                }
                handle_camera_mouse_input(canvas_p0.x, canvas_p0.y, canvas_p1.x, canvas_p1.y);
                break;
            }

            case ViewportContent::SIMULATED_APS_SENSOR:
                if (!simulation_has_data_) {
                    ImGui::SetCursorScreenPos(ImVec2(canvas_p0.x + 24.0f, canvas_p0.y + 30.0f));
                    ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.35f, 1.0f), ICON_MDI_INFORMATION_OUTLINE " Simulation data not baked yet.");
                    ImGui::SetCursorScreenPos(ImVec2(canvas_p0.x + 24.0f, canvas_p0.y + 55.0f));
                    ImGui::TextDisabled("Physical motion blur & sensor noise require baking.");
                    ImGui::SetCursorScreenPos(ImVec2(canvas_p0.x + 24.0f, canvas_p0.y + 85.0f));
                    if (keyframes_.size() >= 2) {
                        if (ImGui::Button(ICON_MDI_ROCKET_LAUNCH " Bake Simulation Now##aps", ImVec2(210, 28))) {
                            start_simulation_bake();
                        }
                    } else {
                        ImGui::TextDisabled("Capture >= 2 keyframes in Trajectory Studio to bake.");
                    }
                } else {
                    render_aspect_image(sim_aps_texture_id_, sensor_tex_w_, sensor_tex_h_);
                }
                break;

            case ViewportContent::EVS_ACCUMULATION: {
                if (!simulation_has_data_) {
                    ImGui::SetCursorScreenPos(ImVec2(canvas_p0.x + 24.0f, canvas_p0.y + 30.0f));
                    ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.35f, 1.0f), ICON_MDI_INFORMATION_OUTLINE " No neuromorphic events generated yet.");
                    ImGui::SetCursorScreenPos(ImVec2(canvas_p0.x + 24.0f, canvas_p0.y + 55.0f));
                    ImGui::TextDisabled("Microsecond event stream requires physical simulation bake.");
                    ImGui::SetCursorScreenPos(ImVec2(canvas_p0.x + 24.0f, canvas_p0.y + 85.0f));
                    if (keyframes_.size() >= 2) {
                        if (ImGui::Button(ICON_MDI_ROCKET_LAUNCH " Bake Simulation Now##evs", ImVec2(210, 28))) {
                            start_simulation_bake();
                        }
                    } else {
                        ImGui::TextDisabled("Capture >= 2 keyframes in Trajectory Studio to bake.");
                    }
                } else {
                    render_aspect_image(evs_texture_id_, sensor_tex_w_, sensor_tex_h_);
                }
                break;
            }

            case ViewportContent::TOP_ORTHO:
                draw_ortho_map(0, draw_list, canvas_p0.x, canvas_p0.y, canvas_p1.x, canvas_p1.y);
                break;

            case ViewportContent::FRONT_ORTHO:
                draw_ortho_map(1, draw_list, canvas_p0.x, canvas_p0.y, canvas_p1.x, canvas_p1.y);
                break;

            case ViewportContent::SIDE_ORTHO:
                draw_ortho_map(2, draw_list, canvas_p0.x, canvas_p0.y, canvas_p1.x, canvas_p1.y);
                break;

            case ViewportContent::WORLD_3D_ORBIT:
                draw_ortho_map(0, draw_list, canvas_p0.x, canvas_p0.y, canvas_p1.x, canvas_p1.y);
                break;

            case ViewportContent::IMU_TELEMETRY:
                render_imu_plots_content();
                break;
        }
    }
    ImGui::End();
}

void GuiApp::render_imu_plots_content() {
    float avail_y = ImGui::GetContentRegionAvail().y;
    float plot_h = std::max(60.0f, (avail_y - 25.0f) * 0.5f);
    double dur = std::max(0.5, config_.duration_sec);

    const double* t_data = nullptr;
    const double* gx_data = nullptr;
    const double* gy_data = nullptr;
    const double* gz_data = nullptr;
    const double* ax_data = nullptr;
    const double* ay_data = nullptr;
    const double* az_data = nullptr;
    int count = 0;

    if (!imu_curve_time_.empty()) {
        t_data = imu_curve_time_.data();
        gx_data = imu_curve_gyro_x_.data();
        gy_data = imu_curve_gyro_y_.data();
        gz_data = imu_curve_gyro_z_.data();
        ax_data = imu_curve_acc_x_.data();
        ay_data = imu_curve_acc_y_.data();
        az_data = imu_curve_acc_z_.data();
        count = static_cast<int>(imu_curve_time_.size());
    } else if (!plot_time_.empty()) {
        t_data = plot_time_.data();
        gx_data = plot_gyro_x_.data();
        gy_data = plot_gyro_y_.data();
        gz_data = plot_gyro_z_.data();
        ax_data = plot_acc_x_.data();
        ay_data = plot_acc_y_.data();
        az_data = plot_acc_z_.data();
        count = static_cast<int>(plot_time_.size());
    }

    double cur_t = current_time_sec_;

    if (ImPlot::BeginPlot("Gyroscope (rad/s)##imu", ImVec2(-1, plot_h))) {
        ImPlot::SetupAxes("Time [s]", "rad/s", ImPlotAxisFlags_None, ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxisLimits(ImAxis_X1, 0.0, dur, ImPlotCond_Always);
        if (count > 0) {
            ImPlot::PlotLine("Gyro X", t_data, gx_data, count);
            ImPlot::PlotLine("Gyro Y", t_data, gy_data, count);
            ImPlot::PlotLine("Gyro Z", t_data, gz_data, count);
        }
        ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.85f, 0.0f, 0.9f), 1.5f);
        ImPlot::PlotInfLines("##Playhead_g", &cur_t, 1);
        ImPlot::EndPlot();
    }

    if (ImPlot::BeginPlot("Accelerometer (m/s^2)##imu", ImVec2(-1, plot_h))) {
        ImPlot::SetupAxes("Time [s]", "m/s^2", ImPlotAxisFlags_None, ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxisLimits(ImAxis_X1, 0.0, dur, ImPlotCond_Always);
        if (count > 0) {
            ImPlot::PlotLine("Acc X", t_data, ax_data, count);
            ImPlot::PlotLine("Acc Y", t_data, ay_data, count);
            ImPlot::PlotLine("Acc Z", t_data, az_data, count);
        }
        ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.85f, 0.0f, 0.9f), 1.5f);
        ImPlot::PlotInfLines("##Playhead_a", &cur_t, 1);
        ImPlot::EndPlot();
    }
}

} // namespace hesim3d
