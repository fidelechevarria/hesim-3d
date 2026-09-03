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

    float header_h = 42.0f;
    float upper_h = total_h - header_h - timeline_height_px_ - 4.0f;
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
        render_single_viewport(0, "Main Viewport (Maximized)", viewport_views_[1]);
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

    // Draw clean visual splitter divider lines
    ImDrawList* fg = ImGui::GetForegroundDrawList();
    if (active_layout_ == MultiViewLayout::VIEW_4_GRID || active_layout_ == MultiViewLayout::VIEW_2_SPLIT || active_layout_ == MultiViewLayout::VIEW_3_SPLIT) {
        ImU32 col_c = s_dragging_col ? IM_COL32(0, 190, 255, 255) : IM_COL32(45, 48, 56, 255);
        fg->AddLine(ImVec2(split_x, upper_y), ImVec2(split_x, upper_y + upper_h), col_c, s_dragging_col ? 3.0f : 2.0f);
    }
    if (active_layout_ == MultiViewLayout::VIEW_4_GRID || active_layout_ == MultiViewLayout::VIEW_3_SPLIT) {
        ImU32 row_c = s_dragging_row ? IM_COL32(0, 190, 255, 255) : IM_COL32(45, 48, 56, 255);
        fg->AddLine(ImVec2(0, upper_y + split_y), ImVec2(total_w, upper_y + split_y), row_c, s_dragging_row ? 3.0f : 2.0f);
    }
}

void GuiApp::render_single_viewport(int quad_idx, const std::string& name, ViewportContent content) {
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;
    std::string win_id = "##ViewportWindow_" + std::to_string(quad_idx);

    if (ImGui::Begin(win_id.c_str(), nullptr, flags)) {
        // Top Toolbar inside each Viewport
        const char* view_names[] = {
            ICON_MDI_CAMERA " Camera (Clean Look-Through)",
            ICON_MDI_LIGHTNING_BOLT " EVS Events (Accumulation)",
            ICON_MDI_VIEW_GRID " Top (Top Ortho X-Z)",
            ICON_MDI_VIEW_AGENDA " Front (Front Ortho X-Y)",
            ICON_MDI_VIEW_WEEK " Side (Side Ortho Z-Y)",
            ICON_MDI_ROTATE_ORBIT " 3D Perspective",
            ICON_MDI_CHART_BELL_CURVE " IMU Telemetry",
            ICON_MDI_IMAGE " Simulated APS (Blur + Noise)"
        };

        int cur_view = static_cast<int>(viewport_views_[quad_idx]);
        ImGui::SetNextItemWidth(210);
        std::string combo_id = "##ViewCombo_" + std::to_string(quad_idx);
        if (ImGui::Combo(combo_id.c_str(), &cur_view, view_names, 8)) {
            viewport_views_[quad_idx] = static_cast<ViewportContent>(cur_view);
        }

        ImGui::SameLine();
        if (viewport_views_[quad_idx] == ViewportContent::CAMERA_CLEAN) {
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
            ImGui::SameLine();
            ImGui::TextDisabled("| Pan: Drag | Zoom: Wheel | Edit: Drag KF");
        } else if (viewport_views_[quad_idx] == ViewportContent::SIDE_ORTHO) {
            ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.35f, 1.0f), ICON_MDI_VIEW_WEEK " [Side Ortho | Z-Y]");
            ImGui::SameLine();
            std::string btn_id = ICON_MDI_FIT_TO_SCREEN " Frame##side" + std::to_string(quad_idx);
            if (ImGui::SmallButton(btn_id.c_str())) frame_ortho_view(2);
            ImGui::SameLine();
            ImGui::TextDisabled("| Pan: Drag | Zoom: Wheel | Edit: Drag KF");
        } else if (viewport_views_[quad_idx] == ViewportContent::FRONT_ORTHO) {
            ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.35f, 1.0f), ICON_MDI_VIEW_AGENDA " [Front Ortho | X-Y]");
            ImGui::SameLine();
            std::string btn_id = ICON_MDI_FIT_TO_SCREEN " Frame##front" + std::to_string(quad_idx);
            if (ImGui::SmallButton(btn_id.c_str())) frame_ortho_view(1);
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
                float cx = img_p0.x + draw_size.x * 0.5f;
                float cy = img_p0.y + draw_size.y * 0.5f;
                draw_list->AddCircle(ImVec2(cx, cy), 14.0f, IM_COL32(95, 120, 240, 180), 16, 1.5f);
                draw_list->AddLine(ImVec2(cx - 20, cy), ImVec2(cx - 5, cy), IM_COL32(95, 120, 240, 180), 1.5f);
                draw_list->AddLine(ImVec2(cx + 5, cy), ImVec2(cx + 20, cy), IM_COL32(95, 120, 240, 180), 1.5f);
                draw_list->AddLine(ImVec2(cx, cy - 20), ImVec2(cx, cy - 5), IM_COL32(95, 120, 240, 180), 1.5f);
                draw_list->AddLine(ImVec2(cx, cy + 5), ImVec2(cx, cy + 20), IM_COL32(95, 120, 240, 180), 1.5f);
            }
        };

        // Render viewport contents
        switch (viewport_views_[quad_idx]) {
            case ViewportContent::CAMERA_CLEAN: {
                if (avail_sz.x > 10.0f && avail_sz.y > 10.0f) {
                    float tex_aspect = static_cast<float>(sensor_tex_w_) / static_cast<float>(sensor_tex_h_);
                    float avail_aspect = avail_sz.x / avail_sz.y;
                    float draw_w = (avail_aspect > tex_aspect) ? (avail_sz.y * tex_aspect) : avail_sz.x;
                    float draw_h = (avail_aspect > tex_aspect) ? avail_sz.y : (avail_sz.x / tex_aspect);

                    uint32_t tw = (static_cast<uint32_t>(draw_w) + 3) & ~3u;
                    uint32_t th = (static_cast<uint32_t>(draw_h) + 1) & ~1u;

                    if (std::abs(static_cast<int>(tw) - static_cast<int>(camera_render_w_)) > 8 ||
                        std::abs(static_cast<int>(th) - static_cast<int>(camera_render_h_)) > 8) {
                        resize_camera_render(tw, th);
                    }
                }
                render_aspect_image(sensor_texture_id_, camera_render_w_, camera_render_h_);
                handle_camera_mouse_input(canvas_p0.x, canvas_p0.y, canvas_p1.x, canvas_p1.y);
                break;
            }

            case ViewportContent::SIMULATED_APS_SENSOR:
                render_aspect_image(sim_aps_texture_id_, sensor_tex_w_, sensor_tex_h_);
                break;

            case ViewportContent::EVS_ACCUMULATION:
                render_aspect_image(evs_texture_id_, camera_render_w_, camera_render_h_);
                break;

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

    if (ImPlot::BeginPlot("Gyroscope (rad/s)##imu", ImVec2(-1, plot_h))) {
        ImPlot::SetupAxes("Time [s]", "rad/s", ImPlotAxisFlags_None, ImPlotAxisFlags_None);
        if (!plot_time_.empty()) {
            ImPlot::PlotLine("Gyro X", plot_time_.data(), plot_gyro_x_.data(), (int)plot_time_.size());
            ImPlot::PlotLine("Gyro Y", plot_time_.data(), plot_gyro_y_.data(), (int)plot_time_.size());
            ImPlot::PlotLine("Gyro Z", plot_time_.data(), plot_gyro_z_.data(), (int)plot_time_.size());
        }
        ImPlot::EndPlot();
    }

    if (ImPlot::BeginPlot("Accelerometer (m/s^2)##imu", ImVec2(-1, plot_h))) {
        ImPlot::SetupAxes("Time [s]", "m/s^2", ImPlotAxisFlags_None, ImPlotAxisFlags_None);
        if (!plot_time_.empty()) {
            ImPlot::PlotLine("Acc X", plot_time_.data(), plot_acc_x_.data(), (int)plot_time_.size());
            ImPlot::PlotLine("Acc Y", plot_time_.data(), plot_acc_y_.data(), (int)plot_time_.size());
            ImPlot::PlotLine("Acc Z", plot_time_.data(), plot_acc_z_.data(), (int)plot_time_.size());
        }
        ImPlot::EndPlot();
    }
}

} // namespace hesim3d
