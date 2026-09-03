#include "gui_app.h"
#include <imgui.h>
#include <implot.h>
#include "icons_material_design.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <filesystem>

namespace hesim3d {

void GuiApp::render_header_bar() {
    ImGuiViewport* vp = ImGui::GetMainViewport();
    float bar_h = 30.0f;

    ImGui::SetNextWindowPos(vp->Pos);
    ImGui::SetNextWindowSize(ImVec2(vp->Size.x, bar_h));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                            ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_MenuBar |
                            ImGuiWindowFlags_NoSavedSettings;

    bool open = ImGui::Begin("##HeaderBar", nullptr, flags);
    ImGui::PopStyleVar();

    if (open) {
        if (ImGui::BeginMenuBar()) {
            // 1. File Menu
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem(ICON_MDI_CONTENT_SAVE " Save Trajectory (.json)...")) {
                    save_trajectory_to_json(current_trajectory_file_);
                }
                if (ImGui::MenuItem(ICON_MDI_FOLDER_OPEN " Load Trajectory (.json)...")) {
                    load_trajectory_from_json(current_trajectory_file_);
                }
                ImGui::Separator();
                if (ImGui::MenuItem(ICON_MDI_EXIT_TO_APP " Exit")) {
                    request_close();
                }
                ImGui::EndMenu();
            }

            // 2. Edit Menu
            if (ImGui::BeginMenu("Edit")) {
                if (ImGui::MenuItem(ICON_MDI_KEY_PLUS " Capture Current Keyframe", "K")) {
                    capture_keyframe_at_current_time();
                }
                if (ImGui::MenuItem(ICON_MDI_TRASH_CAN_OUTLINE " Delete Selected Keyframe")) {
                    if (selected_keyframe_idx_ >= 0) delete_keyframe(selected_keyframe_idx_);
                }
                ImGui::EndMenu();
            }

            // 3. View Menu (Multi-Viewport Layout Selector)
            if (ImGui::BeginMenu("View")) {
                if (ImGui::BeginMenu(ICON_MDI_VIEW_DASHBOARD " Multi-View Layout")) {
                    if (ImGui::MenuItem(ICON_MDI_CHECKBOX_BLANK_OUTLINE " 1 Viewport (Single)", nullptr, active_layout_ == MultiViewLayout::VIEW_1_SINGLE)) {
                        set_multi_view_layout(MultiViewLayout::VIEW_1_SINGLE);
                    }
                    if (ImGui::MenuItem(ICON_MDI_VIEW_SPLIT_VERTICAL " 2 Viewports (Dual Split)", nullptr, active_layout_ == MultiViewLayout::VIEW_2_SPLIT)) {
                        set_multi_view_layout(MultiViewLayout::VIEW_2_SPLIT);
                    }
                    if (ImGui::MenuItem(ICON_MDI_VIEW_AGENDA_OUTLINE " 3 Viewports (Triple Split)", nullptr, active_layout_ == MultiViewLayout::VIEW_3_SPLIT)) {
                        set_multi_view_layout(MultiViewLayout::VIEW_3_SPLIT);
                    }
                    if (ImGui::MenuItem(ICON_MDI_VIEW_GRID " 4 Viewports (2x2 Grid)", nullptr, active_layout_ == MultiViewLayout::VIEW_4_GRID)) {
                        set_multi_view_layout(MultiViewLayout::VIEW_4_GRID);
                    }
                    ImGui::EndMenu();
                }
                ImGui::Separator();
                if (ImGui::MenuItem(ICON_MDI_ARROW_U_LEFT_TOP " Reset Default Layout")) {
                    split_ratio_x_ = 0.5f;
                    split_ratio_y_ = 0.5f;
                    timeline_height_px_ = 280.0f;
                    reset_viewport_resolutions();
                }
                ImGui::EndMenu();
            }

            ImGui::Separator();

            // Mode Switcher Tabs
            bool is_studio = (current_mode_ == AppMode::TRAJECTORY_STUDIO);
            bool is_sim = (current_mode_ == AppMode::SENSOR_SIMULATION);

            if (is_studio) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.35f, 0.85f, 1.0f, 1.0f));
            }
            if (ImGui::MenuItem(ICON_MDI_VECTOR_CURVE " Trajectory Studio", nullptr, is_studio)) {
                set_app_mode(AppMode::TRAJECTORY_STUDIO);
            }
            if (is_studio) ImGui::PopStyleColor();

            if (is_sim) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.35f, 0.95f, 0.55f, 1.0f));
            }
            if (ImGui::MenuItem(ICON_MDI_CHIP " Sensor Simulation", nullptr, is_sim)) {
                set_app_mode(AppMode::SENSOR_SIMULATION);
            }
            if (is_sim) ImGui::PopStyleColor();

            ImGui::Separator();

            // Scenario & Sensor Quick Info
            std::string scene_name = "Default";
            if (!config_.scene_path.empty()) {
                std::filesystem::path sp(config_.scene_path);
                scene_name = sp.stem().string();
            }
            ImGui::TextColored(ImVec4(0.5f, 0.75f, 1.0f, 1.0f), ICON_MDI_CUBE_OUTLINE " %s", scene_name.c_str());
            ImGui::SameLine();
            ImGui::TextDisabled("|");
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.4f, 1.0f), ICON_MDI_CAMERA_IRIS " %s", config_.sensor_name.c_str());

            // Nav speed slider
            ImGui::SameLine();
            ImGui::TextDisabled("|");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(70);
            ImGui::SliderFloat("##nav", &nav_speed_factor_, 0.2f, 4.0f, "%.1fx");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Navigation sensitivity factor (relative to estimated scene scale)");
            }

            // Center: Google Earth Studio Transport Controls
            float center_offset = (vp->Size.x - 420.0f) * 0.5f;
            if (center_offset > ImGui::GetCursorPosX()) {
                ImGui::SetCursorPosX(center_offset);
            }

            // Transport Buttons with Material Design Icons
            if (ImGui::Button(ICON_MDI_PAGE_FIRST "##first", ImVec2(28, 22))) {
                current_time_sec_ = 0.0;
                if (!keyframes_.empty()) jump_to_keyframe(0);
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Jump to start (0.0s)");

            ImGui::SameLine();
            if (ImGui::Button(ICON_MDI_STEP_BACKWARD "##prev", ImVec2(28, 22))) {
                is_playing_ = false;
                current_time_sec_ = std::max(0.0, current_time_sec_ - 1.0 / 30.0);
                apply_spline_sample_at(current_time_sec_);
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Step backward 1 frame (-33ms)");

            ImGui::SameLine();
            if (is_playing_) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.35f, 0.40f, 0.65f, 1.0f));
                if (ImGui::Button(ICON_MDI_PAUSE "##pause", ImVec2(36, 22))) is_playing_ = false;
                ImGui::PopStyleColor();
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Pause playback (Space)");
            } else {
                bool can_play = (keyframes_.size() >= 2);
                if (!can_play) ImGui::BeginDisabled();
                if (ImGui::Button(ICON_MDI_PLAY "##play", ImVec2(36, 22))) is_playing_ = true;
                if (!can_play) {
                    ImGui::EndDisabled();
                    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                        ImGui::SetTooltip("Add at least 2 keyframes to play trajectory");
                    }
                } else if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Play trajectory along SE(3) spline (Space)");
                }
            }

            ImGui::SameLine();
            if (ImGui::Button(ICON_MDI_STEP_FORWARD "##next", ImVec2(28, 22))) {
                is_playing_ = false;
                current_time_sec_ = std::min(config_.duration_sec, current_time_sec_ + 1.0 / 30.0);
                apply_spline_sample_at(current_time_sec_);
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Step forward 1 frame (+33ms)");

            ImGui::SameLine();

            // Render Timecode & Frame Counter in Monospace font for zero width jitter
            int cur_frame = static_cast<int>(current_time_sec_ * 30.0);
            int total_frames = static_cast<int>(config_.duration_sec * 30.0);
            if (font_mono_) ImGui::PushFont(font_mono_);
            ImGui::Text(ICON_MDI_CLOCK_OUTLINE " %02d:%05.2fs " ICON_MDI_FILMSTRIP " [%d/%d f]",
                        static_cast<int>(current_time_sec_ / 60.0),
                        std::fmod(current_time_sec_, 60.0),
                        cur_frame, total_frames);
            if (font_mono_) ImGui::PopFont();

            // Right side: Action controls per mode
            if (current_mode_ == AppMode::TRAJECTORY_STUDIO) {
                float right_pos = vp->Size.x - 260.0f;
                if (right_pos > ImGui::GetCursorPosX()) {
                    ImGui::SetCursorPosX(right_pos);
                }
                bool can_sim = (keyframes_.size() >= 2);
                if (!can_sim) ImGui::BeginDisabled();
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.16f, 0.48f, 0.38f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.62f, 0.48f, 1.0f));
                if (ImGui::Button(ICON_MDI_ROCKET_LAUNCH " Run H-ESIM Simulation", ImVec2(245, 22))) {
                    trigger_hesim_simulation();
                }
                ImGui::PopStyleColor(2);
                if (!can_sim) {
                    ImGui::EndDisabled();
                    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                        ImGui::SetTooltip("Capture at least 2 keyframes in timeline before simulation");
                    }
                }
            } else {
                float right_pos = vp->Size.x - 460.0f;
                if (right_pos > ImGui::GetCursorPosX()) {
                    ImGui::SetCursorPosX(right_pos);
                }
                if (font_mono_) ImGui::PushFont(font_mono_);
                ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.55f, 1.0f), ICON_MDI_LIGHTNING_BOLT " %zu Evts  " ICON_MDI_IMAGE_MULTIPLE " %zu F", sim_total_events_, sim_total_frames_);
                if (font_mono_) ImGui::PopFont();
                ImGui::SameLine();
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.22f, 0.42f, 0.70f, 1.0f));
                if (ImGui::Button(ICON_MDI_DOWNLOAD " Export (.h5)", ImVec2(125, 22))) {
                    export_simulated_dataset(recording_output_path_);
                }
                ImGui::PopStyleColor();
                ImGui::SameLine();
                if (ImGui::Button(ICON_MDI_ARROW_LEFT " Return to Studio", ImVec2(150, 22))) {
                    set_app_mode(AppMode::TRAJECTORY_STUDIO);
                }
            }

            ImGui::EndMenuBar();
        }
    }
    ImGui::End();
}

void GuiApp::render_timeline_panel() {
    ImGuiViewport* vp = ImGui::GetMainViewport();
    float total_w = vp->Size.x;
    float total_h = vp->Size.y;
    float panel_y = total_h - timeline_height_px_;

    ImGuiIO& io = ImGui::GetIO();
    ImVec2 mouse_pos = io.MousePos;

    static bool s_dragging_timeline = false;

    // Persistent Drag State for Timeline Splitter
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        if (std::abs(mouse_pos.y - panel_y) < 8.0f) {
            s_dragging_timeline = true;
        }
    }

    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        s_dragging_timeline = false;
    }

    if (s_dragging_timeline) {
        float new_h = total_h - mouse_pos.y;
        timeline_height_px_ = std::clamp(new_h, 150.0f, total_h * 0.65f);
        panel_y = total_h - timeline_height_px_;
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
    } else if (std::abs(mouse_pos.y - panel_y) < 8.0f) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
    }

    // Draw clean horizontal splitter divider line
    ImDrawList* fg = ImGui::GetForegroundDrawList();
    ImU32 split_col = s_dragging_timeline ? IM_COL32(0, 190, 255, 255) : IM_COL32(45, 48, 56, 255);
    fg->AddLine(ImVec2(0, panel_y), ImVec2(total_w, panel_y), split_col, s_dragging_timeline ? 3.0f : 2.0f);

    // Timeline Main Window
    ImGui::SetNextWindowPos(ImVec2(0, panel_y), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(total_w, timeline_height_px_), ImGuiCond_Always);
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings;

    if (ImGui::Begin("##EarthStudioTimeline", nullptr, flags)) {
        // Toolbar inside timeline
        if (ImGui::Button(ICON_MDI_KEY_PLUS " Capture Keyframe", ImVec2(170, 24))) {
            capture_keyframe_at_current_time();
        }
        ImGui::SameLine();
        if (ImGui::Button(ICON_MDI_TRASH_CAN_OUTLINE " Delete", ImVec2(85, 24))) {
            if (selected_keyframe_idx_ >= 0) delete_keyframe(selected_keyframe_idx_);
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(150);
        const char* interp_names[] = {ICON_MDI_VECTOR_CURVE " SE(3) Spline", ICON_MDI_VECTOR_LINE " Linear + Slerp"};
        int cur_interp = static_cast<int>(interp_mode_);
        if (ImGui::Combo("##Interp", &cur_interp, interp_names, 2)) {
            interp_mode_ = static_cast<TrajectoryInterpolation>(cur_interp);
            rebuild_trajectory();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(160);
        char file_buf[256];
        std::strncpy(file_buf, current_trajectory_file_.c_str(), sizeof(file_buf));
        if (ImGui::InputText("##TrajFile", file_buf, sizeof(file_buf))) {
            current_trajectory_file_ = file_buf;
        }
        ImGui::SameLine();
        if (ImGui::Button(ICON_MDI_CONTENT_SAVE " Save", ImVec2(80, 24))) {
            save_trajectory_to_json(current_trajectory_file_);
        }
        ImGui::SameLine();
        if (ImGui::Button(ICON_MDI_FOLDER_OPEN " Load", ImVec2(80, 24))) {
            load_trajectory_from_json(current_trajectory_file_);
        }

        ImGui::Separator();

        // Split Timeline into Left (Track Hierarchy) and Right (Ruler & Keyframe Lanes)
        float left_track_w = 340.0f;
        float right_canvas_w = std::max(200.0f, total_w - left_track_w - 20.0f);

        // Left Track Controls Column
        ImGui::BeginChild("##TrackHierarchy", ImVec2(left_track_w, 0), true);

        // Track 1: Position
        if (ImGui::TreeNodeEx(ICON_MDI_AXIS_ARROW " Camera Position", ImGuiTreeNodeFlags_DefaultOpen)) {
            float x = static_cast<float>(camera_pos_.x());
            float y = static_cast<float>(camera_pos_.y());
            float z = static_cast<float>(camera_pos_.z());

            double r_pos = scene_bounds_.valid ? scene_bounds_.radius : 2.0;
            float step = static_cast<float>(std::max(0.0005, r_pos * 0.002)) * nav_speed_factor_;
            float x_min = static_cast<float>(scene_bounds_.center.x() - r_pos * 5.0);
            float x_max = static_cast<float>(scene_bounds_.center.x() + r_pos * 5.0);
            float z_min = static_cast<float>(scene_bounds_.center.z() - r_pos * 5.0);
            float z_max = static_cast<float>(scene_bounds_.center.z() + r_pos * 5.0);
            float y_min = static_cast<float>(scene_bounds_.center.y() - r_pos * 5.0);
            float y_max = static_cast<float>(scene_bounds_.center.y() + r_pos * 5.0);
            const char* fmt = (r_pos < 10.0) ? "%.3f" : "%.1f";

            ImGui::SetNextItemWidth(120);
            if (ImGui::DragFloat("Longitude (X)", &x, step, x_min, x_max, fmt)) camera_pos_.x() = x;
            ImGui::SameLine();
            if (ImGui::SmallButton(ICON_MDI_PLUS "##kfx")) capture_keyframe_at_current_time();

            ImGui::SetNextItemWidth(120);
            if (ImGui::DragFloat("Latitude (Z)", &z, step, z_min, z_max, fmt)) camera_pos_.z() = z;
            ImGui::SameLine();
            if (ImGui::SmallButton(ICON_MDI_PLUS "##kfz")) capture_keyframe_at_current_time();

            ImGui::SetNextItemWidth(120);
            if (ImGui::DragFloat("Altitude (Y)", &y, step, y_min, y_max, fmt)) camera_pos_.y() = y;
            ImGui::SameLine();
            if (ImGui::SmallButton(ICON_MDI_PLUS "##kfy")) capture_keyframe_at_current_time();

            ImGui::TreePop();
        }

        // Track 2: Rotation
        if (ImGui::TreeNodeEx(ICON_MDI_ROTATE_ORBIT " Camera Rotation", ImGuiTreeNodeFlags_DefaultOpen)) {
            float yaw = static_cast<float>(camera_yaw_deg_);
            float pitch = static_cast<float>(camera_pitch_deg_);
            float roll = static_cast<float>(camera_roll_deg_);

            ImGui::SetNextItemWidth(120);
            if (ImGui::DragFloat("Pan (Yaw)", &yaw, 0.5f, -180.0f, 180.0f, "%.1f deg")) camera_yaw_deg_ = yaw;
            ImGui::SameLine();
            if (ImGui::SmallButton(ICON_MDI_PLUS "##kfyaw")) capture_keyframe_at_current_time();

            ImGui::SetNextItemWidth(120);
            if (ImGui::DragFloat("Tilt (Pitch)", &pitch, 0.5f, -89.0f, 89.0f, "%.1f deg")) camera_pitch_deg_ = pitch;
            ImGui::SameLine();
            if (ImGui::SmallButton(ICON_MDI_PLUS "##kfpitch")) capture_keyframe_at_current_time();

            ImGui::SetNextItemWidth(120);
            if (ImGui::DragFloat("Roll", &roll, 0.5f, -180.0f, 180.0f, "%.1f deg")) camera_roll_deg_ = roll;
            ImGui::SameLine();
            if (ImGui::SmallButton(ICON_MDI_PLUS "##kfroll")) capture_keyframe_at_current_time();

            ImGui::TreePop();
        }

        ImGui::EndChild();

        ImGui::SameLine();

        // Right Timeline Canvas Column (Ruler + Keyframe Diamonds + Playhead)
        ImGui::BeginChild("##TimelineCanvas", ImVec2(right_canvas_w, 0), true, ImGuiWindowFlags_NoScrollbar);
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
        ImVec2 canvas_sz = ImGui::GetContentRegionAvail();
        ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);

        float ruler_h = 24.0f;
        float total_time = static_cast<float>(std::max(0.5, config_.duration_sec));
        int total_f = static_cast<int>(total_time * 30.0);

        // Draw Ruler Background
        draw_list->AddRectFilled(canvas_p0, ImVec2(canvas_p1.x, canvas_p0.y + ruler_h), IM_COL32(35, 38, 44, 255));
        draw_list->AddLine(ImVec2(canvas_p0.x, canvas_p0.y + ruler_h), ImVec2(canvas_p1.x, canvas_p0.y + ruler_h), IM_COL32(60, 65, 75, 255));

        // Draw Frame Ticks & Numbers (using monospace font)
        int frame_step = (total_f > 300) ? 30 : 15;
        for (int f = 0; f <= total_f; f += frame_step) {
            float ratio = static_cast<float>(f) / static_cast<float>(total_f);
            float tx = canvas_p0.x + ratio * (canvas_sz.x - 20.0f) + 10.0f;
            draw_list->AddLine(ImVec2(tx, canvas_p0.y + ruler_h - 8.0f), ImVec2(tx, canvas_p0.y + ruler_h), IM_COL32(180, 180, 180, 200));
            std::string f_str = std::to_string(f);
            if (font_mono_) {
                draw_list->AddText(font_mono_, 13.0f, ImVec2(tx + 3, canvas_p0.y + 4), IM_COL32(190, 195, 205, 255), f_str.c_str());
            } else {
                draw_list->AddText(ImVec2(tx + 2, canvas_p0.y + 2), IM_COL32(180, 180, 180, 255), f_str.c_str());
            }
        }

        // Draw Track Horizontal Grid Lanes
        float lane_step = 26.0f;
        for (int i = 0; i < 7; ++i) {
            float ly = canvas_p0.y + ruler_h + (i + 1) * lane_step;
            draw_list->AddLine(ImVec2(canvas_p0.x, ly), ImVec2(canvas_p1.x, ly), IM_COL32(40, 44, 52, 180), 1.0f);
        }

        // Guidance message when no or only 1 keyframe exists
        if (keyframes_.empty()) {
            std::string hint = ICON_MDI_INFORMATION_OUTLINE " Free Camera Mode | Navigate with mouse and press [K] or [+ Capture Keyframe] to define trajectory";
            ImVec2 txt_sz = ImGui::CalcTextSize(hint.c_str());
            float tx = canvas_p0.x + std::max(20.0f, (canvas_sz.x - txt_sz.x) * 0.5f);
            float ty = canvas_p0.y + ruler_h + 45.0f;
            draw_list->AddText(ImVec2(tx, ty), IM_COL32(140, 175, 215, 220), hint.c_str());
        } else if (keyframes_.size() == 1) {
            std::string hint = ICON_MDI_ALERT_CIRCLE_OUTLINE " 1 keyframe set. Move camera and capture at least 1 more keyframe to generate SE(3) spline.";
            ImVec2 txt_sz = ImGui::CalcTextSize(hint.c_str());
            float tx = canvas_p0.x + std::max(20.0f, (canvas_sz.x - txt_sz.x) * 0.5f);
            float ty = canvas_p0.y + ruler_h + 45.0f;
            draw_list->AddText(ImVec2(tx, ty), IM_COL32(240, 200, 90, 220), hint.c_str());
        }

        // Draw Keyframe Diamonds on tracks
        for (size_t k = 0; k < keyframes_.size(); ++k) {
            float t_ratio = static_cast<float>(keyframes_[k].time_sec) / total_time;
            float kx = canvas_p0.x + t_ratio * (canvas_sz.x - 20.0f) + 10.0f;

            // Draw vertical diamond line
            draw_list->AddLine(ImVec2(kx, canvas_p0.y + ruler_h), ImVec2(kx, canvas_p1.y), IM_COL32(90, 85, 40, 120), 1.0f);

            for (int lane = 0; lane < 6; ++lane) {
                float ky = canvas_p0.y + ruler_h + lane * lane_step + lane_step * 0.5f;
                float dsz = (static_cast<int>(k) == selected_keyframe_idx_) ? 6.0f : 4.5f;
                ImU32 dcol = (static_cast<int>(k) == selected_keyframe_idx_) ? IM_COL32(255, 235, 50, 255) : IM_COL32(220, 190, 40, 230);

                ImVec2 dt(kx, ky - dsz);
                ImVec2 dr(kx + dsz, ky);
                ImVec2 db(kx, ky + dsz);
                ImVec2 dl(kx - dsz, ky);

                draw_list->AddQuadFilled(dt, dr, db, dl, dcol);
                draw_list->AddQuad(dt, dr, db, dl, IM_COL32(20, 20, 20, 255), 1.0f);

                // Click detection on diamonds
                ImVec2 mouse_pos = ImGui::GetIO().MousePos;
                if (std::abs(mouse_pos.x - kx) < 8.0f && std::abs(mouse_pos.y - ky) < 8.0f) {
                    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                        jump_to_keyframe(static_cast<int>(k));
                    }
                }
            }
        }

        // Handle Playhead Scrubbing on Canvas
        ImGui::InvisibleButton("##TimelineScrubArea", canvas_sz);
        if (ImGui::IsItemActive() || (ImGui::IsItemHovered() && ImGui::IsMouseDown(ImGuiMouseButton_Left))) {
            float mouse_x = ImGui::GetIO().MousePos.x;
            float relative_x = mouse_x - (canvas_p0.x + 10.0f);
            float ratio = std::clamp(relative_x / (canvas_sz.x - 20.0f), 0.0f, 1.0f);
            current_time_sec_ = ratio * total_time;
            apply_spline_sample_at(current_time_sec_);
        }

        // Draw Yellow Playhead Line
        float playhead_ratio = static_cast<float>(current_time_sec_) / total_time;
        float playhead_x = canvas_p0.x + playhead_ratio * (canvas_sz.x - 20.0f) + 10.0f;

        draw_list->AddLine(ImVec2(playhead_x, canvas_p0.y), ImVec2(playhead_x, canvas_p1.y), IM_COL32(255, 220, 40, 255), 2.0f);
        // Playhead handle on ruler
        ImVec2 ph_top(playhead_x - 6.0f, canvas_p0.y);
        ImVec2 ph_right(playhead_x + 6.0f, canvas_p0.y);
        ImVec2 ph_bot(playhead_x, canvas_p0.y + ruler_h);
        draw_list->AddTriangleFilled(ph_top, ph_right, ph_bot, IM_COL32(255, 220, 40, 255));

        ImGui::EndChild();
    }
    ImGui::End();
}

} // namespace hesim3d
