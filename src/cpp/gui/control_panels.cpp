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

    // Global keyboard shortcuts for file actions
    ImGuiIO& io = ImGui::GetIO();
    if (!io.WantTextInput) {
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S)) {
            if (io.KeyShift) {
                prompt_save_trajectory_as();
            } else if (!current_trajectory_file_.empty()) {
                if (save_trajectory_to_json(current_trajectory_file_)) {
                    export_status_msg_ = "Saved trajectory";
                    export_status_timer_ = 4.0f;
                }
            } else {
                prompt_save_trajectory_as();
            }
        } else if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_O)) {
            prompt_load_trajectory();
        }
    }

    if (open) {
        if (ImGui::BeginMenuBar()) {
            // 1. File Menu
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem(ICON_MDI_CONTENT_SAVE " Save Trajectory (.json)", "Ctrl+S")) {
                    if (current_trajectory_file_.empty()) {
                        prompt_save_trajectory_as();
                    } else {
                        if (save_trajectory_to_json(current_trajectory_file_)) {
                            export_status_msg_ = "Saved trajectory";
                            export_status_timer_ = 4.0f;
                        }
                    }
                }
                if (ImGui::MenuItem(ICON_MDI_CONTENT_SAVE_EDIT " Save Trajectory As...", "Ctrl+Shift+S")) {
                    prompt_save_trajectory_as();
                }
                if (ImGui::MenuItem(ICON_MDI_FOLDER_OPEN " Open Trajectory (.json)...", "Ctrl+O")) {
                    prompt_load_trajectory();
                }
                ImGui::Separator();
                if (ImGui::MenuItem(ICON_MDI_FOLDER " Open Trajectories Folder...")) {
                    open_trajectories_folder();
                }
                if (ImGui::MenuItem(ICON_MDI_FOLDER_TABLE " Open Datasets Folder...")) {
                    open_datasets_folder();
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
                if (ImGui::MenuItem(ICON_MDI_SKIP_PREVIOUS " Previous Keyframe", "J")) {
                    jump_to_prev_keyframe();
                }
                if (ImGui::MenuItem(ICON_MDI_SKIP_NEXT " Next Keyframe", "L")) {
                    jump_to_next_keyframe();
                }
                ImGui::Separator();
                if (ImGui::MenuItem(ICON_MDI_TRASH_CAN_OUTLINE " Delete Selected Keyframe", "Del", false, selected_keyframe_idx_ >= 0)) {
                    if (selected_keyframe_idx_ >= 0) delete_keyframe(selected_keyframe_idx_);
                }
                ImGui::Separator();
                if (ImGui::MenuItem(ICON_MDI_ARROW_EXPAND_HORIZONTAL " Fit Timeline to Keyframes", "F")) {
                    frame_timeline_to_all_keyframes();
                }
                if (ImGui::MenuItem(ICON_MDI_FIT_TO_PAGE_OUTLINE " Reset Timeline Zoom")) {
                    reset_timeline_view();
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
                float right_width = 390.0f;
                if (simulation_has_data_ && trajectory_dirty_since_sim_) right_width += 85.0f;

                float right_pos = vp->Size.x - right_width;
                if (right_pos > ImGui::GetCursorPosX()) {
                    ImGui::SetCursorPosX(right_pos);
                }

                // Intermediate physical simulation sampling rate presets
                const char* rate_items[] = {
                    ICON_MDI_FLASH " 300 Hz (Fast)",
                    ICON_MDI_COG " 1000 Hz (Std)",
                    ICON_MDI_CUBE_SCAN " 3200 Hz (HKUST)"
                };
                ImGui::SetNextItemWidth(145);
                ImGui::Combo("##SimRateCombo", &sim_sampling_preset_, rate_items, 3);
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Intermediate physical sampling rate for motion blur & microsecond EVS");
                }

                ImGui::SameLine();
                if (simulation_has_data_ && trajectory_dirty_since_sim_) {
                    ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.22f, 1.0f), ICON_MDI_ALERT_CIRCLE " Re-bake");
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("Trajectory keyframes changed since previous bake. Re-baking recommended.");
                    }
                    ImGui::SameLine();
                }

                bool can_sim = (keyframes_.size() >= 2) && !is_simulating_;
                if (!can_sim) ImGui::BeginDisabled();
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.16f, 0.48f, 0.38f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.62f, 0.48f, 1.0f));
                if (ImGui::Button(ICON_MDI_ROCKET_LAUNCH " Bake Simulation", ImVec2(165, 22))) {
                    start_simulation_bake();
                }
                ImGui::PopStyleColor(2);
                if (!can_sim) {
                    ImGui::EndDisabled();
                    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                        ImGui::SetTooltip("Capture at least 2 keyframes in timeline before baking");
                    }
                } else if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Bake physical sensor dynamics (multi-exposure blur, noise, microsecond EVS)");
                }
            } else {
                float right_pos = vp->Size.x - 460.0f;
                if (right_pos > ImGui::GetCursorPosX()) {
                    ImGui::SetCursorPosX(right_pos);
                }
                if (font_mono_) ImGui::PushFont(font_mono_);
                ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.55f, 1.0f), ICON_MDI_LIGHTNING_BOLT " %zu Evts  " ICON_MDI_IMAGE_MULTIPLE " %zu F", sim_total_events_, sim_total_frames_);
                if (font_mono_) ImGui::PopFont();

                if (trajectory_dirty_since_sim_) {
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.22f, 1.0f), ICON_MDI_ALERT_CIRCLE " Outdated");
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("Trajectory modified. Return to Studio to re-bake with new keyframes.");
                    }
                }

                ImGui::SameLine();
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.22f, 0.42f, 0.70f, 1.0f));
                if (ImGui::Button(ICON_MDI_DOWNLOAD " Export (.h5)...", ImVec2(130, 22))) {
                    if (export_modal_h5_path_.empty()) {
                        export_modal_h5_path_ = recording_output_path_.empty() ? generate_default_dataset_path() : recording_output_path_;
                    }
                    try {
                        std::filesystem::path p(export_modal_h5_path_);
                        export_modal_traj_path_ = (p.parent_path() / (p.stem().string() + "_trajectory.json")).string();
                    } catch (...) {}
                    show_export_modal_ = true;
                }
                ImGui::PopStyleColor();
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Configure and export simulated dataset to HDF5 format");
                }

                if (export_status_timer_ > 0.0f) {
                    ImGui::SameLine();
                    ImVec4 col = (export_status_msg_.find("failed") != std::string::npos)
                        ? ImVec4(1.0f, 0.35f, 0.35f, 1.0f)
                        : ImVec4(0.35f, 1.0f, 0.45f, 1.0f);
                    ImGui::TextColored(col, "%s", export_status_msg_.c_str());
                }
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

    // Draw clean horizontal splitter divider line (hidden during simulation to avoid overlapping modals)
    if (!is_simulating_) {
        ImDrawList* bg = ImGui::GetBackgroundDrawList();
        ImU32 split_col = s_dragging_timeline ? IM_COL32(0, 190, 255, 255) : IM_COL32(45, 48, 56, 255);
        bg->AddLine(ImVec2(0, panel_y), ImVec2(total_w, panel_y), split_col, s_dragging_timeline ? 3.0f : 2.0f);
    }

    // Timeline Main Window
    ImGui::SetNextWindowPos(ImVec2(0, panel_y), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(total_w, timeline_height_px_), ImGuiCond_Always);
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings;

    if (ImGui::Begin("##EarthStudioTimeline", nullptr, flags)) {
        // Toolbar inside timeline
        if (current_mode_ == AppMode::TRAJECTORY_STUDIO) {
            if (ImGui::Button(ICON_MDI_KEY_PLUS " Capture", ImVec2(100, 24))) {
                capture_keyframe_at_current_time();
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Capture camera pose at current time (K)");
            ImGui::SameLine();

            if (ImGui::Button(ICON_MDI_SKIP_PREVIOUS "##prevkf", ImVec2(28, 24))) {
                jump_to_prev_keyframe();
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Jump to previous keyframe (J)");
            ImGui::SameLine();

            if (ImGui::Button(ICON_MDI_SKIP_NEXT "##nextkf", ImVec2(28, 24))) {
                jump_to_next_keyframe();
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Jump to next keyframe (L)");
            ImGui::SameLine();

            bool can_delete = (selected_keyframe_idx_ >= 0 && selected_keyframe_idx_ < static_cast<int>(keyframes_.size()));
            if (!can_delete) ImGui::BeginDisabled();
            if (ImGui::Button(ICON_MDI_TRASH_CAN_OUTLINE " Delete", ImVec2(80, 24))) {
                delete_keyframe(selected_keyframe_idx_);
            }
            if (!can_delete) ImGui::EndDisabled();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) ImGui::SetTooltip("Delete selected keyframe (Del / Backspace)");
            ImGui::SameLine();
        } else {
            ImGui::TextColored(ImVec4(0.35f, 0.95f, 0.55f, 1.0f), ICON_MDI_PLAY_CIRCLE_OUTLINE " Sensor Inspection Timeline");
            ImGui::SameLine();
            ImGui::TextDisabled("| Scrub playhead or hit Space to inspect baked sequence");
            ImGui::SameLine();
        }

        if (ImGui::Button(ICON_MDI_ARROW_EXPAND_HORIZONTAL " Fit", ImVec2(60, 24))) {
            frame_timeline_to_all_keyframes();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Fit view to all keyframes (F)");
        ImGui::SameLine();

        if (ImGui::Button(ICON_MDI_FIT_TO_PAGE_OUTLINE " Reset", ImVec2(65, 24))) {
            reset_timeline_view();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Reset view to full project duration");
        ImGui::SameLine();

        ImGui::SetNextItemWidth(65);
        float dur_val = static_cast<float>(config_.duration_sec);
        if (ImGui::DragFloat("##DurationSec", &dur_val, 0.1f, 0.5f, 600.0f, "%.1fs")) {
            config_.duration_sec = std::max(0.5, static_cast<double>(dur_val));
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Total Project Duration (seconds)");
        ImGui::SameLine();

        ImGui::SetNextItemWidth(140);
        const char* interp_names[] = {ICON_MDI_VECTOR_CURVE " SE(3) Spline", ICON_MDI_VECTOR_LINE " Linear + Slerp"};
        int cur_interp = static_cast<int>(interp_mode_);
        if (ImGui::Combo("##Interp", &cur_interp, interp_names, 2)) {
            interp_mode_ = static_cast<TrajectoryInterpolation>(cur_interp);
            rebuild_trajectory();
        }
        ImGui::SameLine();

        // Trajectory File Badge & Actions
        std::string traj_fname = "untitled.json";
        try {
            if (!current_trajectory_file_.empty()) {
                traj_fname = std::filesystem::path(current_trajectory_file_).filename().string();
            }
        } catch (...) {}

        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.17f, 0.22f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.20f, 0.24f, 0.30f, 1.0f));
        std::string traj_btn_label = ICON_MDI_FILE_DOCUMENT_OUTLINE " " + traj_fname;
        if (ImGui::Button(traj_btn_label.c_str(), ImVec2(0, 24))) {
            prompt_save_trajectory_as();
        }
        ImGui::PopStyleColor(2);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Current Trajectory File:\n%s\n\nClick to Save As...", current_trajectory_file_.c_str());
        }
        ImGui::SameLine();

        if (ImGui::Button(ICON_MDI_CONTENT_SAVE " Save", ImVec2(65, 24))) {
            if (current_trajectory_file_.empty()) {
                prompt_save_trajectory_as();
            } else {
                if (save_trajectory_to_json(current_trajectory_file_)) {
                    export_status_msg_ = "Saved trajectory";
                    export_status_timer_ = 4.0f;
                }
            }
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Save trajectory directly to: %s", current_trajectory_file_.c_str());
        }
        ImGui::SameLine();

        if (ImGui::Button(ICON_MDI_CONTENT_SAVE_EDIT " Save As...", ImVec2(88, 24))) {
            prompt_save_trajectory_as();
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Choose destination and save trajectory with native file browser");
        }
        ImGui::SameLine();

        if (ImGui::Button(ICON_MDI_FOLDER_OPEN " Open...", ImVec2(75, 24))) {
            prompt_load_trajectory();
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Load trajectory from JSON file with native file browser");
        }

        ImGui::Separator();

        // Split Timeline into Left (Track Hierarchy) and Right (Ruler & Keyframe Lanes)
        float left_track_w = 280.0f;
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

            ImGui::SetNextItemWidth(140);
            if (ImGui::DragFloat("Longitude (X)", &x, step, x_min, x_max, fmt)) camera_pos_.x() = x;

            ImGui::SetNextItemWidth(140);
            if (ImGui::DragFloat("Latitude (Z)", &z, step, z_min, z_max, fmt)) camera_pos_.z() = z;

            ImGui::SetNextItemWidth(140);
            if (ImGui::DragFloat("Altitude (Y)", &y, step, y_min, y_max, fmt)) camera_pos_.y() = y;

            ImGui::TreePop();
        }

        // Track 2: Rotation
        if (ImGui::TreeNodeEx(ICON_MDI_ROTATE_ORBIT " Camera Rotation", ImGuiTreeNodeFlags_DefaultOpen)) {
            float yaw = static_cast<float>(camera_yaw_deg_);
            float pitch = static_cast<float>(camera_pitch_deg_);
            float roll = static_cast<float>(camera_roll_deg_);

            ImGui::SetNextItemWidth(140);
            if (ImGui::DragFloat("Pan (Yaw)", &yaw, 0.5f, -180.0f, 180.0f, "%.1f deg")) camera_yaw_deg_ = yaw;

            ImGui::SetNextItemWidth(140);
            if (ImGui::DragFloat("Tilt (Pitch)", &pitch, 0.5f, -89.0f, 89.0f, "%.1f deg")) camera_pitch_deg_ = pitch;

            ImGui::SetNextItemWidth(140);
            if (ImGui::DragFloat("Roll", &roll, 0.5f, -180.0f, 180.0f, "%.1f deg")) camera_roll_deg_ = roll;

            ImGui::TreePop();
        }

        ImGui::EndChild();

        ImGui::SameLine();

        // Right Timeline Canvas Column (Overview + Ruler + Lanes + Playhead)
        ImGui::BeginChild("##TimelineCanvas", ImVec2(right_canvas_w, 0), true, ImGuiWindowFlags_NoScrollbar);
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
        ImVec2 canvas_sz = ImGui::GetContentRegionAvail();
        ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);

        if (!timeline_view_initialized_) {
            timeline_view_t_min_ = 0.0;
            timeline_view_t_max_ = std::max(1.0, config_.duration_sec);
            timeline_view_initialized_ = true;
        }

        double proj_dur = std::max(0.5, config_.duration_sec);
        if (timeline_view_t_max_ <= timeline_view_t_min_ + 0.1) {
            timeline_view_t_max_ = timeline_view_t_min_ + 1.0;
        }

        float overview_h = 16.0f;
        float ruler_h = 24.0f;
        ImVec2 ov_p0 = canvas_p0;
        ImVec2 ov_p1 = ImVec2(canvas_p1.x, canvas_p0.y + overview_h);
        ImVec2 ruler_p0 = ImVec2(canvas_p0.x, canvas_p0.y + overview_h);
        ImVec2 ruler_p1 = ImVec2(canvas_p1.x, ruler_p0.y + ruler_h);

        bool is_canvas_hovered = ImGui::IsWindowHovered();

        // Global hotkeys when timeline canvas is hovered
        if (is_canvas_hovered) {
            if (ImGui::IsKeyPressed(ImGuiKey_K)) {
                capture_keyframe_at_current_time();
            }
            if (ImGui::IsKeyPressed(ImGuiKey_J)) {
                jump_to_prev_keyframe();
            }
            if (ImGui::IsKeyPressed(ImGuiKey_L)) {
                jump_to_next_keyframe();
            }
            if (ImGui::IsKeyPressed(ImGuiKey_F)) {
                frame_timeline_to_all_keyframes();
            }
            if (selected_keyframe_idx_ >= 0 && (ImGui::IsKeyPressed(ImGuiKey_Delete) || ImGui::IsKeyPressed(ImGuiKey_Backspace))) {
                delete_keyframe(selected_keyframe_idx_);
            }
        }

        // --- 1. Draw Overview / Zoom Navigator Bar (Google Earth Studio style) ---
        draw_list->AddRectFilled(ov_p0, ov_p1, IM_COL32(26, 28, 33, 255));
        draw_list->AddLine(ImVec2(ov_p0.x, ov_p1.y), ImVec2(ov_p1.x, ov_p1.y), IM_COL32(45, 48, 55, 255));

        // Draw keyframe ticks on overview bar
        for (const auto& kf : keyframes_) {
            float kf_norm = std::clamp(static_cast<float>(kf.time_sec / proj_dur), 0.0f, 1.0f);
            float ov_kx = ov_p0.x + kf_norm * (canvas_sz.x - 20.0f) + 10.0f;
            draw_list->AddLine(ImVec2(ov_kx, ov_p0.y + 2.0f), ImVec2(ov_kx, ov_p1.y - 2.0f), IM_COL32(230, 195, 40, 200), 1.5f);
        }

        // Highlight box for current view window
        float v_norm0 = std::clamp(static_cast<float>(timeline_view_t_min_ / proj_dur), 0.0f, 1.0f);
        float v_norm1 = std::clamp(static_cast<float>(timeline_view_t_max_ / proj_dur), 0.0f, 1.0f);
        float ov_bx0 = ov_p0.x + v_norm0 * (canvas_sz.x - 20.0f) + 10.0f;
        float ov_bx1 = ov_p0.x + v_norm1 * (canvas_sz.x - 20.0f) + 10.0f;
        if (ov_bx1 < ov_bx0 + 12.0f) ov_bx1 = ov_bx0 + 12.0f;

        draw_list->AddRectFilled(ImVec2(ov_bx0, ov_p0.y + 1.0f), ImVec2(ov_bx1, ov_p1.y - 1.0f), IM_COL32(0, 150, 215, 60), 2.0f);
        draw_list->AddRect(ImVec2(ov_bx0, ov_p0.y + 1.0f), ImVec2(ov_bx1, ov_p1.y - 1.0f), IM_COL32(0, 190, 255, 200), 2.0f, 0, 1.5f);

        // Interaction on Overview Bar
        static bool s_dragging_overview = false;
        bool mouse_in_overview = (mouse_pos.x >= ov_p0.x && mouse_pos.x <= ov_p1.x &&
                                  mouse_pos.y >= ov_p0.y && mouse_pos.y <= ov_p1.y);

        if (mouse_in_overview && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
            reset_timeline_view();
        } else if (mouse_in_overview && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            s_dragging_overview = true;
        }
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            s_dragging_overview = false;
        }

        if (s_dragging_overview) {
            float mouse_norm = std::clamp((mouse_pos.x - (ov_p0.x + 10.0f)) / (canvas_sz.x - 20.0f), 0.0f, 1.0f);
            double center_t = mouse_norm * proj_dur;
            double cur_span = timeline_view_t_max_ - timeline_view_t_min_;
            timeline_view_t_min_ = center_t - cur_span * 0.5;
            timeline_view_t_max_ = timeline_view_t_min_ + cur_span;
            if (timeline_view_t_min_ < -0.2) {
                timeline_view_t_max_ += (-0.2 - timeline_view_t_min_);
                timeline_view_t_min_ = -0.2;
            }
        }

        // --- 2. Timeline Canvas Navigation (Mouse Wheel Zoom & Pan) ---
        if (is_canvas_hovered && !s_dragging_overview && dragging_timeline_kf_idx_ < 0) {
            float wheel = io.MouseWheel;
            if (std::abs(wheel) > 0.01f) {
                double mouse_t = timeline_canvas_x_to_time(mouse_pos.x, canvas_p0.x, canvas_sz.x);
                double zoom = (wheel > 0.0f) ? 0.85 : 1.18;
                double cur_span = timeline_view_t_max_ - timeline_view_t_min_;
                double new_span = std::clamp(cur_span * zoom, 0.2, std::max(300.0, config_.duration_sec * 4.0));
                double t_ratio = (mouse_t - timeline_view_t_min_) / cur_span;
                timeline_view_t_min_ = mouse_t - t_ratio * new_span;
                timeline_view_t_max_ = timeline_view_t_min_ + new_span;
                if (timeline_view_t_min_ < -0.2) {
                    timeline_view_t_max_ += (-0.2 - timeline_view_t_min_);
                    timeline_view_t_min_ = -0.2;
                }
            }

            // Pan with Middle Mouse Drag or Alt + Left Drag
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle) || (io.KeyAlt && ImGui::IsMouseDragging(ImGuiMouseButton_Left))) {
                float dx = io.MouseDelta.x;
                double dt = -dx * (timeline_view_t_max_ - timeline_view_t_min_) / (canvas_sz.x - 20.0f);
                timeline_view_t_min_ += dt;
                timeline_view_t_max_ += dt;
                if (timeline_view_t_min_ < -0.2) {
                    timeline_view_t_max_ += (-0.2 - timeline_view_t_min_);
                    timeline_view_t_min_ = -0.2;
                }
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
            }
        }

        // --- 3. Draw Ruler Background & Dynamic Adaptive Ticks ---
        draw_list->AddRectFilled(ruler_p0, ruler_p1, IM_COL32(35, 38, 44, 255));
        draw_list->AddLine(ImVec2(ruler_p0.x, ruler_p1.y), ruler_p1, IM_COL32(60, 65, 75, 255));

        double view_span = timeline_view_t_max_ - timeline_view_t_min_;
        int visible_frames = static_cast<int>(view_span * 30.0);

        int tick_step = 1;
        int label_step = 5;
        if (visible_frames > 600) {
            tick_step = 30; label_step = 90;
        } else if (visible_frames > 250) {
            tick_step = 15; label_step = 30;
        } else if (visible_frames > 90) {
            tick_step = 5; label_step = 15;
        } else if (visible_frames > 40) {
            tick_step = 2; label_step = 5;
        } else {
            tick_step = 1; label_step = 5;
        }

        int start_f = static_cast<int>(std::floor(timeline_view_t_min_ * 30.0));
        start_f = (start_f / tick_step) * tick_step;
        int end_f = static_cast<int>(std::ceil(timeline_view_t_max_ * 30.0)) + tick_step;

        for (int f = start_f; f <= end_f; f += tick_step) {
            if (f < 0) continue;
            double t = static_cast<double>(f) / 30.0;
            float tx = time_to_timeline_canvas_x(t, canvas_p0.x, canvas_sz.x);
            if (tx < canvas_p0.x || tx > canvas_p1.x) continue;

            bool is_major = (f % label_step == 0);
            float tick_h = is_major ? 10.0f : 5.0f;
            ImU32 tick_col = is_major ? IM_COL32(200, 205, 215, 240) : IM_COL32(110, 115, 125, 160);
            draw_list->AddLine(ImVec2(tx, ruler_p1.y - tick_h), ImVec2(tx, ruler_p1.y), tick_col);

            if (is_major) {
                std::string f_str = std::to_string(f);
                if (font_mono_) {
                    draw_list->AddText(font_mono_, 12.0f, ImVec2(tx + 3, ruler_p0.y + 2), IM_COL32(190, 195, 205, 255), f_str.c_str());
                } else {
                    draw_list->AddText(ImVec2(tx + 2, ruler_p0.y + 2), IM_COL32(180, 180, 180, 255), f_str.c_str());
                }
            }
        }

        // --- 4. Draw Horizontal Grid Lanes ---
        float lane_step = 26.0f;
        for (int i = 0; i < 7; ++i) {
            float ly = ruler_p1.y + (i + 1) * lane_step;
            draw_list->AddLine(ImVec2(canvas_p0.x, ly), ImVec2(canvas_p1.x, ly), IM_COL32(40, 44, 52, 180), 1.0f);
        }

        // Guidance message when no or only 1 keyframe exists
        if (keyframes_.empty()) {
            std::string hint = ICON_MDI_INFORMATION_OUTLINE " Free Camera Mode | Navigate with mouse and press [K] or [+ Capture] to define trajectory";
            ImVec2 txt_sz = ImGui::CalcTextSize(hint.c_str());
            float tx = canvas_p0.x + std::max(20.0f, (canvas_sz.x - txt_sz.x) * 0.5f);
            float ty = ruler_p1.y + 45.0f;
            draw_list->AddText(ImVec2(tx, ty), IM_COL32(140, 175, 215, 220), hint.c_str());
        } else if (keyframes_.size() == 1) {
            std::string hint = ICON_MDI_ALERT_CIRCLE_OUTLINE " 1 keyframe set. Move camera and capture at least 1 more keyframe to generate SE(3) spline.";
            ImVec2 txt_sz = ImGui::CalcTextSize(hint.c_str());
            float tx = canvas_p0.x + std::max(20.0f, (canvas_sz.x - txt_sz.x) * 0.5f);
            float ty = ruler_p1.y + 45.0f;
            draw_list->AddText(ImVec2(tx, ty), IM_COL32(240, 200, 90, 220), hint.c_str());
        }

        // --- 5. Draw Keyframe Diamonds & Interactive Dragging ---
        int hovered_diamond_kf_idx = -1;

        for (size_t k = 0; k < keyframes_.size(); ++k) {
            float kx = time_to_timeline_canvas_x(keyframes_[k].time_sec, canvas_p0.x, canvas_sz.x);

            // Draw vertical guide line
            if (kx >= canvas_p0.x && kx <= canvas_p1.x) {
                draw_list->AddLine(ImVec2(kx, ruler_p1.y), ImVec2(kx, canvas_p1.y), IM_COL32(90, 85, 40, 120), 1.0f);
            }

            for (int lane = 0; lane < 6; ++lane) {
                float ky = ruler_p1.y + lane * lane_step + lane_step * 0.5f;

                bool is_selected = (static_cast<int>(k) == selected_keyframe_idx_);
                bool is_hovered = (std::abs(mouse_pos.x - kx) < 8.0f && std::abs(mouse_pos.y - ky) < 8.0f);

                if (is_hovered) {
                    hovered_diamond_kf_idx = static_cast<int>(k);
                }

                if (kx >= canvas_p0.x - 10.0f && kx <= canvas_p1.x + 10.0f) {
                    float dsz = is_selected ? 6.5f : (is_hovered ? 5.5f : 4.5f);
                    ImU32 dcol = is_selected ? IM_COL32(255, 235, 50, 255)
                                             : (is_hovered ? IM_COL32(255, 220, 90, 255) : IM_COL32(220, 190, 40, 230));

                    ImVec2 dt(kx, ky - dsz);
                    ImVec2 dr(kx + dsz, ky);
                    ImVec2 db(kx, ky + dsz);
                    ImVec2 dl(kx - dsz, ky);

                    draw_list->AddQuadFilled(dt, dr, db, dl, dcol);
                    draw_list->AddQuad(dt, dr, db, dl, IM_COL32(20, 20, 20, 255), 1.0f);
                }
            }
        }

        // Diamond interactions: Click to select, Drag to move in time, Right-click context menu
        if (is_canvas_hovered) {
            if (hovered_diamond_kf_idx >= 0 && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
                selected_keyframe_idx_ = hovered_diamond_kf_idx;
                ImGui::OpenPopup("##KeyframeContextMenu");
            } else if (hovered_diamond_kf_idx >= 0 && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !io.KeyAlt) {
                dragging_timeline_kf_idx_ = hovered_diamond_kf_idx;
                selected_keyframe_idx_ = hovered_diamond_kf_idx;
                jump_to_keyframe(hovered_diamond_kf_idx);
            }
        }

        // Active Keyframe Dragging
        if (dragging_timeline_kf_idx_ >= 0 && dragging_timeline_kf_idx_ < static_cast<int>(keyframes_.size())) {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
                double target_t = timeline_canvas_x_to_time(mouse_pos.x, canvas_p0.x, canvas_sz.x);
                if (!io.KeyShift) {
                    target_t = std::round(target_t * 30.0) / 30.0; // snap to nearest frame
                }
                target_t = std::max(0.0, target_t);
                keyframes_[dragging_timeline_kf_idx_].time_sec = target_t;
                current_time_sec_ = target_t;
                apply_spline_sample_at(current_time_sec_);
            } else {
                // Drag finished: re-sort keyframes and rebuild trajectory
                double dragged_t = keyframes_[dragging_timeline_kf_idx_].time_sec;
                std::sort(keyframes_.begin(), keyframes_.end(), [](const StudioKeyframe& a, const StudioKeyframe& b) {
                    return a.time_sec < b.time_sec;
                });
                for (size_t i = 0; i < keyframes_.size(); ++i) {
                    if (std::abs(keyframes_[i].time_sec - dragged_t) < 0.001) {
                        selected_keyframe_idx_ = static_cast<int>(i);
                        break;
                    }
                }
                rebuild_trajectory();
                dragging_timeline_kf_idx_ = -1;
            }
        }

        // Right-Click Context Menu Popup
        if (ImGui::BeginPopup("##KeyframeContextMenu")) {
            if (ImGui::MenuItem(ICON_MDI_TRASH_CAN_OUTLINE " Delete Keyframe", "Del")) {
                if (selected_keyframe_idx_ >= 0) delete_keyframe(selected_keyframe_idx_);
            }
            if (ImGui::MenuItem(ICON_MDI_TARGET " Jump to Keyframe")) {
                if (selected_keyframe_idx_ >= 0) jump_to_keyframe(selected_keyframe_idx_);
            }
            if (ImGui::MenuItem(ICON_MDI_CAMERA_RETAKE " Update with Current Camera Pose")) {
                if (selected_keyframe_idx_ >= 0) update_keyframe_pose(selected_keyframe_idx_);
            }
            ImGui::EndPopup();
        }

        // --- 6. Playhead Scrubbing on Canvas ---
        if (dragging_timeline_kf_idx_ < 0 && !s_dragging_overview && !ImGui::IsPopupOpen("##KeyframeContextMenu")) {
            bool in_track_area = (mouse_pos.x >= canvas_p0.x && mouse_pos.x <= canvas_p1.x &&
                                  mouse_pos.y >= ruler_p0.y && mouse_pos.y <= canvas_p1.y);

            if (in_track_area && ImGui::IsMouseDown(ImGuiMouseButton_Left) && !io.KeyAlt && hovered_diamond_kf_idx < 0) {
                double t = timeline_canvas_x_to_time(mouse_pos.x, canvas_p0.x, canvas_sz.x);
                current_time_sec_ = std::clamp(t, 0.0, config_.duration_sec);
                apply_spline_sample_at(current_time_sec_);
            }
        }

        // --- 7. Draw Yellow Playhead Line & Handle ---
        float playhead_x = time_to_timeline_canvas_x(current_time_sec_, canvas_p0.x, canvas_sz.x);
        if (playhead_x >= canvas_p0.x - 10.0f && playhead_x <= canvas_p1.x + 10.0f) {
            draw_list->AddLine(ImVec2(playhead_x, ruler_p0.y), ImVec2(playhead_x, canvas_p1.y), IM_COL32(255, 220, 40, 255), 2.0f);

            // Playhead triangle handle on ruler
            ImVec2 ph_top(playhead_x - 6.0f, ruler_p0.y);
            ImVec2 ph_right(playhead_x + 6.0f, ruler_p0.y);
            ImVec2 ph_bot(playhead_x, ruler_p1.y);
            draw_list->AddTriangleFilled(ph_top, ph_right, ph_bot, IM_COL32(255, 220, 40, 255));
        }

        ImGui::EndChild();
    }
    ImGui::End();
}

} // namespace hesim3d
