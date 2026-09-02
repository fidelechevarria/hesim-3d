#include "gui_app.h"
#include <imgui.h>
#include <implot.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>

namespace hesim3d {

void GuiApp::render_header_bar() {
    ImGuiViewport* vp = ImGui::GetMainViewport();
    float bar_h = 42.0f;

    ImGui::SetNextWindowPos(vp->Pos);
    ImGui::SetNextWindowSize(ImVec2(vp->Size.x, bar_h));
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                            ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_MenuBar |
                            ImGuiWindowFlags_NoSavedSettings;

    if (ImGui::Begin("##HeaderBar", nullptr, flags)) {
        if (ImGui::BeginMenuBar()) {
            // 1. Archivo Menu
            if (ImGui::BeginMenu("Archivo")) {
                if (ImGui::MenuItem("💾 Guardar Trayectoria (.json)...")) {
                    save_trajectory_to_json(current_trajectory_file_);
                }
                if (ImGui::MenuItem("📂 Cargar Trayectoria (.json)...")) {
                    load_trajectory_from_json(current_trajectory_file_);
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Salir")) {
                    request_close();
                }
                ImGui::EndMenu();
            }

            // 2. Editar Menu
            if (ImGui::BeginMenu("Editar")) {
                if (ImGui::MenuItem("➕ Capturar Keyframe actual", "K")) {
                    capture_keyframe_at_current_time();
                }
                if (ImGui::MenuItem("🗑 Eliminar Keyframe seleccionado")) {
                    if (selected_keyframe_idx_ >= 0) delete_keyframe(selected_keyframe_idx_);
                }
                ImGui::EndMenu();
            }

            // 3. Ver Menu (Google Earth Studio Multi-Viewport Selector)
            if (ImGui::BeginMenu("Ver")) {
                if (ImGui::BeginMenu("Vista múltiple")) {
                    if (ImGui::MenuItem("1 ventana gráfica", nullptr, active_layout_ == MultiViewLayout::VIEW_1_SINGLE)) {
                        active_layout_ = MultiViewLayout::VIEW_1_SINGLE;
                    }
                    if (ImGui::MenuItem("2 ventanas gráficas (Dual)", nullptr, active_layout_ == MultiViewLayout::VIEW_2_SPLIT)) {
                        active_layout_ = MultiViewLayout::VIEW_2_SPLIT;
                    }
                    if (ImGui::MenuItem("3 ventanas gráficas", nullptr, active_layout_ == MultiViewLayout::VIEW_3_SPLIT)) {
                        active_layout_ = MultiViewLayout::VIEW_3_SPLIT;
                    }
                    if (ImGui::MenuItem("4 ventanas gráficas (2x2 Grid)", nullptr, active_layout_ == MultiViewLayout::VIEW_4_GRID)) {
                        active_layout_ = MultiViewLayout::VIEW_4_GRID;
                    }
                    ImGui::EndMenu();
                }
                ImGui::Separator();
                if (ImGui::MenuItem("↺ Resetear distribución por defecto")) {
                    split_ratio_x_ = 0.5f;
                    split_ratio_y_ = 0.5f;
                    timeline_height_px_ = 280.0f;
                }
                ImGui::EndMenu();
            }

            ImGui::Separator();

            // Scenario & Sensor Quick Info
            ImGui::TextColored(ImVec4(0.5f, 0.7f, 1.0f, 1.0f), "Escenario: %s", config_.scene_path.empty() ? "Checkerboard" : "Sponza");
            ImGui::SameLine();
            ImGui::TextDisabled("|");
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.3f, 1.0f), "Sensor: %s", config_.sensor_name.c_str());

            // Center: Google Earth Studio Transport Controls
            float center_offset = (vp->Size.x - 380.0f) * 0.5f;
            if (center_offset > ImGui::GetCursorPosX()) {
                ImGui::SetCursorPosX(center_offset);
            }

            // Transport Buttons: |<, <, Play/Pause, >, Loop
            if (ImGui::Button("|<##first", ImVec2(28, 22))) {
                current_time_sec_ = 0.0;
                if (!keyframes_.empty()) jump_to_keyframe(0);
            }
            ImGui::SameLine();
            if (ImGui::Button("<##prev", ImVec2(28, 22))) {
                is_playing_ = false;
                current_time_sec_ = std::max(0.0, current_time_sec_ - 1.0 / 30.0);
            }
            ImGui::SameLine();
            if (is_playing_) {
                if (ImGui::Button("❚❚##pause", ImVec2(36, 22))) is_playing_ = false;
            } else {
                if (ImGui::Button("▶##play", ImVec2(36, 22))) is_playing_ = true;
            }
            ImGui::SameLine();
            if (ImGui::Button(">##next", ImVec2(28, 22))) {
                is_playing_ = false;
                current_time_sec_ = std::min(config_.duration_sec, current_time_sec_ + 1.0 / 30.0);
            }
            ImGui::SameLine();

            int cur_frame = static_cast<int>(current_time_sec_ * 30.0);
            int total_frames = static_cast<int>(config_.duration_sec * 30.0);
            ImGui::Text("%02d:%05.2fs [%d/%d f]", static_cast<int>(current_time_sec_ / 60.0), std::fmod(current_time_sec_, 60.0), cur_frame, total_frames);

            // Right side: Recording / Export button
            float right_pos = vp->Size.x - 170.0f;
            if (right_pos > ImGui::GetCursorPosX()) {
                ImGui::SetCursorPosX(right_pos);
            }

            if (!is_recording_) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.25f, 0.25f, 1.0f));
                if (ImGui::Button("● Renderizar H5", ImVec2(150, 22))) {
                    is_recording_ = true;
                }
                ImGui::PopStyleColor();
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.25f, 0.75f, 0.35f, 1.0f));
                if (ImGui::Button("■ Guardar Dataset", ImVec2(150, 22))) {
                    is_recording_ = false;
                }
                ImGui::PopStyleColor();
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
        if (ImGui::Button("➕ Capturar Keyframe", ImVec2(160, 24))) {
            capture_keyframe_at_current_time();
        }
        ImGui::SameLine();
        if (ImGui::Button("🗑 Eliminar", ImVec2(80, 24))) {
            if (selected_keyframe_idx_ >= 0) delete_keyframe(selected_keyframe_idx_);
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(140);
        const char* interp_names[] = {"Spline SE(3)", "Lineal + Slerp"};
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
        if (ImGui::Button("💾 Guardar", ImVec2(80, 24))) {
            save_trajectory_to_json(current_trajectory_file_);
        }
        ImGui::SameLine();
        if (ImGui::Button("📂 Cargar", ImVec2(80, 24))) {
            load_trajectory_from_json(current_trajectory_file_);
        }

        ImGui::Separator();

        // Split Timeline into Left (Track Hierarchy) and Right (Ruler & Keyframe Lanes)
        float left_track_w = 340.0f;
        float right_canvas_w = std::max(200.0f, total_w - left_track_w - 20.0f);

        // Left Track Controls Column
        ImGui::BeginChild("##TrackHierarchy", ImVec2(left_track_w, 0), true);

        // Track 1: Posición
        if (ImGui::TreeNodeEx("▾ Posición de la cámara", ImGuiTreeNodeFlags_DefaultOpen)) {
            float x = static_cast<float>(camera_pos_.x());
            float y = static_cast<float>(camera_pos_.y());
            float z = static_cast<float>(camera_pos_.z());

            ImGui::SetNextItemWidth(120);
            if (ImGui::DragFloat("Longitud (X)", &x, 1.0f, -2000.0f, 2000.0f, "%.1f cm")) camera_pos_.x() = x;
            ImGui::SameLine();
            if (ImGui::SmallButton("◆##kfx")) capture_keyframe_at_current_time();

            ImGui::SetNextItemWidth(120);
            if (ImGui::DragFloat("Latitud (Z)", &z, 1.0f, -2000.0f, 2000.0f, "%.1f cm")) camera_pos_.z() = z;
            ImGui::SameLine();
            if (ImGui::SmallButton("◆##kfz")) capture_keyframe_at_current_time();

            ImGui::SetNextItemWidth(120);
            if (ImGui::DragFloat("Altitud (Y)", &y, 1.0f, -2000.0f, 2000.0f, "%.1f cm")) camera_pos_.y() = y;
            ImGui::SameLine();
            if (ImGui::SmallButton("◆##kfy")) capture_keyframe_at_current_time();

            ImGui::TreePop();
        }

        // Track 2: Rotación
        if (ImGui::TreeNodeEx("▾ Rotación de la cámara", ImGuiTreeNodeFlags_DefaultOpen)) {
            float yaw = static_cast<float>(camera_yaw_deg_);
            float pitch = static_cast<float>(camera_pitch_deg_);
            float roll = static_cast<float>(camera_roll_deg_);

            ImGui::SetNextItemWidth(120);
            if (ImGui::DragFloat("Mover (Pan)", &yaw, 0.5f, -180.0f, 180.0f, "%.1f°")) camera_yaw_deg_ = yaw;
            ImGui::SameLine();
            if (ImGui::SmallButton("◆##kfyaw")) capture_keyframe_at_current_time();

            ImGui::SetNextItemWidth(120);
            if (ImGui::DragFloat("Inclinar (Tilt)", &pitch, 0.5f, -89.0f, 89.0f, "%.1f°")) camera_pitch_deg_ = pitch;
            ImGui::SameLine();
            if (ImGui::SmallButton("◆##kfpitch")) capture_keyframe_at_current_time();

            ImGui::SetNextItemWidth(120);
            if (ImGui::DragFloat("Rodar (Roll)", &roll, 0.5f, -180.0f, 180.0f, "%.1f°")) camera_roll_deg_ = roll;
            ImGui::SameLine();
            if (ImGui::SmallButton("◆##kfroll")) capture_keyframe_at_current_time();

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

        // Draw Frame Ticks & Numbers
        int frame_step = (total_f > 300) ? 30 : 15;
        for (int f = 0; f <= total_f; f += frame_step) {
            float ratio = static_cast<float>(f) / static_cast<float>(total_f);
            float tx = canvas_p0.x + ratio * (canvas_sz.x - 20.0f) + 10.0f;
            draw_list->AddLine(ImVec2(tx, canvas_p0.y + ruler_h - 8.0f), ImVec2(tx, canvas_p0.y + ruler_h), IM_COL32(180, 180, 180, 200));
            std::string f_str = std::to_string(f);
            draw_list->AddText(ImVec2(tx + 2, canvas_p0.y + 2), IM_COL32(180, 180, 180, 255), f_str.c_str());
        }

        // Draw Track Horizontal Grid Lanes
        float lane_step = 26.0f;
        for (int i = 0; i < 7; ++i) {
            float ly = canvas_p0.y + ruler_h + (i + 1) * lane_step;
            draw_list->AddLine(ImVec2(canvas_p0.x, ly), ImVec2(canvas_p1.x, ly), IM_COL32(40, 44, 52, 180), 1.0f);
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
