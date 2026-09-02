#include "gui_app.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <chrono>
#include <thread>
#include <algorithm>
#include <iomanip>

namespace hesim3d {

static inline double deg_to_rad(double deg) { return deg * M_PI / 180.0; }
static inline double rad_to_deg(double rad) { return rad * 180.0 / M_PI; }

static Eigen::Quaterniond euler_deg_to_quat(double yaw_deg, double pitch_deg, double roll_deg) {
    // Earth Studio convention: Yaw around Y, Pitch around X, Roll around Z
    Eigen::AngleAxisd roll(deg_to_rad(roll_deg), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitch(deg_to_rad(pitch_deg), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yaw(deg_to_rad(yaw_deg), Eigen::Vector3d::UnitY());
    return (yaw * pitch * roll).normalized();
}

static Eigen::Vector3d quat_to_euler_deg(const Eigen::Quaterniond& q) {
    Eigen::Matrix3d R = q.toRotationMatrix();
    // Extract Euler angles Y-X-Z
    double pitch = std::asin(std::clamp(R(1, 2), -1.0, 1.0));
    double yaw = 0.0;
    double roll = 0.0;
    if (std::abs(std::cos(pitch)) > 1e-6) {
        yaw = std::atan2(-R(0, 2), R(2, 2));
        roll = std::atan2(-R(1, 0), R(1, 1));
    } else {
        yaw = std::atan2(R(0, 1), R(0, 0));
        roll = 0.0;
    }
    return Eigen::Vector3d(rad_to_deg(yaw), rad_to_deg(pitch), rad_to_deg(roll));
}

GuiApp::GuiApp(const GuiConfig& config) : config_(config) {
    sensor_img_buffer_.resize(sensor_tex_w_ * sensor_tex_h_ * 3, 40);
    evs_img_buffer_.resize(sensor_tex_w_ * sensor_tex_h_ * 3, 20);
    prev_lum_buffer_.resize(sensor_tex_w_ * sensor_tex_h_, 40.0f);

    // Initial default keyframes
    bool is_sponza = (config_.scene_path.find("sponza") != std::string::npos);
    if (is_sponza) {
        camera_pos_ = Eigen::Vector3d(0.0, 180.0, -450.0);
        camera_yaw_deg_ = 0.0;
        camera_pitch_deg_ = 0.0;
        camera_roll_deg_ = 0.0;

        StudioKeyframe kf1{0.0, Eigen::Vector3d(0.0, 180.0, -450.0), Eigen::Vector3d(0.0, 0.0, 0.0), euler_deg_to_quat(0, 0, 0)};
        StudioKeyframe kf2{2.5, Eigen::Vector3d(120.0, 200.0, 0.0), Eigen::Vector3d(18.0, -6.0, 0.0), euler_deg_to_quat(18, -6, 0)};
        StudioKeyframe kf3{5.0, Eigen::Vector3d(0.0, 180.0, 450.0), Eigen::Vector3d(0.0, 0.0, 0.0), euler_deg_to_quat(0, 0, 0)};
        keyframes_ = {kf1, kf2, kf3};
    } else {
        camera_pos_ = Eigen::Vector3d(0.0, 1.5, 2.5);
        camera_yaw_deg_ = 0.0;
        camera_pitch_deg_ = -10.0;
        camera_roll_deg_ = 0.0;

        StudioKeyframe kf1{0.0, Eigen::Vector3d(-2.0, 1.5, 2.0), Eigen::Vector3d(-20.0, -10.0, 0.0), euler_deg_to_quat(-20, -10, 0)};
        StudioKeyframe kf2{2.5, Eigen::Vector3d(0.0, 1.8, 1.5), Eigen::Vector3d(0.0, -10.0, 0.0), euler_deg_to_quat(0, -10, 0)};
        StudioKeyframe kf3{5.0, Eigen::Vector3d(2.0, 1.5, 2.0), Eigen::Vector3d(20.0, -10.0, 0.0), euler_deg_to_quat(20, -10, 0)};
        keyframes_ = {kf1, kf2, kf3};
    }

    rebuild_trajectory();
}

GuiApp::~GuiApp() {
    renderer_.reset();
    if (window_) {
        ImPlot::DestroyContext();
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        glfwDestroyWindow(window_);
        glfwTerminate();
    }
}

void GuiApp::set_spline(const SE3Spline& spline) {
    spline_ = spline;
}

void GuiApp::capture_keyframe_at_current_time() {
    Eigen::Quaterniond q = euler_deg_to_quat(camera_yaw_deg_, camera_pitch_deg_, camera_roll_deg_);
    Eigen::Vector3d rot(camera_yaw_deg_, camera_pitch_deg_, camera_roll_deg_);

    StudioKeyframe new_kf{
        current_time_sec_,
        camera_pos_,
        rot,
        q
    };

    // Check if keyframe already exists near current_time_sec_ (within 0.02s)
    bool replaced = false;
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        if (std::abs(keyframes_[i].time_sec - current_time_sec_) < 0.03) {
            keyframes_[i] = new_kf;
            selected_keyframe_idx_ = static_cast<int>(i);
            replaced = true;
            break;
        }
    }

    if (!replaced) {
        keyframes_.push_back(new_kf);
        std::sort(keyframes_.begin(), keyframes_.end(), [](const StudioKeyframe& a, const StudioKeyframe& b) {
            return a.time_sec < b.time_sec;
        });
        for (size_t i = 0; i < keyframes_.size(); ++i) {
            if (std::abs(keyframes_[i].time_sec - current_time_sec_) < 0.01) {
                selected_keyframe_idx_ = static_cast<int>(i);
                break;
            }
        }
    }

    rebuild_trajectory();
}

void GuiApp::delete_keyframe(int index) {
    if (index >= 0 && index < static_cast<int>(keyframes_.size())) {
        keyframes_.erase(keyframes_.begin() + index);
        if (selected_keyframe_idx_ >= static_cast<int>(keyframes_.size())) {
            selected_keyframe_idx_ = static_cast<int>(keyframes_.size()) - 1;
        }
        rebuild_trajectory();
    }
}

void GuiApp::jump_to_keyframe(int index) {
    if (index >= 0 && index < static_cast<int>(keyframes_.size())) {
        selected_keyframe_idx_ = index;
        current_time_sec_ = keyframes_[index].time_sec;
        camera_pos_ = keyframes_[index].position;
        camera_yaw_deg_ = keyframes_[index].rotation_euler_deg.x();
        camera_pitch_deg_ = keyframes_[index].rotation_euler_deg.y();
        camera_roll_deg_ = keyframes_[index].rotation_euler_deg.z();
        update_simulation_step(0.0);
    }
}

void GuiApp::update_keyframe_pose(int index) {
    if (index >= 0 && index < static_cast<int>(keyframes_.size())) {
        keyframes_[index].position = camera_pos_;
        keyframes_[index].rotation_euler_deg = Eigen::Vector3d(camera_yaw_deg_, camera_pitch_deg_, camera_roll_deg_);
        keyframes_[index].orientation = euler_deg_to_quat(camera_yaw_deg_, camera_pitch_deg_, camera_roll_deg_);
        rebuild_trajectory();
    }
}

void GuiApp::rebuild_trajectory() {
    if (keyframes_.size() < 2) return;

    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Quaterniond> orientations;
    positions.reserve(keyframes_.size());
    orientations.reserve(keyframes_.size());

    for (const auto& kf : keyframes_) {
        positions.push_back(kf.position);
        orientations.push_back(kf.orientation);
    }

    double total_dur = keyframes_.back().time_sec - keyframes_.front().time_sec;
    if (total_dur <= 0.01) total_dur = config_.duration_sec;
    config_.duration_sec = total_dur;

    try {
        spline_.build_from_waypoints(positions, orientations, total_dur);
    } catch (const std::exception& e) {
        std::cerr << "[GuiApp] Error building spline: " << e.what() << std::endl;
    }

    // Cache dense path points for 2D/3D visualization
    path_samples_.clear();
    int num_samples = 120;
    double t_step = total_dur / static_cast<double>(num_samples);
    for (int i = 0; i <= num_samples; ++i) {
        double t = i * t_step;
        TrajectorySample s = spline_.evaluate(t);
        path_samples_.push_back(s.position);
    }
}

bool GuiApp::save_trajectory_to_json(const std::string& path) {
    std::ofstream f(path);
    if (!f.is_open()) return false;

    f << "{\n";
    f << "  \"version\": \"1.0\",\n";
    f << "  \"format\": \"hesim3d_earth_studio_trajectory\",\n";
    f << "  \"scene\": \"" << config_.scene_path << "\",\n";
    f << "  \"duration_sec\": " << std::fixed << std::setprecision(4) << config_.duration_sec << ",\n";
    f << "  \"interpolation\": \"" << (interp_mode_ == TrajectoryInterpolation::SE3_CUMULATIVE_SPLINE ? "spline_se3" : "linear_slerp") << "\",\n";
    f << "  \"keyframes\": [\n";

    for (size_t i = 0; i < keyframes_.size(); ++i) {
        const auto& kf = keyframes_[i];
        f << "    {\n";
        f << "      \"time_sec\": " << std::fixed << std::setprecision(4) << kf.time_sec << ",\n";
        f << "      \"position\": [" << kf.position.x() << ", " << kf.position.y() << ", " << kf.position.z() << "],\n";
        f << "      \"rotation_euler_deg\": [" << kf.rotation_euler_deg.x() << ", " << kf.rotation_euler_deg.y() << ", " << kf.rotation_euler_deg.z() << "],\n";
        f << "      \"orientation_xyzw\": [" << kf.orientation.x() << ", " << kf.orientation.y() << ", " << kf.orientation.z() << ", " << kf.orientation.w() << "]\n";
        f << "    }" << (i + 1 < keyframes_.size() ? "," : "") << "\n";
    }

    f << "  ]\n";
    f << "}\n";
    std::cout << "[GuiApp] Successfully exported trajectory to: " << path << std::endl;
    return true;
}

bool GuiApp::load_trajectory_from_json(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) return false;

    std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    // Simple robust JSON parser for keyframes
    std::vector<StudioKeyframe> loaded_kfs;

    size_t kf_pos = content.find("\"keyframes\"");
    if (kf_pos == std::string::npos) kf_pos = content.find("\"keyframes_6dof\"");
    if (kf_pos == std::string::npos) return false;

    size_t search_pos = kf_pos;
    while (true) {
        size_t obj_start = content.find('{', search_pos);
        if (obj_start == std::string::npos) break;
        size_t obj_end = content.find('}', obj_start);
        if (obj_end == std::string::npos) break;

        std::string obj_str = content.substr(obj_start, obj_end - obj_start + 1);

        auto extract_val = [&](const std::string& key) -> double {
            size_t p = obj_str.find("\"" + key + "\"");
            if (p == std::string::npos) return 0.0;
            size_t colon = obj_str.find(':', p);
            if (colon == std::string::npos) return 0.0;
            return std::stod(obj_str.substr(colon + 1));
        };

        auto extract_vec3 = [&](const std::string& key) -> Eigen::Vector3d {
            size_t p = obj_str.find("\"" + key + "\"");
            if (p == std::string::npos) return Eigen::Vector3d::Zero();
            size_t open_b = obj_str.find('[', p);
            size_t close_b = obj_str.find(']', open_b);
            if (open_b == std::string::npos || close_b == std::string::npos) return Eigen::Vector3d::Zero();
            std::string sub = obj_str.substr(open_b + 1, close_b - open_b - 1);
            std::replace(sub.begin(), sub.end(), ',', ' ');
            std::stringstream ss(sub);
            double x = 0, y = 0, z = 0;
            ss >> x >> y >> z;
            return Eigen::Vector3d(x, y, z);
        };

        double t = extract_val("time_sec");
        if (t == 0.0) t = extract_val("timestamp_sec");
        Eigen::Vector3d pos = extract_vec3("position");
        Eigen::Vector3d rot = extract_vec3("rotation_euler_deg");

        Eigen::Quaterniond q = euler_deg_to_quat(rot.x(), rot.y(), rot.z());
        loaded_kfs.push_back({t, pos, rot, q});
        search_pos = obj_end + 1;
    }

    if (loaded_kfs.size() >= 2) {
        keyframes_ = loaded_kfs;
        std::sort(keyframes_.begin(), keyframes_.end(), [](const StudioKeyframe& a, const StudioKeyframe& b) {
            return a.time_sec < b.time_sec;
        });
        jump_to_keyframe(0);
        rebuild_trajectory();
        std::cout << "[GuiApp] Loaded " << keyframes_.size() << " keyframes from " << path << std::endl;
        return true;
    }
    return false;
}

void GuiApp::compute_camera_pose(Eigen::Vector3d& out_pos, Eigen::Quaterniond& out_ori) {
    if (is_playing_) {
        TrajectorySample sample = spline_.evaluate(current_time_sec_);
        out_pos = sample.position;
        out_ori = sample.orientation;
        camera_pos_ = sample.position;
        Eigen::Vector3d euler = quat_to_euler_deg(sample.orientation);
        camera_yaw_deg_ = euler.x();
        camera_pitch_deg_ = euler.y();
        camera_roll_deg_ = euler.z();
    } else {
        out_pos = camera_pos_;
        out_ori = euler_deg_to_quat(camera_yaw_deg_, camera_pitch_deg_, camera_roll_deg_);
    }
}

void GuiApp::handle_camera_mouse_input(float min_x, float min_y, float max_x, float max_y) {
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 mouse_pos = io.MousePos;

    bool is_hovered = (mouse_pos.x >= min_x && mouse_pos.x <= max_x &&
                       mouse_pos.y >= min_y && mouse_pos.y <= max_y);

    if (!is_hovered) return;

    Eigen::Quaterniond q = euler_deg_to_quat(camera_yaw_deg_, camera_pitch_deg_, camera_roll_deg_);
    Eigen::Matrix3d R = q.toRotationMatrix();

    Eigen::Vector3d look_dir = R * Eigen::Vector3d(0, 0, -1);
    Eigen::Vector3d right_dir = R * Eigen::Vector3d(1, 0, 0);

    bool is_sponza = (config_.scene_path.find("sponza") != std::string::npos);
    float pan_speed = is_sponza ? 1.5f : 0.02f;
    float dolly_speed = is_sponza ? 2.5f : 0.03f;

    // 1. Left-Click + Drag: Horizontal Plane Pan (moves X/Z, keeps altitude Y constant)
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && !io.KeyShift && !io.KeyCtrl) {
        Eigen::Vector3d fwd_xz(look_dir.x(), 0.0, look_dir.z());
        if (fwd_xz.norm() > 1e-4) fwd_xz.normalize();

        Eigen::Vector3d right_xz(right_dir.x(), 0.0, right_dir.z());
        if (right_xz.norm() > 1e-4) right_xz.normalize();

        camera_pos_ -= right_xz * io.MouseDelta.x * pan_speed;
        camera_pos_ += fwd_xz * io.MouseDelta.y * pan_speed;
    }

    // 2. Right-Click + Drag: Dolly forward/backward along Look Vector
    if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
        camera_pos_ += look_dir * (-io.MouseDelta.y) * dolly_speed;
    }

    // 3. Middle-Click + Drag: Orbit / Look Rotation (Pitch & Yaw around target)
    if (ImGui::IsMouseDown(ImGuiMouseButton_Middle) || (ImGui::IsMouseDown(ImGuiMouseButton_Left) && io.KeyAlt)) {
        camera_yaw_deg_ -= io.MouseDelta.x * 0.35;
        camera_pitch_deg_ += io.MouseDelta.y * 0.35;
        camera_pitch_deg_ = std::clamp(camera_pitch_deg_, -89.0, 89.0);
    }

    // 4. Mouse Wheel: Fast Dolly
    if (io.MouseWheel != 0.0f) {
        camera_pos_ += look_dir * io.MouseWheel * (pan_speed * 18.0f);
    }

    // 5. Roll Keys (Q / E)
    if (ImGui::IsKeyDown(ImGuiKey_Q)) camera_roll_deg_ -= 0.8;
    if (ImGui::IsKeyDown(ImGuiKey_E)) camera_roll_deg_ += 0.8;
}

void GuiApp::draw_ortho_map(int ortho_idx, ImDrawList* draw_list, float min_x, float min_y, float max_x, float max_y) {
    if (!draw_list) return;

    float center_x = (min_x + max_x) * 0.5f;
    float center_y = (min_y + max_y) * 0.5f;
    double scale = ortho_scale_[ortho_idx];
    Eigen::Vector2d pan = ortho_pan_[ortho_idx];

    // Coordinate mapping helper
    auto world_to_screen = [&](const Eigen::Vector3d& p) -> ImVec2 {
        if (ortho_idx == 0) { // Top View (X-Z)
            float sx = center_x + static_cast<float>((p.x() - pan.x()) * scale);
            float sy = center_y + static_cast<float>((p.z() - pan.y()) * scale);
            return ImVec2(sx, sy);
        } else if (ortho_idx == 1) { // Front View (X-Y)
            float sx = center_x + static_cast<float>((p.x() - pan.x()) * scale);
            float sy = center_y - static_cast<float>((p.y() - pan.y()) * scale);
            return ImVec2(sx, sy);
        } else { // Side View (Z-Y)
            float sx = center_x + static_cast<float>((p.z() - pan.x()) * scale);
            float sy = center_y - static_cast<float>((p.y() - pan.y()) * scale);
            return ImVec2(sx, sy);
        }
    };

    // Draw coordinate grid lines
    float grid_step = (config_.scene_path.find("sponza") != std::string::npos) ? 200.0f : 1.0f;
    for (int i = -10; i <= 10; ++i) {
        ImVec2 p1 = world_to_screen(Eigen::Vector3d(i * grid_step, 0.0, -2000.0));
        ImVec2 p2 = world_to_screen(Eigen::Vector3d(i * grid_step, 0.0, 2000.0));
        draw_list->AddLine(p1, p2, IM_COL32(50, 55, 65, 120), 1.0f);

        ImVec2 p3 = world_to_screen(Eigen::Vector3d(-2000.0, 0.0, i * grid_step));
        ImVec2 p4 = world_to_screen(Eigen::Vector3d(2000.0, 0.0, i * grid_step));
        draw_list->AddLine(p3, p4, IM_COL32(50, 55, 65, 120), 1.0f);
    }

    // Draw 3D Trajectory Spline Path
    if (path_samples_.size() >= 2) {
        for (size_t i = 0; i + 1 < path_samples_.size(); ++i) {
            ImVec2 sp1 = world_to_screen(path_samples_[i]);
            ImVec2 sp2 = world_to_screen(path_samples_[i + 1]);
            draw_list->AddLine(sp1, sp2, IM_COL32(240, 240, 240, 200), 2.0f);
        }
    }

    // Draw Keyframe Diamonds & Labels
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        ImVec2 kp = world_to_screen(keyframes_[i].position);
        float sz = (static_cast<int>(i) == selected_keyframe_idx_) ? 8.0f : 6.0f;
        ImU32 col = (static_cast<int>(i) == selected_keyframe_idx_) ? IM_COL32(255, 230, 40, 255) : IM_COL32(220, 180, 30, 230);

        // Diamond vertices
        ImVec2 d_top(kp.x, kp.y - sz);
        ImVec2 d_right(kp.x + sz, kp.y);
        ImVec2 d_bot(kp.x, kp.y + sz);
        ImVec2 d_left(kp.x - sz, kp.y);

        draw_list->AddQuadFilled(d_top, d_right, d_bot, d_left, col);
        draw_list->AddQuad(d_top, d_right, d_bot, d_left, IM_COL32(20, 20, 20, 255), 1.5f);

        std::string label = "KF" + std::to_string(i + 1);
        draw_list->AddText(ImVec2(kp.x + sz + 3, kp.y - sz), IM_COL32(255, 255, 255, 200), label.c_str());
    }

    // Helper lambda to draw dashed line segments
    auto draw_dashed_line = [&](const ImVec2& p0, const ImVec2& p1, ImU32 col, float thickness, float dash_len, float gap_len) {
        float dx = p1.x - p0.x;
        float dy = p1.y - p0.y;
        float total_dist = std::sqrt(dx * dx + dy * dy);
        if (total_dist < 1e-3f) return;
        float dir_x = dx / total_dist;
        float dir_y = dy / total_dist;
        float step = dash_len + gap_len;
        for (float d = 0.0f; d < total_dist; d += step) {
            float d_end = std::min(d + dash_len, total_dist);
            draw_list->AddLine(
                ImVec2(p0.x + dir_x * d, p0.y + dir_y * d),
                ImVec2(p0.x + dir_x * d_end, p0.y + dir_y * d_end),
                col, thickness
            );
        }
    };

    // -------------------------------------------------------------------------
    // 3D Camera Coordinate System & Basis Vectors (Filament / OpenGL convention)
    // -------------------------------------------------------------------------
    Eigen::Quaterniond q = euler_deg_to_quat(camera_yaw_deg_, camera_pitch_deg_, camera_roll_deg_);
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d look_dir = R * Eigen::Vector3d(0, 0, -1); // Forward (-Z)
    Eigen::Vector3d right_dir = R * Eigen::Vector3d(1, 0, 0); // Right (+X)
    Eigen::Vector3d up_dir = R * Eigen::Vector3d(0, 1, 0);    // Up (+Y)

    ImVec2 cam_screen = world_to_screen(camera_pos_);

    // -------------------------------------------------------------------------
    // 1. Realistic 3D Camera Frustum Wireframe Pyramid (Google Earth Studio)
    // -------------------------------------------------------------------------
    // Camera intrinsics: 640x480, fx=400, fy=400 -> tan(fov_x/2) = 0.8, tan(fov_y/2) = 0.6
    float aspect_ratio = static_cast<float>(sensor_tex_w_) / std::max(1.0f, static_cast<float>(sensor_tex_h_));
    double tan_fov_y_half = 0.60;
    double tan_fov_x_half = tan_fov_y_half * aspect_ratio; // 0.80

    double frustum_screen_len = 105.0; // Readable pixel length in viewport
    double frustum_depth_w = frustum_screen_len / std::max(0.001, scale);
    double frustum_w = frustum_depth_w * tan_fov_x_half;
    double frustum_h = frustum_depth_w * tan_fov_y_half;

    // 4 corners of the far plane in 3D world space
    Eigen::Vector3d far_center = camera_pos_ + look_dir * frustum_depth_w;
    Eigen::Vector3d p_tl_w = far_center - right_dir * frustum_w + up_dir * frustum_h;
    Eigen::Vector3d p_tr_w = far_center + right_dir * frustum_w + up_dir * frustum_h;
    Eigen::Vector3d p_br_w = far_center + right_dir * frustum_w - up_dir * frustum_h;
    Eigen::Vector3d p_bl_w = far_center - right_dir * frustum_w - up_dir * frustum_h;

    ImVec2 p_tl = world_to_screen(p_tl_w);
    ImVec2 p_tr = world_to_screen(p_tr_w);
    ImVec2 p_br = world_to_screen(p_br_w);
    ImVec2 p_bl = world_to_screen(p_bl_w);

    // Subtle translucent volumetric fill on frustum faces (glass effect)
    draw_list->AddTriangleFilled(cam_screen, p_tl, p_tr, IM_COL32(230, 240, 255, 18));
    draw_list->AddTriangleFilled(cam_screen, p_tr, p_br, IM_COL32(230, 240, 255, 14));
    draw_list->AddTriangleFilled(cam_screen, p_br, p_bl, IM_COL32(230, 240, 255, 10));
    draw_list->AddTriangleFilled(cam_screen, p_bl, p_tl, IM_COL32(230, 240, 255, 14));
    draw_list->AddQuadFilled(p_tl, p_tr, p_br, p_bl, IM_COL32(0, 180, 255, 14));

    // Optical Axis: dashed center line from camera apex through frustum base to target
    Eigen::Vector3d target_w = camera_pos_ + look_dir * (frustum_depth_w * 1.85);
    ImVec2 target_screen = world_to_screen(target_w);
    draw_dashed_line(cam_screen, target_screen, IM_COL32(255, 255, 255, 175), 1.4f, 6.0f, 4.0f);
    draw_list->AddCircle(target_screen, 3.5f, IM_COL32(255, 255, 255, 200), 12, 1.2f);

    // 4 Corner Rays from Apex to Corners
    ImU32 ray_col = IM_COL32(230, 240, 255, 200);
    draw_list->AddLine(cam_screen, p_tl, ray_col, 1.6f);
    draw_list->AddLine(cam_screen, p_tr, ray_col, 1.6f);
    draw_list->AddLine(cam_screen, p_br, ray_col, 1.6f);
    draw_list->AddLine(cam_screen, p_bl, ray_col, 1.6f);

    // Far-plane rectangular base frame
    ImU32 base_col = IM_COL32(240, 245, 255, 230);
    draw_list->AddLine(p_tl, p_tr, base_col, 1.8f);
    draw_list->AddLine(p_tr, p_br, base_col, 1.8f);
    draw_list->AddLine(p_br, p_bl, base_col, 1.8f);
    draw_list->AddLine(p_bl, p_tl, base_col, 1.8f);

    // -------------------------------------------------------------------------
    // 2. 3D Orange Camera Body Pyramid Glyph (Google Earth Studio signature)
    // -------------------------------------------------------------------------
    double body_len_px = 22.0;
    double body_w_px = 13.0;
    double body_h_px = 9.0;

    double body_len_w = body_len_px / std::max(0.001, scale);
    double body_w_w = body_w_px / std::max(0.001, scale);
    double body_h_w = body_h_px / std::max(0.001, scale);

    // Apex at camera center (pointing forward) and base extending backward
    Eigen::Vector3d b_apex_w = camera_pos_ + look_dir * (body_len_w * 0.08);
    Eigen::Vector3d b_base_center = camera_pos_ - look_dir * body_len_w;

    Eigen::Vector3d b_tl_w = b_base_center - right_dir * body_w_w + up_dir * body_h_w;
    Eigen::Vector3d b_tr_w = b_base_center + right_dir * body_w_w + up_dir * body_h_w;
    Eigen::Vector3d b_br_w = b_base_center + right_dir * body_w_w - up_dir * body_h_w;
    Eigen::Vector3d b_bl_w = b_base_center - right_dir * body_w_w - up_dir * body_h_w;

    ImVec2 b_apex = world_to_screen(b_apex_w);
    ImVec2 b_tl = world_to_screen(b_tl_w);
    ImVec2 b_tr = world_to_screen(b_tr_w);
    ImVec2 b_br = world_to_screen(b_br_w);
    ImVec2 b_bl = world_to_screen(b_bl_w);

    // Orthographic view direction for depth-sorting faces (painter's algorithm)
    Eigen::Vector3d view_dir(0, 0, 0);
    if (ortho_idx == 0) view_dir = Eigen::Vector3d(0, -1, 0);      // Top view (looking -Y)
    else if (ortho_idx == 1) view_dir = Eigen::Vector3d(0, 0, -1); // Front view (looking -Z)
    else view_dir = Eigen::Vector3d(-1, 0, 0);                     // Side view (looking -X)

    // Lighting vector for 3D directional facet shading
    Eigen::Vector3d light_dir = Eigen::Vector3d(0.35, 0.85, 0.40).normalized();

    struct CameraBodyFace {
        std::vector<ImVec2> pts_2d;
        Eigen::Vector3d normal_3d;
        Eigen::Vector3d center_3d;
        double depth{0.0};
        bool is_quad{false};
    };

    std::vector<CameraBodyFace> body_faces;

    // Top Face (Apex, TR, TL)
    Eigen::Vector3d n_top = (b_tr_w - b_apex_w).cross(b_tl_w - b_apex_w).normalized();
    Eigen::Vector3d c_top = (b_apex_w + b_tr_w + b_tl_w) / 3.0;
    body_faces.push_back({{b_apex, b_tr, b_tl}, n_top, c_top, c_top.dot(view_dir), false});

    // Right Face (Apex, BR, TR)
    Eigen::Vector3d n_right = (b_br_w - b_apex_w).cross(b_tr_w - b_apex_w).normalized();
    Eigen::Vector3d c_right = (b_apex_w + b_br_w + b_tr_w) / 3.0;
    body_faces.push_back({{b_apex, b_br, b_tr}, n_right, c_right, c_right.dot(view_dir), false});

    // Bottom Face (Apex, BL, BR)
    Eigen::Vector3d n_bot = (b_bl_w - b_apex_w).cross(b_br_w - b_apex_w).normalized();
    Eigen::Vector3d c_bot = (b_apex_w + b_bl_w + b_br_w) / 3.0;
    body_faces.push_back({{b_apex, b_bl, b_br}, n_bot, c_bot, c_bot.dot(view_dir), false});

    // Left Face (Apex, TL, BL)
    Eigen::Vector3d n_left = (b_tl_w - b_apex_w).cross(b_bl_w - b_apex_w).normalized();
    Eigen::Vector3d c_left = (b_apex_w + b_tl_w + b_bl_w) / 3.0;
    body_faces.push_back({{b_apex, b_tl, b_bl}, n_left, c_left, c_left.dot(view_dir), false});

    // Back Quad (TL, TR, BR, BL)
    Eigen::Vector3d n_back = -look_dir;
    Eigen::Vector3d c_back = b_base_center;
    body_faces.push_back({{b_tl, b_tr, b_br, b_bl}, n_back, c_back, c_back.dot(view_dir), true});

    // Sort back-to-front (largest depth first along view_dir)
    std::sort(body_faces.begin(), body_faces.end(), [](const CameraBodyFace& a, const CameraBodyFace& b) {
        return a.depth > b.depth;
    });

    // Render sorted faces with Google Earth Studio directional orange tones
    ImU32 outline_col = IM_COL32(40, 16, 10, 230);
    for (const auto& face : body_faces) {
        double dot = std::max(0.0, face.normal_3d.dot(light_dir));
        double intensity = 0.48 + 0.52 * dot;

        int r = std::clamp(static_cast<int>(255.0 * intensity), 120, 255);
        int g = std::clamp(static_cast<int>(100.0 * intensity + 20.0 * (1.0 - intensity)), 40, 160);
        int b = std::clamp(static_cast<int>(30.0 * intensity), 10, 65);
        ImU32 face_col = IM_COL32(r, g, b, 255);

        if (face.is_quad) {
            draw_list->AddQuadFilled(face.pts_2d[0], face.pts_2d[1], face.pts_2d[2], face.pts_2d[3], face_col);
            draw_list->AddQuad(face.pts_2d[0], face.pts_2d[1], face.pts_2d[2], face.pts_2d[3], outline_col, 1.2f);
        } else {
            draw_list->AddTriangleFilled(face.pts_2d[0], face.pts_2d[1], face.pts_2d[2], face_col);
            draw_list->AddTriangle(face.pts_2d[0], face.pts_2d[1], face.pts_2d[2], outline_col, 1.2f);
        }
    }

    // Camera center pivot dot
    draw_list->AddCircleFilled(cam_screen, 4.0f, IM_COL32(255, 255, 255, 255));
    draw_list->AddCircle(cam_screen, 4.0f, IM_COL32(230, 80, 25, 255), 16, 1.6f);
}

bool GuiApp::init() {
    if (!glfwInit()) {
        std::cerr << "[GuiApp] Failed to initialize GLFW." << std::endl;
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window_ = glfwCreateWindow(config_.window_width, config_.window_height, "hesim-3d | Google Earth Studio Trajectory Editor & Simulator", nullptr, nullptr);
    if (!window_) {
        std::cerr << "[GuiApp] Failed to create GLFW window." << std::endl;
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1); // Enable vsync (60 FPS)

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // Google Earth Studio sleek dark styling
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding = 3.0f;
    style.GrabRounding = 3.0f;
    style.PopupRounding = 4.0f;
    style.ScrollbarRounding = 3.0f;
    style.WindowBorderSize = 1.0f;
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.11f, 0.11f, 0.12f, 1.0f);
    style.Colors[ImGuiCol_Header] = ImVec4(0.20f, 0.22f, 0.26f, 1.0f);
    style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.28f, 0.32f, 0.38f, 1.0f);
    style.Colors[ImGuiCol_HeaderActive] = ImVec4(0.35f, 0.40f, 0.48f, 1.0f);

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    create_gl_textures();

    if (!config_.scene_path.empty()) {
        try {
            renderer_ = std::make_unique<FilamentRenderer>(sensor_tex_w_, sensor_tex_h_, "vulkan");
            if (renderer_->load_scene(config_.scene_path)) {
                std::cout << "[GuiApp] Successfully loaded 3D scene: " << config_.scene_path << std::endl;

                std::string scene_lower = config_.scene_path;
                for (auto& c : scene_lower) c = std::tolower(c);

                if (scene_lower.find("sponza") != std::string::npos) {
                    CameraIntrinsics cam;
                    cam.width = sensor_tex_w_;
                    cam.height = sensor_tex_h_;
                    cam.fx = 400.0;
                    cam.fy = 400.0;
                    cam.cx = sensor_tex_w_ * 0.5;
                    cam.cy = sensor_tex_h_ * 0.5;
                    cam.near_plane = 5.0;
                    cam.far_plane = 15000.0;
                    renderer_->set_intrinsics(cam);
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "[GuiApp] Warning: Filament renderer init failed (" << e.what() << "). Using procedural fallback." << std::endl;
            renderer_.reset();
        }
    }

    // Load custom trajectory if specified
    if (!config_.trajectory_path.empty()) {
        load_trajectory_from_json(config_.trajectory_path);
    }

    is_running_ = true;
    return true;
}

void GuiApp::create_gl_textures() {
    auto create_tex = [](uint32_t w, uint32_t h, const uint8_t* data) -> uint32_t {
        GLuint tex;
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
        return tex;
    };

    sensor_texture_id_ = create_tex(sensor_tex_w_, sensor_tex_h_, sensor_img_buffer_.data());
    evs_texture_id_ = create_tex(sensor_tex_w_, sensor_tex_h_, evs_img_buffer_.data());
    orbit_texture_id_ = create_tex(sensor_tex_w_, sensor_tex_h_, sensor_img_buffer_.data());
}

void GuiApp::update_gl_textures() {
    auto update_tex = [](uint32_t tex_id, uint32_t w, uint32_t h, const uint8_t* data) {
        glBindTexture(GL_TEXTURE_2D, tex_id);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, data);
    };

    update_tex(sensor_texture_id_, sensor_tex_w_, sensor_tex_h_, sensor_img_buffer_.data());
    update_tex(evs_texture_id_, sensor_tex_w_, sensor_tex_h_, evs_img_buffer_.data());
    update_tex(orbit_texture_id_, sensor_tex_w_, sensor_tex_h_, sensor_img_buffer_.data());
}

void GuiApp::update_simulation_step(double dt) {
    if (is_playing_) {
        current_time_sec_ += dt * playback_speed_;
        double max_t = config_.duration_sec;
        if (max_t > 0.0 && current_time_sec_ > max_t) {
            current_time_sec_ = std::fmod(current_time_sec_, max_t);
        }
    }

    Eigen::Vector3d pos;
    Eigen::Quaterniond ori;
    compute_camera_pose(pos, ori);

    // Evaluate trajectory IMU kinematics
    TrajectorySample sample = spline_.evaluate(current_time_sec_);

    plot_time_.push_back(current_time_sec_);
    plot_gyro_x_.push_back(sample.angular_velocity_body.x());
    plot_gyro_y_.push_back(sample.angular_velocity_body.y());
    plot_gyro_z_.push_back(sample.angular_velocity_body.z());

    plot_acc_x_.push_back(sample.imu_acceleration.x());
    plot_acc_y_.push_back(sample.imu_acceleration.y());
    plot_acc_z_.push_back(sample.imu_acceleration.z());

    if (plot_time_.size() > MAX_PLOT_HISTORY) {
        plot_time_.erase(plot_time_.begin());
        plot_gyro_x_.erase(plot_gyro_x_.begin());
        plot_gyro_y_.erase(plot_gyro_y_.begin());
        plot_gyro_z_.erase(plot_gyro_z_.begin());
        plot_acc_x_.erase(plot_acc_x_.begin());
        plot_acc_y_.erase(plot_acc_y_.begin());
        plot_acc_z_.erase(plot_acc_z_.begin());
    }

    if (renderer_) {
        renderer_->set_camera_pose(pos, ori);
        renderer_->render_frame(
            sensor_img_buffer_.data(),
            sensor_img_buffer_.size(),
            static_cast<uint64_t>(current_time_sec_ * 1e6)
        );

        // Real-time EVS neuromorphic accumulation
        for (size_t i = 0; i < sensor_tex_w_ * sensor_tex_h_; ++i) {
            size_t idx = i * 3;
            float r = sensor_img_buffer_[idx];
            float g = sensor_img_buffer_[idx + 1];
            float b = sensor_img_buffer_[idx + 2];
            float lum = 0.299f * r + 0.587f * g + 0.114f * b;
            float prev_lum = prev_lum_buffer_[i];
            float diff = std::log(std::max(1.0f, lum)) - std::log(std::max(1.0f, prev_lum));
            prev_lum_buffer_[i] = lum;

            if (diff > config_.event_threshold) {
                evs_img_buffer_[idx] = 255; evs_img_buffer_[idx + 1] = 40; evs_img_buffer_[idx + 2] = 40; // Red ON (+1)
            } else if (diff < -config_.event_threshold) {
                evs_img_buffer_[idx] = 40; evs_img_buffer_[idx + 1] = 80; evs_img_buffer_[idx + 2] = 255; // Blue OFF (-1)
            } else {
                evs_img_buffer_[idx] = static_cast<uint8_t>(evs_img_buffer_[idx] * 0.88f);
                evs_img_buffer_[idx + 1] = static_cast<uint8_t>(evs_img_buffer_[idx + 1] * 0.88f);
                evs_img_buffer_[idx + 2] = static_cast<uint8_t>(evs_img_buffer_[idx + 2] * 0.88f);
            }
        }
    }
    update_gl_textures();
}

void GuiApp::render_ui() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    render_header_bar();
    render_multi_viewport_grid();
    render_timeline_panel();

    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window_, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.09f, 0.09f, 0.10f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void GuiApp::run() {
    auto last_time = std::chrono::high_resolution_clock::now();

    while (!glfwWindowShouldClose(window_) && is_running_) {
        glfwPollEvents();

        auto now = std::chrono::high_resolution_clock::now();
        double dt = std::chrono::duration<double>(now - last_time).count();
        last_time = now;

        update_simulation_step(dt);
        render_ui();
        glfwSwapBuffers(window_);
    }
}

void GuiApp::request_close() {
    is_running_ = false;
    if (window_) {
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
    }
}

int launch_gui(const GuiConfig& config) {
    GuiApp app(config);
    if (!app.init()) {
        return 1;
    }
    app.run();
    return 0;
}

} // namespace hesim3d
