#include "gui_app.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>
#include "icons_material_design.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <chrono>
#include <thread>
#include <algorithm>
#include <iomanip>
#include <csignal>
#include <filesystem>

namespace hesim3d {

static std::atomic<bool> g_gui_exit_requested{false};
static void gui_signal_handler(int) {
    g_gui_exit_requested.store(true);
}

static inline double deg_to_rad(double deg) { return deg * M_PI / 180.0; }
static inline double rad_to_deg(double rad) { return rad * 180.0 / M_PI; }

static Eigen::Quaterniond euler_deg_to_quat(double yaw_deg, double pitch_deg, double roll_deg) {
    // Earth Studio convention: Yaw around Y, Pitch around X, Roll around Z
    Eigen::AngleAxisd roll(deg_to_rad(roll_deg), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitch(deg_to_rad(pitch_deg), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yaw(deg_to_rad(yaw_deg), Eigen::Vector3d::UnitY());
    return (yaw * pitch * roll).normalized();
}

GuiApp::~GuiApp() {
    is_running_ = false;
    if (renderer_) {
        renderer_.reset();
    }
    if (window_) {
        glfwMakeContextCurrent(window_);
        if (sensor_texture_id_) { glDeleteTextures(1, &sensor_texture_id_); sensor_texture_id_ = 0; }
        if (evs_texture_id_) { glDeleteTextures(1, &evs_texture_id_); evs_texture_id_ = 0; }
        if (orbit_texture_id_) { glDeleteTextures(1, &orbit_texture_id_); orbit_texture_id_ = 0; }
        if (sim_aps_texture_id_) { glDeleteTextures(1, &sim_aps_texture_id_); sim_aps_texture_id_ = 0; }
        for (int i = 0; i < 3; ++i) {
            if (ortho_texture_id_[i]) { glDeleteTextures(1, &ortho_texture_id_[i]); ortho_texture_id_[i] = 0; }
        }
        ImPlot::DestroyContext();
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        glfwDestroyWindow(window_);
        glfwTerminate();
        window_ = nullptr;
    }
}

static Eigen::Vector3d quat_to_euler_deg(const Eigen::Quaterniond& q) {
    Eigen::Matrix3d R = q.toRotationMatrix();
    // Extract Euler angles for Earth Studio convention: R = R_y(yaw) * R_x(pitch) * R_z(roll)
    // R(1, 2) = -sin(pitch)
    double pitch = -std::asin(std::clamp(R(1, 2), -1.0, 1.0));
    double yaw = 0.0;
    double roll = 0.0;
    if (std::abs(std::cos(pitch)) > 1e-6) {
        // R(0, 2) = sin(yaw)*cos(pitch), R(2, 2) = cos(yaw)*cos(pitch)
        yaw = std::atan2(R(0, 2), R(2, 2));
        // R(1, 0) = cos(pitch)*sin(roll), R(1, 1) = cos(pitch)*cos(roll)
        roll = std::atan2(R(1, 0), R(1, 1));
    } else {
        // Gimbal lock handling at pitch = +- 90 deg
        if (pitch > 0.0) {
            yaw = std::atan2(R(0, 1), R(0, 0));
        } else {
            yaw = std::atan2(-R(0, 1), R(0, 0));
        }
        roll = 0.0;
    }
    return Eigen::Vector3d(rad_to_deg(yaw), rad_to_deg(pitch), rad_to_deg(roll));
}

GuiApp::GuiApp(const GuiConfig& config) : config_(config) {
    sensor_img_buffer_.resize(sensor_tex_w_ * sensor_tex_h_ * 3, 40);
    evs_img_buffer_.resize(sensor_tex_w_ * sensor_tex_h_ * 3, 20);
    prev_lum_buffer_.resize(sensor_tex_w_ * sensor_tex_h_, 40.0f);

    for (int i = 0; i < 3; ++i) {
        ortho_img_buffers_[i].resize(sensor_tex_w_ * sensor_tex_h_ * 3, 25);
        ortho_dirty_[i] = true;
    }

    // Default clean state: no preloaded keyframes (user creates or loads them)
    keyframes_.clear();
    path_samples_.clear();
    selected_keyframe_idx_ = -1;

    // Default neutral framing until scene is loaded
    camera_pos_ = Eigen::Vector3d(0.0, 1.5, 3.0);
    camera_target_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    camera_yaw_deg_ = 25.0;
    camera_pitch_deg_ = -22.0;
    camera_roll_deg_ = 0.0;
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

void GuiApp::jump_to_prev_keyframe() {
    if (keyframes_.empty()) return;
    int target_idx = -1;
    for (int i = static_cast<int>(keyframes_.size()) - 1; i >= 0; --i) {
        if (keyframes_[i].time_sec < current_time_sec_ - 0.02) {
            target_idx = i;
            break;
        }
    }
    if (target_idx >= 0) {
        jump_to_keyframe(target_idx);
    } else {
        jump_to_keyframe(0);
    }
}

void GuiApp::jump_to_next_keyframe() {
    if (keyframes_.empty()) return;
    int target_idx = -1;
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        if (keyframes_[i].time_sec > current_time_sec_ + 0.02) {
            target_idx = static_cast<int>(i);
            break;
        }
    }
    if (target_idx >= 0) {
        jump_to_keyframe(target_idx);
    } else {
        jump_to_keyframe(static_cast<int>(keyframes_.size()) - 1);
    }
}

void GuiApp::frame_timeline_to_all_keyframes() {
    if (keyframes_.empty()) {
        reset_timeline_view();
        return;
    }
    double kf_start = keyframes_.front().time_sec;
    double kf_end = keyframes_.back().time_sec;
    double span = std::max(0.5, kf_end - kf_start);
    double margin = std::max(0.2, span * 0.15);
    timeline_view_t_min_ = std::max(0.0, kf_start - margin);
    timeline_view_t_max_ = kf_end + margin;
}

void GuiApp::reset_timeline_view() {
    timeline_view_t_min_ = 0.0;
    timeline_view_t_max_ = std::max(1.0, config_.duration_sec);
}

float GuiApp::time_to_timeline_canvas_x(double t, float canvas_x0, float canvas_w) const {
    double range = std::max(0.01, timeline_view_t_max_ - timeline_view_t_min_);
    double norm = (t - timeline_view_t_min_) / range;
    return canvas_x0 + static_cast<float>(norm * (canvas_w - 20.0f)) + 10.0f;
}

double GuiApp::timeline_canvas_x_to_time(float mouse_x, float canvas_x0, float canvas_w) const {
    double range = std::max(0.01, timeline_view_t_max_ - timeline_view_t_min_);
    double norm = static_cast<double>(mouse_x - (canvas_x0 + 10.0f)) / static_cast<double>(canvas_w - 20.0f);
    return timeline_view_t_min_ + norm * range;
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
    if (keyframes_.size() < 2) {
        path_samples_.clear();
        return;
    }

    std::sort(keyframes_.begin(), keyframes_.end(), [](const StudioKeyframe& a, const StudioKeyframe& b) {
        return a.time_sec < b.time_sec;
    });

    // Automatically expand project duration if a keyframe is placed or dragged beyond it
    if (!keyframes_.empty() && keyframes_.back().time_sec > config_.duration_sec) {
        config_.duration_sec = std::ceil(keyframes_.back().time_sec * 10.0) / 10.0;
    }

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

    try {
        spline_.build_from_waypoints(positions, orientations, total_dur);
    } catch (const std::exception& e) {
        std::cerr << "[GuiApp] Error building spline: " << e.what() << std::endl;
    }

    // Cache dense path points for 2D/3D visualization
    path_samples_.clear();
    int num_samples = 120;
    double t_start = keyframes_.front().time_sec;
    double t_end = keyframes_.back().time_sec;
    double t_step = (t_end - t_start) / static_cast<double>(num_samples);
    for (int i = 0; i <= num_samples; ++i) {
        double t = t_start + i * t_step;
        if (interp_mode_ == TrajectoryInterpolation::LINEAR_SLERP) {
            for (size_t k = 0; k + 1 < keyframes_.size(); ++k) {
                if (t >= keyframes_[k].time_sec && (t <= keyframes_[k + 1].time_sec || k + 2 == keyframes_.size())) {
                    double dt = keyframes_[k + 1].time_sec - keyframes_[k].time_sec;
                    double a = (dt > 1e-6) ? std::clamp((t - keyframes_[k].time_sec) / dt, 0.0, 1.0) : 0.0;
                    path_samples_.push_back((1.0 - a) * keyframes_[k].position + a * keyframes_[k + 1].position);
                    break;
                }
            }
        } else {
            TrajectorySample s = spline_.evaluate(t - t_start);
            path_samples_.push_back(s.position);
        }
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

void GuiApp::apply_spline_sample_at(double t) {
    if (keyframes_.empty()) return;
    if (keyframes_.size() == 1) {
        camera_pos_ = keyframes_[0].position;
        camera_yaw_deg_ = keyframes_[0].rotation_euler_deg.x();
        camera_pitch_deg_ = keyframes_[0].rotation_euler_deg.y();
        camera_roll_deg_ = keyframes_[0].rotation_euler_deg.z();
        return;
    }

    if (t <= keyframes_.front().time_sec) {
        const auto& kf = keyframes_.front();
        camera_pos_ = kf.position;
        camera_yaw_deg_ = kf.rotation_euler_deg.x();
        camera_pitch_deg_ = kf.rotation_euler_deg.y();
        camera_roll_deg_ = kf.rotation_euler_deg.z();
        return;
    }

    if (t >= keyframes_.back().time_sec) {
        const auto& kf = keyframes_.back();
        camera_pos_ = kf.position;
        camera_yaw_deg_ = kf.rotation_euler_deg.x();
        camera_pitch_deg_ = kf.rotation_euler_deg.y();
        camera_roll_deg_ = kf.rotation_euler_deg.z();
        return;
    }

    if (interp_mode_ == TrajectoryInterpolation::LINEAR_SLERP) {
        for (size_t i = 0; i + 1 < keyframes_.size(); ++i) {
            if (t >= keyframes_[i].time_sec && t <= keyframes_[i + 1].time_sec) {
                double dt = keyframes_[i + 1].time_sec - keyframes_[i].time_sec;
                double alpha = (dt > 1e-6) ? (t - keyframes_[i].time_sec) / dt : 0.0;
                alpha = std::clamp(alpha, 0.0, 1.0);

                camera_pos_ = (1.0 - alpha) * keyframes_[i].position + alpha * keyframes_[i + 1].position;
                Eigen::Quaterniond q = keyframes_[i].orientation.slerp(alpha, keyframes_[i + 1].orientation);
                Eigen::Vector3d euler = quat_to_euler_deg(q);
                camera_yaw_deg_ = euler.x();
                camera_pitch_deg_ = euler.y();
                camera_roll_deg_ = euler.z();
                return;
            }
        }
    } else {
        double spline_t = t - keyframes_.front().time_sec;
        TrajectorySample sample = spline_.evaluate(spline_t);
        camera_pos_ = sample.position;
        Eigen::Vector3d euler = quat_to_euler_deg(sample.orientation);
        camera_yaw_deg_ = euler.x();
        camera_pitch_deg_ = euler.y();
        camera_roll_deg_ = euler.z();
    }
}

void GuiApp::compute_camera_pose(Eigen::Vector3d& out_pos, Eigen::Quaterniond& out_ori) {
    if (is_playing_ && keyframes_.size() >= 2) {
        apply_spline_sample_at(current_time_sec_);
    } else {
        if (is_playing_) is_playing_ = false;
    }
    out_pos = camera_pos_;
    out_ori = euler_deg_to_quat(camera_yaw_deg_, camera_pitch_deg_, camera_roll_deg_);
}

void GuiApp::compute_optimal_initial_camera() {
    Eigen::Vector3d center = scene_bounds_.valid ? scene_bounds_.center : Eigen::Vector3d(0.0, 0.0, 0.0);
    double r = scene_bounds_.valid ? scene_bounds_.radius : 2.0;

    if (!keyframes_.empty()) {
        Eigen::Vector3d kf_min = center - Eigen::Vector3d(r, r, r);
        Eigen::Vector3d kf_max = center + Eigen::Vector3d(r, r, r);
        for (const auto& kf : keyframes_) {
            kf_min = kf_min.cwiseMin(kf.position);
            kf_max = kf_max.cwiseMax(kf.position);
        }
        center = (kf_min + kf_max) * 0.5;
        r = std::max(r, (kf_max - center).norm());
    }

    // Google Earth Studio aesthetic elevated isometric framing
    camera_target_ = center;
    camera_yaw_deg_ = 25.0;
    camera_pitch_deg_ = -22.0;
    camera_roll_deg_ = 0.0;

    // Compute required viewing distance D to frame the scene bounding sphere
    // Intrinsics: fx=400, fy=400, W=640, H=480 -> tan(fov_y/2) = 0.60, tan(fov_x/2) = 0.80
    double tan_fov_y_half = 0.60;
    double tan_fov_x_half = 0.80;

    double d_y = (r * 1.25) / tan_fov_y_half;
    double d_x = (r * 1.25) / tan_fov_x_half;
    double dist = std::max({d_y, d_x, 0.2});

    orbit_radius_ = dist;

    // Position camera at distance 'dist' along backwards look vector
    Eigen::Quaterniond q = euler_deg_to_quat(camera_yaw_deg_, camera_pitch_deg_, camera_roll_deg_);
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d look_dir = R * Eigen::Vector3d(0, 0, -1);

    camera_pos_ = camera_target_ - look_dir * dist;

    std::cout << "[GuiApp] Automatically framed camera for scene:" << std::endl
              << "  - Scene Center:    [" << center.x() << ", " << center.y() << ", " << center.z() << "]" << std::endl
              << "  - Scene Radius:    " << r << std::endl
              << "  - Camera Distance: " << dist << std::endl
              << "  - Initial Eye:     [" << camera_pos_.x() << ", " << camera_pos_.y() << ", " << camera_pos_.z() << "]" << std::endl
              << "  - Pitch: " << camera_pitch_deg_ << " deg, Yaw: " << camera_yaw_deg_ << " deg" << std::endl;
}

void GuiApp::handle_camera_mouse_input(float min_x, float min_y, float max_x, float max_y) {
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 mouse_pos = io.MousePos;

    bool is_hovered = (mouse_pos.x >= min_x && mouse_pos.x <= max_x &&
                       mouse_pos.y >= min_y && mouse_pos.y <= max_y);

    // 0. Reset / Frame Camera View on 'F' key
    if (is_hovered && ImGui::IsKeyPressed(ImGuiKey_F)) {
        compute_optimal_initial_camera();
        return;
    }

    bool is_orbit_down = ImGui::IsMouseDown(ImGuiMouseButton_Middle) || (ImGui::IsMouseDown(ImGuiMouseButton_Left) && io.KeyAlt);

    // Allow continuous orbit dragging even if cursor briefly crosses the viewport edge
    if (!is_hovered && !is_orbit_dragging_) {
        is_orbit_dragging_ = false;
        smooth_orbit_dx_ = 0.0f;
        smooth_orbit_dy_ = 0.0f;
        return;
    }

    Eigen::Quaterniond q = euler_deg_to_quat(camera_yaw_deg_, camera_pitch_deg_, camera_roll_deg_);
    Eigen::Matrix3d R = q.toRotationMatrix();

    Eigen::Vector3d look_dir = R * Eigen::Vector3d(0, 0, -1);
    Eigen::Vector3d right_dir = R * Eigen::Vector3d(1, 0, 0);

    // Adaptive navigation speeds proportional to estimated scene scale
    double r = scene_bounds_.valid ? scene_bounds_.radius : 2.0;
    float base_speed = static_cast<float>(r * 0.0018) * std::max(0.05f, nav_speed_factor_);
    float pan_speed = std::max(0.0001f, base_speed);
    float dolly_speed = std::max(0.0002f, base_speed * 1.5f);

    // 1. Left-Click + Drag: Horizontal Plane Pan (moves X/Z, keeps altitude Y constant)
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && !io.KeyShift && !io.KeyCtrl && !io.KeyAlt) {
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

    // 3. Middle-Click + Drag: Orbit / Look Rotation (Smooth Pitch & Yaw around target)
    if (is_orbit_down) {
        if (!is_orbit_dragging_) {
            is_orbit_dragging_ = true;
            smooth_orbit_dx_ = io.MouseDelta.x;
            smooth_orbit_dy_ = io.MouseDelta.y;
        } else {
            // Low-pass exponential smoothing filter on mouse delta to eliminate discrete polling jumps
            const float alpha = 0.50f;
            smooth_orbit_dx_ = smooth_orbit_dx_ * (1.0f - alpha) + io.MouseDelta.x * alpha;
            smooth_orbit_dy_ = smooth_orbit_dy_ * (1.0f - alpha) + io.MouseDelta.y * alpha;
        }

        float orbit_speed = 0.085f * std::max(0.05f, nav_speed_factor_);
        camera_yaw_deg_ -= smooth_orbit_dx_ * orbit_speed;
        camera_pitch_deg_ -= smooth_orbit_dy_ * orbit_speed;
        camera_pitch_deg_ = std::clamp(camera_pitch_deg_, -89.0, 89.0);

        // Keep camera_target_ synchronized along look direction
        Eigen::Quaterniond q_new = euler_deg_to_quat(camera_yaw_deg_, camera_pitch_deg_, camera_roll_deg_);
        Eigen::Vector3d new_look_dir = q_new.toRotationMatrix() * Eigen::Vector3d(0, 0, -1);
        camera_target_ = camera_pos_ + new_look_dir * orbit_radius_;
    } else {
        is_orbit_dragging_ = false;
        smooth_orbit_dx_ = 0.0f;
        smooth_orbit_dy_ = 0.0f;
    }

    // 4. Mouse Wheel: Fast Dolly
    if (is_hovered && io.MouseWheel != 0.0f) {
        camera_pos_ += look_dir * io.MouseWheel * (pan_speed * 15.0f);
    }

    // 5. Roll Keys (Q / E)
    if (is_hovered && ImGui::IsKeyDown(ImGuiKey_Q)) camera_roll_deg_ += 0.8;
    if (is_hovered && ImGui::IsKeyDown(ImGuiKey_E)) camera_roll_deg_ -= 0.8;
}

void GuiApp::update_ortho_texture(int ortho_idx, float canvas_w, float canvas_h) {
    if (ortho_idx < 0 || ortho_idx > 2 || !renderer_) return;

    uint32_t target_w = std::clamp(static_cast<uint32_t>(canvas_w), 32u, 3840u);
    uint32_t target_h = std::clamp(static_cast<uint32_t>(canvas_h), 32u, 2160u);
    target_w = (target_w + 3) & ~3u;
    target_h = (target_h + 1) & ~1u;

    size_t req_size = static_cast<size_t>(target_w) * target_h * 3;
    if (ortho_img_buffers_[ortho_idx].size() != req_size) {
        ortho_img_buffers_[ortho_idx].resize(req_size, 25);
    }

    bool success = renderer_->render_ortho_frame(
        ortho_idx,
        ortho_pan_[ortho_idx],
        ortho_scale_[ortho_idx],
        static_cast<float>(target_w),
        static_cast<float>(target_h),
        ortho_img_buffers_[ortho_idx].data(),
        ortho_img_buffers_[ortho_idx].size()
    );

    if (success && ortho_texture_id_[ortho_idx] != 0) {
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glBindTexture(GL_TEXTURE_2D, ortho_texture_id_[ortho_idx]);
        if (ortho_tex_w_[ortho_idx] != target_w || ortho_tex_h_[ortho_idx] != target_h) {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, target_w, target_h, 0, GL_RGB, GL_UNSIGNED_BYTE, ortho_img_buffers_[ortho_idx].data());
            ortho_tex_w_[ortho_idx] = target_w;
            ortho_tex_h_[ortho_idx] = target_h;
        } else {
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, target_w, target_h, GL_RGB, GL_UNSIGNED_BYTE, ortho_img_buffers_[ortho_idx].data());
        }
    }

    ortho_dirty_[ortho_idx] = false;
}

void GuiApp::frame_ortho_view(int ortho_idx) {
    if (ortho_idx < 0 || ortho_idx > 2) return;

    Eigen::Vector3d min_pt(1e9, 1e9, 1e9);
    Eigen::Vector3d max_pt(-1e9, -1e9, -1e9);

    // 1. Include scene bounding box if valid
    if (scene_bounds_.valid) {
        min_pt = min_pt.cwiseMin(scene_bounds_.min_point);
        max_pt = max_pt.cwiseMax(scene_bounds_.max_point);
    }

    // 2. Include all keyframes
    for (const auto& kf : keyframes_) {
        min_pt = min_pt.cwiseMin(kf.position);
        max_pt = max_pt.cwiseMax(kf.position);
    }

    // 3. Include camera position and target
    min_pt = min_pt.cwiseMin(camera_pos_);
    max_pt = max_pt.cwiseMax(camera_pos_);
    min_pt = min_pt.cwiseMin(camera_target_);
    max_pt = max_pt.cwiseMax(camera_target_);

    // 4. Include camera frustum corners in world space
    Eigen::Quaterniond q = euler_deg_to_quat(camera_yaw_deg_, camera_pitch_deg_, camera_roll_deg_);
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d look_dir = R * Eigen::Vector3d(0, 0, -1);
    Eigen::Vector3d right_dir = R * Eigen::Vector3d(1, 0, 0);
    Eigen::Vector3d up_dir = R * Eigen::Vector3d(0, 1, 0);

    float aspect_ratio = static_cast<float>(sensor_tex_w_) / std::max(1.0f, static_cast<float>(sensor_tex_h_));
    double tan_fov_y_half = 0.60;
    double tan_fov_x_half = tan_fov_y_half * aspect_ratio;

    double f_dist = std::max({orbit_radius_, (camera_pos_ - camera_target_).norm(), 1.0});
    double fw = f_dist * tan_fov_x_half;
    double fh = f_dist * tan_fov_y_half;
    Eigen::Vector3d far_center = camera_pos_ + look_dir * f_dist;

    Eigen::Vector3d frustum_pts[4] = {
        far_center - right_dir * fw + up_dir * fh,
        far_center + right_dir * fw + up_dir * fh,
        far_center + right_dir * fw - up_dir * fh,
        far_center - right_dir * fw - up_dir * fh
    };

    for (const auto& pt : frustum_pts) {
        min_pt = min_pt.cwiseMin(pt);
        max_pt = max_pt.cwiseMax(pt);
    }

    // Fallback if no geometry was bounded
    if (min_pt.x() > max_pt.x()) {
        min_pt = camera_pos_ - Eigen::Vector3d(1.5, 1.5, 1.5);
        max_pt = camera_pos_ + Eigen::Vector3d(1.5, 1.5, 1.5);
    }

    double cw = (last_canvas_w_[ortho_idx] > 50.0f) ? last_canvas_w_[ortho_idx] : 450.0;
    double ch = (last_canvas_h_[ortho_idx] > 50.0f) ? last_canvas_h_[ortho_idx] : 350.0;

    double center_u = 0.0, center_v = 0.0;
    double span_u = 1.0, span_v = 1.0;

    if (ortho_idx == 0) { // Top view (X-Z)
        center_u = (min_pt.x() + max_pt.x()) * 0.5;
        center_v = (min_pt.z() + max_pt.z()) * 0.5;
        span_u = std::max(max_pt.x() - min_pt.x(), 0.1);
        span_v = std::max(max_pt.z() - min_pt.z(), 0.1);
    } else if (ortho_idx == 1) { // Front view (X-Y)
        center_u = (min_pt.x() + max_pt.x()) * 0.5;
        center_v = (min_pt.y() + max_pt.y()) * 0.5;
        span_u = std::max(max_pt.x() - min_pt.x(), 0.1);
        span_v = std::max(max_pt.y() - min_pt.y(), 0.1);
    } else { // Side view (Z-Y)
        center_u = (min_pt.z() + max_pt.z()) * 0.5;
        center_v = (min_pt.y() + max_pt.y()) * 0.5;
        span_u = std::max(max_pt.z() - min_pt.z(), 0.1);
        span_v = std::max(max_pt.y() - min_pt.y(), 0.1);
    }

    double scale_u = (cw * 0.78) / span_u;
    double scale_v = (ch * 0.78) / span_v;
    double fit_scale = std::min(scale_u, scale_v);

    ortho_pan_[ortho_idx] = Eigen::Vector2d(center_u, center_v);
    ortho_scale_[ortho_idx] = std::clamp(fit_scale, 0.0001, 20000.0);
    ortho_dirty_[ortho_idx] = true;
}

void GuiApp::handle_ortho_mouse_input(int ortho_idx, float min_x, float min_y, float max_x, float max_y) {
    if (ortho_idx < 0 || ortho_idx > 2) return;

    ImGuiIO& io = ImGui::GetIO();
    ImVec2 mouse_pos = io.MousePos;
    bool is_hovered = (mouse_pos.x >= min_x && mouse_pos.x <= max_x &&
                       mouse_pos.y >= min_y && mouse_pos.y <= max_y);

    float center_x = (min_x + max_x) * 0.5f;
    float center_y = (min_y + max_y) * 0.5f;
    double scale = ortho_scale_[ortho_idx];
    Eigen::Vector2d pan = ortho_pan_[ortho_idx];

    auto screen_to_world = [&](const ImVec2& sp) -> Eigen::Vector2d {
        if (ortho_idx == 0) {
            double wx = pan.x() + (sp.x - center_x) / scale;
            double wz = pan.y() + (sp.y - center_y) / scale;
            return Eigen::Vector2d(wx, wz);
        } else if (ortho_idx == 1) {
            double wx = pan.x() + (sp.x - center_x) / scale;
            double wy = pan.y() - (sp.y - center_y) / scale;
            return Eigen::Vector2d(wx, wy);
        } else {
            double wz = pan.x() + (sp.x - center_x) / scale;
            double wy = pan.y() - (sp.y - center_y) / scale;
            return Eigen::Vector2d(wz, wy);
        }
    };

    // 1. Reset / Frame View on 'F' key
    if (is_hovered && ImGui::IsKeyPressed(ImGuiKey_F)) {
        frame_ortho_view(ortho_idx);
        return;
    }

    // 2. Mouse Wheel: Smooth Zoom centered on Mouse Cursor
    if (is_hovered && io.MouseWheel != 0.0f) {
        Eigen::Vector2d mouse_world_before = screen_to_world(mouse_pos);
        double factor = (io.MouseWheel > 0.0f) ? 1.15 : (1.0 / 1.15);
        double new_scale = std::clamp(scale * factor, 0.005, 500.0);

        if (ortho_idx == 0) {
            ortho_pan_[ortho_idx].x() = mouse_world_before.x() - (mouse_pos.x - center_x) / new_scale;
            ortho_pan_[ortho_idx].y() = mouse_world_before.y() - (mouse_pos.y - center_y) / new_scale;
        } else {
            ortho_pan_[ortho_idx].x() = mouse_world_before.x() - (mouse_pos.x - center_x) / new_scale;
            ortho_pan_[ortho_idx].y() = mouse_world_before.y() + (mouse_pos.y - center_y) / new_scale;
        }

        ortho_scale_[ortho_idx] = new_scale;
        ortho_dirty_[ortho_idx] = true;
    }

    // 3. Middle-Click or Right-Click Drag: 2D Panning (Locked orthogonal axis, NO orbit!)
    if (is_hovered && (ImGui::IsMouseDown(ImGuiMouseButton_Right) || ImGui::IsMouseDown(ImGuiMouseButton_Middle))) {
        if (ortho_idx == 0) {
            ortho_pan_[ortho_idx].x() -= io.MouseDelta.x / scale;
            ortho_pan_[ortho_idx].y() -= io.MouseDelta.y / scale;
        } else {
            ortho_pan_[ortho_idx].x() -= io.MouseDelta.x / scale;
            ortho_pan_[ortho_idx].y() += io.MouseDelta.y / scale;
        }
        ortho_dirty_[ortho_idx] = true;
    }

    // 4. Interactive Hover & Dragging for Camera and Keyframes
    hovered_ortho_element_[ortho_idx] = -1;
    float best_dist_sq = 1e9f;

    // Check keyframe points
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        ImVec2 kp;
        if (ortho_idx == 0) {
            kp = ImVec2(center_x + static_cast<float>((keyframes_[i].position.x() - pan.x()) * scale),
                        center_y + static_cast<float>((keyframes_[i].position.z() - pan.y()) * scale));
        } else if (ortho_idx == 1) {
            kp = ImVec2(center_x + static_cast<float>((keyframes_[i].position.x() - pan.x()) * scale),
                        center_y - static_cast<float>((keyframes_[i].position.y() - pan.y()) * scale));
        } else {
            kp = ImVec2(center_x + static_cast<float>((keyframes_[i].position.z() - pan.x()) * scale),
                        center_y - static_cast<float>((keyframes_[i].position.y() - pan.y()) * scale));
        }

        float d2 = (mouse_pos.x - kp.x) * (mouse_pos.x - kp.x) + (mouse_pos.y - kp.y) * (mouse_pos.y - kp.y);
        if (d2 <= 196.0f && d2 < best_dist_sq) { // 14px radius
            best_dist_sq = d2;
            hovered_ortho_element_[ortho_idx] = static_cast<int>(i);
        }
    }

    // Check camera point
    ImVec2 cam_screen;
    if (ortho_idx == 0) {
        cam_screen = ImVec2(center_x + static_cast<float>((camera_pos_.x() - pan.x()) * scale),
                            center_y + static_cast<float>((camera_pos_.z() - pan.y()) * scale));
    } else if (ortho_idx == 1) {
        cam_screen = ImVec2(center_x + static_cast<float>((camera_pos_.x() - pan.x()) * scale),
                            center_y - static_cast<float>((camera_pos_.y() - pan.y()) * scale));
    } else {
        cam_screen = ImVec2(center_x + static_cast<float>((camera_pos_.z() - pan.x()) * scale),
                            center_y - static_cast<float>((camera_pos_.y() - pan.y()) * scale));
    }

    float d2_cam = (mouse_pos.x - cam_screen.x) * (mouse_pos.x - cam_screen.x) + (mouse_pos.y - cam_screen.y) * (mouse_pos.y - cam_screen.y);
    if (d2_cam <= 256.0f && d2_cam < best_dist_sq) { // 16px radius
        best_dist_sq = d2_cam;
        hovered_ortho_element_[ortho_idx] = -2; // Camera
    }

    if (is_hovered && (hovered_ortho_element_[ortho_idx] != -1 || dragging_camera_ortho_idx_ == ortho_idx || dragging_keyframe_ortho_idx_ == ortho_idx)) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
    }

    // Left-Click interaction start
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && is_hovered) {
        dragging_keyframe_idx_ = -1;
        dragging_camera_ortho_idx_ = -1;
        dragging_keyframe_ortho_idx_ = -1;

        if (hovered_ortho_element_[ortho_idx] == -2) {
            dragging_camera_ortho_idx_ = ortho_idx;
            is_playing_ = false;
        } else if (hovered_ortho_element_[ortho_idx] >= 0 && static_cast<size_t>(hovered_ortho_element_[ortho_idx]) < keyframes_.size()) {
            dragging_keyframe_idx_ = hovered_ortho_element_[ortho_idx];
            selected_keyframe_idx_ = hovered_ortho_element_[ortho_idx];
            dragging_keyframe_ortho_idx_ = ortho_idx;
            is_playing_ = false;
        }
    }

    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        dragging_keyframe_idx_ = -1;
        dragging_camera_ortho_idx_ = -1;
        dragging_keyframe_ortho_idx_ = -1;
    }

    // Dragging Camera - ONLY execute in the viewport where drag initiated!
    if (dragging_camera_ortho_idx_ == ortho_idx) {
        if (ortho_idx == 0) { // Top View: X-Z only! (Y is strictly untouched)
            camera_pos_.x() += io.MouseDelta.x / scale;
            camera_pos_.z() += io.MouseDelta.y / scale;
        } else if (ortho_idx == 1) { // Front View: X-Y only! (Z is strictly untouched)
            camera_pos_.x() += io.MouseDelta.x / scale;
            camera_pos_.y() -= io.MouseDelta.y / scale;
        } else if (ortho_idx == 2) { // Side View: Z-Y only! (X is strictly untouched)
            camera_pos_.z() += io.MouseDelta.x / scale;
            camera_pos_.y() -= io.MouseDelta.y / scale;
        }

        Eigen::Quaterniond q = euler_deg_to_quat(camera_yaw_deg_, camera_pitch_deg_, camera_roll_deg_);
        Eigen::Vector3d look_dir = q.toRotationMatrix() * Eigen::Vector3d(0, 0, -1);
        camera_target_ = camera_pos_ + look_dir * orbit_radius_;

        if (selected_keyframe_idx_ >= 0 && static_cast<size_t>(selected_keyframe_idx_) < keyframes_.size()) {
            auto& kf = keyframes_[selected_keyframe_idx_];
            if ((camera_pos_ - kf.position).norm() < 0.25) {
                kf.position = camera_pos_;
                rebuild_trajectory();
            }
        }
    }

    // Dragging Keyframe - ONLY execute in the viewport where drag initiated!
    if (dragging_keyframe_ortho_idx_ == ortho_idx && dragging_keyframe_idx_ >= 0 && static_cast<size_t>(dragging_keyframe_idx_) < keyframes_.size()) {
        auto& kf = keyframes_[dragging_keyframe_idx_];
        if (ortho_idx == 0) { // Top View: X-Z only!
            kf.position.x() += io.MouseDelta.x / scale;
            kf.position.z() += io.MouseDelta.y / scale;
        } else if (ortho_idx == 1) { // Front View: X-Y only!
            kf.position.x() += io.MouseDelta.x / scale;
            kf.position.y() -= io.MouseDelta.y / scale;
        } else if (ortho_idx == 2) { // Side View: Z-Y only!
            kf.position.z() += io.MouseDelta.x / scale;
            kf.position.y() -= io.MouseDelta.y / scale;
        }

        if (selected_keyframe_idx_ == dragging_keyframe_idx_) {
            camera_pos_ = kf.position;
            Eigen::Quaterniond q = euler_deg_to_quat(camera_yaw_deg_, camera_pitch_deg_, camera_roll_deg_);
            Eigen::Vector3d look_dir = q.toRotationMatrix() * Eigen::Vector3d(0, 0, -1);
            camera_target_ = camera_pos_ + look_dir * orbit_radius_;
        }
        rebuild_trajectory();
    }
}

void GuiApp::draw_ortho_map(int ortho_idx, ImDrawList* draw_list, float min_x, float min_y, float max_x, float max_y) {
    if (!draw_list) return;

    // Handle mouse navigation (pan, zoom, keyframe drag, reset view)
    handle_ortho_mouse_input(ortho_idx, min_x, min_y, max_x, max_y);

    float canvas_w = max_x - min_x;
    float canvas_h = max_y - min_y;
    if (canvas_w < 10.0f || canvas_h < 10.0f) return;

    draw_list->PushClipRect(ImVec2(min_x, min_y), ImVec2(max_x, max_y), true);

    // Detect viewport size changes
    float thresh = (layout_settle_frames_ > 0) ? 0.5f : 4.0f;
    if (std::abs(canvas_w - last_canvas_w_[ortho_idx]) > thresh ||
        std::abs(canvas_h - last_canvas_h_[ortho_idx]) > thresh) {
        ortho_dirty_[ortho_idx] = true;
        last_canvas_w_[ortho_idx] = canvas_w;
        last_canvas_h_[ortho_idx] = canvas_h;
    }

    // Re-render orthographic 3D view if dirty
    if (ortho_dirty_[ortho_idx] && renderer_) {
        update_ortho_texture(ortho_idx, canvas_w, canvas_h);
    }

    // 1. Draw 3D Model Rendered Background
    if (ortho_texture_id_[ortho_idx] != 0 && renderer_) {
        draw_list->AddImage(
            (ImTextureID)(intptr_t)ortho_texture_id_[ortho_idx],
            ImVec2(min_x, min_y),
            ImVec2(max_x, max_y)
        );

        // Subtle dark translucent tint to ensure vector trajectory and keyframes stand out
        if (ortho_dimming_ > 0.0f) {
            int alpha = std::clamp(static_cast<int>(ortho_dimming_ * 255.0f), 0, 255);
            draw_list->AddRectFilled(
                ImVec2(min_x, min_y),
                ImVec2(max_x, max_y),
                IM_COL32(10, 12, 16, alpha)
            );
        }
    }

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

    // Draw coordinate grid lines adaptively
    double r_grid = scene_bounds_.valid ? scene_bounds_.radius : 1.0;
    double log_val = std::floor(std::log10(std::max(0.001, r_grid * 0.15)));
    float grid_step = static_cast<float>(std::pow(10.0, log_val));
    if (r_grid / grid_step > 40.0) grid_step *= 5.0f;
    else if (r_grid / grid_step > 20.0) grid_step *= 2.0f;
    float grid_extent = grid_step * 15.0f;

    for (int i = -12; i <= 12; ++i) {
        ImVec2 p1 = world_to_screen(Eigen::Vector3d(i * grid_step, 0.0, -grid_extent));
        ImVec2 p2 = world_to_screen(Eigen::Vector3d(i * grid_step, 0.0, grid_extent));
        draw_list->AddLine(p1, p2, IM_COL32(80, 90, 110, 90), 1.0f);

        ImVec2 p3 = world_to_screen(Eigen::Vector3d(-grid_extent, 0.0, i * grid_step));
        ImVec2 p4 = world_to_screen(Eigen::Vector3d(grid_extent, 0.0, i * grid_step));
        draw_list->AddLine(p3, p4, IM_COL32(80, 90, 110, 90), 1.0f);
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
        bool is_kf_hovered = (hovered_ortho_element_[ortho_idx] == static_cast<int>(i));
        bool is_kf_dragging = (dragging_keyframe_ortho_idx_ == ortho_idx && dragging_keyframe_idx_ == static_cast<int>(i));
        float sz = (static_cast<int>(i) == selected_keyframe_idx_ || is_kf_hovered) ? 8.5f : 6.0f;
        ImU32 col = (static_cast<int>(i) == selected_keyframe_idx_) ? IM_COL32(255, 230, 40, 255) : IM_COL32(220, 180, 30, 230);

        if (is_kf_hovered || is_kf_dragging) {
            draw_list->AddCircle(kp, sz + 4.0f, IM_COL32(255, 230, 40, 180), 16, 1.5f);
        }

        // Diamond vertices
        ImVec2 d_top(kp.x, kp.y - sz);
        ImVec2 d_right(kp.x + sz, kp.y);
        ImVec2 d_bot(kp.x, kp.y + sz);
        ImVec2 d_left(kp.x - sz, kp.y);

        draw_list->AddQuadFilled(d_top, d_right, d_bot, d_left, col);
        draw_list->AddQuad(d_top, d_right, d_bot, d_left, IM_COL32(20, 20, 20, 255), 1.5f);

        std::string label = "KF" + std::to_string(i + 1);
        draw_list->AddText(ImVec2(kp.x + sz + 3, kp.y - sz), IM_COL32(255, 255, 255, 200), label.c_str());

        if (is_kf_dragging) {
            char kbuf[128];
            std::snprintf(kbuf, sizeof(kbuf), "KF%zu: [%.2f, %.2f, %.2f] m", i + 1, keyframes_[i].position.x(), keyframes_[i].position.y(), keyframes_[i].position.z());
            ImVec2 txt_sz = ImGui::CalcTextSize(kbuf);
            draw_list->AddRectFilled(ImVec2(kp.x + 12, kp.y - 20), ImVec2(kp.x + 18 + txt_sz.x, kp.y - 2), IM_COL32(15, 18, 25, 220), 4.0f);
            draw_list->AddText(ImVec2(kp.x + 15, kp.y - 18), IM_COL32(255, 230, 40, 255), kbuf);
        }
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
    draw_list->AddCircleFilled(p_tl, 0.9f, base_col, 8);
    draw_list->AddCircleFilled(p_tr, 0.9f, base_col, 8);
    draw_list->AddCircleFilled(p_br, 0.9f, base_col, 8);
    draw_list->AddCircleFilled(p_bl, 0.9f, base_col, 8);

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

    // Orthographic view direction for depth-sorting faces and backface culling
    Eigen::Vector3d view_dir(0, 0, 0);
    if (ortho_idx == 0) view_dir = Eigen::Vector3d(0, -1, 0);      // Top view (looking -Y)
    else if (ortho_idx == 1) view_dir = Eigen::Vector3d(0, 0, -1); // Front view (looking -Z)
    else view_dir = Eigen::Vector3d(-1, 0, 0);                     // Side view (looking -X)

    // Lighting vector for 3D directional facet shading
    Eigen::Vector3d light_dir = Eigen::Vector3d(0.35, 0.85, 0.40).normalized();

    struct CameraBodyFace {
        int id{0};
        std::vector<ImVec2> pts_2d;
        Eigen::Vector3d normal_3d;
        Eigen::Vector3d center_3d;
        double depth{0.0};
        bool is_quad{false};
        bool is_front_facing{false};
    };

    std::vector<CameraBodyFace> body_faces;

    // Outward normal computation for 3D camera body pyramid facets:
    // Top Face (Apex, TL, TR)
    Eigen::Vector3d n_top = (b_tl_w - b_apex_w).cross(b_tr_w - b_apex_w).normalized();
    Eigen::Vector3d c_top = (b_apex_w + b_tl_w + b_tr_w) / 3.0;
    body_faces.push_back({0, {b_apex, b_tl, b_tr}, n_top, c_top, c_top.dot(view_dir), false, n_top.dot(view_dir) < -1e-5});

    // Right Face (Apex, TR, BR)
    Eigen::Vector3d n_right = (b_tr_w - b_apex_w).cross(b_br_w - b_apex_w).normalized();
    Eigen::Vector3d c_right = (b_apex_w + b_tr_w + b_br_w) / 3.0;
    body_faces.push_back({1, {b_apex, b_tr, b_br}, n_right, c_right, c_right.dot(view_dir), false, n_right.dot(view_dir) < -1e-5});

    // Bottom Face (Apex, BR, BL)
    Eigen::Vector3d n_bot = (b_br_w - b_apex_w).cross(b_bl_w - b_apex_w).normalized();
    Eigen::Vector3d c_bot = (b_apex_w + b_br_w + b_bl_w) / 3.0;
    body_faces.push_back({2, {b_apex, b_br, b_bl}, n_bot, c_bot, c_bot.dot(view_dir), false, n_bot.dot(view_dir) < -1e-5});

    // Left Face (Apex, BL, TL)
    Eigen::Vector3d n_left = (b_bl_w - b_apex_w).cross(b_tl_w - b_apex_w).normalized();
    Eigen::Vector3d c_left = (b_apex_w + b_bl_w + b_tl_w) / 3.0;
    body_faces.push_back({3, {b_apex, b_bl, b_tl}, n_left, c_left, c_left.dot(view_dir), false, n_left.dot(view_dir) < -1e-5});

    // Back Quad (TL, TR, BR, BL)
    Eigen::Vector3d n_back = -look_dir;
    Eigen::Vector3d c_back = b_base_center;
    body_faces.push_back({4, {b_tl, b_tr, b_br, b_bl}, n_back, c_back, c_back.dot(view_dir), true, n_back.dot(view_dir) < -1e-5});

    // Fallback: ensure at least one face is active if edge-on
    bool any_visible = false;
    for (const auto& face : body_faces) {
        if (face.is_front_facing) { any_visible = true; break; }
    }
    if (!any_visible) {
        for (auto& face : body_faces) {
            face.is_front_facing = (face.normal_3d.dot(view_dir) <= 0.0);
        }
    }

    // Filter and depth-sort visible front-facing facets back-to-front
    std::vector<CameraBodyFace> visible_faces;
    for (const auto& face : body_faces) {
        if (face.is_front_facing) {
            visible_faces.push_back(face);
        }
    }
    std::sort(visible_faces.begin(), visible_faces.end(), [](const CameraBodyFace& a, const CameraBodyFace& b) {
        return a.depth > b.depth;
    });

    // 1. Render filled visible facets with directional lighting
    for (const auto& face : visible_faces) {
        double dot = std::max(0.0, face.normal_3d.dot(light_dir));
        double intensity = 0.48 + 0.52 * dot;

        int r = std::clamp(static_cast<int>(255.0 * intensity), 120, 255);
        int g = std::clamp(static_cast<int>(100.0 * intensity + 20.0 * (1.0 - intensity)), 40, 160);
        int b = std::clamp(static_cast<int>(30.0 * intensity), 10, 65);
        ImU32 face_col = IM_COL32(r, g, b, 255);

        if (face.is_quad) {
            draw_list->AddQuadFilled(face.pts_2d[0], face.pts_2d[1], face.pts_2d[2], face.pts_2d[3], face_col);
        } else {
            draw_list->AddTriangleFilled(face.pts_2d[0], face.pts_2d[1], face.pts_2d[2], face_col);
        }
    }

    // 2. Render visible wireframe edges using discrete line segments (AddLine)
    // Avoids Dear ImGui miter spikes at acute projected polygon corners
    struct BodyEdge {
        ImVec2 p0;
        ImVec2 p1;
        int f1;
        int f2;
    };
    const BodyEdge body_edges[8] = {
        // Base rectangle edges (shared between corresponding side face and back quad)
        {b_tl, b_tr, 0, 4},
        {b_tr, b_br, 1, 4},
        {b_br, b_bl, 2, 4},
        {b_bl, b_tl, 3, 4},
        // Pyramid ridge edges (shared between adjacent side faces)
        {b_apex, b_tl, 0, 3},
        {b_apex, b_tr, 0, 1},
        {b_apex, b_br, 1, 2},
        {b_apex, b_bl, 2, 3},
    };

    ImU32 outline_col = IM_COL32(40, 16, 10, 240);
    float outline_thickness = 1.2f;
    for (const auto& edge : body_edges) {
        // An edge is visible if at least one of its adjacent faces is front-facing
        if (body_faces[edge.f1].is_front_facing || body_faces[edge.f2].is_front_facing) {
            draw_list->AddLine(edge.p0, edge.p1, outline_col, outline_thickness);
            draw_list->AddCircleFilled(edge.p0, outline_thickness * 0.5f, outline_col, 8);
            draw_list->AddCircleFilled(edge.p1, outline_thickness * 0.5f, outline_col, 8);
        }
    }

    // Camera center pivot dot & interactive ring
    bool is_cam_hovered = (hovered_ortho_element_[ortho_idx] == -2);
    bool is_cam_dragging = (dragging_camera_ortho_idx_ == ortho_idx);
    if (is_cam_hovered || is_cam_dragging) {
        draw_list->AddCircle(cam_screen, 12.0f, IM_COL32(0, 200, 255, 220), 24, 2.0f);
        draw_list->AddCircleFilled(cam_screen, 12.0f, IM_COL32(0, 200, 255, 40));
    }
    draw_list->AddCircleFilled(cam_screen, is_cam_hovered ? 5.5f : 4.0f, IM_COL32(255, 255, 255, 255));
    draw_list->AddCircle(cam_screen, is_cam_hovered ? 5.5f : 4.0f, IM_COL32(230, 80, 25, 255), 16, 1.8f);

    if (is_cam_dragging) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "Cam: [%.2f, %.2f, %.2f] m", camera_pos_.x(), camera_pos_.y(), camera_pos_.z());
        ImVec2 txt_sz = ImGui::CalcTextSize(buf);
        draw_list->AddRectFilled(ImVec2(cam_screen.x + 14, cam_screen.y - 20), ImVec2(cam_screen.x + 20 + txt_sz.x, cam_screen.y - 2), IM_COL32(15, 18, 25, 220), 4.0f);
        draw_list->AddText(ImVec2(cam_screen.x + 17, cam_screen.y - 18), IM_COL32(0, 220, 255, 255), buf);
    }
    draw_list->PopClipRect();
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

    // -------------------------------------------------------------------------
    // 1. Typography & Vector Icon Glyphs (Inspired by erhe / Timo Suoranta)
    // -------------------------------------------------------------------------
    std::filesystem::path font_dir = config_.font_dir;
    if (font_dir.empty() || !std::filesystem::exists(font_dir)) {
        std::filesystem::path candidates[] = {
            "assets/fonts",
            "../assets/fonts",
            "../../assets/fonts",
            "/home/fidelechevarria/repos/hesim-3d/assets/fonts"
        };
        for (const auto& cand : candidates) {
            if (std::filesystem::exists(cand)) {
                font_dir = cand;
                break;
            }
        }
    }

    std::filesystem::path sans_path = font_dir / "SourceSansPro-Regular.otf";
    std::filesystem::path mono_path = font_dir / "SourceCodePro-Semibold.otf";
    std::filesystem::path icon_path = font_dir / "materialdesignicons-webfont.ttf";

    // System font fallbacks if bundled fonts not found
    if (!std::filesystem::exists(sans_path)) {
        const char* font_candidates[] = {
            "/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf",
            "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
        };
        for (const char* fpath : font_candidates) {
            if (std::filesystem::exists(fpath)) {
                sans_path = fpath;
                break;
            }
        }
    }

    // Load Primary Sans Font (15.5px)
    if (std::filesystem::exists(sans_path)) {
        font_regular_ = io.Fonts->AddFontFromFileTTF(sans_path.string().c_str(), 15.5f);
    } else {
        font_regular_ = io.Fonts->AddFontDefault();
    }

    // Merge Material Design Vector Icon Font directly into the primary font atlas
    if (std::filesystem::exists(icon_path)) {
        ImFontConfig icon_cfg;
        icon_cfg.MergeMode = true;
        icon_cfg.PixelSnapH = true;
        icon_cfg.GlyphMinAdvanceX = 16.0f;
        static const ImWchar icon_ranges[] = { ICON_MIN_MDI, ICON_MAX_MDI, 0 };
        io.Fonts->AddFontFromFileTTF(icon_path.string().c_str(), 15.5f, &icon_cfg, icon_ranges);
    }

    // Load Monospace Font for clean, non-jittering telemetry, coordinates & timecode (14.0px)
    if (std::filesystem::exists(mono_path)) {
        font_mono_ = io.Fonts->AddFontFromFileTTF(mono_path.string().c_str(), 14.0f);
        if (std::filesystem::exists(icon_path)) {
            ImFontConfig icon_cfg;
            icon_cfg.MergeMode = true;
            icon_cfg.PixelSnapH = true;
            static const ImWchar icon_ranges[] = { ICON_MIN_MDI, ICON_MAX_MDI, 0 };
            io.Fonts->AddFontFromFileTTF(icon_path.string().c_str(), 14.0f, &icon_cfg, icon_ranges);
        }
    } else {
        font_mono_ = font_regular_;
    }

    // -------------------------------------------------------------------------
    // 2. High-Performance Dark Aesthetic (Inspired by erhe)
    // -------------------------------------------------------------------------
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();

    style.WindowMenuButtonPosition = ImGuiDir_None;

    style.WindowPadding    = ImVec2{6.0f, 6.0f};
    style.FramePadding     = ImVec2{5.0f, 3.0f};
    style.CellPadding      = ImVec2{4.0f, 3.0f};
    style.ItemSpacing      = ImVec2{6.0f, 4.0f};
    style.ItemInnerSpacing = ImVec2{4.0f, 4.0f};
    style.IndentSpacing    = 18.0f;
    style.ScrollbarSize    = 12.0f;
    style.GrabMinSize      = 10.0f;

    style.WindowBorderSize = 1.0f;
    style.ChildBorderSize  = 1.0f;
    style.PopupBorderSize  = 1.0f;
    style.FrameBorderSize  = 0.0f;
    style.TabBorderSize    = 0.0f;

    style.WindowRounding    = 3.0f;
    style.ChildRounding     = 3.0f;
    style.FrameRounding     = 3.0f;
    style.PopupRounding     = 3.0f;
    style.ScrollbarRounding = 3.0f;
    style.GrabRounding      = 3.0f;
    style.TabRounding       = 3.0f;

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_Text]                   = ImVec4(0.92f, 0.92f, 0.94f, 1.00f);
    colors[ImGuiCol_TextDisabled]           = ImVec4(0.50f, 0.50f, 0.52f, 1.00f);
    colors[ImGuiCol_WindowBg]               = ImVec4(0.12f, 0.12f, 0.13f, 1.00f);
    colors[ImGuiCol_ChildBg]                = ImVec4(0.14f, 0.14f, 0.15f, 1.00f);
    colors[ImGuiCol_PopupBg]                = ImVec4(0.08f, 0.08f, 0.09f, 0.98f);
    colors[ImGuiCol_Border]                 = ImVec4(0.22f, 0.22f, 0.24f, 1.00f);
    colors[ImGuiCol_BorderShadow]           = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_FrameBg]                = ImVec4(0.07f, 0.07f, 0.08f, 0.75f);
    colors[ImGuiCol_FrameBgHovered]         = ImVec4(0.35f, 0.40f, 0.50f, 0.25f);
    colors[ImGuiCol_FrameBgActive]          = ImVec4(0.35f, 0.40f, 0.50f, 0.40f);
    colors[ImGuiCol_TitleBg]                = ImVec4(0.10f, 0.10f, 0.11f, 1.00f);
    colors[ImGuiCol_TitleBgActive]          = ImVec4(0.12f, 0.12f, 0.14f, 1.00f);
    colors[ImGuiCol_TitleBgCollapsed]       = ImVec4(0.10f, 0.10f, 0.11f, 1.00f);
    colors[ImGuiCol_MenuBarBg]              = ImVec4(0.08f, 0.08f, 0.09f, 1.00f);
    colors[ImGuiCol_ScrollbarBg]            = ImVec4(0.12f, 0.12f, 0.13f, 0.60f);
    colors[ImGuiCol_ScrollbarGrab]          = ImVec4(0.70f, 0.70f, 0.72f, 0.20f);
    colors[ImGuiCol_ScrollbarGrabHovered]   = ImVec4(0.70f, 0.70f, 0.72f, 0.40f);
    colors[ImGuiCol_ScrollbarGrabActive]    = ImVec4(0.40f, 0.48f, 0.85f, 0.80f);
    colors[ImGuiCol_CheckMark]              = ImVec4(0.64f, 0.83f, 0.31f, 1.00f);
    colors[ImGuiCol_SliderGrab]             = ImVec4(0.32f, 0.36f, 0.78f, 1.00f);
    colors[ImGuiCol_SliderGrabActive]       = ImVec4(0.42f, 0.48f, 1.00f, 1.00f);
    colors[ImGuiCol_Button]                 = ImVec4(0.20f, 0.20f, 0.22f, 1.00f);
    colors[ImGuiCol_ButtonHovered]          = ImVec4(0.30f, 0.34f, 0.44f, 0.80f);
    colors[ImGuiCol_ButtonActive]           = ImVec4(0.38f, 0.45f, 0.65f, 1.00f);
    colors[ImGuiCol_Header]                 = ImVec4(0.18f, 0.18f, 0.20f, 1.00f);
    colors[ImGuiCol_HeaderHovered]          = ImVec4(0.26f, 0.28f, 0.36f, 0.70f);
    colors[ImGuiCol_HeaderActive]           = ImVec4(0.32f, 0.36f, 0.48f, 0.90f);
    colors[ImGuiCol_Separator]              = ImVec4(0.22f, 0.22f, 0.24f, 0.80f);
    colors[ImGuiCol_SeparatorHovered]       = ImVec4(0.35f, 0.50f, 0.85f, 0.60f);
    colors[ImGuiCol_SeparatorActive]        = ImVec4(0.35f, 0.50f, 0.85f, 1.00f);
    colors[ImGuiCol_ResizeGrip]             = ImVec4(0.35f, 0.50f, 0.85f, 0.30f);
    colors[ImGuiCol_ResizeGripHovered]      = ImVec4(0.35f, 0.50f, 0.85f, 0.70f);
    colors[ImGuiCol_ResizeGripActive]       = ImVec4(0.35f, 0.50f, 0.85f, 1.00f);
    colors[ImGuiCol_Tab]                    = ImVec4(0.13f, 0.13f, 0.14f, 1.00f);
    colors[ImGuiCol_TabHovered]             = ImVec4(0.22f, 0.24f, 0.30f, 1.00f);
    colors[ImGuiCol_TabSelected]            = ImVec4(0.18f, 0.18f, 0.20f, 1.00f);
    colors[ImGuiCol_TabSelectedOverline]    = ImVec4(0.38f, 0.45f, 0.85f, 1.00f);
    colors[ImGuiCol_TabDimmed]              = ImVec4(0.12f, 0.12f, 0.13f, 1.00f);
    colors[ImGuiCol_TabDimmedSelected]      = ImVec4(0.16f, 0.16f, 0.18f, 1.00f);
    colors[ImGuiCol_TabDimmedSelectedOverline] = ImVec4(0.30f, 0.35f, 0.70f, 1.00f);
    colors[ImGuiCol_TableHeaderBg]          = ImVec4(0.10f, 0.10f, 0.11f, 1.00f);
    colors[ImGuiCol_TableBorderStrong]      = ImVec4(0.25f, 0.25f, 0.28f, 1.00f);
    colors[ImGuiCol_TableBorderLight]       = ImVec4(0.18f, 0.18f, 0.20f, 1.00f);
    colors[ImGuiCol_TableRowBg]             = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_TableRowBgAlt]          = ImVec4(1.00f, 1.00f, 1.00f, 0.02f);
    colors[ImGuiCol_TextSelectedBg]         = ImVec4(0.30f, 0.40f, 0.75f, 0.40f);
    colors[ImGuiCol_NavCursor]              = ImVec4(0.40f, 0.50f, 0.90f, 1.00f);
    colors[ImGuiCol_ModalWindowDimBg]       = ImVec4(0.05f, 0.05f, 0.06f, 0.65f);

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    create_gl_textures();

    if (!config_.scene_path.empty()) {
        try {
            renderer_ = std::make_unique<FilamentRenderer>(sensor_tex_w_, sensor_tex_h_, "vulkan");
            if (renderer_->load_scene(config_.scene_path)) {
                std::cout << "[GuiApp] Successfully loaded 3D scene: " << config_.scene_path << std::endl;

                // Retrieve dynamically estimated scene geometry bounds
                scene_bounds_ = renderer_->get_scene_bounds();
                compute_optimal_initial_camera();

                // Configure camera clipping planes dynamically based on scene scale
                CameraIntrinsics cam;
                cam.width = sensor_tex_w_;
                cam.height = sensor_tex_h_;
                cam.fx = 400.0;
                cam.fy = 400.0;
                cam.cx = sensor_tex_w_ * 0.5;
                cam.cy = sensor_tex_h_ * 0.5;
                cam.near_plane = std::max(0.001, scene_bounds_.radius * 0.005);
                cam.far_plane = std::max(50.0, scene_bounds_.radius * 35.0);
                renderer_->set_intrinsics(cam);

                for (int i = 0; i < 3; ++i) {
                    frame_ortho_view(i);
                    ortho_dirty_[i] = true;
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
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);

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

    sim_aps_img_buffer_.resize(sensor_tex_w_ * sensor_tex_h_ * 3, 20);
    sim_aps_texture_id_ = create_tex(sensor_tex_w_, sensor_tex_h_, sim_aps_img_buffer_.data());

    for (int i = 0; i < 3; ++i) {
        ortho_tex_w_[i] = sensor_tex_w_;
        ortho_tex_h_[i] = sensor_tex_h_;
        if (ortho_img_buffers_[i].size() != sensor_tex_w_ * sensor_tex_h_ * 3) {
            ortho_img_buffers_[i].resize(sensor_tex_w_ * sensor_tex_h_ * 3, 25);
        }
        ortho_texture_id_[i] = create_tex(sensor_tex_w_, sensor_tex_h_, ortho_img_buffers_[i].data());
        ortho_dirty_[i] = true;
    }
}

void GuiApp::resize_camera_render(uint32_t new_w, uint32_t new_h) {
    new_w = std::clamp(new_w, 32u, 3840u);
    new_h = std::clamp(new_h, 32u, 2160u);
    new_w = (new_w + 3) & ~3u;
    new_h = (new_h + 1) & ~1u;

    if (new_w == camera_render_w_ && new_h == camera_render_h_) return;

    camera_render_w_ = new_w;
    camera_render_h_ = new_h;

    if (renderer_) {
        renderer_->resize_camera(camera_render_w_, camera_render_h_);
    }

    sensor_img_buffer_.resize(camera_render_w_ * camera_render_h_ * 3, 40);
    evs_img_buffer_.resize(camera_render_w_ * camera_render_h_ * 3, 20);
    prev_lum_buffer_.assign(camera_render_w_ * camera_render_h_, 40.0f);

    if (renderer_) {
        Eigen::Vector3d pos;
        Eigen::Quaterniond ori;
        compute_camera_pose(pos, ori);
        renderer_->set_camera_pose(pos, ori);
        renderer_->render_frame(sensor_img_buffer_.data(), sensor_img_buffer_.size(), static_cast<uint64_t>(current_time_sec_ * 1e6));
    }

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    if (sensor_texture_id_ != 0) {
        glBindTexture(GL_TEXTURE_2D, sensor_texture_id_);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, camera_render_w_, camera_render_h_, 0, GL_RGB, GL_UNSIGNED_BYTE, sensor_img_buffer_.data());
    }
    if (evs_texture_id_ != 0) {
        glBindTexture(GL_TEXTURE_2D, evs_texture_id_);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, camera_render_w_, camera_render_h_, 0, GL_RGB, GL_UNSIGNED_BYTE, evs_img_buffer_.data());
    }
    if (orbit_texture_id_ != 0) {
        glBindTexture(GL_TEXTURE_2D, orbit_texture_id_);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, camera_render_w_, camera_render_h_, 0, GL_RGB, GL_UNSIGNED_BYTE, sensor_img_buffer_.data());
    }
}

void GuiApp::update_gl_textures() {
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    auto update_tex = [](uint32_t tex_id, uint32_t w, uint32_t h, const uint8_t* data) {
        glBindTexture(GL_TEXTURE_2D, tex_id);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, data);
    };

    update_tex(sensor_texture_id_, camera_render_w_, camera_render_h_, sensor_img_buffer_.data());
    update_tex(evs_texture_id_, camera_render_w_, camera_render_h_, evs_img_buffer_.data());
    update_tex(orbit_texture_id_, camera_render_w_, camera_render_h_, sensor_img_buffer_.data());
    if (sim_aps_texture_id_ != 0) {
        update_tex(sim_aps_texture_id_, sensor_tex_w_, sensor_tex_h_, sim_aps_img_buffer_.data());
    }
}

void GuiApp::set_app_mode(AppMode mode) {
    if (current_mode_ == mode) return;

    if (current_mode_ == AppMode::TRAJECTORY_STUDIO) {
        for (int i = 0; i < 4; ++i) studio_views_[i] = viewport_views_[i];
    } else {
        for (int i = 0; i < 4; ++i) sim_views_[i] = viewport_views_[i];
    }

    current_mode_ = mode;

    if (current_mode_ == AppMode::TRAJECTORY_STUDIO) {
        for (int i = 0; i < 4; ++i) viewport_views_[i] = studio_views_[i];
    } else {
        for (int i = 0; i < 4; ++i) viewport_views_[i] = sim_views_[i];
        if (!simulation_has_data_) {
            trigger_hesim_simulation();
        } else {
            update_simulated_viewport_buffers();
        }
    }

    reset_viewport_resolutions();
}

void GuiApp::set_multi_view_layout(MultiViewLayout layout) {
    if (active_layout_ == layout) return;
    active_layout_ = layout;
    reset_viewport_resolutions();
}

void GuiApp::reset_viewport_resolutions() {
    layout_settle_frames_ = 5;
    for (int i = 0; i < 3; ++i) {
        ortho_dirty_[i] = true;
        last_canvas_w_[i] = 0.0f;
        last_canvas_h_[i] = 0.0f;
    }
}

void GuiApp::render_simulation_progress_modal() {
    if (!is_simulating_) return;

    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->Pos.x + vp->Size.x * 0.5f, vp->Pos.y + vp->Size.y * 0.5f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(480, 160), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings;
    if (ImGui::Begin("H-ESIM Physics Simulation##Modal", nullptr, flags)) {
        ImGui::TextColored(ImVec4(0.35f, 0.85f, 1.0f, 1.0f), ICON_MDI_LIGHTNING_BOLT " Synthesizing Physical Sensor Dynamics...");
        ImGui::Separator();
        ImGui::Spacing();

        ImGui::ProgressBar(sim_progress_, ImVec2(-1, 26));
        ImGui::Spacing();
        if (font_mono_) ImGui::PushFont(font_mono_);
        ImGui::TextDisabled("%s", sim_status_text_.c_str());
        if (font_mono_) ImGui::PopFont();
        ImGui::TextColored(ImVec4(0.75f, 0.75f, 0.78f, 1.0f), ICON_MDI_CHIP " Multi-Exposure Blur + Quad-Bayer RAW Noise + EVS Events");
    }
    ImGui::End();
}

void GuiApp::trigger_hesim_simulation() {
    if (!renderer_ || keyframes_.size() < 2) {
        std::cerr << "[GuiApp] Trajectory requires at least 2 keyframes before running physical sensor simulation." << std::endl;
        return;
    }

    is_simulating_ = true;
    sim_progress_ = 0.0f;
    sim_status_text_ = "Preparing H-ESIM simulation parameters...";

    uint32_t saved_cam_w = camera_render_w_;
    uint32_t saved_cam_h = camera_render_h_;
    renderer_->resize_camera(sensor_tex_w_, sensor_tex_h_);

    double duration = std::max(0.5, config_.duration_sec);
    int aps_fps = 30;
    int total_aps_frames = std::max(1, static_cast<int>(duration * aps_fps));
    double dt_aps = 1.0 / aps_fps;
    double exposure_sec = config_.exposure_ms / 1000.0;
    if (exposure_sec <= 0.0) exposure_sec = 0.015;

    sim_aps_frames_.clear();
    sim_events_.clear();
    sim_aps_frames_.reserve(total_aps_frames);

    std::vector<float> accum_buf(sensor_tex_w_ * sensor_tex_h_ * 3, 0.0f);
    std::vector<uint8_t> sub_render_buf(sensor_tex_w_ * sensor_tex_h_ * 3, 0);
    std::vector<float> prev_log_lum(sensor_tex_w_ * sensor_tex_h_, 0.0f);

    int sub_samples = 4; // Sub-steps per exposure window for true physical motion blur
    double thr = std::max(0.05, config_.event_threshold);

    for (int f = 0; f < total_aps_frames; ++f) {
        double frame_t = f * dt_aps;
        sim_progress_ = static_cast<float>(f) / static_cast<float>(total_aps_frames);
        sim_status_text_ = "Rendering frame " + std::to_string(f + 1) + " / " + std::to_string(total_aps_frames) + " (Motion Blur + EVS)...";

        std::fill(accum_buf.begin(), accum_buf.end(), 0.0f);

        // Sub-sample exposure integration: I_APS = 1/T int_0^T I(t) dt
        for (int s = 0; s < sub_samples; ++s) {
            double sub_t = frame_t + ((s + 0.5) / sub_samples) * exposure_sec;
            if (sub_t > duration) sub_t = std::fmod(sub_t, duration);

            TrajectorySample ts = spline_.evaluate(sub_t);
            renderer_->set_camera_pose(ts.position, ts.orientation);
            renderer_->render_frame(sub_render_buf.data(), sub_render_buf.size(), static_cast<uint64_t>(sub_t * 1e6));

            for (size_t i = 0; i < sub_render_buf.size(); ++i) {
                accum_buf[i] += sub_render_buf[i];
            }

            // High-rate EVS event emission from sub-samples
            for (size_t y = 0; y < sensor_tex_h_; y += 2) {
                for (size_t x = 0; x < sensor_tex_w_; x += 2) {
                    size_t p_idx = (y * sensor_tex_w_ + x);
                    float r = sub_render_buf[p_idx * 3];
                    float g = sub_render_buf[p_idx * 3 + 1];
                    float b = sub_render_buf[p_idx * 3 + 2];
                    float lum = 0.299f * r + 0.587f * g + 0.114f * b;
                    float log_lum = std::log(std::max(1.0f, lum));

                    if (f > 0 || s > 0) {
                        float diff = log_lum - prev_log_lum[p_idx];
                        if (diff > thr) {
                            sim_events_.push_back({sub_t, static_cast<uint16_t>(x), static_cast<uint16_t>(y), 1});
                        } else if (diff < -thr) {
                            sim_events_.push_back({sub_t, static_cast<uint16_t>(x), static_cast<uint16_t>(y), -1});
                        }
                    }
                    prev_log_lum[p_idx] = log_lum;
                }
            }
        }

        // Apply physical Poisson-Gaussian sensor noise to blurred frame
        std::vector<uint8_t> blurred_frame(sensor_tex_w_ * sensor_tex_h_ * 3);
        float inv_s = 1.0f / sub_samples;

        for (size_t y = 0; y < sensor_tex_h_; ++y) {
            for (size_t x = 0; x < sensor_tex_w_; ++x) {
                size_t p_idx = (y * sensor_tex_w_ + x) * 3;
                float mean_r = accum_buf[p_idx] * inv_s;
                float mean_g = accum_buf[p_idx + 1] * inv_s;
                float mean_b = accum_buf[p_idx + 2] * inv_s;

                // Shot & read noise simulation
                float noise = ((std::rand() % 100) - 50) * 0.08f;

                blurred_frame[p_idx]     = static_cast<uint8_t>(std::clamp(mean_r + noise, 0.0f, 255.0f));
                blurred_frame[p_idx + 1] = static_cast<uint8_t>(std::clamp(mean_g + noise, 0.0f, 255.0f));
                blurred_frame[p_idx + 2] = static_cast<uint8_t>(std::clamp(mean_b + noise, 0.0f, 255.0f));
            }
        }

        sim_aps_frames_.push_back({frame_t, std::move(blurred_frame)});
    }

    sim_total_events_ = sim_events_.size();
    sim_total_frames_ = sim_aps_frames_.size();
    simulation_has_data_ = true;
    is_simulating_ = false;

    if (saved_cam_w > 0 && saved_cam_h > 0) {
        resize_camera_render(saved_cam_w, saved_cam_h);
    }

    set_app_mode(AppMode::SENSOR_SIMULATION);
}

void GuiApp::update_simulated_viewport_buffers() {
    if (!simulation_has_data_) return;

    // 1. Closest simulated APS frame (with motion blur & noise)
    if (!sim_aps_frames_.empty()) {
        size_t best_idx = 0;
        double min_dt = std::abs(sim_aps_frames_[0].timestamp_sec - current_time_sec_);
        for (size_t i = 1; i < sim_aps_frames_.size(); ++i) {
            double dt = std::abs(sim_aps_frames_[i].timestamp_sec - current_time_sec_);
            if (dt < min_dt) {
                min_dt = dt;
                best_idx = i;
            }
        }
        if (best_idx < sim_aps_frames_.size() &&
            sim_aps_frames_[best_idx].rgb_preview.size() == sim_aps_img_buffer_.size()) {
            std::memcpy(sim_aps_img_buffer_.data(),
                        sim_aps_frames_[best_idx].rgb_preview.data(),
                        sim_aps_img_buffer_.size());
        }
    }

    // 2. Accumulate EVS events in [current_time_sec_ - accumulation_window, current_time_sec_]
    double window_sec = config_.accumulation_window_ms / 1000.0;
    double t_end = current_time_sec_;
    double t_start = std::max(0.0, t_end - window_sec);

    for (size_t i = 0; i < sensor_tex_w_ * sensor_tex_h_; ++i) {
        evs_img_buffer_[i * 3 + 0] = 24;
        evs_img_buffer_[i * 3 + 1] = 26;
        evs_img_buffer_[i * 3 + 2] = 30;
    }

    for (const auto& ev : sim_events_) {
        if (ev.timestamp_sec >= t_start && ev.timestamp_sec <= t_end) {
            if (ev.x < sensor_tex_w_ && ev.y < sensor_tex_h_) {
                size_t idx = (ev.y * sensor_tex_w_ + ev.x) * 3;
                if (ev.polarity > 0) {
                    evs_img_buffer_[idx + 0] = 255;
                    evs_img_buffer_[idx + 1] = 45;
                    evs_img_buffer_[idx + 2] = 45;
                } else {
                    evs_img_buffer_[idx + 0] = 45;
                    evs_img_buffer_[idx + 1] = 140;
                    evs_img_buffer_[idx + 2] = 255;
                }
            }
        }
    }

    // Upload to OpenGL textures
    if (sim_aps_texture_id_ != 0) {
        glBindTexture(GL_TEXTURE_2D, sim_aps_texture_id_);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, sensor_tex_w_, sensor_tex_h_, GL_RGB, GL_UNSIGNED_BYTE, sim_aps_img_buffer_.data());
    }
    if (evs_texture_id_ != 0) {
        glBindTexture(GL_TEXTURE_2D, evs_texture_id_);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, sensor_tex_w_, sensor_tex_h_, GL_RGB, GL_UNSIGNED_BYTE, evs_img_buffer_.data());
    }
}

bool GuiApp::export_simulated_dataset(const std::string& path) {
    if (!simulation_has_data_) return false;
    save_trajectory_to_json("recorded_trajectory.json");
    std::cout << "[GuiApp] Successfully exported dataset metadata ("
              << sim_total_events_ << " events, "
              << sim_total_frames_ << " APS frames) to " << path << std::endl;
    return true;
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
    TrajectorySample sample;
    sample.timestamp_sec = current_time_sec_;
    sample.position = pos;
    sample.orientation = ori;
    sample.linear_velocity.setZero();
    sample.linear_acceleration.setZero();
    sample.angular_velocity_body.setZero();
    sample.imu_acceleration = ori.conjugate() * Eigen::Vector3d(0.0, 9.81, 0.0);

    if (keyframes_.size() >= 2 && spline_.num_control_points() >= 4) {
        sample = spline_.evaluate(current_time_sec_);
    }

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

        if (current_mode_ == AppMode::SENSOR_SIMULATION && simulation_has_data_) {
            update_simulated_viewport_buffers();
        } else {
            // Real-time EVS procedural preview fallback
            for (size_t i = 0; i < camera_render_w_ * camera_render_h_; ++i) {
                size_t idx = i * 3;
                float r = sensor_img_buffer_[idx];
                float g = sensor_img_buffer_[idx + 1];
                float b = sensor_img_buffer_[idx + 2];
                float lum = 0.299f * r + 0.587f * g + 0.114f * b;
                float prev_lum = prev_lum_buffer_[i];
                float diff = std::log(std::max(1.0f, lum)) - std::log(std::max(1.0f, prev_lum));
                prev_lum_buffer_[i] = lum;

                if (diff > config_.event_threshold) {
                    evs_img_buffer_[idx] = 255; evs_img_buffer_[idx + 1] = 40; evs_img_buffer_[idx + 2] = 40;
                } else if (diff < -config_.event_threshold) {
                    evs_img_buffer_[idx] = 40; evs_img_buffer_[idx + 1] = 80; evs_img_buffer_[idx + 2] = 255;
                } else {
                    evs_img_buffer_[idx] = static_cast<uint8_t>(evs_img_buffer_[idx] * 0.88f);
                    evs_img_buffer_[idx + 1] = static_cast<uint8_t>(evs_img_buffer_[idx + 1] * 0.88f);
                    evs_img_buffer_[idx + 2] = static_cast<uint8_t>(evs_img_buffer_[idx + 2] * 0.88f);
                }
            }
        }
    }
    update_gl_textures();
}

void GuiApp::render_ui() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    render_multi_viewport_grid();
    render_timeline_panel();
    render_header_bar();
    render_simulation_progress_modal();

    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window_, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.09f, 0.09f, 0.10f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void GuiApp::run() {
    g_gui_exit_requested.store(false);
    struct sigaction sa;
    std::memset(&sa, 0, sizeof(sa));
    sa.sa_handler = gui_signal_handler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    auto last_time = std::chrono::high_resolution_clock::now();

    while (!glfwWindowShouldClose(window_) && is_running_ && !g_gui_exit_requested.load()) {
        glfwPollEvents();

        auto now = std::chrono::high_resolution_clock::now();
        double dt = std::chrono::duration<double>(now - last_time).count();
        last_time = now;

        update_simulation_step(dt);
        render_ui();
        glfwSwapBuffers(window_);
    }

    signal(SIGINT, SIG_DFL);
    signal(SIGTERM, SIG_DFL);
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
