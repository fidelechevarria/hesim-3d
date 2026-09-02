#pragma once

#include <string>
#include <vector>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <cstdint>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "spline_se3.h"
#include "filament_renderer.h"

struct GLFWwindow;
struct ImDrawList;
struct ImVec2;

namespace hesim3d {

enum class MultiViewLayout {
    VIEW_1_SINGLE = 0,
    VIEW_2_SPLIT,
    VIEW_3_SPLIT,
    VIEW_4_GRID
};

enum class ViewportContent {
    CAMERA_SENSOR = 0,
    EVS_ACCUMULATION,
    TOP_ORTHO,
    FRONT_ORTHO,
    SIDE_ORTHO,
    WORLD_3D_ORBIT,
    IMU_TELEMETRY
};

enum class TrajectoryInterpolation {
    SE3_CUMULATIVE_SPLINE = 0,
    LINEAR_SLERP
};

struct StudioKeyframe {
    double time_sec{0.0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d rotation_euler_deg{Eigen::Vector3d::Zero()}; // [Yaw/Pan, Pitch/Tilt, Roll]
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

struct GuiConfig {
    std::string scene_path{""};
    std::string sensor_name{"alpsentek_eiger"};
    std::string trajectory_path{""};
    uint32_t window_width{1600};
    uint32_t window_height{1000};
    double duration_sec{5.0};
    double sim_fps{1000.0};
    double event_threshold{0.20};
    double exposure_ms{10.0};
    double accumulation_window_ms{20.0};
};

class GuiApp {
public:
    GuiApp(const GuiConfig& config);
    ~GuiApp();

    bool init();
    void run();
    void request_close();

    void set_spline(const SE3Spline& spline);

    // Keyframe Management (Google Earth Studio Style)
    void capture_keyframe_at_current_time();
    void delete_keyframe(int index);
    void jump_to_keyframe(int index);
    void update_keyframe_pose(int index);
    void rebuild_trajectory();
    bool save_trajectory_to_json(const std::string& path);
    bool load_trajectory_from_json(const std::string& path);

private:
    GuiConfig config_;
    GLFWwindow* window_{nullptr};

    std::atomic<bool> is_running_{false};
    std::atomic<bool> is_playing_{false};
    std::atomic<bool> is_recording_{false};
    std::string recording_output_path_{"recorded_dataset.h5"};
    std::string current_trajectory_file_{"custom_trajectory.json"};

    double current_time_sec_{0.0};
    double playback_speed_{1.0};
    int layout_init_frames_{5};
    int selected_keyframe_idx_{-1};

    // Interpolation & Spline
    TrajectoryInterpolation interp_mode_{TrajectoryInterpolation::SE3_CUMULATIVE_SPLINE};
    std::vector<StudioKeyframe> keyframes_;
    std::vector<Eigen::Vector3d> path_samples_;
    SE3Spline spline_;

    // Earth Studio Camera State (Free Camera & Navigation)
    bool is_free_camera_{true};
    Eigen::Vector3d camera_pos_{0.0, 180.0, -400.0};
    double camera_yaw_deg_{0.0};    // Heading / Pan (deg)
    double camera_pitch_deg_{0.0};  // Tilt / Incline (deg)
    double camera_roll_deg_{0.0};   // Roll / Bank (deg)
    Eigen::Vector3d camera_target_{0.0, 180.0, 0.0};
    double orbit_radius_{400.0};

    // Multi-View Layout Configuration
    MultiViewLayout active_layout_{MultiViewLayout::VIEW_4_GRID};
    ViewportContent viewport_views_[4]{
        ViewportContent::TOP_ORTHO,
        ViewportContent::CAMERA_SENSOR,
        ViewportContent::SIDE_ORTHO,
        ViewportContent::EVS_ACCUMULATION
    };

    // Resizable Splitters
    float split_ratio_x_{0.5f};
    float split_ratio_y_{0.5f};
    float timeline_height_px_{280.0f};

    // Orthographic View Transforms
    Eigen::Vector2d ortho_pan_[3]{ {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0} }; // Top, Front, Side
    double ortho_scale_[3]{ 0.25, 0.25, 0.25 }; // Scale factor for Sponza centimeter coords

    // Textures for viewports (OpenGL texture IDs)
    uint32_t sensor_texture_id_{0};
    uint32_t evs_texture_id_{0};
    uint32_t orbit_texture_id_{0};

    uint32_t sensor_tex_w_{640};
    uint32_t sensor_tex_h_{480};
    std::vector<uint8_t> sensor_img_buffer_;
    std::vector<uint8_t> evs_img_buffer_;
    std::vector<float> prev_lum_buffer_;

    std::unique_ptr<FilamentRenderer> renderer_{nullptr};

    // Telemetry history for ImPlot
    static constexpr size_t MAX_PLOT_HISTORY = 1000;
    std::vector<double> plot_time_;
    std::vector<double> plot_gyro_x_, plot_gyro_y_, plot_gyro_z_;
    std::vector<double> plot_acc_x_, plot_acc_y_, plot_acc_z_;

    // UI Render methods
    void render_ui();
    void render_header_bar();
    void render_multi_viewport_grid();
    void render_timeline_panel();
    void render_single_viewport(int quad_idx, const std::string& name, ViewportContent content);
    void render_imu_plots_content();

    // 2D Orthographic Draw Helpers
    void draw_ortho_map(int ortho_idx, ImDrawList* draw_list, float min_x, float min_y, float max_x, float max_y);
    void handle_camera_mouse_input(float min_x, float min_y, float max_x, float max_y);

    void update_simulation_step(double dt);
    void create_gl_textures();
    void update_gl_textures();
    void compute_camera_pose(Eigen::Vector3d& out_pos, Eigen::Quaterniond& out_ori);
};

int launch_gui(const GuiConfig& config);

} // namespace hesim3d
