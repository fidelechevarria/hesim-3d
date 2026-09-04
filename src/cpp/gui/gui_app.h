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
struct ImFont;

namespace hesim3d {

enum class MultiViewLayout {
    VIEW_1_SINGLE = 0,
    VIEW_2_SPLIT,
    VIEW_3_SPLIT,
    VIEW_4_GRID
};

enum class AppMode {
    TRAJECTORY_STUDIO = 0,
    SENSOR_SIMULATION
};

enum class ViewportContent {
    CAMERA_CLEAN = 0,
    EVS_ACCUMULATION = 1,
    TOP_ORTHO = 2,
    FRONT_ORTHO = 3,
    SIDE_ORTHO = 4,
    WORLD_3D_ORBIT = 5,
    IMU_TELEMETRY = 6,
    SIMULATED_APS_SENSOR = 7,
    CAMERA_SENSOR = 0 // backward-compatible alias
};

struct SimulatedApsFrame {
    double timestamp_sec{0.0};
    std::vector<uint8_t> rgb_preview;
};

struct SimulatedEvent {
    double timestamp_sec{0.0};
    uint16_t x{0};
    uint16_t y{0};
    int8_t polarity{0};
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
    std::string font_dir{""};
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
    void jump_to_prev_keyframe();
    void jump_to_next_keyframe();
    void update_keyframe_pose(int index);
    void rebuild_trajectory();
    void frame_timeline_to_all_keyframes();
    void reset_timeline_view();
    bool save_trajectory_to_json(const std::string& path);
    bool load_trajectory_from_json(const std::string& path);

private:
    GuiConfig config_;
    GLFWwindow* window_{nullptr};
    ImFont* font_regular_{nullptr};
    ImFont* font_mono_{nullptr};
    ImFont* font_icons_{nullptr};

    std::atomic<bool> is_running_{false};
    std::atomic<bool> is_playing_{false};
    std::atomic<bool> is_recording_{false};
    std::string recording_output_path_{"recorded_dataset.h5"};
    std::string current_trajectory_file_{"custom_trajectory.json"};

    double current_time_sec_{0.0};
    double playback_speed_{1.0};
    int layout_settle_frames_{5};
    int selected_keyframe_idx_{-1};

    // Interpolation & Spline
    TrajectoryInterpolation interp_mode_{TrajectoryInterpolation::SE3_CUMULATIVE_SPLINE};
    std::vector<StudioKeyframe> keyframes_;
    std::vector<Eigen::Vector3d> path_samples_;
    SE3Spline spline_;

    // Earth Studio Camera State (Free Camera & Navigation)
    bool is_free_camera_{true};
    SceneBounds scene_bounds_;
    float nav_speed_factor_{1.0f};
    Eigen::Vector3d camera_pos_{0.0, 1.5, 3.0};
    double camera_yaw_deg_{25.0};   // Heading / Pan (deg)
    double camera_pitch_deg_{-22.0}; // Tilt / Incline (deg)
    double camera_roll_deg_{0.0};   // Roll / Bank (deg)
    Eigen::Vector3d camera_target_{0.0, 0.0, 0.0};
    double orbit_radius_{3.0};
    float smooth_orbit_dx_{0.0f};
    float smooth_orbit_dy_{0.0f};
    bool is_orbit_dragging_{false};

    // Multi-View & Workflow Mode Configuration
    AppMode current_mode_{AppMode::TRAJECTORY_STUDIO};
    std::atomic<bool> is_simulating_{false};
    std::atomic<float> sim_progress_{0.0f};
    std::string sim_status_text_{""};
    bool simulation_has_data_{false};
    size_t sim_total_events_{0};
    size_t sim_total_frames_{0};

    // Incremental simulation bake state
    bool trajectory_dirty_since_sim_{false};
    std::atomic<bool> sim_cancel_requested_{false};
    int sim_sampling_preset_{1}; // 0: 300 Hz (Fast), 1: 1000 Hz (Standard), 2: 3200 Hz (HKUST)
    int sim_current_frame_{0};
    int sim_total_aps_frames_{0};
    double sim_dt_aps_{1.0 / 30.0};
    double sim_exposure_sec_{0.015};
    int sim_sub_samples_{12};
    uint32_t sim_saved_cam_w_{0};
    uint32_t sim_saved_cam_h_{0};
    std::vector<float> sim_accum_buf_;
    std::vector<uint8_t> sim_sub_render_buf_;
    std::vector<float> sim_prev_log_lum_;

    std::vector<SimulatedApsFrame> sim_aps_frames_;
    std::vector<SimulatedEvent> sim_events_;
    uint32_t sim_aps_texture_id_{0};
    std::vector<uint8_t> sim_aps_img_buffer_;

    ViewportContent studio_views_[4]{
        ViewportContent::TOP_ORTHO,
        ViewportContent::CAMERA_CLEAN,
        ViewportContent::SIDE_ORTHO,
        ViewportContent::FRONT_ORTHO
    };

    ViewportContent sim_views_[4]{
        ViewportContent::CAMERA_CLEAN,
        ViewportContent::SIMULATED_APS_SENSOR,
        ViewportContent::EVS_ACCUMULATION,
        ViewportContent::IMU_TELEMETRY
    };

    MultiViewLayout active_layout_{MultiViewLayout::VIEW_4_GRID};
    ViewportContent viewport_views_[4]{
        ViewportContent::TOP_ORTHO,
        ViewportContent::CAMERA_CLEAN,
        ViewportContent::SIDE_ORTHO,
        ViewportContent::FRONT_ORTHO
    };

    // Resizable Splitters
    float split_ratio_x_{0.5f};
    float split_ratio_y_{0.5f};
    float timeline_height_px_{280.0f};

    // Timeline Viewport & Navigation (Zoom & Pan)
    double timeline_view_t_min_{0.0};
    double timeline_view_t_max_{5.0};
    bool timeline_view_initialized_{false};
    int dragging_timeline_kf_idx_{-1};

    float time_to_timeline_canvas_x(double t, float canvas_x0, float canvas_w) const;
    double timeline_canvas_x_to_time(float mouse_x, float canvas_x0, float canvas_w) const;

    // Orthographic View Transforms & Interaction
    Eigen::Vector2d ortho_pan_[3]{ {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0} }; // Top, Front, Side
    double ortho_scale_[3]{ 0.25, 0.25, 0.25 }; // Scale factor for Sponza centimeter coords
    uint32_t ortho_texture_id_[3]{0, 0, 0};
    uint32_t ortho_tex_w_[3]{640, 640, 640};
    uint32_t ortho_tex_h_[3]{480, 480, 480};
    std::vector<uint8_t> ortho_img_buffers_[3];
    bool ortho_dirty_[3]{true, true, true};
    int dragging_camera_ortho_idx_{-1}; // -1 = none, 0 = top, 1 = front, 2 = side
    int dragging_keyframe_ortho_idx_{-1}; // -1 = none, 0 = top, 1 = front, 2 = side
    int dragging_keyframe_idx_{-1};
    int hovered_ortho_element_[3]{-1, -1, -1}; // per ortho viewport: -2 = camera, >= 0 = keyframe index, -1 = none
    float ortho_dimming_{0.15f}; // Background brightness/dimming adjustment
    float last_canvas_w_[3]{0.0f, 0.0f, 0.0f};
    float last_canvas_h_[3]{0.0f, 0.0f, 0.0f};

    // Textures for viewports (OpenGL texture IDs)
    uint32_t sensor_texture_id_{0};
    uint32_t evs_texture_id_{0};
    uint32_t orbit_texture_id_{0};

    uint32_t sensor_tex_w_{640};
    uint32_t sensor_tex_h_{480};
    uint32_t camera_render_w_{640};
    uint32_t camera_render_h_{480};
    std::vector<uint8_t> sensor_img_buffer_;
    std::vector<uint8_t> evs_img_buffer_;
    std::vector<float> prev_lum_buffer_;

    std::unique_ptr<FilamentRenderer> renderer_{nullptr};

    // Telemetry curves for ImPlot
    std::vector<double> imu_curve_time_;
    std::vector<double> imu_curve_gyro_x_, imu_curve_gyro_y_, imu_curve_gyro_z_;
    std::vector<double> imu_curve_acc_x_, imu_curve_acc_y_, imu_curve_acc_z_;
    void compute_imu_profile_curves();

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
    void render_simulation_progress_modal();

    // 2D Orthographic Draw & Mouse Input Helpers
    void draw_ortho_map(int ortho_idx, ImDrawList* draw_list, float min_x, float min_y, float max_x, float max_y);
    void handle_ortho_mouse_input(int ortho_idx, float min_x, float min_y, float max_x, float max_y);
    void update_ortho_texture(int ortho_idx, float canvas_w, float canvas_h);
    void frame_ortho_view(int ortho_idx);
    void handle_camera_mouse_input(float min_x, float min_y, float max_x, float max_y);

    // Workflow & Simulation Methods
    void set_app_mode(AppMode mode);
    void set_multi_view_layout(MultiViewLayout layout);
    void reset_viewport_resolutions();
    void trigger_hesim_simulation();
    void start_simulation_bake();
    void step_simulation_bake();
    void cancel_simulation_bake();
    void finalize_simulation_bake();
    void update_simulated_viewport_buffers();
    bool export_simulated_dataset(const std::string& path);

    void update_simulation_step(double dt);
    void resize_camera_render(uint32_t new_w, uint32_t new_h);
    void create_gl_textures();
    void update_gl_textures();
    void compute_camera_pose(Eigen::Vector3d& out_pos, Eigen::Quaterniond& out_ori);
    void apply_spline_sample_at(double t);
    void compute_optimal_initial_camera();
};

int launch_gui(const GuiConfig& config);

} // namespace hesim3d
