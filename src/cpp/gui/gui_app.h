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

// Forward declare GLFWwindow at global scope
struct GLFWwindow;

namespace hesim3d {

struct GuiConfig {
    std::string scene_path{""};
    std::string sensor_name{"alpsentek_eiger"};
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

private:
    GuiConfig config_;
    GLFWwindow* window_{nullptr};

    std::atomic<bool> is_running_{false};
    std::atomic<bool> is_playing_{true};
    std::atomic<bool> is_recording_{false};
    std::string recording_output_path_{"recorded_dataset.h5"};

    double current_time_sec_{0.0};
    double playback_speed_{1.0};
    int layout_init_frames_{5};

    SE3Spline spline_;

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
    void render_viewports();
    void render_control_panels();
    void render_imu_plots();

    void update_simulation_step(double dt);
    void create_gl_textures();
    void update_gl_textures();
};

int launch_gui(const GuiConfig& config);

} // namespace hesim3d
