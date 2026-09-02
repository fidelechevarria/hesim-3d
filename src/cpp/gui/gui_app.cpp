#include "gui_app.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

namespace hesim3d {

GuiApp::GuiApp(const GuiConfig& config) : config_(config) {
    sensor_img_buffer_.resize(sensor_tex_w_ * sensor_tex_h_ * 3, 40);
    evs_img_buffer_.resize(sensor_tex_w_ * sensor_tex_h_ * 3, 20);
    prev_lum_buffer_.resize(sensor_tex_w_ * sensor_tex_h_, 40.0f);

    // Initialize default trajectory if none provided
    spline_ = SE3Spline::create_eight_loop(
        Eigen::Vector3d(0.0, 0.0, 1.5),
        Eigen::Vector3d(2.5, 1.5, 0.4),
        config_.duration_sec,
        1.0
    );
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

bool GuiApp::init() {
    if (!glfwInit()) {
        std::cerr << "[GuiApp] Failed to initialize GLFW." << std::endl;
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window_ = glfwCreateWindow(config_.window_width, config_.window_height, "hesim-3d | Hybrid Event-Frame Simulator", nullptr, nullptr);
    if (!window_) {
        std::cerr << "[GuiApp] Failed to create GLFW window." << std::endl;
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1); // Enable vsync (60 FPS)

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // Setup ImGui style - sleek dark theme
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 6.0f;
    style.FrameRounding = 4.0f;
    style.GrabRounding = 4.0f;
    style.PopupRounding = 4.0f;
    style.ScrollbarRounding = 4.0f;

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

                    // Smooth trajectory along the central corridor of Sponza Atrium
                    spline_ = SE3Spline::create_eight_loop(
                        Eigen::Vector3d(0.0, 180.0, 0.0),      // Eye level in Sponza (cm)
                        Eigen::Vector3d(150.0, 30.0, 450.0),   // Atrium corridor movement
                        config_.duration_sec,
                        1.0
                    );
                }
            } else {
                std::cerr << "[GuiApp] Warning: Failed to parse 3D scene: " << config_.scene_path << ". Using procedural fallback." << std::endl;
                renderer_.reset();
            }
        } catch (const std::exception& e) {
            std::cerr << "[GuiApp] Warning: Filament renderer init failed (" << e.what() << "). Using procedural fallback." << std::endl;
            renderer_.reset();
        }
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
    current_time_sec_ += dt * playback_speed_;
    double max_t = spline_.max_time();
    if (max_t > 0.0 && current_time_sec_ > max_t) {
        current_time_sec_ = std::fmod(current_time_sec_, max_t);
    }

    // Evaluate trajectory and IMU
    TrajectorySample sample = spline_.evaluate(current_time_sec_);

    // Push to plot history
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
        renderer_->set_camera_pose(sample.position, sample.orientation);
        renderer_->render_frame(
            sensor_img_buffer_.data(),
            sensor_img_buffer_.size(),
            static_cast<uint64_t>(current_time_sec_ * 1e6)
        );

        // Physics-based log-intensity temporal difference for real-time EVS visualization
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
                // Fade out decay
                evs_img_buffer_[idx] = static_cast<uint8_t>(evs_img_buffer_[idx] * 0.88f);
                evs_img_buffer_[idx + 1] = static_cast<uint8_t>(evs_img_buffer_[idx + 1] * 0.88f);
                evs_img_buffer_[idx + 2] = static_cast<uint8_t>(evs_img_buffer_[idx + 2] * 0.88f);
            }
        }
    } else {
        // Fallback procedural checkerboard preview
        double phase = current_time_sec_ * 4.0;
        for (uint32_t y = 0; y < sensor_tex_h_; ++y) {
            for (uint32_t x = 0; x < sensor_tex_w_; ++x) {
                size_t idx = (y * sensor_tex_w_ + x) * 3;
                int cx = (x + static_cast<int>(sample.position.x() * 40)) / 32;
                int cy = (y + static_cast<int>(sample.position.y() * 40)) / 32;
                uint8_t c = ((cx + cy) % 2 == 0) ? 220 : 40;
                sensor_img_buffer_[idx] = c;
                sensor_img_buffer_[idx + 1] = c;
                sensor_img_buffer_[idx + 2] = c;

                if (x % 32 == 0 || y % 32 == 0) {
                    if (std::sin(phase + x * 0.1) > 0.3) {
                        evs_img_buffer_[idx] = 255; evs_img_buffer_[idx + 1] = 40; evs_img_buffer_[idx + 2] = 40;
                    } else {
                        evs_img_buffer_[idx] = 40; evs_img_buffer_[idx + 1] = 80; evs_img_buffer_[idx + 2] = 255;
                    }
                } else {
                    evs_img_buffer_[idx] = 20; evs_img_buffer_[idx + 1] = 20; evs_img_buffer_[idx + 2] = 20;
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

    render_control_panels();
    render_viewports();

    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window_, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.12f, 0.12f, 0.14f, 1.0f);
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

        if (is_playing_) {
            update_simulation_step(dt);
        }

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
