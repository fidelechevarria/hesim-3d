#include "gui_app.h"
#include <imgui.h>
#include <implot.h>
#include <iostream>

namespace hesim3d {

void GuiApp::render_viewports() {
    ImGuiIO& io = ImGui::GetIO();
    float margin = 12.0f;
    float left_w = 380.0f;
    float screen_w = io.DisplaySize.x > 100.0f ? io.DisplaySize.x : static_cast<float>(config_.window_width);
    float screen_h = io.DisplaySize.y > 100.0f ? io.DisplaySize.y : static_cast<float>(config_.window_height);

    float right_x = left_w + 2.0f * margin;
    float right_w = std::max(400.0f, screen_w - right_x - margin);
    float half_w = (right_w - margin) * 0.5f;

    float total_h = screen_h - 2.0f * margin;
    float top_h = total_h * 0.53f;
    float bot_h = total_h - top_h - margin;
    float bot_y = margin + top_h + margin;

    ImGuiCond cond = (layout_init_frames_ > 0) ? ImGuiCond_Always : ImGuiCond_FirstUseEver;

    auto render_aspect_image = [](uint32_t tex_id, uint32_t tex_w, uint32_t tex_h) {
        ImVec2 avail_size = ImGui::GetContentRegionAvail();
        if (tex_id != 0 && avail_size.x > 10 && avail_size.y > 10) {
            float tex_aspect = static_cast<float>(tex_w) / static_cast<float>(tex_h);
            float avail_aspect = avail_size.x / avail_size.y;
            ImVec2 draw_size = avail_size;
            if (avail_aspect > tex_aspect) {
                draw_size.x = avail_size.y * tex_aspect;
            } else {
                draw_size.y = avail_size.x / tex_aspect;
            }
            float offset_x = (avail_size.x - draw_size.x) * 0.5f;
            float offset_y = (avail_size.y - draw_size.y) * 0.5f;
            if (offset_x > 0.0f) ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset_x);
            if (offset_y > 0.0f) ImGui::SetCursorPosY(ImGui::GetCursorPosY() + offset_y);
            ImGui::Image((ImTextureID)(intptr_t)tex_id, draw_size);
        }
    };

    // ------------------------------------------------------------------------
    // Viewport 1: APS Sensor View (Top-Left)
    // ------------------------------------------------------------------------
    ImGui::SetNextWindowPos(ImVec2(right_x, margin), cond);
    ImGui::SetNextWindowSize(ImVec2(half_w, top_h), cond);
    ImGui::Begin("APS Sensor View (Live RAW / sRGB)", nullptr, ImGuiWindowFlags_NoCollapse);
    if (sensor_texture_id_ != 0) {
        render_aspect_image(sensor_texture_id_, sensor_tex_w_, sensor_tex_h_);
    } else {
        ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.2f, 1.0f), "APS Camera Sensor Active");
        ImGui::Text("Simulated FPS: %.1f Hz", config_.sim_fps);
        ImGui::Text("Exposure: %.2f ms", config_.exposure_ms);
    }
    ImGui::End();

    // ------------------------------------------------------------------------
    // Viewport 2: EVS Neuromorphic Event Viewport (Top-Right)
    // ------------------------------------------------------------------------
    ImGui::SetNextWindowPos(ImVec2(right_x + half_w + margin, margin), cond);
    ImGui::SetNextWindowSize(ImVec2(half_w, top_h), cond);
    ImGui::Begin("EVS Neuromorphic Events (Accumulation Map)", nullptr, ImGuiWindowFlags_NoCollapse);
    if (evs_texture_id_ != 0) {
        render_aspect_image(evs_texture_id_, sensor_tex_w_, sensor_tex_h_);
    } else {
        ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "EVS Stream: Red = ON (+1), Blue = OFF (-1)");
        ImGui::Text("Contrast Threshold theta: %.3f", config_.event_threshold);
        ImGui::Text("Accumulation Window: %.1f ms", config_.accumulation_window_ms);
    }
    ImGui::End();

    // ------------------------------------------------------------------------
    // Viewport 3: 3D Orbit View (Bottom-Left)
    // ------------------------------------------------------------------------
    ImGui::SetNextWindowPos(ImVec2(right_x, bot_y), cond);
    ImGui::SetNextWindowSize(ImVec2(half_w, bot_h), cond);
    ImGui::Begin("3D Trajectory & World View", nullptr, ImGuiWindowFlags_NoCollapse);
    if (orbit_texture_id_ != 0) {
        render_aspect_image(orbit_texture_id_, sensor_tex_w_, sensor_tex_h_);
    } else {
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 0.4f, 1.0f), "World View: Active Camera & 3D Spline Path");
        ImGui::Text("Current Timestamp: %.3f s / %.3f s", current_time_sec_, spline_.max_time());
    }
    ImGui::End();

    // ------------------------------------------------------------------------
    // Viewport 4: IMU & Telemetry Waveforms (Bottom-Right)
    // ------------------------------------------------------------------------
    ImGui::SetNextWindowPos(ImVec2(right_x + half_w + margin, bot_y), cond);
    ImGui::SetNextWindowSize(ImVec2(half_w, bot_h), cond);
    render_imu_plots();

    if (layout_init_frames_ > 0) {
        layout_init_frames_--;
    }
}

void GuiApp::render_imu_plots() {
    ImGui::Begin("IMU Telemetry & Kinematics", nullptr, ImGuiWindowFlags_NoCollapse);

    float avail_y = ImGui::GetContentRegionAvail().y;
    float plot_h = std::max(80.0f, (avail_y - 30.0f) * 0.5f);

    if (ImPlot::BeginPlot("Angular Velocity (Gyro) [rad/s]", ImVec2(-1, plot_h))) {
        ImPlot::SetupAxes("Time [s]", "rad/s", ImPlotAxisFlags_None, ImPlotAxisFlags_None);
        if (!plot_time_.empty()) {
            ImPlot::PlotLine("Gyro X", plot_time_.data(), plot_gyro_x_.data(), (int)plot_time_.size());
            ImPlot::PlotLine("Gyro Y", plot_time_.data(), plot_gyro_y_.data(), (int)plot_time_.size());
            ImPlot::PlotLine("Gyro Z", plot_time_.data(), plot_gyro_z_.data(), (int)plot_time_.size());
        }
        ImPlot::EndPlot();
    }

    if (ImPlot::BeginPlot("Linear Acceleration (Acc) [m/s^2]", ImVec2(-1, plot_h))) {
        ImPlot::SetupAxes("Time [s]", "m/s^2", ImPlotAxisFlags_None, ImPlotAxisFlags_None);
        if (!plot_time_.empty()) {
            ImPlot::PlotLine("Acc X", plot_time_.data(), plot_acc_x_.data(), (int)plot_time_.size());
            ImPlot::PlotLine("Acc Y", plot_time_.data(), plot_acc_y_.data(), (int)plot_time_.size());
            ImPlot::PlotLine("Acc Z", plot_time_.data(), plot_acc_z_.data(), (int)plot_time_.size());
        }
        ImPlot::EndPlot();
    }

    ImGui::End();
}

} // namespace hesim3d
