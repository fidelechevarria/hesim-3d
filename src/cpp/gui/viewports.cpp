#include "gui_app.h"
#include <imgui.h>
#include <implot.h>
#include <iostream>

namespace hesim3d {

void GuiApp::render_viewports() {
    // ------------------------------------------------------------------------
    // Viewport 1 & 2: 3D Orbit View & APS First-Person Sensor View
    // ------------------------------------------------------------------------
    ImGui::Begin("3D Trajectory & World View", nullptr, ImGuiWindowFlags_NoCollapse);
    ImVec2 avail_size = ImGui::GetContentRegionAvail();
    if (orbit_texture_id_ != 0) {
        ImGui::Image((ImTextureID)(intptr_t)orbit_texture_id_, avail_size);
    } else {
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 0.4f, 1.0f), "World View: Active Camera & 3D Spline Path");
        ImGui::Text("Resolution: %.0f x %.0f", avail_size.x, avail_size.y);
        ImGui::Text("Current Timestamp: %.3f s / %.3f s", current_time_sec_, spline_.max_time());
    }
    ImGui::End();

    ImGui::Begin("APS Sensor View (Live RAW / sRGB)", nullptr, ImGuiWindowFlags_NoCollapse);
    avail_size = ImGui::GetContentRegionAvail();
    if (sensor_texture_id_ != 0) {
        ImGui::Image((ImTextureID)(intptr_t)sensor_texture_id_, avail_size);
    } else {
        ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.2f, 1.0f), "APS Camera Sensor Active");
        ImGui::Text("Simulated FPS: %.1f Hz", config_.sim_fps);
        ImGui::Text("Exposure: %.2f ms", config_.exposure_ms);
    }
    ImGui::End();

    // ------------------------------------------------------------------------
    // Viewport 3: EVS Neuromorphic Event Viewport
    // ------------------------------------------------------------------------
    ImGui::Begin("EVS Neuromorphic Events (Accumulation Map)", nullptr, ImGuiWindowFlags_NoCollapse);
    avail_size = ImGui::GetContentRegionAvail();
    if (evs_texture_id_ != 0) {
        ImGui::Image((ImTextureID)(intptr_t)evs_texture_id_, avail_size);
    } else {
        ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "EVS Stream: Red = ON (+1), Blue = OFF (-1)");
        ImGui::Text("Contrast Threshold theta: %.3f", config_.event_threshold);
        ImGui::Text("Accumulation Window: %.1f ms", config_.accumulation_window_ms);
    }
    ImGui::End();

    // ------------------------------------------------------------------------
    // Viewport 4: IMU & Telemetry Waveforms (ImPlot)
    // ------------------------------------------------------------------------
    render_imu_plots();
}

void GuiApp::render_imu_plots() {
    ImGui::Begin("IMU Telemetry & Kinematics", nullptr, ImGuiWindowFlags_NoCollapse);

    if (ImPlot::BeginPlot("Angular Velocity (Gyro) [rad/s]", ImVec2(-1, 140))) {
        ImPlot::SetupAxes("Time [s]", "rad/s", ImPlotAxisFlags_None, ImPlotAxisFlags_None);
        if (!plot_time_.empty()) {
            ImPlot::PlotLine("Gyro X", plot_time_.data(), plot_gyro_x_.data(), (int)plot_time_.size());
            ImPlot::PlotLine("Gyro Y", plot_time_.data(), plot_gyro_y_.data(), (int)plot_time_.size());
            ImPlot::PlotLine("Gyro Z", plot_time_.data(), plot_gyro_z_.data(), (int)plot_time_.size());
        }
        ImPlot::EndPlot();
    }

    if (ImPlot::BeginPlot("Linear Acceleration (Acc) [m/s^2]", ImVec2(-1, 140))) {
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
