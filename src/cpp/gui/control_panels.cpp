#include "gui_app.h"
#include <imgui.h>
#include <iostream>

namespace hesim3d {

void GuiApp::render_control_panels() {
    ImGui::Begin("Simulator Controls & Inspector", nullptr, ImGuiWindowFlags_NoCollapse);

    // ------------------------------------------------------------------------
    // 1. Trajectory & Playback Controls
    // ------------------------------------------------------------------------
    if (ImGui::CollapsingHeader("Trajectory & Timeline", ImGuiTreeNodeFlags_DefaultOpen)) {
        // Play / Pause / Step Buttons
        if (is_playing_) {
            if (ImGui::Button("Pause (Space)", ImVec2(100, 30))) {
                is_playing_ = false;
            }
        } else {
            if (ImGui::Button("Play (Space)", ImVec2(100, 30))) {
                is_playing_ = true;
            }
        }

        ImGui::SameLine();
        if (ImGui::Button("Step Forward", ImVec2(100, 30))) {
            is_playing_ = false;
            update_simulation_step(1.0 / 30.0);
        }

        ImGui::SameLine();
        if (ImGui::Button("Reset Time", ImVec2(90, 30))) {
            current_time_sec_ = 0.0;
        }

        // Timeline Scrubber
        float t_val = static_cast<float>(current_time_sec_);
        float max_t = static_cast<float>(std::max(0.1, spline_.max_time()));
        if (ImGui::SliderFloat("Timeline [s]", &t_val, 0.0f, max_t, "%.2f s")) {
            current_time_sec_ = static_cast<double>(t_val);
        }

        // Playback speed
        float speed = static_cast<float>(playback_speed_);
        if (ImGui::SliderFloat("Speed Multiplier", &speed, 0.1f, 3.0f, "%.1fx")) {
            playback_speed_ = static_cast<double>(speed);
        }
    }

    // ------------------------------------------------------------------------
    // 2. Sensor & Noise Parameters (H-ESIM Model)
    // ------------------------------------------------------------------------
    if (ImGui::CollapsingHeader("H-ESIM Sensor Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Active Sensor: %s", config_.sensor_name.c_str());

        float th = static_cast<float>(config_.event_threshold);
        if (ImGui::SliderFloat("Event Threshold (theta)", &th, 0.05f, 0.80f, "%.3f")) {
            config_.event_threshold = static_cast<double>(th);
        }

        float exp_ms = static_cast<float>(config_.exposure_ms);
        if (ImGui::SliderFloat("APS Exposure [ms]", &exp_ms, 1.0f, 50.0f, "%.1f ms")) {
            config_.exposure_ms = static_cast<double>(exp_ms);
        }

        float acc_ms = static_cast<float>(config_.accumulation_window_ms);
        if (ImGui::SliderFloat("EVS Window [ms]", &acc_ms, 1.0f, 100.0f, "%.1f ms")) {
            config_.accumulation_window_ms = static_cast<double>(acc_ms);
        }

        float sim_fps = static_cast<float>(config_.sim_fps);
        if (ImGui::SliderFloat("Simulation Rate [Hz]", &sim_fps, 200.0f, 3200.0f, "%.0f Hz")) {
            config_.sim_fps = static_cast<double>(sim_fps);
        }
    }

    // ------------------------------------------------------------------------
    // 3. Scene & Lighting Controls
    // ------------------------------------------------------------------------
    if (ImGui::CollapsingHeader("Scene & Environment")) {
        ImGui::Text("Scene Path: %s", config_.scene_path.empty() ? "[Default Checkerboard Room]" : config_.scene_path.c_str());
    }

    // ------------------------------------------------------------------------
    // 4. Live Recording & Export Panel
    // ------------------------------------------------------------------------
    if (ImGui::CollapsingHeader("Dataset Recording (HDF5 / MCAP)", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (!is_recording_) {
            if (ImGui::Button("● Start Recording", ImVec2(160, 36))) {
                is_recording_ = true;
                std::cout << "[GUI] Started recording dataset to: " << recording_output_path_ << std::endl;
            }
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.2f, 0.2f, 1.0f));
            if (ImGui::Button("■ Stop Recording", ImVec2(160, 36))) {
                is_recording_ = false;
                std::cout << "[GUI] Recording saved and finalized." << std::endl;
            }
            ImGui::PopStyleColor();
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "RECORDING ACTIVE...");
        }
    }

    ImGui::End();
}

} // namespace hesim3d
