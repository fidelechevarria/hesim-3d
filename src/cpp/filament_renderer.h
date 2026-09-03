#pragma once

#include <filament/Engine.h>
#include <filament/Renderer.h>
#include <filament/Scene.h>
#include <filament/View.h>
#include <filament/Camera.h>
#include <filament/SwapChain.h>
#include <filament/RenderTarget.h>
#include <filament/Texture.h>
#include <filament/Viewport.h>
#include <filament/IndirectLight.h>
#include <filament/Skybox.h>
#include <filament/LightManager.h>
#include <filament/Box.h>
#include <filament/RenderableManager.h>

#include <gltfio/AssetLoader.h>
#include <gltfio/FilamentAsset.h>
#include <gltfio/ResourceLoader.h>
#include <gltfio/MaterialProvider.h>
#include <gltfio/TextureProvider.h>

#include <utils/EntityManager.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>
#include <memory>
#include <cstdint>

#include "buffer_utils.h"

namespace hesim3d {

namespace gltfio = filament::gltfio;

struct SceneBounds {
    Eigen::Vector3d min_point{-1.0, -1.0, -1.0};
    Eigen::Vector3d max_point{1.0, 1.0, 1.0};
    Eigen::Vector3d center{0.0, 0.0, 0.0};
    Eigen::Vector3d extent{2.0, 2.0, 2.0};
    double radius{1.732};
    bool valid{false};
};

struct CameraIntrinsics {
    uint32_t width{640};
    uint32_t height{480};
    double fx{500.0};
    double fy{500.0};
    double cx{320.0};
    double cy{240.0};
    double near_plane{0.05};
    double far_plane{100.0};
};

class FilamentRenderer {
public:
    FilamentRenderer(uint32_t width, uint32_t height, const std::string& backend_type = "vulkan");
    ~FilamentRenderer();

    // Scene & Environment
    bool load_scene(const std::string& glb_path);
    bool load_environment(const std::string& ibl_path = "");
    void setup_default_lighting();
    const SceneBounds& get_scene_bounds() const { return scene_bounds_; }

    // Camera Configuration
    void set_intrinsics(const CameraIntrinsics& intrinsics);
    void set_camera_pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

    // Offscreen Rendering
    bool render_frame(uint8_t* out_rgb_buffer, size_t buffer_size, uint64_t timestamp_us = 0);

    // Orthographic Offscreen Rendering (Top = 0, Front = 1, Side = 2)
    bool render_ortho_frame(int ortho_idx,
                            const Eigen::Vector2d& pan,
                            double scale,
                            float canvas_w,
                            float canvas_h,
                            uint8_t* out_rgb_buffer,
                            size_t buffer_size);
    
    // Batch Rendering to RingBuffer
    size_t render_batch(const std::vector<Eigen::Vector3d>& positions,
                        const std::vector<Eigen::Quaterniond>& orientations,
                        const std::vector<uint64_t>& timestamps_us,
                        RingBuffer& ring_buffer);

    uint32_t width() const { return width_; }
    uint32_t height() const { return height_; }

    filament::Engine* get_engine() { return engine_; }
    filament::Scene* get_scene() { return scene_; }
    filament::View* get_view() { return view_; }

private:
    uint32_t width_;
    uint32_t height_;
    std::string backend_str_;

    filament::Engine* engine_{nullptr};
    filament::Renderer* renderer_{nullptr};
    filament::Scene* scene_{nullptr};
    filament::View* view_{nullptr};
    filament::Camera* camera_{nullptr};
    filament::SwapChain* swap_chain_{nullptr};

    utils::Entity ortho_camera_entity_;
    filament::Camera* ortho_camera_{nullptr};

    filament::IndirectLight* indirect_light_{nullptr};
    filament::Skybox* skybox_{nullptr};
    utils::Entity sunlight_entity_;

    gltfio::MaterialProvider* material_provider_{nullptr};
    gltfio::AssetLoader* asset_loader_{nullptr};
    gltfio::FilamentAsset* asset_{nullptr};
    gltfio::ResourceLoader* resource_loader_{nullptr};
    gltfio::TextureProvider* stb_provider_{nullptr};

    utils::Entity camera_entity_;
    CameraIntrinsics intrinsics_;
    SceneBounds scene_bounds_;

    std::vector<uint8_t> readback_scratch_;

    void init_engine();
    void cleanup();
};

} // namespace hesim3d
