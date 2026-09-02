#include "filament_renderer.h"
#include <backend/PixelBufferDescriptor.h>
#include <math/mat4.h>
#include <math/vec3.h>
#include <utils/EntityManager.h>
#include <iostream>
#include <cstring>
#include <cmath>
#include <numbers>

namespace hesim3d {

FilamentRenderer::FilamentRenderer(uint32_t width, uint32_t height, const std::string& backend_type)
    : width_(width), height_(height), backend_str_(backend_type) {
    readback_scratch_.resize(width_ * height_ * 3, 0);
    init_engine();
}

FilamentRenderer::~FilamentRenderer() {
    cleanup();
}

void FilamentRenderer::init_engine() {
    filament::Engine::Backend backend = filament::Engine::Backend::VULKAN;
    if (backend_str_ == "opengl") {
        backend = filament::Engine::Backend::OPENGL;
    }

    try {
        engine_ = filament::Engine::create(backend);
    } catch (...) {
        std::cerr << "[FilamentRenderer] Failed to initialize Vulkan backend, falling back to OpenGL..." << std::endl;
        engine_ = filament::Engine::create(filament::Engine::Backend::OPENGL);
    }

    if (!engine_) {
        throw std::runtime_error("[FilamentRenderer] Could not initialize Google Filament engine.");
    }

    swap_chain_ = engine_->createSwapChain(width_, height_, filament::SwapChain::CONFIG_READABLE);
    renderer_ = engine_->createRenderer();
    scene_ = engine_->createScene();
    view_ = engine_->createView();

    camera_entity_ = utils::EntityManager::get().create();
    camera_ = engine_->createCamera(camera_entity_);

    view_->setCamera(camera_);
    view_->setScene(scene_);
    view_->setViewport(filament::Viewport(0, 0, width_, height_));

    // Post-processing options for clean sensor rendering (linear HDR colors)
    view_->setPostProcessingEnabled(true);
    view_->setDithering(filament::View::Dithering::NONE);
    view_->setShadowingEnabled(true);

    setup_default_lighting();
}

void FilamentRenderer::cleanup() {
    if (!engine_) return;

    if (asset_ && asset_loader_) {
        scene_->removeEntities(asset_->getEntities(), asset_->getEntityCount());
        asset_loader_->destroyAsset(asset_);
        asset_ = nullptr;
    }

    if (resource_loader_) {
        delete resource_loader_;
        resource_loader_ = nullptr;
    }

    if (stb_provider_) {
        delete stb_provider_;
        stb_provider_ = nullptr;
    }

    if (asset_loader_) {
        gltfio::AssetLoader::destroy(&asset_loader_);
    }

    if (material_provider_) {
        material_provider_->destroyMaterials();
        delete material_provider_;
        material_provider_ = nullptr;
    }

    if (indirect_light_) {
        engine_->destroy(indirect_light_);
        indirect_light_ = nullptr;
    }

    if (skybox_) {
        engine_->destroy(skybox_);
        skybox_ = nullptr;
    }

    if (sunlight_entity_) {
        scene_->remove(sunlight_entity_);
        engine_->destroy(sunlight_entity_);
    }

    if (camera_) {
        engine_->destroyCameraComponent(camera_entity_);
        utils::EntityManager::get().destroy(camera_entity_);
        camera_ = nullptr;
    }

    if (view_) {
        engine_->destroy(view_);
        view_ = nullptr;
    }

    if (scene_) {
        engine_->destroy(scene_);
        scene_ = nullptr;
    }

    if (renderer_) {
        engine_->destroy(renderer_);
        renderer_ = nullptr;
    }

    if (swap_chain_) {
        engine_->destroy(swap_chain_);
        swap_chain_ = nullptr;
    }

    filament::Engine::destroy(&engine_);
}

void FilamentRenderer::set_intrinsics(const CameraIntrinsics& intrinsics) {
    intrinsics_ = intrinsics;
    if (!camera_) return;

    double fov_y_deg = 2.0 * std::atan(0.5 * intrinsics_.height / intrinsics_.fy) * 180.0 / std::numbers::pi;
    double aspect = static_cast<double>(intrinsics_.width) / std::max(1.0, static_cast<double>(intrinsics_.height));

    camera_->setProjection(fov_y_deg, aspect, intrinsics_.near_plane, intrinsics_.far_plane, filament::Camera::Fov::VERTICAL);
}

void FilamentRenderer::set_camera_pose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
    if (!camera_) return;

    Eigen::Matrix3d R = orientation.toRotationMatrix();
    filament::math::mat4f model;

    // Col 0, 1, 2, 3
    model[0] = filament::math::float4(R(0, 0), R(1, 0), R(2, 0), 0.0f);
    model[1] = filament::math::float4(R(0, 1), R(1, 1), R(2, 1), 0.0f);
    model[2] = filament::math::float4(R(0, 2), R(1, 2), R(2, 2), 0.0f);
    model[3] = filament::math::float4(position.x(), position.y(), position.z(), 1.0f);

    camera_->setModelMatrix(model);
}

bool FilamentRenderer::render_frame(uint8_t* out_rgb_buffer, size_t buffer_size, uint64_t timestamp_us) {
    if (!renderer_ || !swap_chain_ || !view_) return false;
    size_t expected_size = width_ * height_ * 3;
    if (buffer_size < expected_size) return false;

    if (renderer_->beginFrame(swap_chain_)) {
        renderer_->render(view_);

        struct CallbackUserData {
            uint8_t* dst;
            size_t size;
            bool done{false};
        };

        CallbackUserData ud{out_rgb_buffer, expected_size, false};

        filament::backend::PixelBufferDescriptor pbd(
            out_rgb_buffer,
            expected_size,
            filament::backend::PixelDataFormat::RGB,
            filament::backend::PixelDataType::UBYTE,
            [](void* buffer, size_t size, void* user) {
                auto* cb = static_cast<CallbackUserData*>(user);
                cb->done = true;
            },
            &ud
        );

        renderer_->readPixels(0, 0, width_, height_, std::move(pbd));
        renderer_->endFrame();
        engine_->flushAndWait();
        return true;
    }
    return false;
}

size_t FilamentRenderer::render_batch(const std::vector<Eigen::Vector3d>& positions,
                                     const std::vector<Eigen::Quaterniond>& orientations,
                                     const std::vector<uint64_t>& timestamps_us,
                                     RingBuffer& ring_buffer) {
    size_t num_frames = positions.size();
    size_t rendered = 0;
    size_t frame_bytes = width_ * height_ * 3;

    for (size_t i = 0; i < num_frames; ++i) {
        set_camera_pose(positions[i], orientations[i]);
        uint64_t t_us = (i < timestamps_us.size()) ? timestamps_us[i] : 0;

        if (render_frame(readback_scratch_.data(), frame_bytes, t_us)) {
            ring_buffer.push(readback_scratch_.data(), frame_bytes, t_us);
            rendered++;
        }
    }
    return rendered;
}

} // namespace hesim3d
