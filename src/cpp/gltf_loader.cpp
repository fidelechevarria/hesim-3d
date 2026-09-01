#include "filament_renderer.h"
#include <gltfio/AssetLoader.h>
#include <gltfio/FilamentAsset.h>
#include <gltfio/ResourceLoader.h>
#include <gltfio/MaterialProvider.h>
#include <utils/EntityManager.h>
#include <fstream>
#include <iostream>

namespace hesim3d {

bool FilamentRenderer::load_scene(const std::string& glb_path) {
    if (!engine_ || !scene_) {
        std::cerr << "[FilamentRenderer] Engine or Scene not initialized." << std::endl;
        return false;
    }

    // Read GLB file
    std::ifstream file(glb_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        std::cerr << "[FilamentRenderer] Failed to open GLB file: " << glb_path << std::endl;
        return false;
    }

    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<uint8_t> buffer(size);
    if (!file.read(reinterpret_cast<char*>(buffer.data()), size)) {
        std::cerr << "[FilamentRenderer] Failed to read GLB contents from: " << glb_path << std::endl;
        return false;
    }

    // Clean previous asset if any
    if (asset_ && asset_loader_) {
        scene_->removeEntities(asset_->getEntities(), asset_->getEntityCount());
        asset_loader_->destroyAsset(asset_);
        asset_ = nullptr;
    }

    if (!material_provider_) {
        material_provider_ = gltfio::createJitShaderProvider(engine_);
    }

    if (!asset_loader_) {
        gltfio::AssetConfiguration config;
        config.engine = engine_;
        config.materials = material_provider_;
        config.entities = &utils::EntityManager::get();
        asset_loader_ = gltfio::AssetLoader::create(config);
    }

    asset_ = asset_loader_->createAsset(buffer.data(), buffer.size());
    if (!asset_) {
        std::cerr << "[FilamentRenderer] Failed to parse glTF asset: " << glb_path << std::endl;
        return false;
    }

    gltfio::ResourceConfiguration res_config;
    res_config.engine = engine_;
    res_config.gltfPath = glb_path.c_str();
    res_config.normalizeSkinningWeights = true;

    gltfio::ResourceLoader resource_loader(res_config);
    if (!resource_loader.loadResources(asset_)) {
        std::cerr << "[FilamentRenderer] Failed to load glTF resources for: " << glb_path << std::endl;
        asset_loader_->destroyAsset(asset_);
        asset_ = nullptr;
        return false;
    }

    scene_->addEntities(asset_->getEntities(), asset_->getEntityCount());
    return true;
}

} // namespace hesim3d
