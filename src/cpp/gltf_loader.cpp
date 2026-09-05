#include <gltfio/AssetLoader.h>
#include <gltfio/FilamentAsset.h>
#include <gltfio/MaterialProvider.h>
#include <gltfio/ResourceLoader.h>
#include <gltfio/TextureProvider.h>
#include <utils/EntityManager.h>

#include <fstream>
#include <iostream>

#include "filament_renderer.h"

namespace hesim3d {

bool FilamentRenderer::load_scene(const std::string& glb_path) {
  if (!engine_ || !scene_) {
    std::cerr << "[FilamentRenderer] Engine or Scene not initialized." << std::endl;
    return false;
  }

  // Read GLB / glTF file
  std::ifstream file(glb_path, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    std::cerr << "[FilamentRenderer] Failed to open scene file: " << glb_path << std::endl;
    return false;
  }

  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);

  std::vector<uint8_t> buffer(size);
  if (!file.read(reinterpret_cast<char*>(buffer.data()), size)) {
    std::cerr << "[FilamentRenderer] Failed to read scene contents from: " << glb_path << std::endl;
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

  if (!stb_provider_) {
    stb_provider_ = gltfio::createStbProvider(engine_);
  }

  if (!resource_loader_) {
    gltfio::ResourceConfiguration res_config;
    res_config.engine = engine_;
    res_config.gltfPath = glb_path.c_str();
    res_config.normalizeSkinningWeights = true;
    resource_loader_ = new gltfio::ResourceLoader(res_config);
    if (stb_provider_) {
      resource_loader_->addTextureProvider("image/png", stb_provider_);
      resource_loader_->addTextureProvider("image/jpeg", stb_provider_);
    }
  }

  if (!resource_loader_->loadResources(asset_)) {
    std::cerr << "[FilamentRenderer] Failed to load glTF resources for: " << glb_path << std::endl;
    asset_loader_->destroyAsset(asset_);
    asset_ = nullptr;
    return false;
  }

  scene_->addEntities(asset_->getEntities(), asset_->getEntityCount());

  // Compute dynamic scene bounds from loaded asset
  scene_bounds_ = SceneBounds{};

  // 1. Try gltfio asset bounding box (filament::Aabb has min and max)
  filament::Aabb asset_box = asset_->getBoundingBox();
  Eigen::Vector3d a_min(asset_box.min.x, asset_box.min.y, asset_box.min.z);
  Eigen::Vector3d a_max(asset_box.max.x, asset_box.max.y, asset_box.max.z);
  Eigen::Vector3d a_extent = a_max - a_min;

  if (a_extent.x() > 1e-4 || a_extent.y() > 1e-4 || a_extent.z() > 1e-4) {
    scene_bounds_.min_point = a_min;
    scene_bounds_.max_point = a_max;
    scene_bounds_.center = (a_min + a_max) * 0.5;
    scene_bounds_.extent = a_extent;
    scene_bounds_.radius = std::max({a_extent.x(), a_extent.y(), a_extent.z()}) * 0.5;
    scene_bounds_.valid = true;
  }

  // 2. If asset bounding box is degenerate, aggregate renderable entity boxes
  if (!scene_bounds_.valid || scene_bounds_.radius < 1e-4) {
    auto& rm = engine_->getRenderableManager();
    const utils::Entity* entities = asset_->getEntities();
    size_t count = asset_->getEntityCount();
    bool found = false;
    Eigen::Vector3d r_min(1e12, 1e12, 1e12);
    Eigen::Vector3d r_max(-1e12, -1e12, -1e12);

    for (size_t i = 0; i < count; ++i) {
      auto inst = rm.getInstance(entities[i]);
      if (inst) {
        filament::Box eb = rm.getAxisAlignedBoundingBox(inst);
        if (std::abs(eb.halfExtent.x) > 1e-5 || std::abs(eb.halfExtent.y) > 1e-5 ||
            std::abs(eb.halfExtent.z) > 1e-5) {
          Eigen::Vector3d b_min(eb.center.x - eb.halfExtent.x, eb.center.y - eb.halfExtent.y,
                                eb.center.z - eb.halfExtent.z);
          Eigen::Vector3d b_max(eb.center.x + eb.halfExtent.x, eb.center.y + eb.halfExtent.y,
                                eb.center.z + eb.halfExtent.z);
          r_min = r_min.cwiseMin(b_min);
          r_max = r_max.cwiseMax(b_max);
          found = true;
        }
      }
    }
    if (found && (r_max - r_min).norm() > 1e-4) {
      scene_bounds_.min_point = r_min;
      scene_bounds_.max_point = r_max;
      scene_bounds_.center = (r_min + r_max) * 0.5;
      scene_bounds_.extent = r_max - r_min;
      scene_bounds_.radius =
          std::max({scene_bounds_.extent.x(), scene_bounds_.extent.y(), scene_bounds_.extent.z()}) *
          0.5;
      scene_bounds_.valid = true;
    }
  }

  // 3. Fallback default bounds if model is empty or has no bounding info
  if (!scene_bounds_.valid || scene_bounds_.radius <= 1e-4) {
    scene_bounds_.min_point = Eigen::Vector3d(-1.0, -1.0, -1.0);
    scene_bounds_.max_point = Eigen::Vector3d(1.0, 1.0, 1.0);
    scene_bounds_.center = Eigen::Vector3d(0.0, 0.0, 0.0);
    scene_bounds_.extent = Eigen::Vector3d(2.0, 2.0, 2.0);
    scene_bounds_.radius = 1.0;
    scene_bounds_.valid = true;
  }

  std::cout << "[FilamentRenderer] Dynamic Scene Geometry Analyzed:" << std::endl
            << "  - Bounding Box Min:    [" << scene_bounds_.min_point.x() << ", "
            << scene_bounds_.min_point.y() << ", " << scene_bounds_.min_point.z() << "]"
            << std::endl
            << "  - Bounding Box Max:    [" << scene_bounds_.max_point.x() << ", "
            << scene_bounds_.max_point.y() << ", " << scene_bounds_.max_point.z() << "]"
            << std::endl
            << "  - Center:              [" << scene_bounds_.center.x() << ", "
            << scene_bounds_.center.y() << ", " << scene_bounds_.center.z() << "]" << std::endl
            << "  - Extent:              [" << scene_bounds_.extent.x() << ", "
            << scene_bounds_.extent.y() << ", " << scene_bounds_.extent.z() << "]" << std::endl
            << "  - Characteristic R:    " << scene_bounds_.radius << std::endl;

  return true;
}

}  // namespace hesim3d
