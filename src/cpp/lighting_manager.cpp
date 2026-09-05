#include <filament/IndirectLight.h>
#include <filament/LightManager.h>
#include <filament/Skybox.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <utils/EntityManager.h>

#include <cmath>
#include <iostream>

#include "filament_renderer.h"

namespace hesim3d {

void FilamentRenderer::setup_default_lighting() {
  if (!engine_ || !scene_) return;

  // 1. Directional Sun Light
  sunlight_entity_ = utils::EntityManager::get().create();
  filament::math::float3 sun_dir(0.5f, -1.0f, -0.8f);
  float len = std::sqrt(sun_dir.x * sun_dir.x + sun_dir.y * sun_dir.y + sun_dir.z * sun_dir.z);
  if (len > 0.0f) sun_dir /= len;

  filament::LightManager::Builder(filament::LightManager::Type::DIRECTIONAL)
      .color(
          filament::Color::toLinear<filament::ACCURATE>(filament::sRGBColor(0.98f, 0.95f, 0.88f)))
      .intensity(110000.0f)  // Lux (bright daylight)
      .direction(sun_dir)
      .castShadows(true)
      .build(*engine_, sunlight_entity_);

  scene_->addEntity(sunlight_entity_);

  // 2. Ambient Indirect Light (Spherical Harmonics irradiance for zero-config neutral PBR)
  static const filament::math::float3 sh_harmonics[9] = {
      filament::math::float3(0.75f, 0.75f, 0.80f),    filament::math::float3(0.10f, 0.10f, 0.15f),
      filament::math::float3(-0.15f, -0.15f, -0.10f), filament::math::float3(0.05f, 0.05f, 0.05f),
      filament::math::float3(0.00f, 0.00f, 0.00f),    filament::math::float3(0.00f, 0.00f, 0.00f),
      filament::math::float3(0.00f, 0.00f, 0.00f),    filament::math::float3(0.00f, 0.00f, 0.00f),
      filament::math::float3(0.00f, 0.00f, 0.00f),
  };

  indirect_light_ = filament::IndirectLight::Builder()
                        .irradiance(3, sh_harmonics)
                        .intensity(30000.0f)
                        .build(*engine_);

  scene_->setIndirectLight(indirect_light_);

  // 3. Neutral Skybox
  skybox_ = filament::Skybox::Builder()
                .color(filament::math::float4(0.85f, 0.88f, 0.95f, 1.0f))
                .build(*engine_);

  scene_->setSkybox(skybox_);
}

bool FilamentRenderer::load_environment(const std::string& ibl_path) {
  if (ibl_path.empty()) {
    setup_default_lighting();
    return true;
  }
  setup_default_lighting();
  return true;
}

}  // namespace hesim3d
