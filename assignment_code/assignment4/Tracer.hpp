#ifndef TRACER_H_
#define TRACER_H_

#include "gloo/Scene.hpp"
#include "gloo/Material.hpp"
#include "gloo/lights/LightBase.hpp"
#include "gloo/components/LightComponent.hpp"

#include "Ray.hpp"
#include "HitRecord.hpp"
#include "TracingComponent.hpp"
#include "CubeMap.hpp"
#include "PerspectiveCamera.hpp"

namespace GLOO {
class Tracer {
 public:
  Tracer(const CameraSpec& camera_spec,
         const glm::ivec2& image_size,
         size_t max_bounces,
         const glm::vec3& background_color,
         const CubeMap* cube_map,
         bool shadows_enabled,
         bool jitter,
         bool filter,
         glm::vec3 fog_color = glm::vec3(0.0f),
         float fog_density = 0.0f,
         float fog_opacity = 0.5f)
      : camera_(camera_spec),
        image_size_(image_size),
        max_bounces_(max_bounces),
        background_color_(background_color),
        cube_map_(cube_map),
        shadows_enabled_(shadows_enabled),
        jitter_enabled_(jitter),
        filter_enabled_(filter),
        fog_color_(fog_color),
        fog_density_(fog_density),
        fog_opacity_(fog_opacity),
        scene_ptr_(nullptr) {
  }
  void Render(const Scene& scene, const std::string& output_file);

 private:
  glm::vec3 TraceRay(const Ray& ray, size_t bounces, HitRecord& record) const;

  glm::vec3 GetBackgroundColor(const glm::vec3& direction) const;

  PerspectiveCamera camera_;
  glm::ivec2 image_size_;
  size_t max_bounces_;

  std::vector<TracingComponent*> tracing_components_;
  std::vector<LightComponent*> light_components_;
  glm::vec3 background_color_;
  const CubeMap* cube_map_;
  bool shadows_enabled_;

  glm::vec3 fog_color_;
  float fog_density_;
  float fog_opacity_;

  bool jitter_enabled_;
  bool filter_enabled_;

  const Scene* scene_ptr_;
};
}  // namespace GLOO

#endif
