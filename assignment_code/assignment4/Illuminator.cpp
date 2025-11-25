#include "Illuminator.hpp"

#include <limits>
#include <stdexcept>

#include <glm/geometric.hpp>

#include "gloo/lights/DirectionalLight.hpp"
#include "gloo/lights/PointLight.hpp"
#include "gloo/SceneNode.hpp"

namespace GLOO {
void Illuminator::GetIllumination(const LightComponent& light_component,
                                  const glm::vec3& hit_pos,
                                  glm::vec3& dir_to_light,
                                  glm::vec3& intensity,
                                  float& dist_to_light) {
  // Calculation will be done in world space.

  auto light_ptr = light_component.GetLightPtr();
  if (light_ptr->GetType() == LightType::Directional) {
    auto directional_light_ptr = static_cast<DirectionalLight*>(light_ptr);
    dir_to_light = -directional_light_ptr->GetDirection();
    intensity = directional_light_ptr->GetDiffuseColor(); // I will assume diffuse color is same as specular color?
    dist_to_light = std::numeric_limits<float>::max();
  } else if (light_ptr->GetType() == LightType::Point) {
    glm::vec3 light_pos = light_component.GetNodePtr()->GetTransform().GetWorldPosition();
    dir_to_light = glm::normalize(light_pos - hit_pos); // points from hit_pos -> light_pos
    dist_to_light = glm::length(light_pos - hit_pos);

    // calculate color at distance
    auto point_light_ptr = static_cast<PointLight*>(light_ptr);
    glm::vec3 color = point_light_ptr->GetDiffuseColor();
    float attenuation = point_light_ptr->GetAttenuation()[2]; // only use quadratic attenuation
      // See (const/linear/quadratic attenuation): https://developer.valvesoftware.com/wiki/Constant-Linear-Quadratic_Falloff
    intensity = color / (attenuation * dist_to_light*dist_to_light);
  } else {  // TODO: Implement point light.
    throw std::runtime_error(
        "Unrecognized light type when computing "
        "illumination");
  }
}
}  // namespace GLOO
