#ifndef PHOTON_MAP_HPP
#define PHOTON_MAP_HPP

#include "gloo/Scene.hpp"
#include "gloo/Material.hpp"
#include "gloo/lights/LightBase.hpp"
#include "gloo/lights/PointLight.hpp"
#include "gloo/lights/DirectionalLight.hpp"
#include "gloo/components/LightComponent.hpp"

#include "Ray.hpp"
#include "HitRecord.hpp"
#include "TracingComponent.hpp"
#include "gloo/Transform.hpp"
#include "gloo/components/MaterialComponent.hpp"

#include <random>
#include <vector>
#include "KdTree.hpp"

namespace GLOO {

class PhotonMap {
private:
  std::vector<Photon> photons;
  KdTree kdtree;

  void EmitPhotons(const Scene& scene, int num_photons) {
    auto& root = scene.GetRootNode();
    auto light_components = root.GetComponentPtrsInChildren<LightComponent>();
    auto tracing_components = root.GetComponentPtrsInChildren<TracingComponent>();

    if (light_components.empty()) return;

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> uniform_dist(0.0f, 1.0f);
    std::uniform_real_distribution<float> angle_dist(0.0f, 2.0f * 3.14159265f);

    for (auto light_component : light_components) {
      for (int i = 0; i < num_photons; ++i) {
        auto light_ptr = light_component->GetLightPtr();

        glm::vec3 photon_pos, photon_dir, photon_power;


        if (light_ptr->GetType() == LightType::Point) {
          auto point_light = static_cast<PointLight*>(light_ptr);
          photon_pos = light_component->GetNodePtr()->GetTransform().GetWorldPosition();
          photon_power = point_light->GetDiffuseColor() / static_cast<float>(num_photons);

          // Uniform sphere sampling
          float phi = angle_dist(rng);
          float cosTheta = 1.0f - 2.0f * uniform_dist(rng);
          float sinTheta = glm::sqrt(glm::max(0.0f, 1.0f - cosTheta * cosTheta));
          photon_dir = glm::normalize(glm::vec3(
            sinTheta * glm::cos(phi),
            cosTheta,
            sinTheta * glm::sin(phi)
          ));
        } else if (light_ptr->GetType() == LightType::Directional) {
          // std::cout << "Emitting photons from directional light" << std::endl;
          auto directional_light = static_cast<DirectionalLight*>(light_ptr);
          glm::vec3 light_dir = glm::normalize(directional_light->GetDirection());
          photon_power = directional_light->GetDiffuseColor() / static_cast<float>(num_photons) * 20.f;

          // --- Emit from a plane covering the scene ---

          // Hardcoded scene bounds (adjust as needed for your scenes)
          glm::vec3 min_bound(0.0f, 0.0f, -2.0f);
          glm::vec3 max_bound(8.0f, 6.0f, 6.0f);
          glm::vec3 center = 0.5f * (min_bound + max_bound);
          glm::vec3 extent = max_bound - min_bound;
          float plane_size = glm::max(extent.x, extent.z) * 1.5f + 1.0f;

          // Build orthonormal basis for the plane
          glm::vec3 axis = light_dir;
          glm::vec3 tmp = (glm::abs(axis.y) < 0.999f) ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
          glm::vec3 right = glm::normalize(glm::cross(tmp, axis));
          glm::vec3 up = glm::normalize(glm::cross(axis, right));

          // Random point on the plane
          float u = uniform_dist(rng) - 0.5f;
          float v = uniform_dist(rng) - 0.5f;
          glm::vec3 plane_center = center + light_dir * (0.5f * extent.y + 2.0f); // offset above scene
          photon_pos = plane_center + u * plane_size * right + v * plane_size * up;

          // Emit all photons in exactly the light direction (no cone)
          photon_dir = light_dir;
        } else {
          continue;
        }

        // Trace photon through scene
        TracePhoton(scene, tracing_components, photon_pos, photon_dir, photon_power, 0);
      }
    }


  }

void TracePhoton(const Scene& scene, const std::vector<TracingComponent*>& tracing_components,
                glm::vec3 position, glm::vec3 direction, glm::vec3 power, int depth, bool has_refracted = false) {
                      // Stop after a few bounces but keep even low-power photons so the map isn’t empty.
    if (depth > 5) return;

    Ray ray(position, direction);
    HitRecord record;
    record.time = std::numeric_limits<float>::max();

    Material hit_material;
    bool hit = false;

    // Find closest intersection
    for (auto p : tracing_components) {
      glm::mat4 transform = p->GetNodePtr()->GetTransform().GetLocalToWorldMatrix();
      glm::mat4 inverse_transform = glm::inverse(transform);
      Ray local_ray(ray.GetOrigin(), ray.GetDirection());
      local_ray.ApplyTransform(inverse_transform);

      HitRecord local_record;
      if (p->GetHittable().Intersect(local_ray, 0.00001f, local_record)) {
        glm::vec4 local_hit = glm::vec4(local_ray.At(local_record.time), 1.0f);
        glm::vec3 world_hit = glm::vec3(transform * local_hit);
        float world_t = glm::length(world_hit - ray.GetOrigin());

        if (world_t < record.time && world_t >= 0.00001f) {
          record.time = world_t;
          record.normal = glm::normalize(glm::transpose(glm::mat3(inverse_transform)) * local_record.normal);
          auto material_component = p->GetNodePtr()->GetComponentPtr<MaterialComponent>();
          hit_material = (material_component == nullptr) ? Material::GetDefault() : material_component->GetMaterial();
          hit = true;
        }
      }
    }

    if (!hit) return;

    glm::vec3 hit_point = ray.At(record.time);

    // Store photon at diffuse surfaces
    if (glm::length(hit_material.GetSpecularColor()) < 0.1f && !hit_material.IsTransparent() && has_refracted) {
    // if (glm::length(hit_material.GetSpecularColor()) < 0.1f && !hit_material.IsTransparent()) {
      // Diffuse surface - store photon
      photons.emplace_back(power, hit_point, direction);
    }

    // Continue tracing for specular/refractive surfaces
    if (glm::length(hit_material.GetSpecularColor()) > 0.1f) {
      // Reflection
      glm::vec3 reflect_dir = glm::reflect(direction, record.normal);
      glm::vec3 new_power = power * hit_material.GetSpecularColor();
      TracePhoton(scene, tracing_components, hit_point + 0.00001f * reflect_dir, reflect_dir, new_power, depth + 1, has_refracted);
    }

    if (hit_material.IsTransparent()) {
      // Refraction
      float ior = hit_material.GetIndexOfRefraction();
      float cosi = glm::clamp(glm::dot(direction, record.normal), -1.0f, 1.0f);
      float etai = 1.0f, etat = ior;
      glm::vec3 normal = record.normal;
      if (cosi < 0) {
        cosi = -cosi;
      } else {
        std::swap(etai, etat);
        normal = -normal;
      }
      float eta = etai / etat;
      float k = 1 - eta * eta * (1 - cosi * cosi);
      if (k >= 0) {
        glm::vec3 refract_dir = eta * direction + (eta * cosi - glm::sqrt(k)) * normal;
        refract_dir = glm::normalize(refract_dir);
        TracePhoton(scene, tracing_components, hit_point + 0.00001f * refract_dir, refract_dir, power, depth + 1, true);
      }
    }
  }

public:
  PhotonMap() {}

  int getNPhotons() const { return photons.size(); }
  const Photon& getIthPhoton(int i) const { return photons[i]; }

  void BuildPhotonMap(const Scene& scene, int num_photons) {
    photons.clear();
    EmitPhotons(scene, num_photons);

    if (photons.empty()) return;

    kdtree.setPoints(photons.data(), photons.size());
    kdtree.buildTree();
  }

  std::vector<int> queryKNearestPhotons(const glm::vec3& p, int k, float& max_dist2) const {
    return kdtree.searchKNearest(p, k, max_dist2);
  }

  std::vector<Photon> getPhotons() const {
    return photons;
  }
};

} // namespace GLOO
#endif
