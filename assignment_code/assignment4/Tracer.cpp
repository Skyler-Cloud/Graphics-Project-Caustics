#include "Tracer.hpp"

#include <glm/gtx/string_cast.hpp>
#include <stdexcept>
#include <algorithm>

#include "gloo/Transform.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/lights/AmbientLight.hpp"

#include "gloo/Image.hpp"
#include "Illuminator.hpp"
#include <random>

namespace GLOO {
void Tracer::Render(const Scene& scene, const std::string& output_file) {
  scene_ptr_ = &scene;

  auto& root = scene_ptr_->GetRootNode();
  tracing_components_ = root.GetComponentPtrsInChildren<TracingComponent>();
  light_components_ = root.GetComponentPtrsInChildren<LightComponent>();

  // Build photon map for caustics if enabled
  if (enable_caustics_) {
    std::cout << "[Photon Mapping] Building photon map with "
              << num_photons_ << " photons..." << std::endl;
    photon_map_.BuildPhotonMap(scene, num_photons_);
    std::cout << "[Photon Mapping] Number of photons stored: " << photon_map_.getNPhotons() << std::endl;
  }
  else {
    std::cout << "[Photon Mapping] Caustics disabled." << std::endl;
  }

  std::default_random_engine generator;
  generator.seed(time(0));
  std::uniform_real_distribution<float> distribution(-1.0f,1.0f);
  float variance = 0.5f;

  Image image(image_size_.x, image_size_.y);

  for (size_t y = 0; y < image_size_.y; y++) {
    for (size_t x = 0; x < image_size_.x; x++) {
      // TODO: For each pixel, cast a ray, and update its value in the image.
      glm::vec2 point = glm::vec2(
        ( 2*(x+0.5f)/image_size_.x ) - 1,
        ( 2*(y+0.5f)/image_size_.y ) - 1
      );
      Ray original_ray = camera_.GenerateRay(point);
      HitRecord original_record;
      glm::vec3 color = TraceRay(original_ray, max_bounces_, original_record);
      float total_weight = 1.0f;

      if (jitter_enabled_) {
        for (int i=0;i<20;i++) { // antialiasing by sampling multiple ray locations
          // get perturbation of the sample
          glm::vec2 offset = glm::vec2(
            distribution(generator), distribution(generator)
          );
          float weight = glm::exp( - glm::length(offset)*glm::length(offset) / (2.0f*variance*variance));
            // See (gaussian function): https://mathworld.wolfram.com/GaussianFunction.html
            // apply Gaussian filter to uniform distribution
          weight = (filter_enabled_)? weight : 1.0f;

          Ray ray = camera_.GenerateRay(
            point + glm::vec2(offset.x/image_size_.x, offset.y/image_size_.y)
          );
          HitRecord record;
          color += ( TraceRay(ray, max_bounces_, record) * weight );
          total_weight += weight;
        }
      }

      image.SetPixel(x, y, color/total_weight);
    }
  }

  // glm::mat4 view_proj = camera_.GetProjectionMatrix() * camera_.GetViewMatrix();
  // std::cout << "[Photon Mapping] Visualizing photons on image..." << std::endl;
  // for (const Photon& photon : photon_map_.getPhotons()) {
  //     glm::vec4 clip = view_proj * glm::vec4(photon.position, 1.0f);
  //     if (clip.w == 0.0f) continue;
  //     glm::vec3 ndc = glm::vec3(clip) / clip.w; // normalized device coordinates
  //     int px = static_cast<int>((ndc.x * 0.5f + 0.5f) * image_size_.x);
  //     int py = static_cast<int>((1.0f - (ndc.y * 0.5f + 0.5f)) * image_size_.y); // flip y

  //     if (px >= 0 && px < image_size_.x && py >= 0 && py < image_size_.y && photon.position.y  == 0.0f) {
  //         image.SetPixel(px, py, glm::vec3(0, 0, 1)); // Blue dot for photon
  //     }
  // }

  if (output_file.size())
    image.SavePNG(output_file);
}


glm::vec3 Tracer::TraceRay(const Ray& ray,
                           size_t bounces,
                           HitRecord& record) const {
  //TODO: Compute the color for the cast ray.

  // Find intersection
  Material hit_material;
  for (auto p : tracing_components_) {
    glm::mat4 transform = p->GetNodePtr()->GetTransform().GetLocalToWorldMatrix();
    glm::mat4 inverse_transform = glm::inverse(transform);
    Ray new_ray = Ray(ray.GetOrigin(), ray.GetDirection());
    new_ray.ApplyTransform(inverse_transform); // Transform ray to object space

    HitRecord local_record;
    if (p->GetHittable().Intersect(new_ray, 0.001f, local_record)) {
      // get t of hit in world position
      glm::vec4 local_hit = glm::vec4(new_ray.At(local_record.time), 1.0f);
      glm::vec3 world_hit = glm::vec3(transform * local_hit);
      float world_t = glm::length(world_hit - ray.GetOrigin());

      if (world_t < record.time && world_t >= 0.001f) {
        record.time = world_t;

        // get world normal
        record.normal = glm::normalize(glm::transpose(glm::mat3(inverse_transform)) * local_record.normal);
        auto material_component = p->GetNodePtr()->GetComponentPtr<MaterialComponent>();
        hit_material = (material_component == nullptr)
                      ? Material::GetDefault()
                      : material_component->GetMaterial();
      }
    }
  }

  if (record.time == std::numeric_limits<float>::max()) { // check if there's no hit
    // Cap fog effect at a certain amount with fog_opacity_
    return GetBackgroundColor(ray.GetDirection()) * (1.0f-fog_opacity_) + fog_opacity_*fog_color_;
  }

  if (hit_material.IsTransparent()) {
    // For transparent materials, we simply continue tracing the ray
    // without any shading at the intersection point.
    glm::vec3 normal = record.normal;
    float ior = hit_material.GetIndexOfRefraction();
    float cosi = glm::clamp(glm::dot(ray.GetDirection(), normal), -1.0f, 1.0f);
    float etai = 1.0f, etat = ior;
    if (cosi < 0) {
      cosi = -cosi;
    } else {
      std::swap(etai, etat);
      normal = -normal;
    }
    float eta = etai / etat;
    float k = 1 - eta * eta * (1 - cosi * cosi);
    glm::vec3 direction;
      if (k < 0) {
        // Total internal reflection: trace reflected ray
        glm::vec3 reflect_dir = glm::reflect(ray.GetDirection(), normal);
        reflect_dir = glm::normalize(reflect_dir);
        Ray reflected_ray(ray.At(record.time), reflect_dir);
        HitRecord reflect_record;
        glm::vec3 result = TraceRay(reflected_ray, bounces, reflect_record);
        if (reflect_record.time == std::numeric_limits<float>::max()) {
          return GetBackgroundColor(reflect_dir);
        }
        return result;
      } else {
        direction = eta * ray.GetDirection() + (eta * cosi - glm::sqrt(k)) * normal;
        direction = glm::normalize(direction);
        Ray transmitted_ray(ray.At(record.time), direction);
        HitRecord transmit_record;
        glm::vec3 result = TraceRay(transmitted_ray, bounces, transmit_record);
        if (transmit_record.time == std::numeric_limits<float>::max()) {
          return GetBackgroundColor(direction);
        }
        return result;
      }
  }

  glm::vec3 diffuse_color = hit_material.GetDiffuseColor();
  glm::vec3 specular_color = hit_material.GetSpecularColor();
  float shininess = hit_material.GetShininess();

  glm::vec3 hit_point = ray.At(record.time); // world space
  glm::vec3 reflect_dir = glm::normalize(glm::reflect(ray.GetDirection(), record.normal));

  // Calculate Total Illumination reflecting from that point
  glm::vec3 color(0.0f);
  for (auto light : light_components_) {
    if (light->GetLightPtr()->GetType() == LightType::Ambient) {
      glm::vec3 ambient_color = hit_material.GetAmbientColor();

      auto ambient_light_ptr = static_cast<AmbientLight*>(light->GetLightPtr());
      glm::vec3 ambient_light = ambient_light_ptr->GetAmbientColor();
      color += ambient_color * ambient_light;
      continue;
    }

    glm::vec3 dir_to_light, light_intensity;
    float dist_to_light;
    Illuminator::GetIllumination(*light, hit_point, dir_to_light, light_intensity, dist_to_light);

    // Calculate shadow intersection point
    if (shadows_enabled_) {
      bool in_shadow = false;
      Ray shadow_ray(hit_point, dir_to_light);
      for (auto p : tracing_components_) {
        glm::mat4 transform = p->GetNodePtr()->GetTransform().GetLocalToWorldMatrix();
        glm::mat4 inverse_transform = glm::inverse(transform);
        Ray shadow_ray = Ray(hit_point, dir_to_light);
        shadow_ray.ApplyTransform(inverse_transform); // Transform ray to object space

        HitRecord shadow_record;
        if (p->GetHittable().Intersect(shadow_ray, 0.001f, shadow_record)) {
          glm::vec4 local_hit = glm::vec4(shadow_ray.At(shadow_record.time), 1.0f);
          glm::vec3 world_hit = glm::vec3(transform * local_hit);
          float shadow_distance = glm::length(world_hit - hit_point);
       