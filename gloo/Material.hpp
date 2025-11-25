#ifndef GLOO_MATERIAL_H_
#define GLOO_MATERIAL_H_

#include <glm/glm.hpp>

namespace GLOO {
class Material {
 public:
  Material()
      : ambient_color_(0.0f),
        diffuse_color_(0.0f),
        specular_color_(0.0f),
        shininess_(0.0f),
        transparent_(false),
        index_of_refraction_(1.0f) {
  }
  Material(const glm::vec3& ambient_color,
           const glm::vec3& diffuse_color,
           const glm::vec3& specular_color,
           float shininess,
           bool transparent = false,
           float index_of_refraction = 1.0f)
      : ambient_color_(ambient_color),
        diffuse_color_(diffuse_color),
        specular_color_(specular_color),
        shininess_(shininess),
        transparent_(transparent),
        index_of_refraction_(index_of_refraction) {
  }

  static const Material& GetDefault() {
    static Material default_material(glm::vec3(0.5f, 0.1f, 0.2f),
                                     glm::vec3(0.5f, 0.1f, 0.2f),
                                     glm::vec3(0.4f, 0.4f, 0.4f), 20.0f);
    return default_material;
  }

  glm::vec3 GetAmbientColor() const {
    return ambient_color_;
  }

  void SetAmbientColor(const glm::vec3& color) {
    ambient_color_ = color;
  }

  glm::vec3 GetDiffuseColor() const {
    return diffuse_color_;
  }

  void SetDiffuseColor(const glm::vec3& color) {
    diffuse_color_ = color;
  }

  glm::vec3 GetSpecularColor() const {
    return specular_color_;
  }

  void SetSpecularColor(const glm::vec3& color) {
    specular_color_ = color;
  }

  float GetShininess() const {
    return shininess_;
  }

  void SetShininess(float shininess) {
    shininess_ = shininess;
  }

  bool IsTransparent() const {
    return transparent_;
  }

  void SetTransparent(bool transparent) {
    transparent_ = transparent;
  }

  float GetIndexOfRefraction() const {
    return index_of_refraction_;
  }

  void SetIndexOfRefraction(float index_of_refraction) {
    index_of_refraction_ = index_of_refraction;
  }

 private:
  glm::vec3 ambient_color_;
  glm::vec3 diffuse_color_;
  glm::vec3 specular_color_;
  float shininess_;
  bool transparent_;
  float index_of_refraction_;
};
}  // namespace GLOO

#endif
