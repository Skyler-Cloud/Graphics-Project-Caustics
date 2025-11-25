#include "Plane.hpp"

namespace GLOO {
Plane::Plane(const glm::vec3& normal, float d) {
  normal_ = glm::normalize(normal);
  d_ = d;
}

bool Plane::Intersect(const Ray& ray, float t_min, HitRecord& record) const {
  // TODO: Implement ray-plane intersection.
  if (glm::dot(normal_, ray.GetDirection()) == 0) {
    return false; // parallel to plane
  }

  float t = (d_ - glm::dot(normal_, ray.GetOrigin())) / glm::dot(normal_, ray.GetDirection());
  if (t < t_min || t >= record.time) {
    return false;
  }
  record.time = t;
  record.normal = normal_;
  return true;
}
}  // namespace GLOO
