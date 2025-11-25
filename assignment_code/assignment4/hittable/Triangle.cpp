#include "Triangle.hpp"

#include <iostream>
#include <stdexcept>

#include <glm/common.hpp>
#include <glm/gtx/string_cast.hpp>

#include "Plane.hpp"

namespace GLOO {
Triangle::Triangle(const glm::vec3& p0,
                   const glm::vec3& p1,
                   const glm::vec3& p2,
                   const glm::vec3& n0,
                   const glm::vec3& n1,
                   const glm::vec3& n2) {
  positions_.push_back(p0);
  positions_.push_back(p1);
  positions_.push_back(p2);
  normals_.push_back(n0);
  normals_.push_back(n1);
  normals_.push_back(n2);
}

Triangle::Triangle(const std::vector<glm::vec3>& positions,
                   const std::vector<glm::vec3>& normals) {
  if (positions.size() != 3 || normals.size() != 3) {
    throw std::runtime_error(
      "Triangle expected 3 positions and 3 normals, got: ("
      + std::to_string(positions.size()) + ", "
      + std::to_string(normals.size()) + ")"
    );
  }
  for (size_t i = 0; i < positions.size(); i++) {
    positions_.push_back(positions[i]);
    normals_.push_back(normals[i]);
  }

}

bool Triangle::Intersect(const Ray& ray, float t_min, HitRecord& record) const {
  // TODO: Implement ray-triangle intersection.

  // calculate barycentric coordinates
  glm::mat3x3 A(positions_[0] - positions_[1],
                positions_[0] - positions_[2],
                ray.GetDirection());
  glm::vec3 output = positions_[0] - ray.GetOrigin();
  glm::vec3 barycentric_coords = glm::inverse(A) * output;
  float t = barycentric_coords.z;

  //check if hit_point is in triangle & valid t
  if (barycentric_coords.x < 0.0f || barycentric_coords.y < 0.0f
     || (barycentric_coords.x + barycentric_coords.y) > 1.0f
     || t < t_min || t >= record.time
    ) {
    return false;
  }
  record.time = t;

  // find normal
  record.normal = glm::normalize(
    (1.0f - barycentric_coords.x - barycentric_coords.y) * normals_[0]
    + barycentric_coords.x * normals_[1]
    + barycentric_coords.y * normals_[2]
  );
  return true;
}
}  // namespace GLOO
