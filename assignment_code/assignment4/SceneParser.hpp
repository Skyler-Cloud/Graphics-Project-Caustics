#ifndef SCENE_PARSER_H_
#define SCENE_PARSER_H_

#include <fstream>

#include "gloo/Scene.hpp"
#include "gloo/Material.hpp"
#include "gloo/components/CameraComponent.hpp"

#include "CubeMap.hpp"
#include "CameraSpec.hpp"

namespace GLOO {

class SceneParser {
 public:
  SceneParser();
  std::unique_ptr<Scene> ParseScene(const std::string& filename);
  glm::vec3 GetBackgroundColor() const {
    return background_.color;
  }
  const CubeMap* GetCubeMapPtr() const {
    return background_.cube_map.get();
  }
  const CameraSpec& GetCameraSpec() const {
    return camera_spec_;
  }
  glm::vec3 GetFogColor() const {
    return fog_.color;
  }
  float GetFogDensity() const {
    return fog_.density;
  }
  float GetFogOpacity() const {
    return fog_.opacity;
  }

 private:
  void ParseBackground();
  void ParseMaterials();
  void ParseFog();

  std::shared_ptr<Material> ParseMaterial();
  std::unique_ptr<SceneNode> ParseSceneNode();
  void ParseTransform(Transform& transform);
  void ParseComponent(const std::string& type, SceneNode& node);
  void ParseCamera();
  void ParseLightComponent(SceneNode& node);
  void ParseMaterialComponent(SceneNode& node);
  void ParseTracingComponent(SceneNode& node);
  void Assert(const std::string& token, const std::string& expected);

  float ReadFloat();
  int ReadInt();
  glm::vec3 ReadVec3();

  std::vector<std::shared_ptr<Material>> materials_;

  struct {
    glm::vec3 color;
    glm::vec3 ambient_light;
    std::unique_ptr<CubeMap> cube_map;
  } background_;

  struct {
    glm::vec3 color;
    float density; // extinction coefficient, see https://en.wikipedia.org/wiki/Beer%E2%80%93Lambert_law
    float opacity; // idk if this is realistic, but I want my fog to be thinner,
      //so I clamp the maximum amount of fog to this value:
      //eg: opacity .25 means there will be at most 25% fog color.
  } fog_;

  CameraSpec camera_spec_;

  std::fstream fs_;
  std::string base_path_;
};
}  // namespace GLOO

#endif
