#include <iostream>
#include <chrono>

#include "gloo/Scene.hpp"
#include "gloo/components/MaterialComponent.hpp"

#include "hittable/Sphere.hpp"
#include "Tracer.hpp"
#include "SceneParser.hpp"
#include "ArgParser.hpp"

using namespace GLOO;

int main(int argc, const char* argv[]) {
  ArgParser arg_parser(argc, argv);
  SceneParser scene_parser;
  auto scene = scene_parser.ParseScene("assignment4/" + arg_parser.input_file);

  Tracer tracer(scene_parser.GetCameraSpec(),
                glm::ivec2(arg_parser.width, arg_parser.height),
                arg_parser.bounces, scene_parser.GetBackgroundColor(),
                scene_parser.GetCubeMapPtr(), arg_parser.shadows, arg_parser.jitter, arg_parser.filter,
                scene_parser.GetFogColor(), scene_parser.GetFogDensity(), scene_parser.GetFogOpacity(),
                /* num_photons */ 100000, arg_parser.enable_caustics);
  tracer.Render(*scene, arg_parser.output_file);
  return 0;
}
