#include "scene.h"
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <glad/glad.h>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>

Scene::Scene(std::unique_ptr<Model> model, int width, int height)
    : model(std::move(model)), screenWidth(width), screenHeight(height), selectedModel(0) {
  setupCamera();
  resetVis();
  // Scan models
  std::filesystem::path modelPath("../model3d");
  if (std::filesystem::exists(modelPath) && std::filesystem::is_directory(modelPath)) {
      for (const auto& entry : std::filesystem::directory_iterator(modelPath)) {
          if (entry.path().extension() == ".ply") {
              modelNames.push_back(entry.path().filename().string());
          }
      }
  }
  std::sort(modelNames.begin(), modelNames.end());
}


void Scene::setupCamera() {
  camera = std::make_unique<Camera>();
  camera->setViewport(screenWidth, screenHeight);
  camera->setPosition(Eigen::Vector3f(0, 0, 5.0f));
  camera->setTarget(Eigen::Vector3f(0, 0, 0));
  camera->setFovY(deg2rad(45.0f));
}

void Scene::resetVis()
{
    drawWireframe = false;
    drawFaceNormal = false;
    drawVertexLabel = false;
}

void Scene::resize(int width, int height) {
  screenWidth = width;
  screenHeight = height;
  camera->setViewport(width, height);
}

bool Scene::loadModel(int index)
{
    if (modelNames.empty()){
        return false;
    }
    selectedModel = index;
    std::string modelPath = "../model3d/" + modelNames[index];
    return loadModel(modelPath);
}

bool Scene::loadModel(const std::string &modelPath) {
  try {
    auto newModel = std::make_unique<Model>(modelPath);
    model = std::move(newModel);
    return true;
  } catch (const std::exception &e) {
    std::cerr << "Error loading model: " << e.what() << std::endl;
    return false;
  }
}
