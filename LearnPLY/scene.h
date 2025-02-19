#pragma once

#include "miscellaneous/camera.h"
#include "model.h"
#include <memory>
#include <vector>

class Scene {
public:
  // Constants
  static constexpr float PI = 3.14159265358979323846f;
  static constexpr float deg2rad(float degrees) { return degrees * (PI / 180.0f); }

  // Constructor
  Scene(std::unique_ptr<Model> model, int width, int height);

  // Resize
  void resize(int width, int height);

  // Getters
  Camera *getCamera() const { return camera.get(); }
  Model *getModel() const { return model.get(); }

  // Setters
  void setDrawWireframe(bool draw) { drawWireframe = draw; }
  void setDrawFaceNormal(bool draw) { drawFaceNormal = draw; }

  // State queries
  bool getDrawWireframe() const { return drawWireframe; }
  bool getDrawFaceNormal() const { return drawFaceNormal; }
  bool getDrawVertexLabel() const { return drawVertexLabel; }

  // Visualization states
  bool drawWireframe;
  bool drawFaceNormal;
  bool drawVertexLabel;
  int selectedModel;
  std::vector<std::string> modelNames;

  bool loadModel(int index);
  bool loadModel(const std::string &modelPath);

private:
  void setupCamera();
  void resetVis();

  std::unique_ptr<Model> model;
  std::unique_ptr<Camera> camera;

  int screenWidth;
  int screenHeight;

};
