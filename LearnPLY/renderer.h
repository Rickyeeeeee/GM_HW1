#pragma once

#include <Eigen/Geometry>
#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include "helpers/axeshelper.h"
#include "helpers/edgehelper.h"
#include "helpers/vertexhelper.h"
#include "learnply.h"
#include "meshprocessor.h"
#include "meshrenderer.h"
#include "miscellaneous/camera.h"
#include "miscellaneous/font_atlas.h"
#include "scene.h"
#include <iostream>
#include <memory>

class Renderer {
public:
  Renderer(int width, int height, Scene *scenePtr);
  ~Renderer();

  void resize(int width, int height);
  void render();

private:
	void setupShaders();
	void releaseShaders();

private:
  int screenWidth;
  int screenHeight;

  // State
  Scene *scene;

  //Shader
  Shader* plyShader;
  Shader* colorShader;
  Shader* textShader;
  Shader* axesShader;

  // Helpers
  std::unique_ptr<VertexHelper> vertexHelper;
  std::unique_ptr<EdgeHelper> edgeHelper;
  std::unique_ptr<AxesHelper> axesHelper;
};
