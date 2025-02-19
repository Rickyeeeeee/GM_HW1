#pragma once
#include "../miscellaneous/Shader.h"
#include <Eigen/Dense>
#include <glad/glad.h>
#include <vector>

class AxesHelper {
public:
  AxesHelper(float size = 10.0f);
  ~AxesHelper();

  void initialize(Shader* axes_shader, float size);
  void draw(const Eigen::Matrix4f &viewMatrix, const Eigen::Matrix4f &projectionMatrix,
            float windowAspectRatio);

private:
  void setupBuffers(float size);

  Shader* axesShader;
  GLuint VAO, VBO, CBO;
};
