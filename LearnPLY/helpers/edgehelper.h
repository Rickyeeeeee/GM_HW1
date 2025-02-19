#pragma once
#include "../miscellaneous/shader.h"
#include <Eigen/Dense>
#include <glad/glad.h>
#include <vector>

class EdgeHelper {
public:
  EdgeHelper();
  ~EdgeHelper();

  void initialize(Shader* edge_shader);
  void draw(const Eigen::Matrix4f &projectionMatrix, const Eigen::Matrix4f &viewMatrix,
            const Eigen::Vector3f &start, const Eigen::Vector3f &end);

private:
  GLuint VAO, VBO;
  Shader *edgeShader;

  void createLine();
};
