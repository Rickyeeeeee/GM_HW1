#pragma once
#include "../miscellaneous/font_atlas.h"
#include "../miscellaneous/shader.h"
#include <Eigen/Dense>
#include <vector>

class VertexHelper {
public:
  VertexHelper();
  ~VertexHelper();

  void initialize(Shader* sphere_shader, Shader* text_shader);
  void use(const Eigen::Matrix4f& projectionMatrix, const Eigen::Matrix4f& viewMatrix);
  void draw(const Eigen::Vector3f &pos);
  void useLabel(const Eigen::Matrix4f& projectionMatrix, const Eigen::Matrix4f& viewMatrix);
  void drawLabel(const Eigen::Vector3f& pos, const Eigen::Vector3f& normal, const char* label);

private:
  // OpenGL buffers
  unsigned int VAO, VBO, EBO;
  size_t vertexCount;
  size_t indicesCount;

  std::unique_ptr<font_atlas> textRenderer;
  Shader *sphereShader;
  Shader *textShader;

  void createSphere(float radius = 0.01f, int segments = 16, int rings = 16);
};
