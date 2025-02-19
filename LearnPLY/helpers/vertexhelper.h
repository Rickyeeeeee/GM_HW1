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
  void draw(const Eigen::Matrix4f &projectionMatrix, const Eigen::Matrix4f &viewMatrix,
            const Eigen::Vector3f &translation);
  void drawLabel(const Eigen::Matrix4f& projectionMatrix, const Eigen::Matrix4f& viewMatrix,
                 const Eigen::Vector3f& translation, const Eigen::Vector3f& normal, const char* label);

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
