#include "edgehelper.h"

EdgeHelper::EdgeHelper() : VAO(0), VBO(0), edgeShader(nullptr) {}

EdgeHelper::~EdgeHelper() {
  delete edgeShader;
  if (VAO != 0)
    glDeleteVertexArrays(1, &VAO);
  if (VBO != 0)
    glDeleteBuffers(1, &VBO);
}

void EdgeHelper::initialize(Shader* edge_shader) {
  edgeShader = edge_shader;
  // Create a basic line from (0,0,0) to (1,0,0)
  // Will be transformed to actual positions in the shader
  std::vector<float> vertices({
      0.0f, 0.0f, 0.0f, // start point
      1.0f, 0.0f, 0.0f  // end point
  });

  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);

  glBindVertexArray(VAO);
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(0);
}

void EdgeHelper::draw(const Eigen::Matrix4f &projectionMatrix, const Eigen::Matrix4f &viewMatrix,
                      const Eigen::Vector3f &start, const Eigen::Vector3f &end) {
  edgeShader->use();
  edgeShader->setMat4("projection_matrix", projectionMatrix);
  edgeShader->setMat4("view_matrix", viewMatrix);
  edgeShader->setMat4("model_matrix", Eigen::Matrix4f::Identity());
  edgeShader->setVec3("start_pos", start);
  edgeShader->setVec3("end_pos", end);

  glBindVertexArray(VAO);
  glLineWidth(3.0f);
  glDrawArrays(GL_LINES, 0, 2);
}
