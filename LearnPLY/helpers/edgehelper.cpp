#include "edgehelper.h"
#define M_PI 3.14159265358979323846f

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
  createCylinder();
}

void EdgeHelper::use(const Eigen::Matrix4f& projectionMatrix, const Eigen::Matrix4f& viewMatrix)
{
    edgeShader->use();
    edgeShader->setVec3("color", 0.91f, 0.40f, 0.41f);
    edgeShader->setMat4("projection_matrix", projectionMatrix);
    edgeShader->setMat4("view_matrix", viewMatrix);
}

void EdgeHelper::draw(const Eigen::Vector3f &start, const Eigen::Vector3f &end) {

  float len = (end - start).norm();
  Eigen::Vector3f lx = end - start;
  lx.normalize();
  Eigen::Vector3f ly(0.0f, 1.0f, 0.0f);
  if (1.0f - abs(lx.dot(ly)) < 1e-6) { ly << 0.0f, 0.0f, 1.0f; }
  Eigen::Vector3f lz = lx.cross(ly);
  lz.normalize();
  ly = lx.cross(lz);
  ly.normalize();
  //rendering
  Eigen::Matrix4f model;
  model << len * lx.x(), ly.x(), lz.x(), start.x(),
           len * lx.y(), ly.y(), lz.y(), start.y(),
           len * lx.z(), ly.z(), lz.z(), start.z(),
           0.0f, 0.0f, 0.0f, 1.0f;
  edgeShader->setMat4("model_matrix", model);
  glBindVertexArray(VAO);
  glDrawElements(GL_TRIANGLES, (GLsizei)indicesCount, GL_UNSIGNED_INT, 0);
}

void EdgeHelper::createCylinder(float radius, int segments)
{
    if (VAO) glDeleteVertexArrays(1, &VAO);
    if (VBO) glDeleteBuffers(1, &VBO);
    if (EBO) glDeleteBuffers(1, &EBO);
    std::vector<float> pointXs({ 0.0f,1.0f });
    std::vector<float> vertices;
    std::vector<float> normals;
    std::vector<unsigned int> indices;
    // Make the points
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < segments; j++)
        {
            float theta = 2.0f * M_PI * float(j) / float(segments);
            vertices.push_back(pointXs[i]);
            vertices.push_back(cos(theta) * radius);
            vertices.push_back(sin(theta) * radius);
        }
    }
    // Setup the indices
    for (int i = 1; i < 2; i++) {
        for (int j = 0; j < segments; j++) {
            int p0 = (i - 1) * segments + (j - 1 + segments) % segments;
            int p1 = (i - 1) * segments + j;
            int p2 = i * segments + (j - 1 + segments) % segments;
            int p3 = i * segments + j;

            indices.push_back(p1);
            indices.push_back(p0);
            indices.push_back(p2);

            indices.push_back(p1);
            indices.push_back(p2);
            indices.push_back(p3);
        }
    }
    // Setup VAO, VBO, EBO
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    glBindVertexArray(0);

    indicesCount = indices.size();
}
