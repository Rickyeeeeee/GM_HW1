#include "meshrenderer.h"

MeshRenderer::MeshRenderer() { }

MeshRenderer::~MeshRenderer() {
  glDeleteVertexArrays(1, &VAO);
  glDeleteBuffers(1, &VBO);
  glDeleteBuffers(1, &NBO);
  glDeleteBuffers(1, &CBO);
  glDeleteVertexArrays(1, &wireVAO);
  glDeleteBuffers(1, &wireVBO);
}

void MeshRenderer::setupBuffers(Polyhedron *poly) {
  const auto &triangles = MeshProcessor::getTriangles(poly);
  const auto &edges = MeshProcessor::getEdges(poly);

  const size_t vertexDataSize = triangles.size() * 3 * 3;
  std::vector<float> vertexData(vertexDataSize);
  std::vector<float> vertexNormalData(vertexDataSize);
  std::vector<float> colorData(vertexDataSize);

  // Process triangles
  size_t idx = 0;
  for (size_t i = 0; i < triangles.size(); i++) {
    const Triangle *tri = triangles[i];

    // Add vertex data for each vertex in triangle
    for (int j = 0; j < 3; j++) {
      const Vertex *vertex = tri->verts[j];

      // Position
      vertexData[idx] = (float)vertex->pos.x();
      vertexData[idx + 1] = (float)vertex->pos.y();
      vertexData[idx + 2] = (float)vertex->pos.z();

      // Vertex normal
      Eigen::Vector3f vertNormal = vertex->normal.cast<float>();
      vertexNormalData[idx] = (float)vertNormal.x();
      vertexNormalData[idx + 1] = (float)vertNormal.y();
      vertexNormalData[idx + 2] = (float)vertNormal.z();

      // Color
      colorData[idx] = vertex->color.x();
      colorData[idx + 1] = vertex->color.y();
      colorData[idx + 2] = vertex->color.z();

      idx += 3;
    }
  }
  vertexCount = vertexData.size() / 3;

  { // Setup main buffers
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &NBO);
    glGenBuffers(1, &CBO);
    glBindVertexArray(VAO);

    // Position attribute (location = 0)
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float), vertexData.data(),
                 GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);

    // Normal attribute (location = 1)
    glBindBuffer(GL_ARRAY_BUFFER, NBO);
    glBufferData(GL_ARRAY_BUFFER, vertexNormalData.size() * sizeof(float), vertexNormalData.data(),
                 GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);

    // Color attribute (location = 2)
    glBindBuffer(GL_ARRAY_BUFFER, CBO);
    glBufferData(GL_ARRAY_BUFFER, colorData.size() * sizeof(float), colorData.data(),
                 GL_STATIC_DRAW);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(2);
  }

  idx = 0;
  const size_t edgeDataSize = edges.size() * 2 * 3;
  std::vector<float> edgeData(edgeDataSize);
  for (const Edge *edge : edges) {
    edgeData[idx] = (float)edge->verts[0]->pos.x();
    edgeData[idx + 1] = (float)edge->verts[0]->pos.y();
    edgeData[idx + 2] = (float)edge->verts[0]->pos.z();

    edgeData[idx + 3] = (float)edge->verts[1]->pos.x();
    edgeData[idx + 4] = (float)edge->verts[1]->pos.y();
    edgeData[idx + 5] = (float)edge->verts[1]->pos.z();

    idx += 6;
  }
  edgeCount = edgeData.size() / 3;

  { // Create and setup wireframe buffers
    glGenVertexArrays(1, &wireVAO);
    glGenBuffers(1, &wireVBO);

    glBindVertexArray(wireVAO);
    glBindBuffer(GL_ARRAY_BUFFER, wireVBO);
    glBufferData(GL_ARRAY_BUFFER, edgeData.size() * sizeof(float), edgeData.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
  }

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}

void MeshRenderer::drawPLY()
{
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, (GLsizei)vertexCount);
    glBindVertexArray(0);
}

void MeshRenderer::drawWireframe()
{
    glBindVertexArray(wireVAO);
    glLineWidth(2.0f);
    glDrawArrays(GL_LINES, 0, (GLsizei)edgeCount);
    glBindVertexArray(0);
}

void MeshRenderer::updateColors(Polyhedron *poly) {
  const auto& triangles = MeshProcessor::getTriangles(poly);
  std::vector<float> colorData(triangles.size() * 3 * 3, 0);

  size_t idx = 0;
  for (const Triangle* tri : triangles) {

      if (tri->selected) {
          for (int j = 0; j < 3; j++) {
              colorData[idx] = 0.0f;
              colorData[idx + 1] = 0.0f;
              colorData[idx + 2] = 1.0f;
              idx += 3;
          }
      }
      else
      {
          for (int j = 0; j < 3; j++) {
              const Vertex* vertex = tri->verts[j];
              colorData[idx] = vertex->color.x();
              colorData[idx + 1] = vertex->color.y();
              colorData[idx + 2] = vertex->color.z();
              idx += 3;
          }
      }
  }

  glBindBuffer(GL_ARRAY_BUFFER, CBO);
  glBufferSubData(GL_ARRAY_BUFFER, 0, colorData.size() * sizeof(float), colorData.data());
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void MeshRenderer::setNormalMode(Polyhedron *poly, bool using_face_normal) {

  const auto &triangles = MeshProcessor::getTriangles(poly);

  glBindBuffer(GL_ARRAY_BUFFER, NBO);
  if (!using_face_normal) {
    size_t vertexDataSize = triangles.size() * 3 * 3;
    std::vector<float> vertexNormalData(vertexDataSize);

    size_t idx = 0;
    for (size_t i = 0; i < triangles.size(); i++) {
      const Triangle *tri = triangles[i];
      for (int j = 0; j < 3; j++) {
        const Vertex *vertex = tri->verts[j];
        Eigen::Vector3f vertNormal = vertex->normal.cast<float>();
        vertexNormalData[idx] = vertNormal.x();
        vertexNormalData[idx + 1] = vertNormal.y();
        vertexNormalData[idx + 2] = vertNormal.z();
        idx += 3;
      }
    }

    glBufferSubData(GL_ARRAY_BUFFER, 0, vertexNormalData.size() * sizeof(float),
                    vertexNormalData.data());
  } else {
    size_t vertexDataSize = triangles.size() * 3 * 3;
    std::vector<float> faceNormalData(vertexDataSize);

    size_t idx = 0;
    for (size_t i = 0; i < triangles.size(); i++) {
      const Triangle *tri = triangles[i];

      // Calculate face normal
      Eigen::Vector3f v1 = (tri->verts[1]->pos - tri->verts[0]->pos).cast<float>();
      Eigen::Vector3f v2 = (tri->verts[2]->pos - tri->verts[0]->pos).cast<float>();
      Eigen::Vector3f faceNormal = v1.cross(v2).normalized();

      for (int j = 0; j < 3; j++) {
        faceNormalData[idx] = faceNormal.x();
        faceNormalData[idx + 1] = faceNormal.y();
        faceNormalData[idx + 2] = faceNormal.z();
        idx += 3;
      }
    }

    glBufferSubData(GL_ARRAY_BUFFER, 0, faceNormalData.size() * sizeof(float),
                    faceNormalData.data());
  }
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void MeshRenderer::setColors(Polyhedron* poly, int mode)
{
    if (mode == 1)
    {
        const float L = 0.05f;
        for (auto& vertex : poly->vlist) {
            float r = (int(vertex->pos.x() / L) % 2 == 0) ? 1.0f : 0.0f;
            float g = (int(vertex->pos.y() / L) % 2 == 0) ? 1.0f : 0.0f;
            float b = (int(vertex->pos.z() / L) % 2 == 0) ? 1.0f : 0.0f;
            vertex->color = Eigen::Vector3f(r, g, b);
        }
    }
    else
    {
        for (auto& vertex : poly->vlist) {
            vertex->color = Eigen::Vector3f(0.58f, 0.72f, 0.75f);
        }
    }
    updateColors(poly);
}
