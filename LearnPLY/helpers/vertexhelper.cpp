#include "vertexhelper.h"

#define M_PI 3.14159265358979323846f

VertexHelper::VertexHelper()
    : sphereShader(nullptr)
    , textShader(nullptr)
    , VAO(0)
    , VBO(0)
    , EBO(0)
{
}

VertexHelper::~VertexHelper() {
    delete sphereShader;
    delete textShader;

    if (VAO) glDeleteVertexArrays(1, &VAO);
    if (VBO) glDeleteBuffers(1, &VBO);
    if (EBO) glDeleteBuffers(1, &EBO);
}

void VertexHelper::initialize(Shader* sphere_shader, Shader* text_shader) {
    sphereShader = sphere_shader;
    textShader = text_shader;
    createSphere();
    textRenderer = std::make_unique<font_atlas>();
    textRenderer->create_atlas("../shaders/BitterPro-Bold.ttf");
}

void VertexHelper::createSphere(float radius, int segments, int rings) {
    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    // Generate vertices
    for (int ring = 0; ring <= rings; ring++) {
        float phi = M_PI * float(ring) / float(rings);
        for (int segment = 0; segment <= segments; segment++) {
            float theta = 2.0f * M_PI * float(segment) / float(segments);

            // Calculate vertex position
            float x = radius * sin(phi) * cos(theta);
            float y = radius * cos(phi);
            float z = radius * sin(phi) * sin(theta);

            // Calculate normal
            float nx = sin(phi) * cos(theta);
            float ny = cos(phi);
            float nz = sin(phi) * sin(theta);

            // Add vertex position and normal
            vertices.push_back(x);
            vertices.push_back(y);
            vertices.push_back(z);
            vertices.push_back(nx);
            vertices.push_back(ny);
            vertices.push_back(nz);
        }
    }

    // Generate indices
    for (int ring = 0; ring < rings; ring++) {
        for (int segment = 0; segment < segments; segment++) {
            unsigned int current = ring * (segments + 1) + segment;
            unsigned int next = current + segments + 1;

            indices.push_back(current);
            indices.push_back(next);
            indices.push_back(current + 1);

            indices.push_back(next);
            indices.push_back(next + 1);
            indices.push_back(current + 1);
        }
    }

    // Setup OpenGL buffers
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // Normal attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    vertexCount = vertices.size();
    indicesCount = indices.size();
}

void VertexHelper::draw(const Eigen::Matrix4f& projectionMatrix,
                       const Eigen::Matrix4f& viewMatrix,
                       const Eigen::Vector3f& translation) {
    // Sphere rendering
    sphereShader->use();
    sphereShader->setVec3("color", 1.0f, 0.0f, 0.0f);
    sphereShader->setMat4("projection_matrix", projectionMatrix);
    sphereShader->setMat4("view_matrix", viewMatrix);
    Eigen::Matrix4f model;
    model << 1.0f, 0.0f, 0.0f, translation.x(),
             0.0f, 1.0f, 0.0f, translation.y(),
             0.0f, 0.0f, 1.0f, translation.z(),
             0.0f, 0.0f, 0.0f, 1.0f;
    sphereShader->setMat4("model_matrix", model);
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, (GLsizei)indicesCount, GL_UNSIGNED_INT, 0);
}

void VertexHelper::drawLabel(const Eigen::Matrix4f& projectionMatrix, const Eigen::Matrix4f& viewMatrix, const Eigen::Vector3f& translation, const Eigen::Vector3f& normal, const char* label)
{
    // Text rendering
    textShader->use();
    textShader->setMat4("projection_matrix", projectionMatrix);
    textShader->setMat4("view_matrix", viewMatrix);
    textShader->setVec3("normal", normal);
    textShader->setVec3("translation", translation);
    textRenderer->Render(label);
}
