#include "axeshelper.h"

AxesHelper::AxesHelper(float size): axesShader(nullptr){
}

AxesHelper::~AxesHelper() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &CBO);
}

void AxesHelper::initialize(Shader* axes_shader, float size)
{
    axesShader = axes_shader;
    setupBuffers(size);
}

void AxesHelper::draw(const Eigen::Matrix4f& viewMatrix, const Eigen::Matrix4f& projectionMatrix, float windowAspectRatio) {
    axesShader->use();

    // Use identity matrices for view and projection
    Eigen::Matrix4f identityView = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f identityProj = Eigen::Matrix4f::Identity();

    // Set up viewport transform for bottom-left corner
    Eigen::Matrix4f viewport = Eigen::Matrix4f::Identity();
    float scale = 0.1f;
    viewport(0,0) = scale;
    viewport(1,1) = scale * windowAspectRatio;  // Adjust Y scale by aspect ratio
    viewport(2,2) = scale;
    viewport(0,3) = -0.85f;  // X translation
    viewport(1,3) = -0.8f;  // Y translation

    // Extract rotation from view matrix to only rotate the axes
    Eigen::Matrix3f rotation = viewMatrix.block<3,3>(0,0);
    Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
    rotationMatrix.block<3,3>(0,0) = rotation;

    // Set uniforms
    axesShader->setMat4("view", rotationMatrix);      // Only apply rotation
    axesShader->setMat4("projection", identityProj);  // Use identity projection
    axesShader->setMat4("viewport", viewport);

    // Enable depth test but always draw axes on top
    glDepthFunc(GL_ALWAYS);

    glBindVertexArray(VAO);
    glDrawArrays(GL_LINES, 0, 6);  // 3 axes, 2 vertices each
    glBindVertexArray(0);

    // Restore depth function
    glDepthFunc(GL_LESS);
}

void AxesHelper::setupBuffers(float size) {
    // Create vertex positions using Eigen
    std::vector<Eigen::Vector3f> vertices = {
        Eigen::Vector3f(0.0f, 0.0f, 0.0f),  // X axis start
        Eigen::Vector3f(size, 0.0f, 0.0f),  // X axis end
        Eigen::Vector3f(0.0f, 0.0f, 0.0f),  // Y axis start
        Eigen::Vector3f(0.0f, size, 0.0f),  // Y axis end
        Eigen::Vector3f(0.0f, 0.0f, 0.0f),  // Z axis start
        Eigen::Vector3f(0.0f, 0.0f, size)   // Z axis end
    };

    // Colors for each axis (RGB)
    std::vector<Eigen::Vector3f> colors = {
        Eigen::Vector3f(1.0f, 0.0f, 0.0f),  // X axis red
        Eigen::Vector3f(1.0f, 0.0f, 0.0f),
        Eigen::Vector3f(0.0f, 1.0f, 0.0f),  // Y axis green
        Eigen::Vector3f(0.0f, 1.0f, 0.0f),
        Eigen::Vector3f(0.0f, 0.0f, 1.0f),  // Z axis blue
        Eigen::Vector3f(0.0f, 0.0f, 1.0f)
    };

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &CBO);

    glBindVertexArray(VAO);

    // Vertex positions
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Eigen::Vector3f),
                vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);

    // Colors
    glBindBuffer(GL_ARRAY_BUFFER, CBO);
    glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(Eigen::Vector3f),
                colors.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}
