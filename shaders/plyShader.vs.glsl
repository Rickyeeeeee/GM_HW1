#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;

uniform mat4 projection_matrix;
uniform mat4 view_matrix;
uniform mat4 model_matrix;

out vec3 fNormal;
out vec3 fColor;
out vec3 fPosition;

void main() {
    // Calculate vertex position
    gl_Position = projection_matrix * view_matrix * model_matrix * vec4(aPos, 1.0);

    fPosition = vec3(model_matrix * vec4(aPos, 1.0));

    // Transform normal to world space
    // Note: Using the transpose of the inverse of the model matrix for correct normal transformation
    mat3 normalMatrix = transpose(inverse(mat3(model_matrix)));
    fNormal = normalize(normalMatrix * aNormal);

    // Pass color to fragment shader
    fColor = aColor;
}
