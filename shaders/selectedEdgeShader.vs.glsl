#version 330 core
layout(location = 0) in vec3 aPos;

uniform mat4 projection_matrix;
uniform mat4 view_matrix;
uniform mat4 model_matrix;
uniform vec3 start_pos;
uniform vec3 end_pos;

void main() {
    vec3 position = mix(start_pos, end_pos, aPos.x);
    gl_Position = projection_matrix * view_matrix * model_matrix * vec4(position, 1.0);
}
