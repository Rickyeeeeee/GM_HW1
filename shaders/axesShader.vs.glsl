#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 viewport;

out vec3 Color;

void main() {
		Color = aColor;
		gl_Position = viewport * projection * view * vec4(aPos, 1.0);
}