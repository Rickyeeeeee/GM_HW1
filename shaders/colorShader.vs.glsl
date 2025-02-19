#version 330

layout (location = 0) in vec3 aPos;

uniform mat4 projection_matrix;	// projection matrix
uniform mat4 view_matrix;	// camera viewing transformation matrix
uniform mat4 model_matrix;	// transformation matrix

void main() {
	gl_Position = projection_matrix * view_matrix * model_matrix * vec4(aPos, 1.0);
}
