#version 330 core
layout (location = 0) in vec2 vertex;
layout (location = 1) in vec2 texture_coord;

out vec2 TexCoords;

uniform mat4 projection_matrix;
uniform mat4 view_matrix;

// Selected Vertex Normal & Translation (a.k.a. Position)
uniform vec3 normal;
uniform vec3 translation;


void main()
{
    // tangent (T)
    vec3 up = vec3(0.0, 1.0, 0.0);
    vec3 tangent = normalize(cross(normal, up));
    if (length(tangent) < 1e-6) { 
        tangent = normalize(cross(normal, vec3(1.0, 0.0, 0.0))); 
    }

    // bitangent (B)
    vec3 bitangent = normalize(cross(normal, tangent));

    // TBN = [T B N], 3x3 matrix for rotation
    mat3 TBN = mat3(tangent, bitangent, normal);

    // rotate
    vec3 r_vertex = TBN * vec3(vertex, 0.0);

    // final pos
    vec3 pos = r_vertex + translation + normal * 0.01;

    // apply final pos
    gl_Position = projection_matrix * view_matrix * vec4(pos, 1.0);

    TexCoords = texture_coord;
}
