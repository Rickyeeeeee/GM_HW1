#version 330 core

in vec3 fNormal;
in vec3 fColor;
in vec3 fPosition;

uniform vec3 view_dir;
out vec4 fragColor;

// Hemisphere lighting calculation
vec3 calculateHemisphereLight(vec3 normal, vec3 skyColor, vec3 groundColor) {
    float hemiMix = 0.5 * (normal.y + 1.0);
    return mix(groundColor, skyColor, hemiMix);
}

// Directional light calculation
vec3 calculateDirectionalLight(vec3 normal, vec3 lightDir, vec3 color) {
    float lightStrength = 1.0;
    float diff = max(dot(normal, -lightDir), 0.0);
    return diff * color * lightStrength;
}

void main() {
    vec3 N = normalize(fNormal);

    // Hemisphere lighting
    vec3 skyColor = vec3(0.7, 0.8, 1.0);
    vec3 groundColor = vec3(0.3, 0.2, 0.1);
    vec3 hemiLight = calculateHemisphereLight(N, skyColor, groundColor);

    // Directional light that follows camera
    vec3 dirLight = calculateDirectionalLight(N, view_dir, fColor);

    // Combine lighting
    vec3 result = (hemiLight + dirLight) * fColor;

    fragColor = vec4(result, 1.0);
}
