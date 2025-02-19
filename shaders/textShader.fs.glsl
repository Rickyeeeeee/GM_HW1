#version 330 core
in vec2 TexCoords;
out vec4 color;

uniform sampler2D text;

void main()
{   
    float alpha = texture(text, TexCoords).r;
    if(alpha < 1.0) {
        discard;
    }
    color = vec4(1.0, 0.0, 0.0, 1.0);
}  
