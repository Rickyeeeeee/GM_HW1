#pragma once
#include <iostream>
#include <map>
#include <string>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <Eigen/Geometry>
#include <filesystem>

// FreeType
#include <ft2build.h>
#include FT_FREETYPE_H

struct Character {
	unsigned int		TextureID;  // ID handle of the glyph texture
	Eigen::Vector2i		Size;       // Size of glyph
	Eigen::Vector2i		Bearing;    // Offset from baseline to left/top of glyph
	unsigned int		Advance;    // Offset to advance to next glyph
};


class font_atlas
{
private:
	std::map<char, Character> m_characters;
	GLuint VAO, VBO_POS, VBO_TEX;

public:

	font_atlas();
	~font_atlas();
	void create_atlas(const char* filename); // Function to create the atlas
	void Bind_atlas();
	void UnBind_atlas();

	void Render(std::string text);
};

