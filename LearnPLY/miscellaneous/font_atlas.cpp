#include "font_atlas.h"

font_atlas::font_atlas() : VAO(0)
{
}

font_atlas::~font_atlas()
{
}

void font_atlas::create_atlas(const char * filename)
{
	// FreeType
	// --------
	FT_Library ft;
	// All functions return a value different than 0 whenever an error occurred
	if (FT_Init_FreeType(&ft))
	{
		std::cout << "ERROR::FREETYPE: Could not init FreeType Library" << std::endl;
		return;
	}

	// load font as face
	FT_Face face;
	if (FT_New_Face(ft, filename, 0, &face)) {
		std::cout << "ERROR::FREETYPE: Failed to load font" << std::endl;
		return;
	}

	FT_Set_Pixel_Sizes(face, 0, 48);
	if (FT_Load_Char(face, 'X', FT_LOAD_RENDER))
	{
		std::cout << "ERROR::FREETYTPE: Failed to load Glyph" << std::endl;
		return;
	}

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1); // disable byte-alignment restriction

	for (unsigned char c = 0; c < 128; c++)
	{
		// load character glyph 
		if (FT_Load_Char(face, c, FT_LOAD_RENDER))
		{
			std::cout << "ERROR::FREETYTPE: Failed to load Glyph" << std::endl;
			continue;
		}
		// generate texture
		unsigned int texture;
		glGenTextures(1, &texture);
		glBindTexture(GL_TEXTURE_2D, texture);
		glTexImage2D(
			GL_TEXTURE_2D,
			0,
			GL_RED,
			face->glyph->bitmap.width,
			face->glyph->bitmap.rows,
			0,
			GL_RED,
			GL_UNSIGNED_BYTE,
			face->glyph->bitmap.buffer
		);
		// set texture options
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		// now store character for later use
		Character character = {
			texture,
			Eigen::Vector2i(face->glyph->bitmap.width, face->glyph->bitmap.rows),
			Eigen::Vector2i(face->glyph->bitmap_left, face->glyph->bitmap_top),
			face->glyph->advance.x
		};
		m_characters.insert(std::pair<char, Character>(c, character));
	}

	// destroy FreeType once we're finished
	FT_Done_Face(face);
	FT_Done_FreeType(ft);

	// Initialize font VAO & VBO
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO_POS);
	glGenBuffers(1, &VBO_TEX);
	glBindVertexArray(VAO);
	//
	glBindBuffer(GL_ARRAY_BUFFER, VBO_POS);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 2, NULL, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), 0);
	glEnableVertexAttribArray(0);
	//
	glBindBuffer(GL_ARRAY_BUFFER, VBO_TEX);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 2, NULL, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), 0);
	glEnableVertexAttribArray(1);
	//
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void font_atlas::Bind_atlas() {

}

void font_atlas::UnBind_atlas() {

}

void font_atlas::Render(std::string text) {
	const float scale = 0.001f;

	// Bind VAO
	glActiveTexture(GL_TEXTURE0);
	glBindVertexArray(VAO);
	std::string::const_iterator c;

	float init_x = 0;
	// Iterate the text
	for (c = text.begin(); c != text.end(); c++)
	{
		Character ch = m_characters[*c];

		float xpos = init_x + ch.Bearing.x() * scale;
		float ypos = (ch.Size.y() - ch.Bearing.y()) * scale;

		float w = ch.Size.x() * scale;
		float h = ch.Size.y() * scale;
		// update VBO for each character
		float vertices[6][2] = {
			{ xpos,     ypos + h },
			{ xpos,     ypos     },
			{ xpos + w, ypos	 },

			{ xpos,     ypos + h },
			{ xpos + w, ypos	 },
			{ xpos + w, ypos + h }
		};
		float textures[6][2] = {
			{ 0.0f, 0.0f },
			{ 0.0f, 1.0f },
			{ 1.0f, 1.0f },

			{ 0.0f, 0.0f },
			{ 1.0f, 1.0f },
			{ 1.0f, 0.0f },
		};
		// render glyph texture over quad
		glBindTexture(GL_TEXTURE_2D, ch.TextureID);
		// update content of VBO vertices
		glBindBuffer(GL_ARRAY_BUFFER, VBO_POS);
		glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
		// Update content of VBO texture
		glBindBuffer(GL_ARRAY_BUFFER, VBO_TEX);
		glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(textures), textures);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		// render quad
		glDrawArrays(GL_TRIANGLES, 0, 6);
		// now advance cursors for next glyph (note that advance is number of 1/64 pixels)
		init_x += (ch.Advance >> 6) * scale; // bitshift by 6 to get value in pixels (2^6 = 64)
	}
	glBindVertexArray(0);
	glBindTexture(GL_TEXTURE_2D, 0);
}