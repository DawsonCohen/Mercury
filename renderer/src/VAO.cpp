#include "VAO.h"
#include "Renderer.h"
#include <iostream>

// Constructor that generates a VAO ID
VAO::VAO()
{
	// GLCall(glGenVertexArrays(1, &ID));
}

VAO::~VAO() {

}

void VAO::GenerateID() {
	GLCall(glGenVertexArrays(1, &ID));
}

// Links a VBO Attribute such as a position or color to the VAO
void VAO::LinkAttrib(const VBO& VBO, GLuint layout, GLuint numComponents, GLenum type, GLsizeiptr stride, void* offset) const
{
	VBO.Bind();
	GLCall(glEnableVertexAttribArray(layout));
	GLCall(glVertexAttribPointer(layout, numComponents, type, GL_FALSE, stride, offset));
}

// Binds the VAO
void VAO::Bind() const
{
	GLCall(glBindVertexArray(ID));
}

// Unbinds the VAO
void VAO::Unbind() const
{
	GLCall(glBindVertexArray(0));
}

void VAO::Delete() const
{
	GLCall(glDeleteVertexArrays(1, &ID));
}
