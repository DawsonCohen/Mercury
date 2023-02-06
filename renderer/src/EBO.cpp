#include "EBO.h"
#include "Renderer.h"

// Constructor that generates a Elements Buffer Object and links it to indices
EBO::EBO()
{}

EBO::EBO(std::vector<unsigned int>& indices) : mCount(indices.size())
{
	GLCall(glGenBuffers(1, &ID));
	GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ID));
	GLCall(glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_DYNAMIC_DRAW));
}

EBO::~EBO() {
	// GLCall(glDeleteBuffers(1, &ID));
}

void EBO::GenerateID() {
	GLCall(glGenBuffers(1, &ID));
}

// Binds the EBO
void EBO::Bind() const
{
	GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ID));
}

void EBO::Bind(const std::vector<unsigned int>& indices) const
{
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, ID));
	GLCall(glBufferData(GL_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_DYNAMIC_DRAW));
}

// Unbinds the EBO
void EBO::Unbind() const
{
	GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
}
