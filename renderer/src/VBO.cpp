#include"VBO.h"
#include "Renderer.h"

// Constructor that generates a Vertex Buffer Object and links it to vertices
VBO::VBO()
{}

VBO::VBO(const std::vector<Vertex>& vertices)
{
	GLCall(glGenBuffers(1, &ID));
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, ID));
	GLCall(glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_DYNAMIC_DRAW));
}

VBO::~VBO() {
	// GLCall(glDeleteBuffers(1, &ID));
}

void VBO::GenerateID() {
	GLCall(glGenBuffers(1, &ID));
}

// Binds the VBO
void VBO::Bind() const
{
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, ID));
}

void VBO::Bind(const std::vector<Vertex>& vertices) const
{
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, ID));
	GLCall(glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_DYNAMIC_DRAW));
}

// Unbinds the VBO
void VBO::Unbind() const
{
	GLCall(glBindBuffer(GL_ARRAY_BUFFER, 0));
}
