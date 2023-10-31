#ifndef __VBO__H__
#define __VBO__H__

#include <glm/glm.hpp>
#include <glad/glad.h>
#include <vector>

// Structure to standardize the vertices used in the meshes
struct Vertex
{
	glm::vec3 position;
	// glm::vec3 normal;
	glm::vec4 color;
	// glm::vec2 texUV;
};

class VBO
{
  /**
   *  @name Vertex Buffer Object
   */
private:
	GLuint ID;
	
public:
	// Constructor that generates a Vertex Buffer Object and links it to vertices
	VBO();
	VBO(const VBO& src): ID(src.ID) {}
	VBO(const std::vector<Vertex>& vertices);
	~VBO();

	// Generates VBO ID
	void GenerateID();
	// Binds the VBO
	void Bind() const;
	void Bind(const std::vector<Vertex>& vertices) const;
	// Unbinds the VBO
	void Unbind() const;
};

#endif