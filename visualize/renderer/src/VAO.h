#ifndef __VAO_H__
#define __VAO_H__

#include<glad/glad.h>
#include"VBO.h"

class VAO
{
  /**
   *  @name Vertex Array Object
   */

private:
	unsigned int ID;

public:
	// Constructor that generates a VAO ID
	VAO();
	VAO(const VAO& src) : ID(src.ID) {};

	~VAO();

	// Links a VBO Attribute such as a position or color to the VAO
	void LinkAttrib(const VBO& VBO, unsigned int layout, unsigned int numComponents, GLenum type, GLsizeiptr stride, void* offset) const;

	unsigned int getID() const {return ID;}

	// Generates VAO ID
	void GenerateID();
	// Binds the VAO
	void Bind() const;
	// Unbinds the VAO
	void Unbind() const;
	// Deletes the VAO
	void Delete() const;
};

#endif