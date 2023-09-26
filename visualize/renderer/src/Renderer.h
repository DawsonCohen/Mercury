#ifndef __RENDERER_H__
#define __RENDERER_H__

#include <glad/glad.h>
#include <csignal>
#include "VAO.h"
#include "VBO.h"
#include "EBO.h"
#include "Camera.h"

#define ASSERT(x) if (!(x)) std::raise(SIGINT);
#define GLCall(x) GLClearError();\
	x;\
	ASSERT(GLLogCall(#x,__FILE__,__LINE__))

void GLClearError();
bool GLLogCall(const char* function, const char* file, int line);

class Renderer {
	void Clear() const;
	
	void Draw(const VAO& VAO, const EBO& EBO, const Shader& shader) const;
	void DrawLines(const VAO& VAO, const EBO& EBO, const Shader& shader) const;
};

#endif