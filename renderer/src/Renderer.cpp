#include "Renderer.h"
#include <iostream>
#include <iomanip>

void GLClearError() {
    while(glGetError());
}

bool GLLogCall(const char* function, const char* file, int line) {
    while (GLenum error = glGetError()) {
        std::cout << "[OpenGL Error] (0x" << std::hex << error << "): " << function <<
			" " << file <<	":" << std::dec << line << std::endl;
        return false;
    }
    return true;
}

void Renderer::Clear() const {
	// Specify the color of the background
	GLCall(glClearColor(0.73f, 0.85f, 0.92f, 1.0f));
	// Clean the back buffer and depth buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::DrawLines(const VAO& VAO, const EBO& EBO, const Shader& shader) const {
    // Bind shader to be able to access uniforms
	shader.Bind();
	VAO.Bind();
	EBO.Bind();

	glDrawElements(GL_LINES, EBO.getCount(), GL_UNSIGNED_INT, 0);
}

// void Renderer::Draw(const VAO& VAO, const EBO& EBO, const Shader& shader, const Camera& camera) const {
//     // Bind shader to be able to access uniforms
// 	shader.Bind();
// 	VAO.Bind();
// 	EBO.Bind();


// 	glDrawElements(GL_LINES, EBO.getCount(), GL_UNSIGNED_INT, 0);

// 	// VBO VBO(SoftBody::mVertices);

// 	// VAO.LinkAttrib(VBO, 0, 3, GL_FLOAT, sizeof(Vertex), (void*)0);
// 	// VAO.LinkAttrib(VBO, 1, 3, GL_FLOAT, sizeof(Vertex), (void*)(3 * sizeof(float)));

// 	// // Take care of the camera Matrix
//     // int modelLoc = glGetUniformLocation(shader.ID, "model");
//     // glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(mObjectModel));
// 	// camera.Matrix(shader, "view");

// 	// // Draw the actual mesh
// }
