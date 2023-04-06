#ifndef __PLANE_H__
#define __PLANE_H__

#include "VAO.h"
#include "EBO.h"
#include "Camera.h"

class PlaneModel {
	std::vector<Vertex> mVertices;
	std::vector<GLuint> mIndices;

	float scale;
	glm::vec3 mObjectPos;
	glm::mat4 mObjectModel;
	// glm::mat4 projection;

	class VAO VAO;

	public:
		// Initializes the RigidBody
		PlaneModel(float scale=100, glm::vec3 objectPos = glm::vec3(0.0f));
		void rotate(float deg, glm::vec3 axis);
		void translate(glm::vec3 translation);

		// Draws the RigidBody
		void Draw(Shader& shader, Camera& camera);
};

#endif