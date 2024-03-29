#ifndef CAMERA_CLASS_H
#define CAMERA_CLASS_H

#include<glad/glad.h>
#include<GLFW/glfw3.h>
#include<glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtc/type_ptr.hpp>
#include<glm/gtx/rotate_vector.hpp>
#include<glm/gtx/vector_angle.hpp>

#include "Shader.h"
#include "visualizerStructs.h"

class Camera
{
private:
	int screenWidth, screenHeight;
	
	// Camera 
	float FOV = 45.0;
	float nearPlane = 0.1f;
	float farPlane = 100.0f;
	
public:
	// Stores the main vectors of the camera
	glm::vec3 Position;
	glm::vec3 Orientation = glm::vec3(0.0f, 0.0f, -1.0f);
	glm::vec3 Up = glm::vec3(0.0f, 1.0f, 0.0f);
	glm::mat4 view = glm::mat4(1.0f);
	glm::mat4 projection = glm::mat4(1.0f);
	glm::mat4 cameraMatrix = glm::mat4(1.0f);

	// Prevents the camera from jumping around when first clicking left click
	bool firstClick = true;

	// Adjust the speed of the camera and it's sensitivity when looking around
	uint tabIdx = 0;
	uint num_tabs = 1;
	float speed = 0.1f;
	float sensitivity = 5.0f;

	// Camera constructor to set up initial values
	Camera(int screenWidth, int screenHeight, glm::vec3 position = glm::vec3(0.0f, 0.75f, 5.0f), float rotX = 0.0f, uint tabCount = 1);
	Camera() {}

	// Updates the camera matrix to the Vertex Shader
	void updateMatrix(float FOVdeg, float nearPlane, float farPlane);
	void Matrix(Shader& shader, const char* uniform);

	void Tab() {
		tabIdx = (tabIdx + 1) % num_tabs;
	}

	// Exports the camera matrix to a shader
	void SetUniforms(Shader& shader) const;
	// Handles camera inputs
	void UpdateCameraPosition(const InputState& inputState);
};
#endif