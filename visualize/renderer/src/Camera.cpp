#include "Camera.h"
#include "Renderer.h"

Camera::Camera(int screenWidth, int screenHeight, glm::vec3 position, float rotX, uint tabCount):
	screenWidth(screenWidth), screenHeight(screenHeight),
	Position(position),
	num_tabs(tabCount)
{
	// Calculates upcoming vertical change in the Orientation
	glm::vec3 newOrientation = glm::rotate(Orientation, glm::radians(-rotX), glm::normalize(glm::cross(Orientation, Up)));

	// Decides whether or not the next vertical Orientation is legal or not
	if (fabsf(glm::angle(newOrientation, Up) - glm::radians(90.0f)) <= glm::radians(85.0f))
	{
		Orientation = newOrientation;
	}
}

void Camera::updateMatrix(float FOVdeg, float nearPlane, float farPlane)
{	
	// Initializes matrices since otherwise they will be the null matrix
	glm::mat4 view = glm::mat4(1.0f);
	glm::mat4 projection = glm::mat4(1.0f);

	// Makes camera look in the right direction from the right position
	view = glm::lookAt(Position, Position + Orientation, Up);
	// Adds perspective to the scene
	projection = glm::perspective(glm::radians(FOVdeg), (float)screenWidth / screenHeight, nearPlane, farPlane);

	// Sets new camera matrix
	cameraMatrix = projection * view;
}

void Camera::Matrix(Shader& shader, const char* uniform)
{
	// Exports camera matrix
	glUniformMatrix4fv(glGetUniformLocation(shader.ID, uniform), 1, GL_FALSE, glm::value_ptr(cameraMatrix));
}

void Camera::SetUniforms(Shader& shader) const
{
	// Exports camera matrix
	shader.SetUniformMatrix4fv("camMatrix", cameraMatrix);
}

void Camera::UpdateCameraPosition(const InputState& state)
{
	if(state.windowWidth != screenWidth || state.windowHeight != screenHeight) {
		screenWidth = state.windowWidth;
		screenHeight = state.windowHeight;
	}

	// Handles movement key inputs
	if (state.isWPressed)
	{
		Position += speed * Orientation;
	}
	if (state.isAPressed)
	{
		Position += speed * -glm::normalize(glm::cross(Orientation, Up));
	}
	if (state.isSPressed)
	{
		Position += speed * -Orientation;
	}
	if (state.isDPressed)
	{
		Position += speed * glm::normalize(glm::cross(Orientation, Up));
	}
	if (state.isSpacePressed)
	{
		Position += speed * Up;
	}
	if (state.isLeftCtrlPressed)
	{
		Position += speed * -Up;
	}
	if (state.isLeftShiftPressed || state.isRightShiftPressed)
	{
		speed = 0.4f;
	} else if(state.isLeftAltPressed) {
		speed = 0.01f;
	} else {
		speed = 0.1f;
	}
	
	// Handles mouse inputs
	if (state.isLeftMousePressed)
	{
		// Normalizes and shifts the coordinates of the cursor such that they begin in the middle of the screen
		// and then "transforms" them into degrees 
		float rotX = sensitivity * (float)(state.mouseY - (screenHeight / 2)) / screenHeight;
		float rotY = sensitivity * (float)(state.mouseX - (screenWidth / 2)) / screenWidth;

		// Calculates upcoming vertical change in the Orientation
		glm::vec3 newOrientation = glm::rotate(Orientation, glm::radians(-rotX), glm::normalize(glm::cross(Orientation, Up)));

		// Decides whether or not the next vertical Orientation is legal or not
		if (abs(glm::angle(newOrientation, Up) - glm::radians(90.0f)) <= glm::radians(85.0f))
		{
			Orientation = newOrientation;
		}

		// Rotates the Orientation left and right
		Orientation = glm::rotate(Orientation, glm::radians(-rotY), Up);
	}

	updateMatrix(FOV, nearPlane, farPlane);
}