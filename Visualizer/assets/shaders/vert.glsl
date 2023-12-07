#version 330 core

// Positions/Coordinates
layout (location = 0) in vec3 aPos;
// Colors
layout (location = 1) in vec4 aColor;

// Outputs the color for the Fragment Shader
out vec4 color;

// Imports the camera matrix from the main function
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat4 camMatrix;

void main()
{
	// Outputs the positions/coordinates of all vertices
	// gl_Position = projection * view * model * vec4(aPos, 1.0);
	gl_Position = camMatrix * model * vec4(aPos, 1.0);
	// Assigns the colors from the Vertex Data to "color"
	color = aColor;
}