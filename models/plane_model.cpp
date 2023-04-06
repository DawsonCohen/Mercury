#include "plane_model.h"

PlaneModel::PlaneModel(float scale, glm::vec3 objectPos) : scale(scale)
{
	Vertex verts[] =
    {
        Vertex{glm::vec3(  0.0f,   0.0f,   0.0f)  ,    glm::vec4(0.6, 0.6f, 0.6f, 1.0f)},
		Vertex{glm::vec3(  100.0f, 0.0f,   0.0f)  ,    glm::vec4(0.6, 0.6f, 0.6f, 1.0f)},
        Vertex{glm::vec3(  0.0f,   0.0f,   100.0f),    glm::vec4(0.6, 0.6f, 0.6f, 1.0f)},
        Vertex{glm::vec3( -100.0f, 0.0f,   0.0f)  ,    glm::vec4(0.6, 0.6f, 0.6f, 1.0f)},
        Vertex{glm::vec3(  0.0f,   0.0f,  -100.0f),    glm::vec4(0.6, 0.6f, 0.6f, 1.0f)},
    };

	GLuint ind[] =
	{
		0,1,2,
		0,2,3,
		0,3,4,
		0,4,1
	};

	PlaneModel::mVertices = std::vector<Vertex>(verts, verts + sizeof(verts) / sizeof(Vertex));
	PlaneModel::mIndices = std::vector<GLuint>(ind, ind + sizeof(ind) / sizeof(GLuint));

    PlaneModel::mObjectPos = objectPos;

	PlaneModel::mObjectModel = scale*glm::mat4(1.0f);
	// PlaneModel::projection = 
	// glm::mat4(glm::vec3(),glm::vec3(),glm::vec3(),glm::vec3());
	mObjectModel = glm::translate(PlaneModel::mObjectModel, PlaneModel::mObjectPos);
	
	VAO.GenerateID();
	VAO.Bind();
	// Generates Vertex Buffer Object and links it to mVertices
	VBO VBO(PlaneModel::mVertices);
	// Generates Element Buffer Object and links it to mIndices
	EBO EBO(PlaneModel::mIndices);
	// Links VBO attributes such as coordinates and colors to VAO
	VAO.LinkAttrib(VBO, 0, 3, GL_FLOAT, sizeof(Vertex), (void*)0);
	VAO.LinkAttrib(VBO, 1, 3, GL_FLOAT, sizeof(Vertex), (void*)(3 * sizeof(float)));
	// Unbind all to prevent accidentally modifying them
	VAO.Unbind();
	VBO.Unbind();
	EBO.Unbind();
}

void PlaneModel::rotate(float deg, glm::vec3 axis) {
    mObjectModel = glm::rotate(mObjectModel, glm::radians(deg), axis);
}

void PlaneModel::translate(glm::vec3 translation) {
    mObjectModel = glm::translate(mObjectModel, translation);
}

// void PlaneModel::Draw(Shader& shader, Camera& camera)
// {
// 	// Bind shader to be able to access uniforms
// 	shader.Bind();
// 	VAO.Bind();

// 	// Take care of the camera Matrix
// 	shader.SetUniformMatrix4fv("model", glm::value_ptr(mObjectModel));
// 	camera.SetUniforms(shader);

// 	// Draw the actual mesh
// 	glDrawElements(GL_TRIANGLES, mIndices.size(), GL_UNSIGNED_INT, 0);
// }

void PlaneModel::Draw(Shader& shader, Camera& camera)
{
	// Bind shader to be able to access uniforms
	shader.Bind();
	VAO.Bind();

	// Take care of the uniforms Matrices
	shader.SetUniformMatrix4fv("model", mObjectModel);
	camera.SetUniforms(shader);

	// Draw the actual mesh
	glDrawElements(GL_TRIANGLES, mIndices.size(), GL_UNSIGNED_INT, 0);
}