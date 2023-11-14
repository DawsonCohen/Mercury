#include "Mesh.h"
#include "Renderer.h"
#include <iostream>

Mesh::Mesh(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices) :
    mVertices(vertices), mIndices(indices)
{ }

Mesh::Mesh(const Mesh& src) :
	mVertices(src.mVertices), mIndices(src.mIndices),
    mVAO(src.mVAO), mVBO(src.mVBO), mEBO(src.mEBO), mLineWidth(src.mLineWidth), mGroupID(src.mGroupID)
	 { }

void Mesh::Bind() {
    setupMesh();
}

void Mesh::Unbind() {
	mVAO.Unbind();
	mVBO.Unbind();
	mEBO.Unbind();
}

void Mesh::setupMesh() {
	mVAO.GenerateID();
	mVBO.GenerateID();
	mEBO.GenerateID();
	mVAO.Bind();
	mVBO.Bind(mVertices);
	mEBO.Bind(mIndices);

	// Links VBO attributes such as coordinates and colors to VAO
	mVAO.LinkAttrib(mVBO, 0, 3, GL_FLOAT, sizeof(Vertex), (void*)0);
	mVAO.LinkAttrib(mVBO, 1, 3, GL_FLOAT, sizeof(Vertex), (void*)(3 * sizeof(float)));

	// Unbind all to prevent accidentally modifying them
	mVAO.Unbind();
}

void Mesh::Draw(Shader& shader, const Camera& camera) {
    shader.Bind();
	mVAO.Bind();
	mVBO.Bind(mVertices);
	mEBO.Bind();

	// Take care of the camera Matrix
	camera.SetUniforms(shader);

	// for(uint i = 0; i < mIndices.size(); i++) {
	// 	Vertex v = mVertices[i];
	// }

	// Draw the mesh
	GLCall(glLineWidth((GLfloat) mLineWidth));
	GLCall(glDrawElements(GL_LINES, mIndices.size(), GL_UNSIGNED_INT, 0));

	mVAO.Unbind();
	shader.Unbind();
}

void Mesh::updateVertex(size_t index, Vertex v) {
	std::swap(mVertices[index], v);
}

void Mesh::updateVertices(std::vector<Vertex> vertices) {
	mVertices = vertices;
}

void Mesh::updateIndices(std::vector<uint> indices) {
	mIndices = indices;
}

void Mesh::updateVertex(size_t index, glm::vec3 pos) {
	mVertices[index].position = pos;
}

void Mesh::updateIndex(size_t index, uint v_idx) {
	mIndices[index] = v_idx;
}