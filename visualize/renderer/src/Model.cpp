#include "Model.h"
#include <iostream>

Model::Model()
{
    mMeshes.push_back(Mesh());

    mObjectModel = glm::mat4(1.0f);
    mColor = glm::vec4(0.1f,0.1f,0.1f, 1.0f);
}

Model::Model(const Model& src):
    mDirectory(src.mDirectory), mPath(src.mPath),
    mObjectModel(src.mObjectModel), mColor(src.mColor),
    mMeshes(src.mMeshes) {}

Model::Model(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices,
        glm::mat4 objectModel) :
    mObjectModel(objectModel)     
{
    mMeshes.push_back(Mesh(vertices, indices));
}

void Model::Bind() {
    for(size_t i = 0; i < mMeshes.size(); i++) {
        mMeshes[i].Bind();
    }
}

void Model::Unbind() {
    for(size_t i = 0; i < mMeshes.size(); i++) {
        mMeshes[i].Unbind();
    }
}

void Model::Draw(Shader &shader, Camera& camera)
{
    shader.Bind();
    Bind();
    shader.SetUniformMatrix4fv("model", mObjectModel);

    for(Mesh& mesh : mMeshes) {
        mesh.Draw(shader,camera);
    }
    Unbind();
	// for(size_t i = 0; i < mMeshes.size(); i++) {
	// 	mMeshes[i].Draw(shader, camera);
	// }
    shader.Unbind();
}

void Model::rotate(float deg, glm::vec3 axis) {
    mObjectModel = glm::rotate(mObjectModel, glm::radians(deg), axis);
}

void Model::translate(glm::vec3 translation) {
    mObjectModel = glm::translate(mObjectModel, translation);
}