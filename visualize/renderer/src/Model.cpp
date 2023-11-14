#include "Model.h"
#include <iostream>

Model::Model()
{
    mObjectModel = glm::mat4(1.0f);
    mColor = glm::vec4(0.1f,0.1f,0.1f, 1.0f);
}

Model::Model(const Model& src):
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

void Model::Draw(Shader& shader, const Camera& camera)
{
    shader.Bind();
    shader.SetUniformMatrix4fv("model", mObjectModel);

    for(Mesh& mesh : mMeshes) {
        mesh.Bind();
        mesh.Draw(shader,camera);
    }
    Unbind();
    shader.Unbind();
}

void Model::DrawGroup(Shader& shader, const Camera& camera, int group)
{
    shader.Bind();
    shader.SetUniformMatrix4fv("model", mObjectModel);

    for(Mesh& mesh : mMeshes) {
        if(mesh.getGroup() == group) {
            mesh.Bind();
            mesh.Draw(shader,camera);
        }
    }
    for(Mesh& mesh : mMeshes) {
        if(mesh.getGroup() == group) {
            mesh.Unbind();
        }
    }
    Unbind();
    shader.Unbind();
}

void Model::rotate(float deg, glm::vec3 axis) {
    mObjectModel = glm::rotate(mObjectModel, glm::radians(deg), axis);
}

void Model::translate(glm::vec3 translation) {
    mObjectModel = glm::translate(mObjectModel, translation);
}