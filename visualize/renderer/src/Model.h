#ifndef __MODEL_H__
#define __MODEL_H__

#include "Mesh.h"
#include "VAO.h"
#include "EBO.h"
#include "Camera.h"

#include "glm/glm.hpp"

class Model {
protected:
	glm::mat4 mObjectModel = glm::mat4(1.0f);
	glm::vec4 mColor;
    std::vector<Mesh> mMeshes;

public:
    Model();
    Model(const Model&);
    Model(std::string path, glm::mat4 objectModel = glm::mat4(1.0f),
        glm::vec4 color = glm::vec4(0.1f,0.1f,0.1f, 1.0f));
    Model(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices,
        glm::mat4 objectModel = glm::mat4(1.0f));

    friend void swap(Model& m1, Model& m2) {
        using std::swap;
        swap(m1.mObjectModel, m2.mObjectModel);
        swap(m1.mColor, m2.mColor);
        swap(m1.mMeshes, m2.mMeshes);
    }

    void Bind();
    void Unbind();
    void rotate(float deg, glm::vec3 axis);
    void translate(glm::vec3 translation);
    void scale(float scale);
    virtual void Draw(Shader& shader, const Camera& camera);
    void DrawGroup(Shader& shader, const Camera& camera, int group);

    void updateVertices();

    void printMesh() const { 
        for(size_t i = 0; i < mMeshes.size(); i++) {
            mMeshes[i].print();
        }
    }

    Model& operator=(Model src)
	{
		swap(*this, src);
		return *this;
	}
};

#endif