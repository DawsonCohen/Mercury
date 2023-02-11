#ifndef __MESH_H__
#define __MESH_H__

#include "VAO.h"
#include "EBO.h"
#include "Camera.h"

#include "glm/glm.hpp"

class Mesh
{
    std::vector<Vertex> mVertices;
    std::vector<unsigned int> mIndices;
    VAO mVAO;
    VBO mVBO;
    EBO mEBO;

public:
    Mesh() {}
    Mesh(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices);
    Mesh(const Mesh& src);

    friend void swap(Mesh& first, Mesh& second)
    {
      using std::swap;
      swap(first.mVertices, second.mVertices);
      swap(first.mIndices, second.mIndices);
      swap(first.mVAO, second.mVAO);
      swap(first.mVBO, second.mVBO);
      swap(first.mEBO, second.mEBO);
    }

    Mesh& operator=(Mesh src)
	{
		swap(*this, src);
		return *this;
	}

    void Bind();
    void Unbind();
    void Draw(Shader& shader, Camera& camera) const;
    void updateVertex(size_t index, Vertex v);
    void addVertex(Vertex v) { mVertices.push_back(v); }
    void addIndex(unsigned int i) { mIndices.push_back(i); }
    void updateVertices(std::vector<Vertex> vertices);
    void updateIndices(std::vector<uint> indices);
    void updateVertex(size_t index, glm::vec3 pos);
    void updateIndex(size_t index, uint vertex_idx);

    void rotate(float deg, glm::vec3 axis);

    void print() const {
        printf("Mesh Print\n");
        for(const Vertex& v : mVertices) {
            glm::vec3 pos = v.position;
            printf("%f,%f,%f\n",pos.x,pos.y,pos.z);
        }
    }

private:
    void setupMesh();
};

#endif