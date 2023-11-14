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
    float mLineWidth = 3.0;
    int mGroupID;

public:
    Mesh(int group = 0) : mGroupID(group) {}
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
      swap(first.mLineWidth, second.mLineWidth);
      swap(first.mGroupID, second.mGroupID);
    }

    Mesh& operator=(Mesh src)
	{
		swap(*this, src);
		return *this;
	}

    void Bind();
    void Unbind();
    void Draw(Shader& shader, const Camera& camera);
    void updateVertex(size_t index, Vertex v);
    void addVertex(Vertex v) { mVertices.push_back(v); }
    void addIndex(unsigned int i) { mIndices.push_back(i); }
    void updateVertices(std::vector<Vertex> vertices);
    void updateIndices(std::vector<uint> indices);
    void updateVertex(size_t index, glm::vec3 pos);
    void updateIndex(size_t index, uint vertex_idx);

    void rotate(float deg, glm::vec3 axis);

    void setLineWidth(float lw) { mLineWidth = lw; }

    void print() const {
        printf("Mesh Print\n");
        for(const Vertex& v : mVertices) {
            glm::vec3 pos = v.position;
            printf("%f,%f,%f\n",pos.x,pos.y,pos.z);
        }
    }

    int getGroup() const { return mGroupID; }

private:
    void setupMesh();
};

#endif