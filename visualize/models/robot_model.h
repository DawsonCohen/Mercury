#ifndef __ROBOT_H__
#define __ROBOT_H__

// Evolvable Soft Body
#include "Model.h"
#include "element.h"
#include <map>

class RobotModel : public Element, public Model {
public:
    RobotModel(Element& robot) : Element(robot), Model() {
        updateMesh();
    };
    
    RobotModel() : Element(),   Model() {
        updateMesh();
    };

    void Update(Element& robot) {
        masses = robot.masses;
        springs = robot.springs;

        updateMesh();
    }

    void updateMesh() {
        std::vector<Vertex> vertices;
        std::vector<uint> indices;

        for(const Spring& s : springs) {
            if(s.material == materials::air) continue;
            Mass m0 = masses[s.m0];
            Mass m1 = masses[s.m1];
            Vertex v0 = {glm::vec3(m0.pos.x(), m0.pos.y(), m0.pos.z()), glm::vec4(s.material.color.r, s.material.color.g, s.material.color.b, s.material.color.a)};
            Vertex v1 = {glm::vec3(m1.pos.x(), m1.pos.y(), m1.pos.z()), glm::vec4(s.material.color.r, s.material.color.g, s.material.color.b, s.material.color.a)};
            vertices.push_back(v0);
            indices.push_back(vertices.size()-1);
            vertices.push_back(v1);
            indices.push_back(vertices.size()-1);
        }
        mMeshes[0].updateVertices(vertices);
        mMeshes[0].updateIndices(indices);
    }

    // // TODO
	// void append(T src) {
    //     SoftBody::append((SoftBody) src);
    //     updateMesh();
    // }
};

#endif
