#ifndef __ROBOT_H__
#define __ROBOT_H__

// Evolvable Soft Body
#include "Model.h"
#include "element.h"
#include <map>

template<typename T>
class RobotModel : public Model {
    T m_robot;
    
public:
    RobotModel(T& robot) : Model(), m_robot(robot) {
        updateMesh();
    };
    
    RobotModel() : Model(), m_robot() {
        updateMesh();
    };

    void Update(T& robot) {
        m_robot = robot;

        updateMesh();
    }

    void updateMesh() {
        std::vector<Vertex> vertices;
        std::vector<uint> indices;

        for(const Spring& s : m_robot.springs) {
            if(s.material == materials::air) continue;
            Mass m0 = m_robot.masses[s.m0];
            Mass m1 = m_robot.masses[s.m1];
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
    //     VoxelRobot::append((VoxelRobot) src);
    //     updateMesh();
    // }

	friend void swap(RobotModel& s1, RobotModel& s2) {
		swap(s1.m_robot, s2.m_robot);
		swap((Model&) s1, (Model&) s2);
    }
};

#endif
