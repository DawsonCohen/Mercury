#ifndef __ROBOT_H__
#define __ROBOT_H__

// Evolvable Soft Body
#include "Model.h"
#include "element.h"
#include "environment.h"
#include <map>


class RobotModel : public Element, public Model {
    void _initialize() {
        mMeshes.push_back(Mesh((int) MESH_GROUP_BODY));
        mMeshes.push_back(Mesh((int) MESH_GROUP_DRAG));
        mMeshes.push_back(Mesh((int) MESH_GROUP_DRAG_ONE));
        mMeshes[1].setLineWidth(1.0f);
        mMeshes[2].setLineWidth(7.0f);
    }

public:
    enum RobotMeshGroup : int {
        MESH_GROUP_BODY = 0,
        MESH_GROUP_DRAG = 1,
        MESH_GROUP_DRAG_ONE = 2
    };

    RobotModel(Element& robot) : Element(robot), Model() {
        _initialize();
    };
    
    RobotModel() : Element(),   Model() {
        _initialize();
    };

    void Update(Element& robot, const std::vector<RobotMeshGroup>& groups) {
        masses = robot.masses;
        springs = robot.springs;
        drawgroups = groups;

        for(size_t i = 0; i < groups.size(); i++) {
            switch(groups[i]) {
                case MESH_GROUP_BODY:
                    updateBodyMesh();
                    break;
                case MESH_GROUP_DRAG:
                    updateDragMesh();
                    break;
                case MESH_GROUP_DRAG_ONE:
                    updateDragOneMesh(0);
                default:
                    break;
            }
        }
    }

    void updateBodyMesh() {
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

    void updateDragMesh() {
        std::vector<Vertex> vertices;
        std::vector<uint> indices;
        for(const Mass& m : masses) {
            if(m.material == materials::air) continue;
            glm::vec3 pos0 = glm::vec3(m.pos.x(), m.pos.y(), m.pos.z());

            float rho = EnvironmentWater.drag;

            //Force due to drag = - (1/2 * rho * |v|^2 * A * Cd) * v / |v| (Assume A and Cd are 1)
            float mag_vel_squared = 
                fmul(m.vel.x(),m.vel.x())
                + fmul(m.vel.y(),m.vel.y())
                + fmul(m.vel.z(),m.vel.z());
            float mag_vel = fsqrt(mag_vel_squared);

            glm::vec3 pos1 = pos0 + glm::vec3(
                -0.5*rho*mag_vel*m.vel.x(),
                -0.5*rho*mag_vel*m.vel.y(),
                -0.5*rho*mag_vel*m.vel.z()
                );

            Vertex v0 = {pos0, glm::vec4(1,0,0,1)};
            Vertex v1 = {pos1, glm::vec4(1,0,0,1)};

            
            vertices.push_back(v0);
            indices.push_back(vertices.size()-1);
            vertices.push_back(v1);
            indices.push_back(vertices.size()-1);
        }
        mMeshes[1].updateVertices(vertices);
        mMeshes[1].updateIndices(indices);
    }

    void updateDragOneMesh(size_t massId) {
        std::vector<Vertex> vertices;
        std::vector<uint> indices;

        Mass m = masses[massId % masses.size()];
        for(size_t i = 0; i < masses.size(); i++) {
            m = masses[(massId + i) % masses.size()];
            if(m.material != materials::air) break;
        }
        
        glm::vec3 pos0 = glm::vec3(m.pos.x(), m.pos.y(), m.pos.z());

        float rho = EnvironmentWater.drag;

        //Force due to drag = - (1/2 * rho * |v|^2 * A * Cd) * v / |v| (Assume A and Cd are 1)
        float mag_vel_squared = 
            fmul(m.vel.x(),m.vel.x())
            + fmul(m.vel.y(),m.vel.y())
            + fmul(m.vel.z(),m.vel.z());
        float mag_vel = fsqrt(mag_vel_squared);

        glm::vec3 pos1 = pos0 + glm::vec3(
            -0.5*rho*mag_vel*m.vel.x(),
            -0.5*rho*mag_vel*m.vel.y(),
            -0.5*rho*mag_vel*m.vel.z()
            );

        Vertex v0 = {pos0, glm::vec4(1,0,0,1)};
        Vertex v1 = {pos1, glm::vec4(1,0,0,1)};

        
        vertices.push_back(v0);
        indices.push_back(vertices.size()-1);
        vertices.push_back(v1);
        indices.push_back(vertices.size()-1);

        mMeshes[2].updateVertices(vertices);
        mMeshes[2].updateIndices(indices);
    }

    void Draw(Shader& shader, const Camera& camera) override {
        for(size_t i = 0; i < drawgroups.size(); i++) {
            DrawGroup(shader, camera, (int) drawgroups[i]);
        }
    }

private:
    std::vector<RobotMeshGroup> drawgroups = {MESH_GROUP_BODY};
};

#endif
