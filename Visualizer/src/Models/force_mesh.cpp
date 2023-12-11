#include "force_mesh.h"

using namespace EvoDevo;
using namespace Elastic;

#define EPS (float) 1e-12

void ForceMesh::Update(Element& robot, Environment& env) {
    m_Edges.clear();

    std::vector<glm::vec3> forces(robot.masses.size(), glm::vec3(0.0f));
    
    for(auto& f : robot.faces) {
        glm::vec3 p0 = glm::vec3(robot.masses[f.m0].pos.x(), robot.masses[f.m0].pos.y(), robot.masses[f.m0].pos.z());
        glm::vec3 p1 = glm::vec3(robot.masses[f.m1].pos.x(), robot.masses[f.m1].pos.y(), robot.masses[f.m1].pos.z());
        glm::vec3 p2 = glm::vec3(robot.masses[f.m2].pos.x(), robot.masses[f.m2].pos.y(), robot.masses[f.m2].pos.z());
        glm::vec3 v0 = glm::vec3(robot.masses[f.m0].vel.x(), robot.masses[f.m0].vel.y(), robot.masses[f.m0].vel.z());
        glm::vec3 v1 = glm::vec3(robot.masses[f.m1].vel.x(), robot.masses[f.m1].vel.y(), robot.masses[f.m1].vel.z());
        glm::vec3 v2 = glm::vec3(robot.masses[f.m2].vel.x(), robot.masses[f.m2].vel.y(), robot.masses[f.m2].vel.z());

        glm::vec3 v = (v0 + v1 + v2) / 3.0f;
        glm::vec3 n = glm::cross(p1 - p0, p2 - p0);
        float area = glm::length(n);
        n = n / (area + EPS);
        n = glm::dot(n, v) > 0 ? n : -n;

        glm::vec3 force = -0.5f*env.drag*area*(glm::dot(v,n)*v + 0.2f*glm::dot(v,v)*n);
        force = force / 3.0f;

        forces[f.m0] += force;
        forces[f.m1] += force;
        forces[f.m2] += force;
    }

    float maxF = 0.0f;
    for(glm::vec3 f : forces) {
        maxF = std::max(maxF, glm::length(f));
    }

    for(uint i = 0; i < robot.masses.size(); i++) {
        glm::vec3 pos = glm::vec3(robot.masses[i].pos.x(), robot.masses[i].pos.y(), robot.masses[i].pos.z());
        forces[i] = forces[i] / maxF;
        m_Vertices.push_back({pos, {0.0f, 0.0f, 0.0f, 0.0f}});
        m_Vertices.push_back({pos + forces[i], {0.0f, 0.0f, 0.0f, 0.0f}});
        m_Edges.push_back({m_Vertices.size()-2, m_Vertices.size()-1, {1.0f, 0.0f, 0.0f, 1.0f}});
    }
}