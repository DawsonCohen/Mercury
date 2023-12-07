#include "robot_model.h"

using namespace EvoDevo;
using namespace Elastic;

RobotModel::RobotModel(Element& robot) {
    Update(robot);
}

void RobotModel::Update(Element& robot) {
    for(const Mass& m : robot.masses) {
        glm::vec3 pos = glm::vec3(m.pos.x(), m.pos.y(), m.pos.z());
        glm::vec4 color = glm::vec4(m.material.color.r, m.material.color.g, m.material.color.b, m.material.color.a);
        m_Vertices.push_back({pos, color});
    }

    for(const Spring& s : robot.springs) {
        size_t m0 = s.m0;
        size_t m1 = s.m1;

        glm::vec4 color = glm::vec4(s.material.color.r, s.material.color.g, s.material.color.b, s.material.color.a);
        m_Edges.push_back({m0, m1, color});
    }

    for(const Spring& s : robot.springs) {
        size_t m0 = s.m0;
        size_t m1 = s.m1;

        glm::vec4 color = glm::vec4(s.material.color.r, s.material.color.g, s.material.color.b, s.material.color.a);
        m_Edges.push_back({m0, m1, color});
    }
    
    SoftBody::Update(robot);
}