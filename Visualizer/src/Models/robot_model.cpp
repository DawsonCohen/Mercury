#include "robot_model.h"

using namespace EvoDevo;
using namespace Elastic;

RobotModel::RobotModel(Element& robot) : SoftBody(robot) {
    Update(robot);
}

void RobotModel::Update(Element& robot, bool drawFaces) {
    SoftBody::Update(robot);
    
    m_Vertices.clear();
    m_Edges.clear();
    m_Facets.clear();
    
    size_t m0, m1, m2;
    glm::vec3 pos;
    glm::vec4 color;
    for(const Mass& m : masses) {
        glm::vec3 pos = glm::vec3(m.pos.x(), m.pos.y(), m.pos.z());
        glm::vec4 color = glm::vec4(m.material.color.r, m.material.color.g, m.material.color.b, m.material.color.a);
        m_Vertices.push_back({pos, color});
    }

    for(const Spring& s : springs) {
        m0 = s.m0;
        m1 = s.m1;

        color = glm::vec4(s.material.color.r, s.material.color.g, s.material.color.b, s.material.color.a);
        m_Edges.push_back({m0, m1, color});
    }

    if(!drawFaces) return;
    for(const Face& f : faces) {
        m0 = f.m0;
        m1 = f.m1;
        m2 = f.m2;

        glm::vec4 color = {0.1, 0.4, 0.1, 0.8};
        m_Facets.push_back({m0, m1, m2, color});
    }
}