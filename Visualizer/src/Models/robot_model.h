#ifndef __ROBOT_MODEL_H__
#define __ROBOT_MODEL_H__

// Evolvable Soft Body
#include "Elastic.h"
#include "EvoDevo.h"
#include <map>


class RobotModel : public EvoDevo::SoftBody, public Elastic::Mesh {
    float m_FaceAlpha = 0.8;
    
public:
    RobotModel(EvoDevo::Element& robot);
    void Update(bool drawFaces = true);
    void Update(EvoDevo::Element& robot, bool drawFaces = true);

    void setFaceAlpha(float alpha) { m_FaceAlpha = alpha; }

    std::string ToOBJ() const;
};

#endif
