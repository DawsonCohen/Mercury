#ifndef __ROBOT_MODEL_H__
#define __ROBOT_MODEL_H__

// Evolvable Soft Body
#include "Elastic.h"
#include "EvoDevo.h"
#include <map>


class RobotModel : public EvoDevo::SoftBody, public Elastic::Mesh {
    
public:
    RobotModel(EvoDevo::Element& robot);
    void Update(EvoDevo::Element& robot, bool drawFaces = true);

    std::string ToOBJ() const;
};

#endif
