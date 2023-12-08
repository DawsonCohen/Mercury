#ifndef __ROBOT_H__
#define __ROBOT_H__

// Evolvable Soft Body
#include "Elastic.h"
#include "EvoDevo.h"
#include "environment.h"
#include <map>


class RobotModel : public EvoDevo::SoftBody, public Elastic::Mesh {
    
public:
    RobotModel(EvoDevo::Element& robot);
    void RobotModel::Update(EvoDevo::Element& robot);
};

#endif
