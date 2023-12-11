#ifndef __FORCE_MESH_H__
#define __FORCE_MESH_H__

// Evolvable Soft Body
#include "Elastic.h"
#include "EvoDevo.h"
#include <map>

class ForceMesh : public Elastic::Mesh {
public:
    ForceMesh() = default;
    void Update(EvoDevo::Element& robot, EvoDevo::Environment& env);
};

#endif
