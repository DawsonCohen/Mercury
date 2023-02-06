#include "robot.h"
#include <map>

Robot::Robot(VoxelRobot& v) : VoxelRobot(v), Model() {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    for(const Mass& m : masses) vertices.push_back({m.pos, m.color});
    for(const Spring& s : springs) {
        indices.push_back(s.m0);
        indices.push_back(s.m1);
    }

    updateMesh();
};

RobotPair Robot::Crossover(const RobotPair& parents) {
    VoxelRobotPair voxelParents(parents.first, parents.second);
    VoxelRobotPair voxelChildren = VoxelRobot::Crossover(voxelParents);
    RobotPair results = {voxelChildren.first, voxelChildren.second};
    return results;
}

float Robot::Distance(const RobotPair& robots) {
    VoxelRobotPair voxelRobots(robots.first, robots.second);
    return VoxelRobot::Distance(voxelRobots);
}

std::vector<float> Robot::findDiversity(std::vector<Robot> pop) {
    size_t pop_size = pop.size();
    size_t v_size = pop[0].getVoxels().size();
    std::vector<float> diversity(pop_size, 0);
    
    std::map<Material, float> mat_count;

    // TODO
    for(size_t i = 0; i < v_size; i ++) {
        mat_count.clear();
        for(size_t j  = 0; j < pop_size; j++){
            Material mat = pop[j].getVoxels()[i].mat;
            if(mat_count.count(mat) == 0) mat_count[mat] = 0;
            mat_count[mat]++;
        }
        for(auto& count : mat_count) {
            mat_count[count.first] = count.second / pop_size;
        }
        for(size_t j = 0; j < pop_size; j++) {
            Voxel v = pop[j].getVoxels()[i];
            diversity[j] += 1 - mat_count[v.mat];
        }
    }
    return diversity;
}

void Robot::updateMesh() {
    std::vector<Vertex> vertices;
    std::vector<uint> indices;

    for(const Spring& s : springs) {
        if(!s.active) continue;
        Mass m0 = masses[s.m0];
        Mass m1 = masses[s.m1];
        vertices.push_back({m0.pos, s.material.color});
        indices.push_back(vertices.size()-1);
        vertices.push_back({m1.pos, s.material.color});
        indices.push_back(vertices.size()-1);
    }
    mMeshes[0].updateVertices(vertices);
    mMeshes[0].updateIndices(indices);
}