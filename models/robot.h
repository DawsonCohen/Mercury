#ifndef __ROBOT_H__
#define __ROBOT_H__

// Evolvable Soft Body
#include "VoxelRobot.h"
#include "Model.h"

class Robot : public VoxelRobot, public Model {
public:
    Robot(float x, float y, float z, float res, std::vector<Voxel> voxels) :
        VoxelRobot(x,y,z,res,voxels), Model()
    { 
        updateMesh();
        // mMeshes.push_back(Mesh());
    }
    
    Robot() : VoxelRobot(), Model() {
        updateMesh();
    };
    
    // Robot(Robot& v);
    Robot(VoxelRobot& v);

    static std::vector<float> findDiversity(std::vector<Robot> pop);

    static CandidatePair<Robot> Crossover(const CandidatePair<Robot>& parents);

    static float Distance(const CandidatePair<Robot>& robots);

    void rotate(float deg, glm::vec3 axis) {
        VoxelRobot::rotate(deg, axis);
        updateMesh();
    }
    void translate(glm::vec3 translation) {
        VoxelRobot::translate(translation);
        updateMesh();
    }

    void Update(Element e) { 
        VoxelRobot::Update(e);
		updateMesh();
	}

    void Randomize() {
        VoxelRobot::Randomize();
        updateMesh();
    }
    
    void Reset() {
        VoxelRobot::Reset();
        updateMesh();
    }
    
	void Clear() {
        VoxelRobot::Clear();
        mMeshes.clear();
        mMeshes.push_back(Mesh());
        updateMesh();
    }

	void updateMesh();

	void append(Robot src) {
        VoxelRobot::append((VoxelRobot) src);
        updateMesh();
    }

    void addSpring(Spring& s) {
        VoxelRobot::addSpring(s);
        updateMesh();
    }
	void setMasses(std::vector<Mass> _masses) {
        VoxelRobot::setMasses(_masses);
        updateMesh();
    }
	void setSprings(std::vector<Spring> _springs) {
        VoxelRobot::setSprings(_springs);
        updateMesh();
    }

	friend void swap(Robot& s1, Robot& s2) {
		swap((VoxelRobot&) s1, (VoxelRobot&) s2);
		swap((Model&) s1, (Model&) s2);
    }

    void Build() {
        VoxelRobot::Build();
        updateMesh();
    }
};

#endif
