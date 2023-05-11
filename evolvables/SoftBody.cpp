#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <map>
#include <chrono>
#include <csignal>
#include <algorithm>
#include "SoftBody.h"

#define min(a,b) a < b ? a : b
#define max(a,b) a > b ? a : b

unsigned SoftBody::seed = std::chrono::system_clock::now().time_since_epoch().count();
std::default_random_engine SoftBody::gen = std::default_random_engine(SoftBody::seed);
std::uniform_real_distribution<> SoftBody::uniform = std::uniform_real_distribution<>(0.0,1.0);

Eigen::Matrix3f rotation_matrix(double degrees, const Eigen::Vector3f& axis)
{
    // Convert angle to radians
    float radians = degrees * M_PI / 180.0;

    // Calculate sine and cosine of half angle
    float theta = radians / 2.0;
    float sin_theta = sin(theta);
    float cos_theta = cos(theta);

    // Create unit vector for axis of rotation
    Eigen::Vector3f u = axis.normalized();

    // Create quaternion for rotation
    Eigen::Quaternionf q(cos_theta, sin_theta * u.x(), sin_theta * u.y(), sin_theta * u.z());

    // Convert quaternion to rotation matrix
    Eigen::Matrix3f R = q.toRotationMatrix();

    return R;
}

Eigen::Matrix4f translation_matrix(const Eigen::Vector3f& translation)
{
    Eigen::Affine3f affine;
    affine = Eigen::Translation3f(translation);
    return affine.matrix();
}

void SoftBody::rotate(float deg, Eigen::Vector3f& axis) {
	Eigen::Matrix3f transform = rotation_matrix(deg, axis);
	for(auto& m : masses) {
		m.pos = (transform * Eigen::Vector3f(m.pos));
        m.protoPos = (m.pos);
	}
}

void SoftBody::translate(Eigen::Vector3f& translation) {
	Eigen::Matrix4f transform = translation_matrix(translation);
	for(Mass& m : masses) {
        Eigen::Vector4f v_homogeneous(m.pos.x(), m.pos.y(), m.pos.z(), 1.0f);
        Eigen::Vector4f v_transformed_homogeneous = transform * v_homogeneous;
		m.pos = Eigen::Vector3f(v_transformed_homogeneous.x(), v_transformed_homogeneous.y(), v_transformed_homogeneous.z());
        m.protoPos = (m.pos);
	}
}

void SoftBody::SimReset() {
    sim_time = 0;
    total_sim_time = 0;
}

void SoftBody::Reset() {
    for(Mass& m : masses) {
        m.pos = m.protoPos;
        m.vel = {0.0f, 0.0f, 0.0f};
        m.acc = {0.0f, 0.0f, 0.0f};
        m.force = {0.0f, 0.0f, 0.0f};
    }
    SimReset();
}

void SoftBody::Clear() {
    masses.clear();
    springs.clear();
}

void SoftBody::append(SoftBody src) {
    std::map<unsigned int, unsigned int> idMap;
    for(Mass& m : src.masses) {
        unsigned int oldID = m.id;
        m.id = masses.size();
        idMap[oldID] = m.id;
        masses.push_back(m);
    }
    for(Spring& s : src.springs) {
        s.m0 = idMap[s.m0];
        s.m1 = idMap[s.m1];
        springs.push_back(s);
    }
}

void SoftBody::calcFitness(SoftBody& R) {
    // Eigen::Vector3f mean_pos = calcMeanPos(R);
    Eigen::Vector3f COM = calcMeanPos(R);

    // R.mFitness = mean_pos.norm();
    // R.mFitness = mean_pos.x() - R.mCOM.x();
    // printf("%f - %f / %f\n", COM.x(), R.mBaseCOM.x(), R.mLength);
    
    R.mFitness = max((COM.x() - R.mBaseCOM.x()) / R.mLength, 0.0f);

    if(R.mFitness > 1000) {
        printf("Length: %f\n",R.mLength);
    }
}

Eigen::Vector3f SoftBody::calcClosestPos(SoftBody& R) {
    Eigen::Vector3f closest_pos = Eigen::Vector3f::Zero();
    for(Mass& m : R.masses) {
        if(m.material == materials::air) continue;
        if(m.pos.x() < closest_pos.x())
            closest_pos = m.pos;
    }

    return closest_pos;
}

float SoftBody::calcLength(SoftBody& R) {
    Eigen::Vector3f closest_pos = Eigen::Vector3f::Zero();
    Eigen::Vector3f furthest_pos = Eigen::Vector3f::Zero();
    for(Mass& m : R.masses) {
        if(m.material == materials::air) continue;
        if(m.pos.x() < closest_pos.x())
            closest_pos  = m.pos;
        if(m.pos.x() > furthest_pos.x())
            furthest_pos = m.pos;
    }

    return max(abs(furthest_pos.x() - closest_pos.x()),1.0f);
}

Eigen::Vector3f SoftBody::calcMeanPos(SoftBody& R) {
    Eigen::Vector3f mean_pos = Eigen::Vector3f::Zero();
    float i = 0;
    for(Mass& m : R.masses) {
        if(m.material == materials::air) continue;
        mean_pos = mean_pos + (m.pos - mean_pos) * 1.0f/(i+1);
        i++;
    }

    return mean_pos;
}

void SoftBody::printObjectPositions() {
    Eigen::Vector3f p = Eigen::Vector3f::Zero();
	for(const Mass& m : masses) {
		p = m.pos; 
		printf("%u - %f, %f, %f\n", m.id, p.x(), p.y(), p.z());
	}
}