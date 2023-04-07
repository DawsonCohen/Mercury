#include "SoftBody.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <map>

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

SoftBody::SoftBody( const SoftBody& src )
: Element{src.masses, src.springs}, Candidate(src),
mDirectEncoding(src.mDirectEncoding), mRadiiEncoding(src.mRadiiEncoding)
{}

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
        uint oldID = m.id;
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

void SoftBody::printObjectPositions() {
    Eigen::Vector3f p = Eigen::Vector3f::Zero();
	for(const Mass& m : masses) {
		p = m.pos; 
		printf("%u - %f, %f, %f\n", m.id, p.x(), p.y(), p.z());
	}
}