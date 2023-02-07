#include "SoftBody.h"
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtx/rotate_vector.hpp>
#include <iostream>
#include <map>

SoftBody::SoftBody( const SoftBody& src )
: Element{src.masses, src.springs}, 
mDirectEncoding(src.mDirectEncoding), mRadiiEncoding(src.mRadiiEncoding)
{}

void SoftBody::rotate(float deg, glm::vec3 axis) {
	glm::mat4 transform = glm::rotate(glm::mat4(1.0f), glm::radians(deg), axis);
	for(auto& m : masses) {
		m.pos = (transform * glm::vec4(m.pos,1.0f));
        m.protoPos = (m.pos);
	}
	// updateMesh();
}

void SoftBody::translate(glm::vec3 translation) {
	glm::mat4 transform = glm::translate(glm::mat4(1.0f), translation);
	for(Mass& m : masses) {
		m.pos = (transform * glm::vec4(m.pos,1.0f));
        m.protoPos = (m.pos);
	}
	// updateMesh();
}

void SoftBody::SimReset() {
    mSimTime = 0;
    mTotalSimTime = 0;
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
	glm::vec3 p(0.0f);
	for(const Mass& m : masses) {
		p = m.pos; 
		#ifndef THREED
		printf("%i - %f, %f\n", m.id, p.x, p.y);
		#else
		printf("%i - %f, %f, %f\n", m.id, p.x, p.y, p.z);
		#endif
	}
}