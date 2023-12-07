#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <map>
#include <chrono>
#include <csignal>
#include <limits>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <thread>
#include "SoftBody.h"

#define min(a,b) a < b ? a : b
#define max(a,b) a > b ? a : b

namespace EvoDevo {

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

    void SoftBody::Reset() {
        for(Mass& m : masses) {
            m.pos = m.protoPos;
            m.vel = {0.0f, 0.0f, 0.0f};
        }
        for(Spring& s : springs) {
            s.mean_length = (masses[s.m0].protoPos - masses[s.m1].protoPos).norm();
            s.rest_length = s.mean_length;
        }
        sim_time = 0;
        total_sim_time = 0;
        updateBaseline();
    }

    void SoftBody::Clear() {
        masses.clear();
        springs.clear();
    }

    void SoftBody::updateClosestPos() {
        Eigen::Vector3f closest_pos = Eigen::Vector3f::Zero();
        for(Mass& m : masses) {
            if(m.material == materials::air) continue;
            if(m.pos.x() < closest_pos.x())
                closest_pos = m.pos;
        }

        mClosestPos = closest_pos;
    }

    void SoftBody::updateLength() {
        Eigen::Vector3f closest_pos = Eigen::Vector3f::Zero();
        Eigen::Vector3f furthest_pos = Eigen::Vector3f::Zero();
        for(Mass& m : masses) {
            if(m.material == materials::air) continue;
            if(m.pos.x() < closest_pos.x())
                closest_pos  = m.pos;
            if(m.pos.x() > furthest_pos.x())
                furthest_pos = m.pos;
        }

        mLength = max(fabsf(furthest_pos.x() - closest_pos.x()),1.0f);
    }

    void SoftBody::updateCOM() {
        Eigen::Vector3f mean_pos = Eigen::Vector3f::Zero();
        float i = 0;
        for(Mass& m : masses) {
            if(m.material == materials::air) continue;
            mean_pos = mean_pos + (m.pos - mean_pos) * 1.0f/(i+1);
            i++;
        }

        mCOM = mean_pos;
    }

    void SoftBody::updateFitness() {
        updateCOM();

        if(!mValid) {
            mFitness = 0.0f;
            return;
        }

        mFitness = max((mCOM.x() - mBaseCOM.x()) / mLength, 0.0f);

        if(mFitness > 1000) {
            printf("Length: %f\n",mLength);
        }
    }

    void SoftBody::printObjectPositions() {
        Eigen::Vector3f p = Eigen::Vector3f::Zero();
        for(const Mass& m : masses) {
            p = m.pos; 
            printf("%u - %f, %f, %f\n", m.id, p.x(), p.y(), p.z());
        }
    }

    std::string SoftBody::Encode() const {
        std::stringstream ss;
        ss << "type=NNRobot\n";
        ss << "masses=";
        for(unsigned int i = 0; i < masses.size(); i++) {
            ss << masses[i];
            ss << (i < (masses.size()- 1) ? ";" : "\n");
        }
        ss << "springs=";
        for(unsigned int i = 0; i < springs.size(); i++) {
            ss << springs[i];
            ss << (i < springs.size()- 1 ? ";" : "\n");
        }
        ss << "faces=";
        for(unsigned int i = 0; i < faces.size(); i++) {
            ss << faces[i];
            ss << (i < faces.size()- 1 ? ";" : "\n");
        }
        ss << "cells=";
        for(unsigned int i = 0; i < cells.size(); i++) {
            ss << cells[i];
            ss << (i < cells.size()- 1 ? ";" : "\n");
        }
        ss << "boundary_count=" << boundaryCount << "\n";
        return ss.str();
    }

    void SoftBody::Decode(const std::string& filename) {
        std::ifstream infile(filename);
        if (!infile.is_open()) {
            std::cerr << "Error decoding robot file: " << filename << std::endl;
            exit(1);
        }
        
        std::string line;
        masses.clear();
        springs.clear();
        faces.clear();
        cells.clear();

        std::getline(infile, line);
        std::getline(infile, line);
        std::size_t pos = line.find('=');
        std::string key = line.substr(0, pos);
        std::string value = line.substr(pos+1);
        if(key == "masses") {
            std::istringstream lineStream(value);
            std::string cell;

            while (std::getline(lineStream, cell, ';')) {
                std::istringstream word(cell);
                std::string param;
                uint id;
                float x,y,z, m;
                Material mat;

                std::getline(word, param, ',');
                id = std::stoi(param);
                std::getline(word, param, ',');
                x = std::stof(param);
                std::getline(word, param, ',');
                y = std::stof(param);
                std::getline(word, param, ',');
                z = std::stof(param);
                std::getline(word, param, ',');
                m = std::stof(param);
                std::getline(word, param, ',');
                mat = materials::id_lookup(stoi(param));
                masses.push_back(Mass(id, x, y, z, m, mat));
            }
        }
        std::getline(infile, line);
        pos = line.find('=');
        key = line.substr(0, pos);
        value = line.substr(pos+1);
        if(key == "springs") {
            std::istringstream lineStream(value);
            std::string cell;

            while (std::getline(lineStream, cell, ';')) {
                std::istringstream word(cell);
                std::string param;
                uint16_t m0, m1;
                float rl, ml;
                Material mat;

                std::getline(word, param, ',');
                m0 = std::stoi(param);
                std::getline(word, param, ',');
                m1 = std::stoi(param);
                std::getline(word, param, ',');
                rl = std::stof(param);
                std::getline(word, param, ',');
                ml = std::stof(param);
                std::getline(word, param, ',');
                mat = materials::id_lookup(std::stoi(param));
                Spring s = {m0, m1, rl, ml, mat};
                springs.push_back(s);
            }
        }
        std::getline(infile, line);
        pos = line.find('=');
        key = line.substr(0, pos);
        value = line.substr(pos+1);
        if(key == "faces") {
            std::istringstream lineStream(value);
            std::string cell;

            while (std::getline(lineStream, cell, ';')) {
                std::istringstream word(cell);
                std::string param;
                uint16_t m0, m1, m2;

                std::getline(word, param, ',');
                m0 = std::stoi(param);
                std::getline(word, param, ',');
                m1 = std::stoi(param);
                std::getline(word, param, ',');
                m2 = std::stoi(param);
                Face f = {m0, m1, m2};
                faces.push_back(f);
            }
        }
        std::getline(infile, line);
        pos = line.find('=');
        key = line.substr(0, pos);
        value = line.substr(pos+1);
        if(key == "cells") {
            std::istringstream lineStream(value);
            std::string cell;

            while (std::getline(lineStream, cell, ';')) {
                std::istringstream word(cell);
                std::string param;
                uint16_t m0, m1, m2, m3;
                float mv;
                Material mat;

                std::getline(word, param, ',');
                m0 = std::stoi(param);
                std::getline(word, param, ',');
                m1 = std::stoi(param);
                std::getline(word, param, ',');
                m2 = std::stoi(param);
                std::getline(word, param, ',');
                m3 = std::stoi(param);
                std::getline(word, param, ',');
                mv = std::stof(param);
                std::getline(word, param, ',');
                mat = materials::decode((uint16_t) std::stoi(param));
                Cell c = {m0, m1, m2, m3, mv, mat};
                cells.push_back(c);
            }
        }
        std::getline(infile, line);
        pos = line.find('=');
        key = line.substr(0, pos);
        value = line.substr(pos+1);
        if(key == "boundary_count") {
            boundaryCount = std::stoi(value);
        }
        updateBaseline();
    }

}
