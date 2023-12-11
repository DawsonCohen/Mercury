#ifndef __ELEMENT_H__
#define __ELEMENT_H__

#include "EvoDevo/Core/Base.h"

#include <Eigen/Core>

#include <iomanip>
#include <iostream>
#include <limits>
#include "material.h"

namespace EvoDevo {

    struct Mass {
        uint	id;

        Eigen::Vector3f 	pos;
        Eigen::Vector3f 	protoPos;
        Eigen::Vector3f		vel = Eigen::Vector3f::Zero();
        Material material = materials::bone;
        
        float mass = 1.0f;

        Mass() {}

        Mass(uint id, float x, float y, float z, float mass = 1.0f, Material mat = materials::bone) :
        id(id), material(mat), mass(mass)
        {
            pos = Eigen::Vector3f(x, y, z);
            protoPos = pos;
        }

        Mass(uint id, Eigen::Vector3f pos, Material mat, float mass = 1.0f) :
        id(id), pos(pos), protoPos(pos), material(mat), mass(mass)
        {}

        friend std::ostream& operator<<(std::ostream& out, const Mass& m) {
            out << std::setprecision(9);
            return out << m.id << "," << m.protoPos.x() << "," << m.protoPos.y() << "," << m.protoPos.z() << "," << m.mass << "," << (unsigned int) m.material.id << std::setprecision(6);
        }
    };

    struct Spring {
        uint16_t m0;
        uint16_t m1;
        float rest_length;
        float mean_length;
        Material material;

        friend std::ostream& operator<<(std::ostream& out, const Spring& s) {
            out << std::setprecision(9);
            return out << s.m0 << "," << s.m1 << "," << (double) s.rest_length << "," << (double) s.mean_length << "," <<
                    (unsigned int) s.material.id << std::setprecision(6);
        }
    };

    struct Face {
        uint16_t m0;
        uint16_t m1;
        uint16_t m2;

        friend std::ostream& operator<<(std::ostream& out, const Face& f) {
            out << std::setprecision(9);
            return out << f.m0 << "," << f.m1 << "," << f.m2 << std::setprecision(6);
        }
    };

    struct Cell {
        uint16_t m0;
        uint16_t m1;
        uint16_t m2;
        uint16_t m3;
        float mean_volume;
        Material material;

        friend std::ostream& operator<<(std::ostream& out, const Cell& c) {
            out << std::setprecision(9);
            return out << c.m0 << "," << c.m1 << "," << c.m2 << "," << c.m3 << "," <<
                    (double) c.mean_volume << "," << (uint16_t) c.material.encoding << std::setprecision(6);
        }
    };

    struct Element {
        std::vector<Mass> masses;
        std::vector<Spring> springs;
        std::vector<Face> faces;
        std::vector<Cell> cells;

        unsigned int boundaryCount = 0;

        float sim_time = 0;
        float total_sim_time = 0;

        void Update(Element e) {
            masses = e.masses;
            springs = e.springs;
            faces = e.faces;
            cells = e.cells;
        }

        friend void swap(Element& e1, Element& e2) {
            using std::swap;
            swap(e1.masses, e2.masses);
            swap(e1.springs, e2.springs);
            swap(e1.faces, e2.faces);
            swap(e1.cells,e2.cells);
            swap(e1.boundaryCount,e2.boundaryCount);
            swap(e1.sim_time,e2.sim_time);
            swap(e1.total_sim_time,e2.total_sim_time);
        }

        std::string ToOBJ() const {
            std::stringstream ss;
            for(const Mass& m : masses) {
                ss << "v " << m.pos.x() << " " << m.pos.y() << " " << m.pos.z() << std::endl;
            }

            for(const Face& f : faces) {
                ss << "f " << f.m0+1 << " " << f.m1+1 << " " << f.m2+1 << std::endl;
            }

            for(const Spring& e : springs) {
                ss << "l " << e.m0+1 << " " << e.m1+1 << std::endl;
            }
            return ss.str();
        }
        
    };

}

#endif