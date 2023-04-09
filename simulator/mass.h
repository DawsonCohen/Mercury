#ifndef __Mass_H__
#define __Mass_H__

#include <stdint.h>
#include <iostream>
#include <Eigen/Core>

#define OMEGA (float) 4.0f
#define AMPLITUDE .14f

struct Color {
	float r;
	float g;
	float b;
	float a;

	constexpr Color(float r, float g, float b, float a) :
	r(r), g(g), b(b), a(a)
	{}

	constexpr Color operator*(float scalar) {
        return Color(r * scalar, g * scalar, b * scalar, a * scalar);
    }

	constexpr Color operator/(float scalar) {
		float rscalar = 1 / scalar;
        return Color(r * rscalar, g * rscalar, b * rscalar, a * rscalar);
    }
};

// k - spring constant 
// spring oscillation L0 = L0 + dL0*sin(wt+phi)
struct Material {
	uint id;
	float k;
	float dL0;
	float omega;
	float phi;
	char encoding;
	Color color = {0.0f, 0.0f, 0.0f, 1.0f};

	bool operator == (const Material& src) const {
		return k==src.k && dL0 == src.dL0 && omega == src.omega && phi == src.phi;
	}

	bool operator < (const Material& src) const {
		if(k < src.k) return true;
		else if(dL0 < src.dL0) return true;
		else if(omega < src.omega) return true;
		else if(phi < src.phi) return true;
		return false;
	}

	bool operator != (const Material& src) const {
		return !(*this == src) ;
	}

	static Material avg(std::vector<Material> M) {
		Material air{0,0,0,0,0, 0x00};
		Material result{0,0,0,0,0, 0x00};
		uint count = 0;
		for(const Material& m : M) {
			if(m == air) continue;
			if(count == 0) {
				result = m; continue;
			}
			result.k = result.k + (m.k - result.k) / count;
			result.dL0 = result.dL0 + (m.dL0 - result.dL0) / count;
			result.omega = result.omega + (m.omega - result.omega) / count;
			result.phi = result.phi + (m.phi - result.phi) / count;
			result.color = m.color;
			result.encoding += m.encoding;
			// result.color = result.color + (m.color - result.color) * (1.0f / (float) count);
			count++;
		}

		return result;
	}

	std::string to_string() const {
		return std::to_string(k) + ", " +
				std::to_string(dL0) + ", " +
				std::to_string(omega) + ", " +
				std::to_string(phi) + ", " +
				std::to_string(color.r) + ", " +
				std::to_string(color.g) + ", " +
				std::to_string(color.b) + ", " +
				std::to_string(color.a);
	}
};

enum MaterialOption {
	AGONIST_MUSCLE = 0,
	ANTAGOINST_MUSCLE = 1,
	TISSUE = 2,
	BONE = 3,
	AIR = 4,
	MATERIAL_FIRST = AGONIST_MUSCLE,
	MATERIAL_LAST = AIR,
	ACTIVE_MATERIAL_COUNT = MATERIAL_LAST,
	MATERIAL_COUNT = MATERIAL_LAST+1
};

class materials {
	public:
	static constexpr Material agonist_muscle    = {0, 5000 , AMPLITUDE, OMEGA, 0   , 0x01, Color(32.0f , 212.0f, 82.0f , 1.0f)*1.0f/255.0f};
	static constexpr Material antagonist_muscle = {1, 5000 , AMPLITUDE, OMEGA, M_PI, 0x02, Color(250.0f, 112.0f, 66.0f , 1.0f)*1.0f/255.0f};
	static constexpr Material tissue            = {2, 4000 , 0        , OMEGA, 0   , 0x04, Color(169.0f, 32.0f , 212.0f, 1.0f)*1.0f/255.0f};
	static constexpr Material bone              = {3, 10000, 0        , OMEGA, 0   , 0x08, Color(245.0f, 231.0f, 54.0f , 1.0f)*1.0f/255.0f};
	static constexpr Material air               = {4, 0    , 0        , 0    , 0   , 0x00, Color(0.0f  , 0.0f  , 0.0f  , 0.0f)*1.0f/255.0f};

	static Material matLookup(uint mat) {
		switch(mat){
		case AGONIST_MUSCLE:
			return agonist_muscle;
		case ANTAGOINST_MUSCLE:
			return antagonist_muscle;
		case TISSUE:
			return tissue;
		case BONE:
			return bone;
		case AIR:
		default:
			return air;
		}
	};

	static Material random() {
		uint matId = rand() % MATERIAL_COUNT;
		return matLookup(matId);
	}
};

struct Mass {
	uint	id;

	Eigen::Vector3f 	pos;
	Eigen::Vector3f 	protoPos;
	Material material = materials::bone;
	
	float mass = 1.0f;
	Eigen::Vector3f vel   = Eigen::Vector3f::Zero();
	Eigen::Vector3f acc   = Eigen::Vector3f::Zero();
	Eigen::Vector3f force = Eigen::Vector3f::Zero();

	bool	active = false;

	Mass() {}

	Mass(uint id, float x, float y, float z, float mass = 1.0f) :
	id(id), mass(mass)
	 {
		pos = Eigen::Vector3f(x, y, z);
		protoPos = pos;
	}

	Mass(uint id, Eigen::Vector3f pos, Material mat, float mass = 1.0f) :
	id(id), pos(pos), protoPos(pos), material(mat), mass(mass)
	{}
};

#endif