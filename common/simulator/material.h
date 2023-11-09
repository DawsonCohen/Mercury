#ifndef __MATERIAL_H__
#define __MATERIAL_H__

#include <iostream>
#include <vector>
#include <math.h>

#define OMEGA (float) 0.1f * (2.0f*M_PI) // rad/sec
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

	Color operator+(const Color& other) const {
        return Color(r + other.r, g + other.g, b + other.b, a + other.a);
    }

	Color operator-(const Color& other) const {
        return Color(r - other.r, g - other.g, b - other.b, a - other.a);
    }

	constexpr Color operator/(float scalar) {
		float rscalar = 1 / scalar;
        return Color(r * rscalar, g * rscalar, b * rscalar, a * rscalar);
    }
};

// k - spring constant 
// spring oscillation L0 = L0 + dL0*sin(wt+phi)
struct Material {
	uint8_t id;
	float k;
	float dL0;
	float omega;
	float phi;
	uint8_t encoding;
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
		Material air{0,0,0,0,0, 0x01};
		Material result{0,0,0,0,0, 0x01};
		uint count = 1;
		for(const Material& m : M) {
			if(m == air) return air;
			if(count == 1) {
				result = m;
				count++;
				continue;
			}
			result.k = result.k + (m.k - result.k) / (float) count;
			result.dL0 = result.dL0 + (m.dL0 - result.dL0) / (float) count;
			result.omega = result.omega + (m.omega - result.omega) / (float) count;
			result.phi = result.phi + (m.phi - result.phi) / (float) count;
			result.color = m.color;
			result.encoding = result.encoding | m.encoding;
			result.color = result.color + (m.color - result.color) / (float) count;
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
	AIR = 0,
	ADDUCTOR_MUSCLE = 1,
	ABDCUTOR_MUSCLE = 2,
	TISSUE = 3,
	BONE = 4,
	MATERIAL_FIRST = AIR,
	MATERIAL_LAST = BONE,
	ACTIVE_MATERIAL_COUNT = MATERIAL_LAST,
	MATERIAL_COUNT = MATERIAL_LAST+1
};

class materials {
	public:
	static constexpr Material air               = {0, 0   , 0        , 0    , 0   , 0x01u, Color(  0.0f/255.0f ,   0.0f/255.0f ,   0.0f/255.0f ,  0.0f)};
	static constexpr Material adductor_muscle   = {1, 50  , AMPLITUDE, OMEGA, 0   , 0x02u, Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle   = {2, 50  , AMPLITUDE, OMEGA, M_PI, 0x04u, Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material tissue            = {3, 40  , 0        , OMEGA, 0   , 0x08u, Color(217.0f/255.0f , 102.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material bone              = {4, 1000, 0        , OMEGA, 0   , 0x10u, Color(240.0f/255.0f , 209.0f/255.0f ,  98.0f/255.0f , 0.00f)};

	static Material materialCombos[1 << MATERIAL_COUNT];

	static Material matLookup(uint mat) {
		switch(mat){
		case ADDUCTOR_MUSCLE:
			return adductor_muscle;
		case ABDCUTOR_MUSCLE:
			return abductor_muscle;
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

	static Material decode(uint8_t encoding) {
        static bool initialized = false;
        static Material compositeMaterials[1 << MATERIAL_COUNT];

        if (!initialized) {
			for(uint i = 0; i < (1 << MATERIAL_COUNT); i++) {
				std::vector<Material> materials;
				for(uint8_t j = 0; j < MATERIAL_COUNT; j++) {
					if(i & (0x01u << j)) materials.push_back(matLookup(j));
				}
				compositeMaterials[i] = Material::avg(materials);
			}
            initialized = true;
        }


        return compositeMaterials[encoding];
    }
};

#endif