#ifndef __MATERIAL_H__
#define __MATERIAL_H__

#include <iostream>
#include <vector>
#include <math.h>

#define OMEGA (float) 2.0f * (2.0f*M_PI) // rad/sec
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
	ADDUCTOR_MUSCLE0 = 0,
	ADDUCTOR_MUSCLE1 = 1,
	ADDUCTOR_MUSCLE2 = 2,
	ADDUCTOR_MUSCLE3 = 3,
	ADDUCTOR_MUSCLE4 = 4,
	ADDUCTOR_MUSCLE5 = 5,
	ADDUCTOR_MUSCLE6 = 6,
	ADDUCTOR_MUSCLE7 = 7,
	ADDUCTOR_MUSCLE8 = 8,
	ADDUCTOR_MUSCLE9 = 9,
	ABDCUTOR_MUSCLE0 = 10,
	ABDCUTOR_MUSCLE1 = 11,
	ABDCUTOR_MUSCLE2 = 12,
	ABDCUTOR_MUSCLE3 = 13,
	ABDCUTOR_MUSCLE4 = 14,
	ABDCUTOR_MUSCLE5 = 15,
	ABDCUTOR_MUSCLE6 = 16,
	ABDCUTOR_MUSCLE7 = 17,
	ABDCUTOR_MUSCLE8 = 18,
	ABDCUTOR_MUSCLE9 = 19,
	TISSUE = 20,
	BONE = 21,
	AIR = 22,
	MATERIAL_FIRST = ADDUCTOR_MUSCLE0,
	MATERIAL_LAST = AIR,
	ACTIVE_MATERIAL_COUNT = MATERIAL_LAST,
	MATERIAL_COUNT = MATERIAL_LAST+1,
	COMPOSITE_COUNT = MATERIAL_LAST*(MATERIAL_COUNT)/2+1
};

class materials {
	public:
	static constexpr Material adductor_muscle0  = {0, 50  , AMPLITUDE, OMEGA, 0   , Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material adductor_muscle1  = {1, 50  , AMPLITUDE, OMEGA, M_PI/8   , Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material adductor_muscle2  = {2, 50  , AMPLITUDE, OMEGA, M_PI/4   , Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material adductor_muscle3  = {3, 50  , AMPLITUDE, OMEGA, 3*M_PI/8   , Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material adductor_muscle4  = {4, 50  , AMPLITUDE, OMEGA, M_PI/2   , Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material adductor_muscle5  = {5, 50  , AMPLITUDE, OMEGA, 5*M_PI/8   , Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material adductor_muscle6  = {6, 50  , AMPLITUDE, OMEGA, 3*M_PI/4   , Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material adductor_muscle7  = {7, 50  , AMPLITUDE, OMEGA, 7*M_PI/8   , Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material adductor_muscle8  = {8, 50  , AMPLITUDE, OMEGA*2, 0   , Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material adductor_muscle9  = {9, 50  , AMPLITUDE, OMEGA/2, 0   , Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle0  = {10,50  , AMPLITUDE, OMEGA, M_PI, Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle1  = {11,50  , AMPLITUDE, OMEGA, 9*M_PI/8, Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle2  = {12,50  , AMPLITUDE, OMEGA, 5*M_PI/4, Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle3  = {13,50  , AMPLITUDE, OMEGA, 11*M_PI/8, Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle4  = {14,50  , AMPLITUDE, OMEGA, 3*M_PI/2, Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle5  = {15,50  , AMPLITUDE, OMEGA, 13*M_PI/8, Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle6  = {16,50  , AMPLITUDE, OMEGA, 7*M_PI/4, Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle7  = {17,50  , AMPLITUDE, OMEGA, 15*M_PI/8, Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle8  = {18,50  , AMPLITUDE, OMEGA*2, M_PI , Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle9  = {19,50  , AMPLITUDE, OMEGA/2, M_PI , Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material tissue            = {20,40  , 0        , OMEGA, 0   , Color(217.0f/255.0f , 102.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material bone              = {21,100 , 0        , OMEGA, 0   , Color(240.0f/255.0f , 209.0f/255.0f ,  98.0f/255.0f , 0.00f)};
	static constexpr Material air               = {22, 0   , 0        , 0    , 0   , Color(  0.0f/255.0f ,   0.0f/255.0f ,   0.0f/255.0f ,  0.0f)};

	static Material matLookup(uint mat) {
		switch(mat){
		case ADDUCTOR_MUSCLE0:
			return adductor_muscle0;
		case ADDUCTOR_MUSCLE1:
			return adductor_muscle1;
		case ADDUCTOR_MUSCLE2:
			return adductor_muscle2;
		case ADDUCTOR_MUSCLE3:
			return adductor_muscle3;
		case ADDUCTOR_MUSCLE4:
			return adductor_muscle4;
		case ADDUCTOR_MUSCLE5:
			return adductor_muscle5;
		case ADDUCTOR_MUSCLE6:
			return adductor_muscle6;
		case ADDUCTOR_MUSCLE7:
			return adductor_muscle7;
		case ADDUCTOR_MUSCLE8:
			return adductor_muscle8;
		case ADDUCTOR_MUSCLE9:
			return adductor_muscle9;
		case ABDCUTOR_MUSCLE0:
			return abductor_muscle0;
		case ABDCUTOR_MUSCLE1:
			return abductor_muscle1;
		case ABDCUTOR_MUSCLE2:
			return abductor_muscle2;
		case ABDCUTOR_MUSCLE3:
			return abductor_muscle3;
		case ABDCUTOR_MUSCLE4:
			return abductor_muscle4;
		case ABDCUTOR_MUSCLE5:
			return abductor_muscle5;
		case ABDCUTOR_MUSCLE6:
			return abductor_muscle6;
		case ABDCUTOR_MUSCLE7:
			return abductor_muscle7;
		case ABDCUTOR_MUSCLE8:
			return abductor_muscle8;
		case ABDCUTOR_MUSCLE9:
			return abductor_muscle9;
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

	static uint8_t get_encoding(uint8_t mat1, uint8_t mat2) {
    	static bool encding_initialized = false;
    	static uint8_t matEncoding[MATERIAL_LAST][MATERIAL_LAST];
    	if(!encding_initialized) {
    		uint8_t count = 0;
    		for(uint i = 0; i < MATERIAL_COUNT; i ++) {
    			matEncoding[i][i] = count;
    			count++;
    		}

			for(uint i = 0; i < MATERIAL_LAST-1; i++) {
        		for(uint j = i+1; j < MATERIAL_LAST; j++) {
        			matEncoding[i][j] = matEncoding[j][i] = count;
        			count++;
        		}
        	}
        	encding_initialized = true;
        }

        return matEncoding[mat1][mat2];
    }

    static Material avg(Material m1, Material m2) {
		Material result{0,0,0,0,0};
		
		result.id = get_encoding(m1.id, m2.id);
		result.k = (m1.k + m2.k) / 2.0;
		result.dL0 = (m1.dL0 + m2.dL0) / 2.0;
		result.omega = (m1.omega + m2.omega) / 2.0;
		result.phi = (m1.phi + m2.phi) / 2.0;
		result.color = (m1.color + m2.color) / 2.0;

		return result;
	}

	static Material decode(uint8_t encoding) {
        static bool composite_initialized = false;
        static Material compositeMaterials[COMPOSITE_COUNT];

        if (!composite_initialized) {  	
        	for(uint i = 0; i < MATERIAL_COUNT; i++) {
        		Material m = matLookup(i);
        		compositeMaterials[m.id] = m;
        	}

			for(uint i = 1; i < MATERIAL_LAST-1; i++) {
				Material mi = matLookup(i);
				for(uint8_t j = i+1; j < MATERIAL_LAST; j++) {
					Material m = avg(mi, matLookup(j));
					compositeMaterials[m.id] = m;
				}
			}
            composite_initialized = true;
        }
        return compositeMaterials[encoding];
    }
};

#endif