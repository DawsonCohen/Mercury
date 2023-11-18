#ifndef __MATERIAL_H__
#define __MATERIAL_H__

#include <iostream>
#include <vector>
#include <math.h>
#include <cassert>

#define OMEGA (float) 1.0f * (2.0f*M_PI) // rad/sec
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
	uint32_t encoding;
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
	AIR,
	ADDUCTOR_MUSCLE0,
	ADDUCTOR_MUSCLE1,
	ADDUCTOR_MUSCLE2,
	ADDUCTOR_MUSCLE3,
	ADDUCTOR_MUSCLE4,
	ADDUCTOR_MUSCLE5,
	ADDUCTOR_MUSCLE6,
	ADDUCTOR_MUSCLE7,
	ADDUCTOR_MUSCLE8,
	ADDUCTOR_MUSCLE9,
	ABDCUTOR_MUSCLE0,
	ABDCUTOR_MUSCLE1,
	ABDCUTOR_MUSCLE2,
	ABDCUTOR_MUSCLE3,
	ABDCUTOR_MUSCLE4,
	ABDCUTOR_MUSCLE5,
	ABDCUTOR_MUSCLE6,
	ABDCUTOR_MUSCLE7,
	ABDCUTOR_MUSCLE8,
	ABDCUTOR_MUSCLE9,
	TISSUE,
	BONE,
	MATERIAL_FIRST = AIR,
	MATERIAL_LAST = BONE,
	MATERIAL_COUNT = MATERIAL_LAST+1,
	COMPOSITE_COUNT = MATERIAL_LAST*(MATERIAL_COUNT)/2+1
};

class materials {
	public:
	static constexpr Material air               = {0 , 0   , 0        , 0       , 0         , 0x01u << 0 , Color(  0.0f/255.0f ,   0.0f/255.0f ,   0.0f/255.0f , 0.00f)};
	static constexpr Material adductor_muscle0  = {1 , 50  , AMPLITUDE, OMEGA   , 0   	    , 0x01u << 1 , Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material adductor_muscle1  = {2 , 50  , AMPLITUDE, OMEGA   , M_PI/8    , 0x01u << 2 , Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material adductor_muscle2  = {3 , 50  , AMPLITUDE, OMEGA   , M_PI/4    , 0x01u << 3 , Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material adductor_muscle3  = {4 , 50  , AMPLITUDE, OMEGA   , 3*M_PI/8  , 0x01u << 4 , Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material adductor_muscle4  = {5 , 50  , AMPLITUDE, OMEGA   , M_PI/2    , 0x01u << 5 , Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material adductor_muscle5  = {6 , 50  , AMPLITUDE, OMEGA   , 5*M_PI/8  , 0x01u << 6 , Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material adductor_muscle6  = {7 , 50  , AMPLITUDE, OMEGA   , 3*M_PI/4  , 0x01u << 7 , Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material adductor_muscle7  = {8 , 50  , AMPLITUDE, OMEGA   , 7*M_PI/8  , 0x01u << 8 , Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material adductor_muscle8  = {9 , 50  , AMPLITUDE, OMEGA*2 , 0         , 0x01u << 9 , Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material adductor_muscle9  = {10, 50  , AMPLITUDE, OMEGA/2 , 0         , 0x01u << 10, Color(106.0f/255.0f , 211.0f/255.0f , 250.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle0  = {11, 50  , AMPLITUDE, OMEGA   , M_PI      , 0x01u << 11, Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle1  = {12, 50  , AMPLITUDE, OMEGA   , 9*M_PI/8  , 0x01u << 12, Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle2  = {13, 50  , AMPLITUDE, OMEGA   , 5*M_PI/4  , 0x01u << 13, Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle3  = {14, 50  , AMPLITUDE, OMEGA   , 11*M_PI/8 , 0x01u << 14, Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle4  = {15, 50  , AMPLITUDE, OMEGA   , 3*M_PI/2  , 0x01u << 15, Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle5  = {16, 50  , AMPLITUDE, OMEGA   , 13*M_PI/8 , 0x01u << 16, Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle6  = {17, 50  , AMPLITUDE, OMEGA   , 7*M_PI/4  , 0x01u << 17, Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle7  = {18, 50  , AMPLITUDE, OMEGA   , 15*M_PI/8 , 0x01u << 18, Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle8  = {19, 50  , AMPLITUDE, OMEGA*2 , M_PI      , 0x01u << 19, Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material abductor_muscle9  = {20, 50  , AMPLITUDE, OMEGA/2 , M_PI      , 0x01u << 20, Color( 89.0f/255.0f , 217.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material tissue            = {21, 40  , 0        , OMEGA   , 0         , 0x01u << 21, Color(217.0f/255.0f , 102.0f/255.0f ,  78.0f/255.0f , 0.00f)};
	static constexpr Material bone              = {22, 100 , 0        , OMEGA   , 0         , 0x01u << 22, Color(240.0f/255.0f , 209.0f/255.0f ,  98.0f/255.0f , 0.00f)};

	static Material matLookup(unsigned int mat) {
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
		unsigned int matId = rand() % MATERIAL_COUNT;
		return matLookup(matId);
	}

	static uint8_t get_composite_id(uint8_t mat1, uint8_t mat2) {
    	static bool composite_id_initialized = false;
    	static uint8_t compositeIds[MATERIAL_COUNT-1][MATERIAL_COUNT-1];
    	if(!composite_id_initialized) {
    		uint8_t count = 1;
    		for(unsigned int i = 0; i < MATERIAL_COUNT-1; i ++) {
    			compositeIds[i][i] = count;
    			count++;
    		}

			for(unsigned int i = 0; i < MATERIAL_COUNT-1; i++) {
        		for(unsigned int j = i+1; j < MATERIAL_COUNT-1; j++) {
        			compositeIds[i][j] = compositeIds[j][i] = count;
        			count++;
        		}
        	}
        	composite_id_initialized = true;
        }

		if(mat1 == materials::air.id || mat2 == materials::air.id) return materials::air.id;
        return compositeIds[mat1-1][mat2-1];
    }

    static Material avg(Material m1, Material m2) {
		Material result{0,0,0,0,0,0};
		
		result.id = get_composite_id(m1.id, m2.id);
		result.k = (m1.k + m2.k) / 2.0;
		result.dL0 = (m1.dL0 + m2.dL0) / 2.0;
		result.omega = (m1.omega + m2.omega) / 2.0;
		result.phi = (m1.phi + m2.phi) / 2.0;
		result.encoding = m1.encoding | m2.encoding;
		result.color = (m1.color + m2.color) / 2.0;

		return result;
	}

	static Material id_lookup(uint8_t id) {
        static bool composite_initialized = false;
        static Material compositeMaterials[COMPOSITE_COUNT];

        if (!composite_initialized) {  	
        	for(unsigned int i = 0; i < MATERIAL_COUNT; i++) {
        		Material m = matLookup(i);
        		compositeMaterials[m.id] = m;
        	}

			for(unsigned int i = 1; i < MATERIAL_COUNT; i++) {
				Material mi = matLookup(i);
				for(uint8_t j = i+1; j < MATERIAL_COUNT; j++) {
					Material m = avg(mi, matLookup(j));
					compositeMaterials[m.id] = m;
				}
			}
            composite_initialized = true;
        }
        return compositeMaterials[id];
    }

	static Material decode(uint32_t encoding) {
        static bool composite_initialized = false;
        static Material compositeMaterials[COMPOSITE_COUNT];

        if (!composite_initialized) {
			compositeMaterials[0] = materials::air;

			Material mi, m;
			unsigned int idx;
			for(uint32_t i = 1; i < MATERIAL_COUNT; i++) {
				mi = matLookup(i);
				idx = (i*(i-1)/2);
				compositeMaterials[idx+1] = mi;
				for(uint32_t j = i+1; j < MATERIAL_COUNT; j++) {
					m = avg(mi, matLookup(j));
					idx = (j*(j-1)/2)+i;
					compositeMaterials[idx+1] = m;
				}
			}
            composite_initialized = true;
        }
		int idx = encodedCompositeIdx(encoding);

		return compositeMaterials[idx];
    }

	static int encodedCompositeIdx(uint32_t encoding) {
		unsigned int idx[2] = {0,0},matIdx,
					 count = 0,
					 bitmask = 0x01u;
        for(unsigned int i = 0; i < COMPOSITE_COUNT; i++) {
            if(encoding & bitmask) {
                idx[count] = i;
				count++;
				if(i == 0 || count == 2) break;
            }
            bitmask <<= 1;
        }
        if(idx[0] == 0) {
            matIdx = 0;
        } else if(idx[1] == 0) {
            matIdx = 1 + idx[0]*(idx[0]-1)/2;
        } else {
            matIdx = 1 + idx[1]*(idx[1]-1)/2 + idx[0];
        }

		return matIdx;
	}
};

#endif