#include "vec_math.cuh"
#include "material.h"
#include <curand_kernel.h>
#include <stdint.h>
#include <assert.h>
#include <stdio.h>

struct DevoOptions {
    uint maxReplacedSprings;
	uint maxSprings;
    uint springsPerElement;
    uint massesPerElement;
    uint replacedSpringsPerElement;
    uint compositeCount;
};

struct DeviceData {
	// MASS DATA
	float    *dPos, *dNewPos, *dVel;
	uint32_t *dMassMatEncodings;

	// SPRING DATA
	ushort	 *dPairs;
	uint32_t *dSpringMatEncodings;
	uint8_t  *dSpringMatIds;
	float	 *dLbars;
	uint     *dSpringIDs;
	float	 *dSpringStresses;
	
	// SPRING DEVO DATA
	ushort   *dRandomPairs;
	uint     *dSpringIDs_Sorted;
	float	 *dSpringStresses_Sorted;

	// FACE DATA
	ushort	 *dFaces;
	
	// CELL DATA
	ushort	 *dCells;
	float	 *dVbars;
	float	 *dMats;
	float	 *dCellStresses;
};

__constant__ float4 compositeMats_encoding[COMPOSITE_COUNT];
__constant__ DevoOptions cDevoOpt;

void setCompositeMats_encoding(float* compositeMats, uint count) {
    cudaMemcpyToSymbol(compositeMats_encoding, compositeMats, sizeof(float)*4*count);
}

void setDevoOpts(DevoOptions devoOpts) {
    cudaMemcpyToSymbol(cDevoOpt, &devoOpts, sizeof(DevoOptions));
}

__global__ void randomIntegerPairKernel(int N, ushort2* results, int min_value, int max_value, unsigned int seed) {

    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x;
    
    curandState state;
    curand_init(seed, tid, 0, &state);
    
    for(uint i = tid; i < N; i += stride) {
        ushort random_integer_x  = curand(&state) % (max_value - min_value + 1) + min_value;
        ushort random_integer_y  = curand(&state) % (max_value - min_value + 1) + min_value;
        while(random_integer_x == random_integer_y) {
            random_integer_y  = curand(&state) % (max_value - min_value + 1) + min_value;
        }
        results[i].x = random_integer_x;
        results[i].y = random_integer_y;
    }
}

void getRandomInterPairs(int N, ushort* randomPairs, int min_value, int max_value, unsigned int seed) {
    int threadsPerBlock = 256;
    int blocksPerGrid = (N + threadsPerBlock - 1) / threadsPerBlock;
    randomIntegerPairKernel<<<blocksPerGrid, threadsPerBlock>>>(N, (ushort2*) randomPairs, min_value, max_value, seed);
}

__global__ void replaceSprings(
    ushort2 *__restrict__ pairs,
    uint32_t *__restrict__ massMatEncodings, 
    float4 *__restrict__ massPos,
    float *__restrict__ Lbars,
    uint32_t *__restrict__ springMatEncodings,
    uint *__restrict__ sortedSpringIds,
    ushort2* newPairs,
    float time
) {
	int tid    = blockIdx.x * blockDim.x + threadIdx.x;
	int stride = blockDim.x;

    uint    elementId,
            massOffset,
            sortedSpringId,
            springId;
	ushort2	newPair;
	ushort	left, right;
    float4  posLeft, posRight;
    float4  newMat;
    float3  posDiff;
    float   rest_length,
            relative_change;
	uint32_t	matEncodingLeft, matEncodingRight, newMatEncoding;
    uint i, j;
    uint idx[2] = {0,0},matIdx,
        count,bitmask;

	for(i = tid; 
        i < cDevoOpt.maxReplacedSprings && (i / cDevoOpt.replacedSpringsPerElement) * cDevoOpt.springsPerElement + i < cDevoOpt.maxSprings; 
        i+=stride)
    {
        elementId = (i / cDevoOpt.replacedSpringsPerElement);
        massOffset = elementId * cDevoOpt.massesPerElement;
        sortedSpringId = elementId * cDevoOpt.springsPerElement + i;

        newPair = __ldg(&newPairs[i]);
        springId = __ldg(&sortedSpringIds[sortedSpringId]);

		pairs[springId] = newPair;
        
        left  = newPair.x;
		right = newPair.y;

        matEncodingLeft = massMatEncodings[left + massOffset];
        matEncodingRight = massMatEncodings[right + massOffset];


        posLeft =  massPos[left+massOffset]; 
        posRight =  massPos[right+massOffset];

        posDiff = {
            posLeft.x - posRight.x,
            posLeft.y - posRight.y,
            posLeft.z - posRight.z
        };

        newMatEncoding = matEncodingLeft | matEncodingRight;

        count = 0;
        bitmask = 0x01u;
        for(j = 0; j < COMPOSITE_COUNT; j++) {
            if(newMatEncoding & bitmask) {
                idx[count] = j;
                count++;
				if(j == 0 || count == 2) break;
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

        newMat = compositeMats_encoding[matIdx];

        rest_length = l2norm(posDiff);
        relative_change = newMat.y * sinf(newMat.z * time + newMat.w);

        Lbars[springId] = rest_length / (1+relative_change);
        springMatEncodings[springId] = newMatEncoding;

    }
}

void devoBodies(DeviceData deviceData, DevoOptions opt, float time) {
    int threadsPerBlock = 256;
    int blocksPerGrid = (opt.maxReplacedSprings + threadsPerBlock - 1) / threadsPerBlock;
    
    replaceSprings<<<blocksPerGrid, threadsPerBlock>>>(
        (ushort2*) deviceData.dPairs,
        deviceData.dMassMatEncodings,
        (float4*) deviceData.dPos,
        deviceData.dLbars,
        deviceData.dSpringMatEncodings,
        deviceData.dSpringIDs_Sorted,
        (ushort2*) deviceData.dRandomPairs,
        time
    );
}