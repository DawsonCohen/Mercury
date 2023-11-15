#ifndef __DEVO_KERNEL_CUH__
#define __DEVO_KERNEL_CUH__

#include "vec_math.cuh"
#include <curand_kernel.h>
#include <stdint.h>
#include <assert.h>
#include <stdio.h>

__global__ void
inline randomIntegerPairKernel(int N, ushort2* results, int min_value, int max_value, unsigned int seed) {

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

struct DevoOptions {
    uint maxReplacedSprings;
	uint maxSprings;
    uint springsPerElement;
    uint massesPerElement;
    uint replacedSpringsPerElement;
    uint compositeCount;
};

__global__ void
inline replaceSprings(
    ushort2 *__restrict__ pairs,
    uint8_t *__restrict__ massMatEncodings, 
    float4 *__restrict__ massPos,
    float *__restrict__ Lbars,
    uint8_t *__restrict__ springMatEncodings,
    uint *__restrict__ sortedSpringIds,
    ushort2* newPairs,
    float4 *__restrict__ compositeMats,
    float time, DevoOptions opt
) {
	extern __shared__ float4 s_devo[];
	float4	*s_compositeMats = s_devo;

	int tid    = blockIdx.x * blockDim.x + threadIdx.x;
	int stride = blockDim.x;


    for(uint i = tid; i < opt.compositeCount; i += stride) {
		s_compositeMats[i] = __ldg(&compositeMats[i]);
	}

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
	uint8_t	matEncodingLeft, matEncodingRight, newMatEncoding;

	for(uint i = tid; 
        i < opt.maxReplacedSprings && (i / opt.replacedSpringsPerElement) * opt.springsPerElement + i < opt.maxSprings; 
        i+=stride)
    {
        elementId = (i / opt.replacedSpringsPerElement);
        massOffset = elementId * opt.massesPerElement;
        sortedSpringId = elementId * opt.springsPerElement + i;

        newPair = __ldg(&newPairs[i]);
        springId = __ldg(&sortedSpringIds[sortedSpringId]);

		pairs[springId] = newPair;
        
        left  = newPair.x;
		right = newPair.y;

        matEncodingLeft = massMatEncodings[left + massOffset];
        matEncodingLeft = massMatEncodings[right + massOffset];


        posLeft =  massPos[left+massOffset]; 
        posRight =  massPos[right+massOffset];

        posDiff = {
            posLeft.x - posRight.x,
            posLeft.y - posRight.y,
            posLeft.z - posRight.z
        };

        newMatEncoding = matEncodingLeft | matEncodingRight;
        newMat = s_compositeMats[newMatEncoding];

        rest_length = l2norm(posDiff);
        relative_change = newMat.y * sinf(newMat.z * time + newMat.w);
        
        Lbars[springId] = rest_length / (1+relative_change);
        springMatEncodings[springId] = newMatEncoding;

    }
}


#endif