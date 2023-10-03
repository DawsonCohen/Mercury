#ifndef __DEVO_KERNEL_CUH__
#define __DEVO_KERNEL_CUH__

#include <curand_kernel.h>
#include <stdint.h>

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
};

__global__ void
inline replaceSprings(ushort2 *__restrict__ pairs, uint8_t *__restrict__ massMatEncodings, 
            uint8_t *__restrict__ springMatEncodings, uint *__restrict__ sortedSpringIds,
            ushort2* newPairs, DevoOptions opt) {

	int tid    = threadIdx.x * blockDim.x + threadIdx.x;
	int stride = blockDim.x;

	__syncthreads();

    uint    elementId,
            massOffset,
            sortedSpringId,
            springId;
	ushort2	newPair;
	ushort	left, right;
	uint8_t	matEncodingLeft, matEncodingRight;

	for(uint i = tid; 
        i < opt.maxReplacedSprings && (i / opt.replacedSpringsPerElement) * opt.springsPerElement < opt.maxSprings; 
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

        springMatEncodings[springId] = matEncodingLeft | matEncodingRight;
    }
}


#endif