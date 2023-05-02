#ifndef __KNN_KERNEL_H__
#define __KNN_KERNEL_H__

#include <stdio.h>
#include <cassert>

// CUDA kernel to compute the distance matrix
// TODO change points from float3
__global__
inline void distance_matrix_kernel(float3* points, uint* ids, float* distances, uint num_points) {
    uint i = threadIdx.x + blockIdx.x * blockDim.x;
    uint j = threadIdx.y + blockIdx.y * blockDim.y;

    if (i < num_points && j < num_points) {
        ids[j * num_points + i] = i;

        if(i == j) {
            distances[j * num_points + i] = INFINITY;
            return;
        }

        float3 pi = points[i];
        float3 pj = points[j];

        float distance = sqrtf(
            (pi.x - pj.x) * (pi.x - pj.x) +
            (pi.y - pj.y) * (pi.y - pj.y) +
            (pi.z - pj.z) * (pi.z - pj.z));

        distances[j * num_points + i] = distance;
    }
}

__global__
inline void symmetric_distance_matrix_kernel(float3* points, uint* ids, float* distances, uint num_points) {
    uint i = blockIdx.x * blockDim.x + threadIdx.x;
    uint j = blockIdx.y * blockDim.y + threadIdx.y;

    if (i < num_points && j < num_points) {
        ids[j * num_points + i] = i;
        
        if(i > j) return;
        
        if(i == j) {
            distances[i * num_points + j] = INFINITY;
            distances[j * num_points + i] = INFINITY;
            return;
        }

        float3 pi = points[i];
        float3 pj = points[j];

        float distance = sqrtf(
            (pi.x - pj.x) * (pi.x - pj.x) +
            (pi.y - pj.y) * (pi.y - pj.y) +
            (pi.z - pj.z) * (pi.z - pj.z));
        
        distances[i * num_points + j] = distance;
        distances[j * num_points + i] = distance;
    }
}

#endif