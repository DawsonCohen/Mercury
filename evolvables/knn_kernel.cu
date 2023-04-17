#ifndef __KNN_KERNEL_H__
#define __KNN_KERNEL_H__

// CUDA kernel to compute the k-nearest neighbors
__global__ 
inline void compute_k_nearest_neighbors(float3* points, uint* knn_indices, float* knn_distances, int points_per_group, int KNN) {
    // extern __shared__ float s[];
	// float  *s_distances = s;
	// uint   *s_indices = (uint*) &s_distances[blockDim.x * KNN];

    uint tid = threadIdx.x;
    uint offset = blockIdx.x * points_per_group;
    uint stride = blockDim.x;

    for(uint i = tid; i < points_per_group; i += stride) {
        float3 p1 = points[i+offset];
        uint start = (i+offset)*KNN;

        for(uint j = 0; j < KNN; j++) {
            knn_distances[start + j] = 10000.0f;
            knn_indices[start + j] = j;
        }

        for(uint j = 0; j < points_per_group; j++) {
            if (i == j) {
                continue;
            }

            float3 p2 = points[j+offset];
            float distance = sqrtf(
                (p1.x - p2.x) * (p1.x - p2.x) +
                (p1.y - p2.y) * (p1.y - p2.y) +
                (p1.z - p2.z) * (p1.z - p2.z));

            for (int k = 0; k < KNN; k++) {
                if (distance < knn_distances[start + k]) { // good
                    for (int l = KNN - 2; l >= k; l--) { // good
                        knn_distances[start+ l + 1] = knn_distances[start + l];
                        knn_indices[start + l + 1] = knn_indices[start + l];
                    }
                    knn_distances[start + k] = distance;
                    knn_indices[start + k] = j;
                    break;
                }
            }

        }

        // for(uint j = 0; j < KNN; j++) {
        //     knn_indices[(i + offset) * KNN + j] = s_indices[tid * KNN + j];
        //     knn_distances[(i + offset) * KNN + j ] = s_distances[tid * KNN + j];
        // }
    }
}

#endif