#include "triangulation.h"
#include "knn_kernel.cu"
#include "mass.h"

#include <iostream>
#include <sys/stat.h>
#include <chrono>

#include <cub/device/device_segmented_radix_sort.cuh>
#include <cub/device/device_run_length_encode.cuh>

#define BLOCK_SIZE 16

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess) 
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}

namespace Triangulation {

void key_value_sort(float* d_keys_in, float* d_keys_out, uint16_t* d_values_in, uint16_t* d_values_out, uint16_t count) {
    // Determine number of segments
    int num_segments = count;

    // Allocate memory on device for offsets
    int* h_offsets = new int[num_segments+1];
    for(uint16_t i = 0; i < num_segments+1; i++) {
        h_offsets[i] = (int) count*i;
    }

    int* d_offsets;
    cudaMalloc(&d_offsets, (num_segments+1) * sizeof(int));
    cudaMemcpy(d_offsets, h_offsets, (num_segments+1) *sizeof(int), cudaMemcpyHostToDevice);

    // Determine temporary storage size
    void* d_temp_storage = NULL;
    size_t temp_storage_bytes = 0;
    cub::DeviceSegmentedRadixSort::SortPairs(
        d_temp_storage, temp_storage_bytes,
        d_keys_in, d_keys_out, d_values_in, d_values_out,
        count*count, num_segments, d_offsets, d_offsets+1);

    // Allocate temporary storage
    cudaMalloc(&d_temp_storage, temp_storage_bytes);

    // Run sorting operation
    cub::DeviceSegmentedRadixSort::SortPairs(
        d_temp_storage, temp_storage_bytes,
        d_keys_in, d_keys_out, d_values_in, d_values_out,
        count*count, num_segments, d_offsets, d_offsets+1);

    delete[] h_offsets;
    cudaFree(d_offsets);
    cudaFree(d_temp_storage);
}

std::vector<Simplex::Edge> KNN(const std::vector<Mass>& masses, uint16_t K)
{
    unsigned int num_masses = masses.size();
    
    // CPU data
    float* h_points = new float[num_masses * 3];
    uint16_t* h_ids = new uint16_t[num_masses * num_masses];
    float* h_distances = new float[num_masses * num_masses];

    for (unsigned int i = 0; i < num_masses; i++)
    {
        h_points[3*i]   = masses[i].protoPos[0];
        h_points[3*i+1] = masses[i].protoPos[1];
        h_points[3*i+2] = masses[i].protoPos[2];
    }

    // GPU data 
    float* d_points;
    cudaMalloc(&d_points, num_masses * 3 * sizeof(float));

    uint16_t* d_ids;
    uint16_t* d_ids_sorted;
    cudaMalloc(&d_ids, num_masses * num_masses * sizeof(uint16_t));
    cudaMalloc(&d_ids_sorted, num_masses * num_masses * sizeof(uint16_t));

    float* d_distances;
    float* d_distances_sorted;
    cudaMalloc(&d_distances, num_masses * num_masses * sizeof(float));
    cudaMalloc(&d_distances_sorted, num_masses * num_masses * sizeof(float));

    // Copy data to device memory
    cudaMemcpy(d_points, h_points, num_masses * 3 * sizeof(float), cudaMemcpyHostToDevice);

    // Compute k-nearest neighbors
    uint16_t block_count = (num_masses + BLOCK_SIZE - 1) / BLOCK_SIZE;
    dim3 num_blocks = {block_count, block_count};

    // Execute the kernel
    symmetric_distance_matrix_kernel<<<num_blocks, {BLOCK_SIZE, BLOCK_SIZE}>>>((float3*) d_points, (ushort*) d_ids, d_distances, num_masses);
    gpuErrchk( cudaPeekAtLastError() );
    cudaDeviceSynchronize();

    key_value_sort(d_distances, d_distances_sorted, d_ids, d_ids_sorted, num_masses);

    cudaMemcpy(h_distances, d_distances_sorted, num_masses * num_masses * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_ids, d_ids_sorted, num_masses * num_masses * sizeof(uint16_t), cudaMemcpyDeviceToHost);
    
    std::vector<std::vector<std::pair<uint16_t,float>>> KNN(
        num_masses, std::vector<std::pair<uint16_t, float>>(K));

    for(uint16_t i = 0; i < num_masses; i++) {
        for (uint16_t j = 0; j < K; j++) {
            KNN[i][j].first = h_ids[i*num_masses + j];
            KNN[i][j].second = h_distances[i*num_masses + j];
        }
    }
    
    // Free memory
    delete[] h_points;
    delete[] h_ids;
    delete[] h_distances;

    cudaFree(d_points);
    cudaFree(d_ids);
    cudaFree(d_ids_sorted);
    cudaFree(d_distances);
    cudaFree(d_distances_sorted);

    std::vector<Simplex::Edge> triangulation;

    for (uint16_t i = 0; i < masses.size(); i++) {
        auto neighbors = KNN[i];
        Material mat1 = masses[i].material,
                 mat2, mat;

        for (auto neighbor : neighbors) {
            Simplex::Edge edge = {i, neighbor.first, neighbor.second};
            triangulation.push_back(edge);
        }
    }

    return triangulation;
}


std::vector<Simplex::Edge> KNN_CPU(const std::vector<Mass>& masses, uint16_t K)
{
    unsigned int num_masses = masses.size();

    std::vector<std::vector<float>> distances(num_masses, std::vector<float>(num_masses));
    for(size_t i = 0; i < num_masses; i++) {
        Eigen::Vector3f p1 = masses[i].pos;
        for(size_t j = 0; j < num_masses; j++) {
            Eigen::Vector3f p2 = masses[j].pos;
            distances[i][j] = distances[j][i] = (p1-p2).norm();
        }
    }

    std::vector<std::vector<std::pair<uint16_t,float>>> KNN(
        num_masses, std::vector<std::pair<uint16_t, float>>(K));

    for(size_t i = 0; i < distances.size(); i++) {
        std::vector<std::pair<uint16_t, float>> neighbors(distances.size());
        for (size_t j = 0; j < distances.size(); j++) {
            if(distances[i][j] == 0.0f)
                neighbors[j] = {j, std::numeric_limits<double>::infinity()};
            else
                neighbors[j] = {j, distances[i][j]};
        }
        sort(neighbors.begin(), neighbors.end(), [](const std::pair<uint, float>& a, const std::pair<uint, float>& b) {
            return a.second < b.second;
        });
        neighbors.resize(K);

        KNN[i] = neighbors;
    }

    std::vector<Simplex::Edge> triangulation;
    for (uint16_t i = 0; i < masses.size(); i++) {
        auto neighbors = KNN[i];

        for (auto neighbor : neighbors) {
            Simplex::Edge edge = {i, neighbor.first, neighbor.second};
            triangulation.push_back(edge);
        }
    }

    return triangulation;
}

}