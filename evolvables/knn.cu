#include "knn.h"
#include "knn_kernel.cu"
#include "mass.h"

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

namespace KNN {

template<typename T>
std::vector<std::vector<std::vector<std::pair<unsigned int,float>>>> Batch(const std::vector<T>& mass_groups, unsigned int K)
{
    unsigned int num_groups = mass_groups.size();
    unsigned int masses_per_group = mass_groups[0].masses.size();
    unsigned int num_masses = num_groups * masses_per_group;

    // CPU data
    float* h_points = new float[num_masses * 3];
    unsigned int* h_indices = new unsigned int[num_masses * K];
    float* h_distances = new float[num_masses * K];

    for(unsigned int i = 0; i < num_groups; i++) {
        std::vector<Mass> masses = mass_groups[i].masses;
        for (unsigned int i = 0; i < masses_per_group; i++)
        {
            h_points[3*i]   = masses[i].pos[0];
            h_points[3*i+1] = masses[i].pos[1];
            h_points[3*i+2] = masses[i].pos[2];
        }
    }

    // GPU data 
    float* d_points;
    cudaMalloc(&d_points, num_masses * 3 * sizeof(float));

    unsigned int* d_indices;
    cudaMalloc(&d_indices, num_masses * K * sizeof(unsigned int));

    float* d_distances;
    cudaMalloc(&d_distances, num_masses * K * sizeof(float));

    // Copy data to device memory
    cudaMemcpy(d_points, h_points, num_masses * 3 * sizeof(float), cudaMemcpyHostToDevice);

    // Shared memory size
    // unsigned int bytesPerThread = (sizeof(float) + sizeof(unsigned int)) * K;
    // unsigned int sharedMemSize = BLOCK_SIZE * bytesPerThread;
    
    // Compute k-nearest neighbors
    int num_blocks = num_groups;

    // printf("Num blocks: %u\n", num_blocks);
    // printf("Block size: %u\n", BLOCK_SIZE);
    // printf("Shared memory size: %u\n", sharedMemSize);
    
    // Execute the kernel
    compute_k_nearest_neighbors<<<num_blocks, BLOCK_SIZE>>>((float3*) d_points, d_indices, d_distances, masses_per_group, K);
    gpuErrchk( cudaPeekAtLastError() );
    cudaDeviceSynchronize();


    cudaMemcpy(h_indices, d_indices, num_masses * K * sizeof(unsigned int), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_distances, d_distances, num_masses * K * sizeof(float), cudaMemcpyDeviceToHost);


    std::vector<std::vector<std::vector<std::pair<unsigned int,float>>>> KNN(
        num_groups, std::vector<std::vector<std::pair<unsigned int, float>>>(masses_per_group, std::vector<std::pair<unsigned int,float>>(K)));
        
    for(unsigned int i = 0; i < num_groups; i++) {
        for(unsigned int j = 0; j < masses_per_group; j++) {
            for(unsigned int k = 0; k < K; k++) {
                int index = i*masses_per_group + j * K + k;
                KNN[i][j][k] = {h_indices[index], h_distances[index]};
            }
        }
    }

    // Free memory
    delete[] h_points;
    delete[] h_indices;
    delete[] h_distances;

    cudaFree(d_points);
    cudaFree(d_indices);
    cudaFree(d_distances);
    
    return KNN;
}

template<typename T>
std::vector<std::vector<std::pair<unsigned int,float>>> KNN(const T& mass_group, unsigned int K)
{
    unsigned int num_masses = mass_group.masses.size();

    // CPU data
    float* h_points = new float[num_masses * 3];
    float* h_distances = new float[num_masses * num_masses];

    std::vector<Mass> masses = mass_group.masses;
    for (unsigned int i = 0; i < num_masses; i++)
    {
        h_points[3*i]   = masses[i].pos[0];
        h_points[3*i+1] = masses[i].pos[1];
        h_points[3*i+2] = masses[i].pos[2];

        // printf("%u: {%f,%f,%f}\n",i,h_points[3*i],h_points[3*i+1],h_points[3*2]);
    }

    // GPU data 
    float* d_points;
    cudaMalloc(&d_points, num_masses * 3 * sizeof(float));

    float* d_distances;
    cudaMalloc(&d_distances, num_masses * num_masses * sizeof(float));

    // Copy data to device memory
    cudaMemcpy(d_points, h_points, num_masses * 3 * sizeof(float), cudaMemcpyHostToDevice);

    // Compute k-nearest neighbors
    uint block_count = (num_masses + BLOCK_SIZE - 1) / BLOCK_SIZE;
    dim3 num_blocks = {block_count, block_count};

    // Execute the kernel
    compute_distance_matrix<<<num_blocks, {BLOCK_SIZE, BLOCK_SIZE}>>>((float3*) d_points, d_distances, num_masses);
    gpuErrchk( cudaPeekAtLastError() );
    cudaDeviceSynchronize();


    cudaMemcpy(h_distances, d_distances, num_masses * num_masses * sizeof(float), cudaMemcpyDeviceToHost);

    std::vector<std::vector<float>> distances(num_masses, std::vector<float>(num_masses));
    for(uint i = 0; i < num_masses; i++) {
        for(uint j = 0; j < num_masses; j++) {
            distances[i][j] = h_distances[num_masses*i + j];
        }
    }

    std::vector<std::vector<std::pair<unsigned int,float>>> KNN(
        num_masses, std::vector<std::pair<unsigned int, float>>(K));

    for(uint i = 0; i < distances.size(); i++) {
        std::vector<std::pair<uint, float>> neighbors(distances.size());
        for (uint j = 0; j < distances.size(); j++) {
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

    
    // Free memory
    delete[] h_points;
    delete[] h_distances;

    cudaFree(d_points);
    cudaFree(d_distances);
    
    return KNN;
}

// Explicit instantiation of bar for NNRobot
template std::vector<std::vector<std::vector<std::pair<unsigned int,float>>>> Batch<NNRobot>(const std::vector<NNRobot>& mass_groups, unsigned int K);
template std::vector<std::vector<std::pair<unsigned int,float>>> KNN<NNRobot>(const NNRobot& mass_group, unsigned int K);

}