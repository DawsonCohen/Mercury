#include "Simulator.h"
#include <math.h>
#include <algorithm>
#include <random>
#include <map>
#include "util.h"
#include "vec_math.cuh"
#include "material.h"
#include <curand_kernel.h>
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include "environment.h"

struct SimOptions {
    float dt;
    uint massesPerBlock;
    uint springsPerBlock;
    uint facesPerBlock;
    uint cellsPerBlock;
    uint boundaryMassesPerBlock;
    uint maxMasses;
    uint maxSprings;
    uint maxFaces;
    uint maxCells;
    uint compositeCount;
    short shiftskip;
    float drag;
    float damping;
    float relaxation;
    float s;
};

struct DevoOptions {
    uint maxReplacedSprings;
    uint maxSprings;
    uint springsPerElement;
    uint massesPerElement;
    uint replacedSpringsPerElement;
    uint compositeCount;
};

__constant__ float4 compositeMats_id[COMPOSITE_COUNT];
__constant__ SimOptions cSimOpt;
__constant__ float4 compositeMats_encoding[COMPOSITE_COUNT];
__constant__ DevoOptions cDevoOpt;



#define EPS (float) 1e-12
#define MAX_FORCE (float) 200000

__global__ inline
void surfaceDragForce(float4 *__restrict__ pos, float4 *__restrict__ newPos,
                 float4 *__restrict__ vel, ushort4 *__restrict__ faces) {
	extern __shared__ float3 s[];
	float3  *s_pos = s;
	float3  *s_vel = (float3*) &s_pos[cSimOpt.boundaryMassesPerBlock];
	float3  *s_force = (float3*) &s_vel[cSimOpt.boundaryMassesPerBlock];
	
	uint massOffset   = blockIdx.x * cSimOpt.massesPerBlock;
	uint faceOffset   = blockIdx.x * cSimOpt.facesPerBlock;
	uint i;

	int tid    = threadIdx.x;
	int stride = blockDim.x;
	
	// Initialize and compute environment forces
	float4 pos4, vel4;
	for(i = tid; i < cSimOpt.boundaryMassesPerBlock && (i+massOffset) < cSimOpt.maxMasses; i+=stride) {
		pos4 = __ldg(&pos[i+massOffset]);
		vel4 = __ldg(&vel[i+massOffset]);
		s_pos[i] = {pos4.x,pos4.y,pos4.z};
		s_vel[i] = {vel4.x,vel4.y,vel4.z};
		s_force[i] = {0.0f, 0.0f, 0.0f};
	}
	
	float rho = cSimOpt.drag, area;
	ushort4 face;
	float3  x0, x1, x2,
	        v0, v1, v2,
			v, normal, force;
	for(i = tid; i < cSimOpt.facesPerBlock && (i+faceOffset) < cSimOpt.maxFaces; i+=stride) {
		// Drag Force: 0.5*rho*A*((Cd - Cl)*dot(v,n)*v + Cl*dot(v,v)*n)
		face = __ldg(&faces[i+faceOffset]);
		if(face.x == face.y || face.x == face.z || face.y == face.z)
			continue;

		x0 = s_pos[face.x];
		x1 = s_pos[face.y];
		x2 = s_pos[face.z];
		v0 = s_vel[face.x];
		v1 = s_vel[face.y];
		v2 = s_vel[face.z];
		
		v = (v0 + v1 + v2) / 3.0f;
		normal = cross((x1 - x0), (x2-x0));
		area = norm3df(normal.x,normal.y,normal.z);
		normal = normal / (area + EPS);
		normal = dot(normal, v) > 0.0f ? normal : -normal;
		force = -0.5*rho*area*(dot(v,normal)*v + 0.2*dot(v,v)*normal);
		force = force / 3.0f; // allocate forces evenly amongst masses
		
		atomicAdd(&(s_force[face.x].x), force.x);
		atomicAdd(&(s_force[face.x].y), force.y);
		atomicAdd(&(s_force[face.x].z), force.z);

		atomicAdd(&(s_force[face.y].x), force.x);
		atomicAdd(&(s_force[face.y].y), force.y);
		atomicAdd(&(s_force[face.y].z), force.z);

		atomicAdd(&(s_force[face.z].x), force.x);
		atomicAdd(&(s_force[face.z].y), force.y);
		atomicAdd(&(s_force[face.z].z), force.z);
	}

	for(i = tid; i < cSimOpt.boundaryMassesPerBlock && (i+massOffset) < cSimOpt.maxMasses; i+=stride) {
		x0 = s_pos[i];
		v0 = s_vel[i];
		force = s_force[i];
		newPos[i+massOffset].x = x0.x + v0.x*cSimOpt.dt + force.x*cSimOpt.dt*cSimOpt.dt;
		newPos[i+massOffset].y = x0.y + v0.y*cSimOpt.dt + force.y*cSimOpt.dt*cSimOpt.dt;
		newPos[i+massOffset].z = x0.z + v0.z*cSimOpt.dt + force.z*cSimOpt.dt*cSimOpt.dt;
	}
}

__global__ inline
void preSolve(float4 *__restrict__ pos, float4 *__restrict__ newPos,
                 float4 *__restrict__ vel) {
	int stride = blockDim.x * gridDim.x;
	float4 velocity;

	for(uint i = blockIdx.x * blockDim.x + threadIdx.x;
		i < cSimOpt.maxMasses; i+=stride) {
		//Force due to drag = - (1/2 * rho * |v|^2 * A * Cd) * v / |v| (Assume A and Cd are 1)
		velocity = __ldg(&vel[i]);
		newPos[i] = __ldg(&pos[i]) + velocity*cSimOpt.dt;
	}
}


/*
	Exended Positon Based Dynamics
	Computes lagrangian (force) for each distance constraint (spring)

	mat: float4 describing spring material
		x - k		stiffness
		y - dL0 	maximum percent change
		z - omega	frequency of oscillation
		w - phi		phase
*/
__global__ inline
void solveDistance(float4 *__restrict__ newPos, ushort2 *__restrict__ pairs, 
				float * __restrict__ stresses, uint8_t *__restrict__ matIds, float *__restrict__ Lbars,
				float time, uint step, bool integrateForce)
{
	extern __shared__ float3 s[];
	float3  *s_pos = s;
	float3  *s_dp = (float3*) &s_pos[cSimOpt.massesPerBlock];
	
	uint massOffset   = blockIdx.x * cSimOpt.massesPerBlock;
	uint springOffset = blockIdx.x * cSimOpt.springsPerBlock;
	uint i;

	int tid    = threadIdx.x;
	int stride = blockDim.x;
	
	// Initialize and compute environment forces
	float4 pos4;
	for(i = tid; i < cSimOpt.massesPerBlock && (i+massOffset) < cSimOpt.maxMasses; i+=stride) {
		pos4 = __ldg(&newPos[i+massOffset]);
		s_pos[i] = {pos4.x,pos4.y,pos4.z};
		s_dp[i] = {0.0f, 0.0f, 0.0f};
	}

	__syncthreads();

	float4	 mat;
	uint8_t  matId;

	float3	 pos0, pos1;
	float	 Lbar,
			 C, alpha,
			 lambda;
	ushort	 v0, v1;
	ushort2	 pair;

	float3	distance, n;

	float	relative_change,
			rest_length,
			d, K;
	float3  dp;
	
	for(i = tid; i < cSimOpt.springsPerBlock && (i+springOffset) < cSimOpt.maxSprings; i+=stride) {
		matId = __ldg(&matIds[i+springOffset]);
		if(matId == materials::air.id) continue;

		pair = __ldg(&pairs[i+springOffset]);
		Lbar = __ldg(&Lbars[i+springOffset]);
		v0 = pair.x; v1 = pair.y;
		pos0 = s_pos[v0];
		pos1 = s_pos[v1];

		mat = compositeMats_id[ matId ];
		alpha = 1.0f / mat.x / cSimOpt.dt / cSimOpt.dt;
		// rest_length = mean_length * (1 + relative_change);
		relative_change = mat.y * sinf(mat.z*time+mat.w);
		rest_length = __fmaf_rn(Lbar, relative_change, Lbar);
		
		K = 2.0f + alpha;
		distance = pos0-pos1;
		d = l2norm(distance);
		n = distance / (d + EPS);
		
		C = d-rest_length;
		lambda = -(C) / (K);
		dp = lambda * n;

		if(integrateForce) stresses[i+springOffset] += lambda / Lbar;

		atomicAdd(&(s_dp[v0].x), dp.x);
		atomicAdd(&(s_dp[v0].y), dp.y);
		atomicAdd(&(s_dp[v0].z), dp.z);

		atomicAdd(&(s_dp[v1].x), -dp.x);
		atomicAdd(&(s_dp[v1].y), -dp.y);
		atomicAdd(&(s_dp[v1].z), -dp.z);
	}
	__syncthreads();

	for(i = tid; i < cSimOpt.massesPerBlock && (i+massOffset) < cSimOpt.maxMasses; i+=stride) {
		pos4 =__ldg(&newPos[i+massOffset]);
		pos4.x += s_dp[i].x;
		pos4.y += s_dp[i].y;
		pos4.z += s_dp[i].z;
		newPos[i+massOffset] = pos4;
	}
}

__global__
inline void update(float4 *__restrict__ pos, float4 *__restrict__ newPos, float4 *__restrict__ vel) {
	// Calculate and store new mass states
	int stride = blockDim.x * gridDim.x;

	float4 newPos4, pos4;
	for(uint i = blockIdx.x * blockDim.x + threadIdx.x; i < cSimOpt.maxMasses; i+=stride) {
		pos4 = __ldg(&pos[i]);
		newPos4 = __ldg(&newPos[i]);
		pos[i] = newPos4;
		vel[i].x = 0.99*(newPos4.x - pos4.x) / cSimOpt.dt;
		vel[i].y = 0.99*(newPos4.y - pos4.y) / cSimOpt.dt;
		vel[i].z = 0.99*(newPos4.z - pos4.z) / cSimOpt.dt;
	}
}

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

__global__ void
inline replaceSprings(
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

#include <cub/device/device_segmented_radix_sort.cuh>

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess) 
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}

template <std::size_t... Is, typename... Ts>
std::string DataToCSVImpl(const std::string& header, const std::vector<std::tuple<Ts...>>& data, std::index_sequence<Is...>, bool writeHeader)
{
    std::ostringstream os;

    // Write the header
	if(writeHeader) os << header << std::endl;

    // Write the data
    for (const auto& row : data)
    {
        bool first = true;
        ((os << (first ? first = false, "" : ","), os << std::get<Is>(row)), ...);
        os << std::endl;
    }

    return os.str();
}

template <typename... Ts>
std::string DataToCSV(const std::string& header, const std::vector<std::tuple<Ts...>>& data, bool writeHeader=true)
{
    return DataToCSVImpl(header, data, std::index_sequence_for<Ts...>(), writeHeader);
}

void Simulator::freeMemory() {
	// Free CPU
	delete[] massBuf;
	delete[] springBuf;
	delete[] faceBuf;
	delete[] cellBuf;
	delete[] envBuf;

	delete[] m_hCompositeMats_encoding;
	delete[] m_hCompositeMats_id;
	
	delete[] m_hPos;
	delete[] m_hVel;
	delete[] m_hMassMatEncodings;

	delete[] m_hPairs;
	delete[] m_hSpringMatEncodings;
	delete[] m_hSpringMatIds;
	delete[] m_hLbars;
	delete[] m_hSpringIDs;
	delete[] m_hSpringStresses;
	
	delete[] m_hFaces;

	delete[] m_hCells;
	delete[] m_hVbars;
	delete[] m_hMats;
	delete[] m_hCellStresses;

	// Free GPU
	cudaFree((void**) m_dPos);
	cudaFree((void**) m_dNewPos);
	cudaFree((void**) m_dVel);
	cudaFree((void**) m_dMassMatEncodings);

	cudaFree((void**) m_dPairs);
	cudaFree((void**) m_dSpringMatEncodings);
	cudaFree((void**) m_dSpringMatIds);
	cudaFree((void**) m_dLbars);
	cudaFree((void**) m_dSpringIDs);
	cudaFree((void**) m_dSpringStresses);
	cudaFree((void**) m_dRandomPairs);
	cudaFree((void**) m_dSpringStresses_Sorted);
	cudaFree((void**) m_dSpringIDs_Sorted);

	cudaFree((void**) m_dFaces);

	cudaFree((void**) m_dCells);
	cudaFree((void**) m_dVbars);
	cudaFree((void**) m_dMats);
	cudaFree((void**) m_dCellStresses);
}

Simulator::~Simulator() {
	if(initialized) freeMemory();
}

void Simulator::Initialize(Config::Simulator config) {
	m_replacedSpringsPerElement = config.replaced_springs_per_element;
	m_deltaT = config.time_step;
	m_config = config;

	_initialize();
}


void Simulator::_initialize() { //uint maxMasses, uint maxSprings) {
	maxEnvs = 1;
	m_total_time = 0.0f;
	
	if(initialized) freeMemory();
	initialized = true;
	
	massBuf   = new Mass[maxMasses];
	springBuf = new Spring[maxSprings];
	faceBuf   = new Face[maxFaces];
	cellBuf   = new Cell[maxCells];
	envBuf 	  = new Environment[1];

	m_hCompositeMats_encoding	= new float[COMPOSITE_COUNT*4];
	m_hCompositeMats_id			= new float[COMPOSITE_COUNT*4];

	m_hPos 	  = new float[maxMasses*4];
	m_hVel 	  = new float[maxMasses*4];
	m_hMassMatEncodings		= new uint32_t[maxMasses];

	m_hPairs  = new ushort[maxSprings*2];
	m_hSpringMatEncodings	= new uint32_t[maxSprings];
	m_hSpringMatIds			= new uint8_t[maxSprings];
	m_hLbars  = new float[maxSprings];
	m_hSpringIDs = new uint[maxSprings];
	m_hSpringStresses  = new float[maxSprings];

	m_hFaces  = new ushort[maxFaces*4];

	m_hCells  = new ushort[maxCells*4];
	m_hVbars  = new float[maxCells];
	m_hMats  = new float[maxCells*4];
	m_hCellStresses  = new float[maxCells];

	memset(m_hPos, 0, maxMasses*4*sizeof(float));
	memset(m_hVel, 0, maxMasses*4*sizeof(float));

	memset(m_hSpringStresses, 0, maxSprings * sizeof(float));
    memset(m_hCellStresses, 0, maxCells * sizeof(float));

    // memset(m_hSpringIDs, 0, maxSprings * sizeof(uint));
    // memset(m_hSpringMatEncodings, 0, maxSprings * sizeof(uint));
	
    unsigned int massSizefloat4     = sizeof(float)    * 4 * maxMasses;
    unsigned int massSizeuint32_t   = sizeof(uint32_t) * 1 * maxMasses;
    unsigned int springSizeuint32_t	= sizeof(uint32_t) * 1 * maxSprings;
    unsigned int springSizeuint8_t	= sizeof(uint8_t)  * 1 * maxSprings;
    unsigned int springSizefloat    = sizeof(float)    * 1 * maxSprings;
    unsigned int springSizeushort2  = sizeof(ushort)   * 2 * maxSprings;
    unsigned int springSizeuint     = sizeof(uint)     * 1 * maxSprings;
    unsigned int replaceSizeushort2 = sizeof(ushort)   * 2 * maxReplaced;
    unsigned int faceSizeushort4    = sizeof(ushort)   * 4 * maxFaces;
    unsigned int cellSizeushort4    = sizeof(ushort)   * 4 * maxCells;
	unsigned int cellSizefloat      = sizeof(float)    * 1 * maxCells;
    unsigned int cellSizefloat4     = sizeof(float)    * 4 * maxCells;

	cudaMalloc((void**)&m_dPos, massSizefloat4);
	cudaMalloc((void**)&m_dNewPos, massSizefloat4);
	cudaMalloc((void**)&m_dVel, massSizefloat4);
	cudaMalloc((void**)&m_dMassMatEncodings,	massSizeuint32_t);
	gpuErrchk( cudaPeekAtLastError() );

	cudaMalloc((void**)&m_dPairs,  springSizeushort2);
	cudaMalloc((void**)&m_dSpringMatEncodings,	springSizeuint32_t);
	cudaMalloc((void**)&m_dSpringMatIds,		springSizeuint8_t);
	cudaMalloc((void**)&m_dLbars,  springSizefloat);
	cudaMalloc((void**)&m_dSpringIDs,  springSizeuint);
	cudaMalloc((void**)&m_dSpringStresses,  springSizefloat);
	cudaMalloc((void**)&m_dRandomPairs,  replaceSizeushort2);
	cudaMalloc((void**)&m_dSpringIDs_Sorted,  springSizeuint);
	cudaMalloc((void**)&m_dSpringStresses_Sorted,  springSizefloat);

	cudaMalloc((void**)&m_dFaces,  faceSizeushort4);
	
	cudaMalloc((void**)&m_dCells,  cellSizeushort4);
	cudaMalloc((void**)&m_dVbars,  cellSizefloat);
	cudaMalloc((void**)&m_dMats,  cellSizefloat4);
	cudaMalloc((void**)&m_dCellStresses,  cellSizefloat);

	switch(m_config.env_type) {
		case ENVIRONMENT_LAND:
			envBuf[0] = EnvironmentLand;
		case ENVIRONMENT_WATER:
			envBuf[0] = EnvironmentWater;
	}
	envCount++;
}

ElementTracker Simulator::SetElement(const Element& element) {
	std::vector<Element> elements = {element};
	std::vector<ElementTracker> trackers = SetElements(elements);
	return trackers[0];
}

std::vector<ElementTracker> Simulator::SetElements(const std::vector<Element>& elements) {
	std::vector<ElementTracker> trackers;

	// cudaFuncAttributes attr;
	// cudaFuncGetAttributes(&attr, integrateBodiesStresses);
	// simThreadsPerBlock = attr.maxThreadsPerBlock;
	// assert( attr.numRegs <= 32768 );
	// cudaFuncGetAttributes(&attr, integrateBodies);
	// if(attr.maxThreadsPerBlock < simThreadsPerBlock)
	// 	simThreadsPerBlock = attr.maxThreadsPerBlock;
	
	// assert( attr.numRegs <= 32768 );
	uint largestElementBoundaryMasses = 0;
	uint largestElementSprings = 0;
	uint largestElementFaces = 0;
	uint largestElementCells = 0;
	for(auto& e : elements) {
		if(e.boundaryCount > largestElementBoundaryMasses) largestElementBoundaryMasses = e.boundaryCount;
		if(e.springs.size() > largestElementSprings) largestElementSprings = e.springs.size();
		if(e.faces.size() > largestElementFaces) largestElementFaces = e.faces.size();
		if(e.cells.size() > largestElementCells) largestElementCells = e.cells.size();
	}

	maxElements = elements.size();
	maxReplaced = m_replacedSpringsPerElement * maxElements;
	massesPerElement = elements[0].masses.size();
	boundaryMassesPerElement = largestElementBoundaryMasses;
	springsPerElement = largestElementSprings;
	facesPerElement = largestElementFaces;
	cellsPerElement = largestElementCells;
	maxMasses = massesPerElement*maxElements;
	maxSprings = largestElementSprings*maxElements;
	maxFaces = largestElementFaces*maxElements;
	maxCells = largestElementCells*maxElements;

	_initialize();

	numElements = 0; numMasses = 0; numSprings = 0; numFaces = 0; numCells = 0;
	for(uint i = 0; i < elements.size(); i++) {
		trackers.push_back(AllocateElement(elements[i]));
	}

	Eigen::Vector3f pos, vel;
	for(uint i = 0; i < numMasses; i++) {
		float  mass = massBuf[i].mass;
		vel  = massBuf[i].vel;
		pos = massBuf[i].pos;

		m_hMassMatEncodings[i] = massBuf[i].material.encoding;

		m_hPos[4*i]   = pos.x();
		m_hPos[4*i+1] = pos.y();
		m_hPos[4*i+2] = pos.z();
		m_hPos[4*i+3] = mass;
		
		m_hVel[4*i]   = vel.x();
		m_hVel[4*i+1] = vel.y();
		m_hVel[4*i+2] = vel.z();
	}

	uint32_t encoding = 0x01u;
	Material mat = materials::decode(encoding);
	m_hCompositeMats_encoding[0] = mat.k;
	m_hCompositeMats_encoding[1] = mat.dL0;
	m_hCompositeMats_encoding[2] = mat.omega;
	m_hCompositeMats_encoding[3] = mat.phi;

	uint idx;
	for(uint i = 1; i < MATERIAL_COUNT; i++) {
		for(uint j = i; j < MATERIAL_COUNT; j++) {
			encoding = (0x01u << i) | (0x01u << j);
			idx = materials::encodedCompositeIdx(encoding);
			mat = materials::decode(encoding);
			m_hCompositeMats_encoding[4*idx] = mat.k;
			m_hCompositeMats_encoding[4*idx+1] = mat.dL0;
			m_hCompositeMats_encoding[4*idx+2] = mat.omega;
			m_hCompositeMats_encoding[4*idx+3] = mat.phi;
		}
	}

	for(uint i = 0; i < COMPOSITE_COUNT; i++) {
		Material mat = materials::id_lookup(i);
		m_hCompositeMats_id[4*i] = mat.k;
		m_hCompositeMats_id[4*i+1] = mat.dL0;
		m_hCompositeMats_id[4*i+2] = mat.omega;
		m_hCompositeMats_id[4*i+3] = mat.phi;
	}

	for(uint i = 0; i < numSprings; i++) {
		float    lbar        = springBuf[i].mean_length;
		ushort	 m0          = springBuf[i].m0,
			     m1	         = springBuf[i].m1;
		uint8_t	 matId 		 = springBuf[i].material.id;
		uint32_t matEncoding = springBuf[i].material.encoding;

		m_hSpringIDs[i] = i;
		m_hPairs[2*i]   = m0;
		m_hPairs[2*i+1] = m1;
		m_hLbars[i] 	= lbar;

		m_hSpringMatIds[i] = matId;
		m_hSpringMatEncodings[i] = matEncoding;
	}

	for(uint i = 0; i < numFaces; i++) {
		ushort   m0          = faceBuf[i].m0,
			     m1          = faceBuf[i].m1,
			     m2          = faceBuf[i].m2;

		m_hFaces[4*i]   = m0;
		m_hFaces[4*i+1] = m1;
		m_hFaces[4*i+2] = m2;
	}

	for(uint i = 0; i < numCells; i++) {
		ushort   m0          = cellBuf[i].m0,
			     m1          = cellBuf[i].m1,
			     m2          = cellBuf[i].m2,
			     m3          = cellBuf[i].m3;
		float    vbar        = cellBuf[i].mean_volume;
        Material mat         = cellBuf[i].material;

		m_hCells[4*i]   = m0;
		m_hCells[4*i+1] = m1;
		m_hCells[4*i+2] = m2;
		m_hCells[4*i+3] = m3;

		m_hMats[4*i]   = mat.k;
		m_hMats[4*i+1] = mat.dL0;
		m_hMats[4*i+2] = mat.omega;
		m_hMats[4*i+3] = mat.phi;

		m_hVbars[i] = vbar;
	}

	cudaMemcpyToSymbol(compositeMats_id,   m_hCompositeMats_id,   COMPOSITE_COUNT*4*sizeof(float));
	cudaMemcpyToSymbol(compositeMats_encoding,   m_hCompositeMats_encoding,   COMPOSITE_COUNT*4*sizeof(float));

	cudaMemcpy(m_dPos, m_hPos,   numMasses   *4*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dVel, m_hVel,   numMasses   *4*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dMassMatEncodings,		m_hMassMatEncodings,   	 numMasses  * sizeof(uint32_t), cudaMemcpyHostToDevice);
	
	cudaMemcpy(m_dPairs,  				m_hPairs			  , numSprings*2*sizeof(ushort),  cudaMemcpyHostToDevice);
	cudaMemcpy(m_dSpringMatEncodings,	m_hSpringMatEncodings , numSprings * sizeof(uint32_t), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dSpringMatIds,   		m_hSpringMatIds		  , numSprings * sizeof(uint8_t), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dLbars,  				m_hLbars			  , numSprings * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dSpringIDs,   			m_hSpringIDs		  , numSprings * sizeof(uint), cudaMemcpyHostToDevice);
	cudaMemset(m_dSpringStresses,  		0.0f				  , numSprings * sizeof(float));
	
	cudaMemcpy(m_dFaces,  m_hFaces,  numFaces *4*sizeof(ushort),  cudaMemcpyHostToDevice);
	
	cudaMemcpy(m_dCells,  		m_hCells,  numCells*4*sizeof(ushort),  cudaMemcpyHostToDevice);
	cudaMemcpy(m_dVbars,  		m_hVbars,  numCells*1*sizeof(float) ,  cudaMemcpyHostToDevice);
	cudaMemcpy(m_dMats ,  		m_hMats ,  numCells*4*sizeof(float) ,  cudaMemcpyHostToDevice);
	cudaMemset(m_dCellStresses, 0.0f	,  numCells*1*sizeof(float));

	gpuErrchk( cudaPeekAtLastError() );

	return trackers;
}

void Simulator::Simulate(float sim_duration, bool trackStresses, bool trace, std::string tracefile) {
	float simTimeRemaining = sim_duration;
	
	/*
	Notes on SM resources:
	thread blocks	: 8
	threads			: 2048
	registers		: 65536
	shared mem		: 49152
	*/

	uint maxSharedMemSize = 49152;
	uint bytesPerMass = sizeof(float3) + sizeof(float3);
	uint elementsPerBlock = 1;
	m_massesPerBlock = massesPerElement * elementsPerBlock;
	m_springsPerBlock = springsPerElement * elementsPerBlock;
	m_facesPerBlock = facesPerElement * elementsPerBlock;
	m_cellsPerBlock = cellsPerElement * elementsPerBlock;
	m_sharedMemSizeSim = m_massesPerBlock * bytesPerMass;
	m_numBlocksSim = (numElements + elementsPerBlock - 1) / elementsPerBlock;

	// printf("%u / %u\n", m_massesPerBlock, (uint) (maxSharedMemSize - COMPOSITE_COUNT*bytesPerMaterial - sizeof(ushort)*2048) / (bytesPerMass));
	// printf("%u / %u\n", m_sharedMemSizeSim, maxSharedMemSize);

	assert(m_sharedMemSizeSim <= maxSharedMemSize);

	uint bytesPerBoundaryMass = sizeof(float3) + sizeof(float3) + sizeof(float3);
	uint boundaryMassesPerBlock = boundaryMassesPerElement * elementsPerBlock;
	uint dragSharedMemorySize = boundaryMassesPerBlock * bytesPerBoundaryMass;
	uint numDragBlocks = (numElements + elementsPerBlock - 1) / elementsPerBlock;

	// printf("%u / %u\n", m_massesPerBlock, (uint) (maxSharedMemSize - COMPOSITE_COUNT*bytesPerMaterial - sizeof(ushort)*2048) / (bytesPerMass));
	// printf("%u / %u\n", m_sharedMemSizeSim, maxSharedMemSize);

	assert(dragSharedMemorySize <= maxSharedMemSize);

	short shiftskip = 20;

	SimOptions opt = {
		m_deltaT,
		m_massesPerBlock, m_springsPerBlock, m_facesPerBlock, m_cellsPerBlock,
		boundaryMassesPerBlock,
		numMasses, numSprings, numFaces, numCells,
		COMPOSITE_COUNT,
		shiftskip,
		envBuf[0].drag,
		envBuf[0].damping,
		1.0,
		0.2
	};
	
	uint step_count = 0;

	static std::map<std::string, int> trace_hist;
	if(trace) {
		if(!trace_hist[tracefile]) trace_hist[tracefile] = 0;
	}

	std::vector<std::tuple<unsigned int, float, float, float, float, float, float, float>> massTrace;
	
	cudaMemcpyToSymbol(cSimOpt, &opt, sizeof(SimOptions));
	
	while(simTimeRemaining > 0.0f) {
		uint numThreads = 512;
		uint blocks = (numMasses + numThreads - 1)  / numThreads;
		surfaceDragForce<<<numDragBlocks,simThreadsPerBlock,dragSharedMemorySize>>>(
			(float4*) m_dPos, (float4*) m_dNewPos, (float4*) m_dVel, (ushort4*) m_dFaces);
		cudaDeviceSynchronize();
		gpuErrchk( cudaPeekAtLastError() );

		preSolve<<<blocks, numThreads>>>(
			(float4*) m_dPos, (float4*) m_dNewPos, (float4*) m_dVel);
		cudaDeviceSynchronize();
		gpuErrchk( cudaPeekAtLastError() );

		solveDistance<<<m_numBlocksSim,simThreadsPerBlock,m_sharedMemSizeSim>>>(
			(float4*) m_dNewPos, (ushort2*)  m_dPairs, 
			(float*) m_dSpringStresses, (uint8_t*) m_dSpringMatIds, (float*) m_dLbars,
			m_total_time, step_count, trackStresses);
		cudaDeviceSynchronize();
		gpuErrchk( cudaPeekAtLastError() );
			
		update<<<blocks,512>>>((float4*) m_dPos, (float4*) m_dNewPos,
			(float4*) m_dVel);
		cudaDeviceSynchronize();
		gpuErrchk( cudaPeekAtLastError() );

		if(trace) {
			if(step_count % 20 == 0) {
				cudaMemcpy(m_hPos,m_dPos,numMasses*4*sizeof(float),cudaMemcpyDeviceToHost);
				cudaMemcpy(m_hVel,m_dVel,numMasses*4*sizeof(float),cudaMemcpyDeviceToHost);

				for(unsigned int i = 0; i < numMasses; i++) {
					float3 pos = {m_hPos[4*i], m_hPos[4*i+1], m_hPos[4*i+2]};
					float3 vel = {m_hVel[4*i], m_hVel[4*i+1], m_hVel[4*i+2]};

					massTrace.push_back({ i, m_total_time, pos.x, pos.y, pos.z, vel.x, vel.y, vel.z });
				}
			}
		}
		
		step_count++;
		m_total_time += m_deltaT;
		simTimeRemaining -= m_deltaT;
	}

	if(trace) {
		bool firstTrace = true;
		if(trace_hist[tracefile] > 0) firstTrace = false;
		std::string massTraceCSV = DataToCSV<unsigned int, float, float, float, float, float, float, float>("id, time, x, y, z, vx, vy, vz",massTrace, firstTrace);
		util::WriteCSV(tracefile, "/mnt/vault/evo-devo/z_results", massTraceCSV, !firstTrace);
		trace_hist[tracefile]++;
	}
}

ElementTracker Simulator::AllocateElement(const Element& e) {
	ElementTracker tracker;

	tracker.mass_begin = massBuf + numMasses;
	tracker.spring_begin = springBuf + numSprings;
	tracker.mass_end = tracker.mass_begin; 
	tracker.spring_end = tracker.spring_begin;
	
	for(const Mass& m : e.masses) {
		massBuf[numMasses] = m;
		tracker.mass_end++;
		numMasses++;
	}

	
	// unsigned seed = rand();
	// std::vector<Spring> shuffledSprings(e.springs);
	// std::shuffle(shuffledSprings.begin(), shuffledSprings.end(), std::default_random_engine(seed));

	// for(const Spring& s : shuffledSprings) {
	uint count = 0;
	for(const Spring& s : e.springs) {
		springBuf[numSprings] = s;
		tracker.spring_end++;

		numSprings++;
		count++;
	}

	// fill up springs to max spring size
	// TODO: variable spring count per robot
	for( ; count < springsPerElement; count++) {
		springBuf[numSprings] = {0,0,0.0f,0.0f,materials::air};
		numSprings++;
	}

	count = 0;
	for(const Face& f : e.faces) {
		faceBuf[numFaces] = f;

		numFaces++;
		count++;
	}

	// fill up faces to max faces size
	// TODO: variable face count per robot
	for( ; count < facesPerElement; count++) {
		faceBuf[numFaces] = {0,0,0};
		numFaces++;
	}

	count = 0;
	for(const Cell& c : e.cells) {
		cellBuf[numCells] = c;

		numCells++;
		count++;
	}

	// fill up cells to max cell size
	// TODO: variable cell count per robot
	for( ; count < cellsPerElement; count++) {
		cellBuf[numCells] = {0,0,0,0,0.0f,materials::air};
		numCells++;
	}
	
	numElements++;
	
	return tracker;
}

Element Simulator::Collect(const ElementTracker& tracker) {
	std::vector<ElementTracker> trackers = {tracker};
	std::vector<Element> results = Collect(trackers);
	return results[0];
}

std::vector<Element> Simulator::Collect(const std::vector<ElementTracker>& trackers) {
	cudaMemcpy(m_hPos,m_dPos,numMasses*4*sizeof(float),cudaMemcpyDeviceToHost);
	cudaMemcpy(m_hVel,m_dVel,numMasses*4*sizeof(float),cudaMemcpyDeviceToHost);
	cudaMemcpy(m_hSpringStresses,   m_dSpringStresses,   numSprings*sizeof(float), cudaMemcpyDeviceToHost);

	cudaMemcpy(m_hSpringMatEncodings, m_dSpringMatEncodings, numSprings*sizeof(uint32_t), cudaMemcpyDeviceToHost);
	cudaMemcpy(m_hPairs, m_dPairs, numSprings*2*sizeof(ushort), cudaMemcpyDeviceToHost);
	cudaMemcpy(m_hLbars, m_dLbars, numSprings * sizeof(float),  cudaMemcpyDeviceToHost);

	
	for(uint i = 0; i < numMasses; i++) {
		float3 pos = {m_hPos[4*i], m_hPos[4*i+1], m_hPos[4*i+2]};
		float3 vel = {m_hVel[4*i], m_hVel[4*i+1], m_hVel[4*i+2]};
		assert(!isnan(pos.x) && !isnan(pos.y) && !isnan(pos.z));
		
		massBuf[i].pos = Eigen::Vector3f(pos.x,pos.y,pos.z);
		massBuf[i].vel = Eigen::Vector3f(vel.x,vel.y,vel.z);
	}

	for(uint i = 0; i < numSprings; i++) {
		springBuf[i].m0 = m_hPairs[2*i];
		springBuf[i].m1 = m_hPairs[2*i+1];
		springBuf[i].mean_length = m_hLbars[i];
		springBuf[i].material = materials::decode(m_hSpringMatEncodings[i]);
	}

	#if defined(FULL_STRESS) && defined(WRITE_STRESS)
	std::vector<std::tuple<uint, float, uint, uint>> stressHistory;

	for(uint i = 0; i < maxSprings; i++) {
		stressHistory.push_back({m_hSpringIDs[i], m_hStresses[i], m_hMaxStressCount[i], m_hMinStressCount[i]});
	}
	std::string stressHistoryCSV = util::DataToCSV("id, stress, max count, min count",stressHistory);
	util::WriteCSV("stress.csv","../z_results/", stressHistoryCSV);
	#endif


	std::vector<Element> elements;
	for(const ElementTracker& tracker : trackers) {
		Element e = CollectElement(tracker);
		elements.push_back(e);
	}
	return elements;
}

Element Simulator::CollectElement(const ElementTracker& tracker) {
	std::vector<Mass> result_masses;
	std::vector<Spring> result_springs;

	for(Mass* i = tracker.mass_begin; i < tracker.mass_end; i++) {
		result_masses.push_back(*i);
	}

	for(Spring* i = tracker.spring_begin; i < tracker.spring_end; i++) {
		result_springs.push_back(*i);
	}
	
	return {result_masses, result_springs};
}

void key_value_sort(float* d_keys_in, float* d_keys_out, uint* d_values_in, uint* d_values_out, uint items_per_segment, uint num_segments) {
    // Determine number of items
    int num_items = num_segments * items_per_segment;

    // Allocate memory on device for offsets
    int* h_offsets = new int[num_segments+1];
    for(uint i = 0; i < num_segments+1; i++) {
        h_offsets[i] = (int) items_per_segment*i;
    }

    int* d_offsets;
    cudaMalloc(&d_offsets, (num_segments+1) * sizeof(int));
    cudaMemcpy(d_offsets, h_offsets, (num_segments+1) *sizeof(int), cudaMemcpyHostToDevice);

    // Determine temporary storage size
    void* d_temp_storage = NULL;
    size_t temp_storage_bytes = 0;
    cub::DeviceSegmentedRadixSort::SortPairsDescending(
        d_temp_storage, temp_storage_bytes,
        d_keys_in, d_keys_out, d_values_in, d_values_out,
        num_items, num_segments, d_offsets, d_offsets+1);

    // Allocate temporary storage
    cudaMalloc(&d_temp_storage, temp_storage_bytes);

    // Run sorting operation
    cub::DeviceSegmentedRadixSort::SortPairsDescending(
        d_temp_storage, temp_storage_bytes,
        d_keys_in, d_keys_out, d_values_in, d_values_out,
        num_items, num_segments, d_offsets, d_offsets+1);

    delete[] h_offsets;
    cudaFree(d_offsets);
    cudaFree(d_temp_storage);
}

__global__ inline void printStress(uint numSprings, float* stress, uint* springId) {
	uint stride = blockDim.x*gridDim.x;
	for(uint i = blockIdx.x*blockDim.x + threadIdx.x; i < numSprings; i += stride) {
		printf("%u: Stress %f, Spring %u\n", i, stress[i], springId[i]);
	}
}

void Simulator::Devo() {
	static int seed = 0;

	uint threadsPerBlock = 1024;
	uint numReplacedSprings = m_replacedSpringsPerElement * numElements;
	uint numBlocks = (numReplacedSprings+threadsPerBlock - 1) / threadsPerBlock;
	
    key_value_sort(m_dSpringStresses, m_dSpringStresses_Sorted, m_dSpringIDs, m_dSpringIDs_Sorted, springsPerElement, numElements);

	// numBlocks = (numSprings+threadsPerBlock-1) / threadsPerBlock;
	// printStress<<<1, threadsPerBlock>>>(numSprings, m_dSpringStresses_Sorted, m_dSpringIDs_Sorted);
	
	cudaDeviceSynchronize();
	gpuErrchk( cudaPeekAtLastError() );
	
	numBlocks = (numReplacedSprings + threadsPerBlock - 1) / threadsPerBlock;
	
	randomIntegerPairKernel<<<numBlocks, threadsPerBlock>>>(numReplacedSprings, (ushort2*) m_dRandomPairs,0,massesPerElement-1,seed);

	DevoOptions opt = {
		numReplacedSprings,
		numSprings,
		springsPerElement,
		massesPerElement,
		m_replacedSpringsPerElement,
		COMPOSITE_COUNT
	};
			
	cudaMemcpyToSymbol(cDevoOpt, &opt, sizeof(DevoOptions));
			
	replaceSprings<<<numBlocks, devoThreadsPerBlock>>>(
			(ushort2*) m_dPairs, (uint32_t*) m_dMassMatEncodings,
			(float4*) m_dPos, (float*) m_dLbars,
			(uint32_t*) m_dSpringMatEncodings,
			(uint*) m_dSpringIDs_Sorted, (ushort2*) m_dRandomPairs, 
			m_total_time
		);
	cudaDeviceSynchronize();
	gpuErrchk( cudaPeekAtLastError() );

	seed++;
}