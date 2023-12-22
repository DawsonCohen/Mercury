#include "vec_math.cuh"
#include "material.h"
#include <math.h>
#include <assert.h>

#define EPS (float) 1e-12
#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess) 
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}
namespace EvoDevo {

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

__constant__ float4 compositeMats_id[COMPOSITE_COUNT];
__constant__ SimOptions cSimOpt;

void setCompositeMats_id(float* compositeMats, uint count) {
	cudaMemcpyToSymbol(compositeMats_id, compositeMats, sizeof(float)*4*count);
}

void setSimOpts(SimOptions opt) {
	cudaMemcpyToSymbol(cSimOpt, &opt, sizeof(SimOptions));
}

__device__
void surfaceDragForce(
		ushort4 face,
		float3 *s_pos,
		float3 *s_vel,
		float3 *s_force
	) {
	
	if(face.x == face.y || face.x == face.z || face.y == face.z)
			return;
	
	float rho = cSimOpt.drag, area;
	float3 v, normal, force;
	
	// Drag Force: 0.5*rho*A*((Cd - Cl)*dot(v,n)*v + Cl*dot(v,v)*n)
	normal = cross((s_pos[face.y] - s_pos[face.x]), (s_pos[face.z]-s_pos[face.x]));
	v = (s_vel[face.x] + s_vel[face.y] + s_vel[face.z]) / 3.0f;
	area = norm3df(normal.x,normal.y,normal.z);
	normal = normal / (area + EPS);
	normal = dot(normal, v) > 0.0f ? normal : -normal;
	force = -0.5f*rho*area*(0.8*dot(v,normal)*v + 0.2*dot(v,v)*normal);
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

/*
	Exended Positon Based Dynamics
	Computes lagrangian (force) for each distance constraint (spring)

	mat: float4 describing spring material
		x - k		stiffness
		y - dL0 	maximum percent change
		z - omega	frequency of oscillation
		w - phi		phase
*/
__global__ 
void solveDistance(
	float4 *__restrict__ pos,
	float4 *__restrict__ vel,
	ushort2 *__restrict__ pairs, 
	ushort4 *__restrict__ faces, 
	float * __restrict__ stresses,
	uint8_t *__restrict__ matIds,
	float *__restrict__ Lbars,
	float time, uint step, bool integrateForce
	)
{
	extern __shared__ float3 s[];
	float3  *s_pos = s;
	float3  *s_vel = (float3*) &s_pos[cSimOpt.massesPerBlock];
	float3  *s_dp  = (float3*) &s_vel[cSimOpt.massesPerBlock];
	float3  *s_force = s_dp;
	
	uint massOffset   = blockIdx.x * cSimOpt.massesPerBlock;
	uint springOffset = blockIdx.x * cSimOpt.springsPerBlock;
	uint faceOffset = blockIdx.x * cSimOpt.facesPerBlock;
	uint i;

	int tid    = threadIdx.x;
	int stride = blockDim.x;
	
	// Initialize and compute environment forces
	float4 pos4, vel4;
	for(i = tid; i < cSimOpt.massesPerBlock && (i+massOffset) < cSimOpt.maxMasses; i+=stride) {
		s_force[i] = {0.0f, 0.0f, 0.0f};
		pos4 = __ldg(&pos[i+massOffset]);
		vel4 = __ldg(&vel[i+massOffset]);
		s_pos[i] = { pos4.x, pos4.y, pos4.z };
		s_vel[i] = { vel4.x, vel4.y, vel4.z };
	}

	for(i = tid; i < cSimOpt.facesPerBlock && (i+faceOffset) < cSimOpt.maxFaces; i+=stride) {
		surfaceDragForce(
			__ldg(&faces[i+faceOffset]),
			s_pos, s_vel, s_force);
	}

	float3 force3;
	for(i = tid; i < cSimOpt.boundaryMassesPerBlock && (i+massOffset) < cSimOpt.maxMasses; i+=stride) {
		force3 = s_force[i];
		vel[i+massOffset].x += force3.x*cSimOpt.dt;
		vel[i+massOffset].y += force3.y*cSimOpt.dt;
		vel[i+massOffset].z += force3.z*cSimOpt.dt;
		s_dp[i] = {0.0f, 0.0f, 0.0f};
	}
	
	float3 force;
	for(i = tid; i < cSimOpt.massesPerBlock && (i+massOffset) < cSimOpt.maxMasses; i+=stride) {
		vel4 = __ldg(&vel[i+massOffset]);
		pos4 = __ldg(&pos[i+massOffset]);
		s_pos[i] = {
			pos4.x + vel4.x*cSimOpt.dt,
			pos4.y + vel4.y*cSimOpt.dt,
			pos4.z + vel4.z*cSimOpt.dt
		};
	}

	__syncthreads();

	float4	mat;
	uint8_t matId;

	float3	pos0, pos1;
	float	Lbar,
			C, alpha,
			lambda;
	ushort  v0, v1;
	ushort2	pair;

	float3	distance, n;

	float	relative_change,
			rest_length,
			d, K;
	float3	dp;
	
	for(i = tid; i < cSimOpt.springsPerBlock && (i+springOffset) < cSimOpt.maxSprings; i+=stride) {
		matId = __ldg(&matIds[i+springOffset]);
		if(matId == materials::air.id) continue;

		pair = __ldg(&pairs[i+springOffset]);
		Lbar = __ldg(&Lbars[i+springOffset]);
		v0 = pair.x; v1 = pair.y;
		pos0 = s_pos[v0];
		pos1 = s_pos[v1];

		mat = compositeMats_id[ matId ];
		alpha = 1.0f/ mat.x / cSimOpt.dt / cSimOpt.dt;
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

	// Calculate and store new mass states
	float3 newPos3;
	for(i = tid; i < cSimOpt.massesPerBlock && (i+massOffset) < cSimOpt.maxMasses; i+=stride) {
		pos4 = __ldg(&pos[i+massOffset]);
		newPos3 = s_pos[i];
		newPos3 = newPos3 + s_dp[i];
		pos[i+massOffset] = { newPos3.x, newPos3.y, newPos3.z, pos4.w };
		vel[i+massOffset].x = 0.99*(newPos3.x - pos4.x) / cSimOpt.dt;
		vel[i+massOffset].y = 0.99*(newPos3.y - pos4.y) / cSimOpt.dt;
		vel[i+massOffset].z = 0.99*(newPos3.z - pos4.z) / cSimOpt.dt;
	}
}

void integrateBodies(DeviceData deviceData, uint numElements,
	SimOptions opt, 
	float time, uint step, bool integrateForce
	) {
	// Calculate and store new mass states
	
	uint numThreadsPerBlockDrag = 1024;
	uint numThreadsPerBlockPreSolve = 256;
	uint numThreadsPerBlockSolve = 1024;
	uint numThreadsPerBlockUpdate = 256;
	
	/*
	Notes on SM resources:
	thread blocks	: 8
	threads			: 2048
	registers		: 65536
	shared mem		: 49152
	*/

	uint maxSharedMemSize = 49152;
	uint bytesPerMass = sizeof(float3) + sizeof(float3) + sizeof(float3);
	uint sharedMemSizeSolve = opt.massesPerBlock * bytesPerMass;
	uint numBlocksSolve = numElements;

	assert(sharedMemSizeSolve <= maxSharedMemSize);

	uint bytesPerBoundaryMass = sizeof(float3) + sizeof(float3) + sizeof(float3);
	uint sharedMemSizeDrag = opt.boundaryMassesPerBlock * bytesPerBoundaryMass;
	uint numBlocksDrag = numElements;

	assert(sharedMemSizeDrag <= maxSharedMemSize);

	uint numBlocksPreSolve = (opt.maxMasses + numThreadsPerBlockPreSolve - 1) / numThreadsPerBlockPreSolve;
	uint numBlocksUpdate = (opt.maxMasses + numThreadsPerBlockUpdate - 1) / numThreadsPerBlockUpdate;

	// if(opt.boundaryMassesPerBlock > 0) {
		// surfaceDragForce<<<numBlocksDrag,numThreadsPerBlockDrag,sharedMemSizeDrag>>>(
		// 	(float4*) deviceData.dPos, (float4*) deviceData.dVel, (ushort4*) deviceData.dFaces);
	// }
	// cudaDeviceSynchronize();

	solveDistance<<<numBlocksSolve,numThreadsPerBlockSolve,sharedMemSizeSolve>>>(
		(float4*) deviceData.dPos,
		(float4*) deviceData.dVel,
		(ushort2*)  deviceData.dPairs,
		(ushort4*) deviceData.dFaces, 
		(float*) deviceData.dSpringStresses,
		(uint8_t*) deviceData.dSpringMatIds,
		(float*) deviceData.dLbars,
		time, step, integrateForce);
	cudaDeviceSynchronize();
}

}