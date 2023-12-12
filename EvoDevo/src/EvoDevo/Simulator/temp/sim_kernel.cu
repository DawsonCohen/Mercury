#include "vec_math.cuh"
#include "material.h"
#include <math.h>
#include <assert.h>

#define EPS (float) 1e-12

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
inline void dragForce(const float4& vel, float3& force,
					float rho) {	
	//Force due to drag = - (1/2 * rho * |v|^2 * A * Cd) * v / |v| (Assume A and Cd are 1)
	float mag_vel_squared = 
		  __fmul_rn(vel.x,vel.x)
		+ __fmul_rn(vel.y,vel.y)
		+ __fmul_rn(vel.z,vel.z);
	float mag_vel = __fsqrt_rn(mag_vel_squared);

	force.x += -0.5*rho*mag_vel*vel.x;
	force.y += -0.5*rho*mag_vel*vel.y;
	force.z += -0.5*rho*mag_vel*vel.z;
}

__device__
inline void environmentForce(float3& pos, const float4& vel, float3& force) {
	dragForce(vel,force,cSimOpt.drag);
}


/*
	mat: float4 describing spring material
		x - k		stiffness
		y - dL0 	maximum percent change
		z - omega	frequency of oscillation
		w - phi		phase
*/
__device__
inline void springForce(const float3& bl, const float3& br, const float4& mat, 
					float mean_length, float time,
					float3& force, float& magF)
{
	float3	dir, diff;

	float	relative_change,
			rest_length,
			Lsqr, rL, L;

	// rest_length = mean_length * (1 + relative_change);
	relative_change = mat.y * sinf(mat.z*time+mat.w);
	rest_length = __fmaf_rn(mean_length, relative_change, mean_length);
	
	diff.x = bl.x - br.x;
	diff.y = bl.y - br.y;
	diff.z = bl.z - br.z;

	Lsqr = dot(diff,diff);
	L = __fsqrt_rn(Lsqr);
	rL = rsqrtf(Lsqr);

	dir = {
		__fmul_rn(diff.x,rL),
		__fmul_rn(diff.y,rL),
		__fmul_rn(diff.z,rL)
	};
	magF = mat.x*(rest_length-L);
	force = magF * dir;
}

__global__ void
inline solveIntegrateBodies(float4 *__restrict__ pos, float4 *__restrict__ vel,
				ushort2 *__restrict__ pairs, uint8_t *__restrict__ matIds, float *__restrict__ Lbars,
				float time, uint step)
{
	extern __shared__ float3 s[];
	float3  *s_pos = s;
	float3  *s_force = (float3*) &s_pos[cSimOpt.massesPerBlock];
	
	uint massOffset   = blockIdx.x * cSimOpt.massesPerBlock;
	uint springOffset = blockIdx.x * cSimOpt.springsPerBlock;
	uint i;

	int tid    = threadIdx.x;
	int stride = blockDim.x;
	
	// Initialize and compute environment forces
	float4 pos4;
	for(i = tid; i < cSimOpt.massesPerBlock && (i+massOffset) < cSimOpt.maxMasses; i+=stride) {
		pos4 = __ldg(&pos[i+massOffset]);
		s_pos[i] = {pos4.x,pos4.y,pos4.z};
	}
	
	for(i = tid; i < cSimOpt.massesPerBlock && (i+massOffset) < cSimOpt.maxMasses; i+=stride) {
		s_force[i] = {0.0f, 0.0f, 0.0f};
	}

	__syncthreads();

	float4	 mat;
	float3	 bl, br;
	float3	 force;
	ushort2	 pair;
	uint8_t  matId;
	float	 Lbar,
			 magF = 0x0f;
	ushort	 left, right;
	
	for(i = tid; i < cSimOpt.springsPerBlock && (i+springOffset) < cSimOpt.maxSprings; i+=stride) {
		matId = __ldg(&matIds[i+springOffset]);
		if(matId == materials::air.id) continue;

		pair = __ldg(&pairs[i+springOffset]);
		left  = pair.x;
		right = pair.y;
		bl = s_pos[left];
		br = s_pos[right];

		mat = compositeMats_id[ matId ];

		Lbar = __ldg(&Lbars[i+springOffset]);
		springForce(bl,br,mat,Lbar,time, force, magF);

		atomicAdd(&(s_force[left].x), force.x);
		atomicAdd(&(s_force[left].y), force.y);
		atomicAdd(&(s_force[left].z), force.z);

		atomicAdd(&(s_force[right].x), -force.x);
		atomicAdd(&(s_force[right].y), -force.y);
		atomicAdd(&(s_force[right].z), -force.z);
	}
	__syncthreads();

	// Calculate and store new mass states
	float4 vel3;
	float3 oldPos, pos3;
	for(i = tid; i < cSimOpt.massesPerBlock && (i+massOffset) < cSimOpt.maxMasses; i+=stride) {
		oldPos = s_pos[i];
		vel3 = __ldg(&vel[i+massOffset]);
		environmentForce(s_pos[i],vel3,s_force[i]);

		// new position = old position + velocity * deltaTime
		s_pos[i].x += vel3.x * cSimOpt.dt + s_force[i].x * cSimOpt.dt*cSimOpt.dt;
		s_pos[i].y += vel3.y * cSimOpt.dt + s_force[i].y * cSimOpt.dt*cSimOpt.dt;
		s_pos[i].z += vel3.z * cSimOpt.dt + s_force[i].z * cSimOpt.dt*cSimOpt.dt;

		// store new position and velocity
		pos3 = s_pos[i];
		vel3.x = cSimOpt.damping * (pos3.x - oldPos.x) / cSimOpt.dt;
		vel3.y = cSimOpt.damping * (pos3.y - oldPos.y) / cSimOpt.dt;
		vel3.z = cSimOpt.damping * (pos3.z - oldPos.z) / cSimOpt.dt;
		pos[i+massOffset] = {pos3.x, pos3.y, pos3.z};
		vel[i+massOffset] = {vel3.x, vel3.y, vel3.z};
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
	uint bytesPerMass = sizeof(float3) + sizeof(float3);
	uint sharedMemSizeSolve = opt.massesPerBlock * bytesPerMass;
	uint numBlocksSolve = numElements;

	assert(sharedMemSizeSolve <= maxSharedMemSize);

	uint bytesPerBoundaryMass = sizeof(float3) + sizeof(float3) + sizeof(float3);
	uint sharedMemSizeDrag = opt.boundaryMassesPerBlock * bytesPerBoundaryMass;
	uint numBlocksDrag = numElements;

	assert(sharedMemSizeDrag <= maxSharedMemSize);

	uint numBlocksPreSolve = (opt.maxMasses + numThreadsPerBlockPreSolve - 1) / numThreadsPerBlockPreSolve;
	uint numBlocksUpdate = (opt.maxMasses + numThreadsPerBlockUpdate - 1) / numThreadsPerBlockUpdate;

	solveIntegrateBodies<<<numBlocksSolve,numThreadsPerBlockPreSolve,sharedMemSizeSolve>>>(
				(float4*) deviceData.dPos, (float4*) deviceData.dVel,
				(ushort2*)  deviceData.dPairs, (uint8_t*) deviceData.dSpringMatIds,  (float*) deviceData.dLbars,
				time, step);

	// surfaceDragForce<<<numBlocksDrag,numThreadsPerBlockDrag,sharedMemSizeDrag>>>(
	// 	(float4*) deviceData.dPos, (float4*) deviceData.dNewPos, 
	// 	(float4*) deviceData.dVel, (ushort4*) deviceData.dFaces);
	// pointDragForce<<<numBlocksPreSolve,numThreadsPerBlockPreSolve>>>(
	// 	(float4*) deviceData.dPos, (float4*) deviceData.dNewPos, 
	// 	(float4*) deviceData.dVel);
	// cudaDeviceSynchronize();

	// preSolve<<<numBlocksPreSolve, numThreadsPerBlockPreSolve>>>(
	// 	(float4*) deviceData.dPos, (float4*) deviceData.dNewPos,
	// 	(float4*) deviceData.dVel);
	// cudaDeviceSynchronize();

	// solveDistance<<<numBlocksSolve,numThreadsPerBlockSolve,sharedMemSizeSolve>>>(
	// 	(float4*) deviceData.dNewPos, (ushort2*)  deviceData.dPairs, 
	// 	(float*) deviceData.dSpringStresses, (uint8_t*) deviceData.dSpringMatIds, (float*) deviceData.dLbars,
	// 	time, step, integrateForce);
	// cudaDeviceSynchronize();
		
	// update<<<numBlocksUpdate,numThreadsPerBlockUpdate>>>((float4*) deviceData.dPos, (float4*) deviceData.dNewPos,
	// 	(float4*) deviceData.dVel);
	cudaDeviceSynchronize();
}

}