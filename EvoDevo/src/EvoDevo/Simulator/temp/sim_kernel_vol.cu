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

__global__
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

__global__
void pointDragForce(float4 *__restrict__ pos, float4 *__restrict__ newPos,
                 float4 *__restrict__ vel) {
	int stride = blockDim.x * gridDim.x;
	float4 velocity, pos4;

	float3 force;
	float  mag_vel;
	for(uint i = blockIdx.x * blockDim.x + threadIdx.x;
		i < cSimOpt.maxMasses; i+=stride) {
		//Force due to drag = - (1/2 * rho * |v|^2 * A * Cd) * v / |v| (Assume A and Cd are 1)
		velocity = __ldg(&vel[i]);
		mag_vel = norm3df(velocity.x,velocity.y,velocity.z);

		force.x = -0.5*cSimOpt.drag*mag_vel*velocity.x;
		force.y = -0.5*cSimOpt.drag*mag_vel*velocity.y;
		force.z = -0.5*cSimOpt.drag*mag_vel*velocity.z;

		pos4 = __ldg(&pos[i]);
		newPos[i].x = pos4.x + velocity.x*cSimOpt.dt + force.x*cSimOpt.dt*cSimOpt.dt;
		newPos[i].y = pos4.y + velocity.y*cSimOpt.dt + force.y*cSimOpt.dt*cSimOpt.dt;
		newPos[i].z = pos4.z + velocity.z*cSimOpt.dt + force.z*cSimOpt.dt*cSimOpt.dt;
	}
}

__global__
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
__global__
void solveDistance(float4 *__restrict__ newPos, ushort2 *__restrict__ pairs, 
				float * __restrict__ stresses,
				uint8_t *__restrict__ matIds, float *__restrict__ Lbars,
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

	float	d, K;
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
		
		K = 2.0f + alpha;
		distance = pos0-pos1;
		d = l2norm(distance);
		n = distance / (d + EPS);
		
		C = d-Lbar;
		lambda = -C / K;
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
void solveVolume(float4 *__restrict__ newPos, ushort4 *__restrict__ cells,
                float * __restrict__ cellStresses,
				float4 *__restrict__ mats, float *__restrict__ Vbars,
                float time, uint step, bool integrateForce
				)
{
	extern __shared__ float3 s[];
	float3  *s_pos = s;
	float3  *s_dp = (float3*) &s_pos[cSimOpt.massesPerBlock];
	
	uint massOffset = blockIdx.x * cSimOpt.massesPerBlock;
	uint cellOffset = blockIdx.x * cSimOpt.cellsPerBlock;
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
	float3	 pos0, pos1, pos2, pos3;
	float3	 grad0, grad1, grad2, grad3;
	ushort4	 cell;
	float	 Vbar,
			 C, alpha,
			 lambda, K;
	ushort	 v0, v1, v2, v3;

	float    volume;

	float	 relative_change,
			 rest_volume;
	
	for(i = tid; i < cSimOpt.cellsPerBlock && (i+cellOffset) < cSimOpt.maxCells; i+=stride) {
		cell = __ldg(&cells[i+cellOffset]);
		if(cell.x == cell.y) continue;
		mat = __ldg(&mats[i+cellOffset]);
		if(mat.x == 0.0f) continue;

		v0 = cell.x; v1 = cell.y; v2 = cell.z; v3 = cell.w;
		pos0 = s_pos[v0];
		pos1 = s_pos[v1];
		pos2 = s_pos[v2];
		pos3 = s_pos[v3];

		Vbar = __ldg(&Vbars[i+cellOffset]);
		
		// rest_length = mean_volume * (1 + relative_change);
		relative_change = mat.y * sinf(mat.z*time+mat.w);
		rest_volume = __fmaf_rn(Vbar, relative_change, Vbar);
		
		volume = dot( cross((pos1-pos0),(pos2-pos0)), (pos3-pos0) ) / 6.0f;
		alpha = 1.0f / mat.x / cSimOpt.dt / cSimOpt.dt;
		grad0 = cross( (pos3-pos1), (pos2-pos1) );
		grad1 = cross( (pos2-pos0), (pos3-pos0) );
		grad2 = cross( (pos3-pos0), (pos1-pos0) );
		grad3 = cross( (pos1-pos0), (pos2-pos0) );
		K = squaredL2norm( grad0 ) + 
		    squaredL2norm( grad1 ) +
			squaredL2norm( grad2 ) +
			squaredL2norm( grad3 ) +
			alpha;
		
		C = volume - rest_volume;
		lambda = -C / K;

		if(integrateForce) cellStresses[i+cellOffset] += lambda / Vbar;

		atomicAdd(&(s_dp[v0].x), lambda*grad0.x);
		atomicAdd(&(s_dp[v0].y), lambda*grad0.y);
		atomicAdd(&(s_dp[v0].z), lambda*grad0.z);

		atomicAdd(&(s_dp[v1].x), lambda*grad1.x);
		atomicAdd(&(s_dp[v1].y), lambda*grad1.y);
		atomicAdd(&(s_dp[v1].z), lambda*grad1.z);

		atomicAdd(&(s_dp[v2].x), lambda*grad2.x);
		atomicAdd(&(s_dp[v2].y), lambda*grad2.y);
		atomicAdd(&(s_dp[v2].z), lambda*grad2.z);

		atomicAdd(&(s_dp[v3].x), lambda*grad3.x);
		atomicAdd(&(s_dp[v3].y), lambda*grad3.y);
		atomicAdd(&(s_dp[v3].z), lambda*grad3.z);
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

__global__ void update(float4 *__restrict__ pos, float4 *__restrict__ newPos, float4 *__restrict__ vel) {
	// Calculate and store new mass states
	int stride = blockDim.x * gridDim.x;

	float4 newPos4, pos4;
	for(uint i = blockIdx.x * blockDim.x + threadIdx.x; i < cSimOpt.maxMasses; i+=stride) {
		pos4 = __ldg(&pos[i]);
		newPos4 = __ldg(&newPos[i]);
		pos[i] = newPos4;
		vel[i].x = (newPos4.x - pos4.x) / cSimOpt.dt;
		vel[i].y = (newPos4.y - pos4.y) / cSimOpt.dt;
		vel[i].z = (newPos4.z - pos4.z) / cSimOpt.dt;
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

	surfaceDragForce<<<numBlocksDrag,numThreadsPerBlockDrag,sharedMemSizeDrag>>>(
		(float4*) deviceData.dPos, (float4*) deviceData.dNewPos, 
		(float4*) deviceData.dVel, (ushort4*) deviceData.dFaces);
	// pointDragForce<<<numBlocksPreSolve,numThreadsPerBlockPreSolve>>>(
	// 	(float4*) deviceData.dPos, (float4*) deviceData.dNewPos, 
	// 	(float4*) deviceData.dVel);
	cudaDeviceSynchronize();

	preSolve<<<numBlocksPreSolve, numThreadsPerBlockPreSolve>>>(
		(float4*) deviceData.dPos, (float4*) deviceData.dNewPos,
		(float4*) deviceData.dVel);
	cudaDeviceSynchronize();

	solveDistance<<<numBlocksSolve,numThreadsPerBlockSolve,sharedMemSizeSolve>>>(
		(float4*) deviceData.dNewPos, (ushort2*)  deviceData.dPairs, 
		(float*) deviceData.dSpringStresses, (uint8_t*) deviceData.dSpringMatIds, (float*) deviceData.dLbars,
		time, step, integrateForce);

	solveVolume<<<numBlocksSolve,numThreadsPerBlockSolve,sharedMemSizeSolve>>>(
			(float4*) deviceData.dNewPos, (ushort4*) deviceData.dCells,
			(float*) deviceData.dCellStresses,
			(float4*) deviceData.dMats, (float*) deviceData.dVbars,
			time, step, integrateForce
		);
	cudaDeviceSynchronize();
		
	update<<<numBlocksUpdate,numThreadsPerBlockUpdate>>>((float4*) deviceData.dPos, (float4*) deviceData.dNewPos,
		(float4*) deviceData.dVel);
	cudaDeviceSynchronize();
}

}