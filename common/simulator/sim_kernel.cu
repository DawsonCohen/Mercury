#ifndef __SIM_KERNEL_CUH__
#define __SIM_KERNEL_CUH__

#include "vec_math.cuh"
#include "environment.h"
#include "material.h"
#include <math.h>
#include <stdint.h>
#include <assert.h>

#define EPS (float) 1e-12
#define MAX_FORCE (float) 200000

struct SimOptions {
	float dt;
	uint massesPerBlock;
	uint springsPerBlock;
	uint facesPerBlock;
	uint boundaryMassesPerBlock;
	uint maxMasses;
	uint maxSprings;
	uint maxFaces;
	uint compositeCount;
	short shiftskip;
	float drag;
	float damping;
	float relaxation;
	float s;
};

__constant__ float4 compositeMats_id[COMPOSITE_COUNT];
__constant__ SimOptions cSimOpt;

__global__ inline
void dragForce(float4 *__restrict__ pos, float4 *__restrict__ newPos,
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
	}
	
	for(i = tid; i < cSimOpt.boundaryMassesPerBlock && (i+massOffset) < cSimOpt.maxMasses; i+=stride) {
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
		if(face.w == 0) continue;

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
void solveDistance(float4 *__restrict__ newPos, float4 *__restrict__ vel,
				float * __restrict__ stresses,
				ushort2 *__restrict__ pairs, uint8_t *__restrict__ matIds, float *__restrict__ Lbars,
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
	}
	
	for(i = tid; i < cSimOpt.massesPerBlock && (i+massOffset) < cSimOpt.maxMasses; i+=stride) {
		s_dp[i] = {0.0f, 0.0f, 0.0f};
	}

	__syncthreads();

	float4	 mat;
	float3	 posL, posR;
	ushort2	 pair;
	uint8_t  matId;
	float	 Lbar,
			 constraint, compliance,
			 lambda;
	ushort	 left, right;

	float3	diff;

	float	relative_change,
			rest_length,
			n;
	float3  dp;
	
	for(i = tid; i < cSimOpt.springsPerBlock && (i+springOffset) < cSimOpt.maxSprings; i+=stride) {
		matId = __ldg(&matIds[i+springOffset]);
		if(matId == materials::air.id) continue;

		pair = __ldg(&pairs[i+springOffset]);
		left = pair.x; right = pair.y;
		posL = s_pos[left];
		posR = s_pos[right];

		mat = compositeMats_id[ matId ];

		Lbar = __ldg(&Lbars[i+springOffset]);
	
		// rest_length = mean_length * (1 + relative_change);
		relative_change = mat.y * sinf(mat.z*time+mat.w);
		rest_length = __fmaf_rn(Lbar, relative_change, Lbar);
		
		diff = posL-posR;
		n = l2norm(diff);
		// if(n < EPS) {
		// 	printf("%u: (%f,%f,%f)\n",step, diff.x,diff.y,diff.z);
		// 	assert(0);
		// }
		constraint = n-rest_length;
		compliance = 1 / mat.x / cSimOpt.dt / cSimOpt.dt;
		lambda = -(constraint) / (2 + compliance);
		dp = lambda * diff / (n + EPS);

		if(integrateForce) stresses[i+springOffset] += lambda / Lbar;

		atomicAdd(&(s_dp[left].x), dp.x);
		atomicAdd(&(s_dp[left].y), dp.y);
		atomicAdd(&(s_dp[left].z), dp.z);

		atomicAdd(&(s_dp[right].x), -dp.x);
		atomicAdd(&(s_dp[right].y), -dp.y);
		atomicAdd(&(s_dp[right].z), -dp.z);
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

#endif
