#ifndef __SIM_KERNEL_CUH__
#define __SIM_KERNEL_CUH__

#include "vec_math.cuh"
#include "environment.h"
#include "material.h"
#include <math.h>
#include <stdint.h>
#include <assert.h>

#define EPS (float) 0.000001
#define MAX_FORCE (float) 200000

struct SimOptions {
	float dt;
	uint massesPerBlock;
	uint springsPerBlock;
	uint maxMasses;
	uint maxSprings;
	uint compositeCount;
	short shiftskip;
	float drag;
	float damping;
};

__constant__ float4 compositeMats_id[COMPOSITE_COUNT];
__constant__ SimOptions cSimOpt;

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
inline integrateBodies(float4 *__restrict__ pos, float4 *__restrict__ vel,
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
	float4 velocity;
	float3 pos3;
	for(i = tid; i < cSimOpt.massesPerBlock && (i+massOffset) < cSimOpt.maxMasses; i+=stride) {
		velocity = __ldg(&vel[i+massOffset]);
		environmentForce(s_pos[i],velocity,s_force[i]);

		velocity.x += (s_force[i].x * cSimOpt.dt)*cSimOpt.damping;
		velocity.y += (s_force[i].y * cSimOpt.dt)*cSimOpt.damping;
		velocity.z += (s_force[i].z * cSimOpt.dt)*cSimOpt.damping;

		// new position = old position + velocity * deltaTime
		s_pos[i].x += velocity.x * cSimOpt.dt;
		s_pos[i].y += velocity.y * cSimOpt.dt;
		s_pos[i].z += velocity.z * cSimOpt.dt;

		// store new position and velocity
		pos3 = s_pos[i];
		pos[i+massOffset] = {pos3.x, pos3.y, pos3.z};
		vel[i+massOffset] = velocity;
	}
}


__global__ void
inline integrateBodiesStresses(float4 *__restrict__ pos, float4 *__restrict__ vel,
				ushort2 *__restrict__ pairs, uint32_t *__restrict__ matEncodings, uint8_t *__restrict__ matIds, float *__restrict__ Lbars,
				ushort *__restrict__ maxStressCount, ushort *__restrict__ minStressCount,
				float *__restrict__ stresses, uint *__restrict__ ids,
				float time, uint step)
{
	extern __shared__ float3 s[];
	float3  *s_pos = s;
	float3  *s_force = (float3*) &s_pos[cSimOpt.massesPerBlock];

	__shared__ ushort maxStressedSprings[1024];
	__shared__ ushort minStressedSprings[1024];
	
	uint massOffset   = blockIdx.x * cSimOpt.massesPerBlock;
	uint springOffset = blockIdx.x * cSimOpt.springsPerBlock;

	int tid    = threadIdx.x;
	int stride = blockDim.x;
	
	// Initialize and compute environment forces
	float4 pos4;
	for(uint i = tid; i < cSimOpt.massesPerBlock && (i+massOffset) < cSimOpt.maxMasses; i+=stride) {
		pos4 = __ldg(&pos[i+massOffset]);
		s_pos[i] = {pos4.x,pos4.y,pos4.z};
	}
	
	for(uint i = tid; i < cSimOpt.massesPerBlock && (i+massOffset) < cSimOpt.maxMasses; i+=stride) {
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

	float	minNormalizedStress = 0.0f,
			maxNormalizedStress  = 0.0f,
			nextMinStress = 0.0f;
	ushort	maxSpringIdx = tid,
			minSpringIdx = tid,
			nextMinSpringIdx = tid+stride,
			nextGroup_MaxSpringIdx,
			nextGroup_MinSpringIdx;
	
	uint i;
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

		// assert(!isnan(force.x) && !isnan(force.y) && !isnan(force.z));

		atomicAdd(&(s_force[left].x), force.x);
		atomicAdd(&(s_force[left].y), force.y);
		atomicAdd(&(s_force[left].z), force.z);

		atomicAdd(&(s_force[right].x), -force.x);
		atomicAdd(&(s_force[right].y), -force.y);
		atomicAdd(&(s_force[right].z), -force.z);

		if(step % (cSimOpt.shiftskip+1) == 0) {
			if(fabsf(magF) > maxNormalizedStress) {
				maxNormalizedStress = __fdiv_rn(fabsf(magF), Lbar);
				maxSpringIdx = i;
			}
			if(((fabsf(magF) < minNormalizedStress) || (minNormalizedStress == 0.0f))) {
				if(i > tid) {
					nextMinStress = minNormalizedStress;
					nextMinSpringIdx = minSpringIdx;
				}

				minNormalizedStress = __fdiv_rn(fabsf(magF), Lbar);
				minSpringIdx = i;
				
			}
		}

		#ifdef FULL_STRESS
		stresses[i + springOffset] = stresses[i + springOffset] + fabsf(magF);
		#endif
	}

	ushort2  cMaxPair,
			 cMinPair;
	uint32_t cMaxMatEncoding,
			 cMinMatEncoding;
	uint8_t  cMaxMatId,
			 cMinMatId;
	ushort	 cMax_MaxCount,
			 cMax_MinCount,
			 cMin_MaxCount,
			 cMin_MinCount;
	float	 cMaxLbar,
			 cMinLbar;
	#ifdef FULL_STRESS
	float	 cMaxStress,
			 cMinStress;
	float	 cMaxID,
			 cMinID;
	#endif

	
	if(minSpringIdx == maxSpringIdx) { // guarentees not both max and min
		minSpringIdx = nextMinSpringIdx;
		minNormalizedStress = nextMinStress;
	}

	if(step % (cSimOpt.shiftskip+1) == 0) {
		maxStressCount[maxSpringIdx + springOffset] += (maxNormalizedStress > 0.0f);
		minStressCount[minSpringIdx + springOffset] += (minNormalizedStress > 0.0f);
		maxStressedSprings[tid]  = maxSpringIdx;
		minStressedSprings[tid]  = minSpringIdx;
		
		// current thread max spring info
		cMaxPair = pairs[maxSpringIdx + springOffset];
		cMax_MaxCount = maxStressCount[maxSpringIdx + springOffset];
		cMax_MinCount = minStressCount[maxSpringIdx + springOffset];
		cMaxMatEncoding = matEncodings[maxSpringIdx + springOffset];
		cMaxMatId = matIds[maxSpringIdx + springOffset];
		cMaxLbar = Lbars[maxSpringIdx + springOffset];

		cMinPair = pairs[minSpringIdx + springOffset];
		cMin_MaxCount = maxStressCount[minSpringIdx + springOffset];
		cMin_MinCount = minStressCount[minSpringIdx + springOffset];
		cMinMatEncoding = matEncodings[minSpringIdx + springOffset];
		cMinMatId = matIds[minSpringIdx + springOffset];
		cMinLbar = Lbars[minSpringIdx + springOffset];

		#ifdef FULL_STRESS
		cMaxStress = stresses[maxSpringIdx + springOffset];
		cMaxID = ids[maxSpringIdx + springOffset];
		cMinStress = stresses[minSpringIdx + springOffset];
		cMinID = ids[minSpringIdx + springOffset];
		#endif
	}
	__syncthreads();

	int tid_next;
	if(step % (cSimOpt.shiftskip+1) == 0) {
		tid_next = (tid+1) % blockDim.x;
		// shift current index to next spring
		nextGroup_MaxSpringIdx  = maxStressedSprings[tid_next];
		nextGroup_MinSpringIdx = minStressedSprings[tid_next];

		pairs[nextGroup_MaxSpringIdx + springOffset] = cMaxPair;
		matEncodings[nextGroup_MaxSpringIdx + springOffset] = cMaxMatEncoding;
		matIds[nextGroup_MaxSpringIdx + springOffset] = cMaxMatId;
		Lbars[nextGroup_MaxSpringIdx + springOffset] = cMaxLbar;
		maxStressCount[nextGroup_MaxSpringIdx + springOffset] = cMax_MaxCount;
		minStressCount[nextGroup_MaxSpringIdx + springOffset] = cMax_MinCount;

		pairs[nextGroup_MinSpringIdx + springOffset] = cMinPair;
		matEncodings[nextGroup_MinSpringIdx + springOffset] = cMinMatEncoding;
		matIds[nextGroup_MinSpringIdx + springOffset] = cMinMatId;
		Lbars[nextGroup_MinSpringIdx + springOffset] = cMinLbar;
		maxStressCount[nextGroup_MinSpringIdx + springOffset] = cMin_MaxCount;
		minStressCount[nextGroup_MinSpringIdx + springOffset] = cMin_MinCount;

		#ifdef FULL_STRESS
		stresses[nextGroup_MaxSpringIdx + springOffset] = cMaxStress;
		ids[nextGroup_MaxSpringIdx + springOffset] = cMaxID;

		stresses[nextGroup_MinSpringIdx + springOffset] = cMinStress;
		ids[nextGroup_MinSpringIdx + springOffset] = cMinID;
		#endif
	}

	// Calculate and store new mass states
	float4 velocity;
	float3 pos3;
	for(uint i = tid; i < cSimOpt.massesPerBlock && (i+massOffset) < cSimOpt.maxMasses; i+=stride) {
		velocity = __ldg(&vel[i+massOffset]);
		environmentForce(s_pos[i],velocity,s_force[i]);

		velocity.x += (s_force[i].x * cSimOpt.dt)*cSimOpt.damping;
		velocity.y += (s_force[i].y * cSimOpt.dt)*cSimOpt.damping;
		velocity.z += (s_force[i].z * cSimOpt.dt)*cSimOpt.damping;

		// new position = old position + velocity * deltaTime
		s_pos[i].x += velocity.x * cSimOpt.dt;
		s_pos[i].y += velocity.y * cSimOpt.dt;
		s_pos[i].z += velocity.z * cSimOpt.dt;

		// store new position and velocity
		pos3 = s_pos[i];
		pos[i+massOffset] = {pos3.x, pos3.y, pos3.z};
		vel[i+massOffset] = velocity;
	}
}

#endif
