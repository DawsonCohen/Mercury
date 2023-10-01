#ifndef __SIM_KERNEL_CUH__
#define __SIM_KERNEL_CUH__

#include "vec_math.cuh"
#include "environment.h"
#include <math.h>
#include <assert.h>
#include <stdio.h>

#define EPS (float) 0.000001
#define MIN_DIST (float) 0.005
#define MAX_FORCE (float) 200000

// Explicity assumes each mass is of unit 1 mass
__device__
inline void gravityForce(float3& force, float g) {
	force.y += -g;
}

__device__
inline void collisionForce(float3 pos, float4 vel, float3& force,
					float floor_stiffness, float friction) {
	if(pos.y > 0.0f) return;
	
	float3 Fc = {0.0f, 0.0f, 0.0f}; // force due to collision
	float3 Ff = {0.0f, 0.0f, 0.0f}; // force due to friction
	float magFc, magFf;

	Fc.y = floor_stiffness * (0.0f - pos.y);

	magFc = l2norm(force);
	magFf = friction * Fc.y;

	//Static Friction
	if(fabsf(vel.x) < EPS && fabsf(vel.z) < EPS) {
		//Check if object has acceleration
		if(fabsf(magFc) > EPS) {
			if(magFc < magFf) {
				Ff = {-force.x, 0.0f, -force.y};
			} else {
				//Calculate direction of force and apply friction opposite based on magnitude
				Ff = magFf*normalize(force);
			}
		}
	} else {
		// Kinetic Friction
		Ff = magFf * normalize(make_float3(-vel.x, 0.0f, -vel.z));
	}

	force.x = Fc.x + Ff.x;
	force.y = Fc.y;
	force.z = Fc.z + Ff.z;
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

	if(mag_vel < EPS) return;

	float magFd = min(0.5*rho*mag_vel_squared, MAX_FORCE);

	force.x += -magFd * __fdiv_rn(vel.x, mag_vel);
	force.y += -magFd * __fdiv_rn(vel.y, mag_vel);
	force.z += -magFd * __fdiv_rn(vel.z, mag_vel);
}

// inline void dragForce(const float4& vel, float3& force,
// 					float rho) {	
// 	//Force due to drag = -1/2 * rho * v * |v| * A * Cd (Assume A and Cd are 1)
// 	float mag_vel = __fsqrt_rn(
// 		  __fmul_rn(vel.x,vel.x)
// 		+ __fmul_rn(vel.y,vel.y)
// 		+ __fmul_rn(vel.z,vel.z)
// 		);
// 	force.x += -0.5f*rho*vel.x*mag_vel;
// 	force.y += -0.5f*rho*vel.y*mag_vel;
// 	force.z += -0.5f*rho*vel.z*mag_vel;
// }

__device__
inline void environmentForce(float3 pos, const float4& vel, float3& force,
						const Environment& env) {
	switch(env.type) {
		case ENVIRONMENT_LAND:
			gravityForce(force, env.g);
			collisionForce(pos,vel,force,env.floor_stiffness,env.friction);
		case ENVIRONMENT_WATER:
			dragForce(vel,force,env.drag);
	}
}


/*
	mat: float4 describing spring material
		x - k		stiffness
		y - dL0 	maximum percent change
		z - omega	frequency of oscillation
		w - phi		phase
*/
__device__
inline void springForce(float3 bl, float3 br, float4 mat, 
					float mean_length, float time,
					float3& force, float& magF)
{
	if(mat.x == 0.0f) {
		force = {0.0f, 0.0f, 0.0f};
		magF = 0.0f;
		return;
	}

	float3	dir, diff;

	float	relative_change,
			rest_length,
			L;

	// b0pos = {bl.x, bl.y, bl.z};
	// b1pos = {br.x, br.y, br.z};

	// rest_length = mean_length * (1 + relative_change);
	relative_change = mat.y * sinf(mat.z*time+mat.w);
	rest_length = __fmaf_rn(mean_length, relative_change, mean_length);
	
	// rest_length = __fmaf_rn(mean_length*mat.y, sinf(mat.z*time+mat.w), mean_length);
	
	diff.x = bl.x - br.x;
	diff.y = bl.y - br.y;
	diff.z = bl.z - br.z;

	L = l2norm(diff);
	dir = {
		__fdiv_rn(diff.x,L),
		__fdiv_rn(diff.y,L),
		__fdiv_rn(diff.z,L)
	};
	if(isnan(dir.x) || isnan(dir.y) || isnan(dir.z)) {
		force = {0.0f, 0.0f, 0.0f};
	} else {
		magF = min(mat.x*(rest_length-L), MAX_FORCE);
		force = magF * dir;
	}
}

struct SimOptions {
	float dt;
	bool devo;
	uint massesPerBlock;
	uint springsPerBlock;
	uint maxMasses;
	uint maxSprings;
	uint materialCount;
	short shiftskip;
	Environment env;
};

// TODO: test with single spring
__global__ void
inline integrateBodies(float4 *__restrict__ newPos, float4 *__restrict__ newVel,
				float4 *__restrict__ oldPos, float4 *__restrict__ oldVel,
				ushort2 *__restrict__ pairs, uint8_t *__restrict__ matEncodings, float *__restrict__ Lbars,
				float4 *__restrict__ compositeMats,
				float time, uint step, SimOptions opt)
{
	extern __shared__ float3 s[];
	float3  *s_pos = s;
	float3  *s_force = (float3*) &s_pos[opt.massesPerBlock];
	float4	*s_compositeMats = (float4*) &s_force[opt.massesPerBlock];
	
	uint massOffset   = blockIdx.x * opt.massesPerBlock;
	uint springOffset = blockIdx.x * opt.springsPerBlock;

	int tid    = threadIdx.x;
	int stride = blockDim.x;
	
	// Initialize and compute environment forces
	float4 pos4;
	for(uint i = tid; i < opt.massesPerBlock && (i+massOffset) < opt.maxMasses; i+=stride) {
		pos4 = oldPos[i+massOffset];
		s_pos[i] = {pos4.x,pos4.y,pos4.z};
	}
	
	for(uint i = tid; i < opt.massesPerBlock && (i+massOffset) < opt.maxMasses; i+=stride) {
		s_force[i] = {0.0f, 0.0f, 0.0f};
		environmentForce(s_pos[i],oldVel[i+massOffset],s_force[i],opt.env);
	}

	for(uint i = tid; i < (1 << opt.materialCount); i += stride) {
		s_compositeMats[i] = compositeMats[i];
	}
	__syncthreads();

	float4	mat;
	float3	bl, br;
	float3	force;
	ushort2	pair;
	uint8_t	matEncoding;
	float	Lbar,
			magF;
	ushort	left, right;
	
	uint i;
	for(i = tid; i < opt.springsPerBlock && (i+springOffset) < opt.maxSprings; i+=stride) {
		pair = __ldg(&pairs[i+springOffset]);
		matEncoding = __ldg(&matEncodings[i+springOffset]);
		left  = pair.x;
		right = pair.y;
		bl = s_pos[left];
		br = s_pos[right];

		mat = s_compositeMats[ matEncoding ];

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
	float4 vel;
	float3 pos3;
	for(uint i = tid; i < opt.massesPerBlock && (i+massOffset) < opt.maxMasses; i+=stride) {
		vel = oldVel[i+massOffset];

		vel.x += (s_force[i].x * opt.dt)*opt.env.damping;
		vel.y += (s_force[i].y * opt.dt)*opt.env.damping;
		vel.z += (s_force[i].z * opt.dt)*opt.env.damping;

		// new position = old position + velocity * deltaTime
		s_pos[i].x += vel.x * opt.dt;
		s_pos[i].y += vel.y * opt.dt;
		s_pos[i].z += vel.z * opt.dt;

		// store new position and velocity
		// assert(!isnan(pos3.x) && !isnan(pos3.y) && !isnan(pos3.z));
		pos3 = s_pos[i];
		newPos[i+massOffset] = {pos3.x, pos3.y, pos3.z};
		newVel[i+massOffset] = vel;
	}
}


__global__ void
inline integrateBodiesStresses(float4 *__restrict__ newPos, float4 *__restrict__ newVel,
				float4 *__restrict__ oldPos, float4 *__restrict__ oldVel,
				ushort2 *__restrict__ pairs, uint8_t *__restrict__ matEncodings, float *__restrict__ Lbars,
				ushort *__restrict__ maxStressCount, ushort *__restrict__ minStressCount,
				float *__restrict__ stresses, uint *__restrict__ ids,
				float4 *__restrict__ compositeMats,
				float time, uint step, SimOptions opt)
{
	extern __shared__ float3 s[];
	float3  *s_pos = s;
	float3  *s_force = (float3*) &s_pos[opt.massesPerBlock];
	float4	*s_compositeMats = (float4*) &s_force[opt.massesPerBlock];

	__shared__ ushort maxStressedSprings[1024];
	__shared__ ushort minStressedSprings[1024];
	
	uint massOffset   = blockIdx.x * opt.massesPerBlock;
	uint springOffset = blockIdx.x * opt.springsPerBlock;

	int tid    = threadIdx.x;
	int stride = blockDim.x;
	
	// Initialize and compute environment forces
	float4 pos4;
	for(uint i = tid; i < opt.massesPerBlock && (i+massOffset) < opt.maxMasses; i+=stride) {
		pos4 = oldPos[i+massOffset];
		s_pos[i] = {pos4.x,pos4.y,pos4.z};
	}
	
	for(uint i = tid; i < opt.massesPerBlock && (i+massOffset) < opt.maxMasses; i+=stride) {
		s_force[i] = {0.0f, 0.0f, 0.0f};
		environmentForce(s_pos[i],oldVel[i+massOffset],s_force[i],opt.env);
	}

	for(uint i = tid; i < opt.materialCount*opt.materialCount; i += stride) {
		s_compositeMats[i] = compositeMats[i];
	}
	__syncthreads();

	float4	mat;
	float3	bl, br;
	float3	force;
	ushort2	pair;
	uint8_t	matEncoding;
	float	Lbar,
			magF;
	ushort	left, right;

	float	minNormalizedStress = 0.0f,
			maxNormalizedStress  = 0.0f,
			nextMinStress = 0.0f;
	ushort	maxSpringIdx = tid,
			minSpringIdx = tid,
			nextMinSpringIdx = tid+stride,
			nextGroup_MaxSpringIdx,
			nextGroup_MinSpringIdx;
	
	uint i;
	for(i = tid; i < opt.springsPerBlock && (i+springOffset) < opt.maxSprings; i+=stride) {
		pair = __ldg(&pairs[i+springOffset]);
		matEncoding = __ldg(&matEncodings[i+springOffset]);
		left  = pair.x;
		right = pair.y;
		bl = s_pos[left];
		br = s_pos[right];

		mat = s_compositeMats[ matEncoding ];
		
		Lbar = __ldg(&Lbars[i+springOffset]);
		springForce(bl,br,mat,Lbar,time, force, magF);

		// assert(!isnan(force.x) && !isnan(force.y) && !isnan(force.z));

		atomicAdd(&(s_force[left].x), force.x);
		atomicAdd(&(s_force[left].y), force.y);
		atomicAdd(&(s_force[left].z), force.z);

		atomicAdd(&(s_force[right].x), -force.x);
		atomicAdd(&(s_force[right].y), -force.y);
		atomicAdd(&(s_force[right].z), -force.z);

		if(step % (opt.shiftskip+1) == 0) {
			if(fabsf(magF) > maxNormalizedStress) {
				maxNormalizedStress = __fdiv_rn(fabsf(magF), Lbar);
				maxSpringIdx = i;
			}
			if(((fabsf(magF) < minNormalizedStress) || (minNormalizedStress == 0.0f)) && fabsf(magF) > 0.0f) {
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

	ushort2 cMaxPair,
			cMinPair;
	uint8_t	cMaxMatEncoding,
			cMinMatEncoding;
	ushort	cMax_MaxCount,
			cMax_MinCount,
			cMin_MaxCount,
			cMin_MinCount;
	float	cMaxLbar,
			cMinLbar;
	#ifdef FULL_STRESS
	float	cMaxStress,
			cMinStress;
	float	cMaxID,
			cMinID;
	#endif

	
	if(minSpringIdx == maxSpringIdx) { // guarentees not both max and min
		minSpringIdx = nextMinSpringIdx;
		minNormalizedStress = nextMinStress;
	}

	if(step % (opt.shiftskip+1) == 0) {
		maxStressCount[maxSpringIdx + springOffset] += (maxNormalizedStress > 0.0f);
		minStressCount[minSpringIdx + springOffset] += (minNormalizedStress > 0.0f);
		maxStressedSprings[tid]  = maxSpringIdx;
		minStressedSprings[tid]  = minSpringIdx;
		
		// current thread max spring info
		cMaxPair = pairs[maxSpringIdx + springOffset];
		cMax_MaxCount = maxStressCount[maxSpringIdx + springOffset];
		cMax_MinCount = minStressCount[maxSpringIdx + springOffset];
		cMaxMatEncoding = matEncodings[maxSpringIdx + springOffset];
		cMaxLbar = Lbars[maxSpringIdx + springOffset];

		cMinPair = pairs[minSpringIdx + springOffset];
		cMin_MaxCount = maxStressCount[minSpringIdx + springOffset];
		cMin_MinCount = minStressCount[minSpringIdx + springOffset];
		cMinMatEncoding = matEncodings[minSpringIdx + springOffset];
		cMinLbar = Lbars[minSpringIdx + springOffset];

		#ifdef FULL_STRESS
		cMaxStress = stresses[maxSpringIdx + springOffset];
		cMaxID = ids[maxSpringIdx + springOffset];
		cMinStress = stresses[minSpringIdx + springOffset];
		cMinID = ids[minSpringIdx + springOffset];
		#endif
	}
	__syncthreads();

	int tid_next = (tid+1) % blockDim.x;
	if(step % (opt.shiftskip+1) == 0) {
		// shift current index to next spring
		nextGroup_MaxSpringIdx  = maxStressedSprings[tid_next];
		nextGroup_MinSpringIdx = minStressedSprings[tid_next];

		pairs[nextGroup_MaxSpringIdx + springOffset] = cMaxPair;
		matEncodings[nextGroup_MaxSpringIdx + springOffset] = cMaxMatEncoding;
		Lbars[nextGroup_MaxSpringIdx + springOffset] = cMaxLbar;
		maxStressCount[nextGroup_MaxSpringIdx + springOffset] = cMax_MaxCount;
		minStressCount[nextGroup_MaxSpringIdx + springOffset] = cMax_MinCount;

		pairs[nextGroup_MinSpringIdx + springOffset] = cMinPair;
		matEncodings[nextGroup_MinSpringIdx + springOffset] = cMinMatEncoding;
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
	float4 vel;
	float3 pos3;
	for(uint i = tid; i < opt.massesPerBlock && (i+massOffset) < opt.maxMasses; i+=stride) {
		vel = oldVel[i+massOffset];

		vel.x += (s_force[i].x * opt.dt)*opt.env.damping;
		vel.y += (s_force[i].y * opt.dt)*opt.env.damping;
		vel.z += (s_force[i].z * opt.dt)*opt.env.damping;

		// new position = old position + velocity * deltaTime
		s_pos[i].x += vel.x * opt.dt;
		s_pos[i].y += vel.y * opt.dt;
		s_pos[i].z += vel.z * opt.dt;

		// store new position and velocity
		pos3 = s_pos[i];
		newPos[i+massOffset] = {pos3.x, pos3.y, pos3.z};
		newVel[i+massOffset] = vel;
	}
}

__global__ void
inline replaceSprings(ushort2 *__restrict__ pairs, ushort *__restrict__ maxStressCount, ushort *__restrict__ minStressCount) {
	
}

#endif
