#ifndef __SIM_KERNEL_H__
#define __SIM_KERNEL_H__

#include <math.h>
#include <algorithm>
#include <functional>
#include <fstream>
#include <iostream>

__host__ __device__ __forceinline__ float3 operator+(const float3 &a, const float3 &b) {
	return make_float3(a.x+b.x, a.y+b.y, a.z+b.z);
}

__host__ __device__ __forceinline__ float3 operator+=(const float3 &a, const float3 &b) {
	return make_float3(a.x+b.x, a.y+b.y, a.z+b.z);
}

__host__ __device__ __forceinline__ float4 operator+(const float4 &a, const float4 &b) {
	return make_float4(a.x+b.x, a.y+b.y, a.z+b.z, a.w+b.w);
}

__host__ __device__ __forceinline__ float4 operator+=(const float4 &a, const float4 &b) {
	return make_float4(a.x+b.x, a.y+b.y, a.z+b.z, a.w+b.w);
}

__host__ __device__ __forceinline__ float3 operator-(const float3 &a, const float3 &b) {
	return make_float3(a.x-b.x, a.y-b.y, a.z-b.z);
}

__host__ __device__ __forceinline__ float3 operator*(const float3 &a, const float3 &b) {
	return make_float3(a.x*b.x, a.y*b.y, a.z*b.z);
}

__host__ __device__ __forceinline__ float4 operator*(const float4 &a, const float4 &b) {
	return make_float4(a.x*b.x, a.y*b.y, a.z*b.z, a.w*b.w);
}

__host__ __device__ __forceinline__ float3 operator*(const float &a, const float3 &vec) {
	return make_float3(a*vec.x, a*vec.y, a*vec.z);
}

__host__ __device__ __forceinline__ float3 operator*(const float3 &vec, const float &a) {
	return make_float3(a*vec.x, a*vec.y, a*vec.z);
}

__host__ __device__ __forceinline__ float3 operator/(const float3 &vec, const float &a) {
	return make_float3(vec.x/a, vec.y/a, vec.z/a);
}

__host__ __device__ __forceinline__ float3 operator-(const float3 &a) {
	return make_float3(-a.x, -a.y, -a.z);
}

__host__ __device__ __forceinline__ float dot(const float3 &a, const float3 &b) {
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

__host__ __device__ __forceinline__ float dot(const float4 &a, const float4 &b) {
	return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
}

__host__ __device__ __forceinline__ float l2norm(const float3 &a) {
	return sqrtf(dot(a,a));
}

__host__ __device__ __forceinline__ float l2norm(const float4 &a) {
	return sqrtf(dot(a,a));
}

__host__ __device__ __forceinline__ float3 normalize(const float3 &a) {
	float norm = l2norm(a);
	return make_float3(a.x / norm, a.y / norm, a.z / norm);
}

__host__ __device__ __forceinline__ float4 normalize(const float4 &a) {
	float norm = l2norm(a);
	return make_float4(a.x / norm, a.y / norm, a.z / norm, a.w / norm);
}

// Explicity assumes each mass is of unit 1 mass
__device__
float3 gravityForce(float4 env) {
	return {0, -9.81f, 0};
}

__device__
float3 collisionForce(float3 pos, float4 vel, float3 force,
					float4 env) {
	if(pos.y > 0.0f) return force;
	
	float3 Fc = {0.0f, 0.0f, 0.0f}; // force due to collision
	float3 Ff = {0.0f, 0.0f, 0.0f}; // force due to friction
	float magFc, magFf;

	Fc.y = env.x * (0.0f - pos.y);

	magFc = l2norm(force);
	magFf = env.y * Fc.y;

	//Static Friction
	if(vel.x == 0.0f && vel.z == 0.0f) {
		//Check if object has acceleration
		// TODO: check within epsilon
		if(magFc != 0) {
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

	return force;
}

/*
	env: float4 describing global environment variables
		x - k		stiffness
		y - mu		coefficient of friction
		z - zeta	damping
		w - g		acceleration due to gravity
*/
__device__
float3 environmentForce(float3 pos, float4 vel, float3 force,
						float4 env) {
	force = gravityForce(env);
	force = collisionForce(pos,vel,force,env);
	return force;
}

/*
	mat: float4 describing spring material
		x - k		stiffness
		y - dL0 	maximum percent change
		z - omega	frequency of oscillation
		w - phi		phase
*/
__device__
float3 springForce(float3 bl, float3 br, float4 mat, 
					float mean_length, float time,
					float3 &force, float &magF)
{
	if(mat.x == 0.0f) {
		force = {0.0f, 0.0f, 0.0f};
		magF = 0;
		return force;
	}

	float3	dir, diff;
	// float3 b0pos, b1pos;

	float	relative_change,
			rest_length,
			L;

	// b0pos = {bl.x, bl.y, bl.z};
	// b1pos = {br.x, br.y, br.z};

	relative_change = mat.y * sinf(mat.z*time+mat.w);
	rest_length = mean_length * (1 + relative_change);

	diff.x = bl.x - br.x;
	diff.y = bl.y - br.y;
	diff.z = bl.z - br.z;

	L = l2norm(diff);
	dir = normalize(diff);

	magF = mat.x*(rest_length-L);

	force = magF * dir;
	
	return force;
}

struct SimOptions {
	float dt;
	float4 env;
	uint massesPerBlock;
	uint springsPerBlock;
	uint maxMasses;
	uint maxSprings;
	short shiftskip;
};

// TODO: test with single spring
// TODO: returns indices of highest stress 10 springs
__global__ void
integrateBodies(float4 *__restrict__ newPos, float4 *__restrict__ newVel,
				float4 *__restrict__ oldPos, float4 *__restrict__ oldVel,
				ushort2 *__restrict__ pairs, float4 *__restrict__ mats, float *__restrict__ Lbars,
				ushort *__restrict__ highStressCount, ushort *__restrict__ lowStressCount,
				float *__restrict__ stresses, uint *__restrict__ ids,
				float time, uint step, SimOptions opt)
{
	extern __shared__ float3 s[];
	float3  *s_pos = s;
	float3  *s_force = (float3*) &s_pos[opt.massesPerBlock];

	#ifdef STRESS_COUNT
	__shared__ ushort largestStressedSprings[1024];
	__shared__ ushort smallestStressedSprings[1024];
	#endif
	
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
		s_force[i] = environmentForce(s_pos[i],oldVel[i+massOffset],s_force[i],opt.env);
	}
	__syncthreads();

	float4	mat;
	float3	bl, br;
	float3	force;
	ushort2	pair;
	float	Lbar,
			magF;
	ushort	left, right;

	#ifdef STRESS_COUNT
	float	smallestStress = INFINITY,
			largestStress  = -1.0f;
	ushort	largestSpringIdx = tid,
			smallestSpringIdx = tid,
			nextSmallestSpringIdx = tid+stride,
			nextGroup_LargestSpringIdx,
			nextGroup_SmallestSpringIdx;
	#endif
	
	for(uint i = tid; i < opt.springsPerBlock && (i+springOffset) < opt.maxSprings; i+=stride) {
		pair = __ldg(&pairs[i+springOffset]);
		left  = pair.x;
		right = pair.y;
		bl = s_pos[left];
		br = s_pos[right];
		mat = __ldg(&mats[i+springOffset]);
		Lbar = __ldg(&Lbars[i+springOffset]);
		springForce(bl,br,mat,Lbar,time, force, magF);

		atomicAdd(&(s_force[left].x), force.x);
		atomicAdd(&(s_force[left].y), force.y);
		atomicAdd(&(s_force[left].z), force.z);

		atomicAdd(&(s_force[right].x), -force.x);
		atomicAdd(&(s_force[right].y), -force.y);
		atomicAdd(&(s_force[right].z), -force.z);
		
		#ifdef STRESS_COUNT
		if(abs(magF) > largestStress) {
			largestStress = magF;
			largestSpringIdx = i;
		}
		if(abs(magF) < smallestStress && abs(magF) > 0) {
			smallestStress = magF;
			nextSmallestSpringIdx = smallestSpringIdx;
			smallestSpringIdx = i;
		}
		#endif

		#ifdef FULL_STRESS
		stresses[i + springOffset] += abs(magF);
		#endif
	}

	#ifdef STRESS_COUNT
	ushort2 cLargestPair,
			cSmallestPair;
	ushort	cLargestCount,
			cSmallestCount;
	float4	cLargestMat,
			cSmallestMat;
	float	cLargestLbar,
			cSmallestLbar;
	#ifdef FULL_STRESS
	float	cLargestStress,
			cSmallestStress;
	float	cLargestID,
			cSmallestID;
	#endif
	if(step % opt.shiftskip == 0) {
		if(smallestSpringIdx == largestSpringIdx) { // guarentees not both largest and smallest
			smallestSpringIdx = nextSmallestSpringIdx;
		}

		largestStressedSprings[tid]  = largestSpringIdx;
		smallestStressedSprings[tid]  = smallestSpringIdx;

		// current thread max spring info
		cLargestPair = pairs[largestSpringIdx + springOffset];
		cLargestCount = highStressCount[largestSpringIdx + springOffset]+1;
		cLargestMat = mats[largestSpringIdx + springOffset];
		cLargestLbar = Lbars[largestSpringIdx + springOffset];
		
		cSmallestPair = pairs[smallestSpringIdx + springOffset];
		cSmallestCount = lowStressCount[smallestSpringIdx + springOffset]+1;
		cSmallestMat = mats[smallestSpringIdx + springOffset];
		cSmallestLbar = Lbars[smallestSpringIdx + springOffset];

		#ifdef FULL_STRESS
		cLargestStress = stresses[largestSpringIdx + springOffset];
		cLargestID = ids[largestSpringIdx + springOffset];
		cSmallestStress = stresses[smallestSpringIdx + springOffset];
		cSmallestID = ids[smallestSpringIdx + springOffset];
		#endif
	}
	#endif

	__syncthreads();

	#ifdef STRESS_COUNT
	int tid_next = (tid+1) % blockDim.x;
	if(step % opt.shiftskip == 0) {
		// shift current index to next spring
		nextGroup_LargestSpringIdx  = largestStressedSprings[tid_next];
		nextGroup_SmallestSpringIdx = smallestStressedSprings[tid_next];
		
		pairs[nextGroup_LargestSpringIdx + springOffset] = cLargestPair;
		mats[nextGroup_LargestSpringIdx + springOffset] = cLargestMat;
		Lbars[nextGroup_LargestSpringIdx + springOffset] = cLargestLbar;
		highStressCount[nextGroup_LargestSpringIdx + springOffset] = cLargestCount;

		pairs[nextGroup_SmallestSpringIdx + springOffset] = cSmallestPair;
		mats[nextGroup_SmallestSpringIdx + springOffset] = cSmallestMat;
		Lbars[nextGroup_SmallestSpringIdx + springOffset] = cSmallestLbar;
		lowStressCount[nextGroup_SmallestSpringIdx + springOffset] = cSmallestCount;

		#ifdef FULL_STRESS
		stresses[nextGroup_LargestSpringIdx + springOffset] = cLargestStress;
		ids[nextGroup_LargestSpringIdx + springOffset] = cLargestID;
		stresses[nextGroup_SmallestSpringIdx + springOffset] = cSmallestStress;
		ids[nextGroup_SmallestSpringIdx + springOffset] = cSmallestID;
		#endif
	}
	#endif

	// Calculate and store new mass states
	float4 vel;
	float3 pos3;
	for(uint i = tid; i < opt.massesPerBlock && (i+massOffset) < opt.maxMasses; i+=stride) {
		vel = oldVel[i+massOffset];

		vel.x += (s_force[i].x * opt.dt)*opt.env.z;
		vel.y += (s_force[i].y * opt.dt)*opt.env.z;
		vel.z += (s_force[i].z * opt.dt)*opt.env.z;

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

#endif
