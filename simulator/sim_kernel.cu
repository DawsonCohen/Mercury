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

// TODO: test with single spring
// TODO: returns indices of highest stress 10 springs
__global__ void
integrateBodies(float4* newPos, float4* newVel,
				float4* oldPos, float4* oldVel,
				ushort* highStressCount, ushort* lowStressCount,
				ushort2* pairs, float4* mats, float* Lbars,
				float dt, float time, float4 env, uint step,
				uint numMasses, uint numSprings,
				uint maxMasses, uint maxSprings)
{
	extern __shared__ float3 s[];
	float3  *s_pos = s;
	float3  *s_force = (float3*) &s_pos[numMasses];

	__shared__ ushort largestStressedSprings[1024];
	__shared__ ushort smallestStressedSprings[1024];
	
	uint massOffset   = blockIdx.x * numMasses;
	uint springOffset = blockIdx.x * numSprings;

	int tid    = threadIdx.x;
	int tid_next = (tid+1) % blockDim.x;
	int stride = blockDim.x;
	
	// Initialize and compute environment forces
	float4 pos4;
	for(uint i = tid; i < numMasses && (i+massOffset) < maxMasses; i+=stride) {
		pos4 = oldPos[i+massOffset];
		s_pos[i] = {pos4.x,pos4.y,pos4.z};
	}
	
	for(uint i = tid; i < numMasses && (i+massOffset) < maxMasses; i+=stride) {
		s_force[i] = environmentForce(s_pos[i],oldVel[i+massOffset],s_force[i],env);
	}
	__syncthreads();

	float4	mat;
	float3	bl, br;
	float3	force;
	ushort2	pair;
	float	Lbar,
			magF,
	    	smallestStress = INFINITY,
			largestStress  = -1.0f;
	ushort	left, right;

	ushort	largestSpringIdx = tid,
			smallestSpringIdx = tid,
			nextSmallestSpringIdx = tid+stride,
			nextGroup_LargestSpringIdx,
			nextGroup_SmallestSpringIdx;
	
	uint n = tid % 3;
	for(uint i = tid; i < numSprings && (i+springOffset) < maxSprings; i+=stride) {
		pair = __ldg(&pairs[i+springOffset]);
		left  = pair.x;
		right = pair.y;
		bl = s_pos[left];
		br = s_pos[right];
		mat = __ldg(&mats[i+springOffset]);
		Lbar = __ldg(&Lbars[i+springOffset]);
		springForce(bl,br,mat,Lbar,time, force, magF);

		// Add force to each connected mass
		// Modulo staggers memory access to increase cache misses
		atomicAdd(&(s_force[left].x) + n, *(&force.x+n));
		atomicAdd(&(s_force[left].x) + (n+1)%3, *(&force.x + (n+1)%3));
		atomicAdd(&(s_force[left].x) + (n+2)%3, *(&force.x + (n+2)%3));

		atomicAdd(&(s_force[right].x) + n, -*(&force.x+n));
		atomicAdd(&(s_force[right].x) + (n+1)%3, -*(&force.x + (n+1)%3));
		atomicAdd(&(s_force[right].x) + (n+2)%3, -*(&force.x + (n+2)%3));
		
		if(abs(magF) > largestStress) {
			largestStress = magF;
			largestSpringIdx = i;
		}
		if(abs(magF) < smallestStress) { // guarentees not both largest and smallest
			smallestStress = magF;
			nextSmallestSpringIdx = smallestSpringIdx;
			smallestSpringIdx = i;
		}
	}

	if(smallestSpringIdx == largestSpringIdx) {
		smallestSpringIdx = nextSmallestSpringIdx;
	}

	largestStressedSprings[tid]  = largestSpringIdx;
	smallestStressedSprings[tid]  = smallestSpringIdx;

	// current thread max spring info
	ushort2 cLargestPair = pairs[largestSpringIdx + springOffset];
	ushort cLargestCount = highStressCount[largestSpringIdx + springOffset]+1;
	float4 cLargestMat = mats[largestSpringIdx + springOffset];
	float cLargestLbar = Lbars[largestSpringIdx + springOffset];
	
	ushort2 cSmallestPair = pairs[smallestSpringIdx + springOffset];
	ushort cSmallestCount = lowStressCount[smallestSpringIdx + springOffset]+1;
	float4 cSmallestMat = mats[smallestSpringIdx + springOffset];
	float cSmallestLbar = Lbars[smallestSpringIdx + springOffset];
	__syncthreads();

	nextGroup_LargestSpringIdx  = largestStressedSprings[tid_next];
	nextGroup_SmallestSpringIdx = smallestStressedSprings[tid_next];
	
	// shift next spring to current index
	pairs[nextGroup_LargestSpringIdx + springOffset] = cLargestPair;
	mats[nextGroup_LargestSpringIdx + springOffset] = cLargestMat;
	Lbars[nextGroup_LargestSpringIdx + springOffset] = cLargestLbar;
	highStressCount[nextGroup_LargestSpringIdx + springOffset] = cLargestCount;

	pairs[nextGroup_SmallestSpringIdx + springOffset] = cSmallestPair;
	mats[nextGroup_SmallestSpringIdx + springOffset] = cSmallestMat;
	Lbars[nextGroup_SmallestSpringIdx + springOffset] = cSmallestLbar;
	lowStressCount[nextGroup_SmallestSpringIdx + springOffset] = cSmallestCount;

	// Calculate and store new mass states
	float4 vel;
	float3 pos3;
	for(uint i = tid; i < numMasses && (i+massOffset) < maxMasses; i+=stride) {
		vel = oldVel[i+massOffset];

		vel.x += (s_force[i].x * dt)*env.z;
		vel.y += (s_force[i].y * dt)*env.z;
		vel.z += (s_force[i].z * dt)*env.z;

		// new position = old position + velocity * deltaTime
		s_pos[i].x += vel.x * dt;
		s_pos[i].y += vel.y * dt;
		s_pos[i].z += vel.z * dt;

		// store new position and velocity
		pos3 = s_pos[i];
		newPos[i+massOffset] = {pos3.x, pos3.y, pos3.z};
		newVel[i+massOffset] = vel;
	}
}

#endif
