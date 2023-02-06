#ifndef __SIM_KERNEL_H__
#define __SIM_KERNEL_H__

#include <math.h>
#include <algorithm>
#include <functional>
#include <fstream>
#include <iostream>

__host__ __device__ float3 operator+(const float3 &a, const float3 &b) {
	return make_float3(a.x+b.x, a.y+b.y, a.z+b.z);
}

__host__ __device__ float3 operator+=(const float3 &a, const float3 &b) {
	return make_float3(a.x+b.x, a.y+b.y, a.z+b.z);
}

__host__ __device__ float4 operator+(const float4 &a, const float4 &b) {
	return make_float4(a.x+b.x, a.y+b.y, a.z+b.z, a.w+b.w);
}

__host__ __device__ float4 operator+=(const float4 &a, const float4 &b) {
	return make_float4(a.x+b.x, a.y+b.y, a.z+b.z, a.w+b.w);
}

__host__ __device__ float3 operator-(const float3 &a, const float3 &b) {
	return make_float3(a.x-b.x, a.y-b.y, a.z-b.z);
}

__host__ __device__ float3 operator*(const float3 &a, const float3 &b) {
	return make_float3(a.x*b.x, a.y*b.y, a.z*b.z);
}

__host__ __device__ float4 operator*(const float4 &a, const float4 &b) {
	return make_float4(a.x*b.x, a.y*b.y, a.z*b.z, a.w*b.w);
}

__host__ __device__ float3 operator*(const float &a, const float3 &vec) {
	return make_float3(a*vec.x, a*vec.y, a*vec.z);
}

__host__ __device__ float3 operator*(const float3 &vec, const float &a) {
	return make_float3(a*vec.x, a*vec.y, a*vec.z);
}

__host__ __device__ float3 operator/(const float3 &vec, const float &a) {
	return make_float3(vec.x/a, vec.y/a, vec.z/a);
}

__host__ __device__ float3 operator-(const float3 &a) {
	return make_float3(-a.x, -a.y, -a.z);
}

__host__ __device__ float dot(const float3 &a, const float3 &b) {
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

__host__ __device__ float dot(const float4 &a, const float4 &b) {
	return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
}

__host__ __device__ float l2norm(const float3 &a) {
	return sqrtf(dot(a,a));
}

__host__ __device__ float l2norm(const float4 &a) {
	return sqrtf(dot(a,a));
}

__host__ __device__ float3 normalize(const float3 &a) {
	float norm = l2norm(a);
	return make_float3(a.x / norm, a.y / norm, a.z / norm);
}

__host__ __device__ float4 normalize(const float4 &a) {
	float norm = l2norm(a);
	return make_float4(a.x / norm, a.y / norm, a.z / norm, a.w / norm);
}

__device__ inline void AtomicAdd(float3& v, float3 val, int reorder)
{
	atomicAdd(&(v.x), val.x);
    atomicAdd(&(v.y), val.y);
    atomicAdd(&(v.z), val.z);
}

__device__
float3 gravityForce(float mass, float3 force, float4 env) {
	force.y += -mass*9.81;

	return force;
}

__device__
float3 collisionForce(float4 pos, float4 vel, float3 force,
					float4 env) {
	if(pos.y > 0) return force;
	
	float3 Fc = {0.0f, 0.0f, 0.0f}; // force due to collision
	float3 Ff = {0.0f, 0.0f, 0.0f}; // force due to friction
	float magFc, magFf;

	Fc.y = env.x * (0 - pos.y);

	force.y = 0;
	vel.y = 0;
	
	magFc = l2norm(force);
	magFf = env.y * force.y;

	//Static Friction
	if(vel.x == 0 && vel.z == 0) {
		//Check if object has acceleration
		// TODO: check within epsilon
		if(magFc != 0) {
			if(magFc < magFf) {
				Ff = -force;
			} else {
				//Calculate direction of force and apply friction opposite based on magnitude
				Ff = magFf*normalize(force);
			}
		}
	} else {
		// Kinetic Friction
		Ff = magFf * normalize(make_float3(-vel.x,-vel.y,-vel.z));
	}

	force.x = Fc.x + Ff.x;
	force.y = Fc.y + Ff.y;
	force.z = Fc.z + Ff.z;
	return force;
}

/*
	env: float3 describing global environment variables
		x - k		stiffness
		y - mu		coefficient of friction
		z - zeta	damping
		w - g		acceleration due to gravity
*/
__device__
float3 environmentForce(float4 pos, float4 vel, float3 force,
						float4 env) {
	force = gravityForce(pos.w, force, env);
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
float3 springForce(float4 bl, float4 br, float4 mat, 
					float mean_length, float time) {

	extern __shared__ float4 s_pos[];

	float3 force = {0.0f, 0.0f, 0.0f};

	if(mat.x == 0.0f) return force;

	float3	b0pos, b1pos,
			dir, diff;

	float	relative_change,
			rest_length,
			L, magF;

	b0pos = {bl.x, bl.y, bl.z};
	b1pos = {br.x, br.y, br.z};

	relative_change = mat.y * sinf(mat.z*time+mat.w);
	rest_length = mean_length * (1 + relative_change);


	diff.x = b0pos.x - b1pos.x;
	diff.y = b0pos.y - b1pos.y;
	diff.z = b0pos.z - b1pos.z;

	L = l2norm(diff);
	dir = normalize(diff);

	magF = mat.x*(rest_length-L);

	force = magF * dir;
	
	return force;
}

__global__ void
integrateBodies(float4* newPos, float4* newVel,
				float4* oldPos, float4* oldVel,
				uint2* pairs, float4* mats, float* Lbars,
				float dt, float time, float4 env,
				uint numMasses, uint numSprings,
				uint maxMasses, uint maxSprings, uint* springCount)
{
	extern __shared__ float4 s[];
	float4  *s_pos = s;
	float3  *s_force = (float3*) &s_pos[numMasses];

	uint massOffset   = blockIdx.x * numMasses;
	uint springOffset = blockIdx.x * numSprings;

	int idx    = threadIdx.x;
	int stride = blockDim.x;

	// Initialize and compute environment forces
	for(int i = idx; i < numMasses && (i+massOffset) < maxMasses; i+=stride) {
		s_pos[i] = oldPos[i+massOffset];
	}
	__syncthreads();
	
	for(int i = idx; i < numMasses && (i+massOffset) < maxMasses; i+=stride) {
		s_force[i] = environmentForce(s_pos[i],oldVel[i+massOffset],s_force[i],env);
	}
	__syncthreads();


	float4 bl, br;
	float3 force;
	uint left, right;

	for(int i = idx; i < numSprings && (i+springOffset) < maxSprings; i+=stride) {
		left  = pairs[i+springOffset].x-massOffset;
		right = pairs[i+springOffset].y-massOffset;
		bl = s_pos[left];
		br = s_pos[right];

		force = springForce(bl,br,mats[i+springOffset],Lbars[i+springOffset],time);
		AtomicAdd(s_force[left],   force, 1);
		AtomicAdd(s_force[right], -force, 1);

		atomicAdd(springCount,1);
	}
	__syncthreads();

	for(int i = idx; i < numMasses && (i+massOffset) < maxMasses; i+=stride) {
		float4 vel = oldVel[i+massOffset];

		vel.x += s_force[i].x * dt;
		vel.y += s_force[i].y * dt;
		vel.z += s_force[i].z * dt;
		

		vel.x *= env.z;
		vel.y *= env.z;
		vel.z *= env.z;
		// printf("Vel: {%f,%f,%f}\n",vel.x, vel.y, vel.z);

		// new position = old position + velocity * deltaTime
		s_pos[i].x += vel.x * dt;
		s_pos[i].y += vel.y * dt;
		s_pos[i].z += vel.z * dt;


		// printf("Loc: {%f,%f,%f}\n",s_pos[i].x, s_pos[i].y, s_pos[i].z);

		// store new position and velocity
		newPos[i+massOffset] = s_pos[i];
		newVel[i+massOffset] = vel;
	}
}

#endif
