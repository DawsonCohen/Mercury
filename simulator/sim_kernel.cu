#ifndef __SIM_KERNEL_H__
#define __SIM_KERNEL_H__

#include "vec_math.cuh"
#include <math.h>

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
		magF = 0.0f;
		return force;
	}

	float3	dir, diff;
	// float3 b0pos, b1pos;

	float	relative_change,
			rest_length,
			L;

	// b0pos = {bl.x, bl.y, bl.z};
	// b1pos = {br.x, br.y, br.z};

	// rest_length = __fmaf_rn(mean_length*mat.y, sinf(__fmaf(mat.z,time,mat.w)), mean_length);

	relative_change = mat.y * sinf(mat.z*time+mat.w);
	// rest_length = __fmaf_rn(relative_change,mean_length,mean_length);
	rest_length = mean_length * (1 + relative_change);

	diff.x = bl.x - br.x;
	diff.y = bl.y - br.y;
	diff.z = bl.z - br.z;

	L = l2norm(diff);
	// L = norm3df(diff.x,diff.y,diff.z);
	dir = {
		__fdiv_rn(diff.x, L),
        __fdiv_rn(diff.y, L),
        __fdiv_rn(diff.z, L) };
	// dir = normalize(diff);

	// magF = mat.x*(rest_length-L);
	magF = __fmaf_rn(mat.x / 1000,rest_length,-mat.x*L / 1000);

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

__global__ void
integrateBodies(
			float4 *__restrict__ newPos, float4 *__restrict__ newVel,
			float4 *__restrict__ oldPos, float4 *__restrict__ oldVel,
			ushort2 *__restrict__ lpairs, float4 *__restrict__ lmats, float *__restrict__ lLbars,
			ushort2 *__restrict__ rpairs, float4 *__restrict__ rmats, float *__restrict__ rLbars,
			ushort2 *__restrict__ lCounts, ushort2 *__restrict__ rCounts,
			float time, uint step, SimOptions opt)
{
	extern __shared__ float3 s[];
	float3  *s_pos = s;
	float3  *s_force = (float3*) &s_pos[opt.massesPerBlock];
	
	uint massOffset   = blockIdx.x * opt.massesPerBlock;
	uint springOffset = blockIdx.x * opt.springsPerBlock;

	uint tid    = threadIdx.x;
	uint stride = blockDim.x;
	
	// Initialize and compute environment forces
	float4 pos4;
	for(uint i = tid; i < opt.massesPerBlock && (i+massOffset) < opt.maxMasses; i+=stride) {
		pos4 = oldPos[i+massOffset];
		s_pos[i] = {pos4.x,pos4.y,pos4.z};
	}
	
	for(uint i = tid; i < opt.massesPerBlock && (i+massOffset) < opt.maxMasses; i+=stride) {
	}
	__syncthreads();

	float4	mat;
	float3	bl, br;
	float3	tforce, force;
	ushort2	pair;
	float	Lbar,
			magF;
	ushort	left, right;
	
	uint i, j;
	ushort	lspring = 0,
			rspring = 0;
	for(i = tid; i < opt.massesPerBlock && (i+massOffset) < opt.maxMasses; i+=stride) {
		tforce = environmentForce(s_pos[i],oldVel[i+massOffset],s_force[i],opt.env);
		lspring = lCounts[i].y;
		rspring = rCounts[i].y;

		// printf("%u--%u\n",lCounts[i].y,rCounts[i].y);
		for(j = 0; j < lCounts[i].x; j++) {
			pair = __ldg(&lpairs[lspring+springOffset]);
			left  = pair.x;
			right = pair.y;
			bl = s_pos[left];
			br = s_pos[right];
			mat = __ldg(&lmats[lspring+springOffset]);
			Lbar = __ldg(&lLbars[lspring+springOffset]);
			springForce(bl,br,mat,Lbar,time, force, magF);
			tforce.x += force.x;
			tforce.y += force.y;
			tforce.z += force.z;
			// printf("{%f,%f,%f}\n",lforce.x,lforce.y,lforce.z);

			lspring++;
		}
		// for(j = 0; j < rCounts[i].x; j++) {
		// 	// pair = __ldg(&rpairs[rspring+springOffset]);
		// 	left  = pair.x;
		// 	right = pair.y;
		// 	bl = s_pos[left];
		// 	br = s_pos[right];
		// 	mat = __ldg(&rmats[rspring+springOffset]);
		// 	Lbar = __ldg(&rLbars[rspring+springOffset]);
		// 	springForce(bl,br,mat,Lbar,time, force, magF);

		// 	tforce.x += force.x;
		// 	tforce.y += force.y;
		// 	tforce.z += force.z;
		// 	// printf("{%f,%f,%f}\n",rforce.x,rforce.y,rforce.z);

		// 	rspring++;
		// }
		// printf("{%f,%f,%f}-{%f,%f,%f}\n",lforce.x,lforce.y, lforce.z,rforce.x,rforce.y,rforce.z);
		s_force[i] = tforce;
	}
	__syncthreads();
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