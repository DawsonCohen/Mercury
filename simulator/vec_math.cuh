#ifndef __VECMATH_H__
#define __VECMATH_H__

__device__ __forceinline__ float3 operator+(const float3 &a, const float3 &b) {
	return {
		__fadd_rn(a.x,b.x),
		__fadd_rn(a.y,b.y),
		__fadd_rn(a.z,b.z)};
}

__device__ __forceinline__ float3 operator+=(const float3 &a, const float3 &b) {
	return {
		__fadd_rn(a.x,b.x),
		__fadd_rn(a.y,b.y),
		__fadd_rn(a.z,b.z) };
}

__device__ __forceinline__ float4 operator+(const float4 &a, const float4 &b) {
	return {
		__fadd_rn(a.x,b.x),
		__fadd_rn(a.y,b.y),
		__fadd_rn(a.z,b.z),
		__fadd_rn(a.w,b.w) };
}

__device__ __forceinline__ float4 operator+=(const float4 &a, const float4 &b) {
	return {
		__fadd_rn(a.x,b.x),
		__fadd_rn(a.y,b.y),
		__fadd_rn(a.z,b.z),
		__fadd_rn(a.w,b.w) };
}

__device__ __forceinline__ float3 operator-(const float3 &a, const float3 &b) {
	return {
		__fsub_rn(a.x,b.x),
		__fsub_rn(a.y,b.y),
		__fsub_rn(a.z,b.z) };
}

__device__ __forceinline__ float3 operator*(const float3 &a, const float3 &b) {
	return {
		__fmul_rn(a.x,b.x),
		__fmul_rn(a.y,b.y),
		__fmul_rn(a.z,b.z) };
}

__device__ __forceinline__ float4 operator*(const float4 &a, const float4 &b) {
	return {
		__fmul_rn(a.x, b.x),
		__fmul_rn(a.y, b.y),
		__fmul_rn(a.z, b.z),
		__fmul_rn(a.w, b.w) };
}

__device__ __forceinline__ float3 operator*(const float &a, const float3 &vec) {
	return {
		__fmul_rn(a, vec.x),
		__fmul_rn(a, vec.y),
		__fmul_rn(a, vec.z) };
}

__device__ __forceinline__ float3 operator*(const float3 &vec, const float &a) {
	return {
		__fmul_rn(a,vec.x),
		__fmul_rn(a,vec.y),
		__fmul_rn(a,vec.z)
	};
}

__device__ __forceinline__ float3 operator/(const float3 &vec, const float &a) {
	return {
        __fdiv_rn(vec.x,a),
        __fdiv_rn(vec.y,a),
        __fdiv_rn(vec.z,a)};
}

__device__ __forceinline__ float3 operator-(const float3 &a) {
	return {-a.x, -a.y, -a.z};
}

__device__ __forceinline__ float dot(const float3 &a, const float3 &b) {
	return {
		__fmul_rn(a.x, b.x)
		+ __fmul_rn(a.y, b.y)
		+ __fmul_rn(a.z, b.z)
	};
}

__device__ __forceinline__ float dot(const float4 &a, const float4 &b) {
	return (
		__fmul_rn(a.x,b.x)
		+ __fmul_rn(a.y,b.y)
		+ __fmul_rn(a.z,b.z)
		+ __fmul_rn(a.w,b.w)
	);
}

__device__ __forceinline__ float l2norm(const float3 &a) {
	return __fsqrt_rn(dot(a,a));
}

__device__ __forceinline__ float l2norm(const float4 &a) {
	return __fsqrt_rn(dot(a,a));
}

__device__ __forceinline__ float3 normalize(const float3 &a) {
	float norm = l2norm(a);
	return { 
        __fdiv_rn(a.x, norm),
        __fdiv_rn(a.y, norm),
        __fdiv_rn(a.z, norm) };
}

__device__ __forceinline__ float4 normalize(const float4 &a) {
	float norm = l2norm(a);
	return {
        __fdiv_rn(a.x, norm),
        __fdiv_rn(a.y, norm),
        __fdiv_rn(a.z, norm),
        __fdiv_rn(a.w, norm) };
}

#endif