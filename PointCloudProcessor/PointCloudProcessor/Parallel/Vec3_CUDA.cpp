#include <string>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

template<typename  T>
struct Vec3_CUDA {
	T x, y, z;

	__device__
	Vec3_CUDA() {}

	__device__
	Vec3_CUDA(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}

	__device__
	Vec3_CUDA operator+(Vec3_CUDA r) { return Vec3_CUDA(x + r.x, y + r.y, z + r.z); }
	__device__
	Vec3_CUDA operator-(Vec3_CUDA r) { return Vec3_CUDA(x - r.x, y - r.y, z - r.z); }
	__device__
	Vec3_CUDA operator*(Vec3_CUDA r) { return Vec3_CUDA(x * r.x, y * r.y, z * r.z); }
	__device__
	bool operator==(Vec3_CUDA r) { return x == r.x && y == r.y && z == r.z; }

	template<typename  R>
	__device__
	Vec3_CUDA operator*(R r) { return Vec3_CUDA(x * r, y * r, z * r); }
	__device__
	Vec3_CUDA operator/(Vec3_CUDA r) { return Vec3_CUDA(x / r.x, y / r.y, z / r.z); }

	template<typename  R>
	__device__
	Vec3_CUDA operator/(R r) { return Vec3_CUDA(x / r, y / r, z / r); }

	__device__
	T length()
	{
		return sqrt(pow(sqrt(pow(x, 2) + pow(y, 2)), 2) + pow(z, 2));
	}

	__device__
	static T distance(Vec3_CUDA v1, Vec3_CUDA v2)
	{
		return (v1 - v2).length();
	}

	__device__
	static Vec3_CUDA<T> cross(Vec3_CUDA<T> v1, Vec3_CUDA<T> v2)
	{
		T x = v1.y * v2.z - v1.z * v2.y;
		T y = v1.z * v2.x - v1.x * v2.z;
		T z = v1.x * v2.y - v1.y * v2.x;
		return Vec3_CUDA(x, y, z);
	}

	__device__
	static Vec3_CUDA<T> normalize(Vec3_CUDA<T> v)
	{
		T squareSum = 0;
		squareSum += pow(v.x, 2);
		squareSum += pow(v.y, 2);
		squareSum += pow(v.z, 2);
		return v / sqrt(squareSum);
	}

	__device__
	static Vec3_CUDA<T> min(Vec3_CUDA<T> v1, Vec3_CUDA<T> v2)
	{
		T x = v1.x < v2.x ? v1.x : v2.x;
		T y = v1.y < v2.y ? v1.y : v2.y;
		T z = v1.z < v2.z ? v1.z : v2.z;
		return Vec3_CUDA_CUDA(x, y, z);
	}

	__device__
	static Vec3_CUDA<T> max(Vec3_CUDA<T> v1, Vec3_CUDA<T> v2)
	{
		T x = v1.x > v2.x ? v1.x : v2.x;
		T y = v1.y > v2.y ? v1.y : v2.y;
		T z = v1.z > v2.z ? v1.z : v2.z;
		return Vec3_CUDA_CUDA(x, y, z);
	}

	__device__
	static Vec3_CUDA<T> crossProduct(Vec3_CUDA<T> a, Vec3_CUDA<T> b)
	{
		T x = a.y * b.z - a.z * b.y;
		T y = -a.x * b.z + a.z * b.x;
		T z = a.x * b.y - a.y * b.x;
		return Vec3_CUDA<T>(x, y, z);
	}

	__device__
	static void crossProduct(int v_A[], int v_B[], int c_P[])
	{
		c_P[0] = v_A[1] * v_B[2] - v_A[2] * v_B[1];
		c_P[1] = -(v_A[0] * v_B[2] - v_A[2] * v_B[0]);
		c_P[2] = v_A[0] * v_B[1] - v_A[1] * v_B[0];
	}

	__device__
	static T dot_product(Vec3_CUDA<T>& v1, Vec3_CUDA<T>& v2)
	{
		return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	}
};