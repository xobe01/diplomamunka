#include <string>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

template<typename  T>
struct Vec3 {
	T x, y, z;

	__host__ __device__ Vec3() {}

	__host__ __device__ Vec3(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}

	__host__ __device__ Vec3 operator+(Vec3 r) { return Vec3(x + r.x, y + r.y, z + r.z); }
	__host__ __device__ Vec3 operator-(Vec3 r) { return Vec3(x - r.x, y - r.y, z - r.z); }
	__host__ __device__ Vec3 operator*(Vec3 r) { return Vec3(x * r.x, y * r.y, z * r.z); }

	__host__ __device__ bool operator==(Vec3 r) { return x == r.x && y == r.y && z == r.z; }

	template<typename  R>
	__host__ __device__ Vec3 operator*(R r) { return Vec3(x * r, y * r, z * r); }

	__host__ __device__ Vec3 operator/(Vec3 r) { return Vec3(x / r.x, y / r.y, z / r.z); }

	template<typename  R>
	__host__ __device__ Vec3 operator/(R r) { return Vec3(x / r, y / r, z / r); }

	__host__ __device__ T length()
	{
		return sqrt(pow(sqrt(pow(x, 2) + pow(y, 2)), 2) + pow(z, 2));
	}

	__host__ __device__ static T distance(Vec3 v1, Vec3 v2)
	{
		return (v1 - v2).length();
	}

	__host__ __device__ static Vec3<T> cross(Vec3<T> v1, Vec3<T> v2)
	{
		T x = v1.y * v2.z - v1.z * v2.y;
		T y = v1.z * v2.x - v1.x * v2.z;
		T z = v1.x * v2.y - v1.y * v2.x;
		return Vec3(x, y, z);
	}

	__host__ __device__ static Vec3<T> normalize(Vec3<T> v)
	{
		T squareSum = 0;
		squareSum += pow(v.x, 2);
		squareSum += pow(v.y, 2);
		squareSum += pow(v.z, 2);
		return v / sqrt(squareSum);
	}

	__host__ __device__ static Vec3<T> min(Vec3<T> v1, Vec3<T> v2)
	{
		T x = v1.x < v2.x ? v1.x : v2.x;
		T y = v1.y < v2.y ? v1.y : v2.y;
		T z = v1.z < v2.z ? v1.z : v2.z;
		return Vec3(x, y, z);
	}

	__host__ __device__ static Vec3<T> max(Vec3<T> v1, Vec3<T> v2)
	{
		T x = v1.x > v2.x ? v1.x : v2.x;
		T y = v1.y > v2.y ? v1.y : v2.y;
		T z = v1.z > v2.z ? v1.z : v2.z;
		return Vec3(x, y, z);
	}

	__host__ __device__ std::string to_string()
	{
		std::string s("");
		s += std::to_string(x) + ";" + std::to_string(y) + ";" + std::to_string(z);
		return s;
	}

	__host__ __device__ static Vec3<T> crossProduct(Vec3<T> a, Vec3<T> b)
	{
		T x = a.y * b.z - a.z * b.y;
		T y = -a.x * b.z + a.z * b.x;
		T z = a.x * b.y - a.y * b.x;
		return Vec3<T>(x, y, z);
	}

	__host__ __device__ static void crossProduct(int v_A[], int v_B[], int c_P[])
	{
		c_P[0] = v_A[1] * v_B[2] - v_A[2] * v_B[1];
		c_P[1] = -(v_A[0] * v_B[2] - v_A[2] * v_B[0]);
		c_P[2] = v_A[0] * v_B[1] - v_A[1] * v_B[0];
	}

	__host__ __device__ static T dot_product(Vec3<T>& v1, Vec3<T>& v2)
	{
		return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	}
};