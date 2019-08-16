#pragma once

template<typename T>
struct vec3T
{
public:
	T x;
	T y;
	T z;

	bool operator==(const vec3T& rhs)
	{
		return x == rhs.x && y == rhs.y && z == rhs.z;
	}

	vec3T& operator+=(const vec3T& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
		return *this;
	}

	vec3T& operator-=(const vec3T& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
		return *this;
	}

	vec3T& operator*=(const vec3T& rhs)
	{
		x *= rhs.x;
		y *= rhs.y;
		z *= rhs.z;
		return *this;
	}

	vec3T& operator/=(const vec3T& rhs)
	{
		x /= rhs.x;
		y /= rhs.y;
		z /= rhs.z;
		return *this;
	}

	vec3T& operator+=(const T t)
	{
		x += t;
		y += t;
		z += t;
		return *this;
	}

	vec3T& operator*=(const T t)
	{
		x *= t;
		y *= t;
		z *= t;
		return *this;
	}

	vec3T& operator/=(const T t)
	{
		x /= t;
		y /= t;
		z /= t;
		return *this;
	}
};

template<typename T>
vec3T<T> operator-(const vec3T<T>& vec)
{
	return vec3T<T>{-vec.x, -vec.y, -vec.z};
};

template<typename T>
T length(const vec3T<T>& vec)
{
	return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

template<typename T>
T length2(const vec3T<T>& vec)
{
	return vec.x*vec.x + vec.y*vec.y + vec.z*vec.z;
}

template<typename T>
inline vec3T<T> operator+(vec3T<T> lhs, const vec3T<T>& rhs)
{
	lhs += rhs;
	return lhs;
}

template<typename T>
inline vec3T<T> operator-(vec3T<T> lhs, const vec3T<T>& rhs)
{
	lhs -= rhs;
	return lhs;
}

template<typename T>
inline vec3T<T> operator*(vec3T<T> lhs, const vec3T<T>& rhs)
{
	lhs *= rhs;
	return lhs;
}

template<typename T>
inline vec3T<T> operator/(vec3T<T> lhs, const vec3T<T>& rhs)
{
	lhs /= rhs;
	return lhs;
}

template<typename T>
inline vec3T<T> operator*(vec3T<T> lhs, T rhs)
{
	lhs *= rhs;
	return lhs;
}

template<typename T>
inline vec3T<T> operator/(vec3T<T> lhs, T rhs)
{
	lhs /= rhs;
	return lhs;
}

template<typename T>
inline vec3T<T> operator+(vec3T<T> lhs, T rhs)
{
	lhs += rhs;
	return lhs;
}

template<typename T>
inline vec3T<T> operator*(T lhs, vec3T<T> rhs)
{
	rhs *= lhs;
	return rhs;
}

template<typename T>
void normalize(vec3T<T>& vec)
{
	T len = 1 / length(vec);
	vec *= len;
}

template<typename T>
vec3T<T> normalized(const vec3T<T>& vec)
{
	return vec / length(vec);
}

template<typename T>
T dot(const vec3T<T>& lhs, const vec3T<T>& rhs)
{
	return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

template<typename T>
vec3T<T> cross(const vec3T<T>& lhs, const vec3T<T>& rhs)
{
	return vec3T<T>{
		lhs.y * rhs.z - lhs.z * rhs.y,
		-(lhs.x * rhs.z - lhs.z * rhs.x),
		lhs.x * rhs.y - lhs.y * rhs.x
		};
}

template<typename T>
vec3T<T> reflect(const vec3T<T>& v, const vec3T<T>& n)
{
	return v - 2.0f * dot(v, n) * n;
}

// Returns refracted vector if refraction occurs, else 0-vector
template<typename T>
bool refract(const vec3T<T>& v, const vec3T<T>& n, float ni_over_nt, vec3T<T>* outRefracted)
{
	vec3T<T> unitV = normalized(v);

	T dotVN = dot(unitV, n);
	T discriminant = 1.0f - ni_over_nt * ni_over_nt * (1.0f - dotVN * dotVN);

	if (discriminant > 0.0f)
	{
		*outRefracted = ni_over_nt * (unitV - n * dotVN) - n * sqrtf(discriminant);
		return true;
	}
	else
	{
		return false;
	}
}


typedef vec3T<float> vec3;
