#pragma once

#include <iostream>
#include <cmath>

class vec3
{
public:
	vec3() {}
	vec3( float _x, float _y, float _z ) : x( _x ), y( _y ), z( _z ) {}

	// Public Member Functions
	float x;
	float y;
	float z;
	
	vec3 operator-() const { return vec3( -x, -y, -z ); };

	bool operator==( const vec3& rhs )
	{
		return x == rhs.x && y == rhs.y && z == rhs.z;
	}
	
	vec3& operator+=( const vec3& rhs ) 
	{ 
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
		return *this;
	}
	
	vec3& operator-=( const vec3& rhs )
	{
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
		return *this;
	}
	
	vec3& operator*=( const vec3& rhs )
	{
		x *= rhs.x;
		y *= rhs.y;
		z *= rhs.z;
		return *this;
	}
	
	vec3& operator/=( const vec3& rhs )
	{
		x /= rhs.x;
		y /= rhs.y;
		z /= rhs.z;
		return *this;
	}

	vec3& operator*=( const float t )
	{
		x *= t;
		y *= t;
		z *= t;
		return *this;
	}

	vec3& operator/=( const float t )
	{
		x /= t;
		y /= t;
		z /= t;
		return *this;
	}

	float length() const
	{
		return sqrt( x*x + y*y + z*z );
	}

	float length2() const
	{
		return x*x + y*y + z*z;
	}

	void normalize()
	{
		float len = 1 / length();
		x *= len; y *= len; z *= len;
	}

	float dot( const vec3& rhs ) const
	{
		return x * rhs.x + y * rhs.y + z * rhs.z;
	}

	vec3 cross( const vec3 rhs ) const
	{
		return vec3( 
			y * rhs.z - z * rhs.y,
			-( x * rhs.z - z * rhs.x ),
			x * rhs.y - y * rhs.x
		 );
	}

	vec3 unitVector() const;
};

inline std::ostream& operator<<( std::ostream& os, const vec3& vec )
{
	os << vec.x << " " << vec.y << " " << vec.z;
	return os;
}

inline vec3 operator+( const vec3& lhs, const vec3& rhs )
{
	return vec3( lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z );
}

inline vec3 operator-( const vec3& lhs, const vec3& rhs )
{
	return vec3( lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z );
}

inline vec3 operator*( const vec3& lhs, const vec3& rhs )
{
	return vec3( lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z );
}

inline vec3 operator/( const vec3& lhs, const vec3& rhs )
{
	return vec3( lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z );
}

inline vec3 operator*( const vec3& lhs, float rhs )
{
	return vec3( lhs.x * rhs, lhs.y * rhs, lhs.z * rhs );
}

inline vec3 operator/( const vec3& lhs, float rhs )
{
	return vec3( lhs.x / rhs, lhs.y / rhs, lhs.z / rhs );
}

inline vec3 operator*( float lhs, const vec3& rhs )
{
	return vec3( lhs * rhs.x, lhs * rhs.y, lhs * rhs.z );
}

inline vec3 operator/( float lhs, const vec3& rhs )
{
	return vec3( lhs / rhs.x, lhs / rhs.y, lhs / rhs.z );
}

inline vec3 reflect(const vec3& v, const vec3& n)
{
	return v - 2 * v.dot(n) * n;
}

inline vec3 vec3::unitVector() const 
{
	return *this / length();
}
