#pragma once

#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "stb_image_write.h"
#include <cstdint>
#include <cassert>

struct Image
{
	size_t width;
	size_t height;
	size_t components;
	unsigned char* data;
};

Image createImage(size_t width, size_t height, size_t components)
{
	unsigned char* data = new unsigned char[width * height * components];
	return Image{ width, height, components, data };
}

void writePixel(Image* image, size_t x, size_t y, const Vec3& rgb)
{

}

void destroyImage(Image* image)
{
	delete[] image->data;
}

bool saveImage(const Image* image, const char* filePath)
{
	size_t bytesToWrite = image->width * image->components * sizeof(unsigned char);

	return stbi_write_png(
		filePath,
		image->width,
		image->height,
		image->components,
		image->data,
		bytesToWrite) == 1;
}

struct Vec3
{
public:
	Vec3() : x(0.f), y(0.f), z(0.f) {}
	Vec3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

	float x;
	float y;
	float z;

	Vec3 operator-() const 
	{ 
		return Vec3(-x, -y, -z); 
	};

	bool operator==(const Vec3& rhs)
	{
		return x == rhs.x && y == rhs.y && z == rhs.z;
	}

	Vec3& operator+=(const Vec3& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
		return *this;
	}

	Vec3& operator-=(const Vec3& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
		return *this;
	}

	Vec3& operator*=(const Vec3& rhs)
	{
		x *= rhs.x;
		y *= rhs.y;
		z *= rhs.z;
		return *this;
	}

	Vec3& operator/=(const Vec3& rhs)
	{
		x /= rhs.x;
		y /= rhs.y;
		z /= rhs.z;
		return *this;
	}

	Vec3& operator*=(const float t)
	{
		x *= t;
		y *= t;
		z *= t;
		return *this;
	}

	Vec3& operator/=(const float t)
	{
		x /= t;
		y /= t;
		z /= t;
		return *this;
	}
};

float length(const Vec3& vec) 
{
	return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

float length2(const Vec3& vec)
{
	return vec.x*vec.x + vec.y*vec.y + vec.z*vec.z;
}

inline Vec3 operator+(Vec3 lhs, const Vec3& rhs)
{
	lhs += rhs;
	return lhs;
}

inline Vec3 operator-(Vec3 lhs, const Vec3& rhs)
{
	lhs -= rhs;
	return lhs;
}

inline Vec3 operator*(Vec3 lhs, const Vec3& rhs)
{
	lhs *= rhs;
	return lhs;
}

inline Vec3 operator/(Vec3 lhs, const Vec3& rhs)
{
	lhs /= rhs;
	return lhs;
}

inline Vec3 operator*(Vec3 lhs, float rhs)
{
	lhs *= rhs;
	return lhs;
}

inline Vec3 operator/(Vec3 lhs, float rhs)
{
	lhs /= rhs;
	return lhs;
}

inline Vec3 operator*(float lhs, Vec3 rhs)
{
	rhs *= lhs;
	return rhs;
}

inline Vec3 operator/(float lhs, Vec3&rhs)
{
	rhs /= lhs;
	return rhs;
}

void normalize(Vec3& vec)
{
	float len = 1 / length(vec);
	vec *= len;
}

Vec3 normalized(const Vec3& vec)
{
	return vec / length(vec);
}

float dot(const Vec3& lhs, const Vec3& rhs)
{
	return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

Vec3 cross(const Vec3& lhs, const Vec3& rhs)
{
	return Vec3(
		lhs.y * rhs.z - lhs.z * rhs.y,
		-(lhs.x * rhs.z - lhs.z * rhs.x),
		lhs.x * rhs.y - lhs.y * rhs.x
	);
}

Vec3 reflect(const Vec3& v, const Vec3& n)
{
	return v - 2 * dot(v, n) * n;
}

int main()
{
	int width = 200;
	int height = 100;
	int components = 3;

	Image image = createImage(width, height, components);

	for (size_t pixel_y = height - 1; pixel_y >= 0; --pixel_y)
	{
		for (size_t pixel_x = 0; pixel_x < width; ++pixel_x)
		{
			Vec3 rgb;
			rgb.x = float(pixel_x) / float(width);
			rgb.y = float(pixel_y) / float(height);
			rgb.z = 0.2f;


		}
	}



	bool success = saveImage(&image, "render.png");
	assert(success);
	
	destroyImage(&image);

	return 0;
}

