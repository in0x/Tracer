#pragma once

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_ERROR 0

#include "stb_image_write.h"
#include <cstdint>
#include <cassert>

template<typename T>
struct Vec3
{
public:
	Vec3() : x(0.f), y(0.f), z(0.f) {}
	Vec3(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}

	T x;
	T y;
	T z;

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

	Vec3& operator*=(const T t)
	{
		x *= t;
		y *= t;
		z *= t;
		return *this;
	}

	Vec3& operator/=(const T t)
	{
		x /= t;
		y /= t;
		z /= t;
		return *this;
	}
};

template<typename T>
T length(const Vec3<T>& vec)
{
	return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

template<typename T>
T length2(const Vec3<T>& vec)
{
	return vec.x*vec.x + vec.y*vec.y + vec.z*vec.z;
}

template<typename T>
inline Vec3<T> operator+(Vec3<T> lhs, const Vec3<T>& rhs)
{
	lhs += rhs;
	return lhs;
}

template<typename T>
inline Vec3<T> operator-(Vec3<T> lhs, const Vec3<T>& rhs)
{
	lhs -= rhs;
	return lhs;
}

template<typename T>
inline Vec3<T> operator*(Vec3<T> lhs, const Vec3<T>& rhs)
{
	lhs *= rhs;
	return lhs;
}

template<typename T>
inline Vec3<T> operator/(Vec3<T> lhs, const Vec3<T>& rhs)
{
	lhs /= rhs;
	return lhs;
}

template<typename T>
inline Vec3<T> operator*(Vec3<T> lhs, T rhs)
{
	lhs *= rhs;
	return lhs;
}

template<typename T>
inline Vec3<T> operator/(Vec3<T> lhs, T rhs)
{
	lhs /= rhs;
	return lhs;
}

template<typename T>
inline Vec3<T> operator*(T lhs, Vec3<T> rhs)
{
	rhs *= lhs;
	return rhs;
}

template<typename T>
inline Vec3<T> operator/(T lhs, Vec3<T>&rhs)
{
	rhs /= lhs;
	return rhs;
}

template<typename T>
void normalize(Vec3<T>& vec)
{
	T len = 1 / length(vec);
	vec *= len;
}

template<typename T>
Vec3<T> normalized(const Vec3<T>& vec)
{
	return vec / length(vec);
}

template<typename T>
T dot(const Vec3<T>& lhs, const Vec3<T>& rhs)
{
	return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

template<typename T>
Vec3<T> cross(const Vec3<T>& lhs, const Vec3<T>& rhs)
{
	return Vec3<T>(
		lhs.y * rhs.z - lhs.z * rhs.y,
		-(lhs.x * rhs.z - lhs.z * rhs.x),
		lhs.x * rhs.y - lhs.y * rhs.x
	);
}

template<typename T>
Vec3<T> reflect(const Vec3<T>& v, const Vec3<T>& n)
{
	return v - 2 * dot(v, n) * n;
}

typedef Vec3<float> Vec3f;

struct Image
{
	size_t width;
	size_t height;
	size_t components;
	uint8_t* data;
};

Image createImage(size_t width, size_t height, size_t components)
{
	assert(components == 3);
	uint8_t* data = new uint8_t[width * height * components];
	return Image{ width, height, components, data };
}

void writePixel(Image* image, size_t x, size_t y, uint8_t r, uint8_t g, uint8_t b)
{
	size_t bytesToWrite = sizeof(int8_t) * 3;
	size_t start = x * image->components + y * image->width * image->components;
	
	uint8_t pixel[3] = {r, g, b};
	
	memcpy(reinterpret_cast<char*>(image->data) + start, pixel, bytesToWrite);
}

void destroyImage(Image* image)
{
	delete[] image->data;
}

bool saveImage(const Image* image, const char* filePath)
{
	return stbi_write_png(
		filePath,
		image->width,
		image->height,
		image->components,
		image->data,
		image->width * image->components) != STB_IMAGE_WRITE_ERROR;
}

int main()
{
	int width = 200;
	int height = 100;
	int components = 3;

	Image image = createImage(width, height, components);

	for (int pixel_y = 0; pixel_y < height; ++pixel_y)
	{
		for (int pixel_x = 0; pixel_x < width; ++pixel_x)
		{
			Vec3f rgb;
			rgb.x = float(pixel_x) / float(width);
			rgb.y = float(pixel_y) / float(height);
			rgb.z = 0.2f;
			rgb *= 255.99;

			writePixel(&image, pixel_x, pixel_y, rgb.x, rgb.y, rgb.z);
		}
	}

	bool success = saveImage(&image, "render.png");
	assert(success);
	
	destroyImage(&image);

	return 0;
}

