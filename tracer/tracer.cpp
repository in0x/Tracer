#pragma once

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_ERROR 0

#include "stb_image_write.h"
#include "Vec3.h"
#include <cstdint>
#include <cassert>
#include <vector>

template<typename T>
T lerp(float t, T a, T b)
{
	return (1.0f - t) * a + t * b;
}

struct Ray
{
	Ray(const Vec3f& origin, const Vec3f direction)
		: m_origin(origin)
		, m_direction(direction) {}

	Vec3f m_origin;
	Vec3f m_direction;
};

Vec3f pointAt(const Ray& ray, float t)
{
	return ray.m_origin + ray.m_direction * t;
}

struct Image
{
	Image(size_t width, size_t height, size_t components)
		: m_width(width)
		, m_height(height)
		, m_components(components)
	{
		assert(components == 3);
		m_data = new uint8_t[width * height * components];
	}

	~Image()
	{
		delete[] m_data;
	}

	void writePixel(size_t x, size_t y, uint8_t r, uint8_t g, uint8_t b)
	{
		size_t bytesToWrite = sizeof(int8_t) * 3;
		size_t start = x * m_components + y * m_width * m_components;

		uint8_t pixel[3] = { r, g, b };

		memcpy(reinterpret_cast<char*>(m_data) + start, pixel, bytesToWrite);
	}

	bool save(const char* filePath) const
	{
		return stbi_write_png(
			filePath,
			m_width,
			m_height,
			m_components,
			m_data,
			m_width * m_components) != STB_IMAGE_WRITE_ERROR;
	}

	size_t m_width;
	size_t m_height;
	size_t m_components;
	uint8_t* m_data;
};

/*
	+y
	|
	| /-z
	|/______ +x
	/
   /+z

   Camera uses right handed coordinate system.
*/

struct Camera
{
	Camera(Vec3f rayOrigin, Vec3f lowerLeft, Vec3f horizontal, Vec3f vertical)
		: m_rayOrigin(rayOrigin)
		, m_lowerLeft(lowerLeft)
		, m_horizontal(horizontal)
		, m_vertical(vertical)
	{}

	Vec3f m_rayOrigin;
	Vec3f m_lowerLeft;
	Vec3f m_horizontal;
	Vec3f m_vertical;
};

Ray getRayThroughPixel(const Camera& camera, int pixelX, int pixelY, int widthPixels, int heightPixels)
{
	float u = (float)pixelX / float(widthPixels);
	float v = (float)pixelY / float(heightPixels);

	return Ray(camera.m_rayOrigin, camera.m_lowerLeft + u * camera.m_horizontal + v * camera.m_vertical);
}

struct Sphere
{
	Sphere(const Vec3f& position, float radius)
		: m_position(position)
		, m_radius(radius)
	{}

	Vec3f m_position;
	float m_radius;
};

struct Intersection
{
	float m_tAt;
	Vec3f m_point;
	Vec3f m_normal;
};

Intersection intersectSphere(const Sphere& sphere, const Ray& ray)
{
	Vec3f toCenter = ray.m_origin - sphere.m_position;

	float a = dot(ray.m_direction, ray.m_direction);
	float b = dot(toCenter, ray.m_direction);
	float c = dot(toCenter, toCenter) - sphere.m_radius * sphere.m_radius;

	float discriminant = b * b - a * c;

	Intersection intersect;

	if (discriminant > 0.0f)
	{
		intersect.m_tAt = (-b - sqrt(discriminant)) / a;
		intersect.m_point = pointAt(ray, intersect.m_tAt);
		intersect.m_normal = (intersect.m_point - sphere.m_position) / sphere.m_radius;
	}
	else
	{
		intersect.m_tAt = -1.0f;
	}

	return intersect;
}

struct World
{
	std::vector<Sphere> m_spheres;
};

Vec3f colorFromRay(const World& world, const Ray& ray)
{
	for (const Sphere& sphere : world.m_spheres)
	{
		Intersection intersect = intersectSphere(sphere, ray);

		if (intersect.m_tAt > 0.0f)
		{
			return 0.5f * (intersect.m_normal + 1.0f);
		}
	}

	Vec3f dirUnit = normalized(ray.m_direction);
	float t = 0.5f * (dirUnit.y + 1.0f);

	Vec3f white(1.0f, 1.0f, 1.0f);
	Vec3f skyBlue(0.5f, 0.7f, 1.0f);

	return lerp(t, white, skyBlue);
}

int main()
{
	int width = 600;
	int height = 300;
	int components = 3;
	Image image(width, height, components);

	Vec3f origin(0.0f, 0.0f, 0.0f);
	Vec3f lowerLeft(-2.0f, -1.0f, -1.0f);
	Vec3f horizontal(4.0f, 0.0f, 0.0f);
	Vec3f vertical(0.0f, 2.0f, 0.0f);
	const Camera camera(origin, lowerLeft, horizontal, vertical);

	World world;
	world.m_spheres.emplace_back(Vec3f(0.0f, 0.0f, -1.0f), 0.5f);

	for (int pixel_y = 0; pixel_y < height; ++pixel_y)
	{
		for (int pixel_x = 0; pixel_x < width; ++pixel_x)
		{
			Ray ray = getRayThroughPixel(camera, pixel_x, pixel_y, image.m_width, image.m_height);

			Vec3f rgb = colorFromRay(world, ray); 
			rgb *= 255.99f;

			image.writePixel(pixel_x, (height - 1) - pixel_y, rgb.x, rgb.y, rgb.z);
		}
	}

	bool bWasWritten = image.save("render.png");
	assert(bWasWritten);

	return 0;
}

