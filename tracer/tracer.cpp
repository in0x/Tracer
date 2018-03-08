#pragma once

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_ERROR 0

#include "stb_image_write.h"
#include "Vec3.h"

#include <stdint.h>
#include <assert.h>
#include <time.h>
#include <vector>

#define EXPAND_HELPER(x) #x
#define EXPAND(x) EXPAND_HELPER(x)
#define __LOCATION_INFO__ "In: " __FILE__ "\nAt: " EXPAND(__LINE__) ", " __FUNCTION__ "() "
#define LOG(x) printf(x __LOCATION_INFO__)

void initRand()
{
	srand(time(0));
}

float randDecimal()
{
	return (float)rand() / (float)RAND_MAX;
}

float randInRange(float min, float max)
{
	return (((float)rand() - min) / (float)RAND_MAX) * (max - min) + min;
}

Vec3f randInUnitSphere()
{
	Vec3f point;

	do
	{
		point.x = randInRange(-1.0f, 1.0f);
		point.y = randInRange(-1.0f, 1.0f);
		point.z = randInRange(-1.0f, 1.0f);
	} while (length2(point) >= 1.0f);

	return point;
}

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

Ray getRayThroughPixelSuperSampled(const Camera& camera, int pixelX, int pixelY, int widthPixels, int heightPixels)
{
	float u = ((float)pixelX + randDecimal()) / float(widthPixels);
	float v = ((float)pixelY + randDecimal()) / float(heightPixels);

	return Ray(camera.m_rayOrigin, camera.m_lowerLeft + u * camera.m_horizontal + v * camera.m_vertical);
}

struct Sphere
{
	Vec3f m_position;
	float m_radius;
};

struct Intersection
{
	float m_tAt;
	Vec3f m_point;
	Vec3f m_normal;
};

struct Material
{
	typedef size_t ID;

	Vec3f albedo;
};

struct Object
{
	union
	{
		Sphere sphere;
	} m_collision;

	enum
	{
		COL_SPHERE
	} m_collisionType;

	Material::ID m_material;
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
	std::vector<Object> m_objects;
	std::vector<Material> m_materials;

	Material::ID m_defaultMatID = std::numeric_limits<size_t>::max();

	void addSphere(const Vec3f& position, float radius)
	{
		Object object;

		object.m_collision.sphere.m_position = position;
		object.m_collision.sphere.m_radius = radius;
		object.m_collisionType = Object::COL_SPHERE;
		object.m_material = m_defaultMatID;
		m_objects.emplace_back(object);
	}

	Material::ID addMaterial(const Material& material)
	{
		m_materials.push_back(material);
		return  m_materials.size() - 1;
	}

	// As with everything, consider offering move here later.
	Material::ID setDefaultMaterial(const Material& defaultMaterial)
	{
		m_defaultMatID = addMaterial(defaultMaterial);
		return m_defaultMatID;
	}
};

Intersection intersectObject(const Object& object, const Ray& ray)
{
	Intersection intersect;

	switch (object.m_collisionType)
	{
	case Object::COL_SPHERE:
		intersect = intersectSphere(object.m_collision.sphere, ray);
		break;

	default:
		LOG("Unhandled collision type");
		assert(false);
		break;
	}

	return intersect;
}

Vec3f getSkyColor(const Ray& ray)
{
	Vec3f dirUnit = normalized(ray.m_direction);
	float t = 0.5f * (dirUnit.y + 1.0f);

	Vec3f white{ 1.0f, 1.0f, 1.0f };
	Vec3f skyBlue{ 0.5f, 0.7f, 1.0f };

	return lerp(t, white, skyBlue);
}

Vec3f colorFromRay(const World& world, const Ray& ray, float tMin, float tMax)
{
	for (const Object& object : world.m_objects)
	{
		Intersection intersect = intersectObject(object, ray);

		if (intersect.m_tAt > 0.0f && intersect.m_tAt > tMin && intersect.m_tAt < tMax)
		{
			Vec3f target = intersect.m_point + intersect.m_normal + randInUnitSphere();
			return 0.5f * colorFromRay(world, Ray(intersect.m_point, target - intersect.m_point), tMin, tMax);
		}
	}

	return getSkyColor(ray);
}

int main()
{
	initRand();

	int width = 600;
	int height = 300;
	int components = 3;
	Image image(width, height, components);

	Vec3f origin{ 0.0f, 0.0f, 0.0f };
	Vec3f lowerLeft{ -2.0f, -1.0f, -1.0f };
	Vec3f horizontal{ 4.0f, 0.0f, 0.0f };
	Vec3f vertical{ 0.0f, 2.0f, 0.0f };
	const Camera camera(origin, lowerLeft, horizontal, vertical);

	const int aaSamples = 2;

	World world;
	world.addSphere(Vec3f{ 0.0f, 0.0f, -1.0f }, 0.5f);
	world.addSphere(Vec3f{ 0.0f, -100.5f, -1.0f }, 100.0f);

	const float tIntersectMin = 0.001f; // Helps with self shadowing.
	const float tIntersectMax = FLT_MAX;

	for (int pixel_y = 0; pixel_y < height; ++pixel_y)
	{
		for (int pixel_x = 0; pixel_x < width; ++pixel_x)
		{
			Vec3f rgb{ 0,0,0 };

			for (int i = 0; i < aaSamples; ++i)
			{
				Ray ray = getRayThroughPixelSuperSampled(camera, pixel_x, pixel_y, image.m_width, image.m_height);
				rgb += colorFromRay(world, ray, tIntersectMin, tIntersectMax);
			}

			rgb /= (float)aaSamples; // Average over aa samples.
			rgb.x = sqrt(rgb.x); rgb.y = sqrt(rgb.y); rgb.z = sqrt(rgb.z); // Gamma2 correct.
			rgb *= 255.99f; // Move up to 0 <-> 255 range.

			image.writePixel(pixel_x, (height - 1) - pixel_y, rgb.x, rgb.y, rgb.z);
		}
	}

	bool bWasWritten = image.save("render.png");
	assert(bWasWritten);

	return 0;
}

