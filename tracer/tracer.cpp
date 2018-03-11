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
	Vec3f m_rayOrigin;
	Vec3f m_lowerLeft;
	Vec3f m_horizontal;
	Vec3f m_vertical;
};

Ray getRayThroughPixel(const Camera& camera, int pixelX, int pixelY, int widthPixels, int heightPixels)
{
	float u = (float)pixelX / float(widthPixels);
	float v = (float)pixelY / float(heightPixels);

	return Ray{ camera.m_rayOrigin, camera.m_lowerLeft + u * camera.m_horizontal + v * camera.m_vertical };
}

Ray getRayThroughPixelSubSampled(const Camera& camera, int pixelX, int pixelY, int widthPixels, int heightPixels)
{
	float u = ((float)pixelX + randDecimal()) / float(widthPixels);
	float v = ((float)pixelY + randDecimal()) / float(heightPixels);

	return Ray{ camera.m_rayOrigin, camera.m_lowerLeft + u * camera.m_horizontal + v * camera.m_vertical };
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
	typedef void(ScatterFunc)(const Material& material, const Intersection& interesect, const Ray& rayIn, Ray& rayOut, Vec3f& attenuation);

	Vec3f m_albedo;
	ScatterFunc* m_scatterFunc;
	float m_roughness;
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

	Material::ID m_materialID;
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
		addSphere(position, radius, m_defaultMatID);
	}

	void addSphere(const Vec3f& position, float radius, Material::ID material)
	{
		Object object;

		object.m_collision.sphere.m_position = position;
		object.m_collision.sphere.m_radius = radius;
		object.m_collisionType = Object::COL_SPHERE;
		object.m_materialID = material;
		m_objects.emplace_back(object);
	}

	void sanitizeMaterial(Material::ID id)
	{
		Material& material = m_materials[id];
		material.m_roughness = fmin(material.m_roughness, 1.0f);
	}

	Material::ID addMaterial(const Material& material)
	{
		m_materials.push_back(material); 
		Material::ID id = m_materials.size() - 1;
		sanitizeMaterial(id);
		return id;
	}

	Material::ID setDefaultMaterial(const Material& material)
	{
		m_defaultMatID = addMaterial(material);
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

const Object* intersectWorld(const World& world, const Ray& ray, Intersection& outIntersect, float tMin, float tMax)
{
	Intersection intersect;
	const Object* hitObject = nullptr;

	float closestT = tMax;

	for (const Object& object : world.m_objects)
	{
		intersect = intersectObject(object, ray);

		if (intersect.m_tAt > tMin && intersect.m_tAt < closestT)
		{
			outIntersect = intersect;
			hitObject = &object;
			closestT = intersect.m_tAt;
		}
	}

	return hitObject;
}

Vec3f getSkyColor(const Ray& ray)
{
	Vec3f dirUnit = normalized(ray.m_direction);
	float t = 0.5f * (dirUnit.y + 1.0f);

	Vec3f white{ 1.0f, 1.0f, 1.0f };
	Vec3f skyBlue{ 0.5f, 0.7f, 1.0f };

	return lerp(t, white, skyBlue);
}

Vec3f colorFromRay(const World& world, const Ray& ray, int bounces, int maxBounces, float tMin, float tMax)
{
	Intersection intersect;

	if (const Object* hitObject = intersectWorld(world, ray, intersect, tMin, tMax))
	{
		const Material& material = world.m_materials[hitObject->m_materialID];

		Ray scattered;
		Vec3f attenuation;
		material.m_scatterFunc(material, intersect, ray, scattered, attenuation);

		if (bounces < maxBounces && length2(attenuation) > 0.0f)
		{
			return attenuation * colorFromRay(world, scattered, bounces + 1, maxBounces, tMin, tMax);
		}
		else
		{
			return Vec3f{ 0,0,0 };
		}
	}
	else
	{
		return getSkyColor(ray);
	}
}

void scatterDiffuse(const Material& material, const Intersection& intersect, const Ray& rayIn, Ray& rayOut, Vec3f& attenuation)
{
	Vec3f target = intersect.m_point + intersect.m_normal + randInUnitSphere();
	rayOut = { intersect.m_point, target - intersect.m_point };
	attenuation = material.m_albedo;
}

void scatterMetal(const Material& material, const Intersection& intersect, const Ray& rayIn, Ray& rayOut, Vec3f& attenuation)
{
	Vec3f reflected = reflect(normalized(rayIn.m_direction), intersect.m_normal);
	rayOut = { intersect.m_point, reflected + material.m_roughness * randInUnitSphere() };

	if (dot(rayOut.m_direction, intersect.m_normal) > 0)
	{
		attenuation = material.m_albedo;
	}
	else
	{
		attenuation = { 0.0f, 0.0f, 0.0f };
	}
}

int main()
{
	initRand();

	int width = 600;
	int height = 300;
	int components = 3;
	Image image(width, height, components);

	Camera camera;
	camera.m_rayOrigin = { 0.0f, 0.0f, 0.0f };
	camera.m_lowerLeft = { -2.0f, -1.0f, -1.0f };
	camera.m_horizontal = { 4.0f, 0.0f, 0.0f };
	camera.m_vertical = { 0.0f, 2.0f, 0.0f };

	const int aaSamples = 4;

	World world;

	Material::ID redDiffuse = world.setDefaultMaterial({ Vec3f{ 0.8f, 0.3f, 0.3f }, &scatterDiffuse });
	Material::ID yellowDiffuse = world.setDefaultMaterial({ Vec3f{ 0.8f, 0.8f, 0.0f }, &scatterDiffuse });
	Material::ID silver = world.setDefaultMaterial({ Vec3f{ 0.8f, 0.8f, 0.8f }, &scatterMetal , 0.3f});
	Material::ID gold = world.setDefaultMaterial({ Vec3f{ 0.8f, 0.6f, 0.2f }, &scatterMetal, 1.0f});

	world.addSphere(Vec3f{ 0.0f, 0.0f, -1.0f }, 0.5f, redDiffuse);
	world.addSphere(Vec3f{ 0.0f, -100.5f, -1.0f }, 100.0f, yellowDiffuse);
	world.addSphere(Vec3f{ 1.0f, 0.0f, -1.0f }, 0.5f, gold);
	world.addSphere(Vec3f{ -1.0f, 0.0f, -1.0f }, 0.5f, silver);

	const float tIntersectMin = 0.001f; // Helps with self shadowing.
	const float tIntersectMax = FLT_MAX;
	const int maxRayBounces = 50;

	for (int pixel_y = 0; pixel_y < height; ++pixel_y)
	{
		for (int pixel_x = 0; pixel_x < width; ++pixel_x)
		{
			Vec3f rgb{ 0,0,0 };

			for (int i = 0; i < aaSamples; ++i)
			{
				Ray ray = getRayThroughPixelSubSampled(camera, pixel_x, pixel_y, image.m_width, image.m_height);
				rgb += colorFromRay(world, ray, 0, maxRayBounces, tIntersectMin, tIntersectMax);
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

