#pragma once

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_ERROR 0

#include "stb_image_write.h"
#include "Vec3.h"

#include "enkiTS/TaskScheduler_c.h"

#include <stdint.h>
#include <assert.h>
#include <time.h>
#include <stdio.h>
#include <direct.h>
#include <vector>
#include <immintrin.h>

#ifdef max
#undef max
#endif

#define EXPAND_HELPER(x) #x
#define EXPAND(x) EXPAND_HELPER(x)
#define __LOCATION_INFO__ "In: " __FILE__ "\nAt: " EXPAND(__LINE__) ", " __FUNCTION__ "() "
#define LOG(x) printf(x __LOCATION_INFO__)
#define PI 3.14159265358979323846

typedef __m128 m128;
typedef __m128i m128i;

void initRand()
{
	srand(time(0));
}

float randDecimal()
{
	return (float)rand() / ((float)RAND_MAX + 1.0f);
}

float randInRange(float min, float max)
{
	return (((float)rand() - min) / ((float)RAND_MAX + 1.0f)) * (max - min) + min;
}

template<typename T>
T const& min(T const& a, T const& b)
{
	return (a < b) ? a : b;
}

template<typename T>
T const& max(T const& a, T const& b)
{
	return (a > b) ? a : b;
}

vec3 randInUnitSphere()
{
	vec3 point;

	do
	{
		point.x = randInRange(-1.0f, 1.0f);
		point.y = randInRange(-1.0f, 1.0f);
		point.z = randInRange(-1.0f, 1.0f);
	} while (length2(point) >= 1.0f);

	return point;
}

vec3 randInUnitDisk()
{
	vec3 point{ 0.0f, 0.0f, 0.0f };

	do
	{
		point.x = randInRange(-1.0f, 1.0f);
		point.y = randInRange(-1.0f, 1.0f);
	} while (dot(point, point) >= 1.0f);

	return point;
}

template<typename T>
T lerp(float t, T a, T b)
{
	return (1.0f - t) * a + t * b;
}

struct Ray
{
	vec3 m_origin;
	vec3 m_direction;
};

vec3 pointAt(const Ray& ray, float t)
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
	vec3 m_rayOrigin;
	vec3 m_lowerLeft;
	vec3 m_horizontal;
	vec3 m_vertical;

	vec3 baseUp; // v
	vec3 baseRight; // u 
	vec3 baseForward; // w

	float m_lensRadius;
};

Camera createCamera(const vec3& origin, const vec3& viewLookAt, const vec3& viewUp, float vertFov, float aspect, float aperture, float focusDist)
{
	float theta = vertFov * PI / 180.0f;
	float halfHeight = tan(theta / 2.0f);
	float halfWidth = aspect * halfHeight;

	Camera camera;

	camera.m_lensRadius = aperture / 2.0f;

	camera.baseForward = normalized(origin - viewLookAt);
	camera.baseRight = normalized(cross(viewUp, camera.baseForward));
	camera.baseUp = cross(camera.baseForward, camera.baseRight);

	camera.m_rayOrigin = origin;
	camera.m_lowerLeft = origin - halfWidth * focusDist * camera.baseRight - halfHeight * focusDist * camera.baseUp - focusDist * camera.baseForward;
	camera.m_horizontal = 2.0f * halfWidth * focusDist * camera.baseRight;
	camera.m_vertical = 2.0f * halfHeight * focusDist * camera.baseUp;

	return camera;
}

Ray getRay(const Camera& camera, float u, float v)
{
	vec3 rand2D = camera.m_lensRadius * randInUnitDisk();
	vec3 offset = camera.baseRight * rand2D.x + camera.baseUp * rand2D.y;

	return Ray{ camera.m_rayOrigin + offset, camera.m_lowerLeft + u * camera.m_horizontal + v * camera.m_vertical - camera.m_rayOrigin - offset };
}

Ray getRayThroughPixel(const Camera& camera, int pixelX, int pixelY, int widthPixels, int heightPixels)
{
	float u = (float)pixelX / float(widthPixels);
	float v = (float)pixelY / float(heightPixels);
	return getRay(camera, u, v);
}

Ray getRayThroughPixelSubSampled(const Camera& camera, int pixelX, int pixelY, int widthPixels, int heightPixels)
{
	float u = ((float)pixelX + randDecimal()) / float(widthPixels);
	float v = ((float)pixelY + randDecimal()) / float(heightPixels);
	return getRay(camera, u, v);
}

struct Sphere
{
	vec3 m_position;
	float m_radius;
};

struct Intersection
{
	Intersection()
		: m_tAt(0.0f)
		, m_bHit(false)
	{}

	vec3 m_point;
	vec3 m_normal;
	float m_tAt;
	bool m_bHit;
};

struct Material
{
	typedef int32_t ID;
	// As a convention, if no light is scattered back, attenuation is set to (0, 0, 0).
	typedef void(ScatterFunc)(const Material& material, const Intersection& intersect, const Ray& rayIn, Ray* rayOut, vec3* attenuation);

	vec3 m_albedo;
	ScatterFunc* m_scatterFunc;
	float m_roughness;
	float m_refractIdx;
};

struct SphereData
{
	SphereData()
		: count(0)
	{}

	std::vector<float> x;
	std::vector<float> y;
	std::vector<float> z;
	std::vector<float> radius;
	std::vector<Material::ID> materials;
	uint32_t count;
};

// Calculates the three dimensional dot
// product between a and b.
inline m128 __vectorcall dot(m128 a, m128 b)
{
	static const m128 channel_mask_xyz = _mm_set_ps(0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000);

	m128 temp = _mm_mul_ps(a, b);
	temp = _mm_and_ps(temp, channel_mask_xyz);
	return _mm_add_ps(temp, temp);
}

// Calculates the three dimensional dot product
// for the for vectors stored in each channel.
inline m128 __vectorcall dot_4(
	m128 x_1, m128 y_1, m128 z_1,
	m128 x_2, m128 y_2, m128 z_2)
{
	m128 dot_x_2 = _mm_mul_ps(x_1, x_2);
	m128 dot_y_2 = _mm_mul_ps(y_1, y_2);
	m128 dot_z_2 = _mm_mul_ps(z_1, z_2);
	m128 dot_sum = _mm_add_ps(dot_x_2, dot_y_2);
	return _mm_add_ps(dot_sum, dot_z_2);
}

// Computes the minimum ps value in v and
// returns it replicated to all channels.
inline m128 __vectorcall _mm_hmin_ps(m128 v)
{
	v = _mm_min_ps(v, _mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 1, 0, 3)));
	v = _mm_min_ps(v, _mm_shuffle_ps(v, v, _MM_SHUFFLE(1, 0, 3, 2)));
	return v;
}

// Returns a 4-bit code where bit0..bit3 is X..W
inline uint32_t __vectorcall mask(m128 v) { return _mm_movemask_ps(v) & 15; }

inline bool __vectorcall any(m128 v) { return mask(v) != 0; }
inline bool __vectorcall all(m128 v) { return mask(v) == 15; }

float __vectorcall store_non_zero_element_ps(m128 v)
{
	float stored_value = 0.0f;
	float values_ps[4];
	_mm_storeu_ps(values_ps, v);

	for (int i = 0; i < 4; ++i)
	{
		float value = values_ps[i];
		if (value != 0.0f)
		{
			assert(stored_value == 0.0f); // We should have only stored one non-zero channel.
			stored_value = value;
		}
	}

	return stored_value;
}

inline float __vectorcall get_at_i(m128 m, int i)
{
	m128i shuffMask = _mm_cvtsi32_si128(i);
	return _mm_cvtss_f32(_mm_permutevar_ps(m, shuffMask));
}

#define SIMD_INTERSECT 1

#if SIMD_INTERSECT

void __vectorcall intersectSphere(SphereData const* __restrict spheres, Ray const* __restrict ray, float tMin, float tMax, Intersection* outIntersect, int32_t* outHitId)
{
	m128 const ray_origin_x = _mm_set_ps1(ray->m_origin.x);
	m128 const ray_origin_y = _mm_set_ps1(ray->m_origin.y);
	m128 const ray_origin_z = _mm_set_ps1(ray->m_origin.z);

	m128 const ray_direction_x = _mm_set_ps1(ray->m_direction.x);
	m128 const ray_direction_y = _mm_set_ps1(ray->m_direction.y);
	m128 const ray_direction_z = _mm_set_ps1(ray->m_direction.z);

	float const a_f = dot(ray->m_direction, ray->m_direction);
	m128 const a = _mm_set_ps1(a_f);

	m128 const t_min = _mm_set_ps1(tMin);
	m128 const zero = _mm_set_ps1(0.0f); 
	m128 const flt_max = _mm_set_ps1(FLT_MAX);

	m128 closest_t_at = _mm_set_ps1(FLT_MAX);
	m128 closest_index = zero;
	m128 closest_sphere_x = zero;
	m128 closest_sphere_y = zero;
	m128 closest_sphere_z = zero;
	m128 closest_radius = zero;

	size_t const numSpheres = spheres->count;

	static const float flt_plus_inf = logf(0);
	m128 const minus_inf = _mm_set_ps1(flt_plus_inf);
	m128 const plus_inf = _mm_set_ps1(-flt_plus_inf);

	for (size_t i = 0; i < numSpheres; i += 4)
	{
		m128 t_max = _mm_set_ps1(tMax);

		m128 const position_x = _mm_loadu_ps(spheres->x.data() + i); // TODO: 16 align the sphere memory so we can load aligned
		m128 const position_y = _mm_loadu_ps(spheres->y.data() + i);
		m128 const position_z = _mm_loadu_ps(spheres->z.data() + i);

		m128 const to_center_x = _mm_sub_ps(ray_origin_x, position_x);
		m128 const to_center_y = _mm_sub_ps(ray_origin_y, position_y);
		m128 const to_center_z = _mm_sub_ps(ray_origin_z, position_z);

		// This contains the b for 4 spheres, x => b for spheres[i], x => b for spheres[i + 1] etc
		m128 const b = dot_4(to_center_x, to_center_y, to_center_z, ray_direction_x, ray_direction_y, ray_direction_z);

		m128 c = dot_4(to_center_x, to_center_y, to_center_z, to_center_x, to_center_y, to_center_z);
		m128 radii = _mm_loadu_ps(spheres->radius.data() + i);
		c = _mm_sub_ps(c, _mm_mul_ps(radii, radii));

		m128 discriminants = _mm_mul_ps(b, b);
		discriminants = _mm_sub_ps(discriminants, _mm_mul_ps(a, c));
		//discriminants = _mm_blendv_ps(zero, discriminants, _mm_cmpgt_ps(discriminants, zero));
		m128 sqrts = _mm_sqrt_ps(discriminants);

		m128 b_neg = _mm_sub_ps(zero, b);
		m128 t_neg = _mm_div_ps(_mm_sub_ps(b_neg, sqrts), a);
		m128 t_pos = _mm_div_ps(_mm_add_ps(b_neg, sqrts), a);

#if 1
		float discriminant_results[4];
		_mm_storeu_ps(discriminant_results, discriminants);
		float t_test_results[8];
		_mm_storeu_ps(t_test_results, t_neg);
		_mm_storeu_ps(t_test_results + 4, t_pos);

		for (int test_idx = 0; test_idx < 4; ++test_idx)
		{
			if (discriminant_results[test_idx] <= 0.0f)
			{
				continue;
			}

			float t_test_neg = t_test_results[test_idx];
			float t_test_pos = t_test_results[test_idx + 4];

			if ((t_test_neg > tMin && t_test_neg < tMax))
			{
				vec3 spherePos = vec3{ get_at_i(position_x, test_idx), get_at_i(position_y, test_idx), get_at_i(position_z, test_idx) };
				float radius = get_at_i(radii, test_idx);

				outIntersect->m_tAt = t_test_neg;
				outIntersect->m_point = pointAt(*ray, outIntersect->m_tAt);
				outIntersect->m_normal = (outIntersect->m_point - spherePos) / radius;
				outIntersect->m_bHit = true;
				*outHitId = i + test_idx;
				tMax = t_test_neg;
			}
			else if (t_test_pos > tMin && t_test_pos < tMax)
			{
				vec3 spherePos = vec3{ get_at_i(position_x, test_idx), get_at_i(position_y, test_idx), get_at_i(position_z, test_idx) };
				float radius = get_at_i(radii, test_idx);

				outIntersect->m_tAt = t_test_pos;
				outIntersect->m_point = pointAt(*ray, outIntersect->m_tAt);
				outIntersect->m_normal = (outIntersect->m_point - spherePos) / radius;
				outIntersect->m_bHit = true;
				*outHitId = i + test_idx;
				tMax = t_test_pos;
			}
		}
	}
#else
		// Filter nans
		//m128 const filtered_tneg_plus = _mm_min_ps(t_neg, plus_inf);
		//m128 const filtered_tpos_plus = _mm_min_ps(t_pos, plus_inf);
		//t_neg = _mm_max_ps(filtered_tneg_plus, minus_inf);
		//t_pos = _mm_max_ps(filtered_tpos_plus, minus_inf);

		m128 within_neg_bounds = _mm_and_ps(_mm_cmpgt_ps(t_neg, t_min), _mm_cmplt_ps(t_neg, t_max));
		m128 within_pos_bounds = _mm_and_ps(_mm_cmpgt_ps(t_pos, t_min), _mm_cmplt_ps(t_pos, t_max));

		m128 discriminant_pass = _mm_cmpgt_ps(discriminants, zero);
		within_neg_bounds = _mm_and_ps(discriminant_pass, within_neg_bounds);
		within_neg_bounds = _mm_and_ps(discriminant_pass, within_pos_bounds);

		m128 anyHit = _mm_and_ps(discriminant_pass, _mm_or_ps(within_neg_bounds, within_pos_bounds));

		if (!any(anyHit))
		{
			continue;
		}


		// do a masked store from either tneg or tpos -> tAt for each intersection
		m128 t_at;
		t_at = _mm_blendv_ps(flt_max, t_neg, within_neg_bounds);
		t_at = _mm_blendv_ps(t_at, t_pos, within_pos_bounds);
		m128 const min_t_at = _mm_hmin_ps(t_at);

		closest_t_at = _mm_min_ps(closest_t_at, min_t_at);
		
		t_max = _mm_min_ps(t_max, closest_t_at);

		// Blend in index of element that was just written if any
		m128 new_closest_t_found = _mm_cmpeq_ps(t_at, closest_t_at);
		closest_t_at = _mm_and_ps(closest_t_at, new_closest_t_found);

		m128 indices = _mm_set_ps(test_idx, test_idx + 1, test_idx + 2, test_idx + 3);
		indices = _mm_blendv_ps(zero, indices, new_closest_t_found);
		closest_index = _mm_blendv_ps(closest_index, indices, new_closest_t_found);

		closest_sphere_x = _mm_blendv_ps(closest_sphere_x, position_x, new_closest_t_found);
		closest_sphere_y = _mm_blendv_ps(closest_sphere_y, position_y, new_closest_t_found);
		closest_sphere_z = _mm_blendv_ps(closest_sphere_z, position_z, new_closest_t_found);

		// zero out all non written channels so we can store either [old_radius] or [0 new_radius 0 0]
		radii = _mm_blendv_ps(zero, radii, new_closest_t_found);
		closest_radius = _mm_blendv_ps(closest_radius, radii, new_closest_t_found);
	}

	bool hitAny = any(_mm_cmplt_ps(closest_t_at, flt_max));

	if (!hitAny)
	{
		outIntersect->m_bHit = false;
		return;
	}

	float hit_t = _mm_cvt_ss2si(closest_t_at);
	float hit_radius = store_non_zero_element_ps(closest_radius);

	vec3 hit_pos;
	hit_pos.x = store_non_zero_element_ps(closest_sphere_x);
	hit_pos.y = store_non_zero_element_ps(closest_sphere_y);
	hit_pos.z = store_non_zero_element_ps(closest_sphere_z);

	outIntersect->m_tAt = hit_t;
	outIntersect->m_point = pointAt(*ray, hit_t);
	outIntersect->m_normal = (outIntersect->m_point - hit_pos) / hit_radius;
	outIntersect->m_bHit = true;
	*outHitId = store_non_zero_element_ps(closest_index);
#endif
}

#else

void __vectorcall intersectSphere(SphereData const* __restrict spheres, Ray const* __restrict ray, float tMin, float tMax, Intersection* outIntersect, int32_t* outHitId)
{
	memset(outIntersect, 0, sizeof(Intersection));

	float a = dot(ray->m_direction, ray->m_direction);

	size_t const numSpheres = spheres->count;
	for (size_t i = 0; i < numSpheres; i++)
	{
		vec3 spherePos = vec3{ spheres->x[i], spheres->y[i], spheres->z[i] };
		float radius = spheres->radius[i];

		vec3 toCenter = ray->m_origin - spherePos;

		float b = dot(toCenter, ray->m_direction);
		float c = dot(toCenter, toCenter) - radius * radius;

		float discriminant = b * b - a * c;

		if (discriminant > 0.0f)
		{
			float tNeg = (-b - sqrt(discriminant)) / a;
			float tPos = (-b + sqrt(discriminant)) / a;

			if ((tNeg > tMin && tNeg < tMax))
			{
				outIntersect->m_tAt = tNeg;
				outIntersect->m_point = pointAt(*ray, outIntersect->m_tAt);
				outIntersect->m_normal = (outIntersect->m_point - spherePos) / radius;
				outIntersect->m_bHit = true;
				*outHitId = i;
				tMax = tNeg;
			}
			else if (tPos > tMin && tPos < tMax)
			{
				outIntersect->m_tAt = tPos;
				outIntersect->m_point = pointAt(*ray, outIntersect->m_tAt);
				outIntersect->m_normal = (outIntersect->m_point - spherePos) / radius;
				outIntersect->m_bHit = true;
				*outHitId = i;
				tMax = tPos;
			}
		}
	}
}

#endif

struct World
{
	SphereData m_spheres;
	std::vector<Material> m_materials;
	Material::ID m_defaultMatID;

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

	Material::ID getDefaultMaterial() const
	{
		return m_defaultMatID;
	}

	Material::ID setDefaultMaterial(const Material& material)
	{
		m_defaultMatID = addMaterial(material);
		return m_defaultMatID;
	}

	void addSphere(const vec3& position, float radius, Material::ID material)
	{
		m_spheres.x.emplace_back(position.x);
		m_spheres.y.emplace_back(position.y);
		m_spheres.z.emplace_back(position.z);
		m_spheres.radius.emplace_back(radius);
		m_spheres.materials.emplace_back(material);
		m_spheres.count++;
	}
};

bool intersectWorld(const World& world, const Ray& ray, float tMin, float tMax, Intersection* outIntersect, Material::ID* outHitMaterial)
{
	size_t hitObjectId;
	float closestT = tMax; // TOOD I think we forgot to check this
	int32_t hitSphereId = 0;

	intersectSphere(&world.m_spheres, &ray, tMin, tMax, outIntersect, &hitSphereId);

	if (outIntersect->m_bHit)
	{
		*outHitMaterial = world.m_spheres.materials[hitSphereId];
	}

	return outIntersect->m_bHit;
}

vec3 getSkyColor(Ray const& ray)
{
	vec3 dirUnit = normalized(ray.m_direction);
	float t = 0.5f * (dirUnit.y + 1.0f);

	static vec3 const white{ 1.0f, 1.0f, 1.0f };
	static vec3 const skyBlue{ 0.5f, 0.7f, 1.0f };

	return lerp(t, white, skyBlue);
}

vec3 colorFromRay(World const& world, Ray const& ray, int bounces, const int maxBounces, const float tMin, const float tMax)
{
	Intersection intersect;
	Material::ID hitMaterial;

	if (intersectWorld(world, ray, tMin, tMax, &intersect, &hitMaterial))
	{
		const Material& material = world.m_materials[hitMaterial];

		Ray scattered;
		vec3 attenuation;
		material.m_scatterFunc(material, intersect, ray, &scattered, &attenuation);

		if (bounces < maxBounces && length2(attenuation) > 0.0f)
		{
			return attenuation * colorFromRay(world, scattered, bounces + 1, maxBounces, tMin, tMax);
		}
		else
		{
			return vec3{ 0,0,0 };
		}
	}
	else
	{
		return getSkyColor(ray);
	}
}

void scatterDiffuse(const Material& material, const Intersection& intersect, const Ray& rayIn, Ray* rayOut, vec3* attenuation)
{
	vec3 target = intersect.m_point + intersect.m_normal + randInUnitSphere();
	*rayOut = { intersect.m_point, target - intersect.m_point };
	*attenuation = material.m_albedo;
}

void scatterMetallic(const Material& material, const Intersection& intersect, const Ray& rayIn, Ray* rayOut, vec3* attenuation)
{
	vec3 reflected = reflect(normalized(rayIn.m_direction), intersect.m_normal);
	*rayOut = { intersect.m_point, reflected + material.m_roughness * randInUnitSphere() };

	if (dot(rayOut->m_direction, intersect.m_normal) > 0)
	{
		*attenuation = material.m_albedo;
	}
	else
	{
		*attenuation = { 0.0f, 0.0f, 0.0f };
	}
}

float schlick(float cosine, float refractIdx)
{
	float r0 = (1.0f - refractIdx) / (1.0f + refractIdx);
	r0 *= r0;
	return r0 + (1.0f - r0) * pow((1.0f - cosine), 5.0f);
}

void scatterDielectric(const Material& material, const Intersection& intersect, const Ray& rayIn, Ray* rayOut, vec3* outAttenuation)
{
	vec3 outwardNormal;
	vec3 reflected = reflect(rayIn.m_direction, intersect.m_normal);

	*outAttenuation = vec3{ 1.0f, 1.0f, 1.0f };
	vec3 refracted;
	float reflectProb = 0.0f;
	float ni_over_nt = 0.0f;
	float cosine = 0.0f;

	if (dot(rayIn.m_direction, intersect.m_normal) > 0.0f)
	{
		outwardNormal = -intersect.m_normal;
		ni_over_nt = material.m_refractIdx;
		cosine = ni_over_nt * dot(rayIn.m_direction, intersect.m_normal) / length(rayIn.m_direction);
	}
	else
	{
		outwardNormal = intersect.m_normal;
		ni_over_nt = 1.0f / material.m_refractIdx;
		cosine = -dot(rayIn.m_direction, intersect.m_normal / length(rayIn.m_direction));
	}

	if (refract(rayIn.m_direction, outwardNormal, ni_over_nt, &refracted))
	{
		reflectProb = schlick(cosine, material.m_refractIdx);
	}
	else
	{
		reflectProb = 1.0f;
	}

	if (randDecimal() < reflectProb)
	{
		*rayOut = Ray{ intersect.m_point, reflected };
	}
	else
	{
		*rayOut = Ray{ intersect.m_point, refracted };
	}
}

Material createDiffuse(const vec3& albedo)
{
	Material diffuse;
	memset(&diffuse, 0, sizeof(Material));

	diffuse.m_albedo = albedo;
	diffuse.m_scatterFunc = &scatterDiffuse;
	return diffuse;
}

Material createMetallic(const vec3& albedo, float roughness)
{
	Material metallic;
	memset(&metallic, 0, sizeof(Material));

	metallic.m_albedo = albedo;
	metallic.m_roughness = roughness;
	metallic.m_scatterFunc = &scatterMetallic;
	return metallic;
}

Material createDielectric(float refractIdx)
{
	Material dielectirc;
	memset(&dielectirc, 0, sizeof(Material));

	dielectirc.m_refractIdx = refractIdx;
	dielectirc.m_scatterFunc = &scatterDielectric;
	return dielectirc;
}

void randomFillWorld(World* world)
{
	world->setDefaultMaterial(createDiffuse(vec3{ 0.5f, 0.5f, 0.5f }));
	world->addSphere(vec3{ 0.0f, -1000.0f, 0.0f }, 1000.0f, world->getDefaultMaterial());

	for (float a = -11.0f; a < 11.0f; a++)
	{
		for (float b = -11.0f; b < 11.0f; b++)
		{
			float choose_mat = randDecimal();

			vec3 center{ a + 0.9f * randDecimal(), 0.2f, b + 0.9f * randDecimal() };

			if (length((center - vec3{ 4.0f, 0.2f, 0.0f })) > 0.9f)
			{
				if (choose_mat < 0.8f)
				{
					Material diffuse = createDiffuse(vec3{ randDecimal() * randDecimal(), randDecimal() * randDecimal(), randDecimal() * randDecimal() });
					world->addSphere(center, 0.2f, world->addMaterial(diffuse));
				}
				else if (choose_mat < 0.95f)
				{
					vec3 albedo{ randInRange(0.5f, 1.0f), randInRange(0.5f, 1.0f), randInRange(0.5f, 1.0f) };
					float roughness = randInRange(0.0f, 0.5f);
					Material metal = createMetallic(albedo, roughness);

					world->addSphere(center, 0.2f, world->addMaterial(metal));
				}
				else
				{
					Material glass = createDielectric(1.5f);
					world->addSphere(center, 0.2f, world->addMaterial(glass));
				}
			}
		}
	}

	Material diffuse = createDiffuse(vec3{ 0.4f, 0.2f, 0.1f });
	world->addSphere(vec3{ -4.0f, 1.0f, 0.0f }, 1.0f, world->addMaterial(diffuse));

	Material metal = createMetallic(vec3{ 0.7f, 0.6f, 0.5f }, 0.0f);
	world->addSphere(vec3{ 4.0f, 1.0f, 0.0f }, 1.0f, world->addMaterial(metal));

	Material glass = createDielectric(1.5f);
	world->addSphere(vec3{0.0f, 1.0f, 0.0f}, 1.0f, world->addMaterial(glass));
}

void basicFillWorld(World* world)
{
	world->setDefaultMaterial(createDiffuse(vec3{ 0.5f, 0.0f, 0.0f }));
	world->addSphere(vec3{ 0.0f, -1000.0f, 0.0f }, 1000.0f, world->getDefaultMaterial());
}

void ensureWorldDataIsPadded(World* world)
{
	int32_t const numSpheres = world->m_spheres.count;
	int32_t const simdWidth = 4;
	int32_t const requiredPadding = max(simdWidth - numSpheres, numSpheres % simdWidth);

	Material diffuse = createDiffuse(vec3{ randDecimal() * randDecimal(), randDecimal() * randDecimal(), randDecimal() * randDecimal() });
	Material::ID mat = world->addMaterial(diffuse);

	for (int i = 0; i < requiredPadding; ++i)
	{
		//vec3 center{ 0.9f * randDecimal(), 0.2f, 0.9f * randDecimal() };
		vec3 center{ -INFINITY, -INFINITY, -INFINITY };
		world->addSphere(center, -0.1f, mat);
	}
}

enkiTaskScheduler* g_taskScheduler;

void initTS()
{
	g_taskScheduler = enkiNewTaskScheduler();
	enkiInitTaskScheduler(g_taskScheduler);
}

void shutdownTS()
{
	enkiDeleteTaskScheduler(g_taskScheduler);
}

struct RayTraceJobData
{
	RayTraceJobData(const Camera* camera, const World* world, Image* image)
		: m_camera(camera)
		, m_world(world)
		, m_image(image)
	{}

	const Camera* m_camera;
	const World* m_world;
	Image* m_image; 
};

static void RunRayTraceJob(uint32_t start, uint32_t end, uint32_t threadnum, void* data)
{
	RayTraceJobData* job = (RayTraceJobData*)data;

	uint32_t width = job->m_image->m_width;
	uint32_t height = job->m_image->m_height;

	const float tIntersectMin = 0.001f;
	const float tIntersectMax = FLT_MAX;
	const int maxRayBounces = 50;
	const int pixelSubSamples = 8;

	//printf("RayTraceJob: THREAD ID -> %d | START -> %d | END -> %d | SIZE -> %d \n", threadnum, start, end, (end - start));

	// Pixels are numbered row-wise, starting in the top left.

	for (uint32_t pixel = start; pixel < end; ++pixel)
	{
		uint32_t pixel_y = pixel / width;
		uint32_t pixel_x = pixel - (pixel_y * width);

		vec3 rgb{ 0,0,0 };

		for (int i = 0; i < pixelSubSamples; ++i)
		{
			Ray ray = getRayThroughPixelSubSampled(*job->m_camera, pixel_x, pixel_y, width, height);
			rgb += colorFromRay(*job->m_world, ray, 0, maxRayBounces, tIntersectMin, tIntersectMax);
		}

		rgb /= (float)pixelSubSamples; // Average over aa samples.
		rgb.x = sqrt(rgb.x); rgb.y = sqrt(rgb.y); rgb.z = sqrt(rgb.z); // Gamma2 correct.
		rgb *= 255.99f; // Move up to 0 <-> 255 range.

		job->m_image->writePixel(pixel_x, (height - 1) - pixel_y, rgb.x, rgb.y, rgb.z);
	}
}

int main()
{
	initRand();

	initTS();
	
	uint32_t width = 1920;
	uint32_t height = 1080;
	uint32_t components = 3;
	Image image(width, height, components);

	vec3 origin{ 13.0f, 2.0f, 3.0f };
	vec3 lookAt{ 0.0f, 0.0f, 0.0f };
	vec3 up{ 0.0f, 1.0f, 0.0f };
	Camera camera = createCamera(origin, lookAt, up, 20.0f, (float)width / (float)height, 0.01f, 10.0f);
	
	World world;
	randomFillWorld(&world);
	//basicFillWorld(&world);
#if SIMD_INTERSECT
	ensureWorldDataIsPadded(&world);
#endif

	RayTraceJobData jobData(&camera, &world, &image);

#define MULTITHREADED 1

#if MULTITHREADED
	enkiTaskSet* task = enkiCreateTaskSet(g_taskScheduler, RunRayTraceJob);
	enkiAddTaskSetToPipeMinRange(g_taskScheduler, task, &jobData, width * height, width);
	enkiWaitForTaskSet(g_taskScheduler, task);
#else
	RunRayTraceJob(0, width * height, 0, (void*)&jobData);
#endif

	printf("Done rendering.\n");

	char img[1024];
	_getcwd(img, 1024);
	strcat_s(img, "\\render.png");

	printf("Saving to: %s.\n", img);

	bool bWasWritten = image.save("render.png");
	assert(bWasWritten);

	printf("Done saving.\n");
	printf("Shutting down TS.\n");

	enkiDeleteTaskSet(task);
	shutdownTS();

	printf("Finished shutting down TS.\n");
	printf("Opening image '%s'\n", img);

	char cmd[1024];
	sprintf_s(cmd, "explorer.exe %s\n", img);
	system(cmd);

	return 0;
}

