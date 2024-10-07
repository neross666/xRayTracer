#pragma once

#include "ray.h"
#include "sampler.h"
#include <spdlog/spdlog.h>

class Camera
{
protected:
	float aspect_ratio;
	Matrix44f camera2world;

public:
	Camera(float aspect_ratio_, const Matrix44f& c2w)
		: aspect_ratio(aspect_ratio_), camera2world(c2w)
	{
		auto forward = multDirMatrix(Vec3f(0.0f, 0.0f, -1.0f), c2w);
		spdlog::info("[Camera] position: ({}, {}, {})", camera2world[3][0], camera2world[3][1],
			camera2world[3][2]);
		spdlog::info("[Camera] forward: ({}, {}, {})", forward[0], forward[1],
			forward[2]);
	}

	void setTransform(const Matrix44f& c2w) {
		camera2world = c2w;
	}

	// sample ray emitting from the given sensor coordinate
	virtual bool sampleRay(const Vec2f& uv, Sampler& sampler, Ray& ray,
		float& pdf) const = 0;
};

// pinhole camera
class PinholeCamera : public Camera
{
private:
	float FOV;
	float scale;

public:
	PinholeCamera(float aspect_ratio_, const Matrix44f& c2w,
		float FOV = 90.0f)
		: Camera(aspect_ratio_, c2w)
	{
		scale = std::tan(FOV * 0.5f * PI / 180.f);
		spdlog::info("[PinholeCamera] scale: {}", scale);
	}

	bool sampleRay(const Vec2f& pixel, Sampler& sampler, Ray& ray,
		float& pdf) const override
	{
		const Vec3f dir((2 * pixel[0] - 1) * scale, (1 - 2 * pixel[1]) * scale / aspect_ratio, -1);
		ray.direction = normalize(multDirMatrix(dir, camera2world));

		ray.origin = Vec3f(camera2world[3][0], camera2world[3][1], camera2world[3][2]);

		pdf = 1.0f;

		return true;
	}
};