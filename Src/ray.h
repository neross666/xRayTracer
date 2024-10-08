﻿#pragma once

#include "geometry.h"

class Ray
{
public:
	Vec3f origin;
	Vec3f direction;
	Vec3f throughput;
	int depth;

	Ray() {}
	Ray(const Vec3f& origin, const Vec3f& direction)
		: origin(origin), direction(direction)
	{
	}

	Vec3f operator()(float t) const { return origin + t * direction; }
};


struct SurfaceInfo {
	Vec3f position;
	Vec3f ng;    // geometric normal
	Vec3f ns;    // shading normal
	Vec3f dpdu;  // tangent vector
	Vec3f dpdv;  // bitangent vector
	Vec2f texcoords;
	Vec2f barycentric;
};

class Object;
struct IntersectInfo {
	float t1 = kInfinity;	// [medium]:distance To Surface
	float t = kInfinity;	// distance to the hit point
	SurfaceInfo surfaceInfo;
	const Object* hitObject = nullptr;
};

struct AABB {
	Vec3f pMin;
	Vec3f pMax;
};