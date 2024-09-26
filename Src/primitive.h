#pragma once
#include <vector>
#include "material.h"
#include "ray.h"

class AreaLight;
class Primitive
{
public:
	Primitive(
		const std::vector<Vec3f>& vertices, 
		const std::vector<Vec3f>& normals, 
		const std::vector<Vec2f>& texcoords,
		Material* material,
		AreaLight* light);
	Primitive(
		std::vector<Vec3f>&& vertices, 
		std::vector<Vec3f>&& normals, 
		std::vector<Vec2f>&& texcoords,
		Material* material,
		AreaLight* light);
	~Primitive() = default;

	const std::vector<Vec3f>& vertices() const {
		return m_vertices;
	}

	const std::vector<Vec3f>& normals() const {
		return m_normals;
	}

	const std::vector<Vec2f>& texcoords() const {
		return m_texcoords;
	}

	bool hasSurface() const { 
		return m_material != nullptr; 
	}

	bool hasAreaLight() const { 
		return m_areaLight != nullptr; 
	}

	Material::MaterialType materialType() const;

	Vec3f evaluate() const;

	Vec3f Le(const SurfaceInfo& info, const Vec3f& wi) const;

private:
	std::vector<Vec3f> m_vertices;
	std::vector<Vec3f> m_normals;
	std::vector<Vec2f> m_texcoords;
	Material* m_material = nullptr;
	AreaLight* m_areaLight = nullptr;
};

class Object
{
public:
	Object() = default;

	virtual ~Object() = default;

	virtual bool intersect(const Ray& ray, IntersectInfo& info) const = 0;

	virtual bool occluded(const Ray& ray, float t_max) const = 0;	
};

class Sphere : public Object
{
public:
	Sphere(Vec3f center, float raduis)
		: m_center(center), m_raduis(raduis), m_raduis2(raduis*raduis), Object() {

	}
	~Sphere() = default;

	bool intersect(const Ray& ray, IntersectInfo& info) const override
	{
		float t = 0.0f;
		if (!doIntersect(ray.origin, ray.direction, t))
			return false;

		if (t < info.t)
		{
			info.t = t;
			info.surfaceInfo.position = ray(t);
			info.surfaceInfo.ng = normalize(ray(t) - m_center);
			info.surfaceInfo.ns = info.surfaceInfo.ng;
			info.surfaceInfo.texcoords = Vec2f(
				(1 + atan2(info.surfaceInfo.ng[2], info.surfaceInfo.ng[0]) / PI) * 0.5,
				acosf(info.surfaceInfo.ng[1]) / PI);
			info.hitPrimitive = nullptr/*this*/;
		}
		return true;
	}

	bool occluded(const Ray& ray, float t_max) const override
	{
		float t = 0.0f;
		return doIntersect(ray.origin, ray.direction, t) && t < t_max;		
	}

private:
	bool doIntersect(
		const Vec3f& orig,
		const Vec3f& dir,
		float& tNear) const
	{
		float t0, t1; // solutions for t if the ray intersects
		// analytic solution
		Vec3f L = orig - m_center;
		float a = dot(dir, dir);
		float b = 2 * dot(dir, L);
		float c = dot(L, L) - m_raduis2;
		if (!solveQuadratic(a, b, c, t0, t1)) return false;

		if (t0 > t1) std::swap(t0, t1);

		if (t0 < 0) {
			t0 = t1; // if t0 is negative, let's use t1 instead
			if (t0 < 0) return false; // both t0 and t1 are negative
		}

		tNear = t0;

		return true;
	}

	// [comment]
	// Compute the roots of a quadratic equation
	// [/comment]
	bool solveQuadratic(const float& a, const float& b, const float& c, float& x0, float& x1) const
	{
		float discr = b * b - 4 * a * c;
		if (discr < 0) return false;
		else if (discr == 0) {
			x0 = x1 = -0.5 * b / a;
		}
		else {
			float q = (b > 0) ?
				-0.5 * (b + sqrt(discr)) :
				-0.5 * (b - sqrt(discr));
			x0 = q / a;
			x1 = c / q;
		}

		return true;
	}


private:
	Vec3f m_center;
	float m_raduis;
	float m_raduis2;
};


class Mesh : public Object
{
public:
	Mesh() = default;
	Mesh(std::vector<Primitive>&& primitives)
		: m_primitives(primitives) {
	}
	Mesh(const std::vector<Primitive>& primitives)
		: m_primitives(primitives) {
	}
	~Mesh() = default;

	bool intersect(const Ray& ray, IntersectInfo& info) const override;

	bool occluded(const Ray& ray, float t_max) const override;

private:
	bool rayTriangleIntersect(
		const Vec3f& orig, const Vec3f& dir,
		const Vec3f& v0, const Vec3f& v1, const Vec3f& v2,
		float& t, float& u, float& v) const;

protected:
	std::vector<Primitive> m_primitives;
};

class SphereMesh : public Mesh {
public:
	SphereMesh(Vec3f center, float radius, int thetaResolution, int phiResolution, Material* mt, AreaLight* light)
		: center_(center), radius_(radius), num_theta_(thetaResolution), num_phi_(phiResolution) {
		Triangulate(mt, light);
	}

private:	
	void Triangulate(Material* mt, AreaLight* light);


private:
	Vec3f center_ = Vec3f(0.0f);
	float radius_ = 1.0f;
	int num_theta_ = 10;	// polar:0~π
	int num_phi_ = 10;		// az:0~2π
};