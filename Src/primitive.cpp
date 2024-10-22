#include "primitive.h"
#include "light.h"
#include "material.h"
#include "Medium.h"


Primitive::Primitive(
	const std::vector<Vec3f>& vertices,
	const std::vector<Vec3f>& normals,
	const std::vector<Vec2f>& texcoords)
	: m_vertices(vertices), m_normals(normals), m_texcoords(texcoords)
{

}

Primitive::Primitive(
	std::vector<Vec3f>&& vertices,
	std::vector<Vec3f>&& normals,
	std::vector<Vec2f>&& texcoords) :
	m_vertices(vertices), m_normals(normals), m_texcoords(texcoords)
{

}

MaterialType Object::materialType() const
{
	if (m_material == nullptr)
		return MaterialType::Unknow;
	return m_material->materialType();
}

Vec3f Object::evaluateBxDF(const Vec3f& wo, const Vec3f& wi, const SurfaceInfo& sinfo) const
{
	if (m_material == nullptr)
		return Vec3f(0.0f);
	return m_material->evaluateBxDF(wo, wi, sinfo);
}

Vec3f Object::sampleBxDF(const Vec3f& wo, const SurfaceInfo& sinfo, Sampler& sampler, Vec3f& wi, float& pdf) const
{
	if (m_material == nullptr)
		return Vec3f(0.0f);
	return m_material->sampleBxDF(wo, sinfo, sampler, wi, pdf);
}


Vec3f Object::sampleDir(const SurfaceInfo& sinfo, Sampler& sampler, float& pdf) const
{
	if (m_material == nullptr)
		return Vec3f(0.0f);
	return m_material->sampleDir(sinfo, sampler, pdf);
}

Vec3f Object::Le(const SurfaceInfo& info, const Vec3f& wi) const
{
	if (m_areaLight == nullptr)
		return Vec3f(0.0f);

	return m_areaLight->Le(info.ns, wi);
}

bool Object::sampleMedium(const Ray& ray, IntersectInfo info, Sampler& sampler, Vec3f& pos, Vec3f& dir, Vec3f& throughput) const
{
	if (m_medium == nullptr)
		return false;
	return m_medium->sampleMedium(ray, info, sampler, pos, dir, throughput);
}

Vec3f Object::sampleTransparency(const Vec3f& p1, const Vec3f& p2, Sampler& sampler) const
{
	if (m_medium == nullptr)
		return Vec3f(0.0f);
	return m_medium->transmittance(p1, p2, sampler);
}

Vec3f Object::evalPhaseFunction(const Vec3f& wo, const Vec3f& wi) const
{
	if (m_medium == nullptr)
		return Vec3f(0.0f);
	return m_medium->evalPhaseFunction(wo, wi);
}

bool Mesh::intersect(const Ray& ray, IntersectInfo& info) const
{
	bool isIntersect = false;
	for (auto& primitive : m_primitives)
	{
		const auto vertices = primitive.vertices();
		const auto normals = primitive.normals();
		const auto texcoords = primitive.texcoords();
		float t = 0.0f;
		float u = 0.0f;
		float v = 0.0f;
		bool rst = rayTriangleIntersect(ray.origin, ray.direction,
			vertices[0], vertices[1], vertices[2],
			t, u, v);
		if (rst)
		{
			isIntersect = true;
			if (t < info.t)
			{
				info.t = t;
				info.surfaceInfo.position = ray(t);
				info.surfaceInfo.barycentric = Vec2f(u, v);
				info.surfaceInfo.texcoords = texcoords[0] * (1.0f - u - v) + texcoords[1] * u + texcoords[2] * v;
				info.surfaceInfo.ng = normalize(cross(vertices[1] - vertices[0], vertices[2] - vertices[0]));
				info.surfaceInfo.ns = normals[0] * (1.0f - u - v) + normals[1] * u + normals[2] * v;
				orthonormalBasis(info.surfaceInfo.ns, info.surfaceInfo.dpdu,
					info.surfaceInfo.dpdv);
				info.hitObject = this;
			}
		}
	}

	return isIntersect;
}

bool Mesh::occluded(const Ray& ray, float t_max) const
{
	for (auto& primitive : m_primitives)
	{
		// 		if (primitive.hasAreaLight())
		// 			continue;

		const auto vertices = primitive.vertices();
		const auto normals = primitive.normals();
		const auto texcoords = primitive.texcoords();
		float t = 0.0f;
		float u = 0.0f;
		float v = 0.0f;
		bool rst = rayTriangleIntersect(ray.origin, ray.direction,
			vertices[0], vertices[1], vertices[2],
			t, u, v);
		if (rst && t < t_max)
			return true;
	}
	return false;
}

bool Mesh::rayTriangleIntersect(const Vec3f& orig, const Vec3f& dir, const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, float& t, float& u, float& v) const
{
	Vec3f v0v1 = v1 - v0;
	Vec3f v0v2 = v2 - v0;
	Vec3f pvec = cross(dir, v0v2);
	float det = dot(v0v1, pvec);

#ifdef CULLING  // When refraction is taken into account, opening the face culling will result in light not passing through the glass material
	// if the determinant is negative the triangle is backfacing
	// if the determinant is close to 0, the ray misses the triangle
	if (det < kEpsilon) return false;
#else
	// ray and triangle are parallel if det is close to 0
	if (fabs(det) < kEpsilon) return false;
#endif
	float invDet = 1 / det;

	Vec3f tvec = orig - v0;
	u = dot(tvec, pvec) * invDet;
	if (u < 0 || u > 1) return false;

	Vec3f qvec = cross(tvec, v0v1);
	v = dot(dir, qvec) * invDet;
	if (v < 0 || u + v > 1) return false;

	t = dot(v0v2, qvec) * invDet;

	return t > kEpsilon;
}

void SphereMesh::Triangulate()
{
	std::vector<Vec3f> vertices;
	std::vector<Vec3f> normals;

	// vertices
	for (int i = 0; i <= num_theta_; ++i) {
		float theta = PI * i / num_theta_;
		for (int j = 0; j <= num_phi_; ++j) {
			float phi = 2 * PI * j / num_phi_;

			Vec3f vertex = { sin(theta) * sin(phi), cos(theta), sin(theta) * cos(phi) };
			vertices.push_back(center_ + radius_ * vertex);
			normals.push_back(vertex);
		}
	}

	// triangle
	for (int i = 0; i < num_theta_; ++i) {
		for (int j = 0; j < num_phi_; ++j) {
			int first = (i * (num_phi_ + 1)) + j;
			int second = first + num_phi_ + 1;
			int first1 = first + 1;
			int second1 = second + 1;

			m_primitives.push_back(Primitive(
				{ vertices[first],vertices[second],vertices[first1] },
				{ normals[first],normals[second],normals[first1] },
				{ Vec2f(0,0),Vec2f(1,0),Vec2f(0,1) }));
			m_primitives.push_back(Primitive(
				{ vertices[second],vertices[second1],vertices[first1] },
				{ normals[second],normals[second1],normals[first1] },
				{ Vec2f(0,0),Vec2f(1,0),Vec2f(0,1) }));
		}
	}
}
