#include "light.h"
#include "primitive.h"
#include <spdlog/spdlog.h>


TriangleLight::TriangleLight(const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, const Matrix44f& l2w, const Vec3f& Le) : AreaLight(l2w, Le)
, v0_(multVecMatrix(v0, l2w))
, v1_(multVecMatrix(v1, l2w))
, v2_(multVecMatrix(v2, l2w))
, e1_(v1_ - v0_)
, e2_(v2_ - v0_)
, Ng_(cross(e1_, e2_))
{
}

TriangleLight::TriangleLight(const Primitive& primitive, const Vec3f& Le) : AreaLight(Matrix44f(), Le)
{
	std::vector<Primitive> primitives{ primitive };
}

Vec3f TriangleLight::sample(const IntersectInfo& info, Vec3<float>& wi, float& pdf, float& tmax, Sampler& sample) const
{
	Vec3f d = uniformSampleTriangle(sample.getNext1D(), sample.getNext1D(), v0_, v1_, v2_) - info.surfaceInfo.position;
	tmax = length(d);
	float d_dot_Ng = dot(d, Ng_);
	if (d_dot_Ng >= 0) return 0;
	wi = d / tmax;
	pdf = (2.f * tmax * tmax * tmax) / std::abs(d_dot_Ng);
	return Le_;
}

std::unique_ptr<Object> TriangleLight::makeObject()
{
	std::vector<Vec3f> vertices{ v0_, v1_, v2_ };
	auto n = normalize(Ng_);
	std::vector<Vec3f> normals{ n,n,n };
	std::vector<Vec2f> texcoords{ Vec2f(0, 0), Vec2f(1, 0),Vec2f(0, 1) };
	std::vector<Primitive> primitives{ Primitive(vertices, normals, texcoords) };

	return std::make_unique< Mesh >(primitives, nullptr, this);
}

Vec3f TriangleLight::uniformSampleTriangle(const float& u, const float& v, const Vec3f& A, const Vec3f& B, const Vec3f& C) const
{
	float su = std::sqrt(u);
	return Vec3f(C + (1.f - su) * (A - C) + (v * su) * (B - C));
}

QuadLight::QuadLight(const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, const Matrix44f& l2w, const Vec3f& Le) : AreaLight(l2w, Le)
, v0_(multVecMatrix(v0, l2w))
, v1_(multVecMatrix(v1, l2w))
, v2_(multVecMatrix(v2, l2w))
, e1_(v1_ - v0_)
, e2_(v2_ - v0_)
, Ng_(cross(e1_, e2_))
{
}

Vec3f QuadLight::sample(const IntersectInfo& info, Vec3<float>& wi, float& pdf, float& tmax, Sampler& sample) const
{
	Vec3f d = (v0_ + e1_ * sample.getNext1D() + e2_ * sample.getNext1D()) - info.surfaceInfo.position;
	tmax = length(d);
	float d_dot_Ng = dot(d, Ng_);
	if (d_dot_Ng >= 0) return 0;
	wi = d / tmax;
	pdf = (tmax * tmax * tmax) / std::abs(d_dot_Ng);
	return Le_;
}

std::unique_ptr<Object> QuadLight::makeObject()
{
	Vec3f v3_ = v0_ + e1_ + e2_;
	auto n = normalize(Ng_);
	std::vector<Vec3f> normals{ n,n,n };
	std::vector<Vec2f> texcoords{ Vec2f(0, 0), Vec2f(1, 0),Vec2f(0, 1) };
	std::vector<Primitive> primitives{
		Primitive({ v0_, v1_, v2_ }, normals, texcoords),
		Primitive({ v1_, v3_, v2_ }, normals, texcoords)
	};

	return std::make_unique< Mesh >(primitives, nullptr, this);
}

SphereLight::SphereLight(const Vec3f& center, float raduis, const Matrix44f& l2w, const Vec3f& Le) : AreaLight(l2w, Le)
, center_(multVecMatrix(center, l2w))
, radius_(raduis)
{
	spdlog::info("[SphereLight] center: ({}, {}, {})", center_[0], center_[1], center_[2]);
	spdlog::info("[SphereLight] raduis: {}", radius_);
	spdlog::info("[SphereLight] le: ({}, {}, {})", Le[0], Le[1], Le[2]);
}

std::unique_ptr<Object> SphereLight::makeObject()
{
	return std::make_unique < SphereMesh >(center_, radius_, 5, 5, nullptr, this);
}

Vec3f SphereLight::UniformSampleSphere(const float& r1, const float& r2) const
{
	float z = 1.f - 2.f * r1; // cos(theta)
	float sin_theta = std::sqrt(1 - z * z);
	float phi = 2 * PI * r2; // azimuthal angle
	return { std::cos(phi) * sin_theta, std::sin(phi) * sin_theta, z };
}

Vec3f SphereLight::UniformSampleCone(float r1, float r2, float cos_theta_max, const Vec3f& x, const Vec3f& y, const Vec3f& z) const
{
	float cos_theta = (1.f - r1) + r1 * cos_theta_max;//  std::lerp(cos_theta_max, 1.f, r1); // need c++20
	float sin_theta = std::sqrt(1.f - cos_theta * cos_theta);
	float phi = 2 * PI * r2;
	return std::cos(phi) * sin_theta * x + std::sin(phi) * sin_theta * y + cos_theta * z;
}

PointLight::PointLight(const Matrix44f& l2w, const Vec3f& c /*= 1*/, const float& i /*= 100.0*/) : DeltaLight(l2w, c, i)
{
	l2w.multVecMatrix(Vec3f(0), pos);
}

Vec3<float> PointLight::sample(const IntersectInfo& info, Vec3<float>& wi, float& pdf, float& t_max) const
{
	auto lightDir = pos - info.surfaceInfo.position;        // Note the very close proximity of point light to point hitPoint
	float distance = length(lightDir);
	wi = lightDir / distance;
	pdf = distance * distance;  // why not 4*PI*distance*distance?
	t_max = distance;
	return color * intensity;
}

DistantLight::DistantLight(const Matrix44f& l2w, const Vec3f& c /*= 1*/, const float& i /*= 1*/) : DeltaLight(l2w, c, i)
{
	l2w.multDirMatrix(Vec3f(0, 0, -1), dir);
	dir = normalize(dir); // in case the matrix scales the light
}

Vec3<float> DistantLight::sample(const IntersectInfo& info, Vec3<float>& wi, float& pdf, float& t_max) const
{
	wi = -dir;
	pdf = 1.0f;
	t_max = kInfinity;
	return color * intensity;
}
