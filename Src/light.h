#pragma once
#include "geometry.h"
#include "ray.h"
#include "sampler.h"
#include "primitive.h"


// [comment]
// Light base class
// [/comment]
class DeltaLight
{
public:
	DeltaLight(const Matrix44f& l2w, const Vec3f& c = 1, const float& i = 1) : lightToWorld(l2w), color(c), intensity(i) {}
	virtual ~DeltaLight() {}

	virtual Vec3<float> sample(const IntersectInfo& info, Vec3<float>& wi, float& pdf, float& t_max) const = 0;

	Vec3f color;
	float intensity;
	Matrix44f lightToWorld;
};

// [comment]
// Distant light
// the default direction is (0,0,-1)
// [/comment]
class DistantLight : public DeltaLight
{
	Vec3f dir;
public:
	DistantLight(const Matrix44f& l2w, const Vec3f& c = 1, const float& i = 1);
	Vec3f direction() const { return dir; }

	Vec3<float> sample(const IntersectInfo& info, Vec3<float>& wi, float& pdf, float& t_max) const override;
};

// [comment]
// Point light
// the default positon is (0,0,0)
// [/comment]
class PointLight : public DeltaLight
{
	Vec3f pos;
public:
	PointLight(const Matrix44f& l2w, const Vec3f& c = 1, const float& i = 100.0);

	Vec3<float> sample(const IntersectInfo& info, Vec3<float>& wi, float& pdf, float& t_max) const override;
};


// treat light as emit-light materail?
class Object;
class AreaLight/* : public Material*/
{
public:
	AreaLight(const Matrix44f& l2w, const Vec3f& Le) : lightToWorld(l2w), Le_(Le) {
	}

	virtual Vec3f sample(const Vec3f& pos, Vec3<float>& wi, float& pdf, float& t_max, Sampler& sample) const = 0;

	virtual Vec3f Le(const Vec3f& ns, const Vec3f& wi) const {
		if (dot(wi, ns) < 0) {
			return Le_;
		}		
		else {	// backface
			return Vec3f(0);
		}
	}

	virtual std::unique_ptr<Object> makeObject() = 0;

protected:
	Vec3f Le_;
	Matrix44f lightToWorld;
};


class TriangleLight : public AreaLight
{
public:
	TriangleLight(const Primitive& primitive, const Vec3f& Le);
	TriangleLight(const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, const Matrix44f& l2w, const Vec3f& Le);

	Vec3f sample(const Vec3f& position, Vec3<float>& wi, float& pdf, float& tmax, Sampler& sample) const override;

	virtual std::unique_ptr<Object> makeObject() override;

private:
	Vec3f uniformSampleTriangle(const float& u, const float& v, const Vec3f& A, const Vec3f& B, const Vec3f& C) const;

private:
	Vec3f v0_;
	Vec3f v1_;
	Vec3f v2_;

private:
	Vec3f e1_;
	Vec3f e2_;
	Vec3f Ng_;
};


class QuadLight : public AreaLight
{
public:
	QuadLight(const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, const Matrix44f& l2w, const Vec3f& Le);

	Vec3f sample(const Vec3f& position, Vec3<float>& wi, float& pdf, float& tmax, Sampler& sample) const override;

	virtual std::unique_ptr<Object> makeObject() override;
private:
	Vec3f v0_;
	Vec3f v1_;
	Vec3f v2_;

private:
	Vec3f e1_;
	Vec3f e2_;
	Vec3f Ng_;
};


class SphereLight : public AreaLight
{
public:
	SphereLight(const Vec3f& center, float raduis, const Matrix44f& l2w, const Vec3f& Le);

	Vec3f sample(const Vec3f& position, Vec3<float>& wi, float& pdf, float& tmax, Sampler& sample) const override
	{
#ifdef AREA_SAMPLING
		Vec3f n = UniformSampleSphere(sample.getNext1D(), sample.getNext1D());
		Vec3f p = center_ + n * radius_;
		Vec3f d = p - position;
		tmax = length(d);
#elif defined(INTERSECT_METHOD)
		Vec3f dz = center_ - position;
		float dz_len_2 = dot(dz, dz);
		float dz_len = std::sqrtf(dz_len_2);
		dz /= dz_len;
		Vec3f dx, dy;

		orthonormalBasis(dz, dy, dx);

		float sin_theta_max_2 = radius_ * radius_ / dz_len_2;
		float cos_theta_max = std::sqrt(std::max(0.f, 1.f - sin_theta_max_2));
		Vec3f sample_dir = UniformSampleCone(sample.getNext1D(), sample.getNext1D(), cos_theta_max, dx, dy, dz);

		IntersectInfo sinfo;
		if (!lightObject->intersect(Ray(position, sample_dir), sinfo)) {
			tmax = dot(center_ - position, sample_dir);
		}
		Vec3f p = position + sample_dir * tmax;
		Vec3f d = p - position;
		if (length(p - center_) < radius_)  return Vec3f(0.0f); // check for x inside the sphere
		Vec3f n = normalize(p - center_);
#else
		Vec3f dz = center_ - position;
		float dz_len_2 = dot(dz, dz);
		float dz_len = std::sqrtf(dz_len_2);
		dz /= -dz_len;
		Vec3f dx, dy;

		orthonormalBasis(dz, dx, dy);

		float sin_theta_max_2 = radius_ * radius_ / dz_len_2;
		float sin_theta_max = std::sqrt(sin_theta_max_2);
		float cos_theta_max = std::sqrt(std::max(0.f, 1.f - sin_theta_max_2));

		float cos_theta = 1 + (cos_theta_max - 1) * sample.getNext1D();
		float sin_theta_2 = 1.f - cos_theta * cos_theta;

		float cos_alpha = sin_theta_2 / sin_theta_max + cos_theta * std::sqrt(1 - sin_theta_2 / (sin_theta_max * sin_theta_max));
		float sin_alpha = std::sqrt(1 - cos_alpha * cos_alpha);
		float phi = 2 * PI * sample.getNext1D();

		Vec3f n = std::cos(phi) * sin_alpha * dx + std::sin(phi) * sin_alpha * dy + cos_alpha * dz;
		Vec3f p = center_ + n * radius_;

		Vec3f d = p - position;
		tmax = length(d);
#endif

		float d_dot_n = dot(d, n);
		if (d_dot_n >= 0) return 0;

#if AREA_SAMPLING
		wi = d / tmax;
		pdf = (2.f * tmax * tmax * tmax) / std::abs(d_dot_n);
#else
		pdf = 1.f / (2.f * PI * (1.f - cos_theta_max));
		wi = d / tmax;
#endif
		return Le_;
	}

	virtual std::unique_ptr<Object> makeObject() override;

private:
	Vec3f UniformSampleSphere(const float& r1, const float& r2) const;

	Vec3f UniformSampleCone(float r1, float r2, float cos_theta_max, const Vec3f& x, const Vec3f& y, const Vec3f& z) const;

private:
	Vec3f center_;
	float radius_;
};