#pragma once
#include "geometry.h"
#include "sampler.h"
#include "ray.h"

class Material
{
public:
	Material() = default;
	virtual ~Material() = default;

	virtual MaterialType materialType() const = 0;

	virtual Vec3f evaluateBxDF(const Vec3f& wo, const Vec3f& wi, const SurfaceInfo& sinfo) const = 0;

	virtual Vec3f sampleBxDF(const Vec3f& wo, const SurfaceInfo& sinfo, Sampler& sampler, Vec3f& wi, float& pdf) const = 0;

	virtual Vec3f sampleDir(const SurfaceInfo& sinfo, Sampler& sampler, float& pdf) const = 0;

	// 	float ior() const {
	// 		return 1.3f;
	// 	}

protected:

};

class Lambert : public Material
{
public:
	Lambert(Vec3f albedo) : m_albedo(albedo) {
	};
	~Lambert() = default;

	MaterialType materialType() const override {
		return MaterialType::Lambert;
	}

	Vec3f evaluateBxDF(const Vec3f& wo, const Vec3f& wi, const SurfaceInfo& sinfo) const override {
		// world to local transform
// 		const Vec3f wo_l =
// 			worldToLocal(wo, sinfo.dpdu, sinfo.ng, sinfo.dpdv);
// 		const Vec3f wi_l =
// 			worldToLocal(wi, sinfo.dpdu, sinfo.ng, sinfo.dpdv);
// 		const Vec3f ng_l = Vec3f(0, 1, 0);

		return m_albedo / PI;
	}

	Vec3f sampleBxDF(const Vec3f& wo, const SurfaceInfo& sinfo, Sampler& sampler, Vec3f& wi, float& pdf) const override {
		wi = sampleDir(sinfo, sampler, pdf);
		return evaluateBxDF(wo, wi, sinfo);
	}

	Vec3f sampleDir(const SurfaceInfo& sinfo, Sampler& sampler, float& pdf) const override {
		float r1 = sampler.getNext1D(); // cos(theta) = N.Light Direction
		float r2 = sampler.getNext1D();
		pdf = 1 / (2 * PI);
		Vec3f sample = uniformSampleHemisphere(r1, r2);
		return localToWorld(sample, sinfo.dpdu, sinfo.ng, sinfo.dpdv);
	}

private:
	Vec3f uniformSampleHemisphere(const float& r1, const float& r2) const
	{
		// cos(theta) = u1 = y
		// cos^2(theta) + sin^2(theta) = 1 -> sin(theta) = srtf(1 - cos^2(theta))
		float sinTheta = sqrtf(1 - r1 * r1);
		float phi = 2 * PI * r2;
		float x = sinTheta * cosf(phi);
		float z = sinTheta * sinf(phi);
		return Vec3f(x, r1, z);
	}

private:
	Vec3f m_albedo = Vec3f(0.0f);
};
