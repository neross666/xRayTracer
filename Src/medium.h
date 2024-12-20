﻿#pragma once
#include "geometry.h"
#include "sampler.h"
#include "primitive.h"


class PhaseFunction
{
public:
	// evaluate phase function
	virtual float evaluate(const Vec3f& wo, const Vec3f& wi) const = 0;

	// sample incident direction
	// its pdf is propotional to the shape of phase function
	virtual float sampleDirection(const Vec3f& wo, Sampler& sampler,
		Vec3f& wi) const = 0;
};

// https://pbr-book.org/3ed-2018/Volume_Scattering/Phase_Functions#PhaseHG
// human skin:g∈[0.85,0.91]
class HenyeyGreenstein : public PhaseFunction
{
private:
	const float g;

public:
	HenyeyGreenstein(float g) : g(g) {}

	float evaluate(const Vec3f& wo, const Vec3f& wi) const override
	{
		const float cosTheta = dot(wo, wi);
		const float denom = 1 + g * g - 2 * g * cosTheta;
		return PI_MUL_4_INV * (1 - g * g) / (denom * std::sqrt(denom));
	}

	// https://pbr-book.org/3ed-2018/Light_Transport_II_Volume_Rendering/Sampling_Volume_Scattering#SamplingPhaseFunctions
	float sampleDirection(const Vec3f& wo, Sampler& sampler,
		Vec3f& wi) const override
	{
		const Vec2f u = sampler.getNext2D();

		// sample cosTheta
		float cosTheta;
		if (std::abs(g) < 1e-3) {
			// when g is small, sample direction uniformly
			cosTheta = 2 * u[0] - 1.0f;
			//cosTheta = 1.0f - 2 * u[0];
		}
		else {
			const float sqrTerm = (1 - g * g) / (1 - g + 2 * g * u[0]);
			cosTheta = (1 + g * g - sqrTerm * sqrTerm) / (2 * g);
		}

		// compute direction
		const float sinTheta =
			std::sqrt(std::max(1.0f - cosTheta * cosTheta, 0.0f));
		const float phi = 2 * PI * u[1];
		const Vec3f wi_local =
			Vec3f(std::cos(phi) * sinTheta, cosTheta, std::sin(phi) * sinTheta);

		// local to world transform
		Vec3f t, b;
		orthonormalBasis(wo, t, b);
		wi = localToWorld(wi_local, t, wo, b);

		return evaluate(wo, wi);
	}
};


class Medium
{
public:
	Medium(float g) : phaseFunction(std::make_unique<HenyeyGreenstein>(g)) {};
	virtual ~Medium() = default;

	virtual std::unique_ptr<Object> makeObject() = 0;

	// false means there is no collision
	virtual bool sampleMedium(const Ray& ray, IntersectInfo info,
		Sampler& sampler, Vec3f& pos, Vec3f& dir,
		Vec3f& throughput) const = 0;

	virtual Vec3f transmittance(const Vec3f& p1, const Vec3f& p2,
		Sampler& sampler) const = 0;

	Vec3f evalPhaseFunction(const Vec3f& wo, const Vec3f& wi) const
	{
		return phaseFunction->evaluate(wo, wi);
	}

	static Vec3f analyticTransmittance(float t, const Vec3f& sigma)
	{
		return exp(-sigma * t);
	}

	// sample index of wavelength by (throughput * single scattering albedo)
	// Wrenninge, Magnus, Ryusuke Villemin, and Christophe Hery. Path traced
	// subsurface scattering using anisotropic phase functions and
	// non-exponential free flights. Tech. Rep. 17-07, Pixar. https://graphics.
	// pixar. com/library/PathTracedSubsurface, 2017.
	static uint32_t sampleWavelength(const Vec3f& throughput, const Vec3f& albedo,
		Sampler& sampler, Vec3f& pmf)
	{
		// create empirical discrete distribution
		const Vec3f throughput_albedo = throughput * albedo;
		DiscreteEmpiricalDistribution1D distribution(throughput_albedo.getPtr(), 3);
		pmf = Vec3f(distribution.getPDF(0), distribution.getPDF(1),
			distribution.getPDF(2));

		// sample index of wavelength from empirical discrete distribution
		float _pdf;
		const uint32_t channel = distribution.sample(sampler.getNext1D(), _pdf);
		return channel;
	}

protected:
	const std::unique_ptr<PhaseFunction> phaseFunction = nullptr;
};


class HomogeneousMedium : public Medium
{
public:
	HomogeneousMedium(float g, Vec3f a, Vec3f s, AABB box) : Medium(g),
		sigma_a(a), sigma_s(s), sigma_t(a + s), box(box) {}
	~HomogeneousMedium() = default;

	std::unique_ptr<Object> makeObject() override {
		return std::make_unique< BoxMesh >(box, this);
	}

	Vec3f transmittance(const Vec3f& p1, const Vec3f& p2,
		Sampler& sampler) const override
	{
		const float t = length(p1 - p2);
		return analyticTransmittance(t, sigma_t);
	}

protected:
	const Vec3f sigma_a;	// absorption coefficient
	const Vec3f sigma_s;	// scattering coefficient
	const Vec3f sigma_t;	// extinction coefficient
	const AABB box;			// bounding box
};

// with spectral MIS
class HomogeneousMediumMIS : public HomogeneousMedium
{
public:
	HomogeneousMediumMIS(float g, Vec3f a, Vec3f s, AABB box) : HomogeneousMedium(g, a, s, box) {}
	~HomogeneousMediumMIS() = default;

	bool sampleMedium(const Ray& ray, IntersectInfo info,
		Sampler& sampler, Vec3f& pos, Vec3f& dir,
		Vec3f& throughput) const override
	{
		// sample wavelength
		Vec3f pmf_wavelength(1.0f);
		const uint32_t channel = sampleWavelength(
			ray.throughput, (sigma_s / sigma_t), sampler, pmf_wavelength);

		// sample collision-free distance
		const float t = -std::log(std::max(1.0f - sampler.getNext1D(), 0.0f)) /
			sigma_t[channel];

		// hit volume boundary, no collision
		float distToSurface = info.t1 - info.t;
		if (t > distToSurface - RAY_EPS) {
			pos = ray(info.t1 + RAY_EPS);
			dir = ray.direction;

			const Vec3f tr = analyticTransmittance(distToSurface, sigma_t);
			const Vec3f p_surface = tr;
			const Vec3f pdf = pmf_wavelength * p_surface;
			throughput = tr / (pdf[0] + pdf[1] + pdf[2]);
			return false;
		}

		// in-scattering
		// sample direction
		phaseFunction->sampleDirection(ray.direction, sampler, dir);

		pos = ray(info.t + t);
		const Vec3f tr = analyticTransmittance(t, sigma_t);
		const Vec3f pdf_distance = sigma_t * tr;
		const Vec3f pdf = pmf_wavelength * pdf_distance;
		throughput = (tr * sigma_s) / (pdf[0] + pdf[1] + pdf[2]);

		return true;
	}
};

// achromatic version
class HomogeneousMediumAchromatic : public HomogeneousMedium
{
public:
	HomogeneousMediumAchromatic(float g, float a, float s, AABB box) : HomogeneousMedium(g, Vec3f(a), Vec3f(s), box) {}
	~HomogeneousMediumAchromatic() = default;

	bool sampleMedium(const Ray& ray, IntersectInfo info,
		Sampler& sampler, Vec3f& pos, Vec3f& dir,
		Vec3f& throughput) const override
	{

		// sample collision-free distance
		const float t = -std::log(std::max(1.0f - sampler.getNext1D(), 0.0f)) /
			sigma_t[0];

		// hit volume boundary, no collision
		float distToSurface = info.t1 - info.t;
		if (t > distToSurface - RAY_EPS) {
			pos = ray(info.t1 + RAY_EPS);
			dir = ray.direction;

			throughput = Vec3f(1.0f);
			return false;
		}

		// ignore emit event

		// in-scattering event
		// sample direction
		phaseFunction->sampleDirection(ray.direction, sampler, dir);

		pos = ray(info.t + t);
		throughput = Vec3f(sigma_s / sigma_t);

		return true;
	}
};

// no spectral MIS version
class HomogeneousMediumNoMIS : public HomogeneousMedium
{
public:
	HomogeneousMediumNoMIS(float g, Vec3f a, Vec3f s, AABB box) : HomogeneousMedium(g, a, s, box) {}
	~HomogeneousMediumNoMIS() = default;

	bool sampleMedium(const Ray& ray, IntersectInfo info,
		Sampler& sampler, Vec3f& pos, Vec3f& dir,
		Vec3f& throughput) const override
	{
		// sample wavelength
		int channel = 3 * sampler.getNext1D();
		if (channel == 3) channel--;
		const float pmf_wavelength = 1.0f / 3.0f;

		// sample collision-free distance
		const float t = -std::log(std::max(1.0f - sampler.getNext1D(), 0.0f)) /
			sigma_t[channel];
		const float pdf_distance =
			sigma_t[channel] * std::exp(-sigma_t[channel] * t);

		// hit volume boundary, no collision
		float distToSurface = info.t1 - info.t;
		if (t > distToSurface - RAY_EPS) {
			pos = ray(info.t1 + RAY_EPS);
			dir = ray.direction;

			const Vec3f tr = analyticTransmittance(distToSurface, sigma_t);
			const float p_surface = std::exp(-sigma_t[channel] * distToSurface);
			throughput = 1.0f / 3.0f * tr / (pmf_wavelength * p_surface);
			return false;
		}

		// in-scattering
		// sample direction
		phaseFunction->sampleDirection(ray.direction, sampler, dir);

		pos = ray(info.t + t);
		throughput = 1.0f / 3.0f * analyticTransmittance(t, sigma_t) * sigma_s /
			(pmf_wavelength * pdf_distance);

		return true;
	}
};

class DensityGrid;
class HeterogeneousMedium : public Medium
{
private:
	const DensityGrid* densityGridPtr;
	const Vec3f absorptionColor;
	const Vec3f scatteringColor;
	const float densityMultiplier;

	float majorant;
	float invMajorant;

	float getDensity(const Vec3f& p) const;

	float getMaxDensity() const;

	Vec3f getSigma_a(float density) const;

	Vec3f getSigma_s(float density) const;

public:
	HeterogeneousMedium(float g, const DensityGrid* densityGridPtr,
		const Vec3f& absorptionColor,
		const Vec3f& scatteringColor,
		float densityMultiplier = 1.0f);


	std::unique_ptr<Object> makeObject() override;

	// delta tracking
	// ignore emission
	bool sampleMedium(const Ray& ray, IntersectInfo info,
		Sampler& sampler, Vec3f& pos, Vec3f& dir,
		Vec3f& throughput) const override;

	Vec3f transmittance(const Vec3f& p1, const Vec3f& p2,
		Sampler& sampler) const override
	{
		return ratioTrackingTransmittance(p1, p2, sampler);
	}

private:
	Vec3f deltaTrackingTransmittance(const Vec3f& p1, const Vec3f& p2,
		Sampler& sampler) const
	{
		const float distToEnd = length(p1 - p2);

		// sample wavelength
		Vec3f pmf_wavelength;
		const uint32_t channel =
			sampleWavelength(Vec3f(1), Vec3f(1), sampler, pmf_wavelength);

		// loop until collision occurs or exit medium
		float t = 0;
		Ray ray(p1, normalize(p2 - p1));
		Vec3f transmittance(1);
		while (true) {
			// sample collision-free distance
			const float s =
				-std::log(std::max(1.0f - sampler.getNext1D(), 0.0f)) * invMajorant;
			t += s;

			// no collision
			if (t > distToEnd) { return transmittance; }

			// compute russian roulette probability
			const float density = getDensity(ray(t));
			const Vec3f sigma_a = getSigma_a(density);
			const Vec3f sigma_s = getSigma_s(density);
			const Vec3f sigma_n = Vec3f(majorant) - sigma_a - sigma_s;
			const Vec3f P_n = sigma_n * invMajorant;

			// collision
			if (sampler.getNext1D() > P_n[channel]) { return Vec3f(0); }

			transmittance *= (sigma_n / sigma_n[channel]);
		}

		return transmittance;
	}

	Vec3f ratioTrackingTransmittance(const Vec3f& p1, const Vec3f& p2,
		Sampler& sampler) const
	{
		const float distToEnd = length(p1 - p2);

		// loop until collision occurs or exit medium
		float t = 0;
		Ray ray(p1, normalize(p2 - p1));
		Vec3f transmittance(1);
		while (true) {
			// sample collision-free distance
			const float s =
				-std::log(std::max(1.0f - sampler.getNext1D(), 0.0f)) * invMajorant;
			t += s;

			// no collision
			if (t > distToEnd) { break; }

			// ratio tracking
			const float density = getDensity(ray(t));
			const Vec3f sigma_n =
				Vec3f(majorant) - getSigma_a(density) - getSigma_s(density);
			transmittance *= (sigma_n * invMajorant);
		}

		return transmittance;
	}
};