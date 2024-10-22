#include "medium.h"
#include "grid.h"


HeterogeneousMedium::HeterogeneousMedium(float g, const DensityGrid* densityGridPtr, const Vec3f& absorptionColor, const Vec3f& scatteringColor, float densityMultiplier /*= 1.0f*/) : Medium(g),
densityGridPtr(densityGridPtr),
absorptionColor(absorptionColor),
scatteringColor(scatteringColor),
densityMultiplier(densityMultiplier)
{
	// compute wavelength independent majorant
	const float max_density = getMaxDensity();
	const Vec3f m =
		absorptionColor * max_density + scatteringColor * max_density;
	majorant = std::max(m[0], std::max(m[1], m[2]));
	invMajorant = 1.0f / majorant;
}


std::unique_ptr<Object> HeterogeneousMedium::makeObject()
{
	return std::make_unique< BoxMesh >(densityGridPtr->getBounds(), this);
}

float HeterogeneousMedium::getDensity(const Vec3f& p) const
{
	return this->densityMultiplier * this->densityGridPtr->getDensity(p);
}

float HeterogeneousMedium::getMaxDensity() const
{
	return this->densityMultiplier * this->densityGridPtr->getMaxDensity();
}

Vec3f HeterogeneousMedium::getSigma_a(float density) const
{
	return this->absorptionColor * density;
}

Vec3f HeterogeneousMedium::getSigma_s(float density) const
{
	return this->scatteringColor * density;
}

bool HeterogeneousMedium::sampleMedium(const Ray& ray, IntersectInfo info, Sampler& sampler, Vec3f& pos, Vec3f& dir, Vec3f& throughput) const  /* bool sampleMedium(const Ray& ray, float distToSurface, Sampler& sampler, */ // Vec3f& pos, Vec3f& dir, Vec3f& throughput) const 
{
	// loop until in-scattering or exiting medium happens
	Vec3f throughput_tracking(1, 1, 1);
	float t = info.t;

	// init sigma_a
	// NOTE: for computing throughput_albedo
	const float density = getDensity(ray(t));
	Vec3f sigma_a = absorptionColor * density;

	while (true) {
		// sample wavelength
		Vec3f pmf_wavelength;
		const uint32_t channel = sampleWavelength(
			ray.throughput * throughput_tracking,
			(Vec3f(majorant) - sigma_a) * invMajorant, sampler, pmf_wavelength);

		// sample collision-free distance
		const float s =
			-std::log(std::max(1.0f - sampler.getNext1D(), 0.0f)) * invMajorant;
		t += s;

		// hit volume boundary, no collision
		float distToSurface = info.t1 - info.t;
		if (t > info.t1 - RAY_EPS) {
			pos = ray(info.t1 + RAY_EPS);
			dir = ray.direction;

			const float dist_to_surface_from_current_pos =
				s - (t - (info.t1 - RAY_EPS));
			const Vec3f tr = analyticTransmittance(dist_to_surface_from_current_pos,
				Vec3f(majorant));
			const Vec3f p_surface = tr;
			const Vec3f pdf = pmf_wavelength * p_surface;
			throughput_tracking *= tr / (pdf[0] + pdf[1] + pdf[2]);

			// nan check
			if (std::isnan(throughput_tracking[0]) ||
				std::isnan(throughput_tracking[1]) ||
				std::isnan(throughput_tracking[2])) {
				throughput = Vec3f(0);
			}
			else {
				throughput = throughput_tracking;
			}

			return false;
		}

		// compute russian roulette probability
		const float density = getDensity(ray(t));
		const Vec3f sigma_s = getSigma_s(density);
		sigma_a = getSigma_a(density);
		const Vec3f sigma_n = Vec3f(majorant) - sigma_a - sigma_s;
		const Vec3f P_s = sigma_s / (sigma_s + sigma_n);
		const Vec3f P_n = sigma_n / (sigma_s + sigma_n);

		// In-Scattering
		if (sampler.getNext1D() < P_s[channel]) {
			pos = ray(t);
			phaseFunction->sampleDirection(ray.direction, sampler, dir);

			const Vec3f tr = analyticTransmittance(s, Vec3f(majorant));
			const Vec3f pdf_distance = majorant * tr;
			const Vec3f pdf = pmf_wavelength * pdf_distance * P_s;
			throughput_tracking *= (tr * sigma_s) / (pdf[0] + pdf[1] + pdf[2]);

			// nan check
			if (std::isnan(throughput_tracking[0]) ||
				std::isnan(throughput_tracking[1]) ||
				std::isnan(throughput_tracking[2])) {
				throughput = Vec3f(0);
			}
			else {
				throughput = throughput_tracking;
			}

			return true;
		}

		// Null-Scattering
		// update throughput
		const Vec3f tr = analyticTransmittance(s, Vec3f(majorant));
		const Vec3f pdf_distance = majorant * tr;
		const Vec3f pdf = pmf_wavelength * pdf_distance * P_n;
		throughput_tracking *= (tr * sigma_n) / (pdf[0] + pdf[1] + pdf[2]);
	}
}
