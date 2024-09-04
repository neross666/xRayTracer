#include <omp.h>
#include <optional>

#include "camera.h"
#include "geometry.h"
#include "image.h"
#include "scene.h"


class PathIntegrator
{
private:
	// number of samples in each pixel
	const uint32_t n_samples;
	const std::shared_ptr<Camera> camera;

public:
	PathIntegrator(const std::shared_ptr<Camera>& camera, uint32_t n_samples)
		: camera(camera), n_samples(n_samples)
	{
	}

	// compute radiance coming from the given ray
	virtual Vec3f integrate(const Ray& ray, const Scene& scene,
		Sampler& sampler) const = 0;

	void render(const Scene& scene, Sampler& sampler, Image& image)
	{
		const uint32_t width = image.getWidth();
		const uint32_t height = image.getHeight();

		spdlog::info("[PathIntegrator] rendering...");
#pragma omp parallel for collapse(2) schedule(dynamic, 1)
		for (uint32_t i = 0; i < height; ++i) {
			for (uint32_t j = 0; j < width; ++j) {
				// init sampler for each pixel
				const std::unique_ptr<Sampler> sampler_per_pixel = sampler.clone();
				sampler_per_pixel->setSeed((sampler.getSeed() + 1) * (j + width * i));

				// warmup sampler
				for (uint32_t k = 0; k < 10; ++k) { sampler_per_pixel->getNext1D(); }

				// iteration
				for (uint32_t k = 0; k < n_samples; ++k) {
					// SSAA
					const float u =
						(2.0f * (j + sampler_per_pixel->getNext1D()) - width) / height;
					const float v =
						(height - 2.0f * (i + sampler_per_pixel->getNext1D())) / height;

					Ray ray;
					float pdf;
					if (camera->sampleRay(Vec2f(u, v), *sampler_per_pixel, ray, pdf)) {
						// compute incoming radiance
						const Vec3f radiance =
							integrate(ray, scene, *sampler_per_pixel) / pdf;

						// invalid radiance check
						if (std::isnan(radiance[0]) || std::isnan(radiance[1]) ||
							std::isnan(radiance[2])) {
							spdlog::error("[PathIntegrator] radiance is NaN: ({}, {}, {})",
								radiance[0], radiance[1], radiance[2]);
							continue;
						}
						else if (std::isinf(radiance[0]) || std::isinf(radiance[1]) ||
							std::isinf(radiance[2])) {
							spdlog::error("[PathIntegrator] radiance is inf: ({}, {}, {})",
								radiance[0], radiance[1], radiance[2]);
							continue;
						}
						else if (radiance[0] < 0 || radiance[1] < 0 || radiance[2] < 0) {
							spdlog::error("[PathIntegrator] radiance is minus: ({}, {}, {})",
								radiance[0], radiance[1], radiance[2]);
							continue;
						}

						image.addPixel(i, j, radiance);
					}
					else {
						image.setPixel(i, j, Vec3f(0));
					}
				}
			}
		}
		spdlog::info("[PathIntegrator] done");

		// take average
		image /= Vec3f(n_samples);
	}
};

class NormalIntegrator : public PathIntegrator
{
public:
	NormalIntegrator(const std::shared_ptr<Camera>& camera)
		: PathIntegrator(camera, 1) {
	}
	virtual ~NormalIntegrator() = default;


	Vec3f integrate(const Ray& ray_in, const Scene& scene,
		Sampler& sampler) const override
	{
		IntersectInfo info;
		if (scene.intersect(ray_in, info)) {
			return 0.5f * (info.surfaceInfo.ns + 1.0f);
		}
		else {
			return Vec3f(0);
		}
	};

private:

};

class PathTracing : public PathIntegrator
{
public:
	PathTracing(const std::shared_ptr<Camera>& camera, uint32_t n_samples,
	uint32_t maxDepth = 100)
	: PathIntegrator(camera, n_samples), m_maxDepth(maxDepth){
	}
	virtual ~PathTracing() = default;

	Vec3f integrate(const Ray& ray_in, const Scene& scene,
		Sampler& sampler) const override
	{
		Vec3f radiance(0);
		Ray ray = ray_in;
		ray.throughput = Vec3f(1, 1, 1);

		uint32_t depth = 0;
		while (depth < m_maxDepth) 
		{
			IntersectInfo info;
			if (scene.intersect(ray, info))
			{
				// russian roulette
				if (depth > 0) {
					const float russian_roulette_prob = std::min(
						(ray.throughput[0] + ray.throughput[1] + ray.throughput[2]) /
						3.0f,
						1.0f);
					if (sampler.getNext1D() >= russian_roulette_prob) { break; }
					ray.throughput /= russian_roulette_prob;
				}

				// ignore medium
			}
		}


		return radiance;
	}

private:
	const uint32_t m_maxDepth;

};
