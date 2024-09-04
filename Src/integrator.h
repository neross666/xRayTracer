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
	PathIntegrator(const std::shared_ptr<Camera>& camera, uint32_t n_samples, uint32_t n_depth)
		: camera(camera), n_samples(n_samples)
	{
	}

	// compute radiance coming from the given ray
	Vec3f integrate(const Ray& ray_in, const Scene& scene,
		Sampler& sampler) const
	{
 		IntersectInfo info;
 		if (scene.intersect(ray_in, info)) {
 			return 0.5f * (info.surfaceInfo.ns + 1.0f);
			//return Vec3f(1.0, 1.0, 0.0);
 		}
 		else {
 			return Vec3f(0);
 		}
//		return Vec3f(1.0, 1.0, 0.0);
	};

	void render(const Scene& scene, Sampler& sampler, Image& image)
	{
		const uint32_t width = image.getWidth();
		const uint32_t height = image.getHeight();

		spdlog::info("[PathIntegrator] rendering...");
#pragma omp parallel for collapse(2) schedule(dynamic, 1)
		for (int i = 0; i < height; ++i) {
			for (int j = 0; j < width; ++j) {
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

