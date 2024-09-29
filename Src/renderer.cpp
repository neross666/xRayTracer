#include "renderer.h"
#include "integrator.h"
#include "camera.h"
#include <spdlog/spdlog.h>
#include <execution>


void NormalRenderer::render(const Scene& scene, Sampler& sampler, Image& image) const
{
	const uint32_t width = image.getWidth();
	const uint32_t height = image.getHeight();

	spdlog::info("[PathIntegrator] rendering...");
	for (uint32_t i = 0; i < height; ++i)
	{
		for (uint32_t j = 0; j < width; ++j)
		{
			// init sampler for each pixel
			const std::unique_ptr<Sampler> sampler_per_pixel = sampler.clone();
			sampler_per_pixel->setSeed(j + width * i);

			// warmup sampler
			//sampler_per_pixel->discard(2);

			// iteration
			for (uint32_t k = 0; k < n_samples; ++k) {
				// SSAA
				const float u =
					(j + sampler_per_pixel->getNext1D()) / width;
				const float v =
					(i + sampler_per_pixel->getNext1D()) / height;

				Ray ray;
				float pdf;
				if (camera->sampleRay(Vec2f(u, v), *sampler_per_pixel, ray, pdf)) {
					// compute incoming radiance
					const Vec3f radiance =
						integrator->integrate(ray, scene, *sampler_per_pixel) / pdf;

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
		std::cout << "\rprogress: " << i << "/" << height;
	}
	spdlog::info("");
	spdlog::info("[PathIntegrator] done");

	// take average
	image /= Vec3f(n_samples);
}

void ParallelRenderer::render(const Scene& scene, Sampler& sampler, Image& image) const
{
	auto indices = image.getImageIdx();
	const uint32_t width = image.getWidth();
	const uint32_t height = image.getHeight();

	spdlog::info("[PathIntegrator] rendering...");
	std::for_each(std::execution::par_unseq, indices.begin(), indices.end(), [&](Image::ImageIdx idx) {
		int i = idx.i;
		int j = idx.j;

		// init sampler for each pixel
		const std::unique_ptr<Sampler> sampler_per_pixel = sampler.clone();
		sampler_per_pixel->setSeed(j + width * i);

		// warmup sampler
		//sampler_per_pixel->discard(2);

		// iteration
		for (uint32_t k = 0; k < n_samples; ++k) {
			// SSAA
			const float u =
				(j + sampler_per_pixel->getNext1D()) / width;
			const float v =
				(i + sampler_per_pixel->getNext1D()) / height;

			Ray ray;
			float pdf;
			if (camera->sampleRay(Vec2f(u, v), *sampler_per_pixel, ray, pdf)) {
				// compute incoming radiance
				const Vec3f radiance =
					integrator->integrate(ray, scene, *sampler_per_pixel) / pdf;

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
	);

	spdlog::info("[PathIntegrator] done");

	// take average
	image /= Vec3f(n_samples);
}
