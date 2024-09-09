#pragma once
#include <omp.h>
#include <optional>
#include <spdlog/spdlog.h>

#include "camera.h"
#include "geometry.h"
#include "image.h"
#include "scene.h"
#include "light.h"


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
				//for (uint32_t k = 0; k < 10; ++k) { sampler_per_pixel->getNext1D(); }

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
		Vec3f radiance(0);

		IntersectInfo info;
		if (scene.intersect(ray_in, info)) {	
			//return 0.5f * (info.surfaceInfo.ns + 1.0f);

			/*auto tt = dot(info.surfaceInfo.ns, -ray_in.direction);
			auto faceRatio = std::max(0.f, dot(info.surfaceInfo.ns, -ray_in.direction));
			return Vec3f(faceRatio);*/

			// diffuse
			for (const auto& light : scene.getDeltaLights())
			{
				Vec3f lightDir, lightIntensity;
				IntersectInfo sinfo;
				light->illuminate(info.surfaceInfo.position, lightDir, lightIntensity, sinfo.t);
				bool vis = !scene.intersect(Ray(info.surfaceInfo.position + info.surfaceInfo.ng * 0.1, -lightDir), sinfo);
				// compute the color of a diffuse surface illuminated
				radiance += vis * info.hitObject->albedo() / PI * lightIntensity * std::max(0.f, dot(info.surfaceInfo.ns, -lightDir));
			}
		}

		return radiance;
	};

private:

};

class WhittedIntegrator : public PathIntegrator
{
public:
	WhittedIntegrator(const std::shared_ptr<Camera>& camera, uint32_t maxDepth = 3)
		: PathIntegrator(camera, 1), m_maxDepth(maxDepth) {
	}
	virtual ~WhittedIntegrator() = default;


	Vec3f integrate(const Ray& ray_in, const Scene& scene,
		Sampler& sampler) const override
	{
		Vec3f total_radiance(0);
		Ray ray_root = ray_in;
		ray_root.depth = 0;
		ray_root.throughput = Vec3f(1, 1, 1);

		std::queue<Ray> rays;
		rays.push(ray_root);

		while (!rays.empty())
		{
			Ray ray = rays.front();	rays.pop();
			if (ray.depth > m_maxDepth)
			{
				total_radiance += ray.throughput * Vec3f(0.235294, 0.67451, 0.843137);
				continue;
			}
			
			IntersectInfo info;
			if (scene.intersect(ray, info))
			{
				switch (info.hitObject->material())
				{
				case 0:// diffuse
				{
					Vec3f radiance(0);
					for (const auto& light : scene.getDeltaLights())
					{
						Vec3f lightDir, lightIntensity;
						IntersectInfo sinfo;
						light->illuminate(info.surfaceInfo.position, lightDir, lightIntensity, sinfo.t);
						bool vis = !scene.intersect(Ray(info.surfaceInfo.position + info.surfaceInfo.ng * 0.1, -lightDir), sinfo);
						// compute the color of a diffuse surface illuminated
						radiance += vis * info.hitObject->albedo() / PI * lightIntensity * std::max(0.f, dot(info.surfaceInfo.ns, -lightDir));
					}
					radiance *= ray.throughput;
					total_radiance += radiance;
				}
				break;
				case 1:// mirror
				{
					// reflect direction
					Ray reflect_ray;
					reflect_ray.origin = info.surfaceInfo.position + 0.001 * info.surfaceInfo.ng;
					reflect_ray.direction = reflect(ray.direction, info.surfaceInfo.ng);
					reflect_ray.throughput = 0.8*ray.throughput;
					reflect_ray.depth = ray.depth + 1;
					rays.push(reflect_ray);
				}
				break;
				case 2:// reflect and refract
				{
					// compute fresnel
					float kr;
					fresnel(ray.direction, info.surfaceInfo.ng, info.hitObject->ior(), kr);
					bool outside = dot(ray.direction, info.surfaceInfo.ng) < 0;
					Vec3f bias = /*options.bias*/0.001 * info.surfaceInfo.ng;
					// compute refraction if it is not a case of total internal reflection
					if (kr < 1) {
						Ray refract_ray;
						refract_ray.direction = normalize(refract(ray.direction, info.surfaceInfo.ng, info.hitObject->ior()));
						refract_ray.origin = outside ? info.surfaceInfo.position - bias : info.surfaceInfo.position + bias;
						refract_ray.throughput = 0.9*ray.throughput * (1 - kr);
						refract_ray.depth = ray.depth + 1;

						rays.push(refract_ray);
					}

					Ray reflect_ray;
					reflect_ray.direction = normalize(reflect(ray.direction, info.surfaceInfo.ng));
					reflect_ray.origin = outside ? info.surfaceInfo.position + bias : info.surfaceInfo.position - bias;
					reflect_ray.throughput = 0.9*ray.throughput * kr;
					reflect_ray.depth = ray.depth + 1;	

					rays.push(reflect_ray);
				}
				break;
				default:
					break;
				}
			}
			else
			{
				total_radiance += ray.throughput * Vec3f(0.235294, 0.67451, 0.843137);
			}
		}

		return total_radiance;
	};

private:
	const uint32_t m_maxDepth;

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
			if (!scene.intersect(ray, info))
				break;
				
			// russian roulette
			if (depth > 0) {
				const float russian_roulette_prob = std::min(
					(ray.throughput[0] + ray.throughput[1] + ray.throughput[2]) /
					3.0f,
					1.0f);
				if (sampler.getNext1D() >= russian_roulette_prob) { break; }
				ray.throughput /= russian_roulette_prob;
			}

			// ignore medium for now


			// Le
			//if (info.hitPrimitive->hasAreaLight()) {
			//	radiance += ray.throughput *
			//		info.hitPrimitive->Le(info.surfaceInfo, -ray.direction);
			//	break;
			//}
			
		}


		return radiance;
	}

private:
	const uint32_t m_maxDepth;

};
