#pragma once
#include <spdlog/spdlog.h>

#include "scene.h"
#include "light.h"
#include "geometry.h"


class Integrator
{
public:
	Integrator() = default;
	virtual ~Integrator() = default;

	// compute radiance coming from the given ray
	virtual Vec3f integrate(const Ray& ray, const Scene& scene,
		Sampler& sampler) const = 0;

};


class NormalIntegrator : public Integrator
{
public:
	NormalIntegrator() = default;
	virtual ~NormalIntegrator() = default;


	Vec3f integrate(const Ray& ray_in, const Scene& scene,
		Sampler& sampler) const override
	{
		Vec3f radiance(0);

		IntersectInfo info;
		if (scene.intersect(ray_in, info)) {
			return 0.5f * (info.surfaceInfo.ns + 1.0f);

			/*auto tt = dot(info.surfaceInfo.ns, -ray_in.direction);
			auto faceRatio = std::max(0.f, dot(info.surfaceInfo.ns, -ray_in.direction));
			return Vec3f(faceRatio);*/

			// diffuse,light sample
			/*for (const auto& light : scene.getDeltaLights())
			{
				Vec3f wi;
				float pdf = 1.0;
				float tmax = 0.0;
				Vec3f light_L = light->sample(info, wi, pdf, tmax);

				float bias = 0.01;
				bool vis = !scene.occluded(Ray(info.surfaceInfo.position + info.surfaceInfo.ng * bias, wi), tmax - bias);

				if (pdf == 0.0f)
					continue;

				radiance += vis * info.hitPrimitive->evaluate() * light_L * std::max(0.f, dot(info.surfaceInfo.ns, wi)) / pdf;
			}*/

			// Furnace Test
			float pdf = 1.0f;
			Vec3f nextDir(0.0f);
			auto fr = info.hitObject->sampleBxDF(ray_in.direction, info.surfaceInfo, sampler, nextDir, pdf);
			float cos = std::max(0.0f, dot(nextDir, info.surfaceInfo.ng));
			Vec3f Li(1.0f);
			// indirectDiffuse*fr*albedo/N/pdf
			radiance = fr * cos * Li / pdf;
		}

		return radiance;
	};

private:

};

class DirectIntegrator : public Integrator
{
public:
	DirectIntegrator() = default;
	virtual ~DirectIntegrator() = default;

	Vec3f integrate(const Ray& ray_in, const Scene& scene,
		Sampler& sampler) const override
	{
		Vec3f radiance(0.0f);
		IntersectInfo info;
		if (scene.intersect(ray_in, info))
		{
			// Light
			if (info.hitObject->hasAreaLight()) {
				return info.hitObject->Le(info.surfaceInfo, ray_in.direction);
			}

			// Object
			for (const auto& light : scene.getAreaLights())
			{
				Vec3f wi;
				float tmax, pdf = 0.0f;
				Vec3f L = light->sample(info.surfaceInfo.position, wi, pdf, tmax, sampler);

				if (pdf == 0)
					continue;

				auto bias = 0.01f;
				bool vis = !scene.occluded(Ray(info.surfaceInfo.position + info.surfaceInfo.ng * bias, wi), tmax - bias);
				float cos = std::max(0.0f, dot(info.surfaceInfo.ng, wi));
				Vec3f fr = info.hitObject->evaluateBxDF(ray_in.direction, wi, info.surfaceInfo);
				radiance += vis * fr * L * cos / pdf;
			}

		}
		else
		{
			return Vec3f(0.18);
		}


		return radiance;
	}
};

class IndirectIntegrator : public Integrator
{
public:
	IndirectIntegrator(int maxDepth) : m_maxDepth(maxDepth) {
	}
	virtual ~IndirectIntegrator() = default;

	Vec3f integrate(const Ray& ray_in, const Scene& scene,
		Sampler& sampler) const override
	{
		Vec3f radiance(0.0f);
		Ray ray = ray_in;
		ray.throughput = Vec3f(1, 1, 1);
		Vec3f background(0.0f);

		uint32_t depth = 0;
		while (depth < m_maxDepth)
		{
			IntersectInfo info;
			if (!scene.intersect(ray, info))
			{
				radiance += ray.throughput * background;
				break;
			}

			// russian roulette
			if (depth > 0) {
				const float russian_roulette_prob = std::min(
					(ray.throughput[0] + ray.throughput[1] + ray.throughput[2]) /
					3.0f,
					1.0f);
				if (sampler.getNext1D() >= russian_roulette_prob) { break; }
				ray.throughput /= russian_roulette_prob;
			}

			// Le
			if (info.hitObject->hasAreaLight())
			{
				//radiance += ray.throughput *
				//	info.hitObject->Le(info.surfaceInfo, ray.direction) / std::max(1.0f, info.t * info.t);
				radiance += ray.throughput *
					info.hitObject->Le(info.surfaceInfo, ray.direction);

				break;
			}

			// Surface Objcet
			// indirect light
			float pdf = 1.0;
			Vec3f nextDir(0.0f);
			auto fr = info.hitObject->sampleBxDF(ray.direction, info.surfaceInfo, sampler, nextDir, pdf);
			float cos = std::max(.0f, dot(nextDir, info.surfaceInfo.ng));
			// Li*cos(theta)*fr/pdf   /distance^2?
			auto bias = 0.01f;
			//ray.throughput *= fr * cos/ pdf / std::max(1.0f, info.t * info.t);
			ray.throughput *= fr * cos / pdf;
			ray.origin = info.surfaceInfo.position + info.surfaceInfo.ng * bias;
			ray.direction = nextDir;


			depth++;
		}

		return radiance;
	}

private:
	const uint32_t m_maxDepth;
};

/*
* think over sphere attenuation:
* object --> eye
* light --> object
* object --> object
*/
class GIIntegrator : public Integrator
{
public:
	GIIntegrator(int maxDepth) : m_maxDepth(maxDepth) {
	}
	virtual ~GIIntegrator() = default;

	Vec3f integrate(const Ray& ray_in, const Scene& scene,
		Sampler& sampler) const override
	{
		Vec3f radiance(0.0f);
		Ray ray = ray_in;
		ray.throughput = Vec3f(1, 1, 1);
		Vec3f background(0.0f);

		uint32_t depth = 0;
		while (depth < m_maxDepth)
		{
			IntersectInfo info;
			if (!scene.intersect(ray, info))
			{
				radiance += ray.throughput * background;
				break;
			}

			// russian roulette
			if (depth > 0) {
				const float russian_roulette_prob = std::min(
					(ray.throughput[0] + ray.throughput[1] + ray.throughput[2]) /
					3.0f,
					1.0f);
				if (sampler.getNext1D() >= russian_roulette_prob) { break; }
				ray.throughput /= russian_roulette_prob;
			}

			// Le
			if (info.hitObject->hasAreaLight())
			{
				if (depth == 0)
				{
					//radiance += ray.throughput *
					//	info.hitObject->Le(info.surfaceInfo, ray.direction) / std::max(1.0f, info.t * info.t);

					radiance += ray.throughput *
						info.hitObject->Le(info.surfaceInfo, ray.direction);
				}
				break;
			}

			// Surface Object
			// direct light
			Vec3f directL(0.0f);
			for (const auto& light : scene.getAreaLights())
			{
				Vec3f L_light(0.0f);
				Vec3f wi;
				float tmax, pdf = 0.0f;
				Vec3f L = light->sample(info.surfaceInfo.position, wi, pdf, tmax, sampler);	// light-->object

				if (pdf == 0)
					continue;

				auto bias = 0.01f;
				bool vis = !scene.occluded(Ray(info.surfaceInfo.position + info.surfaceInfo.ng * bias, wi), tmax - bias);
				float cos = std::max(0.0f, dot(info.surfaceInfo.ng, wi));
				Vec3f fr = info.hitObject->evaluateBxDF(ray.direction, wi, info.surfaceInfo);
				L_light += vis * fr * L * cos / pdf;

				directL += L_light;
			}
			//radiance += ray.throughput * directL / std::max(1.0f, info.t * info.t);
			radiance += ray.throughput * directL;

			// indirect light
			float pdf = 1.0;
			Vec3f nextDir(0.0f);
			auto fr = info.hitObject->sampleBxDF(ray.direction, info.surfaceInfo, sampler, nextDir, pdf);
			float cos = std::max(.0f, dot(nextDir, info.surfaceInfo.ng));
			// Li*cos(theta)*fr/pdf   /distance^2?
			auto bias = 0.01f;
			ray.throughput *= fr * cos / pdf;
			ray.origin = info.surfaceInfo.position + info.surfaceInfo.ng * bias;
			ray.direction = nextDir;


			depth++;
		}

		return radiance;
	}

private:
	const uint32_t m_maxDepth;
};


class WhittedIntegrator : public Integrator
{
public:
	WhittedIntegrator(uint32_t maxDepth = 3)
		: m_maxDepth(maxDepth) {
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
				switch (info.hitObject->materialType())
				{
				case MaterialType::Lambert:// diffuse
				{
					Vec3f radiance(0);
					for (const auto& light : scene.getDeltaLights())
					{
						Vec3f wi;
						float t_max, pdf;
						Vec3f light_L = light->sample(info, wi, pdf, t_max);
						bool vis = !scene.occluded(Ray(info.surfaceInfo.position + info.surfaceInfo.ng * 0.1, wi), t_max);
						auto fr = info.hitObject->evaluateBxDF(ray.direction, wi, info.surfaceInfo);
						radiance += vis * fr * light_L * std::max(0.f, dot(info.surfaceInfo.ns, wi)) / pdf;
					}
					radiance *= ray.throughput;
					total_radiance += radiance;
				}
				break;
				case MaterialType::Metals:// reflect
				{
					// reflect direction
					Ray reflect_ray;
					reflect_ray.origin = info.surfaceInfo.position + 0.001 * info.surfaceInfo.ng;
					reflect_ray.direction = reflect(ray.direction, info.surfaceInfo.ng);
					reflect_ray.throughput = 0.8 * ray.throughput;
					reflect_ray.depth = ray.depth + 1;
					rays.push(reflect_ray);
				}
				break;
				case MaterialType::Glass:// reflect and refract
				{
					// compute fresnel
					float kr;
					fresnel(ray.direction, info.surfaceInfo.ng, 1.3/*info.hitPrimitive->ior()*/, kr);
					bool outside = dot(ray.direction, info.surfaceInfo.ng) < 0;
					Vec3f bias = /*options.bias*/0.001 * info.surfaceInfo.ng;
					// compute refraction if it is not a case of total internal reflection
					if (kr < 1) {
						Ray refract_ray;
						refract_ray.direction = normalize(refract(ray.direction, info.surfaceInfo.ng, 1.3/*info.hitPrimitive->ior()*/));
						refract_ray.origin = outside ? info.surfaceInfo.position - bias : info.surfaceInfo.position + bias;
						refract_ray.throughput = 0.9 * ray.throughput * (1 - kr);
						refract_ray.depth = ray.depth + 1;

						rays.push(refract_ray);
					}

					Ray reflect_ray;
					reflect_ray.direction = normalize(reflect(ray.direction, info.surfaceInfo.ng));
					reflect_ray.origin = outside ? info.surfaceInfo.position + bias : info.surfaceInfo.position - bias;
					reflect_ray.throughput = 0.9 * ray.throughput * kr;
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


class VolumePathTracing : public Integrator
{
public:
	VolumePathTracing(uint32_t maxDepth)
		: m_maxDepth(maxDepth) {
	}
	virtual ~VolumePathTracing() = default;

	Vec3f integrate(const Ray& ray_in, const Scene& scene,
		Sampler& sampler) const override
	{
		Vec3f radiance(0);
		Ray ray = ray_in;
		ray.throughput = Vec3f(1, 1, 1);
		Vec3f background(0.0f);

		uint32_t depth = 0;
		while (depth < m_maxDepth)
		{
			/*spdlog::info("ray.origin({},{},{}) direction({},{},{})",
				ray.origin[0], ray.origin[1], ray.origin[2],
				ray.direction[0], ray.direction[1], ray.direction[2]);*/

			IntersectInfo info;
			if (!scene.intersect(ray, info))
			{
				radiance += ray.throughput * background * (depth != 0);
				break;
			}

			// russian roulette
			if (depth > 0) {
				const float russian_roulette_prob = std::min(
					(ray.throughput[0] + ray.throughput[1] + ray.throughput[2]) /
					3.0f,
					1.0f);
				if (sampler.getNext1D() >= russian_roulette_prob) { break; }
				ray.throughput /= russian_roulette_prob;
			}

			// Le
			if (info.hitObject->hasAreaLight()) {
				radiance += ray.throughput *
					info.hitObject->Le(info.surfaceInfo, ray.direction);
				//spdlog::info("hit AreaLight");
				break;
			}

			// sample medium
			bool is_scattered = false;
			if (info.hitObject->hasMedium()) {
				Vec3f pos;
				Vec3f dir;
				Vec3f throughput_medium;
				is_scattered = info.hitObject->sampleMedium(ray, info, sampler, pos, dir,
					throughput_medium);

				// advance ray
				//spdlog::info("is_scattered:{}, cos:{}", is_scattered, dot(dir, ray.direction));
				ray.origin = pos;
				ray.direction = dir;

				// update throughput
				ray.throughput *= throughput_medium;

				if (is_scattered)
					depth++;
			}
		}

		//spdlog::info("--------");
		return radiance;
	}

private:
	const uint32_t m_maxDepth;

};


class VolumePathTracingNEE : public Integrator
{
public:
	VolumePathTracingNEE(uint32_t maxDepth)
		: m_maxDepth(maxDepth) {
	}
	virtual ~VolumePathTracingNEE() = default;

	Vec3f integrate(const Ray& ray_in, const Scene& scene,
		Sampler& sampler) const override
	{
		Vec3f radiance(0);
		Ray ray = ray_in;
		ray.throughput = Vec3f(1, 1, 1);
		Vec3f background(0.0f);

		uint32_t depth = 0;
		while (depth < m_maxDepth)
		{
			IntersectInfo info;
			if (!scene.intersect(ray, info))
			{
				radiance += ray.throughput * background * (depth != 0);
				break;
			}

			// russian roulette
			if (depth > 0) {
				const float russian_roulette_prob = std::min(
					(ray.throughput[0] + ray.throughput[1] + ray.throughput[2]) /
					3.0f,
					1.0f);
				if (sampler.getNext1D() >= russian_roulette_prob) { break; }
				ray.throughput /= russian_roulette_prob;
			}

			// Le
			if (info.hitObject->hasAreaLight()) {
				if (depth == 0)
				{
					radiance += ray.throughput *
						info.hitObject->Le(info.surfaceInfo, ray.direction);
				}
				spdlog::debug("hit AreaLight");
				break;
			}

			// sample medium
			bool is_scattered = false;
			if (info.hitObject->hasMedium()) 
			{
				Vec3f pos;
				Vec3f dir;
				Vec3f throughput_medium;
				is_scattered = info.hitObject->sampleMedium(ray, info, sampler, pos, dir,
					throughput_medium);

				if (is_scattered)
				{
					Vec3f dir_to_light;
					float dist_to_light;
					float pdf_dir;
					const Vec3f Le = sampleDirectionToLight(scene, pos, sampler, dir_to_light,
						dist_to_light, pdf_dir);

 					Vec3f transmittance;
					if (isVisible(scene, pos, dir_to_light, sampler, transmittance)) 
					{
						// add contribution
						const Vec3f f = info.hitObject->evalPhaseFunction(ray.direction, dir_to_light);
						const Vec3f Ls = transmittance * f * Le / pdf_dir;
						// if (length(Ls) > 1.0f) {
						//   spdlog::info("firefly");
						//   spdlog::info("Ls: ({}, {}, {})", Ls[0], Ls[1], Ls[2]);
						//   spdlog::info("transmittance: ({}, {}, {})", transmittance[0],
						//                transmittance[1], transmittance[2]);
						//   spdlog::info("f: ({}, {}, {})", f[0], f[1], f[2]);
						//   spdlog::info("pdf_dir: {}", pdf_dir);
						// }
						radiance += ray.throughput * throughput_medium * Ls;
					}
				}


				// advance ray
				spdlog::debug("is_scattered:{}, cos:{}", is_scattered, dot(dir, ray.direction));
				ray.origin = pos;
				ray.direction = dir;

				// update throughput
				ray.throughput *= throughput_medium;

				if (is_scattered)
					depth++;
			}
		}

		spdlog::debug("--------");
		return radiance;
	}

private:
	Vec3f sampleDirectionToLight(const Scene& scene, const Vec3f& pos,
		Sampler& sampler, Vec3f& dir,
		float& dist_to_light, float& pdf) const
	{
		// sample light
		float light_choose_prob;
		auto light = scene.sampleAreaLight(sampler, light_choose_prob);

		// sample point on light
		float light_pos_pdf = 0.0f;
		Vec3f L = light->sample(pos, dir, light_pos_pdf, dist_to_light, sampler);  // light-->medium

 		// convert point pdf to directional pdf
 		pdf = light_choose_prob * light_pos_pdf;

		return L;
	}

	bool isVisible(const Scene& scene, const Vec3f& origin,
		const Vec3f& dir, Sampler& sampler, Vec3f& transmittance) const
	{
		// create shadow ray
		Ray shadow_ray;
		shadow_ray.origin = origin;
		shadow_ray.direction = dir;

		// trace shadow ray
		transmittance = Vec3f(1.0f);
		
		IntersectInfo shadow_info;
		if (scene.intersect(shadow_ray, shadow_info))
		{
			// occuluded
			if (shadow_info.hitObject->hasSurface()) { return false; }

			// update transmittance
			if (shadow_info.hitObject->hasMedium())
			{
				transmittance *= shadow_info.hitObject->sampleTransparency(
					shadow_ray.origin, shadow_ray(shadow_info.t1), sampler);
			}
		}
		

		return true;
	}

private:
	const uint32_t m_maxDepth;

};

class VolumeRayMarching : public Integrator
{
public:
	VolumeRayMarching(float step) : m_stepLength(step) {}

	Vec3f integrate(const Ray& ray_in, const Scene& scene,
		Sampler& sampler) const override
	{
		Vec3f radiance(0);
		Vec3f background_colour(0.0f);

		IntersectInfo info;
		if (scene.intersect(ray_in, info)) 
		{
			if (info.hitObject->hasMedium())
			{
				Vec3f ray_start = ray_in(info.t);
				Vec3f ray_stop = ray_in(info.t1);
				Vec3f step_vector = m_stepLength * ray_in.direction;
				float ray_length = info.t1 - info.t;

				float d = 2; // russian roulette "probability"
				Vec3f position = ray_start;				
				Vec3f transparency = 1.0;       // initialize transparency to 1
				while (ray_length > 0) 
				{					
					Vec3f sample_pos = position + step_vector * sampler.getNext1D();

				    // attenuate global transparency by sample transparency
				    transparency *= info.hitObject->sampleTransparency(position, position + step_vector, sampler);

					// russian roulette
					if (transparency[0] < 0.001f && transparency[1] < 0.001f && transparency[2] < 0.001f) {
						if (sampler.getNext1D() > 1.0 / d)
							break;
						else
							transparency *= d;
					}

					// in-scatting					
					//for (const auto& light : scene.getDeltaLights())
					//{
					//	Ray light_ray;
					//	light_ray.origin = sample_pos;
					//	light_ray.direction = /*normalize(light_positon - sample_pos)*/light;
					//	float l_0, l_1;

					//	Vec3f wi;
					//	float tmax, pdf = 0.0f;
					//	Vec3f L = light->sample(info, wi, pdf, tmax);

					//	if (pdf == 0)
					//		continue;

					//	auto bias = 0.01f;
					//	bool vis = !scene.occluded(Ray(info.surfaceInfo.position + info.surfaceInfo.ng * bias, wi), tmax - bias);
					//	float cos = std::max(0.0f, dot(info.surfaceInfo.ng, wi));
					//	Vec3f fr = info.hitObject->evaluateBxDF(ray_in.direction, wi, info.surfaceInfo);
					//	radiance += vis * fr * L * cos / pdf;
					//}

					
					ray_length -= m_stepLength;
					position += step_vector;
				}
				Vec3f volume_color = Vec3f(0.5, 0.2, 0.3);
				radiance = transparency * background_colour + (1.0 - transparency) * volume_color;
			}
		}

		return radiance;
	}
protected:
private:
	float m_stepLength;
};