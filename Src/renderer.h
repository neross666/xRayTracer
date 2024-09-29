#pragma once
#include "scene.h"
#include "Image.h"
#include "Sampler.h"

class Camera;
class Integrator;
class Renderer
{
public:
	Renderer(Camera* cam, Integrator* inte) : camera(cam), integrator(inte) {}
	virtual ~Renderer() = default;

	// render scene
	virtual void render(const Scene& scene, Sampler& sampler, Image& image) const = 0;

protected:
	const Camera* camera;
	const Integrator* integrator;
};

class NormalRenderer : public Renderer
{
public:
	NormalRenderer(uint32_t spp, Camera* cam, Integrator* inte) : n_samples(spp), Renderer(cam, inte) {}
	~NormalRenderer() = default;

	// render scene
	virtual void render(const Scene& scene, Sampler& sampler, Image& image) const override final;

private:
	// number of samples in each pixel
	const uint32_t n_samples;
};

class ParallelRenderer : public Renderer
{
public:
	ParallelRenderer(uint32_t spp, Camera* cam, Integrator* inte) : n_samples(spp), Renderer(cam, inte) {}
	~ParallelRenderer() = default;

	// render scene
	void render(const Scene& scene, Sampler& sampler, Image& image) const override;

private:
	// number of samples in each pixel
	const uint32_t n_samples;
};