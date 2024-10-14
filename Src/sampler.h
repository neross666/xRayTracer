#pragma once
#include <random>
#include <memory>
#include "geometry.h"

class UniformSampler;
// sampler interface
class Sampler
{
public:
	enum class SamplerType
	{
		Uniform
	};
protected:
	std::mt19937 gen;

public:
	Sampler() {}
	Sampler(uint32_t seed) : gen(seed) {}
	virtual ~Sampler() = default;

	static std::unique_ptr<Sampler> makeSampler(SamplerType st);

	void setSeed(uint32_t seed) { gen.seed(seed); }

	void discard(unsigned long long nSkip) {
		gen.discard(nSkip);
	}


	virtual float getNext1D() = 0;
	virtual Vec2f getNext2D() = 0;
};

// uniform distribution sampler
class UniformSampler : public Sampler
{
private:
	std::uniform_real_distribution<float> dis;
	int count = 0;

public:
	UniformSampler() : dis(0.0f, 1.0f), Sampler() {}
	UniformSampler(uint32_t seed) : dis(0.0f, 1.0f), Sampler(seed) {}
	~UniformSampler() = default;

	float getNext1D() override { count++; return dis(gen); }
	Vec2f getNext2D() override { count += 2; return Vec2f(dis(gen), dis(gen)); }
};

