#pragma once
#include <random>

// sampler interface
class Sampler
{
protected:
	std::mt19937 gen;

public:
	Sampler() {}

	Sampler(uint32_t seed) : gen(seed) {}

	void setSeed(uint32_t seed) { gen.seed(seed); }

	void discard(unsigned long long nSkip) {
		gen.discard(nSkip);
	}

	virtual std::unique_ptr<Sampler> clone() const = 0;
	virtual float getNext1D() = 0;
	virtual Vec2f getNext2D() = 0;
};

// uniform distribution sampler
class UniformSampler : public Sampler
{
private:
	std::uniform_real_distribution<float> dis;

public:
	UniformSampler() : dis(0.0f, 1.0f), Sampler() {}
	UniformSampler(uint32_t seed) : dis(0.0f, 1.0f), Sampler(seed) {}

	std::unique_ptr<Sampler> clone() const override
	{
		return std::make_unique<UniformSampler>();
	}

	float getNext1D() override { return dis(gen); }
	Vec2f getNext2D() override { return Vec2f(dis(gen), dis(gen)); }
};

