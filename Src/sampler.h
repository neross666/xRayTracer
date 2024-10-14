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

// sample value from 1D discrete empirical distribution
class DiscreteEmpiricalDistribution1D
{
private:
	std::vector<float> cdf;
	std::vector<float> pdf;

public:
	DiscreteEmpiricalDistribution1D(const float* values, unsigned int N)
	{
		// sum f
		float sum = 0;
		for (std::size_t i = 0; i < N; ++i) { sum += values[i]; }

		// compute cdf
		cdf.resize(N + 1);
		cdf[0] = 0;
		for (std::size_t i = 1; i < N + 1; ++i) {
			cdf[i] = cdf[i - 1] + values[i - 1] / sum;
		}

		// compute pdf
		pdf.resize(N);
		for (std::size_t i = 0; i < N; ++i) { pdf[i] = cdf[i + 1] - cdf[i]; }
	}

	DiscreteEmpiricalDistribution1D(const std::vector<float>& values)
		: DiscreteEmpiricalDistribution1D(values.data(), values.size())
	{
	}

	uint32_t sample(float u, float& pdf) const
	{
		// inverse cdf
		int x = std::lower_bound(cdf.begin(), cdf.end(), u) - cdf.begin();
		if (x == 0) { x++; }

		// compute pdf
		pdf = cdf[x] - cdf[x - 1];

		// NOTE: cdf's index is +1 from values
		return x - 1;
	}

	float getPDF(uint32_t i) const { return pdf[i]; }
};