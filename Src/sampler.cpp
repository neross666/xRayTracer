#include "sampler.h"

std::unique_ptr<Sampler> Sampler::makeSampler(SamplerType st)
{
	switch (st)
	{
	case Sampler::SamplerType::Uniform:
		return std::make_unique<UniformSampler>();
	default:
		return nullptr;
	}
}
