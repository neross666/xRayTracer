#include "geometry.h"

float length(const Vec3f& v)
{
	return std::sqrt(dot(v, v));
}

float length2(const Vec3f& v)
{
	return dot(v, v);
}

Vec3f normalize(const Vec3f& v)
{
	return v / length(v);
}

Vec3f exp(const Vec3f& v)
{
	return Vec3f(std::exp(v[0]), std::exp(v[1]), std::exp(v[2]));
}

void orthonormalBasis(const Vec3f& n, Vec3f& t, Vec3f& b)
{
#if 0
	if (std::fabs(n[0]) > std::fabs(n[1])) {
		float inv_len = 1 / std::sqrt(n[0] * n[0] + n[2] * n[2]);
		t = Vec3f(n[2] * inv_len, 0, -n[0] * inv_len);
	}
	else {
		float inv_len = 1 / std::sqrt(n[1] * n[1] + n[2] * n[2]);
		t = Vec3f(0, -n[2] * inv_len, n[1] * inv_len);
	}
	b = cross(n, t);
#elif 0
	if (std::abs(n[1]) < 0.9f) {
		t = normalize(cross(n, Vec3f(0, 1, 0)));
	}
	else {
		t = normalize(cross(n, Vec3f(0, 0, -1)));
	}
	b = normalize(cross(t, n));
#else
	float sign = std::copysign(1.0f, n[2]);	// n[2] == 0 !!!
	const float a = -1.0f / (sign + n[2]);
	const float c = n[0] * n[1] * a;
	t = Vec3f(1.0f + sign * n[0] * n[0] * a, sign * c, -sign * n[0]);
	b = Vec3f(c, sign + n[1] * n[1] * a, -n[1]);
#endif
}

Vec3f reflect(const Vec3f& I, const Vec3f& N)
{
	return I - 2 * dot(I, N) * N;
}

Vec3f refract(const Vec3f& I, const Vec3f& N, const float& ior)
{
	float cosi = std::clamp(dot(I, N), -1.0f, 1.0f);
	float etai = 1, etat = ior;
	Vec3f n = N;
	if (cosi < 0) { cosi = -cosi; }
	else { std::swap(etai, etat); n = -N; }
	float eta = etai / etat;
	float k = 1 - eta * eta * (1 - cosi * cosi);
	return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
}

void fresnel(const Vec3f& I, const Vec3f& N, const float& ior, float& kr)
{
	float cosi = std::clamp(dot(I, N), -1.0f, 1.0f);
	float etai = 1, etat = ior;
	if (cosi > 0) { std::swap(etai, etat); }
	// Compute sini using Snell's law
	float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
	// Total internal reflection
	if (sint >= 1) {
		kr = 1;
	}
	else {
		float cost = sqrtf(std::max(0.f, 1 - sint * sint));
		cosi = fabsf(cosi);
		float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
		float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
		kr = (Rs * Rs + Rp * Rp) / 2;
	}
	// As a consequence of the conservation of energy, transmittance is given by:
	// kt = 1 - kr;
}
