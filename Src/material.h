#pragma once
#include "geometry.h"


class Material
{
public:
	Material() = default;
	virtual ~Material() = default;

	virtual Vec3f evaluate() const = 0;

	virtual MaterialType materialType() const = 0;

// 	float ior() const {
// 		return 1.3f;
// 	}

protected:

};

class Lambert : public Material
{
public:
	Lambert(Vec3f albedo) : m_albedo(albedo){
	};
	~Lambert() = default;

	MaterialType materialType() const override{
		return MaterialType::Lambert;
	}

	Vec3f evaluate() const override {
		return m_albedo;
	}

private:
	Vec3f m_albedo = Vec3f(0.0f);
};
