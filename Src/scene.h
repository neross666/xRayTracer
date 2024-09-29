#pragma once
#include <filesystem>
#include <spdlog/spdlog.h>
#include "geometry.h"
#include "ray.h"


class Object;
class Material;
class DeltaLight;
class AreaLight;
class Scene
{
public:
	Scene() = default;
	~Scene() = default;

	void loadObj(const std::filesystem::path& filepath);

	void addObj(std::string name, std::unique_ptr<Object> obj);

	void build() {

	}

	void addDeltaLight(std::string name, std::unique_ptr<DeltaLight> light);

	void addAreaLight(std::string name, std::unique_ptr<AreaLight> light);

	const std::vector<std::unique_ptr<DeltaLight>>& getDeltaLights() const;

	const std::vector<std::unique_ptr<AreaLight>>& getAreaLights() const;

	bool intersect(const Ray& ray, IntersectInfo& info) const;

	// ignore semi-translucent
	bool occluded(const Ray& ray, float t_max) const;

protected:
private:
	std::vector<std::unique_ptr<DeltaLight>> m_deltaLights;
	std::vector<std::unique_ptr<AreaLight>> m_areaLights;
	std::unordered_map<std::string, std::unique_ptr<Object>> m_objects;
	std::vector<std::unique_ptr<Material>> m_material;
};

