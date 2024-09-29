#pragma once
#include <filesystem>
#include <spdlog/spdlog.h>
//#include <tiny_obj_loader.h>
#include "geometry.h"
#include "light.h"
#include "primitive.h"


class Scene
{
public:
	Scene() = default;
	~Scene() = default;

	void loadObj(const std::filesystem::path& filepath);

	void addObj(std::string name, std::shared_ptr<Object> obj);

	void build() {

	}

	void makeDeltaLight();

	void makeAreaLight();

	const std::vector<std::shared_ptr<DeltaLight>>& getDeltaLights() const;

	const std::vector<std::shared_ptr<AreaLight>>& getAreaLights() const;

	bool intersect(const Ray& ray, IntersectInfo& info) const;

	// ignore semi-translucent
	bool occluded(const Ray& ray, float t_max) const;

protected:
private:
	std::vector<std::shared_ptr<DeltaLight>> m_deltaLights;
	std::vector<std::shared_ptr<AreaLight>> m_areaLights;
	std::unordered_map<std::string, std::shared_ptr<Object>> m_objects;
	std::vector<std::unique_ptr<Material>> m_material;
};

