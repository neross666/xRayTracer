#pragma once
#include <filesystem>
#include <spdlog/spdlog.h>

class Scene
{
public:
	Scene() {}
	~Scene() {};

	void loadObj(const std::filesystem::path& filepath) {
		spdlog::info("[Scene] loading: {}", filepath.generic_string());
		

	}

	bool intersect(const Ray& ray, IntersectInfo& info) const
	{

	}

protected:
private:
};

 