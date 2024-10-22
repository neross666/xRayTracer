#pragma once
#include "ray.h"
#include <filesystem>
#include <openvdb/openvdb.h>
#include <openvdb/tools/Interpolation.h>
#include <spdlog/spdlog.h>



class DensityGrid
{
public:
	virtual AABB getBounds() const = 0;
	virtual float getDensity(const Vec3f& pos) const = 0;
	virtual float getMaxDensity() const = 0;
};

class VtkGrid : public DensityGrid
{

};

class OpenVDBGrid : public DensityGrid
{
private:
	openvdb::FloatGrid::Ptr gridPtr;

public:
	OpenVDBGrid(const std::filesystem::path& filepath)
	{
		spdlog::info("[OpenVDBGrid] loading: {}", filepath.generic_string());

		openvdb::initialize();

		// open vdb file
		openvdb::io::File file(filepath.generic_string());
		if (!file.open()) {
			spdlog::error("[OpenVDBGrid] failed to load {}",
				filepath.generic_string());
		}

		// get density grid
		this->gridPtr =
			openvdb::gridPtrCast<openvdb::FloatGrid>(file.readGrid("density"));
		if (!this->gridPtr) {
			spdlog::error("[OpenVDBGrid] failed to load density grid");
			return;
		}
		file.close();

		AABB bbox = getBounds();
		spdlog::info("[Scene] OpenVDB Volume bounding box");
		spdlog::info("[Scene] pMin: ({}, {}, {})", bbox.pMin[0], bbox.pMin[1],
			bbox.pMin[2]);
		spdlog::info("[Scene] pMax: ({}, {}, {})", bbox.pMax[0], bbox.pMax[1],
			bbox.pMax[2]);
	}

	AABB getBounds() const override
	{
		AABB ret;
		const auto bbox = gridPtr->evalActiveVoxelBoundingBox();
		const auto pMin = gridPtr->indexToWorld(bbox.getStart());
		const auto pMax = gridPtr->indexToWorld(bbox.getEnd());
		for (int i = 0; i < 3; ++i) {
			ret.pMin[i] = pMin[i];
			ret.pMax[i] = pMax[i];
		}
		return ret;
	}

	float getDensity(const Vec3f& pos) const override
	{
		const auto gridSampler =
			openvdb::tools::GridSampler<openvdb::FloatGrid,
			openvdb::tools::BoxSampler>(*this->gridPtr);
		return gridSampler.wsSample(openvdb::Vec3f(pos[0], pos[1], pos[2]));
	}

	float getMaxDensity() const override
	{
		float minValue, maxValue;
		gridPtr->evalMinMax(minValue, maxValue);
		return maxValue;
	}
};