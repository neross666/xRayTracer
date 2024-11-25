#include <openvdb/openvdb.h>
#include <nanovdb/util/IO.h>
#include <nanovdb/util/OpenToNanoVDB.h>
#include <spdlog/spdlog.h>
#include <variant>


int main()
{
	openvdb::initialize();

	// open vdb file
	std::string dataDir = DATA_DIR;
	std::string filename = "wdas_cloud_quarter.vdb";
	openvdb::io::File file(dataDir + filename);
	if (!file.open()) {
		spdlog::error("[OpenVDBGrid] failed.");
	}

	// get density grid
	auto gridPtr =
		openvdb::gridPtrCast<openvdb::FloatGrid>(file.readGrid("density"));
	if (!gridPtr) {
		spdlog::error("[OpenVDBGrid] failed to load density grid");
		return 1;
	}
	file.close();

	auto handle = nanovdb::openToNanoVDB(gridPtr);
	auto* dstGrid = handle.grid<float>(); // Get a (raw) pointer to the NanoVDB grid form the GridManager.
	if (!dstGrid)
		throw std::runtime_error("GridHandle does not contain a grid with value type float");

	nanovdb::io::writeGrid(dataDir + "wdas_cloud_quarter.nvdb", handle); // Write the NanoVDB grid to file and throw if writing fails


	return 0;
}