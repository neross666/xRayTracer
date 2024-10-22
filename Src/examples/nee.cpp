#include "camera.h"
#include "image.h"
#include "integrator.h"
#include "material.h"
#include "renderer.h"
#include "medium.h"
#include "grid.h"


std::string getCurrentDateTime()
{
	auto now = std::chrono::system_clock::now();
	std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
	std::tm now_tm = *std::localtime(&now_time_t);

	std::stringstream ss;
	ss << std::put_time(&now_tm, "%Y-%m-%d_%H-%M-%S");
	return ss.str();
}

int main(int argc, char** argv)
{
#ifdef _DEBUG
	spdlog::set_level(spdlog::level::info);
#else
	spdlog::set_level(spdlog::level::info);
#endif


	const uint32_t width = 512;
	const uint32_t height = 512;
	const uint32_t n_samples = 32;
	const uint32_t max_depth = 10;


	Image image(width, height);
	const float aspect_ratio = static_cast<float>(width) / height;


	const Matrix44f c2w(
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 70.0, 550.0, 1.0);
	const float FOV = 75.0f;
	const auto camera =
		std::make_unique<PinholeCamera>(aspect_ratio, c2w, FOV);


	// build scene
	Scene scene;
	std::string dataDir = DATA_DIR;
	auto gridData = std::make_unique<OpenVDBGrid>(dataDir + "wdas_cloud_sixteenth.vdb");
	const auto medium = std::make_unique<HeterogeneousMedium>(0.0f, gridData.get(), Vec3f(0.1f), Vec3f(0.8f));
	scene.addObj("medium", medium->makeObject());

	Matrix44<float> xfm_sphere(
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 400.0, 0.0, 1);
	//scene.addAreaLight("SphereLight", std::make_unique<SphereLight>(Vec3f(0.0f), 50.0f, xfm_sphere, Vec3f(20.0f, 20.0f, 20.0f)));

	scene.addAreaLight("QuadLight", std::make_unique<QuadLight>(
		Vec3f(100, 350, 100),
		Vec3f(-100, 350, 100),
		Vec3f(100, 350, -100),
		Matrix44f(), 20.0f * Vec3f(1.0, 1.0, 1.0)));


	// integrator
	//const auto integrator = std::make_unique<NormalIntegrator>();
	const auto integrator = std::make_unique<VolumePathTracingNEE>(max_depth);


	// render
#ifdef _DEBUG
	auto renderer = std::make_unique<NormalRenderer>(n_samples, camera.get(), integrator.get());
#else
	auto renderer = std::make_unique<ParallelRenderer>(n_samples, camera.get(), integrator.get());
#endif // _DEBUG
	renderer->render(scene, Sampler::SamplerType::Uniform, image);


	// gamma correction
	image.gammaCorrection(2.2f);

	// output image
	auto output = image.writeMat();
	cv::imwrite(getCurrentDateTime() + "-NEE.jpg", output);
	cv::imshow("NEE", output);
	cv::waitKey(0);


	return 0;
}