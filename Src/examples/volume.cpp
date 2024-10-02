#include "camera.h"
#include "image.h"
#include "integrator.h"
#include "material.h"
#include "renderer.h"
#include "medium.h"


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
	const uint32_t width = 780;
	const uint32_t height = 585;
	const uint32_t n_samples = 1024;
	const uint32_t max_depth = 8;

	Image image(width, height);
	const float aspect_ratio = static_cast<float>(width) / height;


	const Matrix44f c2w(
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 5.0, 1.0);
	const float FOV = 60.0f;
	const auto camera =
		std::make_unique<PinholeCamera>(aspect_ratio, c2w, FOV);

	// build scene
	Scene scene;

	const auto medium = std::make_unique<HomogeneousMedium>(0.0f, Vec3f(0.1f), Vec3f(0.1f), AABB{ Vec3f(-1.0f), Vec3f(1.0f) });
	scene.addObj("medium", medium->makeObject());

	Matrix44<float> xfm_sphere(
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 1.75, 0, 1);
	scene.addAreaLight("SphereLight", std::make_unique<SphereLight>(Vec3f(0.0f), 0.2f, xfm_sphere, Vec3f(10.0f)));



	// integrator
	//const auto integrator = std::make_unique<NormalIntegrator>();
	//const auto integrator = std::make_unique<WhittedIntegrator>(max_depth);
	//const auto integrator = std::make_unique<DirectIntegrator>();
	//const auto integrator = std::make_unique<IndirectIntegrator>(max_depth);
	//const auto integrator = std::make_unique<GIIntegrator>(max_depth);
	const auto integrator = std::make_unique<VolumePathTracing>(max_depth);


	// render
	UniformSampler sampler;
	//auto renderer = std::make_unique<NormalRenderer>(n_samples, camera.get(), integrator.get());
	auto renderer = std::make_unique<ParallelRenderer>(n_samples, camera.get(), integrator.get());
	renderer->render(scene, sampler, image);


	// gamma correction
	image.gammaCorrection(1.2f);

	// output image
	auto output = image.writeMat();
	cv::imwrite(getCurrentDateTime() + "-volume.jpg", output);
	cv::imshow("output", output);
	cv::waitKey(0);


	return 0;
}