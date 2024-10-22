﻿#include "camera.h"
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
	const uint32_t width = 512;
	const uint32_t height = 512;
	const uint32_t n_samples = 32;
	const float step_length = 0.001;

	Image image(width, height);
	const float aspect_ratio = static_cast<float>(width) / height;


	const Matrix44f c2w(
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 5.0, 1.0);
	//const float FOV = 2.0f * 180.0f * std::atanf(1.0f / 3.0f) / PI;
	const float FOV = 60.0f;
	const auto camera =
		std::make_unique<PinholeCamera>(aspect_ratio, c2w, FOV);

	// build scene
	Scene scene;

	std::string dataDir = DATA_DIR;
	const auto medium = std::make_unique<HomogeneousMediumMIS>(0.0f, Vec3f(0.05f), Vec3f(0.05f), AABB{ Vec3f(-1.0f), Vec3f(1.0f) });
	scene.addObj("medium", medium->makeObject());

	Matrix44<float> xfm_sphere(
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 2, 0.0, 1);
	//scene.addAreaLight("SphereLight", std::make_unique<SphereLight>(Vec3f(0.0f, 380.0f, 0.0f), 50.0f, xfm_sphere, Vec3f(10.0f, 10.0f, 10.0f)));


	// integrator
	//const auto integrator = std::make_unique<NormalIntegrator>();
	const auto integrator = std::make_unique<VolumeRayMarching>(step_length);


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
	cv::imwrite(getCurrentDateTime() + "-VolumeRayMarch.jpg", output);
	cv::imshow("VRM", output);
	cv::waitKey(0);


	return 0;
}