#include "camera.h"
#include "image.h"
#include "integrator.h"
#include "material.h"


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
	const uint32_t n_samples = 16;
	const uint32_t max_depth = 3;

	Image image(width, height);
	const float aspect_ratio = static_cast<float>(width) / height;

	const Matrix44f c2w(
		-1.0, 0, 0, 0,
		0, 1.0, 0, 0,
		0, 0, -1.0, 0,
		278, 274.4, -750.0, 1);
	const float FOV = 60.0f;
	const auto camera =
		std::make_unique<PinholeCamera>(aspect_ratio, c2w, FOV);

	// build scene
	Scene scene;
	std::string dataDir = DATA_DIR;
	scene.loadObj(dataDir + "cornell_box.obj");

	scene.addAreaLight("QuadLight", std::make_unique<QuadLight>(
		Vec3f(343.0, 548.0, 227.0),
		Vec3f(343.0, 548.0, 332.0),
		Vec3f(213.0, 548.0, 227.0),
		Matrix44f(), 15.0f * Vec3f(1.0, 1.0, 1.0)));


	// render
	UniformSampler sampler;
	//NormalIntegrator integrator(camera.get(), n_samples);
	//WhittedIntegrator integrator(camera.get(), n_samples, max_depth);
	DirectIntegrator integrator(camera.get(), n_samples);
	//IndirectIntegrator integrator(camera.get(), n_samples, max_depth);
	//GIIntegrator integrator(camera.get(), n_samples, max_depth);
	//VolumePathTracing integrator(camera.get(), n_samples, max_depth);
	integrator.render(scene, sampler, image);

	// gamma correction
	image.gammaCorrection(1.2f);

	// output image
	auto output = image.writeMat();
	cv::imwrite(getCurrentDateTime() + "-cornellbox.jpg", output);
	cv::imshow("output", output);
	cv::waitKey(0);


	return 0;
}