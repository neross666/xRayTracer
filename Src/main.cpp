#include "camera.h"
#include "image.h"
#include "integrator.h"



int main(int argc, char** argv)
{
	const uint32_t width = 512;
	const uint32_t height = 512;
	const uint32_t n_samples = 1;
	const uint32_t max_depth = 10000;

	Image image(width, height);

	const Vec3f camera_pos = Vec3f(0, 0, 0);
	const Vec3f camera_forward = Vec3f(0, 0, -1);
	const float FOV = 0.25 * PI;

	const auto camera =
		std::make_shared<PinholeCamera>(camera_pos, camera_forward, FOV);

	// build scene
	Scene scene;
	//scene.loadObj("CornellBox-Mist.obj");
	//scene.build();

	// render
	UniformSampler sampler;
	PathIntegrator integrator(camera, n_samples, max_depth);
	integrator.render(scene, sampler, image);

	// gamma correction
	image.gammaCorrection(2.2f);

	// output image
	auto output = image.writeMat();
	cv::imshow("output", output);
	cv::waitKey(0);

	return 0;
}