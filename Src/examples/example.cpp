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

	/*const Matrix44f c2w(
		0.827081, 0, -0.562083, 0,
		-0.152433, 0.962525, -0.224298, 0,
		0.541019, 0.271192, 0.796086, 0,
		2.924339, 1.020801, 0.511729, 1);*/
	const Matrix44f c2w(
		1.0, 0, 0, 0,
		0, 1.0, 0, 0,
		0, 0, 1.0, 0,
		0, 0, 8.0, 1);
	const float FOV = 60.0f;
	const auto camera =
		std::make_unique<PinholeCamera>(aspect_ratio, c2w, FOV);

	const auto material = std::make_unique<Lambert>(Vec3f(0.58));

	// build scene
	Scene scene;
	std::string dataDir = DATA_DIR;
	//scene.addObj("sphere_mesh", std::make_unique<SphereMesh>(Vec3f(0.0), 1.0f, 10, 10, material.get(), nullptr));
	scene.loadObj(dataDir + "cube.obj");
	scene.addObj("sphere_diffuse", std::make_unique<Sphere>(Vec3f(0.0), 1.0f, material.get(), nullptr));
	//scene.addObj("sphere_mirror", std::make_unique<Sphere>(Vec3f(1.5), 1.0f, material.get(), nullptr));
	//scene.addObj("sphere_dielectric", std::make_unique<Sphere>(Vec3f(-1.0, 1.0, -1.0), 1.0f, material.get(), nullptr));

	/*Matrix44f l2w(
		1.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 1.0);*/
	Matrix44f l2w(0.95292, 0.289503, 0.0901785, 0,
		-0.0960954, 0.5704, -0.815727, 0,
		-0.287593, 0.768656, 0.571365, 0,
		0, 0, 0, 1);
	scene.addDeltaLight("DistantLight", std::make_unique<DistantLight>(l2w, Vec3f(1.0, 1.0, 1.0), 1.0f));

	Matrix44f l2w2(
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		5.0, 5.0, -1.0, 1.0);
	scene.addDeltaLight("PointLight", std::make_unique<PointLight>(l2w2, Vec3f(0.63, 0.33, 0.03), 50.0f));


	//Matrix44<float> xfm_sphere(.2, 0, 0, 0, 0, .2, 0, 0, 0, 0, .2, 0, 0, 0, -4, 1);
	//scene.addAreaLight("SphereLight", std::make_unique<SphereLight>(Vec3f(0.0f), 0.2f, xfm_sphere, Vec3f(10.0f)));


	// render
	UniformSampler sampler;
	//NormalIntegrator integrator(camera.get(), 1);
	WhittedIntegrator integrator(camera.get(), n_samples, max_depth);
	//DirectIntegrator integrator(camera.get(), 16);
	//IndirectIntegrator integrator(camera.get(), 16, 3);
	//GIIntegrator integrator(camera.get(), 16, 3);
	//PathTracing integrator(camera.get(), 100, 10);
	integrator.render(scene, sampler, image);

	// gamma correction
	image.gammaCorrection(1.2f);

	// output image
	auto output = image.writeMat();
	cv::imwrite(getCurrentDateTime() + "-xRayTracer.jpg", output);
	cv::imshow("output", output);
	cv::waitKey(0);


	return 0;
}