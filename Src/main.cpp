#include "camera.h"
#include "image.h"
#include "integrator.h"

#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkOBJWriter.h>
#include <vtkPolyData.h>
#include <vtkCubeSource.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>

void makeSpere()
{
	vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
	sphereSource->SetCenter(-.5, 1.5, -1.0); 
	sphereSource->SetRadius(.6);
	sphereSource->SetThetaResolution(8);
	sphereSource->SetPhiResolution(8);
	sphereSource->Update(); 



	vtkSmartPointer<vtkOBJWriter> objWriter = vtkSmartPointer<vtkOBJWriter>::New();
	std::string filename = DATA_DIR;
	objWriter->SetFileName((filename + "sphere.obj").c_str());
	objWriter->SetInputData(sphereSource->GetOutput());
	objWriter->Write();
}

void makeCube()
{
	// 创建立方体源
	vtkSmartPointer<vtkCubeSource> cubeSource = vtkSmartPointer<vtkCubeSource>::New();
	cubeSource->SetCenter(-2.0, 2.0, -2.0);

	// 写入OBJ文件
	vtkSmartPointer<vtkOBJWriter> writer = vtkSmartPointer<vtkOBJWriter>::New();
	std::string filename = DATA_DIR;
	writer->SetFileName((filename + "cube.obj").c_str());
	writer->SetInputConnection(cubeSource->GetOutputPort());
	writer->Write();
}

int main(int argc, char** argv)
{
	//makeSpere();
	//makeCube();


	const uint32_t width = 512;
	const uint32_t height = 512;
	const uint32_t n_samples = 1;
	const uint32_t max_depth = 10000;

	Image image(width, height);

	//const Vec3f camera_pos = Vec3f(260, 260, -500.0);
	//const Vec3f camera_forward = Vec3f(0, 0, 1);
	const Vec3f camera_pos = Vec3f(0, 0, 10.0);
	const Vec3f camera_forward = Vec3f(0, 0, -1);
	const float FOV = 0.25 * PI;

	const auto camera =
		std::make_shared<PinholeCamera>(camera_pos, camera_forward, FOV);

	// build scene
	Scene scene;
	std::string dataDir = DATA_DIR;
	scene.loadObj(dataDir + "cube.obj");
	scene.addObj("sphere_diffuse", std::make_shared<Sphere>(Vec3f(0.0), 1.0f, 2, Vec3f(0.0, 0.0, 0.0)));
	//scene.addObj("sphere_mirror", std::make_shared<Sphere>(Vec3f(1.5), 1.0f, 1, Vec3f(0.0, 1.0, 0.0)));
	scene.addObj("sphere_dielectric", std::make_shared<Sphere>(Vec3f(-1.0, 1.0, -1.0), 1.0f, 0, Vec3f(0.0, 0.0, 1.0)));
	scene.makeDeltaLight();

	// render
	UniformSampler sampler;
	WhittedIntegrator integrator(camera);
	//NormalIntegrator integrator(camera);
	//PathTracing integrator(camera, 100, 10);
	integrator.render(scene, sampler, image);

	// gamma correction
	image.gammaCorrection(1.2f);

	// output image
	auto output = image.writeMat();
	cv::imshow("output", output);
	cv::waitKey(0);

	return 0;
}