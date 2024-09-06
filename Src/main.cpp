#include "camera.h"
#include "image.h"
#include "integrator.h"

#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkOBJWriter.h>
#include <vtkPolyData.h>
#include <vtkCubeSource.h>

void makeSpere()
{
	vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
	sphereSource->SetCenter(0.0, 0.0, 0.0); 
	sphereSource->SetRadius(1.0);
	sphereSource->SetThetaResolution(32);
	sphereSource->SetPhiResolution(32);
	sphereSource->Update(); 

	vtkSmartPointer<vtkCubeSource> cubeSource = vtkSmartPointer<vtkCubeSource>::New();
	cubeSource->SetXLength(3.0);
	cubeSource->SetYLength(3.0);
	cubeSource->SetZLength(0.01); // 给正方形一个很小的高度，使其看起来像一个平面
	cubeSource->SetCenter(0.0, -1.5, 0.0); // 将正方形放在球的下方
	cubeSource->Update();

	vtkSmartPointer<vtkOBJWriter> objWriter = vtkSmartPointer<vtkOBJWriter>::New();
	std::string filename = DATA_DIR;
	objWriter->SetFileName((filename + "sphere32.obj").c_str());
	objWriter->SetInputData(sphereSource->GetOutput());
	objWriter->Write();
}

int main(int argc, char** argv)
{
	//makeSpere();


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
	scene.loadObj(dataDir + "sphere8.obj");
	scene.makeDeltaLight();
	//scene.build();

	// render
	UniformSampler sampler;
	NormalIntegrator integrator(camera);
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