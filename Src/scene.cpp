#include "scene.h"
#include "material.h"
#include <tiny_obj_loader.h>

std::unique_ptr<Material> makeMaterial(const tinyobj::material_t& material)
{
	if (material.unknown_parameter.count("no_surface") == 1) { return nullptr; }

	const Vec3f kd =
		Vec3f(material.diffuse[0], material.diffuse[1], material.diffuse[2]);
	const Vec3f ks =
		Vec3f(material.specular[0], material.specular[1], material.specular[2]);

	switch (material.illum) {
	case 5:
		// mirror
		//return std::make_shared<Mirror>(Vec3(1.0f));
	case 7:
		// glass
		//return std::make_shared<Glass>(Vec3(1.0f), material.ior);
	default:
		// lambert
		return std::make_unique<Lambert>(kd);
	}
}

std::unique_ptr<AreaLight> makeAreaLight(
	const tinyobj::material_t& material, const Primitive& tri)
{
	if (material.emission[0] > 0 || material.emission[1] > 0 ||
		material.emission[2] > 0) {
		const Vec3f le =
			Vec3f(material.emission[0], material.emission[1], material.emission[2]);
		Matrix44f mat;
		return std::make_unique<TriangleLight>(tri, le);
	}
	else {
		return nullptr;
	}
}

void Scene::loadObj(const std::filesystem::path& filepath)
{
	spdlog::info("[Scene] loading: {}", filepath.generic_string());


	std::string inputfile = filepath.generic_string();
	tinyobj::ObjReaderConfig reader_config;
	reader_config.mtl_search_path = DATA_DIR; // Path to material files
	reader_config.triangulate = true;

	tinyobj::ObjReader reader;
	if (!reader.ParseFromFile(inputfile, reader_config)) {
		if (!reader.Error().empty()) {
			spdlog::error("[Scene] failed to load {} : {}",
				filepath.generic_string(), reader.Error());
		}
		exit(1);
	}

	if (!reader.Warning().empty()) {
		spdlog::warn("[Scene] {}", reader.Warning());
	}

	auto& attrib = reader.GetAttrib();
	auto& shapes = reader.GetShapes();
	auto& materials = reader.GetMaterials();

	for (const auto& material : materials) {		
		m_material.push_back(makeMaterial(material));
	}

	// Loop over shapes
	for (size_t s = 0; s < shapes.size(); s++) {
		// Loop over faces(polygon)
		spdlog::info("[Scene] loading shape: {}", shapes[s].name);
		size_t index_offset = 0;

		int materialID = -1;
		std::vector<Primitive> primitives;
		for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
			size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]); // vertex num of current face

			std::vector<Vec3f> vertices;
			std::vector<Vec3f> normals;
			std::vector<Vec2f> texcoords;

			// Loop over vertices in the face. if triangulate == true, then always is 3 times
			for (size_t v = 0; v < fv; v++) {
				// access to vertex
				tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
				tinyobj::real_t vx = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
				tinyobj::real_t vy = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
				tinyobj::real_t vz = attrib.vertices[3 * size_t(idx.vertex_index) + 2];
				vertices.push_back(Vec3f(vx, vy, vz));

				// Check if `normal_index` is zero or positive. negative = no normal data
				if (idx.normal_index >= 0) {
					tinyobj::real_t nx = attrib.normals[3 * size_t(idx.normal_index) + 0];
					tinyobj::real_t ny = attrib.normals[3 * size_t(idx.normal_index) + 1];
					tinyobj::real_t nz = attrib.normals[3 * size_t(idx.normal_index) + 2];
					normals.push_back(Vec3f(nx, ny, nz));
				}

				// Check if `texcoord_index` is zero or positive. negative = no texcoord data
				if (idx.texcoord_index >= 0) {
					tinyobj::real_t tx = attrib.texcoords[2 * size_t(idx.texcoord_index) + 0];
					tinyobj::real_t ty = attrib.texcoords[2 * size_t(idx.texcoord_index) + 1];
					texcoords.push_back(Vec2f(tx, ty));
				}
			}

			// if normals is empty, add geometric normal
			if (normals.size() == 0) {
				const Vec3f v1 = vertices[1] - vertices[0];
				const Vec3f v2 = vertices[2] - vertices[0];
				const Vec3f n = normalize(cross(v1, v2));
				normals.push_back(n);
				normals.push_back(n);
				normals.push_back(n);
			}

			// if texcoords is empty, add barycentric coords
			if (texcoords.size() == 0) {
				texcoords.push_back(Vec2f(0, 0));
				texcoords.push_back(Vec2f(1, 0));
				texcoords.push_back(Vec2f(0, 1));
			}

			// material ID of current face
			const int mID = shapes[s].mesh.material_ids[f];
			if (materialID != mID)
			{
				if (materialID != -1) {
					spdlog::warn("[Scene] Multiple materials in one object. {}", materials[materialID].name);
				}
				else {
					materialID = mID;
				}
			}

			primitives.emplace_back(Primitive(vertices, normals, texcoords));

			index_offset += fv;
		}


		m_objects[shapes[s].name] = std::make_shared<Mesh>(primitives, m_material[materialID].get(), nullptr);
	}
}

void Scene::addObj(std::string name, std::shared_ptr<Object> obj)
{
	m_objects[name] = obj;
}

void Scene::makeDeltaLight()
{

	/*Matrix44f l2w(
		1.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 1.0);*/
	Matrix44f l2w(0.95292, 0.289503, 0.0901785, 0, -0.0960954, 0.5704, -0.815727, 0, -0.287593, 0.768656, 0.571365, 0, 0, 0, 0, 1);
	m_deltaLights.push_back(std::make_shared<DistantLight>(l2w, Vec3f(1.0, 1.0, 1.0), 1.0f));

	Matrix44f l2w2(
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		5.0, 5.0, -1.0, 1.0);
	//m_deltaLights.push_back(std::make_shared<PointLight>(mat, Vec3f(0.63, 0.33, 0.03), 50.0));
	//m_deltaLights.push_back(std::make_shared<PointLight>(l2w2, Vec3f(0.0, 1.0, 0.0), 500.0));


	Matrix44f l2w3(
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 2.0, 1.0);
	//m_deltaLights.push_back(std::make_shared<PointLight>(l2w3, Vec3f(0.63, 0.33, 0.03), 2.0));
	//m_deltaLights.push_back(std::make_shared<PointLight>(l2w3, Vec3f(1.0, 1.0, 1.0), 2.0f));
}

void Scene::makeAreaLight()
{
	auto light = std::make_shared<QuadLight>(
		Vec3f(343.0, 548.0, 227.0),
		Vec3f(343.0, 548.0, 332.0),
		Vec3f(213.0, 548.0, 227.0),
		Matrix44f(), 15.0f * Vec3f(1.0, 1.0, 1.0));

	m_areaLights.push_back(light);
	addObj("QuadLight", light->getObject());



	Matrix44<float> xfm_sphere(.2, 0, 0, 0, 0, .2, 0, 0, 0, 0, .2, 0, 0, 0, -4, 1);
	std::shared_ptr<SphereLight> sphere_light = std::make_shared<SphereLight>(Vec3f(0.0f), 0.2f, xfm_sphere, Vec3f(10.0f));

	//m_areaLights.push_back(sphere_light);
	//addObj("SphereLight", sphere_light->getObject());
}

const std::vector<std::shared_ptr<DeltaLight>>& Scene::getDeltaLights() const
{
	return m_deltaLights;
}

const std::vector<std::shared_ptr<AreaLight>>& Scene::getAreaLights() const
{
	return m_areaLights;
}

bool Scene::intersect(const Ray& ray, IntersectInfo& info) const
{
	bool isIntersect = false;
	for (const auto& [name, obj] : m_objects)
	{
		if (obj->intersect(ray, info))
			isIntersect = true;
	}

	return isIntersect;
}

bool Scene::occluded(const Ray& ray, float t_max) const
{
	for (const auto& [name, obj] : m_objects)
	{
		if ((!obj->hasAreaLight()) && obj->occluded(ray, t_max))
			return true;
	}

	return false;
}

