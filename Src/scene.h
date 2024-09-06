#pragma once
#include <filesystem>
#include <spdlog/spdlog.h>
#include <tiny_obj_loader.h>
#include "geometry.h"
#include "light.h"

class Scene
{
public:
    Scene() = default;
	~Scene() = default;

	void loadObj(const std::filesystem::path& filepath) {
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

        // Loop over shapes
        for (size_t s = 0; s < shapes.size(); s++) {
            // Loop over faces(polygon)
            spdlog::info("[Scene] loading shape: {}", shapes[s].name);
            size_t index_offset = 0;
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

                // populate vertices, indices, normals, texcoords
                this->m_vertices.insert(m_vertices.end(), vertices.begin(), vertices.end());
                this->m_normals.insert(m_normals.end(), normals.begin(), normals.end());
                this->m_texcoords.insert(m_texcoords.end(), texcoords.begin(), texcoords.end());                

                // material ID of current face
                const int materialID = shapes[s].mesh.material_ids[f];
                std::optional<tinyobj::material_t> material = std::nullopt;
                if (materialID != -1) 
                    material = materials[materialID];
                const uint32_t faceID = this->m_vertices.size() / 3 - 1;
                //this->materials.emplace(faceID, material);


                index_offset += fv;
            }
        }

	}


    void makeDeltaLight() {
        
        Matrix44f l2w(
            1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 
            0.0, 1.0, 0.0, 0.0, 
            0.0, 0.0, 0.0, 1.0);
        //Matrix44f l2w(0.95292, 0.289503, 0.0901785, 0, -0.0960954, 0.5704, -0.815727, 0, -0.287593, 0.768656, 0.571365, 0, 0, 0, 0, 1);
        m_deltaLight = std::make_shared<DistantLight>(l2w);

        //Matrix44f mat(
        //    1.0, 0.0, 0.0, 0.0,
        //    0.0, 1.0, 0.0, 0.0, 
        //    0.0, 0.0, 1.0, 0.0, 
        //    0.0, 3.0, 1.0, 1.0);
        //m_deltaLight = std::make_shared<PointLight>(mat);
    }

    std::shared_ptr<Light> getDeltaLight() const{
        return m_deltaLight;
    }


	bool intersect(const Ray& ray, IntersectInfo& info) const
	{
        bool isIntersect = false;
        for (size_t i = 0; i < m_vertices.size(); i+=3)
        {
            float t = 0.0f;
            float u = 0.0;
            float v = 0.0;
            bool rst = rayTriangleIntersect(ray.origin, ray.direction,
                m_vertices[i], m_vertices[i+1], m_vertices[i+2], 
                t, u, v);
            if (rst)
            {
                isIntersect = true;
                if (t < info.t)
                {
                    info.t = t;
                    info.surfaceInfo.position = ray(t);
                    info.surfaceInfo.barycentric = Vec2f(u, v);
                    info.surfaceInfo.texcoords = m_texcoords[i] * (1.0f - u - v) + m_texcoords[i + 1] * u + m_texcoords[i + 2] * v;
                    info.surfaceInfo.ng = normalize(cross(m_vertices[i + 1] - m_vertices[i], m_vertices[i + 2] - m_vertices[i]));
                    info.surfaceInfo.ns = m_normals[i] * (1.0f - u - v) + m_normals[i + 1] * u + m_normals[i + 2] * v;
                    orthonormalBasis(info.surfaceInfo.ns, info.surfaceInfo.dpdu,
                        info.surfaceInfo.dpdv);
                }
            }
        }
        return isIntersect;
	}


    bool rayTriangleIntersect(
        const Vec3f& orig, const Vec3f& dir,
        const Vec3f& v0, const Vec3f& v1, const Vec3f& v2,
        float& t, float& u, float& v) const
    {
        Vec3f v0v1 = v1 - v0;
        Vec3f v0v2 = v2 - v0;
        Vec3f pvec = cross(dir, v0v2);
        float det = dot(v0v1, pvec);
                
#ifdef CULLING
        // if the determinant is negative the triangle is backfacing
        // if the determinant is close to 0, the ray misses the triangle
        if (det < kEpsilon) return false;
#else
        // ray and triangle are parallel if det is close to 0
        if (fabs(det) < kEpsilon) return false;
#endif
        float invDet = 1 / det;

        Vec3f tvec = orig - v0;
        u = dot(tvec,pvec) * invDet;
        if (u < 0 || u > 1) return false;

        Vec3f qvec = cross(tvec, v0v1);
        v = dot(dir, qvec) * invDet;
        if (v < 0 || u + v > 1) return false;

        t = dot(v0v2, qvec) * invDet;

        return t > kEpsilon;
    }

protected:
private:
    std::vector<Vec3f> m_vertices;
    std::vector<Vec3f> m_normals;
    std::vector<Vec2f> m_texcoords;

    std::shared_ptr<Light> m_deltaLight;
};

 