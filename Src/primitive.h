#pragma once

class Primitive
{
public:
	Primitive(const std::vector<Vec3f>& vertices, const std::vector<Vec3f>& normals, const std::vector<Vec2f>& texcoords) :
		m_vertices(vertices), m_normals(normals), m_texcoords(texcoords) {
	}
	Primitive(std::vector<Vec3f>&& vertices, std::vector<Vec3f>&& normals, std::vector<Vec2f>&& texcoords) : 
		m_vertices(vertices), m_normals(normals), m_texcoords(texcoords) {
	}
	~Primitive() = default;

	std::vector<Vec3f>& vertices() {
		return m_vertices;
	}

	std::vector<Vec3f>& normals() {
		return m_normals;
	}

	std::vector<Vec2f>& texcoords() {
		return m_texcoords;
	}

private:
	std::vector<Vec3f> m_vertices;
	std::vector<Vec3f> m_normals;
	std::vector<Vec2f> m_texcoords;
};

class Object
{
public:
	Object(int materialID, std::vector<Primitive>&& primitives, Vec3f albedo = Vec3f(1.0f))
		: m_materialID(materialID), m_primitives(primitives), m_albedo(albedo) {

	}
	Object(int materialID, const std::vector<Primitive>& primitives, Vec3f albedo = Vec3f(1.0f))
		: m_materialID(materialID), m_primitives(primitives), m_albedo(albedo) {

	}
	~Object() = default;

	Vec3f albedo() const {
		return m_albedo;
	}

	float ior() const {
		return 1.3f;
	}

	int material() const{
		return m_materialID;
	}

	std::vector<Primitive>& primitives() {
		return m_primitives;
	}

private:
	const int m_materialID;
	Vec3f m_albedo;
	std::vector<Primitive> m_primitives;
};


