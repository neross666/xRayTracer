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

	const std::vector<Vec3f>& vertices() const {
		return m_vertices;
	}

	const std::vector<Vec3f>& normals() const {
		return m_normals;
	}

	const std::vector<Vec2f>& texcoords() const {
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
	Object(int materialID, Vec3f albedo)
		: m_materialID(materialID), m_albedo(albedo) {

	}

	virtual ~Object() = default;

	virtual bool intersect(const Ray& ray, IntersectInfo& info) = 0;
	
	Vec3f albedo() const {
		return m_albedo;
	}

	float ior() const {
		return 1.3f;
	}

	int material() const{
		return m_materialID;
	}

private:
	const int m_materialID;
	Vec3f m_albedo;
};

class Sphere : public Object
{
public:
	Sphere(Vec3f center, float raduis, int materialID, Vec3f albedo = Vec3f(1.0f))
		: m_center(center), m_raduis(raduis), m_raduis2(raduis*raduis), Object(materialID, albedo) {

	}
	~Sphere() = default;

	bool intersect(const Ray& ray, IntersectInfo& info) override
	{
		float t = 0.0f;
		if (!doIntersect(ray.origin, ray.direction, t))
			return false;

		if (t < info.t)
		{
			info.t = t;
			info.surfaceInfo.position = ray(t);
			info.surfaceInfo.ng = normalize(ray(t) - m_center);
			info.surfaceInfo.ns = info.surfaceInfo.ng;
			info.surfaceInfo.texcoords = Vec2f(
				(1 + atan2(info.surfaceInfo.ng[2], info.surfaceInfo.ng[0]) / PI) * 0.5,
				acosf(info.surfaceInfo.ng[1]) / PI);
			info.hitObject = this;
		}
		return true;
	}

private:
	bool doIntersect(
		const Vec3f& orig,
		const Vec3f& dir,
		float& tNear) const
	{
		float t0, t1; // solutions for t if the ray intersects
		// analytic solution
		Vec3f L = orig - m_center;
		float a = dot(dir, dir);
		float b = 2 * dot(dir, L);
		float c = dot(L, L) - m_raduis2;
		if (!solveQuadratic(a, b, c, t0, t1)) return false;

		if (t0 > t1) std::swap(t0, t1);

		if (t0 < 0) {
			t0 = t1; // if t0 is negative, let's use t1 instead
			if (t0 < 0) return false; // both t0 and t1 are negative
		}

		tNear = t0;

		return true;
	}

	// [comment]
	// Compute the roots of a quadratic equation
	// [/comment]
	bool solveQuadratic(const float& a, const float& b, const float& c, float& x0, float& x1) const
	{
		float discr = b * b - 4 * a * c;
		if (discr < 0) return false;
		else if (discr == 0) {
			x0 = x1 = -0.5 * b / a;
		}
		else {
			float q = (b > 0) ?
				-0.5 * (b + sqrt(discr)) :
				-0.5 * (b - sqrt(discr));
			x0 = q / a;
			x1 = c / q;
		}

		return true;
	}


private:
	Vec3f m_center;
	float m_raduis;
	float m_raduis2;
};


class Mesh : public Object
{
public:
	Mesh(std::vector<Primitive>&& primitives, int materialID, Vec3f albedo = Vec3f(1.0f))
		: Object(materialID, albedo), m_primitives(primitives) {

	}
	Mesh(const std::vector<Primitive>& primitives, int materialID, Vec3f albedo = Vec3f(1.0f))
		: Object(materialID, albedo), m_primitives(primitives) {

	}
	~Mesh() = default;

	bool intersect(const Ray& ray, IntersectInfo& info) override 
	{
		bool isIntersect = false;
		for (auto& primitive : m_primitives)
		{
			const auto vertices = primitive.vertices();	
			const auto normals = primitive.normals();
			const auto texcoords = primitive.texcoords();
			float t = 0.0f;
			float u = 0.0f;
			float v = 0.0f;
			bool rst = rayTriangleIntersect(ray.origin, ray.direction,
				vertices[0], vertices[1], vertices[2],
				t, u, v);
			if (rst)
			{
				isIntersect = true;
				if (t < info.t)
				{
					info.t = t;
					info.surfaceInfo.position = ray(t);
					info.surfaceInfo.barycentric = Vec2f(u, v);
					info.surfaceInfo.texcoords = texcoords[0] * (1.0f - u - v) + texcoords[1] * u + texcoords[2] * v;
					info.surfaceInfo.ng = normalize(cross(vertices[1] - vertices[0], vertices[2] - vertices[0]));
					info.surfaceInfo.ns = normals[0] * (1.0f - u - v) + normals[1] * u + normals[2] * v;
					orthonormalBasis(info.surfaceInfo.ns, info.surfaceInfo.dpdu,
						info.surfaceInfo.dpdv);
					info.hitObject = this;
				}
			}
		}

		return isIntersect;
	}

private:
	bool rayTriangleIntersect(
		const Vec3f& orig, const Vec3f& dir,
		const Vec3f& v0, const Vec3f& v1, const Vec3f& v2,
		float& t, float& u, float& v) const
	{
		Vec3f v0v1 = v1 - v0;
		Vec3f v0v2 = v2 - v0;
		Vec3f pvec = cross(dir, v0v2);
		float det = dot(v0v1, pvec);

#ifdef CULLING  // 当考虑折射时，打开面剔除，会导致光线无法从电介质中穿出
		// if the determinant is negative the triangle is backfacing
		// if the determinant is close to 0, the ray misses the triangle
		if (det < kEpsilon) return false;
#else
		// ray and triangle are parallel if det is close to 0
		if (fabs(det) < kEpsilon) return false;
#endif
		float invDet = 1 / det;

		Vec3f tvec = orig - v0;
		u = dot(tvec, pvec) * invDet;
		if (u < 0 || u > 1) return false;

		Vec3f qvec = cross(tvec, v0v1);
		v = dot(dir, qvec) * invDet;
		if (v < 0 || u + v > 1) return false;

		t = dot(v0v2, qvec) * invDet;

		return t > kEpsilon;
	}

private:
	std::vector<Primitive> m_primitives;
};