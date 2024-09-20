#pragma once
#include "geometry.h"
#include "primitive.h"


// [comment]
// Light base class
// [/comment]
class DeltaLight
{
public:
    DeltaLight(const Matrix44f& l2w, const Vec3f& c = 1, const float& i = 1) : lightToWorld(l2w), color(c), intensity(i) {}
    virtual ~DeltaLight() {}

    virtual Vec3<float> sample(const IntersectInfo& info, Vec3<float>& wi, float& pdf, float& t_max) const = 0;

    Vec3f color;
    float intensity;
    Matrix44f lightToWorld;
};

// [comment]
// Distant light
// the default direction is (0,0,-1)
// [/comment]
class DistantLight : public DeltaLight
{
    Vec3f dir;
public:
    DistantLight(const Matrix44f& l2w, const Vec3f& c = 1, const float& i = 1) : DeltaLight(l2w, c, i)
    {
        l2w.multDirMatrix(Vec3f(0, 0, -1), dir);
        dir = normalize(dir); // in case the matrix scales the light        
    }
    Vec3f direction() const{ return dir; }

    Vec3<float> sample(const IntersectInfo& info, Vec3<float>& wi, float& pdf, float& t_max) const override
    {
        wi = -dir;
        pdf = 1.0f;
        t_max = kInfinity;
        return color * intensity;
    }
};

// [comment]
// Point light
// the default positon is (0,0,0)
// [/comment]
class PointLight : public DeltaLight
{
    Vec3f pos;
public:
    PointLight(const Matrix44f& l2w, const Vec3f& c = 1, const float& i = 100.0) : DeltaLight(l2w, c, i)
    {
        l2w.multVecMatrix(Vec3f(0), pos);
    }

    Vec3<float> sample(const IntersectInfo& info, Vec3<float>& wi, float& pdf, float& t_max) const override
    {
        auto lightDir = pos - info.surfaceInfo.position;        // Note the very close proximity of point light to point hitPoint
        float distance = length(lightDir);
        wi = lightDir / distance;
        pdf = distance * distance;  // why not 4*PI*distance*distance?
        t_max = distance;
        return color * intensity;
    }
};




class AreaLight {
public:
    AreaLight(const Matrix44f& l2w, const Vec3f& Le) : lightToWorld(l2w), Le_(Le) {
    }

    virtual Vec3f sample(const IntersectInfo& info, Vec3<float>& wi, float& pdf, float& t_max, Sampler& sample) const = 0;

    virtual Vec3f Le() const { return Le_; }

    std::shared_ptr<Object> getObject() { return lightObject; };

protected:
    Vec3f Le_;
    Matrix44f lightToWorld;
    std::shared_ptr<Object> lightObject;
};


class TriangleLight : public AreaLight
{
public:
    TriangleLight(const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, const Matrix44f& l2w, const Vec3f& Le)
        : AreaLight(l2w, Le)
        , v0_(multVecMatrix(v0, l2w))
        , v1_(multVecMatrix(v1, l2w))
        , v2_(multVecMatrix(v2, l2w))
        , e1_(v1_ - v0_)
        , e2_(v2_ - v0_)
        , Ng_(cross(e1_, e2_))
    {
        std::vector<Vec3f> vertices{ v0_, v1_, v2_ };
        auto n = normalize(Ng_);
        std::vector<Vec3f> normals{ n,n,n };
        std::vector<Vec2f> texcoords{ Vec2f(0, 0), Vec2f(1, 0),Vec2f(0, 1) };
        std::vector<Primitive> primitives{ Primitive(vertices, normals, texcoords) };

        lightObject = std::make_shared<Mesh>(primitives, 3, Le);
    }

    Vec3f sample(const IntersectInfo& info, Vec3<float>& wi, float& pdf, float& tmax, Sampler& sample) const override
    {
        Vec3f d = uniformSampleTriangle(sample.getNext1D(), sample.getNext1D(), v0_, v1_, v2_) - info.surfaceInfo.position;
        tmax = length(d);
        float d_dot_Ng = dot(d, Ng_);
        if (d_dot_Ng >= 0) return 0;
        wi = d / tmax;
        pdf = (2.f * tmax * tmax * tmax) / std::abs(d_dot_Ng);
        return Le_;
    }


private:
    inline Vec3f uniformSampleTriangle(const float& u, const float& v, const Vec3f& A, const Vec3f& B, const Vec3f& C) const{
        float su = std::sqrt(u);
        return Vec3f(C + (1.f - su) * (A - C) + (v * su) * (B - C));
    }

private:
    Vec3f v0_;
    Vec3f v1_;
    Vec3f v2_;

private:
    Vec3f e1_;
    Vec3f e2_;
    Vec3f Ng_;
};


class QuadLight : public AreaLight
{
public:
    QuadLight(const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, const Matrix44f& l2w, const Vec3f& Le)
        : AreaLight(l2w, Le)
        , v0_(multVecMatrix(v0, l2w))
        , v1_(multVecMatrix(v1, l2w))
        , v2_(multVecMatrix(v2, l2w))
        , e1_(v1_ - v0_)
        , e2_(v2_ - v0_)
        , Ng_(cross(e1_, e2_))
    {
        Vec3f v3_ = v0_ + e1_ + e2_;
        auto n = normalize(Ng_);
        std::vector<Vec3f> normals{ n,n,n };
        std::vector<Vec2f> texcoords{ Vec2f(0, 0), Vec2f(1, 0),Vec2f(0, 1) };
        std::vector<Primitive> primitives{ 
            Primitive({ v0_, v1_, v2_ }, normals, texcoords),
            Primitive({ v1_, v3_, v2_ }, normals, texcoords)
        };

        lightObject = std::make_shared<Mesh>(primitives, 3, Le);
    }

    Vec3f sample(const IntersectInfo& info, Vec3<float>& wi, float& pdf, float& tmax, Sampler& sample) const override
    {
        Vec3f d = (v0_ + e1_ * sample.getNext1D() + e2_ * sample.getNext1D()) - info.surfaceInfo.position;
        tmax = length(d);
        float d_dot_Ng = dot(d, Ng_);
        if (d_dot_Ng >= 0) return 0;
        wi = d / tmax;
        pdf = (tmax * tmax * tmax) / std::abs(d_dot_Ng);
        return Le_;
    }

private:
    Vec3f v0_;
    Vec3f v1_;
    Vec3f v2_;

private:
    Vec3f e1_;
    Vec3f e2_;
    Vec3f Ng_;
};


class SphereLight : public AreaLight
{
public:
    SphereLight(const Vec3f& center, float raduis, const Matrix44f& l2w, const Vec3f& Le)
        : AreaLight(l2w, Le)
        , center_(multVecMatrix(center, l2w))
        , radius_(raduis)
    {
        lightObject = std::make_shared<SphereMesh>(center_, radius_, 10, 10, 3, Le);
    }

    Vec3f sample(const IntersectInfo& info, Vec3<float>& wi, float& pdf, float& tmax, Sampler& sample) const override
    {
#ifdef AREA_SAMPLING
        Vec3f n = UniformSampleSphere(sample.getNext1D(), sample.getNext1D());
        Vec3f p = center_ + n * radius_;
        Vec3f d = p - info.surfaceInfo.position;
        tmax = length(d);
#elif defined(INTERSECT_METHOD)
        Vec3f dz = center_ - info.surfaceInfo.position;
        float dz_len_2 = dot(dz, dz);
        float dz_len = std::sqrtf(dz_len_2);
        dz /= dz_len;
        Vec3f dx, dy;

        orthonormalBasis(dz, dy, dx);

        float sin_theta_max_2 = radius_ * radius_ / dz_len_2;
        float cos_theta_max = std::sqrt(std::max(0.f, 1.f - sin_theta_max_2));
        Vec3f sample_dir = UniformSampleCone(sample.getNext1D(), sample.getNext1D(), cos_theta_max, dx, dy, dz);

        IntersectInfo sinfo;
        if (!lightObject->intersect(Ray(info.surfaceInfo.position, sample_dir), sinfo)) {			
			tmax = dot(center_ - info.surfaceInfo.position, sample_dir);
        }
        Vec3f p = info.surfaceInfo.position + sample_dir * tmax;
        Vec3f d = p - info.surfaceInfo.position;
        if (length(p - center_) < radius_)  return Vec3f(0.0f); // check for x inside the sphere
        Vec3f n = normalize(p - center_);
#else
        Vec3f dz = center_ - info.surfaceInfo.position;
        float dz_len_2 = dot(dz, dz);
        float dz_len = std::sqrtf(dz_len_2);
        dz /= -dz_len;
        Vec3f dx, dy;

        orthonormalBasis(dz, dx, dy);

        float sin_theta_max_2 = radius_ * radius_ / dz_len_2;
        float sin_theta_max = std::sqrt(sin_theta_max_2);
        float cos_theta_max = std::sqrt(std::max(0.f, 1.f - sin_theta_max_2));

        float cos_theta = 1 + (cos_theta_max - 1) * sample.getNext1D();
        float sin_theta_2 = 1.f - cos_theta * cos_theta;

        float cos_alpha = sin_theta_2 / sin_theta_max + cos_theta * std::sqrt(1 - sin_theta_2 / (sin_theta_max * sin_theta_max));
        float sin_alpha = std::sqrt(1 - cos_alpha * cos_alpha);
        float phi = 2 * PI * sample.getNext1D();

        Vec3f n = std::cos(phi) * sin_alpha * dx + std::sin(phi) * sin_alpha * dy + cos_alpha * dz;
        Vec3f p = center_ + n * radius_;

        Vec3f d = p - info.surfaceInfo.position;
        tmax = length(d);
#endif

        float d_dot_n = dot(d, n);
        if (d_dot_n >= 0) return 0;

#if AREA_SAMPLING
        wi = d / tmax;
        pdf = (2.f * tmax * tmax * tmax) / std::abs(d_dot_n);
#else
        pdf = 1.f / (2.f * PI * (1.f - cos_theta_max));
        wi = d / tmax;
#endif
        return Le_;
    }


private:
    inline Vec3f UniformSampleSphere(const float& r1, const float& r2) const{
        float z = 1.f - 2.f * r1; // cos(theta)
        float sin_theta = std::sqrt(1 - z * z);
        float phi = 2 * PI * r2; // azimuthal angle
        return { std::cos(phi) * sin_theta, std::sin(phi) * sin_theta, z };
    }

	inline Vec3f UniformSampleCone(float r1, float r2, float cos_theta_max, const Vec3f& x, const Vec3f& y, const Vec3f& z) const{
        float cos_theta = (1.f - r1) + r1 * cos_theta_max;//  std::lerp(cos_theta_max, 1.f, r1); // need c++20
		float sin_theta = std::sqrt(1.f - cos_theta * cos_theta);
		float phi = 2 * PI * r2;
		return std::cos(phi) * sin_theta * x + std::sin(phi) * sin_theta * y + cos_theta * z;
	}

private:
    Vec3f center_;
    float radius_;
};