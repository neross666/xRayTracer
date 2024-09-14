#pragma once
#include "geometry.h"


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

protected:
    Vec3f Le_;
    Matrix44f lightToWorld;
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
        , Ng_(cross(e1_, e2_)){
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