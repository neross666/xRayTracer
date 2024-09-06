#pragma once
#include "geometry.h"


// [comment]
// Light base class
// [/comment]
class Light
{
public:
    Light(const Matrix44f& l2w, const Vec3f& c = 1, const float& i = 1) : lightToWorld(l2w), color(c), intensity(i) {}
    virtual ~Light() {}
    virtual void illuminate(const Vec3f& P, Vec3f&, Vec3f&, float&) const = 0;
    Vec3f color;
    float intensity;
    Matrix44f lightToWorld;
};

// [comment]
// Distant light
// the default direction is (0,0,-1)
// [/comment]
class DistantLight : public Light
{
    Vec3f dir;
public:
    DistantLight(const Matrix44f& l2w, const Vec3f& c = 1, const float& i = 1) : Light(l2w, c, i)
    {
        l2w.multDirMatrix(Vec3f(0, 0, -1), dir);
        dir = normalize(dir); // in case the matrix scales the light        
    }
    Vec3f direction() const{ return dir; }
    void illuminate(const Vec3f& P, Vec3f& lightDir, Vec3f& lightIntensity, float& distance) const
    {
        lightDir = dir;
        lightIntensity = color * intensity;
        distance = kInfinity;
    }
};

// [comment]
// Point light
// the default positon is (0,0,0)
// [/comment]
class PointLight : public Light
{
    Vec3f pos;
public:
    PointLight(const Matrix44f& l2w, const Vec3f& c = 1, const float& i = 1) : Light(l2w, c, i)
    {
        l2w.multVecMatrix(Vec3f(0), pos);
    }
    // P: is the shaded point
    void illuminate(const Vec3f& P, Vec3f& lightDir, Vec3f& lightIntensity, float& distance) const
    {
        lightDir = (P - pos);
        float r2 = length2(lightDir);
        distance = sqrt(r2);
        lightDir[0] /= distance, lightDir[1] /= distance, lightDir[2] /= distance;
        // avoid division by 0
        lightIntensity = color * intensity / (4 * PI * r2);
    }
};