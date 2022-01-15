#pragma once

#include "Vector.hpp"
#include "global.hpp"

class Object
{
public:
    Object()
        : materialType(DIFFUSE_AND_GLOSSY)
        , ior(1.3)
        , Kd(0.8)
        , Ks(0.2)
        , diffuseColor(0.2)
        , specularExponent(25)
    {}

    virtual ~Object() = default;

    virtual bool intersect(const Vector3f&, const Vector3f&, float&, uint32_t&, Vector2f&) const = 0;

    virtual void getSurfaceProperties(const Vector3f&, const Vector3f&, const uint32_t&, const Vector2f&, Vector3f&,
                                      Vector2f&) const = 0; //根据位置坐标获取该点的属性

    virtual Vector3f evalDiffuseColor(const Vector2f&) const  //根据纹理坐标获取物体颜色
    {
        return diffuseColor;
    }

    // material properties
    MaterialType materialType;
    float ior;  //index of refraction: 折射率，空气折射率为1，玻璃折射率为1.5
    float Kd, Ks; //漫反射系数，高光系数
    Vector3f diffuseColor;  //漫反射颜色，也就是物体的颜色
    float specularExponent; //高光的幂次参数p
};
