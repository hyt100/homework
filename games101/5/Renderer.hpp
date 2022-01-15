#pragma once
#include "Scene.hpp"

struct hit_payload
{
    float tNear;     //射线求交的最小的t值
    uint32_t index;  //相交的triangle索引 (仅triangle)
    Vector2f uv;     //重心坐标系数b1/b2, b3可计算(1-b1-b2)  (仅triangle)
    Object* hit_obj; //物体指针
};

class Renderer
{
public:
    void Render(const Scene& scene);

private:
};