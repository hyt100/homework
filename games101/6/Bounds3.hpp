//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

class Bounds3
{
  public:
    Vector3f pMin, pMax; // two points to specify the bounding box
    Bounds3()
    {
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {}
    Bounds3(const Vector3f p1, const Vector3f p2)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    Vector3f Diagonal() const { return pMax - pMin; }  //对角线向量
    int maxExtent() const //最长的边是哪个
    {
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    double SurfaceArea() const //包围盒的表面积
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; } //中心点
    Bounds3 Intersect(const Bounds3& b) //两个包围盒的交集
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }

    Vector3f Offset(const Vector3f& p) const
    {
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    bool Overlaps(const Bounds3& b1, const Bounds3& b2) //包围盒重叠测试
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    bool Inside(const Vector3f& p, const Bounds3& b) //点是否在内部测试
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }
    inline const Vector3f& operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;
    }

    inline bool IntersectP(const Ray& ray, float &t) const //射线和包围盒相交测试
    {
        // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
        // dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x<0),int(y<0),int(z<0)], use this to simplify your logic
        // TODO test if ray bound intersects
        
        float t_enter = std::numeric_limits<float>::lowest();
        float t_exit = std::numeric_limits<float>::max();
        float near, far;

        if (fabs(ray.direction.x) < 0.00001) { //注意射线x坐标为0时，与yoz平面平行，该组“对面”没有交点；和一组“对面”平行，就降至二维，两对就降至一维。
            if (ray.origin.x < pMin.x || ray.origin.x > pMax.x) //平行则光线x值介于pMin/pMax中间
                return false;
        } else {
            near = (pMin.x - ray.origin.x) * ray.direction_inv.x;
            far  = (pMax.x - ray.origin.x) * ray.direction_inv.x;
            if (ray.dirIsNeg[0]) std::swap(near, far);   //注意射线的方向是否朝着负轴，负轴的near和far计算是相反的
            t_enter = std::max(t_enter, near);
            t_exit  = std::min(t_exit, far);
        }

        if (fabs(ray.direction.y) < 0.00001) {
            if (ray.origin.y < pMin.y || ray.origin.y > pMax.y)
                return false;
        } else {
            near = (pMin.y - ray.origin.y) * ray.direction_inv.y;
            far  = (pMax.y - ray.origin.y) * ray.direction_inv.y;
            if (ray.dirIsNeg[1]) std::swap(near, far);
            t_enter = std::max(t_enter, near);
            t_exit  = std::min(t_exit, far);
        }

        if (fabs(ray.direction.z) < 0.00001) {
            if (ray.origin.z < pMin.z || ray.origin.z > pMax.z)
                return false;
        } else {
            near = (pMin.z - ray.origin.z) * ray.direction_inv.z;
            far  = (pMax.z - ray.origin.z) * ray.direction_inv.z;
            if (ray.dirIsNeg[2]) std::swap(near, far);
            t_enter = std::max(t_enter, near);
            t_exit  = std::min(t_exit, far);
        }

        if (t_enter < t_exit && t_exit >= 0) {
            if (t_enter <= 0) // The ray’s origin is inside the box
                t = 0; 
            else
                t = t_enter;
            return true;
        } else {
            return false;
        }
    } 
};


//两个包围盒合并
inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

//包围盒和一个点合并
inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
