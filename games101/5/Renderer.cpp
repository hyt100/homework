#include <fstream>
#include "Vector.hpp"
#include "Renderer.hpp"
#include "Scene.hpp"
#include <optional>

inline float deg2rad(const float &deg)
{ return deg * M_PI/180.0; }

// Compute reflection direction 计算反射的向量
Vector3f reflect(const Vector3f &I, const Vector3f &N)
{
    return I - 2 * dotProduct(I, N) * N;
} 

// [comment] 计算折射的向量
// Compute refraction direction using Snell's law
//
// We need to handle with care the two possible situations:
//
//    - When the ray is inside the object
//
//    - When the ray is outside.
//
// If the ray is outside, you need to make cosi positive cosi = -N.I
//
// If the ray is inside, you need to invert the refractive indices and negate the normal N
// [/comment]
Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior)
{
    float cosi = clamp(-1, 1, dotProduct(I, N));
    float etai = 1, etat = ior;
    Vector3f n = N;
    if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
    float eta = etai / etat;
    float k = 1 - eta * eta * (1 - cosi * cosi);
    return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
}

// [comment] 计算反射比
// Compute Fresnel equation   菲涅尔方程: https://zhuanlan.zhihu.com/p/31534769
//
// \param I is the incident view direction 注意I的方向是指向交点
//
// \param N is the normal at the intersection point
//
// \param ior is the material refractive index
// [/comment]
// 返回值是反射比，取值范围是0-1， 为1时表示全反射
float fresnel(const Vector3f &I, const Vector3f &N, const float &ior)
{
    float cosi = clamp(-1, 1, dotProduct(I, N));
    float etai = 1, etat = ior;  //入射：空气折射率为1  折射：物体折射率为ior
    if (cosi > 0) {  std::swap(etai, etat); }  //默认是从空气中到物体中，如果是从物体折射到空气中，则交换折射率
    cosi = fabsf(cosi);  //修正入射角的cosine值,因为入射角范围应该是0-90度
    // Compute sint using Snell's law 斯涅耳定律： sini*etai = sint*etat
    float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
    // Total internal reflection
    if (sint >= 1) {  //折射角为90度，全反射
        return 1;
    }
    else {
        float cost = sqrtf(std::max(0.f, 1 - sint * sint));
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        return (Rs * Rs + Rp * Rp) / 2;
    }
    // As a consequence of the conservation of energy, transmittance is given by:
    // kt = 1 - kr; //根据能量守恒，反射占比kr + 折射占比 = 1
}

// [comment]
// Returns true if the ray intersects an object, false otherwise.
//
// \param orig is the ray origin
// \param dir is the ray direction
// \param objects is the list of objects the scene contains
// \param[out] tNear contains the distance to the cloesest intersected object.
// \param[out] index stores the index of the intersect triangle if the interesected object is a mesh.
// \param[out] uv stores the u and v barycentric coordinates of the intersected point
// \param[out] *hitObject stores the pointer to the intersected object (used to retrieve material information, etc.)
// \param isShadowRay is it a shadow ray. We can return from the function sooner as soon as we have found a hit.
// [/comment]
std::optional<hit_payload> trace(
        const Vector3f &orig, const Vector3f &dir,
        const std::vector<std::unique_ptr<Object> > &objects)
{
    float tNear = kInfinity;
    std::optional<hit_payload> payload;
    for (const auto & object : objects)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (object->intersect(orig, dir, tNearK, indexK, uvK) && tNearK < tNear)
        {
            payload.emplace();
            payload->hit_obj = object.get();
            payload->tNear = tNearK;
            payload->index = indexK;
            payload->uv = uvK;
            tNear = tNearK;
        }
    }

    return payload;
}

// [comment]
// Implementation of the Whitted-style light transport algorithm (E [S*] (D|G) L)
//
// This function is the function that compute the color at the intersection point
// of a ray defined by a position and a direction. Note that thus function is recursive (it calls itself).
//
// If the material of the intersected object is either reflective or reflective and refractive,
// then we compute the reflection/refraction direction and cast two new rays into the scene
// by calling the castRay() function recursively. When the surface is transparent, we mix
// the reflection and refraction color using the result of the fresnel equations (it computes
// the amount of reflection and refraction depending on the surface normal, incident view direction
// and surface refractive index).
//
// If the surface is diffuse/glossy we use the Phong illumation model to compute the color
// at the intersection point.
// [/comment]
Vector3f castRay(
        const Vector3f &orig, const Vector3f &dir, const Scene& scene,
        int depth)
{
    if (depth > scene.maxDepth) {
        return Vector3f(0.0,0.0,0.0);
    }

    Vector3f hitColor = scene.backgroundColor;  //没有交点的，使用背景颜色
    if (auto payload = trace(orig, dir, scene.get_objects()); payload) //如果找到交点
    {
        Vector3f hitPoint = orig + dir * payload->tNear;
        Vector3f N; // normal
        Vector2f st; // st coordinates
        payload->hit_obj->getSurfaceProperties(hitPoint, dir, payload->index, payload->uv, N, st); //获取交点处的属性:法线、纹理坐标
        switch (payload->hit_obj->materialType) {
            case REFLECTION_AND_REFRACTION: //材质类型：反射和折射
            {
                Vector3f reflectionDirection = normalize(reflect(dir, N));
                Vector3f refractionDirection = normalize(refract(dir, N, payload->hit_obj->ior));
                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                                             hitPoint - N * scene.epsilon :
                                             hitPoint + N * scene.epsilon;
                Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ?
                                             hitPoint - N * scene.epsilon :
                                             hitPoint + N * scene.epsilon;
                Vector3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1);
                Vector3f refractionColor = castRay(refractionRayOrig, refractionDirection, scene, depth + 1);
                float kr = fresnel(dir, N, payload->hit_obj->ior); //计算反射系数，即为能量比
                hitColor = reflectionColor * kr + refractionColor * (1 - kr); //根据反射比，进行加权平均
                break;
            }
            case REFLECTION: //材质类型：反射
            {
                float kr = fresnel(dir, N, payload->hit_obj->ior);
                Vector3f reflectionDirection = reflect(dir, N);
                // 交点沿着法线方向微小移动是为了什么？ 防止光线打回来造成死循环？
                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                                             hitPoint + N * scene.epsilon :
                                             hitPoint - N * scene.epsilon;
                hitColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1) * kr;
                break;
            }
            default: //材质类型：DIFFUSE_AND_GLOSSY  漫反射、光滑表面 (采用phong光照模型)
            {
                // [comment]
                // We use the Phong illumation model int the default case. The phong model
                // is composed of a diffuse and a specular reflection component.
                // [/comment]
                Vector3f lightAmt = 0, specularColor = 0;
                Vector3f shadowPointOrig = (dotProduct(dir, N) < 0) ?
                                           hitPoint + N * scene.epsilon :
                                           hitPoint - N * scene.epsilon;
                // [comment]
                // Loop over all lights in the scene and sum their contribution up
                // We also apply the lambert cosine law
                // [/comment]
                for (auto& light : scene.get_lights()) { //遍历光源，计算漫反射和高光
                    Vector3f lightDir = light->position - hitPoint;
                    // square of the distance between hitPoint and the light
                    float lightDistance2 = dotProduct(lightDir, lightDir);
                    lightDir = normalize(lightDir);
                    float LdotN = std::max(0.f, dotProduct(lightDir, N));
                    // is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
                    auto shadow_res = trace(shadowPointOrig, lightDir, scene.get_objects());  //求交，如果有交点且交点在和光源连线的中间，则该点在阴影中
                    bool inShadow = shadow_res && (shadow_res->tNear * shadow_res->tNear < lightDistance2);

                    lightAmt += inShadow ? 0 : light->intensity * LdotN;
                    Vector3f reflectionDirection = reflect(-lightDir, N);

                    specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, dir)),
                        payload->hit_obj->specularExponent) * light->intensity;
                }

                hitColor = lightAmt * payload->hit_obj->evalDiffuseColor(st) * payload->hit_obj->Kd + specularColor * payload->hit_obj->Ks;
                break;
            }
        }
    }

    return hitColor;
}

// [comment]
// The main render function. This where we iterate over all pixels in the image, generate
// primary rays and cast these rays into the scene. The content of the framebuffer is
// saved to a file.
// [/comment]
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = std::tan(deg2rad(scene.fov * 0.5f));
    float imageAspectRatio = scene.width / (float)scene.height;

    // OpenGL投影坐标系，透视投影近平面的Z值为-1.0，fov为垂直方向(y)的可视角度
    float z_near = -1.0f;
    float persp_height = scale * (-z_near);
    float persp_width  = persp_height * imageAspectRatio;

    // Use this variable as the eye position to start your rays.
    Vector3f eye_pos(0);
    int m = 0;
    for (int j = 0; j < scene.height; ++j)
    {
        for (int i = 0; i < scene.width; ++i)
        {
            // generate primary ray direction
            float x;
            float y;
            // TODO: Find the x and y positions of the current pixel to get the direction
            // vector that passes through it.
            // Also, don't forget to multiply both of them with the variable *scale*, and
            // x (horizontal) variable with the *imageAspectRatio*   
            x = i * persp_width / scene.width;
            y = (scene.height-1 - j) * persp_height / scene.height;

            x -= persp_width/2.0;
            y -= persp_height/2.0;

            Vector3f dir = Vector3f(x, y, z_near); // Don't forget to normalize this direction!
            framebuffer[m++] = castRay(eye_pos, dir, scene, 0);
        }
        UpdateProgress(j / (float)scene.height);
    }

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
