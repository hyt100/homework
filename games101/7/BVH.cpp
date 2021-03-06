#include <algorithm>
#include <cassert>
#include <chrono>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    if (primitives.empty())
        return;

    auto start = std::chrono::system_clock::now();
    root = recursiveBuild(primitives);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Generation complete:  takes " 
              << std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() << " ms\n";
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        // node->object = objects[0];
        node->objects.emplace_back(objects[0]);
        node->left = nullptr;
        node->right = nullptr;
        node->area = objects[0]->getArea();
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }
        node->splitAxis = dim; //?????????????????????????????????

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray)
{
    Intersection isect;
    if (!root)
        return isect;
    Ray ray_copy = ray;
    isect = BVHAccel::getIntersection(root, ray_copy);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, Ray& ray)
{
    // TODO Traverse the BVH to find intersection
    float t;
    if (!node->bounds.IntersectP(ray, t) || t > ray.t_max) { //????????????????????????t?????????????????????
        return Intersection();
    }
    
    if (node->left == nullptr && node->right == nullptr) {
        Intersection isect_closed;
        for (auto &obj: node->objects) {
            Intersection isect = obj->getIntersection(ray);
            if (isect.happened && isect.distance < ray.t_max) {
                ray.t_max = isect.distance; //????????????????????????????????????ray???t_max?????????????????????t_max?????????????????????????????????
                isect_closed = isect;
            }
        }
        return isect_closed;
    }

    // ????????????: ????????????????????????????????????????????????????????????ray?????????(????????????t_max??????),
    //    ??????ray???axis???????????????????????????child1->child2???????????????????????????child2->child1?????????
    Intersection isect_left, isect_right;
    if (ray.dirIsNeg[node->splitAxis]) {
        isect_right = getIntersection(node->right, ray);
        isect_left = getIntersection(node->left, ray);
    } else {
        isect_left = getIntersection(node->left, ray);
        isect_right = getIntersection(node->right, ray);
    }
    return isect_left.distance <= isect_right.distance ? isect_left : isect_right;
}


void BVHAccel::getSample(BVHBuildNode* node, float p, Intersection &pos, float &pdf){
    if(node->left == nullptr || node->right == nullptr){
        node->objects[0]->Sample(pos, pdf);
        pdf *= node->area;
        return;
    }
    if(p < node->left->area) getSample(node->left, p, pos, pdf);
    else getSample(node->right, p - node->left->area, pos, pdf);
}

void BVHAccel::Sample(Intersection &pos, float &pdf){
    float p = std::sqrt(get_random_float()) * root->area;
    getSample(root, p, pos, pdf);
    pdf /= root->area;
}