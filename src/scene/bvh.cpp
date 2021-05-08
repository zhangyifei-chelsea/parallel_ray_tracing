#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>
#include <limits>
#include <omp.h>
#include "bbox.h"


using namespace std;

namespace CGL {
    namespace SceneObjects {

        BVHAccel::BVHAccel(const std::vector<Primitive*>& _primitives,
                           size_t max_leaf_size) {

            primitives = std::vector<Primitive*>(_primitives);
            size_t num_threads = omp_get_num_threads();
            root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size, 0, num_threads - 1);
        }

        BVHAccel::~BVHAccel() {
            if (root)
                delete root;
            primitives.clear();
        }

        BBox BVHAccel::get_bbox() const { return root->bb; }

        void BVHAccel::draw(BVHNode* node, const Color& c, float alpha) const {
            if (node->isLeaf()) {
                for (auto p = node->start; p != node->end; p++) {
                    (*p)->draw(c, alpha);
                }
            }
            else {
                draw(node->l, c, alpha);
                draw(node->r, c, alpha);
            }
        }

        void BVHAccel::drawOutline(BVHNode* node, const Color& c, float alpha) const {
            if (node->isLeaf()) {
                for (auto p = node->start; p != node->end; p++) {
                    (*p)->drawOutline(c, alpha);
                }
            }
            else {
                drawOutline(node->l, c, alpha);
                drawOutline(node->r, c, alpha);
            }
        }

        BVHNode* BVHAccel::construct_bvh(std::vector<Primitive*>::iterator start,
                                         std::vector<Primitive*>::iterator end,
                                         size_t max_leaf_size, size_t l_thread, size_t r_thread) {

            // TODO (Part 2.1):
            // Construct a BVH from the given vector of primitives and maximum leaf
            // size configuration. The starter code build a BVH aggregate with a
            // single leaf node (which is also the root) that encloses all the
            // primitives.
            if (omp_get_thread_num() == l_thread)
            {
                BBox bbox, midBox;
                int len = 0;

                for (auto p = start; p != end; p++) {
                    BBox bb = (*p)->get_bbox();
                    bbox.expand(bb);
                    midBox.expand(bb.centroid());
                    len++;
                }

                BVHNode* new_node = new BVHNode(bbox);
                if (len <= max_leaf_size) {
                    new_node->start = start;
                    new_node->end = end;
                }
                else {
                    int ax;
                    if (bbox.extent.x >= bbox.extent.y && bbox.extent.x >= bbox.extent.z)
                        ax = 0;
                    else if (bbox.extent.y >= bbox.extent.x && bbox.extent.y >= bbox.extent.z)
                        ax = 1;
                    else
                        ax = 2;

                    Vector3D midPoint = midBox.centroid();

                    int leftLen = 0, rightLen = 0;
                    for (auto p = start; p != end; p++) {
                        if ((*p)->get_bbox().centroid()[ax] <= midPoint[ax])
                            leftLen++;
                        else
                            rightLen++;
                    }
                    if (leftLen == 0 || rightLen == 0) {
                        new_node->start = start;
                        new_node->end = end;
                        return new_node;
                    }


                    vector<Primitive*>* leftNodes = new vector<Primitive*>(), * rightNodes = new vector<Primitive*>();
                    for (auto p = start; p != end; p++) {
                        if ((*p)->get_bbox().centroid()[ax] <= midPoint[ax])
                            leftNodes->push_back(*p);
                        else
                            rightNodes->push_back(*p);
                    }
                    if (l_thread != r_thread)
                    {
                        size_t mid = (l_thread + r_thread) / 2;
                        new_node->l = construct_bvh(leftNodes->begin(), leftNodes->end(), max_leaf_size, l_thread, mid);
                        new_node->r = construct_bvh(rightNodes->begin(), rightNodes->end(), max_leaf_size, mid + 1, r_thread);
                    }
                    else
                    {
                        new_node->l = construct_bvh(leftNodes->begin(), leftNodes->end(), max_leaf_size, l_thread, r_thread);
                        new_node->r = construct_bvh(rightNodes->begin(), rightNodes->end(), max_leaf_size, l_thread, r_thread);
                    }
                }

                return new_node;
            }
        }


        bool BVHAccel::has_intersection(const Ray& ray, BVHNode* node, size_t l_thread, size_t r_thread) const {
            // TODO (Part 2.3):
            // Fill in the intersect function.
            // Take note that this function has a short-circuit that the
            // Intersection version cannot, since it returns as soon as it finds
            // a hit, it doesn't actually have to find the closest hit.
            if (omp_get_thread_num() == l_thread)
            {
                BBox bb = node->bb;
                double t0 = ray.min_t, t1 = ray.max_t;

                if (!bb.intersect(ray, t0, t1))
                    return false;
                if (t0 > ray.max_t || t1 < ray.min_t)
                    return false;
                else {
                    if (node->isLeaf()) {
                        for (auto p = node->start; p != node->end; ++p) {
                            //total_isects++;
                            omp_set_lock(&(ray.lock));
                            if ((*p)->has_intersection(ray))
                                return true;
                            omp_unset_lock(&(ray.lock));
                        }
                        return false;
                    }
                    else {
                        if (l_thread != r_thread)
                        {
                            size_t mid = (l_thread + r_thread) / 2;
                            bool ll = has_intersection(ray, node->l, l_thread, mid);
                            if (ll == true) return true;
                            bool rr = has_intersection(ray, node->r, mid + 1, r_thread);
                            if (rr == true) return true;
                            else return false;
                        }
                        else
                        {
                            return has_intersection(ray, node->l, l_thread, r_thread) || has_intersection(ray, node->r, l_thread, r_thread);
                        }
                    }
                }
                return false;
            }
        }

        bool BVHAccel::intersect(const Ray& ray, Intersection* i, BVHNode* node, size_t l_thread, size_t r_thread, omp_lock_t lock) const {
            // TODO (Part 2.3):
            // Fill in the intersect function.
            if (omp_get_thread_num() == l_thread)
            {
                BBox bb = node->bb;
                double t0 = ray.min_t, t1 = ray.max_t;

                if (!bb.intersect(ray, t0, t1))
                    return false;
                if (t0 > ray.max_t || t1 < ray.min_t)
                    return false;
                else {
                    if (node->isLeaf()) {
                        bool isHit = false;
                        for (auto p = node->start; p != node->end; p++) {
                            omp_set_lock(&lock);
                            total_isects++;
                            Intersection temp_i;
                            if ((*p)->intersect(ray, &temp_i)) {
                                isHit = true;
                                if (temp_i.t < i->t) {
                                    i->t = temp_i.t;
                                    i->n = temp_i.n;
                                    i->bsdf = temp_i.bsdf;
                                    i->primitive = temp_i.primitive;
                                }
                            }
                            omp_unset_lock(&lock);
                        }
                        return isHit;
                    }
                    else {
                        if (l_thread != r_thread)
                        {
                            size_t mid = (l_thread + r_thread) / 2;
                            bool isHit_l = intersect(ray, i, node->l, l_thread, mid, lock);
                            bool isHit_r = intersect(ray, i, node->r, mid + 1, r_thread, lock);
                            return isHit_l || isHit_r;
                        }
                        else
                        {
                            bool isHit_l = intersect(ray, i, node->l, l_thread, r_thread, lock);
                            bool isHit_r = intersect(ray, i, node->r, l_thread, r_thread, lock);
                        }
                    }
                }
            }
        }
    } // namespace SceneObjects
} // namespace CGL
