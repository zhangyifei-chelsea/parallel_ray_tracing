#include "kdtree.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>
#include <limits>

using namespace std;

//In this time, we use the K-D tree structure instead of BVH.
//But to make the modification simple, we also keep the structure name as BVH in order to keep other parts of the program unchanged.
namespace CGL {
    namespace SceneObjects {

        KDTreeAccel::KDTreeAccel(const std::vector<Primitive*>& _primitives,
                           size_t max_leaf_size) {

            primitives = std::vector<Primitive*>(_primitives);
            root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
        }

        KDTreeAccel::~KDTreeAccel() {
            if (root)
                delete root;
            primitives.clear();
        }

        BBox KDTreeAccel::get_bbox() const { return root->bb; }

        void KDTreeAccel::draw(KDNode* node, const Color& c, float alpha) const {
            for (auto p = node->start; p != node->end; p++)
                (*p)->draw(c, alpha);

        }

        void KDTreeAccel::drawOutline(KDNode* node, const Color& c, float alpha) const {
            for (auto p = node->start; p != node->end; p++)
                (*p)->drawOutline(c, alpha);
        }

        KDNode* KDTreeAccel::construct_bvh(std::vector<Primitive*>::iterator start,
                                         std::vector<Primitive*>::iterator end,
                                         size_t max_leaf_size) {
            BBox bbox, midBox;
            int len = 0;
            for (auto p = start; p != end; p++) {
                BBox bb = (*p)->get_bbox();
                bbox.expand(bb);
                midBox.expand(bb.centroid());
                len++;
            }
            int p;
            return construct_kd(start, end, max_leaf_size, 0, bbox, &p);
        }
        KDNode* KDTreeAccel::construct_kd(std::vector<Primitive*>::iterator start,
                                        std::vector<Primitive*>::iterator end,
                                        size_t max_leaf_size, size_t deep, BBox bbfa, int* fnum) {

            // TODO (Part 2.1):
            // Construct a BVH from the given vector of primitives and maximum leaf
            // size configuration. The starter code build a BVH aggregate with a
            // single leaf node (which is also the root) that encloses all the
            // primitives.

            BBox bbox, midBox;
            int len = 0;
            for (auto p = start; p != end; p++) {
                BBox bb = (*p)->get_bbox();
                midBox.expand(bb.centroid());
                len++;
            }

            bbox = bbfa;
            KDNode* new_node = new KDNode(bbfa);
            if (len <= max_leaf_size) {
                new_node->start = start;
                new_node->end = end;
                *fnum = max(len, *fnum);
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
                    if ((*p)->get_bbox().min[ax] <= midPoint[ax])
                        leftLen++;
                    if ((*p)->get_bbox().max[ax] >= midPoint[ax])
                        rightLen++;
                }
                //printf("%d %d\n", leftLen, rightLen);
                //check if here all primitives belong to left and right.

                if (leftLen == 0 || rightLen == 0 || deep > 50 || leftLen == len || rightLen == len) {
                    if (len <= max_leaf_size * 10 || deep > 50)
                    {
                        new_node->start = start;
                        new_node->end = end;
                        *fnum = max(len, *fnum);
                        return new_node;
                    }
                    else
                    {
                        int tx = ax;
                        do {


                            leftLen = 0, rightLen = 0;
                            for (auto p = start; p != end; p++) {
                                if ((*p)->get_bbox().min[ax] <= midPoint[ax])
                                    leftLen++;
                                if ((*p)->get_bbox().max[ax] >= midPoint[ax])
                                    rightLen++;
                            }
                            //printf("%d %d\n", leftLen, rightLen);
                            //check if here all primitives belong to left and right.

                            if (leftLen == 0 || rightLen == 0 || deep > 50 || leftLen == len || rightLen == len) {
                                if (len <= max_leaf_size * 10 || deep > 50)
                                {
                                    new_node->start = start;
                                    new_node->end = end;
                                    *fnum = max(len, *fnum);
                                    return new_node;
                                }
                                else ax = (ax + 1) % 3;
                            }
                            else break;

                        } while (tx != ax);
                    }
                }


                BBox bbl, bbr;
                bbl = BBox(bbfa);
                bbl.max[ax] = midPoint[ax];
                bbl.extent = bbl.max - bbl.min;
                bbr = BBox(bbfa);
                bbr.min[ax] = midPoint[ax];
                bbr.extent = bbr.max - bbr.min;

                vector<Primitive*>* leftNodes = new vector<Primitive*>(), * rightNodes = new vector<Primitive*>();
                for (auto p = start; p != end; p++) {
                    if ((*p)->get_bbox().min[ax] <= midPoint[ax])
                        leftNodes->push_back(*p);
                    if ((*p)->get_bbox().max[ax] >= midPoint[ax])
                        rightNodes->push_back(*p);
                }
                int temp = 0;
                new_node->l = construct_kd(leftNodes->begin(), leftNodes->end(), max_leaf_size, deep + 1, bbl, &temp);
                new_node->r = construct_kd(rightNodes->begin(), rightNodes->end(), max_leaf_size, deep + 1, bbr, &temp);
                *fnum = max(*fnum, temp);
                printf("%d %d\n", deep, temp);
            }

            return new_node;
        }


        bool KDTreeAccel::has_intersection(const Ray& ray, KDNode* node) const {
            // TODO (Part 2.3):
            // Fill in the intersect function.
            // Take note that this function has a short-circuit that the
            // Intersection version cannot, since it returns as soon as it finds
            // a hit, it doesn't actually have to find the closest hit.

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
                        if ((*p)->has_intersection(ray))
                            return true;
                    }
                    return false;
                }
                else {
                    return has_intersection(ray, node->l) || has_intersection(ray, node->r);
                }
            }
            return false;
        }

        bool KDTreeAccel::intersect(const Ray& ray, Intersection* i, KDNode* node) const {
            // TODO (Part 2.3):
            // Fill in the intersect function.

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
                    }
                    return isHit;
                }
                else {
                    bool isHit_l = intersect(ray, i, node->l);
                    bool isHit_r = intersect(ray, i, node->r);
                    return isHit_l || isHit_r;
                }
            }
        }

    } // namespace SceneObjects
} // namespace CGL
