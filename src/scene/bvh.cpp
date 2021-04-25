#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>
#include <vector>
#include <climits>
#include <cassert>

using namespace std;

namespace CGL {
    namespace SceneObjects {

        BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                           size_t max_leaf_size) {

            primitives = std::vector<Primitive *>(_primitives);
            root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
        }

        BVHAccel::~BVHAccel() {
            if (root)
                delete root;
            primitives.clear();
        }

        BBox BVHAccel::get_bbox() const { return root->bb; }

        void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
            if (node->isLeaf()) {
                for (auto p = node->start; p != node->end; p++) {
                    (*p)->draw(c, alpha);
                }
            } else {
                draw(node->l, c, alpha);
                draw(node->r, c, alpha);
            }
        }

        void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
            if (node->isLeaf()) {
                for (auto p = node->start; p != node->end; p++) {
                    (*p)->drawOutline(c, alpha);
                }
            } else {
                drawOutline(node->l, c, alpha);
                drawOutline(node->r, c, alpha);
            }
        }

        double calculate_cost(std::vector<Primitive *>::iterator start,
                              std::vector<Primitive *>::iterator end, double avg, int dimension) {
            BBox left, right, all;
            int left_count = 0, right_count = 0;
            for (auto i = start; i != end; i++) {
                BBox cur = (*i)->get_bbox();
                double cur_axis_position = cur.centroid()[dimension];
                if (cur_axis_position < avg) {
                    left.expand(cur);
                    left_count++;
                } else {
                    right.expand(cur);
                    right_count++;
                }
                all.expand(cur);
            }
            // C_trav + Prob(hit L) * Cost(L) + Prob(hit R) * Cost(R)
            double cost = left.surface_area() / all.surface_area() * left_count +
                          right.surface_area() / all.surface_area() * right_count;
            return cost;
        }

        BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                         std::vector<Primitive *>::iterator end,
                                         size_t max_leaf_size) {

            // TODO (Part 2.1):
            // Construct a BVH from the given vector of primitives and maximum leaf
            // size configuration. The starter code build a BVH aggregate with a
            // single leaf node (which is also the root) that encloses all the
            // primitives.
            BBox bbox;
            int count = 0; // count the number of elements in the current step

            for (auto p = start; p != end; p++) {
                BBox bb = (*p)->get_bbox();
                bbox.expand(bb);
                count = count + 1;
            }

            // leaf node
            if (count <= max_leaf_size) {
                BVHNode *node = new BVHNode(bbox);
                node->start = start;
                node->end = end;
                return node;
            }

            // have to split - compare three dimensions
            double min_cost = INT_MAX;
            int min_dimension = -1;
            double split_edge = -1;


            // need split
            for (int i = 0; i < 3; i++) {
                // get average of centroid
                double avg = 0;
                for (auto p = start; p < end; p++) {
                    // get location
                    BBox cur = (*p)->get_bbox();
                    double cur_axis_position = cur.centroid()[i];
                    avg = avg + cur_axis_position;
                }
                avg = avg / count; // split here

                double cost = calculate_cost(start, end, avg, i);

                if (cost < min_cost) {
                    min_cost = cost;
                    min_dimension = i;
                    split_edge = avg;
                }
            }

            std::vector<Primitive *>::iterator split = partition(start, end,
                                                                 [split_edge, min_dimension](Primitive *p) {
                                                                     return p->get_bbox().centroid()[min_dimension] <
                                                                            split_edge;
                                                                 });
            int countLeft = 0;
            int countRight = 0;
            for (auto p = start; p != split; p++) {
                countLeft++;
            }
            for (auto p = split; p != end; p++) {
                countRight++;
            }
            if ((countLeft == 0) || (countRight == 0)) {
                BVHNode *node = new BVHNode(bbox);
                node->start = start;
                node->end = end;
                return node;
            }

            BVHNode *left = construct_bvh(start, split, max_leaf_size);
            BVHNode *right = construct_bvh(split, end, max_leaf_size);

            // other internal nodes
            BVHNode *node = new BVHNode(bbox);
            node->l = left;
            node->r = right;
            return node;
        }


        bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
            // TODO (Part 2.3):
            // Fill in the intersect function.
            // Take note that this function has a short-circuit that the
            // Intersection version cannot, since it returns as soon as it finds
            // a hit, it doesn't actually have to find the closest hit.
            BBox b = node->bb;
            double t0 = ray.min_t, t1 = ray.max_t;
            if(!b.intersect(ray, t0, t1)) {
                return false;
            }
            if (node->isLeaf()) {
                for (auto p = node->start; p != node->end; p++) {
                    ++total_isects;
                    if ((*p)->has_intersection(ray)) {
                        return true;
                    }
                }
                return false;
            }
            return has_intersection(ray, node->l) || has_intersection(ray, node->r);
        }

        double compute_min(const Ray &r, BBox b) {
            // x,y,z - axis
            double tx_min = (b.min.x - r.o.x) / r.d.x;
            double ty_min = (b.min.y - r.o.y) / r.d.y;
            double tz_min = (b.min.z - r.o.z) / r.d.z;

            // same as 2D
            // min of the max and max of the min
            double tmin = std::max(std::max(tx_min, ty_min), tz_min);
            return tmin;
        }

        Vector3D get_location(const Ray &ray, double t) {
            double x = t * ray.d.x + ray.o.x;
            double y = t * ray.d.y + ray.o.y;
            double z = t * ray.d.z + ray.o.z;
            return Vector3D(x, y, z);
        }

        bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
            // TODO (Part 2.3):
            // Fill in the intersect function.
            // need global comparison for i
            BBox b = node->bb;
            double t0 = ray.min_t, t1 = ray.max_t;
            if(!b.intersect(ray, t0, t1)) {
                return false;
            }
            if (node->isLeaf()) {
                // test intersection with all objects
                // return closest
                bool hit = false;
                for (auto p = node->start; p != node->end; p++) {
                    ++total_isects;
                    if ((*p)->has_intersection(ray)) {
                        if (i->primitive == NULL) {
                            (*p)->intersect(ray, i);
                        } else if (ray.max_t < i->t) {
                            (*p)->intersect(ray, i);
                        }
                        hit = true;
                    }
                }
                return hit;
            }
            bool hit1 = intersect(ray, i, node->l);
            bool hit2 = intersect(ray, i, node->r);
            return hit1 || hit2;
        }

    } // namespace SceneObjects
} // namespace CGL
