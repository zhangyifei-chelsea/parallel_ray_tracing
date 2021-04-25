#ifndef CGL_GRIDS_H
#define CGL_GRIDS_H

#include "scene.h"
#include "aggregate.h"
#include "grid.h"

#include <vector>

namespace CGL {
    namespace SceneObjects {

        class GridAccel : public Aggregate {
        public:

            GridAccel() {}


            GridAccel(const std::vector<Primitive *> &primitives, size_t count);

            /**
             * Destructor.
             * The destructor only destroys the Aggregate itself, the primitives that
             * it contains are left untouched.
             */
            ~GridAccel();

            BBox get_bbox() const { return BBox(); };

            bool has_intersection(const Ray& r) const {
                ++total_rays;
                return false;
            }


            bool intersect(const Ray &r, Intersection *i) const;

            /**
             * Get BSDF of the surface material
             * Note that this does not make sense for the accel aggregate
             * because it does not have a surface material. Therefore this
             * should always return a null pointer.
             */
            BSDF *get_bsdf() const { return NULL; }

            /**
             * Get entry point (root) - used in visualizer
             */
            void get_root() const {}


            void draw(const Color &c, float alpha) const {}


            void drawOutline(const Color &c, float alpha) const {}

            mutable unsigned long long total_rays, total_isects;

        private:
            std::vector<Primitive *> primitives;
            std::vector<Grid *> grids;  // 3D grids arranged as 1D
            Grid boundary;  // 3D grids
            int grid_count;
            Vector3D grid_size;

            void construct_grid(std::vector<Primitive *>::iterator start, std::vector<Primitive *>::iterator end,
                                size_t count);
        };

    } // namespace SceneObjects
} // namespace CGL

#endif // CGL_GRIDS_H
