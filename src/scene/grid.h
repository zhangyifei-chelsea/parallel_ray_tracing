#ifndef CGL_GRID_H
#define CGL_GRID_H

#include <utility>
#include <algorithm>

#include "CGL/CGL.h"

#include "pathtracer/ray.h"
#include "primitive.h"

namespace CGL {

/**
 * Axis-aligned grid.
 * An AABB is given by two positions in space, the min and the max. An addition
 * component, the extent of the bounding box is stored as it is useful in a lot
 * of the operations on bounding boxes.
 */
    struct Grid {

        Vector3D max;	    ///< min corner of the bounding box
        Vector3D min;	    ///< max corner of the bounding box
        std::vector<SceneObjects::Primitive*> objects;

        /**
         * Constructor.
         * The default constructor creates a new bounding box which contains no
         * points.
         */
        Grid() {
            max = Vector3D(-INF_D, -INF_D, -INF_D);
            min = Vector3D( INF_D,  INF_D,  INF_D);
        }

        /**
         * Constructor.
         * Creates a bounding box with given bounds.
         * \param min the min corner
         * \param max the max corner
         */
        Grid(const Vector3D min, const Vector3D max):
                min(min), max(max) { }

        /**
         * Ray - grid intersection.
         * Intersects ray with bounding box, does not store shading information.
         * \param r the ray to intersect with
         * \param t0 lower bound of intersection time
         * \param t1 upper bound of intersection time
         */
        bool intersect(const Ray& r, double& t0, double& t1) const;
    };

} // namespace CGL

#endif // CGL_BBOX_H
