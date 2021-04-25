#include "grid.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

    bool Grid::intersect(const Ray& r, double& t0, double& t1) const {
        // Implement ray - grid intersection test
        // If the ray intersected the grid within the range given by
        // t0, t1, update t0 and t1 with the new intersection times.
        if(t0 > t1) { // just in case
            return false;
        }

        // x,y,z - axis
        double tx_min = (min.x - r.o.x) / r.d.x;
        double tx_max = (max.x - r.o.x) / r.d.x;
        if (tx_min > tx_max)
            std::swap(tx_min, tx_max);
        double ty_min = (min.y - r.o.y) / r.d.y;
        double ty_max = (max.y - r.o.y) / r.d.y;
        if (ty_min > ty_max)
            std::swap(ty_min, ty_max);
        double tz_min = (min.z - r.o.z) / r.d.z;
        double tz_max = (max.z - r.o.z) / r.d.z;
        if (tz_min > tz_max)
            std::swap(tz_min, tz_max);

        // same as 2D
        // min of the max and max of the min
        double tmin = std::max(std::max(tx_min, ty_min), tz_min);
        double tmax = std::min(std::min(tx_max, ty_max), tz_max);
        if(tmin > tmax) {
            return false;
        }
        if(tmax < t0 || tmin > t1) {
            return false;
        }
        if (tmin >= t0 && tmax <= t1) {
            t0 = tmin;
            t1 = tmax;
        }
        else if(tmax <= t1) {
            t1 = tmax;
        }
        else if(tmin >= t0) {
            t0 = tmin;
        }
        return true;
    }
} // namespace CGL