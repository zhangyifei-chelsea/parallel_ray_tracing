#include "grids.h"

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

        GridAccel::GridAccel(const std::vector<Primitive *> &_primitives, size_t count) {

            primitives = std::vector<Primitive *>(_primitives);
            construct_grid(primitives.begin(), primitives.end(), count);
        }

        GridAccel::~GridAccel() {

        }


        void GridAccel::construct_grid(std::vector<Primitive *>::iterator start,
                                         std::vector<Primitive *>::iterator end,
                                         size_t count) {
            // 1. Find bounding box
            Vector3D grid_boundary_min(INF_D, INF_D, INF_D), grid_boundary_max(-INF_D, -INF_D, -INF_D);
            for (auto i = start; i != end; i++) {
                BBox cur = (*i)->get_bbox();
                grid_boundary_min.x = min(grid_boundary_min.x, cur.min.x);
                grid_boundary_min.y = min(grid_boundary_min.y, cur.min.y);
                grid_boundary_min.z = min(grid_boundary_min.z, cur.min.z);
                grid_boundary_max.x = max(grid_boundary_max.x, cur.max.x);
                grid_boundary_max.y = max(grid_boundary_max.y, cur.max.y);
                grid_boundary_max.z = max(grid_boundary_max.z, cur.max.z);
            }

            boundary = Grid(grid_boundary_min, grid_boundary_max);
            cout << "Grid boundary min: " << boundary.min << endl;
            cout << "Grid boundary min: " << grid_boundary_min << endl;
            cout << "Grid boundary max: " << boundary.max << endl;
            cout << "Grid boundary max: " << grid_boundary_max << endl;

            grid_count = std::cbrt(count);

            cout << "Grid count: " << grid_count << endl;

            grid_size = (grid_boundary_max - grid_boundary_min) / grid_count;

            cout << "Grid size: " << grid_size << endl;

            for(int i = 0; i < grid_count; ++i) {
                for(int j = 0; j < grid_count; ++j) {
                    for(int k = 0; k < grid_count; ++k) {
                        Vector3D grid_min = grid_boundary_min + Vector3D(grid_size.x * i, grid_size.y * j, grid_size.z * k);
                        Vector3D grid_max = grid_min + grid_size;
                        Grid* g = new Grid(grid_min, grid_max);
                        grids.push_back(g);
                    }
                }
//                cout << grids.back()->min << " " << grids.back()->max << endl;
            }

            cout << grids[(grid_count - 1) * grid_count * grid_count + (grid_count - 1) * grid_count + (grid_count - 1)]->max << endl;

            for (auto i = start; i != end; i++) {
                // if the bounding box of the primitive intersects with a grid, push it into the grid
                BBox cur = (*i)->get_bbox();
                Vector3D min_coord = (cur.min - grid_boundary_min) / grid_size;
                Vector3D max_coord = (cur.max - grid_boundary_min) / grid_size;
                for(int x = (int) min_coord.x; x < max_coord.x; ++x) {
                    for(int y = (int) min_coord.y; y < max_coord.y; ++y) {
                        for(int z = (int) min_coord.z; z < max_coord.z; ++z) {
                            grids[x * grid_count * grid_count + y * grid_count + z]->objects.push_back(*i);
                        }
                    }
                }
            }

        }

        bool GridAccel::intersect(const Ray &ray, Intersection *i) const {
            // Fill in the intersect function.
            // referring https://www.scratchapixel.com/lessons/advanced-rendering/introduction-acceleration-structure/grid
            // check whether ray intersect with boundary
            double min_t = 0.0, max_t = INT_MAX;
            if (!boundary.intersect(ray, min_t, max_t)) {
                return false;
            }

            Vector3D exit, step;

            // calculate ray's starting grid
            ray.min_t = min_t;
            ray.max_t = max_t;
//            cout << ray.min_t << " " << ray.max_t << endl;
            Vector3D ray_start = ray.o + ray.min_t * ray.d;
//            cout << "ray start: " << ray_start << endl;
            Vector3D start_grid = (ray_start - boundary.min) / grid_size;
//            cout << "start grid: " << start_grid << endl;
            Vector3D deltaT, nextCrossingT;
            Vector3D cell((int)start_grid.x, (int)start_grid.y, (int)start_grid.z);
            if(cell.x >= grid_count) {
                cell.x = 90;
            }
            if(cell.y >= grid_count) {
                cell.y = 90;
            }
            if(cell.z >= grid_count) {
                cell.z = 90;
            }
//            cout << "start cell: " << cell << endl;
            for (int dim = 0; dim < 3; ++dim) {
                if (ray.d[dim] < 0) {
                    deltaT[dim] = - grid_size[dim] / ray.d[dim];
                    nextCrossingT[dim] = ray.min_t + (cell[dim] * grid_size[dim] - (ray_start - boundary.min)[dim]) / ray.d[dim];
                    exit[dim] = -1;
                    step[dim] = -1;
                }
                else {
                    deltaT[dim] = grid_size[dim] / ray.d[dim];
                    nextCrossingT[dim] = ray.min_t + ((cell[dim] + 1) * grid_size[dim] - (ray_start - boundary.min)[dim]) / ray.d[dim];
                    exit[dim] = grid_size[dim];
                    step[dim] = 1;
                }
            }

            // walk through each cell of the grid and test for an intersection if
            // current cell contains geometry
            bool hit = false;
            int loop = 0;
            while (true) {
                loop ++;
                Grid* g = grids[(int)cell.x * grid_count * grid_count + (int)cell.y * grid_count + (int)cell.z];
                if(g == NULL)
                    return false;
                for(auto p:g->objects) {
                    ++total_isects;
                    if (p->has_intersection(ray)) {
                        if (i->primitive == NULL) {
                            p->intersect(ray, i);
                        } else if (ray.max_t < i->t) {
                            p->intersect(ray, i);
                        }
                        // check whether the intersection is inside the grid
                        Vector3D hit_point = ray.o + ray.max_t * ray.d;
                        if(hit_point.x > g->min.x && hit_point.x < g->max.x &&
                        hit_point.y > g->min.y && hit_point.y < g->max.y &&
                        hit_point.z > g->min.z && hit_point.z < g->max.z) {
                            hit = true;
                        }
                        else {
                            continue;
                        }

                    }
                }
                if(hit)
                    return hit;
                uint8_t k =
                        ((nextCrossingT[0] < nextCrossingT[1]) << 2) +
                        ((nextCrossingT[0] < nextCrossingT[2]) << 1) +
                        ((nextCrossingT[1] < nextCrossingT[2]));
                static const uint8_t map[8] = {2, 1, 2, 1, 2, 2, 0, 0};
                uint8_t axis = map[k];
                if (ray.max_t < nextCrossingT[axis]) break;
                cell[axis] += step[axis];
                if (cell[axis] == exit[axis]) break;
                nextCrossingT[axis] += deltaT[axis];
            }

            return false;

        }

    } // namespace SceneObjects
} // namespace CGL
