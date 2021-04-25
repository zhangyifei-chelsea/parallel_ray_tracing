#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  double a = dot(r.d, r.d);
  double b = 2 * dot(r.o - this->o, r.d);
  double c = dot(r.o - this->o, r.o - this->o) - this->r2;
  double delta = b * b - 4.0 * a * c;
  if(delta < 0)
      return false;
  t1 = (-b - sqrt(delta)) / 2.0 / a;
  t2 = (-b + sqrt(delta)) / 2.0 / a;
  if(t1 >= r.min_t && t1 <= r.max_t)
      return true;
  else if(t2 >= r.min_t && t2 <= r.max_t) {
      t1 = t2;
      return true;
  }
  return false;
}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  double t1, t2;
  bool has_inter = test(r, t1, t2);
  if(has_inter)
      r.max_t = t1;
  return has_inter;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
    double t1, t2;
    bool has_inter = test(r, t1, t2);
    if(has_inter) {
        r.max_t = t1;
        i->primitive = this;
        i->t = t1;
        i->n = (r.o + t1 * r.d - this->o).unit();
        i->bsdf = this->get_bsdf();
    }
    return has_inter;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
