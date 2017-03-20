#include "sphere.h"

#include <cmath>

#include  "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CGL { namespace StaticScene {

bool Sphere::test(const Ray& r, double& t1, double& t2) const {

  // TODO Part 1, task 4:
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  double a = dot(r.d, r.d);
  double b = dot(2. * (r.o - o), r.d);
  double c = dot(r.o - o, r.o - o) - r2;
  double b24ac = pow(b, 2.) - 4*a*c;
  if (b24ac < 0) {
    return false;
  }
  double sqrtb24ac = sqrt(b24ac);
  double ta = (-b + sqrtb24ac) / (2. * a);
  double tb = (-b - sqrtb24ac) / (2. * a);
  double maxt = max(ta, tb), mint = min(ta, tb);
  bool mint_valid = true, maxt_valid = true;
  if ( mint < 0 || mint > r.max_t || mint < r.min_t) {
    mint_valid = false;
  }
  if (maxt < 0 || maxt > r.max_t || maxt < r.min_t) {
    maxt_valid = false;
  }
  if (!mint_valid && !maxt_valid) {
    return false;
  } else if (mint_valid && maxt_valid) {
    t1 = mint;
    t2 = maxt;
  } else if (mint_valid) {
    t1 = mint;
  } else {
    t1 = maxt;
  }
  return true;

}

bool Sphere::intersect(const Ray& r) const {

  // TODO Part 1, task 4:
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  double t1, t2;
  if (test(r, t1, t2) == false) {
    return false;
  }
  r.max_t = t1;
  return true;

}

bool Sphere::intersect(const Ray& r, Intersection *i) const {

  // TODO Part 1m task 4:
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  double t1, t2;
  if (test(r, t1, t2) == false) {
    return false;
  }
  Vector3D intersectPoint = r.o + t1*r.d;
  Vector3D normal = intersectPoint - o;
  normal.normalize();
  r.max_t = t1;
  i->n = normal;
  i->t = t1;
  i->primitive = this;
  if (object != NULL) {
    i->bssrdf = get_bssrdf();
    i->bsdf = get_bsdf();
  }
  return true;
}

void Sphere::draw(const Color& c) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color& c) const {
    //Misc::draw_sphere_opengl(o, r, c);
}


} // namespace StaticScene
} // namespace CGL
