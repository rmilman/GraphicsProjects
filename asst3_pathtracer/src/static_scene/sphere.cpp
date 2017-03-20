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
  double a = dot(r.d,r.d);
  double b = dot(2*(r.o-o), r.d);
  double c = dot((r.o-o),(r.o-o))-r2;
  double d = b*b-4*a*c;
  if (d<0){
    return false;
  }
  double tp = (-b+sqrt(d))/(2*a);
  double tm = (-b-sqrt(d))/(2*a);
  t1 = min(tp,tm);
  t2 = max(tp,tm);
  // if (t1 <= r.min_t){
  //   t1 = t2;
  //   if (t2 <= r.min_t){
  //    return false;
  //   }
  // }
  return true;
}

bool Sphere::intersect(const Ray& r) const {

  // TODO Part 1, task 4:
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
 double t1 = 0;
 double t2 = 0;

 if (test(r, t1, t2)){

    if (t1 < r.min_t || t1 > r.max_t){
      if (t2 < r.min_t || t2 > r.max_t){
        return false;  
      }else{
        r.max_t = t2;
        return true;
      }
    } else {
      r.max_t = t1;
      return true;
    }
 } else {
  return false;
 }
}

bool Sphere::intersect(const Ray& r, Intersection *i) const {

  // TODO Part 1m task 4:
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  double t1 = 0;
  double t2 = 0;
  if (test(r, t1, t2)){
    // if (t1 < r.min_t || t2 > r.max_t){ //debug journey red on spheres || t2 < r.min_t
    //   return false;
    // } else {
    //   r.max_t = t1;
    //   i->t = t1;
    //   i->n = normal(r.o+t1*r.d);
    //   i->primitive = this;
    //   i->bsdf = get_bsdf();
    //   return true;
    // }
    if (t1 < r.min_t || t1 > r.max_t){
      if (t2 < r.min_t || t2 > r.max_t){
        return false;  
      }else{
        r.max_t = t2;
        i->t = t2;
        i->n = normal(r.o+t2*r.d);
        i->primitive = this;
        i->bsdf = get_bsdf();
        return true;
      }
    } else {
      r.max_t = t1;   
      i->t = t1;
      i->n = normal(r.o+t1*r.d);
      i->primitive = this;
      i->bsdf = get_bsdf();
      return true;
    }
  } else {
    return false;
  }
}

void Sphere::draw(const Color& c) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color& c) const {
    //Misc::draw_sphere_opengl(o, r, c);
}


} // namespace StaticScene
} // namespace CGL
