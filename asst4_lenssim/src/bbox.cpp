#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO Part 2, task 2:
  // Implement ray - bounding box intersection test
  // If the ray intersected the bounding box within the range given by
  // t0, t1, update t0 and t1 with the   intersection times.
  // if (r.o[0]<=max[0] && r.o[0]>=min[0] && r.o[1]<=max[1] && r.o[1]>=min[1] && r.o[2]<=max[2] && r.o[2]>=min[2]){
  //   t0 = 0;
  //   double t0x;
  //   double t0y;
  //   double t0z;
  //   if (r.d[0] >= 0) {
  //     t0x = (min[0]-r.o[0])/r.d[0];
  //   }else{
  //     t0x = (max[0]-r.o[0])/r.d[0];
  //   }
  //   if (r.d[1] >= 0) {
  //     t0y = (min[1]-r.o[1])/r.d[1];
  //   }else{
  //     t0y = (max[1]-r.o[1])/r.d[1];
  //   }
  //   if (r.d[2] >= 0) {
  //     t0z = (min[2]-r.o[2])/r.d[2];
  //   }else{
  //     t0z = (max[2]-r.o[2])/r.d[2];
  //   }
  //   t1 = std::min(std::min(t0x, t0y), t0z);
  //   return true;
  //   }

  double t0x = (min[0]-r.o[0])/r.d[0];
  double t0y = (min[1]-r.o[1])/r.d[1];
  double t0z = (min[2]-r.o[2])/r.d[2];
  double t1x = (max[0]-r.o[0])/r.d[0];
  double t1y = (max[1]-r.o[1])/r.d[1];
  double t1z = (max[2]-r.o[2])/r.d[2];
  if (r.d[0] < 0) {
    double temp = t0x;
    t0x = t1x;
    t1x = temp;
  }
  if (r.d[1] < 0) {
    double temp = t0y;
    t0y = t1y;
    t1y = temp;
  }
  if (r.d[2] < 0) {
    double temp = t0z;
    t0z = t1z;
    t1z = temp;
  }
  if ((t0x > t1y) || (t0y > t1x))
    return false;
  if (t0y > t0x)
      t0x = t0y;
  if (t1y < t1x)
      t1x = t1y;
  if ((t0x > t1z) || (t0z > t1x))
    return false;
  if (t0z > t0x)
     t0x = t0z;
  if (t1z < t1x)
    t1x = t1z;

  t0 = t0x;
  t1 = t1x;
  return ((t0x < r.max_t) && (t1x > r.min_t));//( (t0x < t1) && (tmax > t0) );
  // double tempt0 = std::max(std::max(t0x,t0y),t0z);
  // double tempt1 = std::min(std::min(t1x,t1y),t1z);
  // if (tempt0 < r.min_t || tempt1-tempt0 <= 0){ //tempt0 < r.min_t || tempt1-tempt0 <= 0
  //   return false;
  // }
  // t0 = tempt0;
  // t1 = tempt1;
  // return true;
 
}

void BBox::draw(Color c) const {

  glColor4f(c.r, c.g, c.b, c.a);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
