#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

double maxi(double a, double b, double c) {
  if (a > b && a > c) {
    return a;
  }
  if (b > c && b > a) {
    return b;
  }
  return c;
}

double mini(double a, double b, double c) {
  if (a < b && a < c) {
    return a;
  }
  if (b < c && b < a) {
    return b;
  }
  return c;
}

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO Part 2, task 2:
  // Implement ray - bounding box intersection test
  // If the ray intersected the bounding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
  double pxmin = min.x, pxmax = max.x, pymin = min.y, pymax = max.y, pzmin = min.z, pzmax = max.z;
  double ox = r.o.x, oy = r.o.y, oz = r.o.z, dx = r.d.x, dy = r.d.y, dz = r.d.z;
  double tx1 = (pxmin - ox)/dx, ty1 = (pymin-oy)/dy, tz1 = (pzmin-oz)/dz;
  double tx2 = (pxmax - ox)/dx, ty2 = (pymax-oy)/dy, tz2 = (pzmax-oz)/dz;
  double txmin = tx1 < tx2 ? tx1 : tx2;
  double txmax = tx1 < tx2 ? tx2 : tx1;
  double tymin = ty1 < ty2 ? ty1 : ty2;
  double tymax = ty1 < ty2 ? ty2 : ty1;
  double tzmin = tz1 < tz2 ? tz1 : tz2;
  double tzmax = tz1 < tz2 ? tz2 : tz1;

  double tenter = maxi(txmin, tymin, tzmin);
  double texit = mini(txmax, tymax, tzmax);
  double tmin = t0 < tenter ? tenter : t0;
  double tmax = t1 > texit ? texit : t1;

  if (tmax < tmin || tmin < 0) {
    return false;
  }

  t0 = tmin;
  t1 = tmax;
  return true;
}

bool BBox::inside(Vector3D pt) {
  return (pt.x >= min.x && pt.x <= max.x && 
    pt.y >= min.y && pt.y <= max.y &&
    pt.z >= min.z && pt.z <= max.z);
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

  //printf("%f, %f, %f", c.r,c.g,c.b);
}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
