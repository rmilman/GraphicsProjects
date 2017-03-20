#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL { namespace StaticScene {

Triangle::Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3) :
    mesh(mesh), v1(v1), v2(v2), v3(v3) { }

BBox Triangle::get_bbox() const {

  Vector3D p1(mesh->positions[v1]), p2(mesh->positions[v2]), p3(mesh->positions[v3]);
  BBox bb(p1);
  bb.expand(p2); 
  bb.expand(p3);
  return bb;

}

bool Triangle::intersect(const Ray& r) const {
  // TODO Part 1, task 3: implement ray-triangle intersection
  Vector3D p1(mesh->positions[v1]), p2(mesh->positions[v2]), p3(mesh->positions[v3]);
  Vector3D E1 = p2 - p1,      E2 = p3 - p1,     S = r.o - p1,
           S1 = cross(r.d, E2), S2 = cross(S, E1);

  Vector3D intersectVec = Vector3D(dot(S2, E2), dot(S1, S), dot(S2, r.d));
  double scale = dot(S1, E1);
  if (scale == 0) {
    return false;
  }
  intersectVec *= 1./scale;
  double t = intersectVec.x;
  double b1 = intersectVec.y;
  double b2 = intersectVec.z;

  if (t > r.max_t || t < r.min_t || b1 < 0 || b2 < 0 || (1.-b1-b2) < 0) {
    return false;
  }
  r.max_t = t;
  return true;
}

bool Triangle::intersect(const Ray& r, Intersection *isect) const {
  
  // TODO Part 1, task 3:   
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
  Vector3D p1(mesh->positions[v1]), p2(mesh->positions[v2]), p3(mesh->positions[v3]);
  Vector3D n1(mesh->normals[v1]), n2(mesh->normals[v2]), n3(mesh->normals[v3]);

  Vector3D E1 = p2 - p1,      E2 = p3 - p1,     S = r.o - p1,
           S1 = cross(r.d, E2), S2 = cross(S, E1);

  Vector3D intersectVec = Vector3D(dot(S2, E2), dot(S1, S), dot(S2, r.d));
  double scale = dot(S1, E1);
  if (scale == 0) {
    return false;
  }
  intersectVec *= 1./scale;
  double t = intersectVec.x;
  double b1 = intersectVec.y;
  double b2 = intersectVec.z;

  if (t > r.max_t || t < r.min_t || b1 < 0 || b2 < 0 || (1.-b1-b2) < 0) {
    return false;
  }
  r.max_t = t;
  Vector3D normal = (1. - b1 - b2)*n1 + b1*n2 + b2*n3;
  normal.normalize();
  isect->n = normal;
  isect->t = t;
  isect->primitive = this;
  isect->bsdf = get_bsdf();
  isect->bssrdf = get_bssrdf();
  return true;
}

void Triangle::draw(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_TRIANGLES);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

void Triangle::drawOutline(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_LINE_LOOP);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}



} // namespace StaticScene
} // namespace CGL
