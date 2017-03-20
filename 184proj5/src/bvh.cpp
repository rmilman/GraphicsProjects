#include "bvh.h"

#include "CGL/CGL.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL { namespace StaticScene {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  root = construct_bvh(_primitives, max_leaf_size);

}

BVHAccel::~BVHAccel() {
  if (root) delete root;
}

BBox BVHAccel::get_bbox() const {
  return root->bb;
}

void BVHAccel::draw(BVHNode *node, const Color& c) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->draw(c);
  } else {
    draw(node->l, c);
    draw(node->r, c);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color& c) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->drawOutline(c);
  } else {
    drawOutline(node->l, c);
    drawOutline(node->r, c);
  }
}

BVHNode *BVHAccel::construct_bvh(const std::vector<Primitive*>& prims, size_t max_leaf_size) {
  
  // TODO Part 2, task 1:
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  // step 1
  BBox centroid_box, bbox;
  for (Primitive *p : prims) {
    BBox bb = p->get_bbox();
    bbox.expand(bb);
    Vector3D c = bb.centroid();
    centroid_box.expand(c);
  }
  // You'll want to adjust this code.
  // Right now we just return a single node containing all primitives.

  // step 2
  BVHNode *node = new BVHNode(bbox);

  // step 3
  if (prims.size() <= max_leaf_size) {
    node->prims = new vector<Primitive *>(prims);
    return node;
  }

  // step 4
  Vector3D extent = centroid_box.extent;
  int axis = 0;
  if (extent.y > extent.x) {
    axis = 1;
  }
  if (extent.z > extent.x && extent.z > extent.y) {
    axis = 2;
  }
  // step 5
  double center = centroid_box.centroid()[axis];
  // step 6
  std::vector<Primitive*> left, right;
  for (Primitive *p : prims) {
    if (p->get_bbox().centroid()[axis] < center) {
      left.push_back(p);
    } else {
      right.push_back(p);
    }
  }
  // step 7
  if (left.size() == 0 || right.size() == 0) {
    node->prims = new vector<Primitive *>(prims);
    return node;
  } 
  BVHNode *leftnode, *rightnode;
  leftnode = construct_bvh(left, max_leaf_size);
  rightnode = construct_bvh(right, max_leaf_size);
  node->l = leftnode;
  node->r = rightnode;
  // step 8
  return node;
  
}


bool BVHAccel::intersect(const Ray& ray, BVHNode *node) const {

  // TODO Part 2, task 3:
  // Implement BVH intersection.
  // Currently, we just naively loop over every primitive.
  double mi = ray.min_t, ma = ray.max_t;
  if (node->bb.intersect(ray, mi, ma) == false) {
    return false;
  }
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims)) {
        total_isects++;
        if (p->intersect(ray)) 
          return true;
    }
    return false;
  }
  bool hasIntersect = (intersect(ray, node->r) || intersect(ray, node->l));
  return hasIntersect;
}


bool BVHAccel::intersect(const Ray& ray, Intersection* i, BVHNode *node) const {

  // TODO Part 2, task 3:
  // Implement BVH intersection.
  // Currently, we just naively loop over every primitive.
  double mi = ray.min_t, ma = ray.max_t;
  if (node->bb.intersect(ray, mi, ma) == false) {
    return false;
  }
  bool hasIntersect = false;
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims)) {
        total_isects++;
        if (p->intersect(ray, i)) 
          hasIntersect = true;
    }
    if (hasIntersect) return true;
    return false;
  }
  hasIntersect = intersect(ray, i, node->r);
  hasIntersect = (intersect(ray, i, node->l) || hasIntersect);
  return hasIntersect;

}

}  // namespace StaticScene
}  // namespace CGL