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

void BVHAccel::helperSplit(const std::vector<Primitive*>& prims, BVHNode* node, int axis, int level, int direction, int max_leaf_size) {
     // takes in a bbox finds lefties and righties, if either are zero repeats by splitting split point in half
      BBox bb = node->bb;
      std::vector<CGL::StaticScene::Primitive *> pLeft = std::vector<CGL::StaticScene::Primitive *>();
      std::vector<CGL::StaticScene::Primitive *> pRight = std::vector<CGL::StaticScene::Primitive *>();

      double size = bb.extent[axis];
      double split = 0.0;
      
      if (direction == -1) { //nothing on the right
        split = size/(pow(2,level))+bb.min[axis];
        float g = bb.min[axis];

      } else { //nothing on the left
        float g = bb.max[axis];
        split =  -size/(pow(2,level))+bb.max[axis];
      }
      int lefties = 0;
      int righties = 0;
      for (Primitive *p : prims) {
        if (p->get_bbox().centroid()[axis] < split){
          pLeft.push_back(p);
          lefties += 1;
        } else {
          pRight.push_back(p);
          righties += 1;
      }
    }
      if (lefties == 0){
        if (level>3){
            std::vector<CGL::StaticScene::Primitive *> pL = std::vector<CGL::StaticScene::Primitive *>(prims.begin(),prims.begin() + prims.size()/2);
            std::vector<CGL::StaticScene::Primitive *> pR = std::vector<CGL::StaticScene::Primitive *>(prims.begin() + prims.size()/2,prims.end());
            std::vector<CGL::StaticScene::Primitive *> * Left = new std::vector<CGL::StaticScene::Primitive *>(pL);
            std::vector<CGL::StaticScene::Primitive *> * Right = new std::vector<CGL::StaticScene::Primitive *>(pR);
            node->l = construct_bvh(*Left, max_leaf_size);
            node->r = construct_bvh(*Right, max_leaf_size);
        return;
        }else{
          helperSplit(prims, node, axis, level+1, 1, max_leaf_size);
          return;
        }
      } else if (righties == 0){
        if (level>3){
            std::vector<CGL::StaticScene::Primitive *> pL = std::vector<CGL::StaticScene::Primitive *>(prims.begin(),prims.begin() + prims.size()/2);
            std::vector<CGL::StaticScene::Primitive *> pR = std::vector<CGL::StaticScene::Primitive *>(prims.begin() + prims.size()/2,prims.end());
            std::vector<CGL::StaticScene::Primitive *> * Left = new std::vector<CGL::StaticScene::Primitive *>(pL);
            std::vector<CGL::StaticScene::Primitive *> * Right = new std::vector<CGL::StaticScene::Primitive *>(pR);
            node->l = construct_bvh(*Left, max_leaf_size);
            node->r = construct_bvh(*Right, max_leaf_size);
        return;
        } else {
          helperSplit(prims, node, axis, level+1, -1, max_leaf_size);
          return;
        }
      }else{
        std::vector<CGL::StaticScene::Primitive *> * Left = new std::vector<CGL::StaticScene::Primitive *>(pLeft);
        std::vector<CGL::StaticScene::Primitive *> * Right = new std::vector<CGL::StaticScene::Primitive *>(pRight);
        node->l = construct_bvh(*Left, max_leaf_size);
        node->r = construct_bvh(*Right, max_leaf_size);
        return;
      }
}

BVHNode *BVHAccel::construct_bvh(const std::vector<Primitive*>& prims, size_t max_leaf_size) {
  
  // TODO Part 2, task 1:
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.


  BBox centroid_box, bbox;

  for (Primitive *p : prims) {
    BBox bb = p->get_bbox();
    bbox.expand(bb);
    Vector3D c = bb.centroid();
    centroid_box.expand(c);
  }

  // You'll want to adjust this code.
  // Right now we just return a single node containing all primitives.
  BVHNode *node = new BVHNode(bbox); 
  if (prims.size() > max_leaf_size){ //need to split
      double x_size = bbox.extent[0]; 
      double y_size = bbox.extent[1]; 
      double z_size = bbox.extent[2]; 
      int axis = 0;
      //double split = 0;
      if (x_size >= y_size && x_size >= z_size){ //split along x
        axis = 0;
      }else if (y_size >= x_size && y_size >= z_size){ //split along y
        axis = 1;
      }else{ //split along z
        axis = 2;
      }
      helperSplit(prims, node, axis, 1, 1, max_leaf_size);
  } else {
    node->prims = new vector<Primitive *>(prims);
  }
  return node;
}


bool BVHAccel::intersect(const Ray& ray, BVHNode *node) const {

  // TODO Part 2, task 3:
  // Implement BVH intersection.
  // Currently, we just naively loop over every primitive.
  double t0; double t1;
  if (get_bbox().intersect(ray, t0, t1) == false){ //node.getbbox?
    return false;
  }
  if (node->isLeaf()){
    for (Primitive *p : *(node->prims)) {
      if (p->intersect(ray)) 
        total_isects++;
        return true;
    }
    return false;
  }else{
    return intersect(ray, node->l) || intersect(ray, node->r);
  }
  // for (Primitive *p : *(root->prims)) {
  //   total_isects++;
  //   if (p->intersect(ray)) 
  //     return true;
  // }
  // return false;

}

bool BVHAccel::intersect(const Ray& ray, Intersection* i, BVHNode *node) const {

  // TODO Part 2, task 3:
  // Implement BVH intersection.
  // Currently, we just naively loop over every primitive.

  // bool hit = false;
  // for (Primitive *p : *(root->prims)) {
  //   total_isects++;
  //   if (p->intersect(ray, i)) 
  //     hit = true;
  // }
  // return hit;

  double t0; double t1;
  bool hit = false;
  if (node->bb.intersect(ray, t0, t1) == false){ //node.getbbox?
    return false;
  }
  // int smallT;
  // Intersection closest;
  // Intersection current;
  if (node->isLeaf()){
    for (Primitive *p : *(node->prims)) {
      if (p->intersect(ray, i)) { //&current
         total_isects++;
        // if (current.t < smallT){
        //   closest = current;
        //   smallT = current.t;
          hit = true;
        //}
      }
    } 
   // i = &closest;
    return hit;
  }else{
    //Intersection left; Intersection right;
    bool leftbool = intersect(ray, i, node->l);
    bool rightbool = intersect(ray, i, node->r);
    // if (leftbool && rightbool){
    //   if (left.t < right.t){
    //     i = &left;
    //     return leftbool;
    //   }else{
    //     i = &right;
    //     return rightbool;
    //   }
    // } else if (leftbool){
    //     i = &left;
    //     return leftbool;
    // } else if (rightbool){
    //     i = &right;
    //     return rightbool;
    // } else {
    //     return false;
    // }

   return leftbool || rightbool;
 }

}

}  // namespace StaticScene
}  // namespace CGL
