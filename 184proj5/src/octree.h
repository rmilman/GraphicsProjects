/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   octree.h
 * Author: elizabeth & Trevor
 *
 * Created on April 21, 2016, 9:24 PM
 */

#ifndef OCTREE_H
#define OCTREE_H

#include "diffusionReflection.h"
#include "bbox.h"
#include "point.h"
#include "draw_util.h"


namespace  CGL {

    class Octree {

    public:

        Octree(BBox b) {
            bounds = b;
            isLeaf = true;
            sumArea = 0.f;
            depth = 0;
            for (int i = 0; i < 8; ++i) {
                ips[i] = NULL;
                children[i] = NULL;
            }
        }

        ~Octree() {
            for (int i = 0; i < 8; ++i) {
                if (children[i] != NULL) {
                    delete children[i];
                }
            }
        }

        /*
         * Returns the radiant exitance at point pt 
         */
         Spectrum mo(BBox &nodeBound, Vector3D &pt, DiffusionReflection Rd, float maxError);

        /*
         * Recursively insert a new point into the octree.
         */
        void insert(BBox &octreeBounds, Point *ip);

        /*
         * Recursively fill in pos, irradiance, and sumArea for entire Octree, working our way bottom up.
         */
        void init_heirarchy();

        Vector3D p;
        bool isLeaf;
        // bool isRoot;
        Spectrum irradiance;
        float sumArea;
        Octree *children[8];
        Point *ips[8];
        BBox bounds;
        int depth;

        /**
   * Draw the BVH with OpenGL - used in visualizer
   */
  void draw(Color& c){ }
  void draw(Octree *node, Color& c, Color colors[]) ;

  /**
   * Draw the BVH outline with OpenGL - used in visualizer
   */
   void drawOutline(Color& c) { }
   void drawOutline(Octree *node, Color c);


  /**
   * Get entry point (root) - used in visualizer
   */
  // Octree* get_root() const { return root; }


    private:
        // helper functions for insert
        BBox octreeChildBound(int child, const BBox &nodeBound, const Vector3D &pMid);
        void add_to_interior_node(Point *ip, Vector3D pMid, BBox &octreeBounds);
        void convert_leaf_to_interior_node(Vector3D pMid, BBox &octreeBounds);
        bool add_to_leaf(Point *ip, Vector3D pMid, BBox &octreeBounds);

        // helper functions for init_heirarchy
        void init_leaf();
        void init_interior_node();
        // Octree* root; ///< root node of the BVH


    };

}

#endif /* OCTREE_H */

