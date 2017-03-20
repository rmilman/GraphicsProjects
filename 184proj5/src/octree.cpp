//
// Created by Trevor Nesbitt on 4/26/16.
//

#include "octree.h"
#include <iostream>

namespace CGL {

    void Octree::init_heirarchy() {
        if (isLeaf) init_leaf();

        else {

            for (int i = 0; i < 8; ++i) {
                Octree* child = children[i];
                if (child != NULL) {
                    child->init_heirarchy();
                }
            }

            init_interior_node();
        }
    }

    void Octree::init_interior_node() {
        float sumWt = 0.f;
        uint32_t i;
        irradiance = Spectrum();
        for (i = 0; i < 8; ++i) {
            if (children[i] == NULL) break;
            /*
             * I don't think they use illum() in the paper,
             * but I think it should be fine.
             */
            float wt = children[i]->irradiance.illum();
            irradiance += children[i]->irradiance;
            p += wt * children[i]->p;
            sumWt += wt;
            sumArea += children[i]->sumArea;
        }
        if (sumWt > 0.f) p /= sumWt;
        if (i != 0) irradiance /= (float) i;
    }

    void Octree::init_leaf() {
        float sumWt = 0.f;
        uint32_t i;
        irradiance = Spectrum();
        p = Vector3D();
        for (i = 0; i < 8; ++i) {
            if (ips[i] == NULL) break;
            /*
             * I don't think they use illum() in the paper,
             * but I think it should be fine.
             */
            float wt = ips[i]->irradiance.illum();
            irradiance += ips[i]->irradiance;
            p += wt * ips[i]->pos;
            sumWt += wt;
            sumArea += ips[i]->area;
        }
        if (sumWt > 0.f) p /= sumWt;
        if (i != 0) irradiance /= (float) i;
    }

    void Octree::insert(BBox &octreeBounds, Point *ip) {
        Vector3D pMid = .5 * octreeBounds.min + .5 * octreeBounds.max;
        if (isLeaf) {
            /** if add_to_leaf returns false, that means we
             * converted the root to an interior node and now
             * we have to add the point to the octree as an interior node. */
            if (add_to_leaf(ip, pMid, octreeBounds)) {
                return;
            }
        }
        add_to_interior_node(ip, pMid, octreeBounds);
    }

    bool Octree::add_to_leaf(Point *ip, Vector3D pMid, BBox &octreeBounds) {
        for (int i = 0; i < 8; ++i) {
            if (ips[i] == NULL) {
                ips[i] = ip;
                return true;
            }
        }
        convert_leaf_to_interior_node(pMid, octreeBounds);
        return false;
    }

    void Octree::convert_leaf_to_interior_node(Vector3D pMid, BBox &octreeBounds) {
        isLeaf = false;
        Point *localIps[8];
        for (int i = 0; i < 8; ++i) {
            localIps[i] = ips[i];
            children[i] = NULL;
        }
        for (int i = 0; i < 8; ++i) {
            Point *ip = localIps[i];
            add_to_interior_node(ip, pMid, octreeBounds);
        }
    }

    void Octree::add_to_interior_node(Point *ip, Vector3D pMid, BBox &octreeBounds) {
        Vector3D pos = ip->pos;
        // add point to interior node
        int child = (pos.x > pMid.x ? 4 : 0) +
                    (pos.y > pMid.y ? 2 : 0) +
                    (pos.z > pMid.z ? 1 : 0);
        BBox childBound = octreeChildBound(child, octreeBounds, pMid);
        if (!children[child]) {
            children[child] = new Octree(childBound);
            children[child]->depth = depth+1;
        }
        children[child]->insert(childBound, ip);
    }

    BBox Octree::octreeChildBound(int child, const BBox &nodeBound, const Vector3D &pMid) {
        BBox childBound = BBox();

        childBound.min.x = (child & 4) ? pMid.x : nodeBound.min.x;
        childBound.max.x = (child & 4) ? nodeBound.max.x : pMid.x;

        childBound.min.y = (child & 2) ? pMid.y : nodeBound.min.y;
        childBound.max.y = (child & 2) ? nodeBound.max.y : pMid.y;

        childBound.min.z = (child & 1) ? pMid.z : nodeBound.min.z;
        childBound.max.z = (child & 1) ? nodeBound.max.z : pMid.z;

        return childBound;
    }

    Spectrum Octree::mo(BBox &nodeBound, Vector3D &pt, DiffusionReflection Rd, float maxError) {
        // Compute Mo at node if error is low enough
        Vector3D d = pt - p;
        float dist_squared = (float) (d.x*d.x + d.y*d.y + d.z*d.z);
        float dw = sumArea / dist_squared;
        nodeBound.inside(pt);

//        if (dw > maxError)
//            printf("depth: %d, error: %f\n", depth, dw);
        if (dw < maxError && !nodeBound.inside(pt)) {
            return Rd(p, pt) * irradiance * sumArea; // Question: po, pi given to Rd correctly?
        }
        // Otherwise, compute Mo from points in leaf or recursively visit children
        Spectrum Mo = Spectrum(0, 0, 0);
        if (isLeaf) {
            // accumulate Mo from leaf node
            int i;
            for (i = 0; i < 8; i++) {
                if (!ips[i]) break;
                Mo += Rd(ips[i]->pos, pt) * ips[i]->irradiance * ips[i]->area;
            }
        } else {
            // recursively visit children nodes to compute Mo
            Vector3D pMid = .5 * nodeBound.min + .5 * nodeBound.max;
            for (int child = 0; child < 8; child++) {
                if (!children[child]) continue;
                BBox childBound = octreeChildBound(child, nodeBound, pMid);
                Mo += children[child]->mo(childBound, pt, Rd, maxError);
            }
        }
        return Mo;
    }
    void Octree::draw(Octree *node, Color& c, Color colors[]) {
      if (node->isLeaf) {
            glDisable(GL_LIGHTING);
            glDisable(GL_DEPTH_TEST);
            glColor4f(c.r, c.b, c.g, c.a); 
            
            glPointSize(3.0f);//set point size to 5 pixels
            
            glBegin(GL_POINTS); //starts drawing of points
            for (int j = 0; j < 8; j++) {
                if (node->ips[j] != NULL || node->ips[j] != nullptr) {
                    glVertex3d(node->ips[j]->pos.x,node->ips[j]->pos.y,node->ips[j]->pos.z);
                }
            }
            glEnd();//end drawing of points
            glEnable(GL_LIGHTING); 
            glEnable(GL_DEPTH_TEST);
      } else {
        for (int i = 0; i<8; i++){
            if (node->children[i] != NULL || node->children[i] != nullptr) {
                draw(node->children[i], colors[i], colors);
            }
        }
        
      }
    }




void Octree::drawOutline(Octree *node, Color c) {
  if (node == NULL){

  }else if (node->isLeaf) {
      node->bounds.draw(c);
  } else {
    // for (int i = 0; i < 8; i++){
    //     drawOutline(node->children[i], c);
    // }
     node->bounds.draw(c);

 }
}

}