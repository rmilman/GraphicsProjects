/*
 * Student solution for UC Berkeley Project 2
 *
 * Implemented by ____ on ____.
 *
 */

#include "student_code.h"
#include "mutablePriorityQueue.h"

namespace CGL {

 

    void BezierPatch::preprocess() {
        // TODO Part 1.
        // TODO If you use the matrix form for Bezier patch evaluation, you will need to
        // TODO compute your matrices based on the 16 control points here. 
        // TODO You will also need to define your matrices
        // TODO as member variables in the "BezierPatch" class.
        // TODO If you use De Casteljau's recursive algorithm, you will not need to do anything here.

    }

    float BezierPatch::bPolynomial(const int index, const float u) const{
        if (index == 0) {
            return pow(1-u,3);
        } else if (index == 1) {
            return pow(1-u,2)*u*3;
        } else if (index == 2) {
            return (1-u)*u*u*3;
        }else if (index == 3) {
            return pow(u,3);
        } else {
            return 0;
        }
    }

    Vector3D BezierPatch::evaluate(double u, double v) const {
        // TODO Part 1.
        // TODO Returns the 3D point whose parametric coordinates are (u, v) on the Bezier patch.
        // TODO Note that both u and v are within [0, 1].
        Vector3D rtn = Vector3D();
        for (int i = 0; i<4; i++){
            for (int j = 0; j<4; j++){
                rtn += controlPoints[i][j]*bPolynomial(i,u)*bPolynomial(j,v);
            }
        }
        return rtn;
    }

    void BezierPatch::add2mesh(Polymesh* mesh) const {
        // TODO Part 1.
        // TODO Tessellate the given Bezier patch into triangles uniformly on a 8x8 grid(8x8x2=128 triangles) in parameter space.
        // TODO You will call your own evaluate function here to compute vertex positions of the tessellated triangles.
        // TODO The "addTriangle" function inherited from the "BezierPatchLoader" class may help you add triangles to the output mesh. 
        float a = .125;
        for (int i = 0; i<8; i++){
            for (int j = 0; j<8; j++){
                float u =  i * a;
                float v = j * a;
                Vector3D v0 = evaluate(u,v);
                Vector3D v1 = evaluate(u+a,v);
                Vector3D v2 = evaluate(u,v+a);
                Vector3D v3 = evaluate(u+a, v+a);
                //cout<<"print2 "<< endl;
                addTriangle(mesh,v0,v1,v3);
                addTriangle(mesh,v3,v2,v0);
            }
        }
    }

    Vector3D Vertex::normal(void) const
    // TODO Part 2.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
    {
        // TODO Compute and return the area-weighted unit normal.
        Vector3D normal = Vector3D();
        Vector3D p0 = Vector3D();
        Vector3D p1 = Vector3D();
        Vector3D p2 = Vector3D();
        Vector3D v01 = Vector3D();
        Vector3D v02= Vector3D();
        HalfedgeCIter h_orig = halfedge(); 
        HalfedgeCIter h = h_orig;
        int i = 1;
        p0 = h->vertex()->position; 
        h = h->next();
        p1 = h->vertex()->position; 
        h = h->next();
        p2 = h->vertex()->position; //original vertex
        v01 = p1-p0;
        v02 = p2-p0;
        normal += cross(v01, v02);
        // if (isBoundary()){
        //     return normal.unit();
        // } 
        h = h->twin();
        while (h != h_orig){
            p0 = h->vertex()->position; 
            h = h->next();
            p1 = h->vertex()->position; 
            h = h->next();
            p2 = h->vertex()->position; 
            v01 = p1-p0;
            v02 = p2-p0;
            normal += cross(v01, v02);
            h = h->twin();
            i++;         
        }
        normal = normal/i;
        return normal.unit();
    }

    EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0)
     {
        HalfedgeIter h0 = e0->halfedge();
        if (h0->face()->isBoundary() || h0->twin()->face()->isBoundary()){
            return e0;
        }
        //storing everything
        //halfedges
        HalfedgeIter h1 = h0->next();
        HalfedgeIter h2 = h1->next();
        HalfedgeIter h3 = h0->twin();
        HalfedgeIter h4 = h3->next();
        HalfedgeIter h5 = h4->next();
        HalfedgeIter h6 = h1->twin();
        HalfedgeIter h7 = h2->twin();
        HalfedgeIter h8 = h4->twin();
        HalfedgeIter h9 = h5->twin();
        //vertices
        VertexIter v0 = h0->vertex();
        VertexIter v1 = h1->vertex();
        VertexIter v2 = h2->vertex();
        VertexIter v3 = h5->vertex();
        //edges
        EdgeIter e1 = h1->edge();
        EdgeIter e2 = h2->edge();
        EdgeIter e3 = h4->edge();
        EdgeIter e4 = h5->edge();
        
        //reassigning vertices
        v3->halfedge() = h0;
        v2->halfedge() = h3;
        v1->halfedge() = h9;
        v0->halfedge() = h7;

        h0->vertex() = v3;
        h1->vertex() = v2;
        h2->vertex() = v0;
        h3->vertex() = v2;
        h4->vertex() = v3;
        h5->vertex() = v1;

        //reassigning twins
        h1->twin() = h7;
        h2->twin() = h8;
        h4->twin() = h9;
        h5->twin() = h6;
        h6->twin() = h5;
        h7->twin() = h1;
        h8->twin() = h2;
        h9->twin() = h4;
        
        //reassigning edges
        h1->edge() = e2;
        h2->edge() = e3;
        h4->edge() = e4;
        h5->edge() = e1;

        e1->halfedge() = h5;
        e2->halfedge() = h1;
        e3->halfedge() = h2;
        e4->halfedge() = h4;


        //stuff that didn't change
        //reassinging faces (all same)
        //faces
            // FaceIter f0 = h0->face();
            // FaceIter f1 = h3->face();
        //reassignin nexts (all same)
        // h0->edge() = e0; //same
        // h3->edge() = e0; //same
        // e0->halfedge() = h0;
          // h3->twin() = h0; //technically same
        // h0->twin() = h3; //technically same

        return e0;
    }

    VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
        // TODO Part 4.
        // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
        // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
        HalfedgeIter h0 = e0->halfedge();
        if (h0->face()->isBoundary()){
            //storing everything
            HalfedgeIter h3 = h0->twin();
            HalfedgeIter h4 = h3->next();
            HalfedgeIter h5 = h4->next();
            HalfedgeIter h8 = h4->twin();
            HalfedgeIter h9 = h5->twin();
            VertexIter v0 = h0->vertex();
            VertexIter v1 = h3->vertex();
            VertexIter v3 = h5->vertex();
            EdgeIter e3 = h4->edge();
            EdgeIter e4 = h5->edge();
            FaceIter b = h0->face();
            FaceIter f1 = h3->face();

            //making new things
            VertexIter v4 = newVertex();
            FaceIter f3 = newFace();
            EdgeIter e5 = newEdge();
            EdgeIter e7 = newEdge();
            HalfedgeIter h10 = newHalfedge();
            HalfedgeIter h11 = newHalfedge();
            HalfedgeIter h12 = newHalfedge();
            HalfedgeIter h13 = newHalfedge();

            //reassigning things

            h0->setNeighbors(h0->next(),h3,v4,e0,b); //h0's next?
            h3->setNeighbors(h4,h0,v1,e0,f1);
            h4->setNeighbors(h5,h10,v4,e5,f1);
            h5->setNeighbors(h3,h9,v3,e4,f1);
           
            h10->setNeighbors(h11,h4,v3,e5,f3);
            h11->setNeighbors(h12,h13,v4,e7,f3);
            h12->setNeighbors(h10,h8,v0,e3,f3);
            h13->setNeighbors(h0,h11,v0,e7,b);

            f1->halfedge() = h3; //same
            f3->halfedge() = h11;

            v0->halfedge() = h13;
            v1->halfedge() = h3; //same
            v3->halfedge() = h10; //same
            v4->halfedge() = h11;

            e0->halfedge() = h0; //same
            e3->halfedge() = h8;
            e4->halfedge() = h9; //same
            e5->halfedge() = h4;
            e7->halfedge() = h11;

            //outer half edges
            h8->vertex() = v3;
            h9->vertex() = v1;
            h8->twin() = h12;
            h9->twin() = h5; 
            h8->edge() = e3;
            h9->edge() = e4;

              
            //update position
            v4->position = v0->position + v1->position; //+ v2->position + v3->position
            Vector3D v = v4->position;
            v4->position = v*.5; //error for write up!
            return v4;
        } else if (h0->twin()->face()->isBoundary()){
            //save
            HalfedgeIter h1 = h0->next();
            HalfedgeIter h2 = h1->next();
            HalfedgeIter h3 = h0->twin();
            HalfedgeIter h4 = h3->next();
            HalfedgeIter h6 = h1->twin();
            HalfedgeIter h7 = h2->twin();
            VertexIter v0 = h0->vertex();
            VertexIter v1 = h1->vertex();
            VertexIter v2 = h2->vertex();
            EdgeIter e1 = h1->edge();
            EdgeIter e2 = h2->edge();
            FaceIter f0 = h0->face();
            FaceIter b = h0->twin()->face();
            //make new
            VertexIter v4 = newVertex();
            FaceIter f2 = newFace();
            EdgeIter e6 = newEdge();
            EdgeIter e7 = newEdge();
            HalfedgeIter h11 = newHalfedge();
            HalfedgeIter h13 = newHalfedge();
            HalfedgeIter h14 = newHalfedge();
            HalfedgeIter h15 = newHalfedge();

            //reset pointers
            h0->setNeighbors(h1,h3,v4,e0,f0); 
            h1->setNeighbors(h2,h6,v1,e1,f0);
            h2->setNeighbors(h0,h14,v2,e6,f0);
            h3->setNeighbors(h11,h0,v1,e0,b);

            h11->setNeighbors(h4,h13,v4,e7,b); 
            h13->setNeighbors(h14,h11,v0,e7,f2);
            h14->setNeighbors(h15,h2,v4,e6,f2);
            h15->setNeighbors(h13,h7,v2,e2,f2);

            f0->halfedge() = h0; //same
            f2->halfedge() = h13;
        
            v0->halfedge() = h13;
            v1->halfedge() = h3; //same
            v2->halfedge() = h2;  //same
            v4->halfedge() = h11;

            e0->halfedge() = h0; //same
            e1->halfedge() = h6;  //same
            e2->halfedge() = h7;
            e6->halfedge() = h2;
            e7->halfedge() = h11;

            //outer half edges
            h6->vertex() = v2;
            h7->vertex() = v0;
            h6->twin() = h1; 
            h7->twin() = h15;
            h6->edge() = e1;
            h7->edge() = e2;

            //update position
            v4->position = v0->position + v1->position; //+ v2->position + v3->position
            Vector3D v = v4->position;
            v4->position = v*.5; //error for write up!
            return v4;
         }
        //storing everything
        //halfedges
        HalfedgeIter h1 = h0->next();
        HalfedgeIter h2 = h1->next();
        HalfedgeIter h3 = h0->twin();
        HalfedgeIter h4 = h3->next();
        HalfedgeIter h5 = h4->next();
        HalfedgeIter h6 = h1->twin();
        HalfedgeIter h7 = h2->twin();
        HalfedgeIter h8 = h4->twin();
        HalfedgeIter h9 = h5->twin();
        //vertices
        VertexIter v0 = h0->vertex();
        VertexIter v1 = h3->vertex();
        VertexIter v2 = h2->vertex();
        VertexIter v3 = h5->vertex();
        //edges
        EdgeIter e1 = h1->edge();
        EdgeIter e2 = h2->edge();
        EdgeIter e3 = h4->edge();
        EdgeIter e4 = h5->edge();
        //faces
        FaceIter f0 = h0->face();
        FaceIter f1 = h3->face();

        //making new things
        VertexIter v4 = newVertex();

        FaceIter f2 = newFace();
        FaceIter f3 = newFace();

        EdgeIter e5 = newEdge();
        EdgeIter e6 = newEdge();
        EdgeIter e7 = newEdge();

        HalfedgeIter h10 = newHalfedge();
        HalfedgeIter h11 = newHalfedge();
        HalfedgeIter h12 = newHalfedge();
        HalfedgeIter h13 = newHalfedge();
        HalfedgeIter h14 = newHalfedge();
        HalfedgeIter h15 = newHalfedge();

        //reassigning things
        h0->setNeighbors(h1,h3,v4,e0,f0);
        h1->setNeighbors(h2,h6,v1,e1,f0);
        h2->setNeighbors(h0,h14,v2,e6,f0);
        h3->setNeighbors(h4,h0,v1,e0,f1);
        h4->setNeighbors(h5,h10,v4,e5,f1);
        h5->setNeighbors(h3,h9,v3,e4,f1);
       
        h10->setNeighbors(h11,h4,v3,e5,f3);
        h11->setNeighbors(h12,h13,v4,e7,f3);
        h12->setNeighbors(h10,h8,v0,e3,f3);
        h13->setNeighbors(h14,h11,v0,e7,f2);
        h14->setNeighbors(h15,h2,v4,e6,f2);
        h15->setNeighbors(h13,h7,v2,e2,f2);

        f0->halfedge() = h0; //same
        f1->halfedge() = h3; //same
        f2->halfedge() = h13;
        f3->halfedge() = h11;

        v0->halfedge() = h13;
        v1->halfedge() = h3; //same
        v2->halfedge() = h2;  //same
        v3->halfedge() = h10; //same
        v4->halfedge() = h11;

        e0->halfedge() = h0; //same
        e1->halfedge() = h6;  //same
        e2->halfedge() = h7;
        e3->halfedge() = h8;
        e4->halfedge() = h9; //same
        e5->halfedge() = h4;
        e6->halfedge() = h2;
        e7->halfedge() = h11;

        //outer half edges
        //same
        h6->vertex() = v2;
        h7->vertex() = v0;
        h8->vertex() = v3;
        h9->vertex() = v1;

        h6->twin() = h1; 
        h7->twin() = h15;
        h8->twin() = h12;
        h9->twin() = h5; 
        //same
        h6->edge() = e1;
        h7->edge() = e2;
        h8->edge() = e3;
        h9->edge() = e4;

        //getting the new vertex position
        v4->position = v0->position + v1->position; //+ v2->position + v3->position
        Vector3D v = v4->position;
        v4->position = v*.5; //error for write up!
        return v4;
    }

    void MeshResampler::upsample(HalfedgeMesh& mesh)
    // TODO Part 5.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    {
        // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
        // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
        // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
        // the new subdivided (fine) mesh, which has more elements to traverse.  We will then assign vertex positions in
        // the new mesh based on the values we computed for the original mesh.
       
        // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
        // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
        // TODO a vertex of the original mesh.

         //old vertex, new position
        
        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
            v->isNew = false; 
            if (v->isBoundary()){
                v->newPosition = v->position;
            } else {
            float u = .1875;
            float g = .375;
            float d = v->degree();
            if (d != 3){
                u = g/d;
            }
            Vector3D sumPositions = Vector3D();
            HalfedgeIter h_orig = v->halfedge();
            HalfedgeIter h = h_orig->twin();
            sumPositions += h->vertex()->position;
            h = h->next();
            while (h != h_orig){
                h = h->twin();
                sumPositions+= h->vertex()->position;
                h = h->next();
            }
            v->newPosition = v->position * (1-d*u) + u * sumPositions;
            }
        }
        
        // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
        //new vertex, new position
        float t = .375;
        float a = .125;
        
        //double c = .42857142857;
        double c = .5;
        double b = 0;
        //double b = 1-2*c;
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++){
            e->isNew = false;
            if (e->isBoundary()){
                Vector3D sumPositions = Vector3D();
                sumPositions += (c*(e->halfedge()->vertex()->position));
                sumPositions += (c*(e->halfedge()->twin()->vertex()->position));
                if (e->halfedge()->face()->isBoundary()){
                    //sumPositions += (b*(e->halfedge()->twin()->next()->next()->vertex()->position));
                } else if (e->halfedge()->twin()->face()->isBoundary()){
                    //sumPositions += (b*(e->halfedge()->next()->next()->vertex()->position));
                }
                e->newPosition = sumPositions;
            } else {
                Vector3D sumPositions = Vector3D();
                sumPositions += t*(e->halfedge()->vertex()->position);
                sumPositions += t*(e->halfedge()->twin()->vertex()->position);
                sumPositions += a*(e->halfedge()->next()->next()->vertex()->position);
                sumPositions += a*(e->halfedge()->twin()->next()->next()->vertex()->position);
                e->newPosition = sumPositions;
            }
        }

        // TODO Next, we're going to split every edge in the mesh, in any order.  For future
        // TODO reference, we're also going to store some information about which subdivided
        // TODO edges come from splitting an edge in the original mesh, and which edges are new,
        // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
        // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
        // TODO just split (and the loop will never end!)
        EdgeIter e = mesh.edgesBegin();

        //the big fix!!!!
        while (e != mesh.edgesEnd()) {
            EdgeIter nextEdge = e;
            nextEdge++;
              if (e->isNew==false && e->isBoundary()==false && e->halfedge()->vertex()->isNew==false 
                &&  e->halfedge()->twin()->vertex()->isNew==false) { //here too
                VertexIter nv = mesh.splitEdge(e);
                nv->halfedge()->edge()->isNew = false;
                nv->halfedge()->next()->next()->edge()->isNew = true;
                nv->halfedge()->twin()->next()->edge()->isNew = true;
                //nv->halfedge()->twin()->next()->twin()->next()->edge()->isNew = false;
                nv->isNew = true;
                nv->newPosition = e->newPosition; 

            } else if (e->isNew==false && e->halfedge()->vertex()->isNew==false 
                 &&  e->halfedge()->twin()->vertex()->isNew==false) {
                VertexIter nv = mesh.splitEdge(e);
                  nv->halfedge()->edge()->isNew = false;
                if (e->halfedge()->face()->isBoundary()){
                     nv->halfedge()->next()->next()->edge()->isNew = true;
                } else if (e->halfedge()->twin()->face()->isBoundary()){
                     nv->halfedge()->twin()->next()->next()->edge()->isNew = true;
                }
                nv->isNew = true;
                nv->newPosition = e->newPosition; 
            }
            e = nextEdge;
        }



        // TODO Now flip any new edge that connects an old and new vertex.
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++){
            if (e->isNew && e->isBoundary()==false){
                if (e->halfedge()->vertex()->isNew && e->halfedge()->twin()->vertex()->isNew==false)  {
                    mesh.flipEdge(e);
                } else if (e->halfedge()->vertex()->isNew==false && e->halfedge()->twin()->vertex()->isNew){
                    mesh.flipEdge(e);
                }else{
                    //can't flip boundary ones
                }
            }
        }

        // TODO Finally, copy the new vertex positions into final Vertex::position.
        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
                v->position = v->newPosition;  
                //v->isNew = false;
        }

    }

}
