#ifndef CGL_DRAW_UTIL_H
#define CGL_DRAW_UTIL_H

#include <cstdlib>
#include "point.h"

#include "CGL/CGL.h"

#include "GL/glew.h"


namespace CGL {


inline void draw_points(std::vector<Point*>& points_to_draw, Color c) {
    //clear color and depth buffer 
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glColor4f(c.r, c.g, c.b, c.a); 
    
    glPointSize(2.0f);//set point size to 5 pixels
    
    glBegin(GL_POINTS); //starts drawing of points
    for (std::vector<Point*>::iterator it = points_to_draw.begin(); it != points_to_draw.end(); ++it) {
        glVertex3d((*it)->pos.x,(*it)->pos.y,(*it)->pos.z);
    }
    glEnd();//end drawing of points
    glEnable(GL_LIGHTING); 
    glEnable(GL_DEPTH_TEST);
}

} // namespace CGL

#endif  // CGL_DRAW_UTIL_H
