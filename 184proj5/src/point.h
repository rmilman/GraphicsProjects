/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Point.h
 * Author: elizabeth
 *
 * Created on April 22, 2016, 2:05 PM
 */

#ifndef CGL_POINT_H
#define CGL_POINT_H


#include <vector>

#include "CGL/vector3D.h"
#include "CGL/spectrum.h"
#include "CGL/misc.h"


/**
 * A record of a point which includes the 3D coordinates of the point
 * and other information needed for sampling a surface
 */
namespace CGL {
class Point{
    public:
        Point() : t (INF_D), area(INF_F) { }

        double t;    ///< time of intersection

        double area; ///< area this sample point is "covering"

        Vector3D pos;  ///< 3D coordinates of this point

        Vector3D n;  ///< normal at point of intersection

        Spectrum irradiance; // referred to as "E" in the readings

};
}

#endif /* POINT_H */

