#include "bsdf.h"

#include <iostream>
#include <algorithm>
#include <utility>

using std::min;
using std::max;
using std::swap;

namespace CGL {

void make_coord_space(Matrix3x3& o2w, const Vector3D& n) {

    Vector3D z = Vector3D(n.x, n.y, n.z);
    Vector3D h = z;
    if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z)) h.x = 1.0;
    else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z)) h.y = 1.0;
    else h.z = 1.0;

    z.normalize();
    Vector3D y = cross(h, z);
    y.normalize();
    Vector3D x = cross(z, y);
    x.normalize();

    o2w[0] = x;
    o2w[1] = y;
    o2w[2] = z;
}

// Diffuse BSDF //

Spectrum DiffuseBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return albedo * (1.0 / PI);
}

Spectrum DiffuseBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *wi = sampler.get_sample(pdf);
  return albedo * (1.0 / PI);
}

// Mirror BSDF //

Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  // TODO Part 5:
  reflect(wo,wi);
  *pdf = (float) 1.0;
  Spectrum s = reflectance/(std::abs(wi->z));

  return s;
}

// Glossy BSDF //

/*
Spectrum GlossyBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlossyBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *pdf = 1.0f;
  return reflect(wo, wi, reflectance);
}
*/

// Refraction BSDF //

Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  // TODO Part 5:
 refract(wo, wi, ior);
 *pdf = (float) 1;
 Spectrum s = transmittance/(std::abs(wi->z)); //.8 *lol
 return transmittance/(std::abs(wi->z));
}

// Glass BSDF //

Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  // TODO Part 5:
  // Compute Fresnel coefficient and either reflect or refract based on it.
  //wo = incoming direction
  Spectrum s = Spectrum(0,0,0); //refract(wo,wi,ior);  *pdf = (float) 1.0; return s;
  if (!refract(wo,wi,ior)){
    reflect(wo,wi);
    *pdf = (float) 1.0;
    s = reflectance/(std::abs(wi->z));
    
 }else{
    refract(wo,wi,ior);
    float cos_t = std::abs(wo.z); //help
    float no; float ni;
    if (wo.z < 0) { //coming from outside material
      no = ior;
      ni = 1.0f;
    } else { //coming from inside material
      no = 1.0f;
      ni = ior;
    }
    float Ro = pow((no-ni)/(no+ni),2);
    float R = clamp(Ro + (1-Ro)*(pow(1-cos_t,5)),0.0,1.0);
    // std::cout<<R<<std::endl;
   
    if(coin_flip(R)){
      reflect(wo,wi);
      *pdf = R;
      s = R*reflectance/(std::abs(wi->z));
      
    }else{
      refract(wo,wi,ior); 
      *pdf = 1- R;
      s = (1-R)*transmittance*(ni*ni)/(no*no)/(std::abs(wi->z)); //bug? noni 
      //std::cout<<s.r<<std::endl;
    }
  }
  return s;
}

void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {
  Vector3D v = 2*(Vector3D(0,0,std::abs(wo.z)))-wo;
  *wi = v; //debugin journey, was assigning random memory
  // TODO Part 5:
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
  

}

bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {
 
  // TODO Part 5:
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.
 // *wi = -wo;
 // return true;
  float no; 
  float ni;
  float sign = 1.0f;
  if (wo.z < 0) { //coming from outside material
      no = ior;
      ni = 1.0f;
      sign = -1.0f;
      //n = Vector3D(0,0,1);
  } else { //coming from inside material
      no = 1.0f;
      ni = ior;
      
      //n = Vector3D(0,0,-1);
  }
  Vector3D n = Vector3D(0,0,sign);
  //double to = acos(std::abs(wo.z));
 
  //double ti = asin(ior*sin(to));
  //Vector3D v = Vector3D(wo.x * (ni/no), wo.y * (ni/no), -wo.z);
  float r = no/ni;
  float c = dot(-n,-wo); //- vs positive
  Vector3D v = -r*wo + (r*c - sqrt(1-r*r*(1-c*c)))*n;
  *wi = v;//-wo, parentheses
  if (no*sin_theta(wo) >= ni){
    //std::cout<<"t"<<std::endl;
    return false;
  }
  return true;

}

// Emission BSDF //

Spectrum EmissionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum EmissionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *pdf = 1.0 / PI;
  *wi  = sampler.get_sample(pdf);
  return Spectrum();
}

} // namespace CGL
