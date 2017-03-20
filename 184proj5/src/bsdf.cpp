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
  // Implement MirrorBSDF
  *pdf = 1;
  reflect(wo, wi);
  Spectrum ret = reflectance / abs_cos_theta(*wi);
  return ret;
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
  // Implement RefractionBSDF
    float no = 1, ni = ior;
    if (wo.z > 0) {
        no = ior; ni = 1;
    }
    *pdf = 1;
    refract(wo, wi, ior);
    return transmittance * (pow(no/ni, 2.) / abs_cos_theta(*wi));
}

// Glass BSDF //

Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  // TODO Part 5:
  // Compute Fresnel coefficient and either reflect or refract based on it.

  float no = ior, ni = 1;
  if (wo.z > 0) {
      no = 1; ni = ior;
  }

  if (!refract(wo, wi, ior)) {
      *pdf = 1;
      reflect(wo, wi);
      return reflectance / abs_cos_theta(*wi);
  }
  double R0 = pow((ni-no)/(no+ni), 2.);
  double R = R0 + (1-R0) * pow(1 - abs_cos_theta(*wi), 5.); // schlick
  R = R > 1 ? 1 : R;
  R = R < 0 ? 0 : R;
  if (coin_flip(R)) {
    *pdf = (float) R;
    reflect(wo, wi);
    return reflectance * (R / abs_cos_theta(*wi));
  }
  *pdf = (float) (1 - R);
  refract(wo, wi, ior);
  return transmittance * ((1 - R) * pow(no/ni, 2.) / abs_cos_theta(*wi));
}

void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {

  // TODO Part 5:
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
  *wi = (2. * wo.z) * Vector3D(0., 0., 1.) - wo;
  wi->normalize();
}

bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {

  // TODO Part 5:
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.

  float no = ior, ni = 1;
    if (wo.z > 0) {
        no = 1; ni = ior;
  }
  double sin_theta_wi = (no/ni)*sin_theta(wo);
  double cos_theta_wi = sqrt(1 - sin_theta_wi*sin_theta_wi);
  if (wo.z > 0) {
      cos_theta_wi *= -1;
  }
  double cos_phi_wi = -cos_phi(wo);
  double sin_phi_wi = -sin_phi(wo);
  *wi = Vector3D(sin_theta_wi*cos_phi_wi, sin_theta_wi*sin_phi_wi, cos_theta_wi);
  wi->normalize();
  return sin_theta(*wi) < no/ni;
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
