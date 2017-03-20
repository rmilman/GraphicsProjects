//
// Created by Trevor Nesbitt on 4/21/16.
//
#include "random_util.h"
#include "bssrdf.h"
#include <cmath>
#include "sampler.h"

namespace  CGL {


    Spectrum BSSRDF::get_L_out(const Vector3D& po, const Vector3D& wo, Octree* o) {
        // convert hemisphere sampling to global sampling:
        if (!multiple_channels) return Spectrum(1.f, 1.f, 1.f)*diffuse_approximation(po, wo, o, rd);
        return diffuse_approximation_3ch(po, wo, o);
    }

    float BSSRDF::diffuse_approximation(Vector3D po, Vector3D wo, Octree* o,
                                         DiffusionReflection diffuse_ref) {
        const float MAX_ERROR = 0; // Smaller (at least 0) results in going deeper into octree.
        Spectrum mo = o->mo(o->bounds, po, diffuse_ref, MAX_ERROR);
        float ft = 1 - schlick_approximation_of_fr(wo);
        return (ft * rd.fdt * mo.r) / (float) PI; // we can use mo.r bc all mo channels are the same
    }

    Spectrum BSSRDF::diffuse_approximation_3ch(Vector3D po, Vector3D wo, Octree *o) {
        float r = diffuse_approximation(po, wo, o, rd_red);
        float g = diffuse_approximation(po, wo, o, rd_green);
        float b = diffuse_approximation(po, wo, o, rd_blue);
        return Spectrum(r ,g, b);
    }

    float BSSRDF::schlick_approximation_of_fr(Vector3D wo) {
        float tmp1 = (float) (1 - wo.z);
        float tmp2 = (rd.ior - 1) / (rd.ior + 1);
        float R0 = tmp2*tmp2;
        return R0 + (1 - R0) * tmp1 * tmp1 * tmp1 * tmp1 * tmp1;
    }

    // write phase function
    float BSSRDF::phase_function(const Vector3D& wi, const Vector3D& wo) {
        float numerator = 1 - rd.g*rd.g;
        float denom = (float) pow(1 + (rd.g*rd.g) - (2*rd.g)*cos_world(wi, wo), 1.5);
        return numerator / (denom * 4 * (float) PI);
    }

    float BSSRDF::calc_so_prime() {
        float e = (float) random_uniform();
        float average_t = (rd_red.sigma_t_prime + rd_blue.sigma_t_prime + rd_green.sigma_t_prime) / 3;
        return -1*((float) log(e) / average_t);
    }

}