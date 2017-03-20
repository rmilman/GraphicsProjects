//
// Created by Trevor Nesbitt on 4/21/16.
//

#ifndef DIFFUSIONREFLECTION_H
#define DIFFUSIONREFLECTION_H
#include "CGL/CGL.h"
#include "CGL/vector3D.h"
#include "CGL/matrix3x3.h"

#include <algorithm>


namespace CGL {

    class DiffusionReflection {
        public:

            DiffusionReflection() : DiffusionReflection(1, 1, 1, 1) { }

            DiffusionReflection(float ior, float sigma_a, float sigma_s, float g = 0.0) :
                    ior(ior), sigma_a(sigma_a), sigma_s(sigma_s), g(g) {
                sigma_s_prime = (1 - g) * sigma_s;
                sigma_t_prime = sigma_a + sigma_s_prime;
                sigma_tr = (float) sqrt(3 * sigma_t_prime * sigma_a);
                if (ior >= 1.0) {
                    fdr = -1.4399f / (ior * ior) + 0.7099f / ior + 0.6681f +
                          0.0636f * ior;
                } else {
                    fdr = -0.4399f + .7099f / ior - .3319f / (ior * ior) +
                          .0636f / (ior * ior * ior);
                }
                fdt = 1 - fdr;
                a = (1 + fdr) / (1 - fdr);
                d = 1 / (3 * sigma_t_prime);
                z_plus = 1 / sigma_t_prime;     // this is below the surface, but should be positive for calculations.
                z_minus = z_plus + 4 * a * d;
            }

            double operator()(Vector3D pi, Vector3D po) {
                Vector3D plus_dipole = Vector3D(pi.x, pi.y, pi.z-z_plus), // check the z term
                         minus_dipole = Vector3D(pi.x, pi.y, pi.z+z_minus);

                double d_plus = (plus_dipole - po).norm(),
                       d_minus = (minus_dipole - po).norm();

                double quant1 = (z_plus * (d_plus * sigma_tr + 1) * exp(-sigma_tr * d_plus)) /
                                (d_plus * d_plus * d_plus);

                double quant2 = (z_minus * (d_minus * sigma_tr + 1) * exp(-sigma_tr * d_minus)) /
                                (d_minus * d_minus * d_minus);
                return (quant1 - quant2) / (4*PI);  // i'm fairly confident that this is right, though
                                                    //perhaps the 4PI is somehow taken care of by PDF
            }

            /* The following do not depend on the input points, but are needed for computation.
             * Note that all of the variables below (except z_plus and z_minus) are defined on
             * pages 902-906 of chapter 16.5 of Light Transport II: Volume Rendering.
             * z-plus and z-minus are defined on the right side of page three
             * here: https://graphics.stanford.edu/papers/bssrdf/bssrdf.pdf.
             *
             * NOTE: Adding/deleting variables will require changes to initialize_variables. */

            float ior;              // Index of refraction. Given.
            float sigma_a;          // Absorption coefficient. Given.
            float sigma_s;          // Scattering coefficient. Given.
            float g;                // Mean cosine of scattering direction. Given (or assumed to be 0?)

            float sigma_s_prime;    // Reduced scattering coefficient. Depends only on g and sigma_s.
            float sigma_t_prime;    // Depends on sigma_s_prime and sigma_a.
            float sigma_tr;         // Depends on sigma_t_prime and sigma_a
            float fdr;              // Diffuse Fresnel reflectance. Depends only on ior.
            float fdt;              // Depends only on fdr
            float a;                // Depends only on fdr
            float d;                // Depends only on sigma_t_prime

            float z_plus;           // Depends only on sigma_t_prime
            float z_minus;          // Depends only on z_plus, a, and d
            // The following will need to be calculated dependent on the input points: d_plus, d_minus
    };

}

#endif //BSSRDF_H
