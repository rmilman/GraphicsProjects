//
// Created by Trevor Nesbitt on 4/21/16.
//

#ifndef BSSRDF_H
#define BSSRDF_H
#include "CGL/CGL.h"
#include "CGL/spectrum.h"
#include "CGL/vector3D.h"
#include "CGL/matrix3x3.h"


#include <algorithm>
#include "sampler.h"
#include "octree.h"
#include "diffusionReflection.h"


namespace CGL {

    class BSSRDF {

    public:

        BSSRDF(float ior, float sigma_a, float sigma_s, float g) {
            rd = DiffusionReflection(ior, sigma_a, sigma_s, g);
            multiple_channels = false;
        }

        /* In this constructor, we take in spectrums and average their values.
         * This should be changed if we want non-greyscale BSSRDF outputs. */
        BSSRDF(float ior, Spectrum sigma_a, Spectrum sigma_s, float g) {
            multiple_channels = true;
            rd_red = DiffusionReflection(ior, sigma_a.r, sigma_s.r, g);
            rd_blue = DiffusionReflection(ior, sigma_a.b, sigma_s.b, g);
            rd_green = DiffusionReflection(ior, sigma_a.g, sigma_s.g, g);
            float avg_sigma_a = (sigma_a.r + sigma_a.g + sigma_a.b) / ((float) 3.0);
            float avg_sigma_s = (sigma_s.r + sigma_s.g + sigma_s.b) / ((float) 3.0);
            rd = DiffusionReflection(ior, avg_sigma_a, avg_sigma_s, g);

        }

        float calc_so_prime();

        Spectrum get_L_out(const Vector3D& po, const Vector3D& wo, Octree* o);

        float schlick_approximation_of_fr(Vector3D wo);

        DiffusionReflection rd;

        /**
         * Implements the Henyey-Greenstein phase function
         **/
        float phase_function(const Vector3D& wi, const Vector3D& wo);

    private:

        
        float diffuse_approximation(Vector3D po, Vector3D wo, Octree* o, DiffusionReflection diffuse_ref);
        Spectrum diffuse_approximation_3ch(Vector3D po, Vector3D wo, Octree* o);
        bool multiple_channels;

        inline double cos_world(const Vector3D& wi, const Vector3D& wo) {
            return dot(wi, wo) / (wi.norm() * wo.norm());
        }



        
        DiffusionReflection rd_red;
        DiffusionReflection rd_blue;
        DiffusionReflection rd_green;

    };
}

#endif //BSSRDF_H
