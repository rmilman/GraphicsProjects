#ifndef CGL_LENSCAMERA_H
#define CGL_LENSCAMERA_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include "camera.h"
#include "bsdf.h"
#include "random_util.h"
#include "pathtracer.h"

namespace CGL {

class PathTracer;

struct LensElement {
public:
  bool pass_through(Ray &r, double &prev_ior) const;
  double center;
  double radius;
  double ior;
  double aperture;
private:
  bool intersect(const Ray &r, Vector3D *hit_p) const;
  bool refract(Ray& r, const Vector3D& hit_p, const double& prev_ior) const;
};

struct Lens {

  Lens(std::string filename) { parse_lens_file(filename); }

  void parse_lens_file(std::string filename);

  void set_focus_params();

  bool trace(Ray &r, std::vector<Vector3D> *trace = NULL) const;
  bool trace_backwards(Ray &r, std::vector<Vector3D> *trace = NULL) const;

  float focus_depth(float d) const;

  Vector3D back_lens_sample() const;

  mutable std::vector<LensElement> elts;
  double back_elt, infinity_focus, near_focus, focal_length;
  double ap_radius, ap_original;
  size_t ap_i;
  double sensor_depth;
}; 

class LensCamera : public Camera {
public:
  LensCamera();

  Ray generate_ray(double x, double y, int& rays_tried, double& cos4_term) const;

  void move_sensor(float delta);
  void stop_down(float ratio);

  Lens& curr_lens() { return lenses[lens_ind]; }
  void mount_lens(int i);

  double focus_metric(const ImageBuffer& ib) const;

  void autofocus();

  void dump_settings(std::string filename);
  void load_settings(std::string filename);

  std::vector<Lens> lenses;
  int lens_ind;
  PathTracer *pt;
};


} // namespace CGL

#endif // CGL_LENSCAMERA_H
