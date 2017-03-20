#include "lenscamera.h"

#include "image.h"

using namespace std;

namespace CGL {


/****** Helpers ******/
  

// Extract the R, G, or B channel out of an RGBA color stored in a single 32bit integer
static uint32_t red_channel(uint32_t color) {
    return (255 & (color >> (0)));
}

static uint32_t green_channel(uint32_t color) {
    return (255 & (color >> (8)));
}

static uint32_t blue_channel(uint32_t color) {
    return (255 & (color >> (16)));
}

// Convert from millimeters to meters
static const double scale = .001;







/****** LensElement functions ******/


bool LensElement::pass_through(Ray &r, double &prev_ior) const {
  // Part 1 Task 1: Implement this. It takes r and passes it through this lens element.
  Vector3D hit_p = Vector3D();
  if (!intersect(r, &hit_p)) {
    return false;
  }
  if (!refract(r, hit_p, prev_ior)) {
    return false;
  }
  prev_ior = ior; // are we still supposed to do this when radius == 0? (i think so)
  return true;
}
bool LensElement::intersect(const Ray &r, Vector3D *hit_p) const {
  // Part 1 Task 1: Implement this. It intersects r with this spherical lens elemnent 
  // (or aperture diaphragm). You'll want to reuse some sphere intersect code.
  Vector3D o = Vector3D(0., 0., center);
  if (radius == 0.0) {
    double t = (center - r.o.z) / r.d.z;
    *hit_p = r.o + r.d * t;
  } else {
    double r2 = radius * radius;
    double a = dot(r.d, r.d);
    double b = dot(2. * (r.o - o), r.d);
    double c = dot(r.o - o, r.o - o) - r2;
    double b24ac = pow(b, 2.) - 4*a*c;
    if (b24ac < 0) {
      return false;
    }
    double sqrtb24ac = sqrt(b24ac);
    double ta = (-b + sqrtb24ac) / (2. * a);
    double tb = (-b - sqrtb24ac) / (2. * a);
    double maxt = max(ta, tb), mint = min(ta, tb);
    if ((radius < 0 && r.d.z < 0) || (radius > 0 && r.d.z > 0)) { // i think this is right
      *hit_p = r.o + mint * r.d;
    } else {
      *hit_p = r.o + maxt * r.d;
    }
  }
  double dist_from_z_axis = pow((hit_p->x * hit_p->x) + (hit_p->y * hit_p->y), 0.5);
  return dist_from_z_axis <= aperture / 2;
}

bool LensElement::refract(Ray& r, const Vector3D& hit_p, const double& prev_ior) const {
  // Part 1 Task 1: Implement this. It refracts the Ray r with this lens element or 
  // does nothing at the aperture element.
  // You'll want to consult your refract function from the previous assignment.
  Vector3D n;
  Vector3D origin = Vector3D(0., 0., center);
  Matrix3x3 o2w = Matrix3x3();
  if (radius == 0.0) {
    r.o = hit_p;
    return true;
  }
  if (radius < 0) { // i *think* this condition is right (actually i don't think it matters)
    n = hit_p - origin;
  } else {
    n = origin - hit_p;
  }
  double no = prev_ior, ni = ior;  // this is right, (at least for o=r.d --> i)
  if (r.d.z > 0) {
    ni = prev_ior;
    no = ior;  // this is right, (at least for o=r.d --> i)
    n = -n;  // not sure if necessary, but he mentioned this in discussion.
  }
  n.normalize();

  make_coord_space(o2w, n);
  Matrix3x3 w2o = o2w.T();

  r.o = hit_p;
  r.d = w2o * r.d; // to obj coords

  double sin_theta_wi = (no/ni)*sin_theta(r.d);
  double cos_theta_wi = sqrt(1 - sin_theta_wi*sin_theta_wi);
  if (cos_theta(r.d) < 0) {
    cos_theta_wi *= -1;
  }
  double cos_phi_wi = cos_phi(r.d);
  double sin_phi_wi = sin_phi(r.d);
  r.d = Vector3D(sin_theta_wi*cos_phi_wi, sin_theta_wi*sin_phi_wi, cos_theta_wi);
  bool ret = sin_theta(r.d) < no/ni;
  r.d = o2w * r.d; // to world coords
  r.d.normalize();

  return ret;
}






/****** Lens functions ******/



void Lens::parse_lens_file(std::string filename) {

  ifstream infile(filename);
  string line;
  double z_coord = 0;
  double z_ap;
  vector<LensElement> backwards;
  elts.clear();
  bool first = true;
  while (getline(infile, line)) {
    if (first) {
      cout << "[Lens] Loading lens file " << line << endl;
      first = false;
    }
    if (line[0] == '#')
      continue;
    stringstream ss(line);
    LensElement lens;
    double offset;
    ss >> lens.radius >> offset >> lens.ior >> lens.aperture;
    lens.center = z_coord;
    if (!lens.radius) {
      z_ap = z_coord;
    }
    z_coord += offset;
    backwards.push_back(lens);
  }
  for (int i = backwards.size() - 1; i >= 0; --i) {
    LensElement l = backwards[i];
    l.center = (l.center - z_ap) + l.radius;
    if (i) l.ior = backwards[i-1].ior;
    else l.ior = 1;
    if (!l.ior) l.ior = 1;
    elts.push_back(l);
    if (!l.radius)
      ap_i = elts.size()-1;
    // cout << "Lens element edge first " << (l.center - l.radius) << " " 
    //   << l.radius << " " << l.center << " " << l.ior << " " << l.aperture << endl;
  }
  double c = elts.front().center, r = elts.front().radius, a = elts.front().aperture * .5;
  back_elt = c - (r>0?1:-1) * sqrt(r*r-a*a);
  ap_radius = ap_original = elts[ap_i].aperture;

  // Get infinity and close focus depths, also get focal length.
  set_focus_params();
  // Focus at infinity to start.
  sensor_depth = infinity_focus;
       
}


void Lens::set_focus_params() {

  // Part 1 Task 2: Implement this. 
  // After this function is called, the three variables
  // infinity_focus, near_focus, and focal_length
  // should be set correctly.
  double t;
  Vector3D intersect_x_plane;
  const double epsilon = ap_radius*0.1;

  // INFINITY FOCUS
  Ray r = Ray(Vector3D(epsilon, 0., std::numeric_limits<double>::min()),
              Vector3D(0., 0., 1.));
  trace_backwards(r); // r should now be the ray coming out of the final lens
  t = -r.o.x/r.d.x;
  intersect_x_plane = r.o + t * r.d;
  infinity_focus = intersect_x_plane.z;

  // FOCAL LENGTH
  t = -(epsilon - r.o.x) / r.d.x; // time it would take to go from r.o.x to epsilon at rate of -r.d.x
  double p_prime = r.o.z + t * -r.d.z;
  focal_length = abs(p_prime - infinity_focus);

  // NEAR FOCUS
  Vector3D o = Vector3D(0., 0., -5. * focal_length);
  Vector3D dir = Vector3D(epsilon, 0., elts[0].center - elts[0].radius) - o;
  r = Ray(o, dir.unit());
  trace_backwards(r); // r should now be the ray coming out of the final lens
  t = -r.o.x/r.d.x;
  intersect_x_plane = r.o + t * r.d;
  near_focus = intersect_x_plane.z;

  ////////////////////////////////////////////////////////////////////
  cout << "[Lens] Infinity focus depth is " << infinity_focus << endl;
  cout << "[Lens] Close focus depth is " << near_focus << endl;
  cout << "[Lens] True focal length is " << focal_length << endl;
}




bool Lens::trace(Ray &r, std::vector<Vector3D> *trace) const {
  // Part 1 Task 1: Implement this. It traces a ray from the sensor out into the world.

  double prev_ior = 1.;
  for (int i = 0; i < elts.size(); i++) {
    LensElement l = elts[i];
    if (!l.pass_through(r, prev_ior)) {
      return false;
    }
    if (trace != nullptr)
      trace->push_back(r.o);
  }

  return true;
}

bool Lens::trace_backwards(Ray &r, std::vector<Vector3D> *trace) const {
  // Part 1 Task 1: Implement this. It traces a ray from the world backwards through 
  // the lens towards the sensor.
  double prev_ior;
  for (int i = (int) elts.size()-1; i >= 0; i--) {
    if (i == 0) {
      prev_ior = 1.;
    } else {
      prev_ior = elts[i-1].ior;
    }
    LensElement l = elts[i];
    if (!l.pass_through(r, prev_ior)) {
      return false;
    }
    if (trace != nullptr)
      trace->push_back(r.o);
  }

  return true;
}

float Lens::focus_depth(float d) const {

  // Part 1 Task 2: Implement this. Should find the conjugate of a ray
  // starting from the sensor at depth d.
  const double epsilon = ap_radius*0.1;
  Vector3D o = Vector3D(0., 0., (double) d);
  double to_z = min(-1., elts[elts.size()-1.].center - elts[elts.size()-1.].radius);
  Vector3D dir = Vector3D(epsilon, 0., to_z) - o;
  Ray r = Ray(o, dir.unit());
  trace(r); // r should now be the ray coming out of the final lens
  double t = -r.o.x/r.d.x;
  Vector3D intersect_z_axis = r.o + t * r.d;
  return (float) intersect_z_axis.z;
}

Vector3D Lens::back_lens_sample() const {

  // Part 1 Task 2: Implement this. Should return a point randomly sampled
  // on the back element of the lens (the element closest to the sensor)
  double r = elts[0].aperture * 0.5,
         x_coord = r,
         y_coord = r,
         z_coord = elts[0].center - elts[0].radius;

  while (sqrt(x_coord*x_coord + y_coord*y_coord) > abs(r)) {
    x_coord = ((random_uniform() - 0.5) * 2.) * r;
    y_coord = ((random_uniform() - 0.5) * 2.) * r;
  }

  return Vector3D(x_coord, y_coord, z_coord);
}



/****** LensCamera functions ******/


LensCamera::LensCamera(): pt(NULL) {
  string path = string(__FILE__).substr(0,string(__FILE__).find_last_of('/')+1) + "../lenses/";
  static const vector<string> lens_files = {"dgauss.50mm.dat", "wide.22mm.dat", "telephoto.250mm.dat", "fisheye.10mm.dat"};
  for (string lens_file : lens_files)
    lenses.emplace_back(path + lens_file);

  mount_lens(0);
}


Ray LensCamera::generate_ray(double x, double y, int& num_tries, double& cos_factor) const {

  Ray r = Ray(Vector3D(),Vector3D() );
  if (lens_ind >= 0) {

    // Part 1 Task 2: Implement this. It generates a ray from sensor pixel (x,y)
    // pointing toward the back element of the lens (use back_lens_sample) and traces
    // it through the Lens (using your "trace" function)
    double film_d = sqrt(24*24+36*36);
    double screen_d = sqrt(screenW*screenW + screenH*screenH);
    double film_w = film_d * screenW / screen_d;
    double film_h = film_d * screenH / screen_d;
    Vector3D sensor_point(-(x-0.5)*film_w, -(y-0.5)*film_h, lenses[lens_ind].sensor_depth);
    bool continU = true;
    num_tries = 0;
    int MAX_NUM_TRIES = 8;
    while (continU && num_tries < MAX_NUM_TRIES) {
      Vector3D end = lenses[lens_ind].back_lens_sample(),
              dir = (end - sensor_point).unit();
      r.o = sensor_point;
      r.d = dir;
      cos_factor = pow(r.d.z, 4.);
      continU = !lenses[lens_ind].trace(r);
      num_tries++;
    }
    if (continU) {
      cos_factor = 0.;
      r = Ray(sensor_point, Vector3D(0., 0., 1.));
    }
    /***** end of your code ******/


    // This code converts the ray you traced through the lens into world coordinates.
    r.o = pos + c2w * r.o * scale;
    r.d = (c2w * r.d).unit();

  } else {

    // Generate ray for a pinhole camera. Same as in the previous assignment.
    num_tries = 1;
    x = 2*(x-.5); y = 2*(y-.5);
    r = Ray(pos,(c2w*Vector3D(x*tan(radians(hFov)*.5),y*tan(radians(vFov)*.5),-1)).unit());

  }

  r.min_t = nClip; r.max_t = fClip;
  return r;
}



void LensCamera::move_sensor(float delta) {
  if (lens_ind < 0) return;
  curr_lens().sensor_depth += delta;
  cout << "[LensCamera] Sensor plane moved to " << curr_lens().sensor_depth
       << ", focus now at " << lenses[lens_ind].focus_depth(lenses[lens_ind].sensor_depth) << endl;
}

void LensCamera::stop_down(float ratio) {
  float ap = curr_lens().ap_radius * ratio;
  if (ap > curr_lens().ap_original) ap = curr_lens().ap_original;
  curr_lens().ap_radius = ap;
  cout << "[LensCamera] Aperture is now " << curr_lens().ap_radius << "mm" << endl;
}

void LensCamera::mount_lens(int i) {
  lens_ind = i;
  if (i >= 0) {
    cout << "[LensCamera] Switched to lens #" << (i+1) 
         << " with focal length " << curr_lens().focal_length << "mm" << endl;
  } else {
    cout << "[LensCamera] Switched to pinhole camera" << endl;
  }
}



// A dummy function to demonstrate how to work with the image buffer.
// Calculates the average value of the green color channel in the image.
// You'll have to remember your 2D array indexing in order to take differences
// of neighboring pixels in a more sophisticated metric function.
static double mean_green(const ImageBuffer& ib) {
  double sum = 0;
  for (int i = 0; i < ib.w * ib.h; ++i) {
      sum += green_channel(ib.data[i]);
  }
  double mean = sum / (ib.w * ib.h);
  
  return mean;
}
 static double mean_red(const ImageBuffer& ib) {
   double sum = 0;
   for (int i = 0; i < ib.w * ib.h; ++i) {
     sum += red_channel(ib.data[i]);
   }
   double mean = sum / (ib.w * ib.h);

   return mean;
 }
  static double mean_blue(const ImageBuffer& ib) {
    double sum = 0;
    for (int i = 0; i < ib.w * ib.h; ++i) {
      sum += blue_channel(ib.data[i]);
    }
    double mean = sum / (ib.w * ib.h);

    return mean;
  }


double focus_metric_global(const ImageBuffer& ib)  {

    // Part 2 Task 1: Implement this. Design a metric to judge how "in-focus"
    // the image patch stored in the provided ImageBuffer is.
    const double mean_r = mean_red(ib),
            mean_g = mean_green(ib),
            mean_b = mean_blue(ib);
    // x, y is y * width + x
    double heuristic = 0.;
    for (int x = 0; x < ib.w; x++) {
      for (int y = 0; y < ib.h; y++) {
        uint32_t xy_pixel = ib.data[y * ib.w + x];
        heuristic += pow(mean_r - (double) red_channel(xy_pixel), 2.);
        heuristic += pow(mean_g - (double) green_channel(xy_pixel), 2.);
        heuristic += pow(mean_b - (double) blue_channel(xy_pixel), 2.);
      }
    }
    // sharper images will have higher variance
    return heuristic;
  }

double LensCamera::focus_metric(const ImageBuffer& ib) const {

  // Part 2 Task 1: Implement this. Design a metric to judge how "in-focus"
  // the image patch stored in the provided ImageBuffer is.
  return focus_metric_global(ib);
}


void LensCamera::autofocus() {


  // Part 2 Task 2: Implement this. Design a global search using your 
  // focus metric to set the sensor to be at the depth where the 
  // render cell is most "in focus". Provided code shows how to 
  // move the sensor, request a render of the cell, and evaluate the focus metric.

  // This call ensures that your pathtracer is rendering at high enough quality.
  // Increase samples per pixel to 16 and samples per light to 16.

  pt->bump_settings();

  // Example code. Nothing to do with your actual implementation except to 
  // demonstrate functionality.
  int MODE = 0;

  ImageBuffer ib;
  double const step_size = 2. * sqrt(36*36 + 24*24) / sqrt(screenW*screenW + screenH*screenH);
  double best_depth = curr_lens().infinity_focus;
  double best_variance = 0.;

  if (MODE == 0) {
    for (curr_lens().sensor_depth = curr_lens().infinity_focus; curr_lens().sensor_depth <= curr_lens().near_focus;
         curr_lens().sensor_depth += step_size) {
      printf("Trying depth %f (%f to %f) ... ", curr_lens().sensor_depth, curr_lens().infinity_focus, curr_lens().near_focus);

      pt->raytrace_cell(ib);
      double heuristic = focus_metric(ib);
      printf("Heuristic is %f\n", heuristic);
      if (heuristic > best_variance) {
        best_variance = heuristic;
        best_depth = curr_lens().sensor_depth;
      }
    }
  } else if (MODE == 1) {
    const double gr = (sqrt(5.) - 1) / 2.;
    double a = curr_lens().infinity_focus - (step_size / 5.),
           b = curr_lens().near_focus + (step_size / 5.),
           c = b - gr * (b - a);
    curr_lens().sensor_depth = c;
    pt->raytrace_cell(ib);
    printf("Trying depth %f (%f to %f) ... ", curr_lens().sensor_depth, curr_lens().infinity_focus,
                                              curr_lens().near_focus);
    double c_score = focus_metric(ib);
    printf("Heuristic is %f\n", c_score);

    while (abs(b-a) > step_size) {
      double d = c + gr * (b - c);
      curr_lens().sensor_depth = d;
      printf("Trying depth %f (%f to %f) ... ", curr_lens().sensor_depth, curr_lens().infinity_focus,
                                              curr_lens().near_focus);
      pt->raytrace_cell(ib);
      double d_score = focus_metric(ib);
      printf("Heuristic is %f\n", d_score);
      if (d_score > c_score) {
        a = c;
        c = d;
        c_score = d_score;
      } else {
        b = a;
        a = d;
      }
    }
    best_depth = (a + b) / 2.;
  }
  curr_lens().sensor_depth = best_depth;
  printf("Chose depth %f\n", best_depth);
  pt->raytrace_cell(ib);
}





void LensCamera::dump_settings(string filename) {
  ofstream file(filename);
  file << hFov << " " << vFov << " " << ar << " " << nClip << " " << fClip << endl;
  for (int i = 0; i < 3; ++i)
    file << pos[i] << " ";
  for (int i = 0; i < 3; ++i)
    file << targetPos[i] << " ";
  file << endl;
  file << phi << " " << theta << " " << r << " " << minR << " " << maxR << endl;
  for (int i = 0; i < 9; ++i)
    file << c2w(i/3, i%3) << " ";
  file << endl;
  file << screenW << " " << screenH << " " << screenDist << endl;

  file << lens_ind << endl;
  for (Lens &lens : lenses) {
    file << lens.sensor_depth << " ";
  }
  file << endl;

  cout << "[LensCamera] Dumped settings to " << filename << endl;
}

void LensCamera::load_settings(string filename) {
  ifstream file(filename);

  file >> hFov >> vFov >> ar >> nClip >> fClip;
  for (int i = 0; i < 3; ++i)
    file >> pos[i];
  for (int i = 0; i < 3; ++i)
    file >> targetPos[i];
  file >> phi >> theta >> r >> minR >> maxR;
  for (int i = 0; i < 9; ++i)
    file >> c2w(i/3, i%3);
  file >> screenW >> screenH >> screenDist;

  file >> lens_ind;
  for (Lens &lens : lenses) {
    file >> lens.sensor_depth;
  }

  cout << "[LensCamera] Loaded settings from " << filename << endl;
}


} // namespace CGL

