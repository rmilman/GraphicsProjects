#include "lenscamera.h"

#include "image.h"
#include "math.h"
#include "CGL/lodepng.h"

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
  Vector3D hit_p;
  if (intersect(r, &hit_p)){
    // if (radius == 0) { //aperture element
    //     //Ray g = Ray(hit_p,r.d); //hit_p
    //     //cout<<hit_p.x<<" new aperture ray "<<r.d.x<<endl;
    //     //r = g;
    //     prev_ior = ior;
    //     return true;
    if (refract(r, hit_p, prev_ior)){
      if(radius != 0.){
   // prev_ior = this->ior;
       prev_ior = ior;}

      return true;
    }
  }
  return false;
}

bool LensElement::intersect(const Ray &r, Vector3D *hit_p) const {
  // Part 1 Task 1: Implement this. It intersects r with this spherical lens elemnent 
  // (or aperture diaphragm). You'll want to reuse some sphere intersect code.

  //if aperature element: plane intersection else sphere element
  Vector3D cen = Vector3D(0,0,center);
  double t = 0.0;
  if (radius == 0){
    double t = (center-r.o[2])/r.d[2]; //intersects plane
    return true;
  }
  //sphere intersect
  double a = dot(r.d,r.d);
  double b = dot(2*(r.o-cen), r.d);
  double c = dot((r.o-cen),(r.o-cen))-radius*radius;
  double d = b*b-4*a*c;
  if (d<0)return false;
  double tp = (-b+sqrt(d))/(2*a);
  double tm = (-b-sqrt(d))/(2*a);
  double t1 = min(tp,tm);
  double t2 = max(tp,tm);
    
  if (radius*r.d.z>0){
    t = t1; //debugging journey
  }else{
    t = t2;  
  }
  *hit_p = r.o+r.d*t;
  if ((hit_p->x*hit_p->x + hit_p->y*hit_p->y) > (aperture*aperture*.25)) { // missed element, debugging journey squared aperature
      return false;
  }
  return true;
}

bool LensElement::refract(Ray& r, const Vector3D& hit_p, const double& prev_ior) const {
  // Part 1 Task 1: Implement this. It refracts the Ray r with this lens element or 
  // does nothing at the aperture element.
  // You'll want to consult your refract function from the previous assignment.
  if (radius == 0){
    return true;
  }

  Matrix3x3 o2w;
  Vector3D cen = Vector3D(0,0,center);
  make_coord_space(o2w,(hit_p-cen)/radius);
  Matrix3x3 w2o = o2w.T();
  Vector3D wo = (w2o*r.d);

  float sign = 1.0f;
  double ni, no;
  if (r.d.z < 0) { //forward ray needs to flip normal
    no = prev_ior; 
    ni = ior;
    sign = -1.0f; 
  }else{//backward ray
    ni = prev_ior; 
    no = ior;
  }

  Vector3D n = Vector3D(0,0,sign);
  double p = no/ni; //no/ni
  double c = dot(-n, wo); //- vs positive
  Vector3D v = p*wo + (p*c - sqrt(1-p*p*(1-c*c)))*n;
  Ray g = Ray(hit_p, o2w*v);//-wo, parentheses d= v debugging journey v to world coordinates
  
  if (no*sin_theta(wo) >= ni){
    return false;
  }
  r = g;
  return true;


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
  double epsilon = elts.back().aperture/100;
  double infinity = 1000000;//HUGE_VAL; //debugging journey


  //infinity_focus: trace ray backwards from infinity through small paraxial distance
  Ray test0 = Ray(Vector3D(epsilon,0.0,-1*infinity), Vector3D(0.0,0.0,1)); 
  Ray test1 = Ray(Vector3D(epsilon,0.0,-1*infinity), Vector3D(0.0,0.0,1)); 
  
  bool b = trace_backwards(test1);
  double tI = -test1.o.x/test1.d.x; //only checking where x intersects because I have no y component in original ray
  infinity_focus = (test1.o+test1.d*tI).z; 


  //focus_length: Finding P'-infinity focus using the intersection of infinite ray 
    //and its final out ray to inf focus
  //Intersection of test0 and test1
  //think of test 0 as plane at epsilon = z
  double tf = (epsilon-test1.o[0])/test1.d[0]; //intersects plane
  double pPrime = (test1.o+test1.d*tf).z; 
  focal_length = abs(pPrime-infinity_focus);


  //near_focus: trace ray backwards from near focus object through small paraxial distance
  //r.d = (Vector3D(epsilon,0,front_lens_z) - r.o).unit();
  //front_lens_z == elts.back().center-elts.back().radius
  // r.o= elts.back().center - elts.back().radius - (1 + log(focal_length))*focal_length)

   Vector3D near_orig(0, 0, elts.back().center - elts.back().radius - (1 + log(focal_length))*focal_length);
   Ray test2 = Ray(near_orig, (Vector3D(epsilon, 0, elts.back().center-elts.back().radius) - near_orig).unit());
  //Ray test2 = Ray(Vector3D(epsilon,0.0, 5*focal_length), Vector3D(0,0,0)); //debugging journey wrong origin x value
  //test2.o = Vector3D(0,0,elts.back().center - elts.back().radius - (1 + log(focal_length))*focal_length); //Piazza post help
  //test2.d = (Vector3D(epsilon,0,elts.back().center-elts.back().radius) - test2.o).unit();
 
  trace_backwards(test2);
  double tn = 0.0-test2.o[0]/test2.d[0]; //only checking where x intersects x=0 because I have no y component in original ray
  near_focus = (test2.o+test2.d*tn).z; //problem with near focus



  cout << "[Lens] Infinity focus depth is " << infinity_focus << endl;
  cout << "[Lens] Close focus depth is " << near_focus << endl;
  cout << "[Lens] True focal length is " << focal_length << endl;
}




bool Lens::trace(Ray &r, std::vector<Vector3D> *trace) const {
  // Part 1 Task 1: Implement this. It traces a ray from the sensor out into the world.
  double prev_ior = 1.0;
  bool passed = true;
  //bool passed = elts[0].pass_through(r,start_ior);  
  for (int i=0; i < elts.size(); i++){ //elts.size()  //i=1
    if (passed){
      //cout << "Ray Origin: x "<< r.o.x << " y " << r.o.y << " z " << r.o.z <<endl;
      passed = elts[i].pass_through(r, prev_ior); //elts[i-1].ior
      if (trace!=NULL)trace->push_back(r.o);   
    }else{
      return false;
    }
  }
  return passed;
}

bool Lens::trace_backwards(Ray &r, std::vector<Vector3D> *trace) const {
  // Part 1 Task 1: Implement this. It traces a ray from the world backwards through 
  // the lens towards the sensor.
  bool passed = true;
  double prev_ior = 1.0;
  for (int i = elts.size()-1; i >= 0; --i) {
    if (passed){
       prev_ior = i > 0 ? elts[i-1].ior : 1;
       passed = elts[i].pass_through(r,prev_ior);
       //cout<<"ray final origin x" << r.o.x << " ray final dir x"<< r.d.x << endl;
        if (trace!=NULL)trace->push_back(r.o);   
    }
  }
  return true;
}

float Lens::focus_depth(float d) const {

  // Part 1 Task 2: Implement this. Should find the conjugate of a ray
  // starting from the sensor at depth d.
  double epsilon = elts[0].aperture/600;
  double infinity = 100000;

  Ray test = Ray(Vector3D(0.0,0.0,d), Vector3D(epsilon,0,-1).unit());

  trace(test);
  double t = -test.o[0]/test.d[0]; //only checking where x intersects because I have no y component in original ray
  return (test.o+test.d*t).z;
  //return 1/(1/infinity_focus-1/d);

}

Vector3D Lens::back_lens_sample() const {

  // Part 1 Task 2: Implement this. Should return a point randomly sampled
  // on the back element of the lens (the element closest to the sensor)
  double aperture = elts.front().aperture;
  double radius = elts.front().radius;
  double center = elts.front().center;
  //double d = elts[0].aperture;
  // Vector2D sample = Vector2D(d*(random_uniform()-.5), d*(random_uniform()-.5)); //change to shift

  // while (sample.x*sample.x + sample.y*sample.y > d*d/4) {
  //   sample = Vector2D(d * (random_uniform()-.5), d * (random_uniform()-.5));
  // }
  // double better_z = center - (radius > 0 ? 1 : -1) * sqrt(radius * radius - aperture * aperture * 0.25);
  // return Vector3D(sample.x,sample.y,better_z);

  double theta = 2.0*M_PI*random_uniform();
  double r = aperture*.5*sqrt(random_uniform());
  double x = r*cos(theta);
  double y = r*sin(theta);
  double better_z = center - (radius > 0 ? 1 : -1) * sqrt(radius * radius - aperture * aperture * 0.25);
  return Vector3D(x,y,better_z);
}



/****** LensCamera functions ******/


LensCamera::LensCamera(): pt(NULL) {
  string path = string(__FILE__).substr(0,string(__FILE__).find_last_of('/')+1) + "../lenses/";
  static const vector<string> lens_files = {"dgauss.50mm.dat", "wide.22mm.dat", "telephoto.250mm.dat", "fisheye.10mm.dat"};
  for (string lens_file : lens_files)
    lenses.emplace_back(path + lens_file);

  mount_lens(0);
}


Ray LensCamera::generate_ray(double x, double y, int& rays_tried, double& cos4_term) const {

  Ray r = Ray(Vector3D(),Vector3D());
  if (lens_ind >= 0) {

    // Part 1 Task 2: Implement this. It generates a ray from sensor pixel (x,y)
    // pointing toward the back element of the lens (use back_lens_sample) and traces
    // it through the Lens (using your "trace" function)
    double film_d = sqrt(24*24+36*36);
    double screen_d = sqrt(screenW*screenW + screenH*screenH);
    double film_w = film_d * screenW / screen_d;
    double film_h = film_d * screenH / screen_d;
    Vector3D sensor_point(-(x-0.5)*film_w, -(y-0.5)*film_h, lenses[lens_ind].sensor_depth);

    r = Ray(sensor_point, lenses[lens_ind].back_lens_sample()-sensor_point);
    r.d.normalize();
    cos4_term = pow(r.d.z,4);     
    bool b = (lenses[lens_ind]).trace(r);      
    while (!b) {
       r.d = lenses[lens_ind].back_lens_sample()-sensor_point;
       r.d.normalize();
       cos4_term = pow(r.d.z,4);
       r.o = sensor_point; //debugging error
       b = (lenses[lens_ind]).trace(r);

       rays_tried+=1;
      if (rays_tried > 20){
        r.o = Vector3D(); //debugging error
        r.d = Vector3D(0,0,1);
        cos4_term = 0;
        break;
      } 
    }
    //cout<<cos4_term<<endl;
     //r.o = sensor_point;

    /***** end of your code ******/

    // This code converts the ray you traced through the lens into world coordinates.
    r.o = pos + c2w * r.o * scale;
    r.d = (c2w * r.d).unit();
     //cout<<r.o.x<<"world y " << r.o.y << " z "<< r.o.z<<endl;
    //cout<<"direction z "<<r.d.z<<endl;

  } else {

    // Generate ray for a pinhole camera. Same as in the previous assignment.
    x = 2*(x-.5); y = 2*(y-.5);
    r = Ray(pos,(c2w*Vector3D(x*tan(radians(hFov)*.5),y*tan(radians(vFov)*.5),-1)).unit());

  }
  //rays_tried = 1;
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

double LensCamera::focus_metric(const ImageBuffer& ib) const {

  // Part 2 Task 1: Implement this. Design a metric to judge how "in-focus"
  // the image patch stored in the provided ImageBuffer is.

//   Implement focus_metric, which takes an ImageBuffer instance and computes a value that
//    is higher when the image is more in focus. For this task we want you to implement the
//     metric as the variance of the image patch. Blurrier (out of focus) image patches will
//      have lower variance, and sharper ones will have higher variance (why?). This is easily
//       confused with noise however, so make sure to pay attention to tip 1 above.

// Some helper functions at the top of lenscamera.cpp allow you to extract individual
//  color channels from the ImageBuffer uint32_t data members. Starter code mean_green
//   shows a demo of how to work with the ImageBuffer.

// You can evaluate your metric on the currently selected render cell (with the current
//  sample rate and ray depth settings) by pressing A in cell render mode. The metric 
// will be written to the command line.

 // variance = expectation of (x- average)^2
  double avg_r = 0.0;
  double avg_g = 0.0;
  double avg_b = 0.0;
  for (int i = 0; i < ib.h * ib.w; i++){
    avg_r += red_channel(ib.data[i]);
    avg_g += green_channel(ib.data[i]);
    avg_b += blue_channel(ib.data[i]);
  }
  avg_r = avg_r/(ib.h * ib.w);
  avg_g = avg_g/(ib.h * ib.w);
  avg_b = avg_b/(ib.h * ib.w);

  double var_r = 0.0;
  double var_g = 0.0;
  double var_b = 0.0;
  for (int i = 0; i < ib.h * ib.w; i++){
    var_r += (red_channel(ib.data[i])-avg_r)*(red_channel(ib.data[i])-avg_r);
    var_g += (green_channel(ib.data[i])-avg_g)*(green_channel(ib.data[i])-avg_g);
    var_b += (blue_channel(ib.data[i])-avg_b)*(blue_channel(ib.data[i])-avg_b);
  }
  var_r = var_r/(ib.h * ib.w);
  var_g = var_g/(ib.h * ib.w);
  var_b = var_b/(ib.h * ib.w);

  //return max(var_r, max(var_g, var_b)); 
  return (var_r+var_b+var_g)/3;
}


void LensCamera::autofocus() {


  // Part 2 Task 2: Implement this. Design a global search using your 
  // focus metric to set the sensor to be at the depth where the 
  // render cell is most "in focus". Provided code shows how to 
  // move the sensor, request a render of the cell, and evaluate the focus metric.

  // This call ensures that your pathtracer is rendering at high enough quality.
  // Increase samples per pixel to 16 and samples per light to 16.
  pt->bump_settings();
  
  //cout << "[LensCamera] The variance is " << focus_metric(ib) << endl;
  double num_steps = 100.0;
  double step_size = (curr_lens().near_focus-curr_lens().infinity_focus)/num_steps;
  double max_var = 0.0;
  double curr_var = 0.0;
  int focused_i = 0.0;
  double sensor_depth_start = curr_lens().infinity_focus;
  curr_lens().sensor_depth = sensor_depth_start;
  for (int i = 0; i < num_steps; i++){
    curr_lens().sensor_depth+=step_size;
    ImageBuffer ib;
    pt->raytrace_cell(ib);
    curr_var = focus_metric(ib);
    //ib.data.clear();
    //ib.data.clear();
    if (curr_var > max_var){
      focused_i = i;
      max_var = curr_var;
     }
    // if (curr_var < .7*max_var){
    //   break;
    // }
    // Use your focus_depth function to get the world-side conjugates 
     //for 100 evenly spaced sensor depths ranging from close focus to infinity. 
     //Plot sensor depth vs. conjugate to show the approximate inverse relationship.
      //cout << "[LensCamera] The variance is " << curr_var << " i is " << focused_i << " step " << i<< endl;
   //cout << "["<< curr_lens().sensor_depth<<", "<< curr_lens().focus_depth(curr_lens().sensor_depth)<<"]"<< ","<<endl;
  cout << "["<< curr_lens().sensor_depth<<", "<< curr_var <<"]"<< ","<<endl;
  }
  curr_lens().sensor_depth = sensor_depth_start+focused_i*step_size;
  //pt->raytrace_cell(ib);
  
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

