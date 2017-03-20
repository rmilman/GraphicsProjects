#include "pathtracer.h"
#include "bsdf.h"
#include "ray.h"
#include "point.h"
#include "octree.h"
#include "bssrdf.h"

#include "lenscamera.h"

#include <stack>
#include <random>
#include <algorithm>
#include <sstream>
#include <time.h>

#include "CGL/CGL.h"
#include "CGL/vector3D.h"
#include "CGL/matrix3x3.h"
#include "CGL/lodepng.h"

#include "GL/glew.h"

#include "static_scene/sphere.h"
#include "static_scene/triangle.h"
#include "static_scene/light.h"


using namespace CGL::StaticScene;

using std::min;
using std::max;

namespace CGL {

PathTracer::PathTracer(size_t ns_aa,
                       size_t max_ray_depth, size_t ns_area_light,
                       size_t ns_diff, size_t ns_glsy, size_t ns_refr,
                       size_t num_threads, HDRImageBuffer* envmap,
                       string filename) {
      state = INIT,
      this->ns_aa = ns_aa;
      this->max_ray_depth = max_ray_depth;
      this->ns_area_light = ns_area_light;
      this->ns_diff = ns_diff;
      this->ns_glsy = ns_diff;
      this->ns_refr = ns_refr;
      this->filename = filename;

      if (envmap) {
        this->envLight = new EnvironmentLight(envmap);
      } else {
        this->envLight = NULL;
      }

      bvh = NULL;
      octopus = NULL;
      scene = NULL;
      camera = NULL;

      gridSampler = new UniformGridSampler2D();
      hemisphereSampler = new UniformHemisphereSampler3D();

      show_rays = true;

      imageTileSize = 32;
      numWorkerThreads = num_threads;
      workerThreads.resize(numWorkerThreads);

      tm_gamma = 2.2f;
      tm_level = 1.0f;
      tm_key = 0.18;
      tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete octopus;
  delete bvh;
  delete gridSampler;
  delete hemisphereSampler;

}

void PathTracer::set_scene(Scene *scene) {

  if (state != INIT) {
    return;
  }

  if (this->scene != nullptr) {
    delete scene;
    delete bvh;
    delete octopus;
    selectionHistory.pop();
    OCTselectionHistory.pop();
  }

  if (this->envLight != nullptr) {
    scene->lights.push_back(this->envLight);
  }

  this->scene = scene;
  build_accel();
  create_octree();
  if (has_valid_configuration()) {
    state = READY;
  }
}

void PathTracer::set_camera(LensCamera *camera) {

  if (state != INIT) {
    return;
  }

  this->camera = camera;
  this->camera->pt = this;

  if (has_valid_configuration()) {
    state = READY;
  }

}

void PathTracer::set_frame_size(size_t width, size_t height) {
  if (state != INIT && state != READY) {
    stop();
  }
  sampleBuffer.resize(width, height);
  frameBuffer.resize(width, height);
  cell_tl = Vector2D(0,0); 
  cell_br = Vector2D(width, height);
  render_cell = false;
  if (has_valid_configuration()) {
    state = READY;
  }
}

bool PathTracer::has_valid_configuration() {
  return scene && camera && gridSampler && hemisphereSampler &&
         (!sampleBuffer.is_empty());
}

void PathTracer::update_screen() {
  switch (state) {
    case INIT:
    case READY:
      break;
    case VISUALIZE:
      visualize_accel();
      break;
    case VISUALIZE_OCT:
      visualize_oct();
      break;
    case RENDERING:
      glDrawPixels(frameBuffer.w, frameBuffer.h, GL_RGBA,
                   GL_UNSIGNED_BYTE, &frameBuffer.data[0]);
      if (render_cell)
        visualize_cell();
      break;
    case VISUALIZE_POINTS:
      visualize_surface_points();
      break;
    case DONE:
        //sampleBuffer.tonemap(frameBuffer, tm_gamma, tm_level, tm_key, tm_wht);
      glDrawPixels(frameBuffer.w, frameBuffer.h, GL_RGBA,
                   GL_UNSIGNED_BYTE, &frameBuffer.data[0]);
      if (render_cell)
        visualize_cell();
      break;
  }
}

void PathTracer::stop() {
  switch (state) {
    case INIT:
    case READY:
      break;
    case VISUALIZE:
      while (selectionHistory.size() > 1) {
        selectionHistory.pop();
      }
      state = READY;
      break;
    case VISUALIZE_OCT:
      while (OCTselectionHistory.size() > 1) {
        OCTselectionHistory.pop();
      }
      state = READY;
      break;
    case VISUALIZE_POINTS:
      while (selectionHistory.size() > 1) {
        selectionHistory.pop();
      }
      state = READY;
      break;
    case RENDERING:
      continueRaytracing = false;
    case DONE:
      for (int i=0; i<numWorkerThreads; i++) {
            workerThreads[i]->join();
            delete workerThreads[i];
        }
      state = READY;
      break;
  }
  render_silent = false;
}

void PathTracer::clear() {
  if (state != READY) return;
  delete bvh;
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  octopus = NULL;
  selectionHistory.pop();
  OCTselectionHistory.pop();
  sampleBuffer.resize(0, 0);
  frameBuffer.resize(0, 0);
  state = INIT;
  render_cell = false;
}

void PathTracer::visualize_surface_points() {

    if (visualization_points.empty()) {
        cout << "Generating new sample points..."<< endl;
        visualization_points = sample_surface_points();
    }

  // code stolen from visualize_accel(), creates the
  // general outline of the mesh to color points on
  glPushAttrib(GL_ENABLE_BIT);
  glDisable(GL_LIGHTING);
  glLineWidth(1);
  glEnable(GL_DEPTH_TEST);

  // hardcoded color settings
  Color cnode = Color(.5, .5, .5, .25);
  Color cnode_hl = Color(1., .25, .0, .6);
  Color cnode_hl_child = Color(1., 1., 1., .6);

  Color cprim_hl_left = Color(.6, .6, 1., 1);
  Color cprim_hl_right = Color(.8, .8, 1., 1);
  Color cprim_hl_edges = Color(0., 0., 0., 0.5);

  BVHNode *selected = selectionHistory.top();

  // render solid geometry (with depth offset)
  glPolygonOffset(1.0, 1.0);
  glEnable(GL_POLYGON_OFFSET_FILL);

  if (selected->isLeaf()) {
    bvh->draw(selected, cprim_hl_left);
  } else {
    bvh->draw(selected->l, cprim_hl_left);
    bvh->draw(selected->r, cprim_hl_right);
  }

  glDisable(GL_POLYGON_OFFSET_FILL);

  // draw geometry outline
  bvh->drawOutline(selected, cprim_hl_edges);
  // test line for visualization of bounding sphere
  // bounding_sphere.draw(Color(.5, .5, .5, .25)); 
  glPopAttrib();


  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);


  // draw the found surface points
  draw_points(visualization_points, Color(1.f,0.f,0.f,1.f));
}

void PathTracer::start_visualizing() {
  if (state != READY) {
    return;
  }
  state = VISUALIZE;
}

void PathTracer::start_visualizing_oct() {
  if (state != READY) {
    return;
  }
  state = VISUALIZE_OCT;
}

void PathTracer::start_visualizing_points() {
  if (state != READY) {
    return;
  }
  state = VISUALIZE_POINTS;
}

void PathTracer::start_raytracing() {
  if (state != READY) return;

  // Intersection isect;
  // Ray r = camera->center_ray();
  // if (camera->lens_ind >= 0&& bvh->intersect(r, &isect)) {
  //   camera->focus_at(isect.t);
  // }

  rayLog.clear();
  workQueue.clear();

  state = RENDERING;
  continueRaytracing = true;
  workerDoneCount = 0;

  sampleBuffer.clear();
  if (!render_cell) {
    frameBuffer.clear();
    num_tiles_w = sampleBuffer.w / imageTileSize + 1;
    num_tiles_h = sampleBuffer.h / imageTileSize + 1;
    tilesTotal = num_tiles_w * num_tiles_h;
    tilesDone = 0;
    tile_samples.resize(num_tiles_w * num_tiles_h);
    memset(&tile_samples[0], 0, num_tiles_w * num_tiles_h * sizeof(int));

    // populate the tile work queue
    for (size_t y = 0; y < sampleBuffer.h; y += imageTileSize) {
        for (size_t x = 0; x < sampleBuffer.w; x += imageTileSize) {
            workQueue.put_work(WorkItem(x, y, imageTileSize, imageTileSize));
        }
    }
  } else {
    int w = (cell_br-cell_tl).x;
    int h = (cell_br-cell_tl).y;
    int imTS = imageTileSize / 4;
    num_tiles_w = w / imTS + 1;
    num_tiles_h = h / imTS + 1;
    tilesTotal = num_tiles_w * num_tiles_h;
    tilesDone = 0;
    tile_samples.resize(num_tiles_w * num_tiles_h);
    memset(&tile_samples[0], 0, num_tiles_w * num_tiles_h * sizeof(int));

    // populate the tile work queue
    for (size_t y = cell_tl.y; y < cell_br.y; y += imTS) {
        for (size_t x = cell_tl.x; x < cell_br.x; x += imTS) {
            workQueue.put_work(WorkItem(x, y, imTS, imTS));
        }
    }
  }

  bvh->total_isects = 0; bvh->total_rays = 0;
  // launch threads
  if (!render_silent)  fprintf(stdout, "[PathTracer] Rendering... "); fflush(stdout);
  for (int i=0; i<numWorkerThreads; i++) {
      workerThreads[i] = new std::thread(&PathTracer::worker_thread, this);
  }
}

void PathTracer::render_to_file(string filename) {
  unique_lock<std::mutex> lk(m_done);
  start_raytracing();
  cv_done.wait(lk, [this]{ return state == DONE; });
  lk.unlock();
  save_image(filename);
  if (!render_silent)  fprintf(stdout, "[PathTracer] Job completed.\n");
}


void PathTracer::build_accel() {

  // collect primitives //
  fprintf(stdout, "[PathTracer] Collecting primitives... "); fflush(stdout);
  timer.start();
  vector<Primitive *> primitives;
  for (SceneObject *obj : scene->objects) {
    const vector<Primitive *> &obj_prims = obj->get_primitives();
    primitives.reserve(primitives.size() + obj_prims.size());
    primitives.insert(primitives.end(), obj_prims.begin(), obj_prims.end());
  }
  timer.stop();
  fprintf(stdout, "Done! (%.4f sec)\n", timer.duration());

  // build BVH //
  fprintf(stdout, "[PathTracer] Building BVH from %lu primitives... ", primitives.size()); 
  fflush(stdout);
  timer.start();
  bvh = new BVHAccel(primitives);
  timer.stop();
  fprintf(stdout, "Done! (%.4f sec)\n", timer.duration());

  // initial visualization //
  selectionHistory.push(bvh->get_root());
}
void PathTracer::visualize_oct() {
  if (octopus == NULL) {
    cout << "OCTREE IS NULL" << endl;
    return;
  }
  glPushAttrib(GL_ENABLE_BIT);
  glDisable(GL_LIGHTING);
  glLineWidth(1);
  glEnable(GL_DEPTH_TEST);

  // hardcoded color settings
  Color dark = Color(.5, .5, .5, .25);
  Color orange = Color(1., .25, .0, .6);
  Color white = Color(1., 1., 1., .6);

  Color cprim_hl_1 = Color(1., .619, .619, 1.); //pink
  Color cprim_hl_2 = Color(.941, .619, 1., 1); //purple
  Color cprim_hl_3 = Color(.658, .619, 1., 1); //indigo
  Color cprim_hl_4 = Color(.619, .88, 1., 1); //blue
  Color cprim_hl_5 = Color(.619, 1, .854, 1); //green
  Color cprim_hl_6 = Color(.858, 1, .619, 1); //lime
  Color cprim_hl_7 = Color(1., .94, .619, 1);  //yellow
  Color cprim_hl_8 = Color(1., .729, .619, 1); //orange
  Color edges = Color(0, 0, 0, .5); //white
  Color colors[] = {cprim_hl_1,cprim_hl_2,cprim_hl_3,cprim_hl_4,cprim_hl_5,cprim_hl_6,cprim_hl_7,cprim_hl_8};

  BVHNode *selected = selectionHistory.top();
  Octree *OCTselected = OCTselectionHistory.top();

  if (OCTselected == NULL || OCTselected == nullptr) {
    cout << "OCTselected is null!" << endl;
    return;
  }

  // render solid geometry (with depth offset)
  glPolygonOffset(1.0, 1.0);
  glEnable(GL_POLYGON_OFFSET_FILL);

  //drawing all the points of the octree
  if (OCTselected->isLeaf) {
    octopus->draw(OCTselected, cprim_hl_1, colors);
  } else {
    for (int i = 0; i<8; i++){
      if (OCTselected->children[i] != NULL || OCTselected->children[i] != nullptr) {
        octopus->draw(OCTselected->children[i], colors[i], colors);
      }
    }
  }

  glDisable(GL_POLYGON_OFFSET_FILL);

  // draw geometry outline
  bvh->drawOutline(selected, edges);

  //draw octree bounding boxes
  octopus->drawOutline(OCTselected, white);

  // keep depth buffer check enabled so that mesh occluded bboxes, but
  // disable depth write so that bboxes don't occlude each other.
  glDepthMask(GL_FALSE);

  // create traversal stack
  std::stack<Octree *> otstack;

  // push initial traversal data
  otstack.push(octopus);

  // draw all oct bboxes with non-highlighted color
  while (!otstack.empty()) {

    Octree *current = otstack.top();
    otstack.pop();

    current->bounds.draw(dark);
    for (int i = 0; i<8; i++){
      if (current->children[i]){
       otstack.push(current->children[i]);
      }
    }

  }

  // draw selected node bbox and primitives

   for (int i = 0; i<8; i++){
      if (OCTselected->children[i]){
        OCTselected->children[i]->bounds.draw(white);
      }
    }

  glLineWidth(3.f);
  OCTselected->bounds.draw(orange);

  glDepthMask(GL_TRUE);
  glPopAttrib();

}

void PathTracer::visualize_accel() const {

  glPushAttrib(GL_ENABLE_BIT);
  glDisable(GL_LIGHTING);
  glLineWidth(1);
  glEnable(GL_DEPTH_TEST);

  // hardcoded color settings
  Color cnode = Color(.5, .5, .5, .25);
  Color cnode_hl = Color(1., .25, .0, .6);
  Color cnode_hl_child = Color(1., 1., 1., .6);

  Color cprim_hl_left = Color(.6, .6, 1., 1);
  Color cprim_hl_right = Color(.8, .8, 1., 1);
  Color cprim_hl_edges = Color(0., 0., 0., 0.5);

  BVHNode *selected = selectionHistory.top();

  // render solid geometry (with depth offset)
  glPolygonOffset(1.0, 1.0);
  glEnable(GL_POLYGON_OFFSET_FILL);

  if (selected->isLeaf()) {
    bvh->draw(selected, cprim_hl_left);
  } else {
    bvh->draw(selected->l, cprim_hl_left);
    bvh->draw(selected->r, cprim_hl_right);
  }

  glDisable(GL_POLYGON_OFFSET_FILL);

  // draw geometry outline
  bvh->drawOutline(selected, cprim_hl_edges);

  // keep depth buffer check enabled so that mesh occluded bboxes, but
  // disable depth write so that bboxes don't occlude each other.
  glDepthMask(GL_FALSE);

  // create traversal stack
  stack<BVHNode *> tstack;

  // push initial traversal data
  tstack.push(bvh->get_root());

  // draw all BVH bboxes with non-highlighted color
  while (!tstack.empty()) {

    BVHNode *current = tstack.top();
    tstack.pop();

    current->bb.draw(cnode);
    if (current->l) tstack.push(current->l);
    if (current->r) tstack.push(current->r);
  }

  // draw selected node bbox and primitives
  if (selected->l) selected->l->bb.draw(cnode_hl_child);
  if (selected->r) selected->r->bb.draw(cnode_hl_child);

  glLineWidth(3.f);
  selected->bb.draw(cnode_hl);

  // now perform visualization of the rays
  if (show_rays) {
      glLineWidth(1.f);
      glBegin(GL_LINES);

      for (size_t i=0; i<rayLog.size(); i+=500) {

          const static double VERY_LONG = 10e4;
          double ray_t = VERY_LONG;

          // color rays that are hits yellow
          // and rays this miss all geometry red
          if (rayLog[i].hit_t >= 0.0) {
              ray_t = rayLog[i].hit_t;
              glColor4f(1.f, 1.f, 0.f, 0.1f);
          } else {
              glColor4f(1.f, 0.f, 0.f, 0.1f);
          }

          Vector3D end = rayLog[i].o + ray_t * rayLog[i].d;

          glVertex3f(rayLog[i].o[0], rayLog[i].o[1], rayLog[i].o[2]);
          glVertex3f(end[0], end[1], end[2]);
      }
      glEnd();
  }

  glDepthMask(GL_TRUE);
  glPopAttrib();
}

void PathTracer::visualize_cell() const {
  glPushAttrib(GL_VIEWPORT_BIT);
  glViewport(0, 0, sampleBuffer.w, sampleBuffer.h);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0, sampleBuffer.w, sampleBuffer.h, 0, 0, 1);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glTranslatef(0, 0, -1);

  glColor4f(1.0, 0.0, 0.0, 0.8);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);

  // Draw the Red Rectangle.
  glBegin(GL_LINE_LOOP);
  glVertex2f(cell_tl.x, sampleBuffer.h-cell_br.y);
  glVertex2f(cell_br.x, sampleBuffer.h-cell_br.y);
  glVertex2f(cell_br.x, sampleBuffer.h-cell_tl.y);
  glVertex2f(cell_tl.x, sampleBuffer.h-cell_tl.y);
  glEnd();

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();

  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  glPopAttrib();

  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);
}

void PathTracer::key_press(int key) {
static double adder = 1.;
  BVHNode *current = selectionHistory.top();
  Octree *curr_oct = OCTselectionHistory.top();

  if (state == VISUALIZE_OCT){
    switch (key) {
    case KEYBOARD_UP:
        if (OCTselectionHistory.size()>1){
          OCTselectionHistory.pop();
        }
        break;
    case '1':
        if (curr_oct->children[0]) {
            OCTselectionHistory.push(curr_oct->children[0]);
        }
        break;
    case '2':
        if (curr_oct->children[1]) {
            OCTselectionHistory.push(curr_oct->children[1]);
        }
        break;
    case '3':
        if (curr_oct->children[2]) {
            OCTselectionHistory.push(curr_oct->children[2]);
        }
        break;
    case '4':
        if (curr_oct->children[3]) {
            OCTselectionHistory.push(curr_oct->children[3]);
        }
        break;
    case '5':
        if (curr_oct->children[4]) {
            OCTselectionHistory.push(curr_oct->children[4]);
        }
        break;
   case '6':
        if (curr_oct->children[5]) {
            OCTselectionHistory.push(curr_oct->children[5]);
        }
        break;
    case '7':
        if (curr_oct->children[6]) {
            OCTselectionHistory.push(curr_oct->children[6]);
        }
        break;
    case '8':
        if (curr_oct->children[7]) {
            OCTselectionHistory.push(curr_oct->children[7]);
        }
        break;
    default:
        return;
  }
}  else if (key >= '0' && key <= '4') {
    camera->mount_lens(key - '1');
    return;
  }
  switch (key) {
  case ']':
      ns_aa *=2;
      fprintf(stdout, "[PathTracer] Samples per pixel changed to %lu\n", ns_aa);
      //tm_key = clamp(tm_key + 0.02f, 0.0f, 1.0f);
      break;
  case '[':
      //tm_key = clamp(tm_key - 0.02f, 0.0f, 1.0f);
      ns_aa /=2;
      if (ns_aa < 1) ns_aa = 1;
      fprintf(stdout, "[PathTracer] Samples per pixel changed to %lu\n", ns_aa);
      break;
  case '=': case '+':
      ns_area_light *= 2;
      fprintf(stdout, "[PathTracer] Area light sample count increased to %zu.\n", ns_area_light);
      break;
  case '-': case '_':
      if (ns_area_light > 1) ns_area_light /= 2;
      fprintf(stdout, "[PathTracer] Area light sample count decreased to %zu.\n", ns_area_light);
      break;
  case '.': case '>':
      max_ray_depth++;
      fprintf(stdout, "[PathTracer] Max ray depth increased to %zu.\n", max_ray_depth);
      break;
  case ',': case '<':
      if (max_ray_depth) max_ray_depth--;
      fprintf(stdout, "[PathTracer] Max ray depth decreased to %zu.\n", max_ray_depth);
      break;

  case ';':
    camera->move_sensor(adder);

  break;
  case '\'':
    camera->move_sensor(-adder);


  break;
  case 'Q':
    adder *= 10;
    cout << "[PathTracer] Sensor depth increment is now " << adder << endl;
  break;
  case 'W':
    adder /= 10;
    cout << "[PathTracer] Sensor depth increment is now " << adder << endl;
  break;

  case 'Z':
    camera->stop_down(sqrt(2.));
  break;
  case 'X':
    camera->stop_down(1./sqrt(2.));
  break;

  case 'C':
    render_cell = !render_cell;
    if (render_cell)
      fprintf(stdout, "[PathTracer] Now in cell render mode.\n");
    else
      fprintf(stdout, "[PathTracer] No longer in cell render mode.\n");
  break;
  case 'F':
    if (render_cell)
      camera->autofocus();
  break;
  case 'A':
    if (render_cell) {
      ImageBuffer ib;
      raytrace_cell(ib,true);
      cout << "[LensCamera] Focus metric on windowed view is " << camera->focus_metric(ib) << endl;
    }
  break;

  case KEYBOARD_UP:
      if (current != bvh->get_root()) {
          selectionHistory.pop();
      }
      break;
  case KEYBOARD_LEFT:
      if (current->l) {
          selectionHistory.push(current->l);
      }
      break;
  case KEYBOARD_RIGHT:
      if (current->l) {
          selectionHistory.push(current->r);
      }
      break;
  default:
      return;
  }
}

Sphere PathTracer::get_bounding_sphere() {
     // collect primitives
    vector<Primitive *> primitives;
    bool t = scene == NULL;
    if (scene->objects.empty()) {
    } else {
      for (SceneObject *obj : scene->objects) {
        const vector<Primitive *> &obj_prims = obj->get_primitives();
        if (obj_prims.empty()) {
        } else {
          primitives.reserve(primitives.size() + obj_prims.size());
          primitives.insert(primitives.end(), obj_prims.begin(), obj_prims.end());
        }
      }
    }
    //find bounding box
    BBox scene_bbox;
    for (Primitive *p : primitives) {
      BBox bb = p->get_bbox();
      scene_bbox.expand(bb);
    }
    // add in camera position
    scene_bbox.expand(camera->position());
    
    // find center point, radius, and create sphere
    Vector3D half_dist = (scene_bbox.extent / 2.0);
    Vector3D sphere_center = scene_bbox.min + half_dist;
    double sphere_radius = max(max(half_dist.x, half_dist.y), half_dist.z) + 0.5; //buffer
    Sphere bounding_sphere = Sphere(NULL, sphere_center, sphere_radius);
    return bounding_sphere;
}



vector<Point*> PathTracer::sample_surface_points() {
    
    // Create bounding sphere for scene
    Sphere bounding_sphere = get_bounding_sphere();
    
    const double MIN_DIST_BETWEEN_PTS = 0.01; //
    const int MAX_REPEATED_FAILS = 2000; // probably don't need to adjust
    const int STOPPING_CRITERION = 10000; // Higher = brighter

    int totalPathsTraced = 0,
        numPointsAdded = 0,
        repeatedFails = 0,
        badCount = 0;
    const double area = MIN_DIST_BETWEEN_PTS * MIN_DIST_BETWEEN_PTS * PI;

    vector<Point*> candidates;
    vector<Point*> final_points;
    UniformSphereSampler3D sphere_sampler = UniformSphereSampler3D();
    while (true) {
      int pathsTraced;
      for (pathsTraced = 0; pathsTraced < MAX_REPEATED_FAILS; pathsTraced++) {
        // follow ray path and attempt to deposit candidate sample point
        Vector3D origin = camera->position();
        Vector3D dir = sphere_sampler.get_sample();
        dir.normalize();
        Ray r = Ray(origin, dir);
        while (r.depth < 30) {
        // find ray intersection with scene geometry or bounding sphere
          Intersection isect;
          bool hit_on_sphere = false;
          if (!bvh->intersect(r, &isect)) {
            if (!bounding_sphere.intersect(r, &isect)) {
              badCount++;
              cout << "Didn't intersect the scene anywhere." <<endl;
              break;
            }
            hit_on_sphere = true;
          }

          // make hit normal face on same side as r
          Vector3D hit_normal = isect.n;
          if (dot(hit_normal, -1*dir) < 0) {
            hit_normal = hit_normal * -1;
          }

          Vector3D hit_point = origin + (r.d * isect.t);
          // store candidate point at ray intersection
          // TODO - CHECK BRSSDF TO SEE IF TRANSLUCENT
          if (!hit_on_sphere && r.depth >= 3 && isect.bssrdf != NULL) {
            Point *p = new Point();
            p->t = isect.t;
            p->area = area;
            p->pos = hit_point;
            p->n = hit_normal;
            candidates.push_back(p);
          }

          // generate random ray from intersection point
          dir = sphere_sampler.get_sample();
          dir.normalize();
          origin = hit_point;
          // make new direction face same side as hit normal
          if (dot(hit_normal, dir) < 0) {
            dir = dir * -1;
          }
          int next_depth = (int) r.depth + 1;
          r = Ray(origin + (dir * 0.0001), dir);
          r.depth = (size_t) next_depth;
        }

      }

      if (repeatedFails > MAX_REPEATED_FAILS) {
        cout << "Exceeded max repeated rejections. Total points = " << numPointsAdded << endl;
        return final_points;
      }
      totalPathsTraced += pathsTraced;
      // for each candidate in list, Poisson check against all in final points to see if added
      for (uint32_t i = 0; i < candidates.size(); i++) {
        PoissonCheck check = PoissonCheck((float) MIN_DIST_BETWEEN_PTS, *candidates[i]);
        for (uint32_t j = 0; j < final_points.size(); j++) {
          if (!check(*final_points[j])) {
            repeatedFails++;
            if (repeatedFails > MAX_REPEATED_FAILS) {
              cout << "Exceeded max repeated rejections. Total points = " << numPointsAdded << endl;
              return final_points;
            }
            break;
          }
        }
        if (!check.failed) {
          numPointsAdded++;
          repeatedFails = 0;
          final_points.push_back(candidates[i]);
          if (numPointsAdded % 1000 == 0)
            printf("Added %d points so far.\n", numPointsAdded);
          if (numPointsAdded >= STOPPING_CRITERION) {
            printf("Stopping because total points is %d\n", numPointsAdded);
            return final_points;
          }
        }
      }

      if (totalPathsTraced > 50000 && numPointsAdded == 0) {
        printf("Warning: Does not appear as though objects in scene to intersect,"
                       "no points added Total points = %d", numPointsAdded);
        return final_points;
      }

      if (final_points.size() > STOPPING_CRITERION) {
        printf("""Stopping because total points has exceeded %d. Total points = %d ",
               STOPPING_CRITERION, numPointsAdded);
        return final_points;
      }

      candidates.erase(candidates.begin(), candidates.end());

    }

    cout << "At end, Total points = " << numPointsAdded << endl;
    return final_points;
}


void PathTracer::fill_in_point_irradiances(vector<Point*> &pts) {
  Matrix3x3 o2w;

  float distToLight, pdf;
  Vector3D wi;

  for (uint32_t k = 0; k < pts.size(); ++k) {
    Point *sp = pts[k];
    Spectrum L_out = Spectrum();
    make_coord_space(o2w, sp->n);
    Matrix3x3 w2o = o2w.T();
    for (int i = 0; i < scene->lights.size(); i++) {
      SceneLight* sl = scene->lights[i];
      // step 2
      int num_samples = sl->is_delta_light() ? 1 : (int) ns_area_light;
      Spectrum L_temp = Spectrum();
      for (int j = 0; j < num_samples; j++) {
        // step 3
        Spectrum radiance = sl->sample_L(sp->pos, &wi, &distToLight, &pdf);
        // step 4
        Vector3D w_in = w2o * wi;
        // step 5
        if (w_in.z < 0)
          continue;
        // step 6
        Ray new_r = Ray(sp->pos + EPS_D * wi, wi);
        new_r.max_t = (double) distToLight;
        bool hasIntersectionWithScene = bvh->intersect(new_r);
        // // step 7
        if (hasIntersectionWithScene)
          continue;
        L_temp += (radiance * w_in.z) / ((double) pdf); // factors 1 & 3
      }
      L_out += L_temp / (double) num_samples; // factor 2
    }
    sp->irradiance = L_out;
  }
}

void PathTracer::create_octree() {
  // sample the points and fill in irradiance values
  vector<Point*> pts = sample_surface_points();
  fill_in_point_irradiances(pts);
  // create bounds from points
  BBox octreeBounds;
  for (Point *p : pts) {
    octreeBounds.expand(p->pos);
  }

  // create octree with these bounds
  octopus = new Octree(octreeBounds);
  // add in points
  int i = 0;
  for (int k = 0; k < pts.size(); ++k) {
      octopus->insert(octreeBounds, pts[k]);
  }
  // initiate hierarchy
  octopus->init_heirarchy();
  OCTselectionHistory.push(octopus);

  int num_empty = investigate_octree(octopus);
  cout << "Number of empty children in tree is: " << num_empty << endl;
  if (num_empty > 0) {
    prune_octree(octopus);
    num_empty = investigate_octree(octopus);
    cout << "Number of empty children after prune is: " << num_empty << endl;
  }
}

int PathTracer::investigate_octree(Octree* oct) {
  if (oct->isLeaf) {
    int num_null_child = 0;
    for (int i = 0; i < 8; i++) {
        if (!oct->ips[i]) num_null_child++;
    }
    if (num_null_child == 8) {
      return 1;
    }
    return 0;
  } else {
    int num_empty_child = 0;
    for (int child = 0; child < 8; child++) {
        if (!oct->children[child]) continue;
        num_empty_child += investigate_octree(oct->children[child]);
    }
    return num_empty_child;
  }
}

bool PathTracer::prune_octree(Octree* oct) {
  if (oct->isLeaf) {
    int num_null_child = 0;
    for (int i = 0; i < 8; i++) {
        if (!oct->ips[i]) num_null_child++;
    }
    if (num_null_child == 8) {
      return true;
    }
    return false;
  } else {
    int num_empty_child = 0;
    for (int child = 0; child < 8; child++) {
        if (!oct->children[child]) {
          num_empty_child++;
        } else if (prune_octree(oct->children[child])) {
          cout << "deleting empty child" << endl;
          delete oct->children[child];
          oct->children[child] = NULL;
          num_empty_child++;
        }
    }
    if (num_empty_child == 8) return true;
    return false;
  }
}


Spectrum PathTracer::estimate_direct_lighting(const Ray& r, const Intersection& isect) {

  // TODO Part 3

  // SINGLE SCATTERING VARIABLES
  const float SUBSURFACE_DEPTH_REDUCTION = 0.0001; // originally 0.0001. Higher results in deeper subsurface rays.
  const float MAX_GLOW = 5; // Originally 5.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D& hit_p = r.o + r.d * isect.t;
  const Vector3D& w_out = w2o * (-r.d);
  Spectrum L_out;
  float distToLight, pdf;
  Vector3D wi, w_in;
  Spectrum s;
  SceneLight* sl;
  int num_samples;
  bool hasIntersectionWithScene;
  /////////////////////////////
  // Note: The location of this code will have to be changed if single scatterincg is implemented.
  // A PDF will have to be incorporated.
  if (isect.bssrdf != NULL) {
    Spectrum diffuse_contribution = isect.bssrdf->get_L_out(hit_p, w_out, octopus); // do we need to convert hit_p to object coords?
    // return diffuse_contribution;
    Intersection isect_inner;
    Spectrum L_temp = Spectrum();
    // Calculate single scattering term
    int times_light_added = (int) scene->lights.size();
    for (int i = 0; i < scene->lights.size(); i++) {
      sl = scene->lights[i];

      // calculate so'
      float so_prime = isect.bssrdf->calc_so_prime();
      // calculate inner hit point
      const Vector3D& scatter_point = hit_p + r.d * so_prime * SUBSURFACE_DEPTH_REDUCTION; //
      // get wi by sample_L
      Spectrum radiance = sl->sample_L(scatter_point, &wi, &distToLight, &pdf);

      w_in = w2o * wi;
      // step 5
      if (w_in.z < 0) continue;

      // get hitpoint for si
      Ray to_surface = Ray(scatter_point, wi);
      bool intersected_surface = bvh->intersect(to_surface, &isect_inner);
      if (!intersected_surface) {
        times_light_added--;
        continue;
      }
      const Vector3D& surface_point = to_surface.o + to_surface.d * isect_inner.t;
      float si = (float) (surface_point - scatter_point).norm();
      float denom = sqrt(1 - (float) pow((1 / isect.bssrdf->rd.ior), 2)*(1 - (float) pow(abs(dot(wi, isect_inner.n)), 2)));
      float si_prime = si * (float) abs(dot(wi, isect_inner.n)) / (float) denom;

      Ray to_light = Ray(surface_point + (EPS_D * wi), wi);
      to_light.max_t = (double) distToLight - (si + EPS_D*5);
      hasIntersectionWithScene = bvh->intersect(to_light);
      // // step 7
      if (hasIntersectionWithScene) {
        times_light_added--;
        continue;
      }

      float G = (float) abs(dot(isect_inner.n, -r.d)) / (float) abs(dot(isect_inner.n, to_surface.d));
      float sigma_tc = isect.bssrdf->rd.sigma_t_prime + G*isect.bssrdf->rd.sigma_t_prime;
      float F = (1 - isect.bssrdf->schlick_approximation_of_fr(-r.d))*(1 - isect.bssrdf->schlick_approximation_of_fr(wi));
      float phase_fn_result = isect.bssrdf->phase_function(-r.d, wi);
      float coefficient = ((isect.bssrdf->rd.sigma_s*F*phase_fn_result) / sigma_tc) /
                                exp(-si_prime*isect.bssrdf->rd.sigma_t_prime)*exp(-so_prime*SUBSURFACE_DEPTH_REDUCTION
                                                                                  *isect.bssrdf->rd.sigma_t_prime);
      s = isect.bsdf->f(w_out, w_in);

      if (coefficient < 0) {
        coefficient = -1*coefficient;
      }
      if (coefficient > MAX_GLOW) {
        coefficient = MAX_GLOW;
      }
      // cout << coefficient << endl;
      L_temp = coefficient*((radiance *s* w_in.z) / ((double) pdf)); // factors 1 & 3
      L_out += L_temp;
    }
    if (times_light_added == 0) {
      return diffuse_contribution;
    }
    // cout << L_out << endl;
    // return L_out + diffuse_contribution;
    // make bigger clamp
    return (L_out / ((float) times_light_added)) + diffuse_contribution;
  }
  /////////////////////////////
  // step 1
  for (int i = 0; i < scene->lights.size(); i++) {
    sl = scene->lights[i];
    // step 2
    num_samples = sl->is_delta_light() ? 1 : (int) ns_area_light;
    Spectrum L_temp = Spectrum();
    for (int j = 0; j < num_samples; j++) {
      // step 3
      Spectrum radiance = sl->sample_L(hit_p, &wi, &distToLight, &pdf);
      // step 4
      w_in = w2o * wi;
      // step 5
      if (w_in.z < 0)
        continue;
      // step 6
      Ray new_r = Ray(hit_p + EPS_D * wi, wi);
      new_r.max_t = (double) distToLight;
      hasIntersectionWithScene = bvh->intersect(new_r);
      // // step 7
      if (hasIntersectionWithScene)
        continue;
      s = isect.bsdf->f(w_out, w_in);
      L_temp += (radiance * s * w_in.z) / ((double) pdf); // factors 1 & 3
    }
    L_out += L_temp / (double) num_samples; // factor 2
  }
  return L_out;
}

Spectrum PathTracer::estimate_indirect_lighting(const Ray& r, const Intersection& isect) {

//   TODO Part 4

  double NOISE_REDUCTION_VAL = 20.; // somewhere between 1 and 10. Higher = slower but smoother.

  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();
  Spectrum s;

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  float pdf;
  Vector3D w_in;
  // my code starting here.
  // step 1
  s = isect.bsdf->sample_f(w_out, &w_in, &pdf); // we discussed this when we met on 4/24 - do not use BSSRDF
  // step 2
  double illum_val = NOISE_REDUCTION_VAL * (double) s.illum();
  illum_val = clamp(illum_val, 0, 1);
  double terminationProbability = 1 - illum_val;
  // low reflectance should mean high termination probability
  bool terminate = coin_flip(terminationProbability);
  if (terminate) {
    return Spectrum();
  }
  // step 3
  Vector3D d = o2w * w_in;
  Ray r2(hit_p + EPS_D*d, d);
  r2.depth = r.depth - 1;
  // set camera ray depth to max_ray_depth in raytrace_pixel
  // step 4
  return trace_ray(r2, isect.bsdf->is_delta()) * (s * abs_cos_theta(w_in) / (pdf * (1. - terminationProbability)));

}

Spectrum PathTracer::trace_ray(const Ray &r, bool includeLe) {

  Intersection isect;
  Spectrum L_out;

  // You will extend this in part 2. 
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.
  if (!bvh->intersect(r, &isect)) 
    return L_out;

  // We only include the emitted light if the previous BSDF was a delta distribution
  // or if the previous ray came from the camera.
  if (includeLe)
    L_out += isect.bsdf->get_emission();

  // You will implement this in part 3. 
  // Delta BSDFs have no direct lighting since they are zero with probability 1 --
  // their values get accumulated through indirect lighting, where the BSDF 
  // gets to sample itself.
 // if (!isect.bsdf->is_delta()) {
    L_out += estimate_direct_lighting(r, isect);
 // }

  // You will implement this in part 4.
  // If the ray's depth is zero, then the path must terminate
  // and no further indirect lighting is calculated.
  if (r.depth > 0)
    L_out += estimate_indirect_lighting(r, isect);

  return L_out;

}

Spectrum PathTracer::raytrace_pixel(size_t x, size_t y) {

  // Part 1, Task 1:
  // Make a loop that generates num_samples camera rays and traces them 
  // through the scene. Return the average Spectrum. 

  int num_samples = ns_aa; // total samples to evaluate
  Vector2D origin = Vector2D(x,y); // bottom left corner of the pixel

  Spectrum s;
  int total_tries = 0;
  for (int i = 0; i < num_samples; ++i) {
    Vector2D p = origin + (ns_aa>1?gridSampler->get_sample():Vector2D(.5,.5));
    int num_tries = 0;
    double cos_factor = 1.;
    Ray r = camera->generate_ray(p.x/sampleBuffer.w, p.y/sampleBuffer.h, num_tries, cos_factor);
    r.depth = max_ray_depth;
    total_tries += num_tries;
    if (cos_factor > 0.) {
      s += trace_ray(r,true)* (float) cos_factor;
    }
  }
  return s * (1./(double) total_tries);

}

void PathTracer::raytrace_tile(int tile_x, int tile_y,
                               int tile_w, int tile_h) {

  size_t w = sampleBuffer.w;
  size_t h = sampleBuffer.h;

  size_t tile_start_x = tile_x;
  size_t tile_start_y = tile_y;

  size_t tile_end_x = std::min(tile_start_x + tile_w, w);
  size_t tile_end_y = std::min(tile_start_y + tile_h, h);

  size_t tile_idx_x = tile_x / imageTileSize;
  size_t tile_idx_y = tile_y / imageTileSize;
  size_t num_samples_tile = tile_samples[tile_idx_x + tile_idx_y * num_tiles_w];

  for (size_t y = tile_start_y; y < tile_end_y; y++) {
    if (!continueRaytracing) return;
    for (size_t x = tile_start_x; x < tile_end_x; x++) {
        Spectrum s = raytrace_pixel(x, y);
        sampleBuffer.update_pixel(s, x, y);
    }
  }

  tile_samples[tile_idx_x + tile_idx_y * num_tiles_w] += 1;
  sampleBuffer.toColor(frameBuffer, tile_start_x, tile_start_y, tile_end_x, tile_end_y);
}

void PathTracer::raytrace_cell(ImageBuffer& buffer, bool to_record) {


  size_t tile_start_x = cell_tl.x;
  size_t tile_start_y = cell_tl.y;

  size_t tile_end_x = cell_br.x;
  size_t tile_end_y = cell_br.y;

  size_t w = tile_end_x - tile_start_x;
  size_t h = tile_end_y - tile_start_y;
  HDRImageBuffer sb(w, h);
  buffer.resize(w,h);


  stop();
  render_cell = true;
  render_silent = true;
  {
    unique_lock<std::mutex> lk(m_done);
    start_raytracing();
    cv_done.wait(lk, [this]{ return state == DONE; });
    lk.unlock();
  }

  for (size_t y = tile_start_y; y < tile_end_y; y++) {
    for (size_t x = tile_start_x; x < tile_end_x; x++) {
        buffer.data[w*(y-tile_start_y)+(x-tile_start_x)] = frameBuffer.data[x+y*sampleBuffer.w];
    }
  }

  return;


  for (size_t y = tile_start_y; y < tile_end_y; y++) {
    for (size_t x = tile_start_x; x < tile_end_x; x++) {
        Spectrum s = raytrace_pixel(x, y);
        sb.update_pixel(s, x - tile_start_x, y - tile_start_y);
    }
  }

  sb.toColor(buffer, 0, 0, w, h);
  if (to_record) {
    for (size_t y = tile_start_y; y < tile_end_y; y++) {
      for (size_t x = tile_start_x; x < tile_end_x; x++) {
          frameBuffer.data[x+y*sampleBuffer.w] = buffer.data[w*(y-tile_start_y)+(x-tile_start_x)];
      }
    }
  }

}

void PathTracer::worker_thread() {

  Timer timer;
  timer.start();

  WorkItem work;
  while (continueRaytracing && workQueue.try_get_work(&work)) {
    raytrace_tile(work.tile_x, work.tile_y, work.tile_w, work.tile_h);
    { 
      lock_guard<std::mutex> lk(m_done);
      ++tilesDone;
      if (!render_silent)  cout << "\r[PathTracer] Rendering... " << int((double)tilesDone/tilesTotal * 100) << '%';
      cout.flush();
    }
  }

  workerDoneCount++;
  if (!continueRaytracing && workerDoneCount == numWorkerThreads) {
    timer.stop();
    if (!render_silent)  fprintf(stdout, "\n[PathTracer] Rendering canceled!\n");
    state = READY;
  }

  if (continueRaytracing && workerDoneCount == numWorkerThreads) {
    timer.stop();
    if (!render_silent)  fprintf(stdout, "\r[PathTracer] Rendering... 100%%! (%.4fs)\n", timer.duration());
    if (!render_silent)  fprintf(stdout, "[PathTracer] BVH traced %llu rays.\n", bvh->total_rays);
    if (!render_silent)  fprintf(stdout, "[PathTracer] Averaged %f intersection tests per ray.\n", (((double)bvh->total_isects)/bvh->total_rays));

    lock_guard<std::mutex> lk(m_done);
    state = DONE;
    cv_done.notify_one();
  }
}

void PathTracer::save_image(string filename) {

  if (state != DONE) return;

  if (filename == "") {
    time_t rawtime;
    time (&rawtime);

    time_t t = time(nullptr);
    tm *lt = localtime(&t);
    stringstream ss;
    ss << this->filename << "_screenshot_" << lt->tm_mon+1 << "-" << lt->tm_mday << "_" 
      << lt->tm_hour << "-" << lt->tm_min << "-" << lt->tm_sec << ".png";
    filename = ss.str();  
  }


  uint32_t* frame = &frameBuffer.data[0];
  size_t w = frameBuffer.w;
  size_t h = frameBuffer.h;
  uint32_t* frame_out = new uint32_t[w * h];
  for(size_t i = 0; i < h; ++i) {
    memcpy(frame_out + i * w, frame + (h - i - 1) * w, 4 * w);
  }

  fprintf(stderr, "[PathTracer] Saving to file: %s... ", filename.c_str());
  lodepng::encode(filename, (unsigned char*) frame_out, w, h);
  fprintf(stderr, "Done!\n");
}

}  // namespace CGL
