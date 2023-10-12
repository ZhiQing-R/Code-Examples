#include "camera.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include "CGL/misc.h"
#include "CGL/vector2D.h"
#include "CGL/vector3D.h"

using std::cout;
using std::endl;
using std::max;
using std::min;
using std::ifstream;
using std::ofstream;

namespace CGL {

using Collada::CameraInfo;

Ray Camera::generate_ray_for_thin_lens(double x, double y, double rndR, double rndTheta) const {

  // TODO Assignment 7: Part 4
  // compute position and direction of ray from the input sensor sample coordinate.
  // Note: use rndR and rndTheta to uniformly sample a unit disk.

  double d2r = PI/180.0;
  double h_width = tan(0.5*hFov*d2r);
  double h_height = tan(0.5*vFov*d2r);
  
  Vector3D d(x*h_width*2.0-h_width,y*h_height*2.0-h_height,-1);
  d.normalize();

  Vector3D pLens = Vector3D(lensRadius*sqrt(rndR)*cos(rndTheta), lensRadius*sqrt(rndR)*sin(rndTheta), 0);
  Vector3D pfocus = focalDistance*d - pLens;
  pfocus.normalize();

  Ray r(c2w*pLens+pos,c2w*pfocus);
  r.min_t = nClip;
  r.max_t = fClip;
  
  return r;
}


} // namespace CGL
