#include "pathtracer/camera.h"
#include "pathtracer/intersection.h"
#include "scene/light.h"
#include "pathtracer.h"
#include "pathtracer/sampler.h"

using namespace CGL::SceneObjects;
namespace CGL
{

struct EndpointInteraction : Intersection {
    union {
        const Camera *camera;
        const SceneLight *light;
    };
    EndpointInteraction() : Intersection(), light(nullptr) {}
    EndpointInteraction(const Intersection &it, const Camera *camera)
        : Intersection(it), camera(camera) {}

};

enum class BDPT_VertexType { Camera, Light, Surface};
struct BDPT_Vertex {
  Vector3D point;
  Vector3D wi;
  Vector3D wo;
  Vector3D throughput;
  Intersection itsec;
  int depth;
  double vc=0.0, vcm=0.0, rr = 0.0, pdf = 1.0;

  BDPT_Vertex () {};

  BDPT_Vertex (Vector3D point, Vector3D wi, Vector3D wo, Intersection itsec, int depth)
    : point(point), wi(wi), wo(wo), itsec(itsec), depth(depth) {};

  BDPT_Vertex(Vector3D point, Vector3D wi, Vector3D wo, Intersection itsec)
    : point(point), wi(wi), wo(wo), itsec(itsec), depth(0) {};

};

class BDPT : public PathTracer {
  public:

  /* MIS implementation */
  Vector3D Li( const Ray& ray);

  /* MIS sub path evaluation */
  Vector3D ConnectVertices (BDPT_Vertex& p0, BDPT_Vertex& p1);

  /* MIS direct light sampling */
  Vector3D ConnectLight (BDPT_Vertex& BDPT_Vertex, SceneLight* light);

  /* Naive implementation */
  Vector3D Shade( const Ray& ray);

  /* Calculate Path value */
  Vector3D evalPath(vector<BDPT_Vertex> &camPath, vector<BDPT_Vertex> &lightPath, int nCam, int nLight);

  void raytrace_pixel(size_t x, size_t y);
  Vector3D est_radiance_global_illumination(const Ray& ray);


  private:

  bool useMIS = true;

  double inline MIS(double t){
    return useMIS ? t*t : 1.0f;
  }
    




};


}