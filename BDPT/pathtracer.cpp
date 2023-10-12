#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading
  Vector3D wi, wi_w;
  double pdf_inv = 2.0*PI;

  for (int i = 0; i < num_samples; ++i) {
    wi = hemisphereSampler->get_sample();
    wi_w = o2w*wi;

    Ray ray = Ray(hit_p + EPS_D*wi_w, wi_w);
    Intersection its;

    if (bvh->intersect(ray, &its)) {
      L_out += its.bsdf->get_emission()*isect.bsdf->f(wi, w_out)*wi.z;
    }
  }

  return L_out*pdf_inv/num_samples;

}

Vector3D PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out;

  Vector3D wi, wi_o;
  double distToLight, pdf;

  for (SceneLight* L : scene->lights) {
    if (L->is_delta_light()) {
      Vector3D sample = L->sample_L(hit_p,&wi,&distToLight,&pdf);
      wi_o = w2o*wi;
      if (wi_o.z <= 0) continue;
      Ray ray = Ray(hit_p + (EPS_D * wi), wi, distToLight-EPS_D);
      if (!bvh->has_intersection(ray)) {
        L_out += (sample*isect.bsdf->f(wi_o,w_out)*wi_o.z)/pdf;
      }
    }
    else {
      Vector3D spec_sum;
      for (int i = 0; i < ns_area_light; ++i) {
        Vector3D sample = L->sample_L(hit_p,&wi,&distToLight,&pdf);
        wi_o = w2o*wi;
        if (wi_o.z <= 0) continue;
        Ray ray = Ray(hit_p + EPS_D*wi, wi, distToLight-EPS_D);
        if (!bvh->has_intersection(ray)) {
          spec_sum += (sample*isect.bsdf->f(wi_o,w_out)*wi_o.z)/pdf;
        }
      }
      L_out += spec_sum/ns_area_light;
    }
  }


  return L_out;

}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light

  return isect.bsdf->get_emission();

}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`
  if (direct_hemisphere_sample)
    return estimate_direct_lighting_hemisphere(r,isect);
  else
    return estimate_direct_lighting_importance(r,isect);

}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out(0, 0, 0);

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.
  if (!isect.bsdf->is_delta()) L_out += one_bounce_radiance(r, isect);

  if (r.depth+1 >= max_ray_depth) return L_out;
  

  double russianP = 0.7;
  if (coin_flip(russianP)) {
    Intersection its;
    Vector3D wi_o, wi_w;
    double pdf;

    Vector3D sample = isect.bsdf->sample_f(w_out, &wi_o, &pdf);
    wi_w = o2w * wi_o;
    Ray ray = Ray(hit_p + (EPS_D * wi_w), wi_w, INF_D, r.depth+1);
    if(bvh->intersect(ray,&its)) {
      Vector3D L = at_least_one_bounce_radiance(ray, its);
      if (isect.bsdf->is_delta())
        L += zero_bounce_radiance(ray, its);
      L_out += (L*sample*fabs(wi_o.z))/(pdf*russianP);
    }

  }
  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;
  
  if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;

  //L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);
  //return L_out;

  // TODO (Part 3): Return the direct illumination.
  //return zero_bounce_radiance(r,isect) + one_bounce_radiance(r,isect);

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct
  if (max_ray_depth == 0) return zero_bounce_radiance(r, isect);
  return zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect);


}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"

  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel

  //Jittered Sampling
  int nx = sqrt(ns_aa);
  int num_samples = nx*nx;
  double one = 1.0-EPS_D;
  vector<Vector2D> samps;
  double dx = 1.0/nx;
  for (int yt = 0; yt < nx; ++yt) {
    for (int xt = 0; xt < nx; ++xt) {
      double jx = random_uniform();
      double jy = random_uniform();
      samps.emplace_back(x + std::min((xt+jx)*dx, one),y + std::min((yt+jy)*dx, one));
    }
  }
  shuffle(samps.begin(),samps.end(),default_random_engine(0));
  Vector2D randomSample;
  Vector3D sample;
  double w_inv = 1.0/sampleBuffer.w;
  double h_inv = 1.0/sampleBuffer.h;

  if (samplesPerBatch > 1) {
    double s1 = 0;
    double s2 = 0;
    double mean = 0;
    double var2 = 0;
    double illum;
    Vector3D spec;
    int i;
    for (i = 1; i <= num_samples; ++i) {
      //randomSample.x = (random_uniform() + x)/sampleBuffer.w;
      //randomSample.y = (random_uniform() + y)/sampleBuffer.h;
      spec = est_radiance_global_illumination(camera->generate_ray(samps[i-1].x*w_inv, samps[i-1].y*h_inv));
      sample += spec;

      illum = spec.illum();
      s1 += illum;
      s2 += illum*illum;

      if (i%samplesPerBatch == 0) {
        mean = s1/i;
        var2 = (s2 - (s1*s1)/i)/(i-1);
        if (1.96*sqrt(var2/i) <= maxTolerance*mean) break;
      }
    }
    sample /= i;
    num_samples = i;
  }
  else {
    for (int i = 0; i < num_samples; ++i){
      //randomSample.x = (random_uniform() + x)/sampleBuffer.w;
      //randomSample.y = (random_uniform() + y)/sampleBuffer.h;
      sample += est_radiance_global_illumination(camera->generate_ray(samps[i].x*w_inv, samps[i].y*h_inv));
    }
    sample /= num_samples;
  }
  sampleBuffer.update_pixel(sample, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;

}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
