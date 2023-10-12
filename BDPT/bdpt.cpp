#include "pathtracer/bdpt.h"

using namespace CGL::SceneObjects;
using namespace std;

namespace CGL {


/* 
   MIS implementation
   Still have errors
*/
Vector3D BDPT::Li(const Ray& ray) {
    if (!bvh->has_intersection(ray)){
        return Vector3D();
    }
    //Sample single light source here
    double pdf = 1.0;
    SceneLight* light = scene->lights[0];

    //Gen initial light ray
    Ray light_ray;
    light_ray.d = hemisphereSampler->get_sample();
    double emission_pdf;
    double light_pdf;
    double coslight;
    Vector3D le = light->sample_O(light_ray,&emission_pdf,&light_pdf,&coslight);

    //Cal light path
    vector<BDPT_Vertex> light_path;
    double vc = light->is_delta_light() ? 0.0f: MIS(coslight / emission_pdf);
    double vcm = MIS(light_pdf / emission_pdf);
    Vector3D throughput = le * coslight / (emission_pdf * pdf);
    double rr = 1.0f;
    Ray wi(light_ray);

    while (light_path.size() < 0) {
        
        BDPT_Vertex vert;
        if (!bvh->intersect(wi, &vert.itsec)) {
            break;
        }

        double dist2 = vert.itsec.t*vert.itsec.t;
        double cosits = fabs(dot(wi.d, vert.itsec.n));

        vcm *= MIS(dist2);
        vcm /= MIS(cosits);
        vc /= MIS(cosits);

        rr = 1.0f;
        if (throughput.illum() < 0.01f)
            rr = 0.5f;

        vert.point = wi.at_time(vert.itsec.t);
        vert.wi = -wi.d;

        vert.throughput = throughput;
        vert.vcm = vcm;
        vert.vc = vc;
        vert.rr = rr;
        vert.depth = (light_path.size() + 1);

        light_path.push_back(vert);

        if (!coin_flip(rr))
            break;

        Matrix3x3 o2w;
        make_coord_space(o2w, vert.itsec.n);

        double bsdf_pdf;
        Vector3D bsdf_val = vert.itsec.bsdf->sample_f(vert.wi, &vert.wo, &bsdf_pdf);
        vert.wo = o2w*vert.wo;
        bsdf_pdf *= rr;

        if( 0.0f == bsdf_pdf )
            break;

        const auto cosOut = fabs(dot(vert.wo, vert.itsec.n));
        throughput = cosOut * throughput*bsdf_val / bsdf_pdf;

        if (throughput == Vector3D())
            break;

        double inv_bsdf_pdfw = vert.itsec.bsdf->pdf_inv( vert.wo , vert.wi ) * rr;
        vc = MIS(cosOut/bsdf_pdf) * ( MIS(inv_bsdf_pdfw) * vc + vcm ) ;
        vcm = MIS(1.0/bsdf_pdf);

        wi = Ray(vert.point + vert.wo*EPS_D, vert.wo);

    }

    //Init Cam path generation
    Vector3D li;
    wi = Ray(ray);
    int lps = light_path.size();
    double total_pixel = camera->screenW*camera->screenH;
    double cosCam = fabs(wi.o.z);
    double camPdfW = camera->screenDist * camera->screenDist / cosCam;
    throughput = 1.0;
    auto light_path_len = 0;
    vc = 0.0;
    vcm = total_pixel/camPdfW;
    rr = 1.0;

    //Gen cam path while connecting to light path
    while (light_path_len < max_ray_depth){

        BDPT_Vertex vert;
        vert.depth = light_path_len;

        if (!bvh->intersect(wi, &vert.itsec)) break;

        if (light_path_len == 0) li += vert.itsec.bsdf->get_emission();

        double dist2 = vert.itsec.t * vert.itsec.t;
        double cosits = fabs(dot(wi.d, vert.itsec.n));
        vcm *= MIS(dist2);
        vcm /= MIS(cosits);
        vc /= MIS(cosits);

        rr = 1.0f;
        if (throughput.illum() < 0.01 )
            rr = 0.5f;

        vert.point = wi.at_time(vert.itsec.t);
        vert.wi = -wi.d;

        vert.throughput = throughput;
        vert.vc = vc;
        vert.vcm = vcm;
        vert.rr = rr;

        //Direct light sampling
        li += ConnectLight(vert, light) / pdf;

        //Sub path sampling
        for (unsigned j = 0; j < lps; ++j)
            li += ConnectVertices(light_path[j], vert);

        ++light_path_len;

        if (!coin_flip(rr))
            break;

        Matrix3x3 o2w;
        make_coord_space(o2w, vert.itsec.n);

        double bsdf_pdf;
        Vector3D bsdf_val = vert.itsec.bsdf->sample_f(vert.wi, &vert.wo, &bsdf_pdf);
        vert.wo = o2w*vert.wo;

        if(bsdf_pdf == 0.0)
            break;

        bsdf_pdf *= rr;
        const auto cosOut = fabs(dot(vert.wo, vert.itsec.n));
        throughput = cosOut * throughput*bsdf_val/bsdf_pdf;

        if (throughput == Vector3D())
            break;

        double inv_bsdf_pdfw = vert.itsec.bsdf->pdf_inv( vert.wo , vert.wi ) * rr;
        vc = MIS(cosOut/bsdf_pdf) * ( MIS(inv_bsdf_pdfw) * vc + vcm ) ;
        vcm = MIS(1.0/bsdf_pdf);

        //Update new direction
        wi = Ray(vert.point + vert.wo*EPS_D, vert.wo);
    }

    return li;


}

Vector3D BDPT::ConnectVertices (BDPT_Vertex& p0, BDPT_Vertex& p1) {

    if(p0.depth + p1.depth >= max_ray_depth )
        return 0.0f;

    Vector3D btw = p0.point - p1.point;
    double dist = btw.norm();
    double invDist2 = 1.0 / (dist*dist);
    btw.normalize();

    Ray r = Ray(p1.point + btw*0.0001, btw, dist-0.00001);
    r.max_t = dist;
    if (bvh->has_intersection(r)) return Vector3D();

    double cosP0 = fabs(dot(p0.itsec.n, btw));
    double cosP1 = fabs(dot(p1.itsec.n, btw));
    Vector3D g = p1.itsec.bsdf->f(p1.wi, btw)*p0.itsec.bsdf->f(p0.wi, -btw) * invDist2;

    double p0_bsdf_pdfw = p0.itsec.bsdf->pdf_inv(p0.wi, -btw) * p0.rr;
    double p0_bsdf_inv_pdfw = p0.itsec.bsdf->pdf_inv(-btw, p0.wi) * p0.rr;
    double p1_bsdf_pdfw = p1.itsec.bsdf->pdf_inv(p1.wi, btw) * p1.rr;
    double p1_bsdf_inv_pdfw = p1.itsec.bsdf->pdf_inv(btw, p1.wi) * p1.rr;

    double p0_a = p1_bsdf_pdfw * cosP0 * invDist2;
    double p1_a = p0_bsdf_pdfw * cosP1 * invDist2;

    double mis_0 = MIS( p0_a ) * ( p0.vcm + p0.vc * MIS( p0_bsdf_inv_pdfw ) );
    double mis_1 = MIS( p1_a ) * ( p1.vcm + p1.vc * MIS( p1_bsdf_inv_pdfw ) );

    double weight = 1.0 / (mis_0 + 1.0+ mis_1);


    Vector3D li = p0.throughput * p1.throughput * g * weight;

    return li;
}

Vector3D BDPT::ConnectLight (BDPT_Vertex& BDPT_Vertex, SceneLight* light) {
    if( BDPT_Vertex.depth >= max_ray_depth)
        return Vector3D();

    double emission_pdf;
    double light_pdf;
    double distToLight;
    double coslight;
    Vector3D wi, wi_o;
    Vector3D li = light->sample_L2(BDPT_Vertex.point,&wi,&distToLight,&light_pdf, &emission_pdf, &coslight);
    Ray r = Ray(BDPT_Vertex.point + (EPS_D * wi), wi, distToLight-0.0001);
    if (bvh->has_intersection(r)){
        return Vector3D();
    } 

    Matrix3x3 o2w;
    make_coord_space(o2w, BDPT_Vertex.itsec.n);
    Matrix3x3 w2o = o2w.T();
    wi_o = w2o*wi;
    Vector3D w_out = w2o * (-BDPT_Vertex.wi);

    if(light_pdf < EPS_D)
        return 0.0;
    
    double cosBDPT_Vertex = fabs(dot(BDPT_Vertex.itsec.n, wi));

    li = li * BDPT_Vertex.throughput * BDPT_Vertex.itsec.bsdf->f(w_out,wi_o) * cosBDPT_Vertex / light_pdf;


    double eye_bsdf_pdfw = BDPT_Vertex.itsec.bsdf->pdf_inv(w_out,wi_o) * BDPT_Vertex.rr;
    double eye_bsdf_inv_pdfw = BDPT_Vertex.itsec.bsdf->pdf_inv(wi_o, w_out) * BDPT_Vertex.rr;

    double mis0 = light->is_delta_light() ? 0.0:MIS(eye_bsdf_pdfw / light_pdf);
    double mis1 = MIS(cosBDPT_Vertex*emission_pdf/(coslight*light_pdf)) * (BDPT_Vertex.vcm+BDPT_Vertex.vc*MIS(eye_bsdf_inv_pdfw));
    double weight = mis1 / (mis0 + mis1 + 1.0);
    //cout << (BDPT_Vertex.vcm) << endl;
    return li*weight;
}

void BDPT::raytrace_pixel(size_t x, size_t y) {
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel

  /*int num_samples = ns_aa;          // total samples to evaluate
  Vector2D randomSample;
  Vector3D sample;

  for (int i = 0; i < num_samples; ++i){
    randomSample.x = (random_uniform() + x)/sampleBuffer.w;
    randomSample.y = (random_uniform() + y)/sampleBuffer.h;
    sample += est_radiance_global_illumination(camera->generate_ray(randomSample.x,randomSample.y));
  }
  sample /= num_samples;

  sampleBuffer.update_pixel(sample, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;*/

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
      sample += est_radiance_global_illumination(camera->generate_ray(samps[i].x*w_inv, samps[i].y*h_inv));
    }
    sample /= num_samples;
  }
  sampleBuffer.update_pixel(sample, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
}


//Evaluate path value for sub path
Vector3D BDPT::evalPath(vector<BDPT_Vertex> &camPath, vector<BDPT_Vertex> &lightPath, int nCam, int nLight) {
    BDPT_Vertex cv = camPath[nCam - 1];
    BDPT_Vertex lv = lightPath[nLight - 1];
    Vector3D L(1.0);

    //Get throughput from prior vertex
    if (nCam > 1)
      L = L * camPath[nCam - 2].throughput;
    if (nLight > 1)
      L = L * lightPath[nLight - 2].throughput;

    //Calculate G term
    Vector3D btw = lv.point - cv.point;
    double dist2 = btw.norm2();
    btw /= sqrt(dist2);

    if (dist2 < 0.01)
      return Vector3D();

    double g = fabs(dot(btw, cv.itsec.n)) * fabs(dot(-btw, cv.itsec.n)) / dist2;

    //Calculate bsdfs
    Matrix3x3 o2w1;
    make_coord_space(o2w1, cv.itsec.n);
    Matrix3x3 w2o1(o2w1.T());

    L = L * cv.throughput * cv.itsec.bsdf->f(cv.wo, (w2o1 * btw).unit());

    Matrix3x3 o2w2;
    make_coord_space(o2w2, lv.itsec.n);
    Matrix3x3 w2o2(o2w2.T());

    L = L * lv.throughput * lv.itsec.bsdf->f((w2o2 * (-btw)).unit(), lv.wo);

    L = L * g;

    return L;
}

Vector3D BDPT::est_radiance_global_illumination(const Ray& ray) {
    Intersection isect;
    if (!bvh->intersect(ray, &isect)) return Vector3D();
    return isect.bsdf->get_emission() + Shade(ray);
}

//Naive BDPT implementation
Vector3D BDPT::Shade(const Ray& ray) {

    //Get initial light ray
    SceneLight* light = scene->lights[0];
    Vector3D Le;
    Ray lightRay;
    lightRay.d = hemisphereSampler->get_sample();
    double lightPdf, a, b;
    Le = light->sample_O(lightRay, &lightPdf, &a, &b);

    vector<BDPT_Vertex> CamPath, LightPath;
    int ray_depth = 0;
    //Use cumulative pdf along the path to evaluate path weight
    double cum_pdf = 1.0;
    Vector3D throughput(1);
    Ray wi(lightRay);

    //Gen light path
    while (ray_depth < max_ray_depth) {
        BDPT_Vertex vert;

        if (!bvh->intersect(wi,&vert.itsec))
            break;

        //Update vertex point
        vert.point = wi.at_time(vert.itsec.t);

        Matrix3x3 o2w;
        make_coord_space(o2w, vert.itsec.n);
        Matrix3x3 w2o(o2w.T());

        vert.wi = (w2o * (-wi.d)).unit();
        double bsdf_pdf;

        //Update throughput.
        //Filter out undesired pdf to support microfact
        Vector3D sample = vert.itsec.bsdf->sample_f(vert.wi, &vert.wo, &bsdf_pdf);
        if (bsdf_pdf > 0) {
            throughput = throughput * sample * fabs(vert.wo.z) / bsdf_pdf;
            cum_pdf *= bsdf_pdf;
        }
        else
            break;

        vert.throughput = throughput;
        vert.pdf = cum_pdf;

        ++ray_depth;

        //If go into a dark vertex, stop
        if (throughput.illum() < 0.0001)
          break;

        LightPath.push_back(vert);

        wi = Ray(vert.point, (o2w * vert.wo).unit());
        wi.o += wi.d * EPS_D;
    }

    //Gen Cam path
    wi = Ray(ray);
    wi.max_t = INF_D;
    ray_depth = 0;
    throughput = Vector3D(1);
    cum_pdf = 1.0;

    while (ray_depth < max_ray_depth) {
        BDPT_Vertex vert;

        if (!bvh->intersect(wi,&vert.itsec))
            break;

        vert.point = wi.at_time(vert.itsec.t);

        Matrix3x3 o2w;
        make_coord_space(o2w, vert.itsec.n);
        Matrix3x3 w2o = o2w.T();

        vert.wo = (w2o*(-wi.d)).unit();

        double bsdf_pdf;
        Vector3D sample = vert.itsec.bsdf->sample_f(vert.wo, &vert.wi, &bsdf_pdf);
        if (bsdf_pdf > 0){
            throughput = throughput * sample * fabs(vert.wi.z) / bsdf_pdf;
            cum_pdf *= bsdf_pdf;
        }
        else
            break;
        
        vert.throughput = throughput;
        vert.pdf = cum_pdf;

        ++ray_depth;
        CamPath.push_back(vert);

        //do not stop even when throughput is 0
        //delta material need to be recorded

        wi = Ray(vert.point, (o2w * vert.wi).unit());
        wi.o += wi.d * EPS_D;
    }

    //Get denominator for path weight
    double weight = 0.0;
    for (auto& c : CamPath) {
        for (auto& l : LightPath) {
            weight += c.pdf*l.pdf;
        }
    }

    Vector3D li;

    for (int i=1; i<CamPath.size()+1; i++){
        BDPT_Vertex &camVert = CamPath[i-1];

        Vector3D dir2light;
        double dist2light, pdf;
        Vector3D le;

        le = light->sample_L(camVert.point, &dir2light, &dist2light, &pdf);
        Matrix3x3 o2w;
        make_coord_space(o2w, camVert.itsec.n);
        Matrix3x3 w2o = o2w.T();

        Vector3D wi_o = (w2o*dir2light).unit();

        //Direct light sampling
        if (!bvh->has_intersection(Ray(camVert.point + EPS_D * dir2light, dir2light, dist2light-EPS_D))){
            if (i > 1)
                le = le * CamPath[i-2].throughput;

            Vector3D f = camVert.itsec.bsdf->f(wi_o, camVert.wo);
            li += le * f * fabs(wi_o.z) / pdf;
        }

        //get emission from the next vertex for specular surface
        if (camVert.itsec.bsdf->is_delta() && i < CamPath.size())
            li += CamPath[i].itsec.bsdf->get_emission() * camVert.throughput;

        //get sub path value
        for (int j=1; j<LightPath.size()+1; j++){
            BDPT_Vertex lvert = LightPath[j-1];
            Vector3D btw = lvert.point - camVert.point;
            if (!bvh->has_intersection(Ray(lvert.point + EPS_D * lvert.itsec.n, btw.unit(), btw.norm()-EPS_D)))
                continue;
            li += Le * evalPath(CamPath, LightPath, i, j) * lvert.pdf * camVert.pdf / weight;
        }

    }
    return li;
}



}