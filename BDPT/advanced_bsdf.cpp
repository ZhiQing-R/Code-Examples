#include "bsdf.h"

#include <algorithm>
#include <iostream>
#include <utility>

#include "application/visual_debugger.h"

using std::max;
using std::min;
using std::swap;

namespace CGL {

// Mirror BSDF //

Vector3D MirrorBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D MirrorBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {

  // TODO Assignment 7: Part 1
  // Implement MirrorBSDF
  *pdf = 1.0;
  reflect(wo,wi);

  return reflectance/fabs(wi->z);
}

void MirrorBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Mirror BSDF"))
  {
    DragDouble3("Reflectance", &reflectance[0], 0.005);
    ImGui::TreePop();
  }
}

// Microfacet BSDF //

double MicrofacetBSDF::G(const Vector3D wo, const Vector3D wi) {
  return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
}

double MicrofacetBSDF::D(const Vector3D h) {
  // TODO Assignment 7: Part 2
  // Compute Beckmann normal distribution function (NDF) here.
  // You will need the roughness alpha.
  
  double thetah = getTheta(h);
  double alpha2 = alpha*alpha;

  return exp(-(pow(tan(thetah),2)/alpha2)) / (PI*alpha2*pow(h.z,4));
}

Vector3D MicrofacetBSDF::F(const Vector3D wi) {
  // TODO Assignment 7: Part 2
  // Compute Fresnel term for reflection on dielectric-conductor interface.
  // You will need both eta and etaK, both of which are Vector3D.
  Vector3D etak2 = eta*eta + k*k;
  Vector3D etacos = 2*eta*wi.z;
  Vector3D cos2 = Vector3D(wi.z*wi.z);

  Vector3D Rs = (etak2 - etacos + cos2)/(etak2 + etacos + cos2);
  Vector3D Rp = ((etak2*cos2) - etacos + 1) / ((etak2*cos2) + etacos + 1);

  return (Rs+Rp)/2;
}

Vector3D MicrofacetBSDF::f(const Vector3D wo, const Vector3D wi) {
  // TODO Assignment 7: Part 2
  // Implement microfacet model here.
  if (wo.z < 0 || wi.z < 0){
    return Vector3D();
  } 

  Vector3D h = (wo + wi).unit();
  return (F(wi)*G(wo, wi)*D(h)) / (4*wo.z*wi.z);
}

Vector3D MicrofacetBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
  // TODO Assignment 7: Part 2
  // *Importance* sample Beckmann normal distribution function (NDF) here.
  // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
  //       and return the sampled BRDF value.
  double alpha2 = alpha*alpha;

  double thetah = atan(sqrt(-alpha2*log(1.0-random_uniform())));
  double sint = sin(thetah);
  double cost = cos(thetah);

  double phih = random_uniform()*2.0*PI;

  double pthetah = (2.0*sint*exp(-pow(tan(thetah),2)/alpha2))/(alpha2*pow(cost, 3));
  double pphih = 0.5/PI;

  Vector3D h = Vector3D((sint*cos(phih)), (sint*sin(phih)), cost);
  *wi = (2*dot(wo, h)*h) - wo;
  if (wi->z < 0) {
    *pdf = 0;
    return Vector3D();
  }

  double pwh = (pthetah*pphih) / sint;
  *pdf = pwh / (4.0*dot(*wi, h));

  return f(wo,*wi);

  //*wi = cosineHemisphereSampler.get_sample(pdf);
  //return MicrofacetBSDF::f(wo, *wi);
}

double MicrofacetBSDF::pdf_inv (const Vector3D wo, const Vector3D wi) {
  double alpha2 = alpha*alpha;
  Vector3D h = (wi + wo);
  h.normalize();
  double cost = h.z;
  double sint = sqrt(h.x*h.x+h.y*h.y);
  double tant = sint/cost;
  double pthetah = (2.0*sint*exp(-pow(tant,2)/alpha2))/(alpha2*pow(cost, 3));
  double pphih = 0.5/PI;
  double pwh = (pthetah*pphih) / sint;
  return pwh / (4.0*dot(wi, h));

}

void MicrofacetBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Micofacet BSDF"))
  {
    DragDouble3("eta", &eta[0], 0.005);
    DragDouble3("K", &k[0], 0.005);
    DragDouble("alpha", &alpha, 0.005);
    ImGui::TreePop();
  }
}

// Refraction BSDF //

Vector3D RefractionBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D RefractionBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
  // TODO Assignment 7: Part 1
  // Implement RefractionBSDF
  *pdf = 1.;
  if (!refract(wo,wi,ior)) return Vector3D();

  double eta_inv;
  if (wo.z > 0) eta_inv = ior;
  else eta_inv = 1./ior;
  return transmittance*eta_inv*eta_inv/fabs(wi->z);
  
}

void RefractionBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Refraction BSDF"))
  {
    DragDouble3("Transmittance", &transmittance[0], 0.005);
    DragDouble("ior", &ior, 0.005);
    ImGui::TreePop();
  }
}

// Glass BSDF //

Vector3D GlassBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D GlassBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {

  // TODO Assignment 7: Part 1
  // Compute Fresnel coefficient and either reflect or refract based on it.

  //No refraction
  if (!refract(wo, wi, ior)) {
    *pdf = 1;
    reflect(wo, wi);
    return reflectance/fabs(wi->z);
  }

  // compute Fresnel coefficient and use it as the probability of reflection
  double R0 = pow((1.0-ior)/(1.0+ior),2);
  double R = R0 + (1-R0)*pow((1-fabs(wo.z)), 5);
  if (coin_flip(R)) {
    *pdf = R;
    reflect(wo, wi);
    return R*reflectance /fabs(wi->z);
  }
  else {
    *pdf = 1.0-R;
    refract(wo, wi, ior);
    double eta_inv;
    if (wo.z > 0) eta_inv = ior;
    else eta_inv = 1./ior;
    return (1.0-R)*transmittance*eta_inv*eta_inv/fabs(wi->z);
  }

}

void GlassBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Refraction BSDF"))
  {
    DragDouble3("Reflectance", &reflectance[0], 0.005);
    DragDouble3("Transmittance", &transmittance[0], 0.005);
    DragDouble("ior", &ior, 0.005);
    ImGui::TreePop();
  }
}

void BSDF::reflect(const Vector3D wo, Vector3D* wi) {

  // TODO Assignment 7: Part 1
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
  *wi = Vector3D(-wo.x,-wo.y,wo.z);

}

bool BSDF::refract(const Vector3D wo, Vector3D* wi, double ior) {

  // TODO Assignment 7: Part 1
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.
  double eta;
  if (wo.z > 0) eta = 1./ior;
  else eta = ior;

  double cos2 = 1-eta*eta*(1-wo.z*wo.z);
  if (cos2 < 0) return false;

  if (wo.z>0) *wi = Vector3D(-eta*wo.x,-eta*wo.y,-sqrt(cos2));
  else *wi = Vector3D(-eta*wo.x,-eta*wo.y,sqrt(cos2));

  return true;

}

} // namespace CGL
