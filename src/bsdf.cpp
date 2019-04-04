#include "bsdf.h"

#include <iostream>
#include <algorithm>
#include <utility>

using std::min;
using std::max;
using std::swap;

namespace CGL {

void make_coord_space(Matrix3x3& o2w, const Vector3D& n) {
  Vector3D z = Vector3D(n.x, n.y, n.z);
  Vector3D h = z;
  if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z)) h.x = 1.0;
  else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z)) h.y = 1.0;
  else h.z = 1.0;
  
  z.normalize();
  Vector3D y = cross(h, z);
  y.normalize();
  Vector3D x = cross(z, y);
  x.normalize();

  o2w[0] = x;
  o2w[1] = y;
  o2w[2] = z;
}

// Mirror BSDF //

Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO: 1.2
  // Using BSDF::reflect(), implement sample_f for a mirror surface
  *pdf = 1.0f;
  reflect(wo, wi);
  return reflectance/abs_cos_theta(*wi);
}

// Microfacet BSDF //

double MicrofacetBSDF::G(const Vector3D& wo, const Vector3D& wi) {
    return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
}

double MicrofacetBSDF::D(const Vector3D& h) {
  // TODO: 2.2
  // Compute Beckmann normal distribution function (NDF) here.
  // You will need the roughness alpha.
  double thetaH = getTheta(h);
  double numerator = exp(-pow(tan(thetaH),2)/pow(alpha, 2));
  double denominator = PI*pow(alpha,2)*pow(cos(thetaH), 4);

  return numerator/denominator;
}

Spectrum MicrofacetBSDF::F(const Vector3D& wi) {
  // TODO: 2.3
  // Compute Fresnel term for reflection on dielectric-conductor interface.
  // You will need both eta and etaK, both of which are Spectrum.
  Spectrum sumOfSquareNK = eta*eta + k*k;
  double thetaI = getTheta(wi); 
  Spectrum fresnelPerpendicular = ((sumOfSquareNK) - 2 * eta*cos(thetaI) + pow(cos(thetaI),2))/
                                ((sumOfSquareNK) + 2*eta*cos(thetaI) + pow(cos(thetaI),2));
  Spectrum fresnelParallel =  ((sumOfSquareNK)*pow(cos(thetaI),2) - 2*eta*cos(thetaI) + 1)/
                              ((sumOfSquareNK)*pow(cos(thetaI),2) + 2*eta*cos(thetaI) + 1);


  return (fresnelParallel + fresnelPerpendicular)/2.0;
}

Spectrum MicrofacetBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  // TODO: 2.1
  // Implement microfacet model here

  Vector3D normal(0,0,1);
  
  if (dot(normal, wi) < 0 || dot(normal, wo) < 0){
    return Spectrum();
  }

  Vector3D halfVector = (wo + wi).unit();

  return (F(wi)*G(wo, wi)*D(halfVector))/(4 * dot(normal, wo) * dot(normal, wi));
}

Spectrum MicrofacetBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO: 2.4
  // *Importance* sample Beckmann normal distribution function (NDF) here.
  // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
  //       and return the sampled BRDF value.

  // *wi = cosineHemisphereSampler.get_sample(pdf);
  // return MicrofacetBSDF::f(wo, *wi);
  

  Vector2D randomR = sampler.get_sample();
  double thetaH = atan(sqrt(-pow(alpha, 2)*log(1 - randomR.x)));
  double phiH = 2*PI*randomR.y;
  Vector3D normal = Vector3D(sin(thetaH)*cos(phiH), sin(thetaH)*sin(phiH), cos(thetaH));
  *wi = -wo + 2*dot(wo, normal)*normal;
  wi->normalize();
  if (wi->z <= 0){
    pdf = 0;
    return Spectrum();
  }
  double denomPDFTheta = pow(alpha, 2)*pow(cos(thetaH), 3);
  double numeratorPDFTheta = exp(-pow(tan(thetaH), 2)/pow(alpha, 2));
  double pdfThetaH = 2*sin(thetaH) * numeratorPDFTheta / denomPDFTheta;
  double pdfPhiH = 1 / (2*PI);
  double pdfOmegaH = pdfThetaH*pdfPhiH/sin(thetaH);
  double pdfOmega = pdfOmegaH/(4*dot(*wi, normal));
  *pdf = pdfOmega;
  return  MicrofacetBSDF::f(wo, *wi);
}

// Refraction BSDF //

Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  return Spectrum();
}

// Glass BSDF //

Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  // TODO: 1.4
  // Compute Fresnel coefficient and either reflect or refract based on it.

  if (!refract(wo, wi, ior)){
    *pdf = 1.0;
    reflect(wo, wi);
    return reflectance / abs_cos_theta(*wi);
  } else{
    float r0sqrt = (1-ior)/(1+ior);
    float r0 = r0sqrt*r0sqrt;
    float schlickCoeff = r0 + (1 - r0)*(1 - abs_cos_theta(wo))*(1 - abs_cos_theta(wo));
    
    if (coin_flip(schlickCoeff)){
      reflect(wo, wi);
      *pdf = schlickCoeff;
      return schlickCoeff * reflectance / abs_cos_theta(*wi);
    }else{
      refract(wo, wi, ior);
      float eta;
      if (wo.z > 0){
        eta = 1/ior;
      } else {
        eta = ior;
      }
      *pdf = 1 - schlickCoeff;
      return (1 - schlickCoeff) * transmittance / abs_cos_theta(*wi) / (eta*eta);
    }
  }
  return Spectrum();
}

void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {

  // TODO: 1.1
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
  
  Vector3D there = Vector3D(-wo.x, -wo.y, wo.z);
  *wi = there;

}

bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {

  // TODO: 1.3
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.

  float refractionIndex;
  if (wo.z > 0){
    refractionIndex = 1/ior;
  } else {
    refractionIndex = ior;
  }

  float cosThetaPrimeSquared = 1 - refractionIndex*refractionIndex*(1-wo.z*wo.z);
  if (cosThetaPrimeSquared < 0){
    return false;
  } else {
    wi->x = -refractionIndex*wo.x;
    wi->y = -refractionIndex*wo.y;
    if (wo.z < 0){
      wi->z = sqrt(cosThetaPrimeSquared);
    } else {
      wi->z = -sqrt(cosThetaPrimeSquared);
    }
    return true;
  }

  
}

// Emission BSDF //

Spectrum EmissionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum EmissionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *pdf = 1.0 / PI;
  *wi  = sampler.get_sample(pdf);
  return Spectrum();
}

} // namespace CGL
