//
// TODO: Copy over 3-1 code after turning on BUILD_3-1 flag
//

#include "part1_code.h"
#include <time.h>

using namespace CGL::StaticScene;

using std::min;
using std::max;

namespace CGL {

  Spectrum PathTracer::estimate_direct_lighting_hemisphere(const Ray& r, const Intersection& isect) {
    // Estimate the lighting from this intersection coming directly from a light.
    // For this function, sample uniformly in a hemisphere. 

    // make a coordinate system for a hit point
    // with N aligned with the Z direction.
    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);
    Matrix3x3 w2o = o2w.T();

    // w_out points towards the source of the ray (e.g.,
    // toward the camera if this is a primary ray)
    const Vector3D& hit_p = r.o + r.d * isect.t;
    const Vector3D& w_out = w2o * (-r.d);

    // This is the same number of total samples as estimate_direct_lighting_importance (outside of delta lights). 
    // We keep the same number of samples for clarity of comparison.
    int num_samples = scene->lights.size() * ns_area_light;
    Spectrum L_out;

    // TODO (Part 3.2): 
    // Write your sampling loop here
    // COMMENT OUT `normal_shading` IN `est_radiance_global_illumination` BEFORE YOU BEGIN

    for (int i=0;i< num_samples; i++){
      Vector3D wi = hemisphereSampler->get_sample();
      Vector3D wi_world = o2w * wi;
      Ray sampleRay = Ray(hit_p + EPS_D*wi_world, wi_world);
      Intersection newIsect;

      if (bvh->intersect(sampleRay, &newIsect)){
        //bsdf sets the amount of the light reflected
        //cosine is angle and amount of light coming out based on the return value
        //pdf is by monte carlo

        //This is high variance.
        Spectrum emission = newIsect.bsdf->get_emission();
        Spectrum sample = emission*isect.bsdf->f(w_out, wi) * cos_theta(wi) * 2.0 * (double)PI;
        L_out += sample;

      }
    }

    return L_out/(double)num_samples;


  }

  Spectrum PathTracer::estimate_direct_lighting_importance(const Ray& r, const Intersection& isect) {
    // Estimate the lighting from this intersection coming directly from a light.
    // To implement importance sampling, sample only from lights, not uniformly in a hemisphere. 

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

    // TODO (Part 3.2): 
    // Here is where your code for looping over scene lights goes
    // COMMENT OUT `normal_shading` IN `est_radiance_global_illumination` BEFORE YOU BEGIN
    int num_lights = scene->lights.size();

    for (int i=0;i< num_lights;i++){
        Vector3D wi;
        float distToLight;
        float pdf;


      if (scene->lights[i]->is_delta_light()){
        Spectrum lightSample = scene->lights[i]->sample_L(hit_p, &wi, &distToLight, &pdf);
          Vector3D w_in = w2o * wi;

          if (w_in.z >= 0){
            Intersection shadowCheck;
            Ray shadowRay(hit_p + EPS_D*wi, wi);
            shadowRay.max_t = distToLight;
            bool intersected = bvh->intersect(shadowRay, &shadowCheck);
              
            if (!intersected){
              L_out += (lightSample * cos_theta(w_in) * isect.bsdf->f(w_out, w_in))/pdf;
            }
          }


      }else{

        Spectrum runningSample;
        int num_samples = (int)ns_area_light;
        for (int j=0; j< num_samples; j++){

          Spectrum lightSample = scene->lights[i]->sample_L(hit_p, &wi, &distToLight, &pdf);
          Vector3D w_in = w2o * wi;

          if (w_in.z >= 0){
            Intersection shadowCheck;
            Ray shadowRay(hit_p + EPS_D*wi, wi);
            shadowRay.max_t = distToLight;
            bool intersected = bvh->intersect(shadowRay, &shadowCheck);
              
            if (!intersected){
              runningSample += (lightSample * cos_theta(w_in) * isect.bsdf->f(w_out, w_in))/pdf;
            }
          }

        }

        L_out += runningSample/(double)num_samples;
      }
    }
    

    return L_out;
  }

  Spectrum PathTracer::zero_bounce_radiance(const Ray&r, const Intersection& isect) {
    // TODO (Part 4.2):
    // Returns the light that results from no bounces of light

    return isect.bsdf->get_emission();

  }

 
  Spectrum PathTracer::one_bounce_radiance(const Ray&r, const Intersection& isect) {
    
    // TODO (Part 4.2):
    // Returns either the direct illumination by hemisphere or importance sampling
    // depending on `direct_hemisphere_sample`
    // (you implemented these functions in Part 3)

    if (direct_hemisphere_sample){
      return estimate_direct_lighting_hemisphere(r, isect);
    } else {
      return estimate_direct_lighting_importance(r, isect);
    }

    
  }

  Spectrum PathTracer::at_least_one_bounce_radiance(const Ray&r, const Intersection& isect) {
    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);
    Matrix3x3 w2o = o2w.T();

    Vector3D hit_p = r.o + r.d * isect.t;
    Vector3D w_out = w2o * (-r.d);


    Spectrum L_out(0,0,0);

    if (!isect.bsdf->is_delta()){
      L_out += one_bounce_radiance(r, isect);
    }
    // TODO (Part 4.2): 
    // Here is where your code for sampling the BSDF,
    // performing Russian roulette step, and returning a recursively 
    // traced ray (when applicable) goes
    Vector3D w_in;
    float pdf; 
    Spectrum bsdf = isect.bsdf->sample_f(w_out, &w_in, &pdf);
    float rouletteProb = 0.7;

    if(max_ray_depth <= 1){
      return L_out;
    }else if (coin_flip(rouletteProb) || r.depth == max_ray_depth){
      Vector3D direction = o2w* w_in;
      Ray bounceRay = Ray(hit_p + EPS_D*direction, direction, (int)r.depth - 1 );
      Intersection bounceIsect;
      if (bvh->intersect(bounceRay, &bounceIsect)){
        Spectrum bounceSample = at_least_one_bounce_radiance(bounceRay, bounceIsect);
        if (isect.bsdf->is_delta()){
          bounceSample += zero_bounce_radiance(bounceRay, bounceIsect);
        }
        Spectrum weightedSample = bounceSample *bsdf * abs_cos_theta(w_in)/ pdf / rouletteProb;
        L_out += weightedSample;
      }
    }
    

    
    return L_out;

  }

 
  Spectrum PathTracer::est_radiance_global_illumination(const Ray &r) {
    Intersection isect;
    Spectrum L_out;

    // You will extend this in assignment 3-2. 
    // If no intersection occurs, we simply return black.
    // This changes if you implement hemispherical lighting for extra credit.

    if (!bvh->intersect(r, &isect)) 
      return L_out;

    // This line returns a color depending only on the normal vector 
    // to the surface at the intersection point.
    // REMOVE IT when you are ready to begin Part 3.

    return normal_shading(isect.n);

    // TODO (Part 3): Return the direct illumination.

    // L_out = estimate_direct_lighting_hemisphere(r, isect);
    // L_out = estimate_direct_lighting_importance(r, isect);

    // TODO (Part 4): Accumulate the "direct" and "indirect" 
    // parts of global illumination into L_out rather than just direct

    L_out = zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect);
    return L_out;
  }

  Spectrum PathTracer::raytrace_pixel(size_t x, size_t y, bool useThinLens) {

    // TODO (Part 1.1):
    // Make a loop that generates num_samples camera rays and traces them 
    // through the scene. Return the average Spectrum. 
    // You should call est_radiance_global_illumination in this function.

    double width = (double)x;
    double height = (double)y;
    if (ns_aa == 1){
      Ray ray = camera->generate_ray((width+ 0.5)/(double)sampleBuffer.w, (height + 0.5)/(double)sampleBuffer.h);
      ray.depth = max_ray_depth;
      return est_radiance_global_illumination(ray);
    }

    Spectrum sum = Spectrum(0, 0, 0);
    float illumSum = 0;
    float squaredIllumSum = 0;
    int currentNumSamples = 0;
    for (int i=0; i < ns_aa ; i++){
      Vector2D randomSample = gridSampler->get_sample();
      Ray ray = camera->generate_ray((width+randomSample.x)/(double)sampleBuffer.w, (height + randomSample.y)/(double)sampleBuffer.h);
      ray.depth = max_ray_depth;
      Spectrum currentLightSample = est_radiance_global_illumination(ray);
      sum += currentLightSample;

      //Adapative Sampling variables
      illumSum += currentLightSample.illum();
      squaredIllumSum += currentLightSample.illum() * currentLightSample.illum();
      currentNumSamples++;

      // //Calculate variance and mean
      // if (currentNumSamples % samplesPerBatch == 0){
      //   double mean = illumSum/currentNumSamples;
      //   double variance = (squaredIllumSum - (illumSum*illumSum/currentNumSamples))/(currentNumSamples-1);
      //   double standardDeviation = sqrt(variance);
      //   double convergence = 1.96*standardDeviation/sqrt(currentNumSamples);
      //   if (convergence <= maxTolerance*mean){
      //     break;
      //   }
      // }

    }


    // TODO (Part 5):
    // Modify your implementation to include adaptive sampling.
    // Use the command line parameters "samplesPerBatch" and "maxTolerance"

    sampleCountBuffer[x + y * frameBuffer.w] = currentNumSamples;
    return sum/(double)currentNumSamples;


  }

  // Diffuse BSDF //

  Spectrum DiffuseBSDF::f(const Vector3D& wo, const Vector3D& wi) {

    // TODO (Part 3.1): 
    // This function takes in both wo and wi and returns the evaluation of
    // the BSDF for those two directions.
    return (this->reflectance/PI);
  }

  Spectrum DiffuseBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

    // TODO (Part 3.1): 
    // This function takes in only wo and provides pointers for wi and pdf,
    // which should be assigned by this function.
    // After sampling a value for wi, it returns the evaluation of the BSDF
    // at (wo, *wi).
    *wi = sampler.get_sample(pdf);
    return (this->reflectance/PI);
  }

  // Camera //
  Ray Camera::generate_ray(double x, double y) const {

    // TODO (Part 1.2):
    // compute position of the input sensor sample coordinate on the
    // canonical sensor plane one unit away from the pinhole.
    // Note: hFov and vFov are in degrees.
    // 
    
    Vector3D bottomLeft = Vector3D(-tan(radians(hFov)*.5), -tan(radians(vFov)*.5),(double)-1);
    Vector3D topRight = Vector3D(tan(radians(hFov)*.5),  tan(radians(vFov)*.5),(double)-1);

    Vector3D pointInCameraSpace = Vector3D((1-x)*bottomLeft.x + x*topRight.x, (1-y)*bottomLeft.y + y*topRight.y, (double)-1.0);
    Vector3D dirInWorldSpace = c2w*pointInCameraSpace;
    dirInWorldSpace.normalize();
    Ray result = Ray(pos, dirInWorldSpace);

    result.min_t = nClip;
    result.max_t = fClip;

    return result;


  }
}
