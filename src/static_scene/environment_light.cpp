#include "environment_light.h"

#include <algorithm>
#include <iostream>
#include <fstream>

namespace CGL { namespace StaticScene {

EnvironmentLight::EnvironmentLight(const HDRImageBuffer* envMap)
    : envMap(envMap) {
    	init();
}

EnvironmentLight::~EnvironmentLight() {
    delete[] pdf_envmap;
    delete[] conds_y;
    delete[] marginal_y;
}


void EnvironmentLight::init() {
	uint32_t w = envMap->w, h = envMap->h;
  pdf_envmap = new double[w * h];
	conds_y = new double[w * h];
	marginal_y = new double[h];

	std::cout << "[PathTracer] Initializing environment light...";

  // 3.3 step 1
	// Store the environment map pdf to pdf_envmap

	double sum = 0;
	for (int j = 0; j < h; ++j) {
		for (int i = 0; i < w; ++i) {
            pdf_envmap[w * j + i] = envMap->data[w * j + i].illum() * sin(PI * (j+.5) / h);
            sum += pdf_envmap[w * j + i];

		}
	}

  
  // TODO: 3.3 step 2
  // Store the marginal distribution for y to marginal_y. Make sure to normalize pdf_envmap.

  for (int m = 0; m< h*w; m++){
      pdf_envmap[m] = pdf_envmap[m]/sum;
   
  }


  double preCalcPart2[h];
  double runningSum = 0;
  double currentDensity = 0;
  for (int j = 0; j< h; j++){
    for (int i = 0; i < w; i++){
      currentDensity += pdf_envmap[w * j + i];
      runningSum += pdf_envmap[w * j + i];
    }
    preCalcPart2[j] = currentDensity;
    marginal_y[j] = runningSum;
    currentDensity = 0;
  }

  // TODO: 3.3 step 3
  // Store the conditional distribution for x given y to conds_y

  for (int j = 0; j< h; j++){
    for (int i = 1; i < w; i++){
      conds_y[w * j + i] = conds_y[w * j + i - 1] + pdf_envmap[w * j + i] / preCalcPart2[j];
    }
  }

	if (true) {
    std::cout << "Saving out probability_debug image for debug." << std::endl;
    save_probability_debug();
  }

	std::cout << "done." << std::endl;
}

// Helper functions

void EnvironmentLight::save_probability_debug() {
	uint32_t w = envMap->w, h = envMap->h;
	uint8_t* img = new uint8_t[4*w*h];

	for (int j = 0; j < h; ++j) {
		for (int i = 0; i < w; ++i) {
			img[4 * (j * w + i) + 3] = 255;
			img[4 * (j * w + i) + 0] = 255 * marginal_y[j];
			img[4 * (j * w + i) + 1] = 255 * conds_y[j * w + i];
			img[4 * (j * w + i) + 2] = 0;
		}
	}

    lodepng::encode("probability_debug.png", img, w, h);
    delete[] img;
}

Vector2D EnvironmentLight::theta_phi_to_xy(const Vector2D &theta_phi) const {
    uint32_t w = envMap->w, h = envMap->h;
    double x = theta_phi.y / 2. / PI * w;
    double y = theta_phi.x / PI * h;
    return Vector2D(x, y);
}

Vector2D EnvironmentLight::xy_to_theta_phi(const Vector2D &xy) const {
    uint32_t w = envMap->w, h = envMap->h;
    double x = xy.x;
    double y = xy.y;
    double phi = x / w * 2.0 * PI;
    double theta = y / h * PI;
    return Vector2D(theta, phi);
}

Vector2D EnvironmentLight::dir_to_theta_phi(const Vector3D &dir) const {
    Vector3D unit_dir = dir.unit();
    double theta = acos(unit_dir.y);
    double phi = atan2(-unit_dir.z, unit_dir.x) + PI;
    return Vector2D(theta, phi);
}

Vector3D EnvironmentLight::theta_phi_to_dir(const Vector2D& theta_phi) const {
    double theta = theta_phi.x;
    double phi = theta_phi.y;

    double y = cos(theta);
    double x = cos(phi - PI) * sin(theta);
    double z = -sin(phi - PI) * sin(theta);

    return Vector3D(x, y, z);
}

// Credits to Luowen Qian from Spring 2018 for this more robust bilerp
Spectrum EnvironmentLight::bilerp(const Vector2D& xy) const {
    long right = lround(xy.x), left, v = lround(xy.y);
    double u1 = right - xy.x + .5, v1;
    if (right == 0 || right == envMap->w) {
        left = envMap->w - 1;
        right = 0;
    } else left = right - 1;
    if (v == 0) v1 = v = 1; else if (v == envMap->h) {
        v = envMap->h - 1;
        v1 = 0;
    } else v1 = v - xy.y + .5;
    auto bottom = envMap->w * v, top = bottom - envMap->w;
    auto u0 = 1 - u1;
    return (envMap->data[top + left] * u1 + envMap->data[top + right] * u0) * v1 +
        (envMap->data[bottom + left] * u1 + envMap->data[bottom + right] * u0) * (1 - v1);
}


Spectrum EnvironmentLight::sample_L(const Vector3D& p, Vector3D* wi,
                                    float* distToLight,
                                    float* pdf) const {
  // TODO: 3.2
	// First implement uniform sphere sampling for the environment light
  Vector3D dir = sampler_uniform_sphere.get_sample();
  Vector2D thetaphi = dir_to_theta_phi(dir);
  Spectrum radiance = bilerp(theta_phi_to_xy(thetaphi));
  *wi = dir;
  *distToLight = INF_D;
  *pdf = 1.0/(4.0*PI);
  return radiance;
  // TODO: 3.3
	// Later implement full importance sampling

  Vector2D sample = sampler_uniform2d.get_sample();
	int xVal, yVal;

	yVal = (int) (std::upper_bound(marginal_y, marginal_y + envMap->h, sample.x) - marginal_y);
	xVal = (int) (std::upper_bound(conds_y + yVal * envMap->w, conds_y + yVal * envMap->w + envMap->w, sample.y) - (conds_y + yVal * envMap->w));

	Vector2D newSample = Vector2D(xVal, yVal);
	*wi = theta_phi_to_dir(xy_to_theta_phi(newSample));
	*distToLight = INF_D;
	*pdf = pdf_envmap[xVal + yVal * envMap->w] * (double)envMap->w * (double)envMap->h / (2 * PI * PI * sin(xy_to_theta_phi(newSample).x));

	return envMap->data[xVal + yVal * envMap->w];
}

Spectrum EnvironmentLight::sample_dir(const Ray& r) const {
  // TODO: 3.1
	// Use the helper functions to convert r.d into (x,y)
	// then bilerp the return value

	return bilerp(theta_phi_to_xy(dir_to_theta_phi(r.d)));

}

} // namespace StaticScene
} // namespace CGL
