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

void Camera::configure(const CameraInfo& info, size_t screenW, size_t screenH) {
  this->screenW = screenW;
  this->screenH = screenH;
  nClip = info.nClip;
  fClip = info.fClip;
  hFov = info.hFov;
  vFov = info.vFov;

  double ar1 = tan(radians(hFov) / 2) / tan(radians(vFov) / 2);
  ar = static_cast<double>(screenW) / screenH;
  if (ar1 < ar) {
    // hFov is too small
    hFov = 2 * degrees(atan(tan(radians(vFov) / 2) * ar));
  } else if (ar1 > ar) {
    // vFov is too small
    vFov = 2 * degrees(atan(tan(radians(hFov) / 2) / ar));
  }
  screenDist = ((double) screenH) / (2.0 * tan(radians(vFov) / 2));
}

void Camera::place(const Vector3D& targetPos, const double phi,
                   const double theta, const double r, const double minR,
                   const double maxR) {
  double r_ = min(max(r, minR), maxR);
  double phi_ = (sin(phi) == 0) ? (phi + EPS_F) : phi;
  this->targetPos = targetPos;
  this->phi = phi_;
  this->theta = theta;
  this->r = r_;
  this->minR = minR;
  this->maxR = maxR;
  compute_position();
}

void Camera::copy_placement(const Camera& other) {
  pos = other.pos;
  targetPos = other.targetPos;
  phi = other.phi;
  theta = other.theta;
  minR = other.minR;
  maxR = other.maxR;
  c2w = other.c2w;
  nClip = other.nClip;
  fClip = other.fClip;
}

void Camera::set_screen_size(const size_t screenW, const size_t screenH) {
  this->screenW = screenW;
  this->screenH = screenH;
  ar = 1.0 * screenW / screenH;
  hFov = 2 * degrees(atan(((double) screenW) / (2 * screenDist)));
  vFov = 2 * degrees(atan(((double) screenH) / (2 * screenDist)));
}

void Camera::move_by(const double dx, const double dy, const double d) {
  const double scaleFactor = d / screenDist;
  const Vector3D& displacement =
    c2w[0] * (dx * scaleFactor) + c2w[1] * (dy * scaleFactor);
  pos += displacement;
  targetPos += displacement;
}

void Camera::move_forward(const double dist) {
  double newR = min(max(r - dist, minR), maxR);
  pos = targetPos + ((pos - targetPos) * (newR / r));
  r = newR;
}

void Camera::rotate_by(const double dPhi, const double dTheta) {
  phi = clamp(phi + dPhi, 0.0, (double) PI);
  theta += dTheta;
  compute_position();
}

void Camera::compute_position() {
  double sinPhi = sin(phi);
  if (sinPhi == 0) {
    phi += EPS_F;
    sinPhi = sin(phi);
  }
  const Vector3D dirToCamera(r * sinPhi * sin(theta),
                             r * cos(phi),
                             r * sinPhi * cos(theta));
  pos = targetPos + dirToCamera;
  Vector3D upVec(0, sinPhi > 0 ? 1 : -1, 0);
  Vector3D screenXDir = cross(upVec, dirToCamera);
  screenXDir.normalize();
  Vector3D screenYDir = cross(dirToCamera, screenXDir);
  screenYDir.normalize();

  c2w[0] = screenXDir;
  c2w[1] = screenYDir;
  c2w[2] = dirToCamera.unit();   // camera's view direction is the
                                 // opposite of of dirToCamera, so
                                 // directly using dirToCamera as
                                 // column 2 of the matrix takes [0 0 -1]
                                 // to the world space view direction
}

  // double hFov, vFov, ar, nClip, fClip;

  // // Current position and target point (the point the camera is looking at).
  // Vector3D pos, targetPos;

  // // Orientation relative to target, and min & max distance from the target.
  // double phi, theta, r, minR, maxR;

  // // camera-to-world rotation matrix (note: also need to translate a
  // // camera-space point by 'pos' to perform a full camera-to-world
  // // transform)
  // Matrix3x3 c2w;

  // // Info about screen to render to; it corresponds to the camera's full field
  // // of view at some distance.
  // size_t screenW, screenH;
  // double screenDist;
void Camera::dump_settings(string filename) {
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
  file << focalDistance << " " << lensRadius << endl;
  cout << "[Camera] Dumped settings to " << filename << endl;
}

void Camera::load_settings(string filename) {
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
  file >> focalDistance >> lensRadius;
  cout << "[Camera] Loaded settings from " << filename << endl;
}


Ray Camera::generate_ray_for_thin_lens(double x, double y, double rndR, double rndTheta) const {

    // TODO: 4.1
    // compute position and direction of ray from the input sensor sample coordinate.
    // Note: use rndR and rndTheta to uniformly sample a unit disk.


    Vector3D bottomLeft = Vector3D(-tan(radians(hFov)*.5), -tan(radians(vFov)*.5),(double)1);
    Vector3D topRight = Vector3D(tan(radians(hFov)*.5),  tan(radians(vFov)*.5),(double)1);

    Vector3D pointOnSensor = Vector3D(-((1-x)*bottomLeft.x + x*topRight.x), -((1-y)*bottomLeft.y + y*topRight.y), (double)1.0);
    Vector3D dirToFocalPointFromCentre = Vector3D(0,0,0)-pointOnSensor;
    Ray throughCentreLensRay = Ray(Vector3D(0,0,0), dirToFocalPointFromCentre);
    Vector3D pLens = Vector3D(lensRadius*sqrt(rndR)*cos(rndTheta), lensRadius*sqrt(rndR)*sin(rndTheta), 0);

    double tValAtFocalPoint = -focalDistance/dirToFocalPointFromCentre.z;
    Vector3D pFocal = throughCentreLensRay.at_time(tValAtFocalPoint);
    Vector3D dirToFocal = pFocal-pLens;
    dirToFocal.normalize();

    Vector3D worldDirToFocal = c2w*dirToFocal;
    Vector3D worldPLens = c2w*pLens + pos;

    Ray rayForLens = Ray(worldPLens, worldDirToFocal);
    rayForLens.min_t = nClip;
    rayForLens.max_t = fClip;
    return rayForLens;
}


} // namespace CGL
