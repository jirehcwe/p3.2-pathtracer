#include "bvh.h"

#include "CGL/CGL.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL { namespace StaticScene {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  root = construct_bvh(_primitives, max_leaf_size);

}

BVHAccel::~BVHAccel() {
  if (root) delete root;
}

BBox BVHAccel::get_bbox() const {
  return root->bb;
}

void BVHAccel::draw(BVHNode *node, const Color& c, float alpha) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->draw(c, alpha);
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color& c, float alpha) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->drawOutline(c, alpha);
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(const std::vector<Primitive*>& prims, size_t max_leaf_size) {
  
  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox bbox;
  Vector3D mainCentroid;
  BVHNode *node;


  //Finding total bounding box
  for (Primitive *p : prims) {
        BBox bb = p->get_bbox();
        bbox.expand(bb); 
  }
  
 


  node = new BVHNode(bbox);


  if (prims.size() <= max_leaf_size){
    node->prims = new vector<Primitive *>(prims);
    return node;
  }

  //Calculating largest axis to split along
  double splitAxis;
  int axisNum;
  Vector3D mainExtent = bbox.extent;
  
  if (mainExtent.x > mainExtent.y){
    if (mainExtent.x > mainExtent.z){
      axisNum = 0;
    } else {
      axisNum = 2;
    }
  }else {
    if (mainExtent.y > mainExtent.z){
      axisNum = 1;
    } else {
      axisNum = 2;
    }
  }

  //Calculating average centroid
  Vector3D sum(0);
  int counter = 0;
  for (Primitive *p : prims){
    sum += p->get_bbox().centroid();
    counter++;
  }

  sum = sum/counter;

  vector<Primitive *>left, right;

  //Splitting by centroid 
  for (Primitive *p : prims) {
      Vector3D pCentroid = p->get_bbox().centroid();
      switch (axisNum){
        case 0:
          if ( pCentroid.x < sum.x ){
            left.push_back(p);
          } else {
            right.push_back(p);
          }
          break;
        case 1:
          if ( pCentroid.y < sum.y ){
            left.push_back(p);
          } else {
            right.push_back(p);
          }
          break;
        case 2:
          if ( pCentroid.z < sum.z ){
            left.push_back(p);
          } else {
            right.push_back(p);
          }
          break;
      }
  }

  if (left.size() == 0){
      left.push_back(*(right.end()));
      right.pop_back();
    
  } else if (right.size() == 0){
      right.push_back(*(left.end()));
      left.pop_back();
  }
  
  node->l = construct_bvh(left, max_leaf_size);
  node->r = construct_bvh(right, max_leaf_size);

  return node;
  

}




bool BVHAccel::intersect(const Ray& ray, BVHNode *node) const {

// TODO (Part 2.3):
// Fill in the intersect function.
// Take note that this function has a short-circuit that the
// Intersection version cannot, since it returns as soon as it finds
// a hit, it doesn't actually have to find the closest hit.


  double t0 = ray.min_t, t1 = ray.max_t;
  if (!node->bb.intersect(ray, t0, t1)){
    return false;
  }

  if (t0 < ray.min_t || t1 < ray.min_t ||t0 > ray.max_t || t1 > ray.max_t){
    return false;
  }


  if (node->isLeaf()){
    for (Primitive *p : *node->prims){
      total_isects++;
      if (p->intersect(ray)){
        return true;
      }
    }

    return false;
  }

  return (intersect(ray, node->l) || intersect(ray, node->r));

}

bool BVHAccel::intersect(const Ray& ray, Intersection* i, BVHNode *node) const {

  // TODO (Part 2.3):
  //Fill in the intersection function.

double t0 = ray.min_t, t1 = ray.max_t;
  if (!node->bb.intersect(ray, t0, t1)){
    return false;
  }

  if (t0 < ray.min_t || t1 < ray.min_t || t0 > ray.max_t || t1 > ray.max_t){
    return false;
  }

  bool intersected = false;
  if (node->isLeaf()){
    for (Primitive *p : *node->prims){
      total_isects++;
      if (p->intersect(ray, i)){
        intersected = true;
      }
    }

    return intersected;
  }

  bool hit1 = intersect(ray, i, node->l);
  bool hit2 = intersect(ray, i, node->r);

  return (hit1 || hit2);

}

}  // namespace StaticScene
}  // namespace CGL
