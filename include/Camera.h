#ifndef CAMERA_H
#define CAMERA_H

#ifndef M_PI
#define M_PI 3.14159265359
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdlib.h>

#include "Ray.h"

Eigen::Vector3d random_ray(){
  Eigen::Vector3d p;
  do{
    p = 2.0 * Eigen::Vector3d(drand48(),drand48(),0) - Eigen::Vector3d(1,1,0);
  }while(p.dot(p) >= 1.0);
  return p;
}

class Camera{
  public:
    Camera(Eigen::Vector3d from, Eigen::Vector3d at, Eigen::Vector3d v_up,double v_fov, double aspect, double aperture, double focus_dist){
      lens_radius = aperture / 2;
      double theta = v_fov * M_PI /180;
      double half_height = std::tan(theta/2);
      double half_width = aspect * half_height;
      origin = from;
      w = (from - at).normalized();
      u = v_up.cross(w).normalized();
      v = w.cross(u);
      llc = origin - half_width * focus_dist * u - half_height * focus_dist * v - focus_dist * w;
      horizontal = 2 * half_width * focus_dist * u;
      vertical = 2 * half_height * focus_dist * v;
    }

    Ray generateRay(float u, float v){
      Eigen::Vector3d ray_origin = origin;
      Eigen::Vector3d ray_direction = llc + u * horizontal + v * vertical - origin;
      return Ray(ray_origin, ray_direction);
    }
  private:
  Eigen::Vector3d origin;
  Eigen::Vector3d llc;
  Eigen::Vector3d horizontal;
  Eigen::Vector3d vertical;
  Eigen::Vector3d u;
  Eigen::Vector3d v;
  Eigen::Vector3d w;
  double lens_radius;
};

#endif
