#ifndef RAY_H
#define RAY_H

#include <Eigen/Core>

class Ray{
  public:
    Ray(){

    }
    
    Ray(Eigen::Vector3d orig, Eigen::Vector3d dir){
      origin = orig;
      direction = dir;
    }

    Eigen::Vector3d getOrigin(){
      return origin;
    }

    Eigen::Vector3d getDirection(){
      return direction;
    }

    Eigen::Vector3d getPointAt(double t){
      return origin + t * direction;
    }
  private:
    Eigen::Vector3d origin;
    Eigen::Vector3d direction;
};

#endif
