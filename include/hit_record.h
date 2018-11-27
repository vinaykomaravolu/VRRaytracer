#ifndef HITRECORD_H
#define HITRECORD_H

#include <Eigen/Core>

#include "Ray.h"

class Material;

struct HitRecord{
    float t;
    Eigen::Vector3d p;
    Eigen::Vector3d normal;
    Material *mat_ptr;
};

#endif