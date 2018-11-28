#ifndef COLOR_H
#define COLOR_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <float.h>
#include "material.h"
#include "objects.h"
#include "write_ppm.h"
#include "Camera.h"
#include "hit_record.h"
#include "scene.h"

#define MAX_T 10000000
#define MIN_T 0.001

#define EMPTY_COLOR Eigen::Vector3d(0.0,0.0,0.0);

Eigen::Vector3d raytrace_color(Ray & ray, Scene & scene, int depth,int max_iter){
    HitRecord record;
    if(scene.hit_closest(ray, MIN_T,MAX_T,record)){
        Ray fragment;
        Eigen::Vector3d attentuation;
        if(depth < max_iter && record.mat_ptr->scatter(ray,record, attentuation, fragment)){
            return (attentuation.array()*raytrace_color(fragment,scene, depth + 1,max_iter).array()).matrix();
        }else{
            return EMPTY_COLOR
        }
    }else{
        Eigen::Vector3d unit = ray.getDirection().normalized();
        double t = 0.5 * (unit.y() + 1.0);
        return (1.0 - t) * Eigen::Vector3d(1.0,1.0,1.0) + t * Eigen::Vector3d(0.5,0.7,1.0);
    }
}

#endif