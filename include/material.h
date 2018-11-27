#ifndef Material_H
#define Material_H

#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct HitRecord;

using namespace std;

#include "Ray.h"
#include "hit_record.h"
//reference from https://github.com/petershirley

double schlick(double cosine, double ref_idx) {
    double r0 = (1-ref_idx) / (1+ref_idx);
    r0 = r0*r0;
    return r0 + (1-r0)*pow((1 - cosine),5);
}

bool refract(const Eigen::Vector3d& v, const Eigen::Vector3d& n, double ni_over_nt, Eigen::Vector3d& refracted) {
    Eigen::Vector3d uv =v.normalized();
    double dt = uv.dot(n);
    double discriminant = 1.0 - ni_over_nt*ni_over_nt*(1-dt*dt);
    if (discriminant > 0) {
        refracted = ni_over_nt*(uv - n*dt) - n*std::sqrt(discriminant);
        return true;
    }
    else 
        return false;
}


Eigen::Vector3d reflect(const Eigen::Vector3d& v, const Eigen::Vector3d& n) {
     return v - 2*v.dot(n)*n;
}


class Material {
    public:
        virtual bool scatter(Ray & r_in, const HitRecord& rec,Eigen::Vector3d & attenuation, Ray & scattered ) = 0;
        
};

Eigen::Vector3d random_in_unit_sphere() {
    Eigen::Vector3d p;
    do {
        
        p = 2.0*Eigen::Vector3d(drand48(),drand48(),drand48()) - Eigen::Vector3d(1,1,1);
    } while (std::sqrt(p.dot(p)) >= 1.0);
    return p;
}


class lambertian : public Material {
    public:
        lambertian(const Eigen::Vector3d& a) : albedo(a) {}
        bool scatter(Ray& r_in, const HitRecord& rec, Eigen::Vector3d& attenuation, Ray& scattered) {
             Eigen::Vector3d target = rec.p + rec.normal + random_in_unit_sphere();
             scattered = Ray(rec.p, target-rec.p);
             attenuation = albedo;
             return true;
        }

        Eigen::Vector3d albedo;
};

class metal : public Material {
    public:
        metal(const Eigen::Vector3d& a, double f) : albedo(a) { if (f < 1) fuzz = f; else fuzz = 1; }
        bool scatter(Ray& r_in, const HitRecord& rec, Eigen::Vector3d& attenuation, Ray& scattered)  {
            Eigen::Vector3d reflected = reflect(r_in.getDirection().normalized(), rec.normal);
            scattered = Ray(rec.p, reflected + fuzz*random_in_unit_sphere());
            attenuation = albedo;
            return scattered.getDirection().dot(rec.normal) > 0;
        }
        Eigen::Vector3d albedo;
        double fuzz;
};

class dielectric : public Material { 
    public:
        dielectric(double ri) : ref_idx(ri) {}
        bool scatter(Ray& r_in, const HitRecord& rec, Eigen::Vector3d& attenuation, Ray& scattered){
             Eigen::Vector3d outward_normal;
             Eigen::Vector3d reflected = reflect(r_in.getDirection(), rec.normal);
             double ni_over_nt;
             attenuation = Eigen::Vector3d(1.0, 1.0, 1.0); 
             Eigen::Vector3d refracted;
             double reflect_prob;
             double cosine;
             if (r_in.getDirection().dot(rec.normal) > 0) {
                  outward_normal = -rec.normal;
                  ni_over_nt = ref_idx;
         //         cosine = ref_idx * dot(r_in.getDirection(), rec.normal) / r_in.getDirection().length();
                  cosine = r_in.getDirection().dot(rec.normal) / std::sqrt(r_in.getDirection().dot(r_in.getDirection()));
                  cosine = std::sqrt(1 - ref_idx*ref_idx*(1-cosine*cosine));
             }
             else {
                  outward_normal = rec.normal;
                  ni_over_nt = 1.0 / ref_idx;
                  cosine = -r_in.getDirection().dot(rec.normal) / std::sqrt(r_in.getDirection().dot(r_in.getDirection()));
             }
             if (refract(r_in.getDirection(), outward_normal, ni_over_nt, refracted)) 
                reflect_prob = schlick(cosine, ref_idx);
             else 
                reflect_prob = 1.0;
             if (drand48() < reflect_prob) 
                scattered = Ray(rec.p, reflected);
             else 
                scattered = Ray(rec.p, refracted);
             return true;
        }

        double ref_idx;
};

#endif