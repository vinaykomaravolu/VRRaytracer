#ifndef OBJECTS_H
#define OBJECTS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <cmath>
#include "Ray.h"
#include "hit_record.h"
#include <vector>

#ifndef Material_H
#define Material_H
#include "material.h"
#endif

class Object
{
  public:
    virtual bool hit(Ray &ray, double tmin, double tmax, HitRecord &rec)
    {
    }
};

class PlaneBound : public Object
{
  public:
    Eigen::Vector3d point;
    Eigen::Vector3d normal;
    Eigen::Vector3d minP;
    Eigen::Vector3d maxP;
    Material *mat_ptr;

    PlaneBound(Eigen::Vector3d p, Eigen::Vector3d n, Eigen::Vector3d min, Eigen::Vector3d max, Material *m)
    {
        minP = min;
        maxP = max;
        point = p;
        normal = n;
        mat_ptr = m;
    }

    bool hit(Ray &ray, double tmin, double tmax, HitRecord &rec)
    {
        double d = normal.normalized().dot(ray.getDirection());
        if (std::abs(d) > 0.0001)
        {

            double t = (point - ray.getOrigin()).dot(normal.normalized()) / double(d);
            if (t >= tmin && t < tmax)
            {
                Eigen::Vector3d hit = ray.getPointAt(t);
                if (hit.x() <= maxP.x() && hit.y() <= maxP.y() && hit.z() <= maxP.z() && hit.x() >= minP.x() && hit.y() >= minP.y() && hit.z() >= minP.z())
                {

                    rec.t = t;
                    rec.p = hit;
                    rec.normal = normal.normalized();

                    // To fix normals of both sides
                    if (-1 * ray.getDirection().normalized().dot(rec.normal) < 0)
                    {
                        rec.normal = -1 * rec.normal;
                    }
                    rec.mat_ptr = mat_ptr;
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }
        return false;
    }
};

class Triangle : public Object
{
  public:
    Eigen::Vector3d A;
    Eigen::Vector3d B;
    Eigen::Vector3d C;
    Material *mat_ptr;

    Triangle(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c, Material *mat)
    {
        A = a;
        B = b;
        C = c;
        mat_ptr = mat;
    }

    bool hit(Ray &ray, double tmin, double tmax, HitRecord &rec)
    {
        Eigen::Vector3d n1 = (B - A).cross(C - A);
        double a = A.x() - B.x();
        double b = A.y() - B.y();
        double c = A.z() - B.z();
        double d = A.x() - C.x();
        double e = A.y() - C.y();
        double f = A.z() - C.z();
        double g = ray.getDirection().x();
        double h = ray.getDirection().y();
        double i = ray.getDirection().z();
        double j = A.x() - ray.getOrigin().x();
        double k = A.y() - ray.getOrigin().y();
        double l = A.z() - ray.getOrigin().z();
        double M = (a * ((e * i) - (h * f))) + (b * ((g * f) - (d * i))) + (c * ((d * h) - (e * g)));
        double beta = ((j * ((e * i) - (h * f))) + (k * ((g * f) - (d * i))) + (l * ((d * h) - (e * g)))) / M;
        double gamma = ((i * ((a * k) - (j * b))) + (h * ((j * c) - (a * l))) + (g * ((b * l) - (k * c)))) / M;
        double t1 = -((f * ((a * k) - (j * b))) + (e * ((j * c) - (a * l))) + (d * ((b * l) - (k * c)))) / M;
        //std::cout << "M: " << M << " B: " << beta << " G: " << gamma << " t: " << t1 << std::endl;
        if (tmin <= t1 && t1 < tmax && gamma > 0 && beta > 0 && beta + gamma < 1)
        {
            rec.t = t1;
            rec.normal = n1.normalized();

            //to fix different facing triangles
            if (-1 * ray.getDirection().normalized().dot(rec.normal) < 0)
            {
                rec.normal = -1 * rec.normal;
            }
            rec.mat_ptr = mat_ptr;
            rec.p = ray.getPointAt(rec.t);
            return true;
        }
        return false;
    }
};

class Plane : public Object
{
  public:
    Eigen::Vector3d point;
    Eigen::Vector3d normal;
    Material *mat_ptr;

    Plane(Eigen::Vector3d p, Eigen::Vector3d n, Material *m)
    {
        point = p;
        normal = n;
        mat_ptr = m;
    }

    bool hit(Ray &ray, double tmin, double tmax, HitRecord &rec)
    {
        double d = normal.normalized().dot(ray.getDirection());
        if (std::abs(d) > 0.0001)
        {

            double t = (point - ray.getOrigin()).dot(normal.normalized()) / double(d);
            if (t >= tmin && t < tmax)
            {
                rec.t = t;
                rec.p = ray.getPointAt(rec.t);
                rec.normal = normal.normalized();
                // To fix normals of both sides
                if (-1 * ray.getDirection().normalized().dot(rec.normal) < 0)
                {
                    rec.normal = -1 * rec.normal;
                }
                rec.mat_ptr = mat_ptr;
                return true;
            }
        }
        return false;
    }
};

class Sphere : public Object
{
  public:
    Eigen::Vector3d center;
    double radius;
    Material *mat_ptr;

    Sphere(Eigen::Vector3d cen, double rad, Material *m)
    {
        center = cen;
        radius = rad;
        mat_ptr = m;
    }

    bool hit(Ray &ray, double tmin, double tmax, HitRecord &rec)
    {
        double A = ray.getDirection().dot(ray.getDirection());
        double B = ray.getDirection().dot(ray.getOrigin() - center);
        double C = (ray.getOrigin() - center).dot(ray.getOrigin() - center) - std::pow(radius, 2.0);
        double discr = std::pow(B, 2.0) - A * C;

        //Checking the discriminant to see if there are solutions/intersection points
        if (discr < 0)
        {
            return false;
        }
        else if (discr > 0)
        {
            double t1 = (-B + std::sqrt(discr)) / A;
            double t2 = (-B - std::sqrt(discr)) / A;
            if (tmin <= t1 && tmin <= t2)
            {
                rec.t = fmin(t1, t2);
                rec.normal = (2 * ((ray.getOrigin() + rec.t * ray.getDirection()) - center)).normalized();
                rec.p = ray.getPointAt(rec.t);
                rec.mat_ptr = mat_ptr;
            }
            else if (tmin > t1 && tmin <= t2)
            {
                rec.t = t2;
                rec.normal = (2 * ((ray.getOrigin() + rec.t * ray.getDirection()) - center)).normalized();
                rec.p = ray.getPointAt(rec.t);
                rec.mat_ptr = mat_ptr;
            }
            else if (tmin <= t1 && tmin > t2)
            {
                rec.t = t1;
                rec.normal = (2 * ((ray.getOrigin() + rec.t * ray.getDirection()) - center)).normalized();
                rec.p = ray.getPointAt(rec.t);
                rec.mat_ptr = mat_ptr;
            }
            else
            {
                return false;
            }
            return true;
        }
        else
        {
            double t1 = -B / A;
            if (tmin <= t1)
            {
                rec.t = t1;
                rec.normal = (2 * ((ray.getOrigin() + rec.t * ray.getDirection()) - center)).normalized();
                rec.p = ray.getPointAt(rec.t);
                rec.mat_ptr = mat_ptr;
                return true;
            }
            return false;
        }
    }
};

class Mesh : public Object
{
  public:
    vector<Triangle> faces;

    Mesh(Eigen::Vector3d translate, Eigen::Vector3d rotate, Eigen::Vector3d scale, vector<Eigen::Vector3d> ind, Material *mat)
    {

        Eigen::Affine3d rx =
            Eigen::Affine3d(Eigen::AngleAxisd(rotate.x() * M_PI / 180.0, Eigen::Vector3d(1, 0, 0)));
        Eigen::Affine3d ry =
            Eigen::Affine3d(Eigen::AngleAxisd(rotate.y() * M_PI / 180.0, Eigen::Vector3d(0, 1, 0)));
        Eigen::Affine3d rz =
            Eigen::Affine3d(Eigen::AngleAxisd(rotate.z() * M_PI / 180.0, Eigen::Vector3d(0, 0, 1)));
        Eigen::Affine3d rot = rz * ry * rx;
        Eigen::Affine3d tran = Eigen::Affine3d(Eigen::Translation3d(translate));
        Eigen::Affine3d sca(Eigen::Scaling(scale));
        Eigen::Affine3d modelMatrix = tran * rot * sca;
        for (int i = 0; i < ind.size(); i += 3)
        {
            Eigen::Vector3d p1 = modelMatrix * ind[i];
            Eigen::Vector3d p2 = modelMatrix * ind[i + 1];
            Eigen::Vector3d p3 = modelMatrix * ind[i + 2];
            Triangle new_face(p1, p2, p3, mat);
            faces.push_back(new_face);
        }
    }

    bool hit(Ray &ray, double tmin, double tmax, HitRecord &rec)
    {
        HitRecord closest;
        bool hit = false;
        double closest_value = tmax;
        for(int object = 0; object < faces.size(); object++){
            if(faces[object].hit(ray,tmin,closest_value,closest)){
                hit = true;
                closest_value = closest.t;
                rec = closest;
            }
        }
        return hit;
    }
};

#endif