#include <iostream>
#include "../include/material.h"
#include "../include/objects.h"
#include "../include/write_ppm.h"
#include "../include/Camera.h"
#include "../include/hit_record.h"
#include "../include/color.h"
#include "../include/load_obj.h"
#include <Eigen/Core>

#include <vector>
#include <stdlib.h>
using namespace std;

#define R_FILENAME "image.ppm"

//raytrace recursion iterations
#define R_MAX_REC_RAY 50
//scene
#define R_WIDTH 400
#define R_HEIGHT 300
#define R_S_RANDOM 10

//camera
#define R_LOOK_FROM Eigen::Vector3d(0,5,10)
#define R_LOOK_AT Eigen::Vector3d(0,0,0)
#define R_DIST_TO_FOCUS 10.0
#define R_APERTURE 0.1

int main(int argc, char ** argv){
    vector<Eigen::Vector3d> cube;
    load_obj("models/cube.obj",cube);
    //set up scene
    Scene scene;
    Eigen::Vector3d look_at = R_LOOK_AT;
    Eigen::Vector3d look_from(0.0,0.0,0.0);
    Camera camera(R_LOOK_FROM, R_LOOK_AT, Eigen::Vector3d(0,1,0), 20, double(R_WIDTH) / double(R_HEIGHT), R_APERTURE, R_DIST_TO_FOCUS);
   
    scene.AddObject(new Sphere(Eigen::Vector3d(0,1,0), 1,new metal(Eigen::Vector3d(1.0, 1.0, 1.0), 0.2) ));
    scene.AddObject(new Sphere(Eigen::Vector3d(-1.0,0.5,2), 0.4,new metal(Eigen::Vector3d(1.0, 1.0, 1.0), 0.0) ));
    scene.AddObject(new Sphere(Eigen::Vector3d(1.5,0.5,0), 0.5, new lambertian(Eigen::Vector3d(1.0, 0.2, 0.5))));
    scene.AddObject(new Sphere(Eigen::Vector3d(0,0.5,2), 0.5, new lambertian(Eigen::Vector3d(0.0, 1.0, 0.0))));
     
    scene.AddObject(new PlaneBound(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,-1,0),Eigen::Vector3d(-2,-2, -2),Eigen::Vector3d(2,2,2),new lambertian(Eigen::Vector3d(0.5, 0.2, 0.7)))); 
    //scene.AddObject(new Plane(Eigen::Vector3d(0.0,0.0,0.0),Eigen::Vector3d(0,1,0), new lambertian(Eigen::Vector3d(0.5, 0.2, 0.7))));
    //scene.AddObject(new Triangle(Eigen::Vector3d(10.0,0.0,10.0),Eigen::Vector3d(0.0,0.0,-10.0),Eigen::Vector3d(-10.0,0.0,10.0),new lambertian(Eigen::Vector3d(0.5, 0.2, 0.7))));


    double max_percent = R_HEIGHT * R_WIDTH;
    std::vector<unsigned char> image;
    for(int j = R_HEIGHT - 1; j >= 0; j--){
        for(int i = 0; i < R_WIDTH;i++){
            Eigen::Vector3d color(0.0,0.0,0.0);
            for(int s = 0; s < R_S_RANDOM; s++){
                double u = double(i + drand48()) / double(R_WIDTH);
                double v = double(j + drand48()) / double(R_HEIGHT);
                Ray ray = camera.generateRay(u,v);
                Eigen::Vector3d p = ray.getPointAt(2.0);
                color += raytrace_color(ray, scene, 0,R_MAX_REC_RAY);
            }
            color = (color.array() / double(R_S_RANDOM)).matrix();
            color =  Eigen::Vector3d(sqrt(color.x()),sqrt(color.y()),sqrt(color.z()));
            int r = int(255.99 * color.x());
            int g = int(255.99 * color.y());
            int b = int(255.99 * color.z());
            image.push_back(r);
            image.push_back(g);
            image.push_back(b);
        }
        //cout << "Progress: " << (double(R_HEIGHT - j) / R_HEIGHT ) * 100.0<< endl;
    }
    write_ppm(R_FILENAME,image,R_WIDTH,R_HEIGHT,3);
}