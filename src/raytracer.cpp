#include <iostream>
#include "../include/material.h"
#include "../include/objects.h"
#include "../include/write_ppm.h"
#include "../include/Camera.h"
#include "../include/hit_record.h"
#include "../include/color.h"
#include "../include/load_obj.h"
#include <Eigen/Core>
#include <chrono>
#include <vector>
#include <stdlib.h>
using namespace std;

#define R_FILENAME "image.ppm"
//raytrace recursion iterations
#define R_MAX_REC_RAY 50
#define R_WIDTH 1980
#define R_HEIGHT 1080
#define R_S_RANDOM 50
//camera
#define R_LOOK_FROM Eigen::Vector3d(0,3,10)
#define R_LOOK_AT Eigen::Vector3d(0,0,0)
#define R_DIST_TO_FOCUS 10.0
#define R_APERTURE 0.1

int main(int argc, char ** argv){
    // Model loading Due
    vector<Eigen::Vector3d> cube;
    load_obj("models/cube.obj",cube);

    //set up scene
    Scene scene;
    Camera camera(R_LOOK_FROM, R_LOOK_AT, Eigen::Vector3d(0,1,0), 20, double(R_WIDTH) / double(R_HEIGHT), R_APERTURE, R_DIST_TO_FOCUS);
   
    // Add objects to the scene
    // Custom meshes WARNING: LONG LOAD TIMES
    scene.AddObject(new Mesh(Eigen::Vector3d(-1,0.0,0),Eigen::Vector3d(0,45,0),Eigen::Vector3d(0.5,0.5,0.5), cube,new metal(Eigen::Vector3d(1.0, 1.0, 1.0),0.0)));
    scene.AddObject(new Mesh(Eigen::Vector3d(1,0.0,0),Eigen::Vector3d(0,-45,0),Eigen::Vector3d(0.5,0.5,0.5), cube,new metal(Eigen::Vector3d(1.0, 0.0, 0.0),0.0)));
    scene.AddObject(new Mesh(Eigen::Vector3d(0,1.0,0),Eigen::Vector3d(0,-45,0),Eigen::Vector3d(0.5,0.5,0.5), cube,new metal(Eigen::Vector3d(1.0, 1.0, 1.0),0.5)));
    //Spheres
    scene.AddObject(new Sphere(Eigen::Vector3d(0,-0.1,0.4), 0.4,new lambertian(Eigen::Vector3d(0.5, 0.2, 0.7))));
    scene.AddObject(new Sphere(Eigen::Vector3d(-1.5,-0.2,0.9), 0.2,new lambertian(Eigen::Vector3d(1.0, 0.5, 0.0))));
    scene.AddObject(new Sphere(Eigen::Vector3d(1.5,-0.2,0.9), 0.2,new lambertian(Eigen::Vector3d(1.0, 0.5, 0.0))));
    // Plane Bounds
    scene.AddObject(new PlaneBound(Eigen::Vector3d(0,-0.5,0),Eigen::Vector3d(0,1,0),Eigen::Vector3d(-2,-2, -2),Eigen::Vector3d(2,2,2), new lambertian(Eigen::Vector3d(0.0, 0.3 ,0.3)))); 
    // Planes
    scene.AddObject(new Plane(Eigen::Vector3d(0.0,-0.7,0.0),Eigen::Vector3d(0,1,0), new lambertian(Eigen::Vector3d(0.0, 0.2, 0.7))));
    // Triangles

    //render scene
    double max_percent = R_HEIGHT * R_WIDTH;
    std::vector<unsigned char> image;
    for(int j = R_HEIGHT - 1; j >= 0; j--){
        auto start = std::chrono::high_resolution_clock::now();
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
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;        
        cout << "Progress: " << (double(R_HEIGHT - j) / R_HEIGHT ) * 100.0<< " Time Estimate(S): "  << (elapsed.count() * R_HEIGHT) / 60 / 60 <<endl;
    }
    // Write to file
    write_ppm(R_FILENAME,image,R_WIDTH,R_HEIGHT,3);
}