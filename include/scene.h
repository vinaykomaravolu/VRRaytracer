#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <Eigen/core>
#include "objects.h"
#include "Ray.h"

class Scene{
    public:
        Scene(){
            size = 0;
        }

        void AddObject(Object * obj){
            objects.push_back(obj);
            size++;
        }

        int getSize(){
            return size;
        }

        std::vector<Object * > getObjects(){
            return objects;
        }

        bool hit_object(Ray & ray, double tmin, double tmax, HitRecord & rec){
            for(int object = 0; object < objects.size(); object++){
                if(objects[object]->hit(ray,tmin,tmax,rec)){
                    return true;
                }
            }
            return false;
        }

        bool hit_closest(Ray & ray, double tmin, double tmax, HitRecord & rec){
            HitRecord closest;
            bool hit = false;
            double closest_value = tmax;
            for(int object = 0; object < objects.size(); object++){
                if(objects[object]->hit(ray,tmin,closest_value,closest)){
                    hit = true;
                    closest_value = closest.t;
                    rec = closest;
                }
            }
            return hit;
        }

    private:
        int size;
        std::vector<Object * > objects;
};

#endif