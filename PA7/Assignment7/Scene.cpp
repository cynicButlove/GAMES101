//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    // get p
    auto p= intersect(ray);
    if(!p.happened) return Vector3f();
    if(p.m->hasEmission()) return p.m->getEmission();
    Vector3f w0=-ray.direction;
    Intersection lightSample;
    float pdf;
    sampleLight(lightSample,pdf);
    auto Light= lightSample.coords;
    auto N_light=lightSample.normal;
    auto emit=lightSample.emit;
    Ray ray_pToLight=Ray(p.coords,(Light-p.coords).normalized());
    auto q=intersect(ray_pToLight);
    //If the ray is not blocked in the middle
    Vector3f L_dir;
    double dis_p_x=(Light-p.coords).norm();
    if(dis_p_x-q.distance<0.001f) {
        float a=dotProduct(p.normal, ray_pToLight.direction,true);
        float b=dotProduct(N_light,-ray_pToLight.direction,true);
         L_dir= emit*p.m->eval(ray_pToLight.direction,w0,p.normal)
        *a
        *b
        /pow((Light-p.coords).norm(),2)/pdf;
    }

    Vector3f L_indir;
    if(get_random_float()< RussianRoulette) {
        auto wi= p.m->sample(w0,p.normal).normalized();
        auto pdf2= p.m->pdf(w0,wi,p.normal);
        if(pdf2>0.001f) {
            auto q2=  intersect(Ray(p.coords,wi));
            if(q2.happened&&!q2.obj->hasEmit()) {
                L_indir=castRay(Ray(p.coords,wi),depth+1)
                *p.m->eval(wi,w0,p.normal)
                *dotProduct(wi,p.normal,true)
                /pdf2
                /RussianRoulette;
            }
        }

    }

    return L_dir+L_indir;
}