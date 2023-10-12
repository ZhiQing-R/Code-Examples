#pragma once

#include "la.h"

struct Ray
{
    glm::vec3 o;
    glm::vec3 d;
};

// ref: https://iquilezles.org/articles/intersectors/
static float raySphereIntersect(const Ray& ray, const glm::vec3& center, float r = 0.07)
{
    glm::vec3 oc = ray.o - center;
    // oc project on ray
    float ocProj = glm::dot(oc, ray.d);
    // min distance square to sphere surface
    float minDist2 = glm::dot(oc, oc) - r*r;
    float diff = ocProj*ocProj - minDist2;
    if(diff < 0.f) return FLT_MAX;
    return ocProj - sqrt(diff);
}

// ref: https://iquilezles.org/articles/intersectors/
// https://hugi.scene.org/online/hugi24/coding%20graphics%20chris%20dragan%20raytracing%20shapes.htm
static float rayCylinderIntersect(const Ray& ray, const glm::vec3& a,
                                  const glm::vec3& b, float r = 0.05)
{
    // this V is actually V * |ba|
    // so later we will scale ABC to correct the equation
    glm::vec3 V = b  - a;
    glm::vec3 X = ray.o - a;
    float VV = glm::dot(V, V);
    float VD = glm::dot(V, ray.d);
    float VX = glm::dot(V, X);

    // use A, B, C to solve At^2 + Bt + C = 0;
    float A = VV - VD * VD;
    // here B is actually B/2
    float B = VV * glm::dot(X, ray.d) - VX * VD;
    float C = VV * glm::dot(X, X) - VX * VX - r*r * VV;

    // this is (b^2 - 4ac)/2
    float delta = B*B - A*C;
    // no solusion
    if(delta < 0.f) return FLT_MAX;

    // calculate near t using (-b - sqrt(b^2 - 4ac)) / 2a
    // remember delta is actually sqrt(b^2 - 4ac) / 2
    delta = sqrt(delta);
    float t = (-B - delta) / A;
    // body intersection test
    // check dot((o + td - a), V) < VV
    // this equals dot( X + td, V) < VV
    // so y is |ab| * P project on ba
    float y = VX + t * VD;
    if(y > 0.f && y < VV) return t;

    // caps intersection test
    // two planes are (a, -V/|V|), (a + V, V/|V|)
    // plane intersection: t = -X*V / D*V
    t = (( (y<0.0) ? 0.0 : VV) - VX) / VD;
    // t should between t1, t2
    // i.e. it should inside an infinite cylinder
    // (-B - delta) / A < t < (-B + delta) / A
    if(abs(B + A*t) < delta)
        return t;
    return FLT_MAX;
}


// moller trumbore algorithm
// https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection.html
static float rayTriangleIntersection(const Ray& ray, const glm::vec3& a,
                                     const glm::vec3& b, const glm::vec3& c)
{
    glm::vec3 e1 = b - a;
    glm::vec3 e2 = c - a;
    glm::vec3 pvec = glm::cross(ray.d, e2);
    float det = glm::dot(pvec, e1);

    if (abs(det) < FLT_EPSILON)
        return FLT_MAX;

    float invDet = 1.0f / det;
    glm::vec3 tvec = ray.o - a;
    float u = invDet * glm::dot(tvec, pvec);
    if (u < 0.0f || u > 1.0f)
        return FLT_MAX;

    glm::vec3 qvec = glm::cross(tvec, e1);
    float v = invDet * glm::dot(qvec, ray.d);
    if (v < 0.0f || u + v > 1.0f)
        return FLT_MAX;

    return glm::dot(e2, qvec) * invDet;
}
