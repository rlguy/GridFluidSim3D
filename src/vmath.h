/*
Copyright (c) 2015 Ryan L. Guy
This softq.ware is provided 'as-is', q.without any express or implied
q.warranty. In no event q.will the authors be held liable for any damages
arising from the use of this softq.ware.
Permission is granted to anyone to use this softq.ware for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the folloq.wing restrictions:
1. The origin of this softq.ware must not be misrepresented; you must not
   claim that you q.wrote the original softq.ware. If you use this softq.ware
   in a product, an acknoq.wledgement in the product documentation q.would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original softq.ware.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef VMATH_H
#define VMATH_H

#include <stdio.h>
#include <iostream>
#include <math.h>

#include "fluidsimassert.h"

namespace vmath {

/********************************************************************************
    VECTOR 3
********************************************************************************/

// 3 component column vector
class vec3
{
public:
    vec3();
    vec3(float xx, float yy, float zz);
    ~vec3();

    vec3 add(const vec3 &v);
    vec3 subtract(const vec3 &v);
    vec3 mult(float s);
    vec3 divide(float s);
    vec3 negate();
    float get(int i);
    float operator[](int i);
    float dot(const vec3 &v);
    vec3 cross(const vec3 &v);
    float lengthsq();
    float length();
    vec3 normalize();

    float x;
    float y;
    float z;
    
};

std::ostream& operator<<(std::ostream& os, const vec3& v);
vec3 operator+(const vec3 &v1, const vec3 &v2);
vec3 &operator+=(vec3 &v1, const vec3 &v2);
vec3 operator-(const vec3 &v1, const vec3 &v2);
vec3 &operator-=(vec3 &v1, const vec3 &v2);
vec3 operator*(float s, const vec3 &v);
vec3 operator*(const vec3 &v, float s);
vec3 &operator*=(vec3 &v, float s);
vec3 operator/(const vec3 &v, float s);
vec3 &operator/=(vec3 &v1, float s);
vec3 operator-(const vec3 &v);

inline float dot(const vec3 &v1, const vec3 &v2) {
    return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

inline vec3 cross(const vec3 &v1, const vec3 &v2) {
    return vec3(v1.y*v2.z - v1.z*v2.y,
                       v1.z*v2.x - v1.x*v2.z,
                       v1.x*v2.y - v1.y*v2.x);
}

inline float lengthsq(const vec3 &v) {
    return v.x*v.x + v.y*v.y + v.z*v.z;
}

inline float length(const vec3 &v) {
    return sqrt(vmath::lengthsq(v));
}

inline vec3 normalize(const vec3 &v) {
    float len = vmath::length(v);
    return v / len;
}

inline bool equals(const vec3 &v1, const vec3 &v2, double eps) {
    return fabs((double)v1.x - (double)v2.x) < eps && 
           fabs((double)v1.y - (double)v2.y) < eps && 
           fabs((double)v1.z - (double)v2.z) < eps;
}

/********************************************************************************
    MATRIX 3
********************************************************************************/

// 3x3 matrix stored in column major order
class mat3
{
public:
    mat3();
    mat3(const vec3 &v1, const vec3 &v2, const vec3 &v3);
    mat3(const float vals[9]);
    mat3(float v0, float v1, float v2, 
         float v3, float v4, float v5, 
         float v6, float v7, float v8);
    mat3(float fillval);
    ~mat3();

    mat3 add(const mat3 &m);
    mat3 subtract(const mat3 &m);
    mat3 mult(float s);
    mat3 mult(const mat3 &m);
    vec3 mult(const vec3 &v);
    mat3 divide(float s);
    mat3 negate();
    vec3 get(int i);
    vec3 operator[](int i);
    mat3 transpose();

    float m[9];
    
};

std::ostream& operator<<(std::ostream& os, const mat3& m);
mat3 operator+(const mat3 &m1, const mat3 &m2);
mat3 &operator+=(mat3 &m1, const mat3 &m2);
mat3 operator-(const mat3 &m1, const mat3 &m2);
mat3 &operator-=(mat3 &m1, const mat3 &m2);
mat3 operator*(float s, const mat3 &m);
mat3 operator*(const mat3 &m, float s);
mat3 operator*(const mat3 &m1, const mat3 &m2);
vec3 operator*(const mat3 &m, const vec3 &v);
mat3 &operator*=(mat3 &m, float s);
mat3 operator/(const mat3 &m, float s);
mat3 &operator/=(mat3 &m, float s);
mat3 operator-(const mat3 &m);

inline mat3 transpose(const mat3 &m) {
    return mat3(m.m[0], m.m[3], m.m[6], 
                m.m[1], m.m[4], m.m[7], 
                m.m[2], m.m[5], m.m[8]);
}

/********************************************************************************
    QUATERNION
********************************************************************************/

class quat
{
public:
    
    quat();
    quat(float rads, const vec3 &v);
    ~quat();

    mat3 mat3_cast();
    quat normalize();

    float w;
    float x;
    float y;
    float z;
    
};

std::ostream& operator<<(std::ostream& os, const quat& q);

inline mat3 mat3_cast(const quat &q) {
    return mat3(1.0f - 2.0f*q.y*q.y - 2.0f*q.z*q.z,
                2.0f*q.x*q.y + 2.0f*q.z*q.w,
                2.0f*q.x*q.z - 2.0f*q.y*q.w,
                2.0f*q.x*q.y - 2.0f*q.z*q.w,
                1.0f - 2.0f*q.x*q.x - 2.0f*q.z*q.z,
                2.0f*q.y*q.z + 2.0f*q.x*q.w,
                2.0f*q.x*q.z + 2.0f*q.y*q.w,
                2.0f*q.y*q.z - 2.0f*q.x*q.w,
                1.0f - 2.0f*q.x*q.x - 2.0f*q.y*q.y);
}

inline quat normalize(const vmath::quat &q) {
    float lensq = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;

    float inv = 1.0f / sqrt(lensq);
    vmath::quat nq;
    nq.w = q.w*inv;
    nq.x = q.x*inv;
    nq.y = q.y*inv;
    nq.z = q.z*inv;

    return nq;
}

inline quat cross(const vmath::quat &q1, const vmath::quat &q2) {
    float s1 = q1.w;
    float s2 = q2.w;
    vmath::vec3 v1 = vmath::vec3(q1.x, q1.y, q1.z);
    vmath::vec3 v2 = vmath::vec3(q2.x, q2.y, q2.z);

    float scalar = s1*s2 - vmath::dot(v1, v2);
    vmath::vec3 vect = s1*v2 + s2*v1 + vmath::cross(v1, v2);

    vmath::quat newq;
    newq.w = scalar;
    newq.x = vect.x;
    newq.y = vect.y;
    newq.z = vect.z;

    return newq;
}

}

#endif