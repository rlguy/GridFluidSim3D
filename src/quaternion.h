#ifndef QUATERNION_H
#define QUATERNION_H

#include "vmath.h"

class Quaternion
{
public:
    Quaternion();
    Quaternion(float nx, float ny, float nz, float rads);
    Quaternion(vmath::vec3 v);
    Quaternion(vmath::vec3 v, float rads);

    void set(float x, float y, float z, float w);
    float length();
    void print();

    Quaternion mult(Quaternion q);
    Quaternion multScalar(float s);
    Quaternion inverse();
    vmath::vec3 rotateVector(vmath::vec3 v);
    vmath::mat4 getRotationMatrix();
    float dot(Quaternion q2);
    Quaternion normalize();
    Quaternion add(Quaternion q2);
    Quaternion negate();
    Quaternion slerp(Quaternion q2, float t);

    float x, y, z;      // vector part
    float w;            // scalar part

private:
    bool isFloatZero(float f);

};

#endif // QUATERNION_H