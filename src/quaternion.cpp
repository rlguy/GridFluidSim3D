#include "quaternion.h"
#include <cmath>

// quaternion identity
Quaternion::Quaternion() {
    x = 0; y = 0; z = 0;
    w = 1;
}

// quaternion from point
Quaternion::Quaternion(glm::vec3 v) {
    x = v.x, y = v.y, z = v.z;
    w = 0.0;
}

// quaternion from unit vector v, and angle rads
Quaternion::Quaternion(glm::vec3 v, float rads) {
    float sinval = sin(0.5*rads);
    x = v.x*sinval; y = v.y*sinval; z = v.z*sinval;
    w = cos(0.5*rads);
}

// quaternion from unit vector <nx, ny, nz>, and angle rads
Quaternion::Quaternion(float nx, float ny, float nz, float rads)
{
    float sinval = sin(0.5*rads);
    x = nx*sinval; y = ny*sinval; z = nz*sinval;
    w = cos(0.5*rads);
}

// multiply (this * q2)
Quaternion Quaternion::mult(Quaternion q2) {
    float s1 = w;
    float s2 = q2.w;
    glm::vec3 v1 = glm::vec3(x, y, z);
    glm::vec3 v2 = glm::vec3(q2.x, q2.y, q2.z);

    float scalar = s1*s2 - glm::dot(v1, v2);
    glm::vec3 vect = s1*v2 + s2*v1 + glm::cross(v1, v2);

    Quaternion newQ = Quaternion();
    newQ.set(vect.x, vect.y, vect.z, scalar);
    return newQ;
}

// multiply (this * scalar)
Quaternion Quaternion::multScalar(float s) {
    Quaternion q = Quaternion();
    q.set(s*x, s*y, s*z, s*w);
    return q;
}

bool Quaternion::isFloatZero(float f) {
    float eps = 0.0000000001;
    return fabs(f) < eps;
}

// inverse of quaternion
Quaternion Quaternion::inverse() {
    Quaternion q = Quaternion();
    float lensqr = x*x + y*y + z*z + w*w;
    if (isFloatZero(lensqr)) {
        return q;
    }
    float inv = 1/lensqr;
    q.set(-inv*x, -inv*y, -inv*z, inv*w);
    return q;
}

float Quaternion::length() {
    return sqrt(x*x + y*y + z*z + w*w);
}

// rotate vector v by this quaternion
glm::vec3 Quaternion::rotateVector(glm::vec3 v) {
    Quaternion vq = Quaternion(v);
    Quaternion r = (this->mult(vq)).mult(this->inverse());

    return glm::vec3(r.x, r.y, r.z);
}

// generate 4d rotation matrix from this quaternion
glm::mat4 Quaternion::getRotationMatrix() {
    return glm::transpose(
           glm::mat4( 1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w, 0.0,
                      2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w, 0.0,
                      2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y, 0.0,
                      0.0, 0.0, 0.0, 1.0 ));
}

// return a unit quaternion from this quaternion
Quaternion Quaternion::normalize() {
    float lensqr = x*x + y*y + z*z + w*w;
    if (isFloatZero(lensqr)) {
        return Quaternion();
    }
    float inv = 1/sqrt(lensqr);
    x *= inv; y *= inv; z *= inv; w *= inv;
    Quaternion q = Quaternion();
    q.set(x, y, z, w);
    return q;
}

// add (this + q2)
Quaternion Quaternion::add(Quaternion q2) {
    float xx = x + q2.x;
    float yy = y + q2.y;
    float zz = z + q2.z;
    float ww = w + q2.w;
    Quaternion q = Quaternion();
    q.set(xx, yy, zz, ww);
    return q;
}

// dot product of (this . q2)
float Quaternion::dot(Quaternion q2) {
    return x*q2.x + y*q2.y + z*q2.z + w*q2.w;
}

// negate each component
Quaternion Quaternion::negate() {
    Quaternion q;
    q.set(-x, -y, -z, -w);
    return q;
}

// Spherical linear interpolation from this to q2 parameterized by t
Quaternion Quaternion::slerp(Quaternion q2, float t) {
    float dot = this->dot(q2);
    if (dot < 0) {
        dot = -1.0*dot;
        q2 = q2.negate();
    }

    float invsin = 1/sin(dot);
    float c1 = invsin * sin((1 - t)*dot);
    float c2 = invsin * sin(t*dot);

    return (this->multScalar(c1)).add(q2.multScalar(c2));
}

// set values of this quaternion manually
void Quaternion::set(float xx, float yy, float zz, float ww) {
    x = xx; y = yy; z = zz; w = ww;
}