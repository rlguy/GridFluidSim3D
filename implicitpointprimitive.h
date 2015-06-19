#pragma once

#include <stdio.h>
#include <iostream>

#include <assert.h>
#include "glm/glm.hpp"


class ImplicitPointPrimitive
{
public:
    ImplicitPointPrimitive();
    ImplicitPointPrimitive(double x, double y, double z, double r);
    ImplicitPointPrimitive(glm::vec3 p, double r);
    ~ImplicitPointPrimitive();

    void setPosition(double x, double y, double z) { _position = glm::vec3(x, y, z); }
    void setPosition(glm::vec3 p) { _position = p; }
    void setRadius(double r) { _radius = r; _initFieldFunctionCoefficients(); }
    void translate(double tx, double ty, double tz) { _position += glm::vec3(tx, ty, tz); }
    void translate(glm::vec3 t) { _position += t; }

    glm::vec3 getPosition() { return _position; }
    void getPosition(double *x, double *y, double *z) { *x = _position.x;
                                                        *y = _position.y;
                                                        *z = _position.z; }
    double getRadius() { return _radius; }

    double getFieldValue(double x, double y, double z) { return getFieldValue(glm::vec3(x, y, z)); };
    double getFieldValue(glm::vec3 p);

private:
    void _initFieldFunctionCoefficients();
    double _evaluateFieldFunction(double r);

    glm::vec3 _position;
    double _radius = 1;

    double _coef1;
    double _coef2;
    double _coef3;

};

