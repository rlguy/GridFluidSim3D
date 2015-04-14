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

    void setPosition(double x, double y, double z) { position = glm::vec3(x, y, z); }
    void setPosition(glm::vec3 p) { position = p; }
    void setRadius(double r) { radius = r; _initFieldFunctionCoefficients(); }
    void translate(double tx, double ty, double tz) { position += glm::vec3(tx, ty, tz); }
    void translate(glm::vec3 t) { position += t; }

    glm::vec3 getPosition() { return position; }
    void getPosition(double *x, double *y, double *z) { *x = position.x;
                                                        *y = position.y;
                                                        *z = position.z; }
    double getRadius() { return radius; }

    double getFieldValue(double x, double y, double z) { return getFieldValue(glm::vec3(x, y, z)); };
    double getFieldValue(glm::vec3 p);

private:
    void _initFieldFunctionCoefficients();
    double _evaluateFieldFunction(double r);

    glm::vec3 position;
    double radius = 1;

    double _coef1;
    double _coef2;
    double _coef3;

};

