/*
Copyright (c) 2016 Ryan L. Guy

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgement in the product documentation would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef IMPLICITPOINTPRIMITIVE_H
#define IMPLICITPOINTPRIMITIVE_H

#include <stdio.h>
#include <iostream>

#include "vmath.h"

class ImplicitPointPrimitive
{
public:
    ImplicitPointPrimitive();
    ImplicitPointPrimitive(double x, double y, double z, double r);
    ImplicitPointPrimitive(vmath::vec3 p, double r);
    ~ImplicitPointPrimitive();

    void setPosition(double x, double y, double z) { _position = vmath::vec3(x, y, z); }
    void setPosition(vmath::vec3 p) { _position = p; }
    void setRadius(double r) { _radius = r; _initFieldFunctionCoefficients(); }
    void translate(double tx, double ty, double tz) { _position += vmath::vec3(tx, ty, tz); }
    void translate(vmath::vec3 t) { _position += t; }

    vmath::vec3 getPosition() { return _position; }
    void getPosition(double *x, double *y, double *z) { *x = _position.x;
                                                        *y = _position.y;
                                                        *z = _position.z; }
    double getRadius() { return _radius; }

    double getFieldValue(double x, double y, double z) { return getFieldValue(vmath::vec3(x, y, z)); };
    double getFieldValue(vmath::vec3 p);

private:
    void _initFieldFunctionCoefficients();
    double _evaluateFieldFunction(double r);

    vmath::vec3 _position;
    double _radius = 1;

    double _coef1;
    double _coef2;
    double _coef3;

};

#endif
