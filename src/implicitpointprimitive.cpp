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
#include "implicitpointprimitive.h"

ImplicitPointPrimitive::ImplicitPointPrimitive() {
    _initFieldFunctionCoefficients();
}

ImplicitPointPrimitive::ImplicitPointPrimitive(double x, double y, double z, double r) :
                                               _position(x, y, z), _radius(r)
{
    _initFieldFunctionCoefficients();
}

ImplicitPointPrimitive::ImplicitPointPrimitive(vmath::vec3 p, double r) :
                                               _position(p), _radius(r)
{
    _initFieldFunctionCoefficients();
}

ImplicitPointPrimitive::~ImplicitPointPrimitive()
{
}

void ImplicitPointPrimitive::_initFieldFunctionCoefficients() {
    double r = _radius;
    _coef1 = (4.0 / 9.0)*(1.0 / (r*r*r*r*r*r));
    _coef2 = (17.0 / 9.0)*(1.0 / (r*r*r*r));
    _coef3 = (22.0 / 9.0)*(1.0 / (r*r));
}

double ImplicitPointPrimitive::getFieldValue(vmath::vec3 p) {
    vmath::vec3 v = p - _position;
    double distsq = vmath::dot(v, v);

    if (distsq > _radius*_radius) {
        return 0.0;
    }

    return _evaluateFieldFunction(sqrt(distsq));
}

double ImplicitPointPrimitive::_evaluateFieldFunction(double r) {
    return 1.0 - _coef1*r*r*r*r*r*r + _coef2*r*r*r*r - _coef3*r*r;
}
