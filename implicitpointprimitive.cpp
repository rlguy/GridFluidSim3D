#include "implicitpointprimitive.h"


ImplicitPointPrimitive::ImplicitPointPrimitive() : _position(glm::vec3(0.0, 0.0, 0.0))
{
    _initFieldFunctionCoefficients();
}

ImplicitPointPrimitive::ImplicitPointPrimitive(double x, double y, double z, double r) :
                                               _position(glm::vec3(x, y, z)), _radius(r)
{
    _initFieldFunctionCoefficients();
}

ImplicitPointPrimitive::ImplicitPointPrimitive(glm::vec3 p, double r) :
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

double ImplicitPointPrimitive::getFieldValue(glm::vec3 p) {
    glm::vec3 v = p - _position;
    double distsq = glm::dot(v, v);

    if (distsq > _radius*_radius) {
        return 0.0;
    }

    return _evaluateFieldFunction(sqrt(distsq));
}

double ImplicitPointPrimitive::_evaluateFieldFunction(double r) {
    return 1.0 - _coef1*r*r*r*r*r*r + _coef2*r*r*r*r - _coef3*r*r;
}
