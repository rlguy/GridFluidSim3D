#pragma once

#include <stdio.h>
#include <iostream>
#include <vector>
#include <assert.h>

#include "implicitpointprimitive.h"
#include "glm/glm.hpp"

struct ImplicitPointData {
    glm::vec3 position = glm::vec3(0.0, 0.0, 0.0);
    double radius = 0.0;
};

class ImplicitField
{
public:
    ImplicitField();
    ImplicitField(double width, double height, double depth);
    ~ImplicitField();

    void addPoint(double x, double y, double z, double r) { addPoint(glm::vec3(x, y, z), r); };
    void addPoint(glm::vec3 p, double r);

    void getDimensions(double *w, double *h, double *d) { *w = width; *h = height; *d = depth; }
    double getWidth()  { return width; }
    double getHeight() { return height; }
    double getDepth()  { return depth; }

    double getSurfaceThreshold() { return surfaceThreshold; }
    double getRicciBlend() { return ricciBlend; }
    int getNumPoints() { return (int)points.size(); }

    void setSurfaceThreshold(double t) { surfaceThreshold = t; }
    void setRicciBlend(double k) { assert(k > 0.0); ricciBlend = k; }

    void clear();

    double getFieldValue(double x, double y, double z) { getFieldValue(glm::vec3(x, y, z)); }
    double getFieldValue(glm::vec3 p);
    bool isInside(double x, double y, double z) { return isInside(glm::vec3(x, y, z)); }
    bool isInside(glm::vec3 p);
    bool isOutside(double x, double y, double z) { return isOutside(glm::vec3(x, y, z)); }
    bool isOutside(glm::vec3 p);

    std::vector<ImplicitPointData> getImplicitPointData();

    double width = 0;
    double height = 0;
    double depth = 0;

private:
    std::vector<ImplicitPointPrimitive> points;

    double surfaceThreshold = 0.5;
    double ricciBlend = 1.0;

};

