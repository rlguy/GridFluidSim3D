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
    ImplicitField(int i_width, int j_height, int k_depth, double cell_size);
    ~ImplicitField();

    void addPoint(double x, double y, double z, double r) { addPoint(glm::vec3(x, y, z), r); };
    void addPoint(glm::vec3 p, double r);
    void addCuboid(double x, double y, double z, double w, double h, double d) {
        addCuboid(glm::vec3(x, y, z), w, h, d);
    }
    void addCuboid(glm::vec3 p, double width, double height, double depth);

    void getDimensions(double *w, double *h, double *d) { *w = width; *h = height; *d = depth; }
    double getWidth()  { return width; }
    double getHeight() { return height; }
    double getDepth()  { return depth; }

    double getSurfaceThreshold() { return surfaceThreshold; }
    double getRicciBlend() { return ricciBlend; }
    int getNumPoints() { return (int)points.size(); }
    int getNumCuboids() { return (int)cuboids.size(); }

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

    double width, height, depth;
private:

    // cuboid has a field value of (surfaceThreshold + epsilon) within
    // its volume, 0 outside
    struct Cuboid {
        glm::vec3 position;
        double width, height, depth;

        Cuboid() : position(glm::vec3(0.0, 0.0, 0.0)),
                   width(0.0), height(0.0), depth(0.0) {}

        Cuboid(glm::vec3 p, double w, double h, double d) :
               position(p), width(w), height(h), depth(d) {}
    };

    bool _isPointInsideCuboid(glm::vec3 p, Cuboid c);

    int i_width, j_height, k_depth;
    double dx = 1.0;

    std::vector<ImplicitPointPrimitive> points;
    std::vector<Cuboid> cuboids;

    double surfaceThreshold = 0.5;
    double ricciBlend = 1.0;

};

