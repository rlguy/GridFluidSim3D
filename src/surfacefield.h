/*
Copyright (c) 2015 Ryan L. Guy

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
#ifndef SURFACEFIELD_H
#define SURFACEFIELD_H

#include <stdio.h>
#include <iostream>
#include <vector>
#include <assert.h>

#include "implicitpointprimitive.h"
#include "array3d.h"
#include "grid3d.h"
#include "vmath.h"


class SurfaceField
{
public:
    SurfaceField();
    SurfaceField(int i_width, int j_height, int k_depth, double cell_size);
    ~SurfaceField();

    void getDimensions(double *w, double *h, double *d) { *w = width; *h = height; *d = depth; }
    double getWidth()  { return width; }
    double getHeight() { return height; }
    double getDepth()  { return depth; }

    void getGridDimensions(int *i, int *j, int *k) { *i = i_width; *j = j_height; *k = k_depth; }
    double getCellSize() { return dx; }

    void setSurfaceThreshold(double val) { surfaceThreshold = val; }
    void setMaterialGrid(Array3d<int> matGrid);
    void setMaterialGrid();
    double getSurfaceThreshold() { return surfaceThreshold; }

    virtual void clear();

    virtual double getFieldValue(vmath::vec3 p);
    double getFieldValue(double x, double y, double z) { return getFieldValue(vmath::vec3(x, y, z)); }
    bool isInside(double x, double y, double z) { return isInside(vmath::vec3(x, y, z)); }
    bool isInside(vmath::vec3 p);
    bool isOutside(double x, double y, double z) { return isOutside(vmath::vec3(x, y, z)); }
    bool isOutside(vmath::vec3 p);
    bool _isPointNearSolid(vmath::vec3 p);

    double width, height, depth;

protected:

    int i_width, j_height, k_depth;
    double dx = 1.0;

    Array3d<int> materialGrid;
    bool isMaterialGridSet = false;

    double surfaceThreshold;

    
};

#endif
