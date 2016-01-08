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
#ifndef IMPLICITSURFACESCALARFIELD_H
#define IMPLICITSURFACESCALARFIELD_H

#include <stdio.h>
#include <iostream>
#include <limits>

#include "vmath.h"
#include "array3d.h"
#include "grid3d.h"
#include "interpolation.h"
#include "aabb.h"
#include "fluidmaterialgrid.h"

class ImplicitSurfaceScalarField
{
public:
    ImplicitSurfaceScalarField();
    ImplicitSurfaceScalarField(int i, int j, int k, double dx);
    ~ImplicitSurfaceScalarField();

    void getGridDimensions(int *i, int *j, int *k) { *i = _isize; *j = _jsize; *k = _ksize; }
    double getCellSize() { return _dx; }

    void clear();
    void setPointRadius(double r);
    void enableCellCenterValues();
    void enableWeightField();
    void applyWeightField();
    void addPoint(vmath::vec3 pos, double radius);
    void addPoint(vmath::vec3 pos);
    void addPointValue(vmath::vec3 pos, double radius, double value);
    void addPointValue(vmath::vec3 pos, double value);
    void addCuboid(vmath::vec3 pos, double w, double h, double d);
    void addEllipsoid(vmath::vec3 p, vmath::mat3 G, double r);
    void addEllipsoid(vmath::vec3 p, vmath::mat3 G);
    void addEllipsoidValue(vmath::vec3 p, vmath::mat3 G, double r, double value);
    void addEllipsoidValue(vmath::vec3 p, vmath::mat3 G, double value);
    void setSurfaceThreshold(double t) { _surfaceThreshold = t; }
    double getSurfaceThreshold() { return _surfaceThreshold; }
    void setMaterialGrid(FluidMaterialGrid &matGrid);
    void setMaterialGrid(GridIndexVector &solidCells);
    void setSolidCells(GridIndexVector &solidCells);
    void getScalarField(Array3d<float> &field);
    double getScalarFieldValue(GridIndex g);
    double getScalarFieldValue(int i, int j, int k);
    double getRawScalarFieldValue(GridIndex g);
    double getRawScalarFieldValue(int i, int j, int k);
    bool isCellInsideSurface(int i, int j, int k);
    void setTricubicWeighting();
    void setTrilinearWeighting();
    double getWeight(int i, int j, int k);
    double getWeight(GridIndex g);
    void getWeightField(Array3d<float> &field);
    void setScalarFieldValue(int i, int j, int k, double value);
    void setScalarFieldValue(GridIndex g, double value);
    void setCellFieldValues(int i, int j, int k, double value);
    void setCellFieldValues(GridIndex g, double value);
    double tricubicInterpolation(vmath::vec3 p);
    void setOffset(vmath::vec3 offset);

private:

    inline double _hatFunc(double r) {
        if (r >= 0.0 && r <= 1.0) {
            return 1 - r;
        } else if (r >= -1.0 && r <= 0.0) {
            return 1 + r;
        }

        return 0.0;
    }

    double _evaluateTricubicFieldFunctionForRadiusSquared(double rsq);
    double _evaluateTrilinearFieldFunction(vmath::vec3 v);

    void _calculateCenterCellValueForPoint(vmath::vec3 p, int i, int j, int k);
    void _calculateCenterCellValueForCuboid(AABB &bbox, int i, int j, int k);

    int M_SOLID = 2;
    int WEIGHT_TRICUBIC = 0;
    int WEIGHT_TRILINEAR = 1;
    int _weightType = 0;

    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;
    double _dx = 0.0;

    double _radius = 0.0;
    double _invRadius = 1.0;
    double _coef1 = 0.0;
    double _coef2 = 0.0;
    double _coef3 = 0.0;

    double _surfaceThreshold = 0.5;

    Array3d<float> _field;
    Array3d<float> _centerField;
    Array3d<bool> _isVertexSolid;
    Array3d<float> _weightField;

    bool _isCenterFieldEnabled = false;
    bool _isWeightFieldEnabled = false;

    vmath::vec3 _gridOffset;
};

#endif
