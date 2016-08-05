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
#include "fluidsimassert.h"

class ScalarField
{
public:
    ScalarField();
    ScalarField(int i, int j, int k, double dx);
    ~ScalarField();

    void getGridDimensions(int *i, int *j, int *k) { *i = _isize; *j = _jsize; *k = _ksize; }
    double getCellSize() { return _dx; }

    void clear();
    void setPointRadius(double r);
    double getPointRadius();
    void setSurfaceThreshold(double t);
    double getSurfaceThreshold();
    void setMaxScalarFieldThreshold(double t);
    void setMaxScalarFieldThreshold();
    double getMaxScalarFieldThreshold();
    bool isMaxScalarFieldThresholdSet();
    void enableWeightField();
    bool isWeightFieldEnabled();
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
    void setMaterialGrid(FluidMaterialGrid &matGrid);
    void setMaterialGrid(GridIndexVector &solidCells);
    void setSolidCells(GridIndexVector &solidCells);
    void getScalarField(Array3d<float> &field);
    double getScalarFieldValue(GridIndex g);
    double getScalarFieldValue(int i, int j, int k);
    double getScalarFieldValueAtCellCenter(GridIndex g);
    double getScalarFieldValueAtCellCenter(int i, int j, int k);
    void getSetScalarFieldValues(Array3d<bool> &field);
    bool isScalarFieldValueSet(GridIndex g);
    bool isScalarFieldValueSet(int i, int j, int k);
    double getRawScalarFieldValue(GridIndex g);
    double getRawScalarFieldValue(int i, int j, int k);
    bool isCellInsideSurface(int i, int j, int k);
    double getWeight(int i, int j, int k);
    double getWeight(GridIndex g);
    void getWeightField(Array3d<float> &field);
    void setScalarFieldValue(int i, int j, int k, double value);
    void setScalarFieldValue(GridIndex g, double value);
    void setCellFieldValues(int i, int j, int k, double value);
    void setCellFieldValues(GridIndex g, double value);
    void addScalarFieldValue(int i, int j, int k, double value);
    void addScalarFieldValue(GridIndex g, double value);
    void addCellFieldValues(int i, int j, int k, double value);
    void addCellFieldValues(GridIndex g, double value);
    double tricubicInterpolation(vmath::vec3 p);
    void setOffset(vmath::vec3 offset);
    vmath::vec3 getOffset();
    bool isPointInside(vmath::vec3 p);

    Array3d<float>* getPointerToScalarField();
    Array3d<float>* getPointerToWeightField();

private:

    double _evaluateTricubicFieldFunctionForRadiusSquared(double rsq);

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
    double _maxScalarFieldThreshold = 0.0;
    bool _isMaxScalarFieldThresholdSet = false;

    Array3d<float> _field;
    Array3d<bool> _isVertexSolid;
    Array3d<float> _weightField;
    Array3d<bool> _isVertexSet;

    bool _isWeightFieldEnabled = false;

    vmath::vec3 _gridOffset;
};

#endif
