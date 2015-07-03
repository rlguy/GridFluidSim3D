#pragma once

#include <stdio.h>
#include <iostream>

#include "glm/glm.hpp"
#include "array3d.h"
#include "aabb.h"

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
    void addPoint(glm::vec3 pos, double radius);
    void addPoint(glm::vec3 pos);
    void addCuboid(glm::vec3 pos, double w, double h, double d);
    void setSurfaceThreshold(double t) { _surfaceThreshold = t; }
    double getSurfaceThreshold() { return _surfaceThreshold; }
    void setMaterialGrid(Array3d<int> &matGrid);
    void getScalarField(Array3d<double> &field);
    bool isCellInsideSurface(int i, int j, int k);

private:
    inline GridIndex _positionToGridIndex(glm::vec3 p) {
        double invdx = 1.0 / _dx;
        return GridIndex((int)floor(p.x*invdx), 
                         (int)floor(p.y*invdx), 
                         (int)floor(p.z*invdx));
    }

    inline glm::vec3 _GridIndexToPosition(GridIndex g) {
        return glm::vec3(g.i*_dx, g.j*_dx, g.k*_dx);
    }

    inline glm::vec3 _GridIndexToPosition(int i, int j, int k) {
        return glm::vec3(i*_dx, j*_dx, k*_dx);
    }

    inline glm::vec3 _GridIndexToCellCenter(int i, int j, int k) {
        return glm::vec3(i*_dx + 0.5*_dx, j*_dx + 0.5*_dx, k*_dx + 0.5*_dx);
    }

    void _getGridIndexBounds(glm::vec3 pos, double r, 
                             GridIndex *gmin, GridIndex *gmax);

    double _evaluateFieldFunctionForRadiusSquared(double rsq);
    void _getCellVertexIndices(int i, int j, int k, GridIndex vertices[8]);

    void _calculateCenterCellValueForPoint(glm::vec3 p, int i, int j, int k);
    void _calculateCenterCellValueForCuboid(AABB &bbox, int i, int j, int k);

    int M_SOLID = 2;

    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;
    double _dx = 0.0;

    double _radius = 0.0;
    double _coef1 = 0.0;
    double _coef2 = 0.0;
    double _coef3 = 0.0;

    double _surfaceThreshold = 0.5;

    Array3d<double> _field;
    Array3d<double> _centerField;
    Array3d<bool> _isVertexSolid;

    bool _isCenterFieldEnabled = false;
};

