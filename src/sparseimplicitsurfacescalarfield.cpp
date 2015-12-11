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
#include "SparseImplicitSurfaceScalarField.h"


SparseImplicitSurfaceScalarField::SparseImplicitSurfaceScalarField() {
}


SparseImplicitSurfaceScalarField::
        SparseImplicitSurfaceScalarField(int i, int j, int k, double dx) :
                                                _isize(i), _jsize(j), _ksize(k), _dx(dx),
                                                _field(i, j, k, 0.0),
                                                _isVertexSolid(i, j, k, false) {
    _field.enableSparseIndexTracking();
}

SparseImplicitSurfaceScalarField::~SparseImplicitSurfaceScalarField() {
}

void SparseImplicitSurfaceScalarField::getGridDimensions(int *i, int *j, int *k) { 
    *i = _isize; 
    *j = _jsize; 
    *k = _ksize; 
}

double SparseImplicitSurfaceScalarField::getCellSize() { 
    return _dx; 
}

void SparseImplicitSurfaceScalarField::clear() {
    _field.clear();
    _isVertexSolid.clear();
}

void SparseImplicitSurfaceScalarField::reserve(int n) {
    _field.reserve(n);
    _isVertexSolid.reserve(n);
}

void SparseImplicitSurfaceScalarField::setPointRadius(double r) {
    _radius = r;
    _invRadius = 1 / r;
    _coef1 = (4.0 / 9.0)*(1.0 / (r*r*r*r*r*r));
    _coef2 = (17.0 / 9.0)*(1.0 / (r*r*r*r));
    _coef3 = (22.0 / 9.0)*(1.0 / (r*r));
}

void SparseImplicitSurfaceScalarField::addPoint(vmath::vec3 p, double r) {
    setPointRadius(r);
    addPoint(p);
}

void SparseImplicitSurfaceScalarField::addPoint(vmath::vec3 p) {
    GridIndex gmin, gmax;
    Grid3d::getGridIndexBounds(p, _radius, _dx, _isize, _jsize, _ksize, &gmin, &gmax);

    vmath::vec3 gpos;
    vmath::vec3 v;
    double rsq = _radius*_radius;
    double distsq;
    double weight;
    for (int k = gmin.k; k <= gmax.k; k++) {
        for (int j = gmin.j; j <= gmax.j; j++) {
            for (int i = gmin.i; i <= gmax.i; i++) {
                gpos = Grid3d::GridIndexToPosition(i, j, k, _dx);
                v = gpos - p;
                distsq = vmath::dot(v, v);
                if (distsq < rsq) {
                    if (_weightType == WEIGHT_TRICUBIC) {
                        weight = _evaluateTricubicFieldFunctionForRadiusSquared(distsq);
                    } else {
                        weight = _evaluateTrilinearFieldFunction(v);
                    }

                    _field.add(i, j, k, (float)weight);
                }
            }
        }
    }

}

void SparseImplicitSurfaceScalarField::addPointValue(vmath::vec3 p, double r, double value) {
    setPointRadius(r);
    addPointValue(p, value);
}

void SparseImplicitSurfaceScalarField::addPointValue(vmath::vec3 p, double scale) {
    GridIndex gmin, gmax;
    Grid3d::getGridIndexBounds(p, _radius, _dx, _isize, _jsize, _ksize, &gmin, &gmax);

    vmath::vec3 gpos;
    vmath::vec3 v;
    double rsq = _radius*_radius;
    double distsq;
    double weight;
    for (int k = gmin.k; k <= gmax.k; k++) {
        for (int j = gmin.j; j <= gmax.j; j++) {
            for (int i = gmin.i; i <= gmax.i; i++) {
                gpos = Grid3d::GridIndexToPosition(i, j, k, _dx);
                v = gpos - p;
                distsq = vmath::dot(v, v);
                if (distsq < rsq) {
                    if (_weightType == WEIGHT_TRICUBIC) {
                        weight = _evaluateTricubicFieldFunctionForRadiusSquared(distsq);
                    } else {
                        weight = _evaluateTrilinearFieldFunction(v);
                    }

                    _field.add(i, j, k, (float)(weight*scale));
                }
            }
        }
    }

}

void SparseImplicitSurfaceScalarField::addCuboid(vmath::vec3 pos, 
                                                 double w, double h, double d) {
    GridIndex gmin = Grid3d::positionToGridIndex(pos, _dx);
    GridIndex gmax = Grid3d::positionToGridIndex(pos + vmath::vec3(w, h, d), _dx);
    AABB bbox = AABB(pos, w, h, d);

    double eps = 10e-6;
    vmath::vec3 gpos;
    for (int k = gmin.k; k <= gmax.k; k++) {
        for (int j = gmin.j; j <= gmax.j; j++) {
            for (int i = gmin.i; i <= gmax.i; i++) {
                gpos = Grid3d::GridIndexToPosition(i, j, k, _dx);
                if (bbox.isPointInside(gpos)) {
                    _field.add(i, j, k, (float)(_surfaceThreshold + eps));
                }

            }
        }
    }
}

void SparseImplicitSurfaceScalarField::setSolidCells(std::vector<GridIndex> &solidCells) {
    GridIndex vertices[8];
    for (unsigned int i = 0; i < solidCells.size(); i++) {
        Grid3d::getGridIndexVertices(solidCells[i], vertices);
        for (int idx = 0; idx < 8; idx++) {
            _isVertexSolid.set(vertices[idx], true);
        }
    }
}

void SparseImplicitSurfaceScalarField::getScalarField(SparseArray3d<float> &field) {
    assert(field.width == _field.width && 
           field.height == _field.height && 
           field.depth == _field.depth);

    std::vector<GridIndex> indices = _field.getSparseIndices();

    double eps = 10e-9;
    double val;

    GridIndex g;
    for (unsigned int i = 0; i < indices.size(); i++) {
        g = indices[i];
        val = _field(g);
        if (val < eps) {
            continue;
        }

        if (val > _surfaceThreshold && _isVertexSolid(g)) {
            val = _surfaceThreshold;
        } 

        field.set(g, (float)val);
    }

}

void SparseImplicitSurfaceScalarField::setTricubicWeighting() {
    _weightType = WEIGHT_TRICUBIC;
}

void SparseImplicitSurfaceScalarField::setTrilinearWeighting() {
    _weightType = WEIGHT_TRILINEAR;
}

double SparseImplicitSurfaceScalarField::
        _evaluateTricubicFieldFunctionForRadiusSquared(double rsq) {
    return 1.0 - _coef1*rsq*rsq*rsq + _coef2*rsq*rsq - _coef3*rsq;
}

double SparseImplicitSurfaceScalarField::_evaluateTrilinearFieldFunction(vmath::vec3 v) {
    double invdx = 1 / _dx;
    return _hatFunc(v.x*invdx) * _hatFunc(v.y*invdx) * _hatFunc(v.z*invdx);
}
