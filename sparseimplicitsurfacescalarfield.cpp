#include "SparseImplicitSurfaceScalarField.h"


SparseImplicitSurfaceScalarField::SparseImplicitSurfaceScalarField() {
}


SparseImplicitSurfaceScalarField::
        SparseImplicitSurfaceScalarField(int i, int j, int k, double dx) :
                                                _isize(i), _jsize(j), _ksize(k), _dx(dx),
                                                _field(i, j, k, 0.0),
                                                _isVertexSolid(i, j, k, false) {
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

void SparseImplicitSurfaceScalarField::addPoint(glm::vec3 p, double r) {
    setPointRadius(r);
    addPoint(p);
}

void SparseImplicitSurfaceScalarField::addPoint(glm::vec3 p) {
    GridIndex gmin, gmax;
    Grid3d::getGridIndexBounds(p, _radius, _dx, _isize, _jsize, _ksize, &gmin, &gmax);

    glm::vec3 gpos;
    glm::vec3 v;
    double rsq = _radius*_radius;
    double distsq;
    double weight;
    for (int k = gmin.k; k <= gmax.k; k++) {
        for (int j = gmin.j; j <= gmax.j; j++) {
            for (int i = gmin.i; i <= gmax.i; i++) {
                gpos = Grid3d::GridIndexToPosition(i, j, k, _dx);
                v = gpos - p;
                distsq = glm::dot(v, v);
                if (distsq < rsq) {
                    if (_weightType == WEIGHT_TRICUBIC) {
                        weight = _evaluateTricubicFieldFunctionForRadiusSquared(distsq);
                    } else {
                        weight = _evaluateTrilinearFieldFunction(v);
                    }

                    _field.add(i, j, k, weight);
                }
            }
        }
    }

}

void SparseImplicitSurfaceScalarField::addPointValue(glm::vec3 p, double r, double value) {
    setPointRadius(r);
    addPointValue(p, value);
}

void SparseImplicitSurfaceScalarField::addPointValue(glm::vec3 p, double scale) {
    GridIndex gmin, gmax;
    Grid3d::getGridIndexBounds(p, _radius, _dx, _isize, _jsize, _ksize, &gmin, &gmax);

    glm::vec3 gpos;
    glm::vec3 v;
    double rsq = _radius*_radius;
    double distsq;
    double weight;
    for (int k = gmin.k; k <= gmax.k; k++) {
        for (int j = gmin.j; j <= gmax.j; j++) {
            for (int i = gmin.i; i <= gmax.i; i++) {
                gpos = Grid3d::GridIndexToPosition(i, j, k, _dx);
                v = gpos - p;
                distsq = glm::dot(v, v);
                if (distsq < rsq) {
                    if (_weightType == WEIGHT_TRICUBIC) {
                        weight = _evaluateTricubicFieldFunctionForRadiusSquared(distsq);
                    } else {
                        weight = _evaluateTrilinearFieldFunction(v);
                    }

                    _field.add(i, j, k, weight*scale);
                }
            }
        }
    }

}

void SparseImplicitSurfaceScalarField::addCuboid(glm::vec3 pos, 
                                                 double w, double h, double d) {
    GridIndex gmin = Grid3d::positionToGridIndex(pos, _dx);
    GridIndex gmax = Grid3d::positionToGridIndex(pos + glm::vec3(w, h, d), _dx);
    AABB bbox = AABB(pos, w, h, d);

    double eps = 10e-6;
    glm::vec3 gpos;
    for (int k = gmin.k; k <= gmax.k; k++) {
        for (int j = gmin.j; j <= gmax.j; j++) {
            for (int i = gmin.i; i <= gmax.i; i++) {
                gpos = Grid3d::GridIndexToPosition(i, j, k, _dx);
                if (bbox.isPointInside(gpos)) {
                    _field.add(i, j, k, _surfaceThreshold + eps);
                }

            }
        }
    }
}

void SparseImplicitSurfaceScalarField::setSolidCells(std::vector<GridIndex> &solidCells) {
    GridIndex vertices[8];
    for (int i = 0; i < solidCells.size(); i++) {
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
    for (int i = 0; i < indices.size(); i++) {
        g = indices[i];
        val = _field(g);
        if (val < eps) {
            continue;
        }

        if (val > _surfaceThreshold && _isVertexSolid(g)) {
            val = _surfaceThreshold;
        } 

        field.set(g, val);
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

double SparseImplicitSurfaceScalarField::_evaluateTrilinearFieldFunction(glm::vec3 v) {
    double invdx = 1 / _dx;
    return _hatFunc(v.x*invdx) * _hatFunc(v.y*invdx) * _hatFunc(v.z*invdx);
}
