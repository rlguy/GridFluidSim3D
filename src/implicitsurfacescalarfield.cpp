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
#include "implicitsurfacescalarfield.h"


ImplicitSurfaceScalarField::ImplicitSurfaceScalarField() {
}


ImplicitSurfaceScalarField::ImplicitSurfaceScalarField(int i, int j, int k, double dx) :
                                                       _isize(i), _jsize(j), _ksize(k), _dx(dx),
                                                       _field(i, j, k, 0.0),
                                                       _centerField(0, 0, 0, 0.0),
                                                       _isVertexSolid(i, j, k, false),
                                                       _weightField(0, 0, 0, 0.0),
                                                       _weightCountField(0, 0, 0, 0){
}

ImplicitSurfaceScalarField::~ImplicitSurfaceScalarField() {
}

void ImplicitSurfaceScalarField::clear() {
    _field.fill(0.0);
    _centerField.fill(0.0);
}

void ImplicitSurfaceScalarField::setPointRadius(double r) {
    _radius = r;
    _invRadius = 1 / r;
    _coef1 = (4.0 / 9.0)*(1.0 / (r*r*r*r*r*r));
    _coef2 = (17.0 / 9.0)*(1.0 / (r*r*r*r));
    _coef3 = (22.0 / 9.0)*(1.0 / (r*r));
}

void ImplicitSurfaceScalarField::enableCellCenterValues() {
    if (_isCenterFieldEnabled) {
        return;
    }

    _centerField = Array3d<float>(_isize-1, _jsize-1, _ksize-1, 0.0f);

    _isCenterFieldEnabled = true;
}

void ImplicitSurfaceScalarField::enableWeightField() {
    if (_isWeightFieldEnabled) {
        return;
    }

    _weightField = Array3d<float>(_isize, _jsize, _ksize, 0.0f);
    _weightCountField = Array3d<int>(_isize, _jsize, _ksize, 0);

    _isWeightFieldEnabled = true;
}

void ImplicitSurfaceScalarField::applyWeightField() {
    if (!_isWeightFieldEnabled) {
        return;
    }

    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                float weight = _weightField(i, j, k);
                if (weight > 0.0) {
                    float v = _field(i, j, k) / weight;
                    _field.set(i, j, k, v);
                }
            }
        }
    }
}

double ImplicitSurfaceScalarField::getWeight(GridIndex g) {
    return getWeight(g.i, g.j, g.k);
}

double ImplicitSurfaceScalarField::getWeight(int i, int j, int k) {
    if (!_isWeightFieldEnabled) {
        return 0.0;
    }

    assert(_weightField.isIndexInRange(i, j, k));
    return _weightField(i, j, k);
}

int ImplicitSurfaceScalarField::getWeightCount(GridIndex g) {
    return getWeightCount(g.i, g.j, g.k);
}

int ImplicitSurfaceScalarField::getWeightCount(int i, int j, int k) {
    if (!_isWeightFieldEnabled) {
        return 0;
    }

    assert(_weightCountField.isIndexInRange(i, j, k));
    return _weightCountField(i, j, k);
}

void ImplicitSurfaceScalarField::addPoint(glm::vec3 p, double r) {
    setPointRadius(r);
    addPoint(p);
}

void ImplicitSurfaceScalarField::addPoint(glm::vec3 p) {
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

                    _field.add(i, j, k, (float)weight);

                    if (_isWeightFieldEnabled) {
                        _weightField.add(i, j, k, (float)weight);
                        _weightCountField.add(i, j, k, 1);
                    }
                }

                if (_isCenterFieldEnabled) {
                    _calculateCenterCellValueForPoint(p, i, j, k);
                }
            }
        }
    }

}

void ImplicitSurfaceScalarField::addPointValue(glm::vec3 p, double r, double value) {
    setPointRadius(r);
    addPointValue(p, value);
}

void ImplicitSurfaceScalarField::addPointValue(glm::vec3 p, double scale) {
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

                    _field.add(i, j, k, (float)(weight*scale));

                    if (_isWeightFieldEnabled) {
                        _weightField.add(i, j, k, (float)weight);
                        _weightCountField.add(i, j, k, 1);
                    }
                }
            }
        }
    }

}

void ImplicitSurfaceScalarField::addCuboid(glm::vec3 pos, double w, double h, double d) {
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
                    _field.add(i, j, k, (float)(_surfaceThreshold + eps));

                    if (_isWeightFieldEnabled) {
                        _weightField.add(i, j, k, (float)(_surfaceThreshold + eps));
                        _weightCountField.add(i, j, k, 1);
                    }
                }

                if (_isCenterFieldEnabled) {
                    _calculateCenterCellValueForCuboid(bbox, i, j, k);
                }
            }
        }
    }
}

void ImplicitSurfaceScalarField::addEllipsoid(glm::vec3 p, glm::mat3 G, double r) {
    setPointRadius(r);
    addEllipsoid(p, G);
}

void ImplicitSurfaceScalarField::addEllipsoid(glm::vec3 p, glm::mat3 G) {
    GridIndex gmin, gmax;
    Grid3d::getGridIndexBounds(p, _radius, G, _dx, _isize, _jsize, _ksize, &gmin, &gmax);

    glm::vec3 gpos;
    glm::vec3 v;
    double rsq = _radius*_radius;
    double distsq;
    double weight;
    for (int k = gmin.k; k <= gmax.k; k++) {
        for (int j = gmin.j; j <= gmax.j; j++) {
            for (int i = gmin.i; i <= gmax.i; i++) {
                gpos = Grid3d::GridIndexToPosition(i, j, k, _dx);
                v = (gpos - p);
                v = G*v;

                distsq = glm::dot(v, v);

                if (distsq < rsq) {

                    if (_weightType == WEIGHT_TRICUBIC) {
                        weight = _evaluateTricubicFieldFunctionForRadiusSquared(distsq);
                    } else {
                        weight = _evaluateTrilinearFieldFunction(v);
                    }

                    _field.add(i, j, k, (float)weight);

                    if (_isWeightFieldEnabled) {
                        _weightField.add(i, j, k, (float)weight);
                        _weightCountField.add(i, j, k, 1);
                    }
                }

                if (_isCenterFieldEnabled) {
                    _calculateCenterCellValueForPoint(p, i, j, k);
                }
            }
        }
    }
}

void ImplicitSurfaceScalarField::addEllipsoidValue(glm::vec3 p, glm::mat3 G, double r, double value) {
    setPointRadius(r);
    addEllipsoidValue(p, G, value);
}

void ImplicitSurfaceScalarField::addEllipsoidValue(glm::vec3 p, glm::mat3 G, double scale) {
    GridIndex gmin, gmax;
    Grid3d::getGridIndexBounds(p, _radius, G, _dx, _isize, _jsize, _ksize, &gmin, &gmax);

    glm::vec3 gpos;
    glm::vec3 v;
    double rsq = _radius*_radius;
    double distsq;
    double weight;
    for (int k = gmin.k; k <= gmax.k; k++) {
        for (int j = gmin.j; j <= gmax.j; j++) {
            for (int i = gmin.i; i <= gmax.i; i++) {
                gpos = Grid3d::GridIndexToPosition(i, j, k, _dx);
                v = (gpos - p);
                v = G*v;

                distsq = glm::dot(v, v);

                if (distsq < rsq) {

                    if (_weightType == WEIGHT_TRICUBIC) {
                        weight = _evaluateTricubicFieldFunctionForRadiusSquared(distsq);
                    } else {
                        weight = _evaluateTrilinearFieldFunction(v);
                    }

                    _field.add(i, j, k, (float)weight*scale);

                    if (_isWeightFieldEnabled) {
                        _weightField.add(i, j, k, (float)weight);
                        _weightCountField.add(i, j, k, 1);
                    }
                }

                if (_isCenterFieldEnabled) {
                    _calculateCenterCellValueForPoint(p, i, j, k);
                }

            }
        }
    }
}

void ImplicitSurfaceScalarField::setSolidCells(std::vector<GridIndex> &solidCells) {
    setMaterialGrid(solidCells);
}

void ImplicitSurfaceScalarField::setMaterialGrid(std::vector<GridIndex> &solidCells) {
    GridIndex vertices[8];
    GridIndex g;
    for (unsigned int i = 0; i < solidCells.size(); i++) {
        g = solidCells[i];

        assert(Grid3d::isGridIndexInRange(g, _isize-1, _jsize-1, _ksize-1));
        Grid3d::getGridIndexVertices(g, vertices);
        for (int idx = 0; idx < 8; idx++) {
            _isVertexSolid.set(vertices[idx], true);
        }
    }
}

void ImplicitSurfaceScalarField::setMaterialGrid(Array3d<int> &matGrid) {
    assert(matGrid.width == _isize-1 && 
           matGrid.height == _jsize-1 && 
           matGrid.depth == _ksize-1);

    GridIndex vertices[8];
    for (int k = 0; k < _ksize-1; k++) {
        for (int j = 0; j < _jsize-1; j++) {
            for (int i = 0; i < _isize-1; i++) {
                if (matGrid(i, j, k) == M_SOLID) {
                    Grid3d::getGridIndexVertices(i, j, k, vertices);
                    for (int idx = 0; idx < 8; idx++) {
                        _isVertexSolid.set(vertices[idx], true);
                    }
                }
            }
        }
    }
}

void ImplicitSurfaceScalarField::getWeightField(Array3d<float> &field) {
    if (!_isWeightFieldEnabled) {
        return;
    }

    assert(field.width == _field.width && 
           field.height == _field.height && 
           field.depth == _field.depth);

    for (int k = 0; k < field.depth; k++) {
        for (int j = 0; j < field.height; j++) {
            for (int i = 0; i < field.width; i++) {
                field.set(i, j, k, _weightField(i, j, k));
            }
        }
    }
}

void ImplicitSurfaceScalarField::getScalarField(Array3d<float> &field) {
    assert(field.width == _field.width && 
           field.height == _field.height && 
           field.depth == _field.depth);

    double val;
    for (int k = 0; k < field.depth; k++) {
        for (int j = 0; j < field.height; j++) {
            for (int i = 0; i < field.width; i++) {
                val = _field(i, j, k);
                if (_isVertexSolid(i, j, k) && val > _surfaceThreshold) {
                    val = _surfaceThreshold;
                } 

                field.set(i, j, k, (float)val);
            }
        }
    }
}

bool ImplicitSurfaceScalarField::isCellInsideSurface(int i, int j, int k) {
    assert(_isCenterFieldEnabled);
    assert(_centerField.isIndexInRange(i, j, k));

    return _centerField(i, j, k) > _surfaceThreshold;
}

void ImplicitSurfaceScalarField::setTricubicWeighting() {
    _weightType = WEIGHT_TRICUBIC;
}

void ImplicitSurfaceScalarField::setTrilinearWeighting() {
    _weightType = WEIGHT_TRILINEAR;
}

void ImplicitSurfaceScalarField::setFieldValue(int i, int j, int k, double value) {
    assert(Grid3d::isGridIndexInRange(i, j, k, _field.width, _field.height, _field.depth));
    _field.set(i, j, k, value);
}

void ImplicitSurfaceScalarField::setFieldValue(GridIndex g, double value) {
    setFieldValue(g.i, g.j, g.k, value);
}

void ImplicitSurfaceScalarField::setCellFieldValues(int i, int j, int k, double value) {
    assert(Grid3d::isGridIndexInRange(i, j, k, _field.width-1, _field.height-1, _field.depth-1));
    GridIndex vertices[8];
    Grid3d::getGridIndexVertices(i, j, k, vertices);
    for (int i = 0; i < 8; i++) {
        _field.set(vertices[i], value);
    }
}

void ImplicitSurfaceScalarField::setCellFieldValues(GridIndex g, double value) {
    setCellFieldValues(g.i, g.j, g.k, value);
}

double ImplicitSurfaceScalarField::tricubicInterpolation(glm::vec3 p) {
    if (!Grid3d::isPositionInGrid(p.x, p.y, p.z, _dx, _isize, _jsize, _ksize)) {
        return 0.0;
    }

    int i, j, k;
    double gx, gy, gz;
    Grid3d::positionToGridIndex(p.x, p.y, p.z, _dx, &i, &j, &k);
    Grid3d::GridIndexToPosition(i, j, k, _dx, &gx, &gy, &gz);

    double inv_dx = 1 / _dx;
    double ix = (p.x - gx)*inv_dx;
    double iy = (p.y - gy)*inv_dx;
    double iz = (p.z - gz)*inv_dx;

    int refi = i - 1;
    int refj = j - 1;
    int refk = k - 1;
    double points[4][4][4];
    for (int pk = 0; pk < 4; pk++) {
        for (int pj = 0; pj < 4; pj++) {
            for (int pi = 0; pi < 4; pi++) {
                if (_field.isIndexInRange(pi + refi, pj + refj, pk + refk)) {
                    points[pi][pj][pk] = _field(pi + refi, pj + refj, pk + refk);
                } else {
                    points[pi][pj][pk] = 0;
                }
            }
        }
    }

    return Interpolation::tricubicInterpolate(points, ix, iy, iz);
}

double ImplicitSurfaceScalarField::_evaluateTricubicFieldFunctionForRadiusSquared(double rsq) {
    return 1.0 - _coef1*rsq*rsq*rsq + _coef2*rsq*rsq - _coef3*rsq;
}

double ImplicitSurfaceScalarField::_evaluateTrilinearFieldFunction(glm::vec3 v) {
    double invdx = 1 / _dx;
    return _hatFunc(v.x*invdx) * _hatFunc(v.y*invdx) * _hatFunc(v.z*invdx);
}

void ImplicitSurfaceScalarField::_calculateCenterCellValueForPoint(glm::vec3 p, int i, int j, int k) {
    if ( i == _isize - 1 || j == _jsize - 1 || k == _ksize - 1 ) {
        return;
    }

    glm::vec3 gpos = Grid3d::GridIndexToCellCenter(i, j, k, _dx);
    glm::vec3 v = gpos - p;
    double distsq = glm::dot(v, v);
    if (distsq < _radius*_radius) {
        double val = _evaluateTricubicFieldFunctionForRadiusSquared(distsq);
        _centerField.add(i, j, k, (float)val);
    }
}

void ImplicitSurfaceScalarField::_calculateCenterCellValueForCuboid(AABB &bbox, int i, int j, int k) {
    if ( i == _isize - 1 || j == _jsize - 1 || k == _ksize - 1 ) {
        return;
    }

    glm::vec3 gpos = Grid3d::GridIndexToCellCenter(i, j, k, _dx);
    if (bbox.isPointInside(gpos)) {
        double eps = 10e-6;
        _centerField.add(i, j, k, (float)(_surfaceThreshold + eps));
    }
}