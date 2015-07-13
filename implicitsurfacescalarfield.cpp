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

    _centerField = Array3d<double>(_isize-1, _jsize-1, _ksize-1, 0.0);

    _isCenterFieldEnabled = true;
}

void ImplicitSurfaceScalarField::enableWeightField() {
    if (_isWeightFieldEnabled) {
        return;
    }

    _weightField = Array3d<double>(_isize, _jsize, _ksize, 0.0);
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
                double weight = _weightField(i, j, k);
                if (weight > 0.0) {
                    double v = _field(i, j, k) / weight;
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
    _getGridIndexBounds(p, _radius, &gmin, &gmax);

    glm::vec3 gpos;
    glm::vec3 v;
    double rsq = _radius*_radius;
    double distsq;
    double weight;
    for (int k = gmin.k; k <= gmax.k; k++) {
        for (int j = gmin.j; j <= gmax.j; j++) {
            for (int i = gmin.i; i <= gmax.i; i++) {
                gpos = _GridIndexToPosition(i, j, k);
                v = gpos - p;
                distsq = glm::dot(v, v);
                if (distsq < rsq) {
                    weight = _evaluateFieldFunctionForRadiusSquared(distsq);
                    _field.add(i, j, k, weight);

                    if (_isWeightFieldEnabled) {
                        _weightField.add(i, j, k, weight);
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
    _getGridIndexBounds(p, _radius, &gmin, &gmax);

    glm::vec3 gpos;
    glm::vec3 v;
    double rsq = _radius*_radius;
    double distsq;
    double weight;
    for (int k = gmin.k; k <= gmax.k; k++) {
        for (int j = gmin.j; j <= gmax.j; j++) {
            for (int i = gmin.i; i <= gmax.i; i++) {
                gpos = _GridIndexToPosition(i, j, k);
                v = gpos - p;
                distsq = glm::dot(v, v);
                if (distsq < rsq) {
                    weight = _evaluateFieldFunctionForRadiusSquared(distsq);
                    _field.add(i, j, k, weight*scale);

                    if (_isWeightFieldEnabled) {
                        _weightField.add(i, j, k, weight);
                        _weightCountField.add(i, j, k, 1);
                    }
                }
            }
        }
    }

}

void ImplicitSurfaceScalarField::addCuboid(glm::vec3 pos, double w, double h, double d) {
    GridIndex gmin = _positionToGridIndex(pos);
    GridIndex gmax = _positionToGridIndex(pos + glm::vec3(w, h, d));
    AABB bbox = AABB(pos, w, h, d);

    double eps = 10e-6;
    glm::vec3 gpos;
    for (int k = gmin.k; k <= gmax.k; k++) {
        for (int j = gmin.j; j <= gmax.j; j++) {
            for (int i = gmin.i; i <= gmax.i; i++) {
                gpos = _GridIndexToPosition(i, j, k);
                if (bbox.isPointInside(gpos)) {
                    _field.add(i, j, k, _surfaceThreshold + eps);

                    if (_isWeightFieldEnabled) {
                        _weightField.add(i, j, k, _surfaceThreshold + eps);
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

void ImplicitSurfaceScalarField::setMaterialGrid(Array3d<int> &matGrid) {
    assert(matGrid.width == _isize-1 && 
           matGrid.height == _jsize-1 && 
           matGrid.depth == _ksize-1);

    GridIndex vertices[8];
    for (int k = 0; k < _ksize-1; k++) {
        for (int j = 0; j < _jsize-1; j++) {
            for (int i = 0; i < _isize-1; i++) {
                if (matGrid(i, j, k) == M_SOLID) {
                    _getCellVertexIndices(i, j, k, vertices);
                    for (int idx = 0; idx < 8; idx++) {
                        _isVertexSolid.set(vertices[idx], true);
                    }
                }
            }
        }
    }
}

void ImplicitSurfaceScalarField::getScalarField(Array3d<double> &field) {
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

                field.set(i, j, k, val);
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

void ImplicitSurfaceScalarField::_getGridIndexBounds(glm::vec3 pos, double r, 
                                                     GridIndex *gmin, GridIndex *gmax) {
    GridIndex c = _positionToGridIndex(pos);
    glm::vec3 cpos = _GridIndexToPosition(c);
    glm::vec3 trans = pos - cpos;
    double inv = 1.0 / _dx;

    int imin = c.i - (int)fmax(0, ceil((r-trans.x)*inv));
    int jmin = c.j - (int)fmax(0, ceil((r-trans.y)*inv));
    int kmin = c.k - (int)fmax(0, ceil((r-trans.z)*inv));
    int imax = c.i + (int)fmax(0, ceil((r-_dx+trans.x)*inv));
    int jmax = c.j + (int)fmax(0, ceil((r-_dx+trans.y)*inv));
    int kmax = c.k + (int)fmax(0, ceil((r-_dx+trans.z)*inv));

    *gmin = GridIndex((int)fmax(imin, 0), 
                      (int)fmax(jmin, 0), 
                      (int)fmax(kmin, 0));
    *gmax = GridIndex((int)fmin(imax, _isize-1), 
                      (int)fmin(jmax, _jsize-1), 
                      (int)fmin(kmax, _ksize-1));
}

double ImplicitSurfaceScalarField::_evaluateFieldFunctionForRadiusSquared(double rsq) {
    if (_weightType == WEIGHT_TRICUBIC) {
        return 1.0 - _coef1*rsq*rsq*rsq + _coef2*rsq*rsq - _coef3*rsq;
    } else {
        double r = sqrt(rsq);
        return (_radius - r) * _invRadius;
    }
    
}

void ImplicitSurfaceScalarField::_getCellVertexIndices(int i, int j, int k, GridIndex vertices[8]){
    vertices[0] = GridIndex(i,     j,     k);
    vertices[1] = GridIndex(i + 1, j,     k);
    vertices[2] = GridIndex(i + 1, j,     k + 1);
    vertices[3] = GridIndex(i,     j,     k + 1);
    vertices[4] = GridIndex(i,     j + 1, k);
    vertices[5] = GridIndex(i + 1, j + 1, k);
    vertices[6] = GridIndex(i + 1, j + 1, k + 1);
    vertices[7] = GridIndex(i,     j + 1, k + 1);
}

void ImplicitSurfaceScalarField::_calculateCenterCellValueForPoint(glm::vec3 p, int i, int j, int k) {
    if ( i == _isize - 1 || j == _jsize - 1 || k == _ksize - 1 ) {
        return;
    }

    glm::vec3 gpos = _GridIndexToCellCenter(i, j, k);
    glm::vec3 v = gpos - p;
    double distsq = glm::dot(v, v);
    if (distsq < _radius*_radius) {
        double val = _evaluateFieldFunctionForRadiusSquared(distsq);
        _centerField.add(i, j, k, val);
    }
}

void ImplicitSurfaceScalarField::_calculateCenterCellValueForCuboid(AABB &bbox, int i, int j, int k) {
    if ( i == _isize - 1 || j == _jsize - 1 || k == _ksize - 1 ) {
        return;
    }

    glm::vec3 gpos = _GridIndexToCellCenter(i, j, k);
    if (bbox.isPointInside(gpos)) {
        double eps = 10e-6;
        _centerField.add(i, j, k, _surfaceThreshold + eps);
    }
}