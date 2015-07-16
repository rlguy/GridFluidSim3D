#include "levelset.h"

LevelSet::LevelSet()
{
}

LevelSet::LevelSet(int i, int j, int k, double dx) : 
                                 _isize(i), _jsize(j), _ksize(k), _dx(dx),
                                 _signedDistance(Array3d<double>(i, j, k, 0.0)),
                                 _indexGrid(Array3d<unsigned char>(i, j, k, -1)),
                                 _isDistanceSet(Array3d<bool>(i, j, k, false)),
                                 _distanceField(LevelSetField(i, j, k, dx)) {
}

LevelSet::~LevelSet()
{
}

void LevelSet::setSurfaceMesh(TriangleMesh m) {
    _surfaceMesh = m;
}

void LevelSet::_resetSignedDistanceField() {
    _signedDistance.fill(0.0);
    _indexGrid.fill(-1);
    _isDistanceSet.fill(false);
}

void LevelSet::_getTriangleGridCellOverlap(Triangle t, std::vector<GridIndex> &cells) {
    std::vector<GridIndex> testcells;
    AABB tbbox = AABB(t, _surfaceMesh.vertices);
    tbbox.getOverlappingGridCells(_dx, testcells);

    AABB cbbox = AABB(glm::vec3(0.0, 0.0, 0.0), _dx, _dx, _dx);
    for (int i = 0; i < (int)testcells.size(); i++) {
        cbbox.position = _gridIndexToPosition(testcells[i]);
        if (cbbox.isOverlappingTriangle(t, _surfaceMesh.vertices)) {
            cells.push_back(testcells[i]);
        }
    }
}

void LevelSet::_calculateDistancesSquaredForTriangle(int index) {
    Triangle t = _surfaceMesh.triangles[index];

    std::vector<GridIndex> cells;
    _getTriangleGridCellOverlap(t, cells);

    GridIndex g;
    double distsq;
    for (int i = 0; i < (int)cells.size(); i++) {
        g = cells[i];
        distsq = _minDistToTriangleSquared(g, index);

        if (!_isDistanceSet(g) || distsq < _signedDistance(g)) {
            _setLevelSetCell(g, distsq, index);
        }
    }
}

void LevelSet::_calculateUnsignedSurfaceDistanceSquared() {
    for (int i = 0; i < (int)_surfaceMesh.triangles.size(); i++) {
        _calculateDistancesSquaredForTriangle(i);
    }
}

void LevelSet::_getNeighbourGridIndices6(GridIndex g, GridIndex n[6]) {
    n[0] = GridIndex(g.i-1, g.j, g.k);
    n[1] = GridIndex(g.i+1, g.j, g.k);
    n[2] = GridIndex(g.i, g.j-1, g.k);
    n[3] = GridIndex(g.i, g.j+1, g.k);
    n[4] = GridIndex(g.i, g.j, g.k-1);
    n[5] = GridIndex(g.i, g.j, g.k+1);
}

void LevelSet::_getLayerCells(int idx, std::vector<GridIndex> &layer, 
                                       std::vector<GridIndex> &nextLayer,
                                       Array3d<unsigned char> &layerGrid) {
    GridIndex ns[6];
    GridIndex g, n;
    for (int i = 0; i < (int)layer.size(); i++) {
        g = layer[i];
        _getNeighbourGridIndices6(g, ns);
        for (int j = 0; j < 6; j++) {
            n = ns[j];
            if (_isCellIndexInRange(n) && layerGrid(n) == -1) {
                nextLayer.push_back(n);
                layerGrid.set(n, idx);
            }
        }
    }
}

void LevelSet::_getCellLayers(std::vector<std::vector<GridIndex>> &layers) {
    Array3d<unsigned char> layerGrid(_isize, _jsize, _ksize, -1);

    std::vector<GridIndex> layer;
    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                if (_isDistanceSet(i, j, k)) {
                    layer.push_back(GridIndex(i, j, k));
                    layerGrid.set(i, j, k, 0);
                }
            }
        }
    }

    std::vector<GridIndex> q;
    layers.push_back(q);
    _getLayerCells(1, layer, layers[0], layerGrid);

    for (int i = 2; layers[i-2].size() > 0 && i < _numLayers; i++) {
        std::vector<GridIndex> q;
        layers.push_back(q);
        _getLayerCells(i, layers[i - 2], layers[i - 1], layerGrid);
    }
}

void LevelSet::_setLevelSetCell(GridIndex g, double dist, int tidx) {
    _signedDistance.set(g, dist);
    _indexGrid.set(g, tidx);
    _isDistanceSet.set(g, true);
}

void LevelSet::_resetLevelSetCell(GridIndex g) {
    _signedDistance.set(g, 0.0);
    _indexGrid.set(g, -1);
    _isDistanceSet.set(g, false);
}

void LevelSet::_calculateUnsignedDistanceSquaredForLayer(std::vector<GridIndex> &q) {

    GridIndex g, n;
    GridIndex ns[6];
    double distsq;

    while (!q.empty()) {
        g = q[q.size() - 1];
        q.pop_back();

        _getNeighbourGridIndices6(g, ns);
        for (int j = 0; j < 6; j++) {
            n = ns[j];
            if (_isCellIndexInRange(n) && _isDistanceSet(n)) {
                int tidx = _indexGrid(n);
                distsq = _minDistToTriangleSquared(n, tidx);

                if (!_isDistanceSet(g) || distsq < _signedDistance(g)) {
                    _setLevelSetCell(g, distsq, tidx);
                }

                if (distsq < _signedDistance(n)) {
                    _resetLevelSetCell(n);
                    q.push_back(n);
                }
            }
        }

        if (!_isDistanceSet(g)) {
            q.push_back(g);
        }
    }
}

void LevelSet::_calculateUnsignedDistanceSquared() {
    std::vector<std::vector<GridIndex>> cellLayers;
    _getCellLayers(cellLayers);

    for (int i = 0; i < (int)cellLayers.size(); i++) {
        _calculateUnsignedDistanceSquaredForLayer(cellLayers[i]);
    }
}

void LevelSet::_squareRootDistanceField() {
    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                if (_isDistanceSet(i, j, k)) {
                    _signedDistance.set(i, j, k, sqrt(_signedDistance(i, j, k)));
                }
            }
        }
    }
}

void LevelSet::_updateCellSign(GridIndex g, std::vector<glm::vec3> &triangleCenters, 
                                            std::vector<glm::vec3> &triangleDirections) {
    // Use this convention:
    //     outside mesh, signed distance is negative
    //     inside mesh, signed distance is positive

    int tidx = _indexGrid(g);
    glm::vec3 p = _gridIndexToCellCenter(g);
    glm::vec3 ct = triangleCenters[tidx];
    glm::vec3 n = triangleDirections[tidx];
    glm::vec3 v = ct - p;

    if (glm::dot(v, n) < 0) {
        _signedDistance.set(g, -1.0*_signedDistance(g));
    }
}

void LevelSet::_calculateDistanceFieldSigns() {
    std::vector<glm::vec3> triangleFaceDirections;
    std::vector<glm::vec3> triangleFaceCenters;
    triangleFaceDirections.reserve(_surfaceMesh.triangles.size());
    triangleFaceCenters.reserve(_surfaceMesh.triangles.size());

    for (int i = 0; i < (int)_surfaceMesh.triangles.size(); i++) {
        triangleFaceDirections.push_back(_surfaceMesh.getTriangleFaceDirection(i));
        triangleFaceCenters.push_back(_surfaceMesh.getTriangleCenter(i));
    }

    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                if (_isDistanceSet(i, j, k)) {
                    _updateCellSign(GridIndex(i, j, k), triangleFaceCenters,
                                                        triangleFaceDirections);
                }
            }
        }
    }
}

void LevelSet::calculateSignedDistanceField() {
    calculateSignedDistanceField((int)fmax(fmax(_isize, _jsize), _ksize));
}

void LevelSet::calculateSignedDistanceField(int numLayers) {
    _numLayers = numLayers;

    _resetSignedDistanceField();
    _calculateUnsignedSurfaceDistanceSquared();
    _calculateUnsignedDistanceSquared();
    _squareRootDistanceField();
    _calculateDistanceFieldSigns();

    _distanceField.setSignedDistanceField(_signedDistance);
}

double LevelSet::_minDistToTriangleSquared(GridIndex g, int tidx) {
    glm::vec3 p = _gridIndexToCellCenter(g);
    return _minDistToTriangleSquared(p, tidx);
}

double LevelSet::_minDistToTriangleSquared(glm::vec3 p, int tidx) {
    glm::vec3 tri[3];
    _surfaceMesh.getTrianglePosition(tidx, tri);
    glm::vec3 tp = Collision::findClosestPointOnTriangle(p, tri[0], tri[1], tri[2]);
    glm::vec3 v = tp - p;
    return glm::dot(v, v);
}

double LevelSet::_minDistToTriangleSquared(glm::vec3 p, int tidx, glm::vec3 *point) {
    glm::vec3 tri[3];
    _surfaceMesh.getTrianglePosition(tidx, tri);
    glm::vec3 tp = Collision::findClosestPointOnTriangle(p, tri[0], tri[1], tri[2]);
    glm::vec3 v = tp - p;

    *point = tp;
    return glm::dot(v, v);
}

glm::vec3 LevelSet::_findClosestPointOnSurface(GridIndex g) {
    assert(_isCellIndexInRange(g) && _isDistanceSet(g));

    glm::vec3 tri[3];
    _surfaceMesh.getTrianglePosition(_indexGrid(g), tri);
    glm::vec3 p0 = _gridIndexToCellCenter(g);
    
    return Collision::findClosestPointOnTriangle(p0, tri[0], tri[1], tri[2]);
}

glm::vec3 LevelSet::_findClosestPointOnSurface(glm::vec3 p) {
    GridIndex g = _positionToGridIndex(p);

    GridIndex ns[7];
    _getNeighbourGridIndices6(g, ns);
    ns[6] = g;

    bool isFound = false;
    double mindistsq = std::numeric_limits<double>::infinity();
    glm::vec3 minpoint = p;
    GridIndex n;
    for (int i = 0; i < 7; i++) {
        n = ns[i];

        if (_isCellIndexInRange(n) && _isDistanceSet(n)) {
            glm::vec3 surfacePoint;
            double distsq = _minDistToTriangleSquared(p, _indexGrid(n), &surfacePoint);

            if (distsq < mindistsq) {
                mindistsq = distsq;
                minpoint = surfacePoint;
                isFound = true;
            }
        }
    }

    return minpoint;
}

glm::vec3 LevelSet::_findClosestPointOnSurface(glm::vec3 p, int *tidx) {
    GridIndex g = _positionToGridIndex(p);

    GridIndex ns[7];
    _getNeighbourGridIndices6(g, ns);
    ns[6] = g;

    bool isFound = false;
    double mindistsq = std::numeric_limits<double>::infinity();
    glm::vec3 minpoint = p;
    int mint;
    GridIndex n;
    for (int i = 0; i < 7; i++) {
        n = ns[i];

        if (_isCellIndexInRange(n) && _isDistanceSet(n)) {
            glm::vec3 surfacePoint;
            double distsq = _minDistToTriangleSquared(p, _indexGrid(n), &surfacePoint);

            if (distsq < mindistsq) {
                mindistsq = distsq;
                minpoint = surfacePoint;
                mint = _indexGrid(n);
                isFound = true;
            }
        }
    }

    *tidx = mint;
    return minpoint;
}

glm::vec3 LevelSet::_evaluateVelocityAtPosition(MACVelocityField &vgrid, glm::vec3 p) {
    if (_isPointInsideSurface(p)) {
        return vgrid.evaluateVelocityAtPosition(p);
    } else {
        p = _findClosestPointOnSurface(p);
        return vgrid.evaluateVelocityAtPosition(p);
    }

    return vgrid.evaluateVelocityAtPosition(p);
}

glm::vec3 LevelSet::_evaluateVelocityAtGridIndex(MACVelocityField &vgrid, GridIndex g) {
    if (_isCellInsideSurface(g)) {
        return vgrid.evaluateVelocityAtCellCenter(g.i, g.j, g.k);
    } else {
        glm::vec3 p = _findClosestPointOnSurface(g);
        return vgrid.evaluateVelocityAtPosition(p);
    }
}

glm::vec3 LevelSet::getClosestPointOnSurface(glm::vec3 p) {
    return _findClosestPointOnSurface(p);
}

void LevelSet::_getGridIndexBounds(glm::vec3 pos, double r, 
                                   GridIndex *gmin, GridIndex *gmax) {
    GridIndex c = _positionToGridIndex(pos);
    glm::vec3 cpos = _gridIndexToPosition(c);
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

void LevelSet::_getCellsInsideSurfaceWithinRadius(glm::vec3 p, double r, 
                                                  std::vector<GridIndex> &cells) {
    GridIndex gmin, gmax;
    _getGridIndexBounds(p, r, &gmin, &gmax);

    double rsq = r*r;
    glm::vec3 v;
    for (int k = gmin.k; k <= gmax.k; k++) {
        for (int j = gmin.j; j <= gmax.j; j++) {
            for (int i = gmin.i; i <= gmax.i; i++) {
                if (_isDistanceSet(i, j, k) && _isCellInsideSurface(i, j, k)) {
                    v = _gridIndexToCellCenter(i, j, k) - p;
                    double distsq = glm::dot(v, v);
                    if (distsq < rsq) {
                        cells.push_back(GridIndex(i, j, k));
                    }
                }
            }
        }
    }
}

void LevelSet::_getCurvatureSamplePoints(glm::vec3 p, std::vector<glm::vec3> &points) {
    std::vector<GridIndex> cells;
    _getCellsInsideSurfaceWithinRadius(p, _surfaceCurvatureSampleRadius*_dx, cells);

    if (cells.size() == 0) {
        return;
    }

    int min = 0;
    int max = cells.size() - 1;
    int maxSamples = fmin(8*max, _maxSurfaceCurvatureSamples);
    GridIndex g;
    glm::vec3 jitter;
    for (int i = 0; i < maxSamples; i++) {
        int randIdx =  min + (rand() % (int)(max - min + 1));
        g = cells[randIdx];
        jitter = glm::vec3((float)rand()) / ((float)(RAND_MAX / (_dx - 0.0)),
                           (float)rand()) / ((float)(RAND_MAX / (_dx - 0.0)),
                           (float)rand()) / ((float)(RAND_MAX / (_dx - 0.0)));
        points.push_back(_gridIndexToPosition(g) + jitter);
    }
}

void LevelSet::_calculateCurvatureAtVertex(int idx) {
    glm::vec3 v = _surfaceMesh.vertices[idx];
    std::vector<glm::vec3> samples;
    _getCurvatureSamplePoints(v, samples);

    double radius = _surfaceCurvatureSampleRadius;
    glm::vec3 vnorm = _surfaceMesh.normals[idx];
    glm::vec3 p, tnorm, xji;
    int tidx;
    double curvature = 0.0;
    for (int i = 0; i < samples.size(); i++) {
        p = samples[i];
        p = _findClosestPointOnSurface(p, &tidx);

        if (tidx < 0 || tidx >= _surfaceMesh.triangles.size()) {
            continue;
        }

        tnorm = _surfaceMesh.getTriangleNormalSmooth(tidx, p);

        xji = p - v;
        double k = (1 - glm::dot(vnorm, tnorm)) * (1 - (glm::length(xji)/radius));
        if (glm::dot(glm::normalize(xji), vnorm) > 0.0) {
            k = 0.0;
        }

        curvature += k;
    }

    _vertexCurvatures.push_back(curvature);
}

void LevelSet::calculateSurfaceCurvature() {
    _vertexCurvatures.clear();

    for (int i = 0; i < (int)_surfaceMesh.vertices.size(); i++) {
        _calculateCurvatureAtVertex(i);
    }
}

double LevelSet::getSurfaceCurvature(glm::vec3 p) {
    assert(_vertexCurvatures.size() == _surfaceMesh.vertices.size());

    int tidx;
    p = _findClosestPointOnSurface(p, &tidx);

    assert(tidx <= _surfaceMesh.triangles.size());

    Triangle t = _surfaceMesh.triangles[tidx];
    double k0 = _vertexCurvatures[t.tri[0]];
    double k1 = _vertexCurvatures[t.tri[1]];
    double k2 = _vertexCurvatures[t.tri[2]];
    glm::vec3 bary = _surfaceMesh.getBarycentricCoordinates(tidx, p);

    return (float)bary.x*k0 + (float)bary.y*k1 + (float)bary.z*k2;
}

double LevelSet::getSurfaceCurvature(unsigned int tidx) {
    if (_vertexCurvatures.size() != _surfaceMesh.vertices.size()) {
        return 0.0;
    }
    if (tidx >= _surfaceMesh.triangles.size()) {
        return 0.0;
    }

    assert(_vertexCurvatures.size() == _surfaceMesh.vertices.size());
    assert(tidx < _surfaceMesh.triangles.size());

    Triangle t = _surfaceMesh.triangles[tidx];
    double k0 = _vertexCurvatures[t.tri[0]];
    double k1 = _vertexCurvatures[t.tri[1]];
    double k2 = _vertexCurvatures[t.tri[2]];

    return (k0 + k1 + k2) / 3.0;
}