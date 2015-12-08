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
#include "levelset.h"

LevelSet::LevelSet()
{
}

LevelSet::LevelSet(int i, int j, int k, double dx) : 
                                 _isize(i), _jsize(j), _ksize(k), _dx(dx),
                                 _signedDistance(Array3d<float>(i, j, k, 0.0f)),
                                 _indexGrid(Array3d<int>(i, j, k, -1)),
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
    for (unsigned int i = 0; i < testcells.size(); i++) {
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
    for (unsigned int i = 0; i < cells.size(); i++) {
        g = cells[i];
        distsq = _minDistToTriangleSquared(g, index);

        if (!_isDistanceSet(g) || distsq < _signedDistance(g)) {
            _setLevelSetCell(g, distsq, index);
        }
    }
}

void LevelSet::_calculateUnsignedSurfaceDistanceSquared() {
    for (unsigned int i = 0; i < _surfaceMesh.triangles.size(); i++) {
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
                                       Array3d<int> &layerGrid) {
    GridIndex ns[6];
    GridIndex g, n;
    for (unsigned int i = 0; i < layer.size(); i++) {
        g = layer[i];
        _getNeighbourGridIndices6(g, ns);
        for (int j = 0; j < 6; j++) {
            n = ns[j];
            if (Grid3d::isGridIndexInRange(n, _isize, _jsize, _ksize) && 
                    layerGrid(n) == -1) {
                nextLayer.push_back(n);
                layerGrid.set(n, idx);
            }
        }
    }
}

void LevelSet::_getCellLayers(std::vector<std::vector<GridIndex>> &layers) {
    Array3d<int> layerGrid(_isize, _jsize, _ksize, -1);

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
    _signedDistance.set(g, (float)dist);
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
            if (Grid3d::isGridIndexInRange(n, _isize, _jsize, _ksize) && 
                    _isDistanceSet(n)) {
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

    for (unsigned int i = 0; i < cellLayers.size(); i++) {
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
        _signedDistance.set(g, (float)-1.0f*_signedDistance(g));
    }
}

void LevelSet::_calculateDistanceFieldSigns() {
    std::vector<glm::vec3> triangleFaceDirections;
    std::vector<glm::vec3> triangleFaceCenters;
    triangleFaceDirections.reserve(_surfaceMesh.triangles.size());
    triangleFaceCenters.reserve(_surfaceMesh.triangles.size());

    for (unsigned int i = 0; i < _surfaceMesh.triangles.size(); i++) {
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

void LevelSet::_floodFillWithDistance(GridIndex seed, double val) {
    std::vector<GridIndex> queue;
    queue.push_back(seed);
    _isDistanceSet.set(seed, true);

    GridIndex g;
    GridIndex ns[6];
    while (!queue.empty()) {
        g = queue[queue.size() - 1];
        queue.pop_back();

        Grid3d::getNeighbourGridIndices6(g, ns);
        for (int i = 0; i < 6; i++) {
            if (Grid3d::isGridIndexInRange(ns[i], _isize, _jsize, _ksize) && 
                    !_isDistanceSet(ns[i])) {
                queue.push_back(ns[i]);
                _isDistanceSet.set(ns[i], true);
            }
        }

        _setLevelSetCell(g, val, -1);
    }
}

void LevelSet::_floodFillMissingSignedDistances() {
    GridIndex ns[6];
    GridIndex n;
    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                if (_isDistanceSet(i, j, k)) {
                    Grid3d::getNeighbourGridIndices6(i, j, k, ns);
                    for (int idx = 0; idx < 6; idx++) {
                        n = ns[idx];
                        if (Grid3d::isGridIndexInRange(n, _isize, _jsize, _ksize) && 
                                !_isDistanceSet(n)) {
                            double dist;
                            if (_signedDistance(i, j, k) > 0.0) {
                                dist = std::numeric_limits<double>::infinity();
                            } else {
                                dist = -std::numeric_limits<double>::infinity();
                            }
                            _floodFillWithDistance(n, dist);
                        }
                    }
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
    _floodFillMissingSignedDistances();

    _distanceField.setSignedDistanceField(_signedDistance);
}

double LevelSet::_minDistToTriangleSquared(GridIndex g, int tidx) {
    glm::vec3 p = _gridIndexToCellCenter(g);
    return _minDistToTriangleSquared(p, tidx);
}

double LevelSet::_minDistToTriangleSquared(glm::vec3 p, int tidx) {
    if (tidx < 0 || (unsigned int)tidx > _surfaceMesh.triangles.size()) {
        return std::numeric_limits<double>::infinity();
    }

    glm::vec3 tri[3];
    _surfaceMesh.getTrianglePosition(tidx, tri);
    glm::vec3 tp = Collision::findClosestPointOnTriangle(p, tri[0], tri[1], tri[2]);
    glm::vec3 v = tp - p;
    return glm::dot(v, v);
}

double LevelSet::_minDistToTriangleSquared(glm::vec3 p, int tidx, glm::vec3 *point) {
    if (tidx < 0 || (unsigned int)tidx > _surfaceMesh.triangles.size()) {
        return std::numeric_limits<double>::infinity();
    }

    glm::vec3 tri[3];
    _surfaceMesh.getTrianglePosition(tidx, tri);
    glm::vec3 tp = Collision::findClosestPointOnTriangle(p, tri[0], tri[1], tri[2]);
    glm::vec3 v = tp - p;

    *point = tp;
    return glm::dot(v, v);
}

glm::vec3 LevelSet::_findClosestPointOnSurface(GridIndex g) {
    assert(Grid3d::isGridIndexInRange(g, _isize, _jsize, _ksize) && _isDistanceSet(g));

    glm::vec3 tri[3];
    _surfaceMesh.getTrianglePosition(_indexGrid(g), tri);
    glm::vec3 p0 = _gridIndexToCellCenter(g);
    
    return Collision::findClosestPointOnTriangle(p0, tri[0], tri[1], tri[2]);
}

glm::vec3 LevelSet::_findClosestPointOnSurface(glm::vec3 p) {
    GridIndex g = Grid3d::positionToGridIndex(p, _dx);

    GridIndex ns[7];
    _getNeighbourGridIndices6(g, ns);
    ns[6] = g;

    double mindistsq = std::numeric_limits<double>::infinity();
    glm::vec3 minpoint = p;
    GridIndex n;
    for (int i = 0; i < 7; i++) {
        n = ns[i];

        if (Grid3d::isGridIndexInRange(g, _isize, _jsize, _ksize) && 
                _isDistanceSet(n)) {
            glm::vec3 surfacePoint;
            int tidx = _indexGrid(n);

            if (tidx < 0) {
                continue;
            }

            double distsq = _minDistToTriangleSquared(p, tidx, &surfacePoint);

            if (distsq < mindistsq) {
                mindistsq = distsq;
                minpoint = surfacePoint;
            }
        }
    }

    return minpoint;
}

glm::vec3 LevelSet::_findClosestPointOnSurface(glm::vec3 p, int *tidx) {
    GridIndex g = Grid3d::positionToGridIndex(p, _dx);

    GridIndex ns[7];
    _getNeighbourGridIndices6(g, ns);
    ns[6] = g;

    double mindistsq = std::numeric_limits<double>::infinity();
    glm::vec3 minpoint = p;
    int mint = -1;
    GridIndex n;
    for (int i = 0; i < 7; i++) {
        n = ns[i];

        if (Grid3d::isGridIndexInRange(g, _isize, _jsize, _ksize) && 
                _isDistanceSet(n)) {
            glm::vec3 surfacePoint;
            double distsq = _minDistToTriangleSquared(p, _indexGrid(n), &surfacePoint);

            if (distsq < mindistsq) {
                mindistsq = distsq;
                minpoint = surfacePoint;
                mint = _indexGrid(n);
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

glm::vec3 LevelSet::getClosestPointOnSurface(glm::vec3 p, int *tidx) {
    return _findClosestPointOnSurface(p, tidx);
}

void LevelSet::_getTrianglePatch(int vertidx, double *area, std::vector<int> &tris) {
    std::vector<int> nbs;
    std::vector<int> processedTriangles;
    _surfaceMesh.getVertexNeighbours(vertidx, nbs);

    std::queue<int> queue;
    for (unsigned int i = 0; i < nbs.size(); i++) {
        queue.push(nbs[i]);
        _triangleHash.set(nbs[i], 0, 0, true);
        processedTriangles.push_back(nbs[i]);
    }
    nbs.clear();

    double totalarea = 0.0;
    while (!queue.empty() && totalarea < *area) {
        int t = queue.front();
        queue.pop();

        _surfaceMesh.getFaceNeighbours(t, nbs);
        for (unsigned int i = 0; i < nbs.size(); i++) {
            if (!_triangleHash(nbs[i], 0, 0)) {
                queue.push(nbs[i]);
                _triangleHash.set(nbs[i], 0, 0, true);
                processedTriangles.push_back(nbs[i]);
            }
        }
        nbs.clear();

        totalarea += _surfaceMesh.getTriangleArea(t);
        tris.push_back(t);
    }

    for (unsigned int i = 0; i < processedTriangles.size(); i++) {
        _triangleHash.set(processedTriangles[i], 0, 0, false);
    }

    *area = totalarea;
}

int LevelSet::_getRandomTriangle(std::vector<int> &tris, 
                                 std::vector<double> &distribution) {
    if (tris.size() == 1) {
        return tris[0];
    }

    float r = (float)rand() / (float)(RAND_MAX);
    for (unsigned int i = 0; i < tris.size()-1; i++) {
        if (r >= distribution[i] && r < distribution[i + 1]) {
            return tris[i];
        }
    }

    return tris.size() - 1;
}

glm::vec3 LevelSet::_getRandomPointInTriangle(int tidx) {
    Triangle t = _surfaceMesh.triangles[tidx];
    glm::vec3 A = _surfaceMesh.vertices[t.tri[0]];
    glm::vec3 B = _surfaceMesh.vertices[t.tri[1]];
    glm::vec3 C = _surfaceMesh.vertices[t.tri[2]];

    float r1 = (float)rand() / (float)(RAND_MAX);
    float r2 = (float)rand() / (float)(RAND_MAX);
    float sqrtr1 = sqrt(r1);

    return (1.0f - sqrtr1)*A + (sqrtr1*(1.0f - r2))*B + (r2*sqrtr1)*C;
}

void LevelSet::_getCurvatureSamplePoints(int vidx, std::vector<glm::vec3> &points,
                                                   std::vector<int> &tris) {
    double r = _surfaceCurvatureSampleRadius*_dx;
    double area = 3.141592653*r*r;
    std::vector<int> patch;
    _getTrianglePatch(vidx, &area, patch);

    if (patch.size() == 0) {
        return;
    }

    std::vector<double> areaDistribution;
    areaDistribution.reserve(patch.size());

    double currentArea = 0.0;
    for (unsigned int i = 0; i < patch.size(); i++) {
        areaDistribution.push_back(currentArea);
        currentArea += _surfaceMesh.getTriangleArea(patch[i]) / area;
    }

    int maxSamples = (int)fmin(_maxSurfaceCurvatureSamples, patch.size());
    glm::vec3 p;
    for (int i = 0; i < maxSamples; i++) {
        int tidx = _getRandomTriangle(patch, areaDistribution);
        p = _getRandomPointInTriangle(tidx);
        tris.push_back(tidx);
        points.push_back(p);
    }
}

double LevelSet::_calculateCurvatureAtVertex(int idx) {

    std::vector<glm::vec3> samples;
    std::vector<int> sampletris;
    _getCurvatureSamplePoints(idx, samples, sampletris);

    double radius = _surfaceCurvatureSampleRadius*_dx;
    double rsq = radius*radius;
    glm::vec3 vnorm = _surfaceMesh.normals[idx];
    glm::vec3 p, tnorm, xji;
    glm::vec3 v = _surfaceMesh.vertices[idx];
    int tidx;
    double curvature = 0.0;
    for (unsigned int i = 0; i < samples.size(); i++) {
        p = samples[i];
        tidx = sampletris[i];

        xji = p - v;
        double distsq = glm::dot(xji, xji);
        if (distsq < rsq) {
            tnorm = _surfaceMesh.getTriangleNormalSmooth(tidx, p);
            double k = (1 - glm::dot(vnorm, tnorm)) * (1 - (sqrt(distsq)/radius));
            if (glm::dot(glm::normalize(xji), vnorm) > 0.0) {
                k = 0.0;
            }

            curvature += k;
        }
    }

    return curvature;
}

void LevelSet::calculateSurfaceCurvature() {
    _surfaceMesh.updateVertexTriangles();
    _surfaceMesh.updateTriangleAreas();
    _triangleHash = Array3d<bool>(_surfaceMesh.triangles.size(), 1, 1, false);

    _vertexCurvatures.clear();
    for (unsigned int i = 0; i < _surfaceMesh.vertices.size(); i++) {
        double k = _calculateCurvatureAtVertex(i);
        _vertexCurvatures.push_back(k);
    }

    _triangleHash = Array3d<bool>(0, 0, 0);
    _surfaceMesh.clearTriangleAreas();
    _surfaceMesh.clearVertexTriangles();
}

double LevelSet::getSurfaceCurvature(glm::vec3 p) {
    glm::vec3 n;
    return getSurfaceCurvature(p, &n);
}

double LevelSet::getSurfaceCurvature(glm::vec3 p, glm::vec3 *normal) {
    assert(_vertexCurvatures.size() == _surfaceMesh.vertices.size());

    int tidx;
    p = _findClosestPointOnSurface(p, &tidx);

    if (tidx < 0 || (unsigned int)tidx >= _surfaceMesh.triangles.size()) {
        *normal = glm::vec3(1.0, 0.0, 0.0);
        return 0.0;
    }

    Triangle t = _surfaceMesh.triangles[tidx];
    double k0 = _vertexCurvatures[t.tri[0]];
    double k1 = _vertexCurvatures[t.tri[1]];
    double k2 = _vertexCurvatures[t.tri[2]];
    glm::vec3 bary = _surfaceMesh.getBarycentricCoordinates(tidx, p);

    *normal = _surfaceMesh.getTriangleNormalSmooth(tidx, p);

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

double LevelSet::_trilinearInterpolate(double p[8], double x, double y, double z) {
    return p[0] * (1 - x) * (1 - y) * (1 - z) +
           p[1] * x * (1 - y) * (1 - z) + 
           p[2] * (1 - x) * y * (1 - z) + 
           p[3] * (1 - x) * (1 - y) * z +
           p[4] * x * (1 - y) * z + 
           p[5] * (1 - x) * y * z + 
           p[6] * x * y * (1 - z) + 
           p[7] * x * y * z;
}

double LevelSet::_interpolateSignedDistance(glm::vec3 p) {
    p -= glm::vec3(0.5*_dx, 0.5*_dx, 0.5*_dx);

    GridIndex g = Grid3d::positionToGridIndex(p, _dx);
    glm::vec3 gpos = Grid3d::GridIndexToPosition(g, _dx);

    double inv_dx = 1 / _dx;
    double ix = (p.x - gpos.x)*inv_dx;
    double iy = (p.y - gpos.y)*inv_dx;
    double iz = (p.z - gpos.z)*inv_dx;

    //assert(ix >= 0 && ix < 1 && iy >= 0 && iy < 1 && iz >= 0 && iz < 1);

    double points[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (Grid3d::isGridIndexInRange(g.i,   g.j,   g.k, _isize, _jsize, _ksize))   { 
        points[0] = _signedDistance(g.i,   g.j,   g.k); 
    }
    if (Grid3d::isGridIndexInRange(g.i+1, g.j,   g.k, _isize, _jsize, _ksize))   { 
        points[1] = _signedDistance(g.i+1, g.j,   g.k); 
    }
    if (Grid3d::isGridIndexInRange(g.i,   g.j+1, g.k, _isize, _jsize, _ksize))   { 
        points[2] = _signedDistance(g.i,   g.j+1, g.k); 
    }
    if (Grid3d::isGridIndexInRange(g.i,   g.j,   g.k+1, _isize, _jsize, _ksize)) {
        points[3] = _signedDistance(g.i,   g.j,   g.k+1); 
    }
    if (Grid3d::isGridIndexInRange(g.i+1, g.j,   g.k+1, _isize, _jsize, _ksize)) { 
        points[4] = _signedDistance(g.i+1, g.j,   g.k+1); 
    }
    if (Grid3d::isGridIndexInRange(g.i,   g.j+1, g.k+1, _isize, _jsize, _ksize)) { 
        points[5] = _signedDistance(g.i,   g.j+1, g.k+1); 
    }
    if (Grid3d::isGridIndexInRange(g.i+1, g.j+1, g.k, _isize, _jsize, _ksize))   { 
        points[6] = _signedDistance(g.i+1, g.j+1, g.k); 
    }
    if (Grid3d::isGridIndexInRange(g.i+1, g.j+1, g.k+1, _isize, _jsize, _ksize)) { 
        points[7] = _signedDistance(g.i+1, g.j+1, g.k+1); 
    }

    return _trilinearInterpolate(points, ix, iy, iz);
}

double LevelSet::getDistance(glm::vec3 p) {
    GridIndex g = Grid3d::positionToGridIndex(p, _dx);
    assert(Grid3d::isGridIndexInRange(g, _isize, _jsize, _ksize));

    if (!_isDistanceSet(g)) {
        return std::numeric_limits<double>::infinity();
    }

    return fabs(_interpolateSignedDistance(p));
}

double LevelSet::getSignedDistance(glm::vec3 p) {
    GridIndex g = Grid3d::positionToGridIndex(p, _dx);
    assert(Grid3d::isGridIndexInRange(g, _isize, _jsize, _ksize));

    if (!_isDistanceSet(g)) {
        return std::numeric_limits<double>::infinity();
    }

    return _interpolateSignedDistance(p);
}

double LevelSet::getDistance(GridIndex g) {
    assert(Grid3d::isGridIndexInRange(g, _isize, _jsize, _ksize));

    if (!_isDistanceSet(g)) {
        return std::numeric_limits<double>::infinity();
    }

    return fabs(_signedDistance(g));
}

double LevelSet::getSignedDistance(GridIndex g) {
    assert(Grid3d::isGridIndexInRange(g, _isize, _jsize, _ksize));

    if (!_isDistanceSet(g)) {
        return std::numeric_limits<double>::infinity();
    }

    return _signedDistance(g);
}

bool LevelSet::isPointInInsideCell(glm::vec3 p) {
    assert(Grid3d::isPositionInGrid(p, _dx, _isize, _jsize, _ksize));

    GridIndex g = Grid3d::positionToGridIndex(p, _dx);
    assert(_isDistanceSet(g));

    return _signedDistance(g) > 0.0;
}