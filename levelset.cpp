#include "levelset.h"

LevelSet::LevelSet()
{
}

LevelSet::LevelSet(int i, int j, int k, double dx) : 
                                 _isize(i), _jsize(j), _ksize(k), _dx(dx),
                                 _signedDistance(Array3d<double>(i, j, k, 0.0)),
                                 _indexGrid(Array3d<int>(i, j, k, -1)),
                                 _isDistanceSet(Array3d<bool>(i, j, k, false)){
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
    for (int i = 0; i < testcells.size(); i++) {
        cbbox.position = _gridIndexToPosition(testcells[i]);
        if (cbbox.isOverlappingTriangle(t, _surfaceMesh.vertices)) {
            cells.push_back(testcells[i]);
        }
    }
}

void LevelSet::_calculateDistancesForTriangle(int index) {
    Triangle t = _surfaceMesh.triangles[index];
    glm::vec3 tri[3];
    _surfaceMesh.getTrianglePosition(index, tri);

    std::vector<GridIndex> cells;
    _getTriangleGridCellOverlap(t, cells);

    GridIndex g;
    glm::vec3 p, d;
    double dist;
    for (int i = 0; i < cells.size(); i++) {
        g = cells[i];
        p = _gridIndexToCellCenter(g);
        d = Collision::findClosestPointOnTriangle(p, tri[0], tri[1], tri[2]);
        dist = glm::length(d - p);

        if (!_isDistanceSet(g) || dist < _signedDistance(g)) {
            _setLevelSetCell(g, dist, index);
        }
    }
}

void LevelSet::_calculateUnsignedSurfaceDistance() {
    for (int i = 0; i < _surfaceMesh.triangles.size(); i++) {
        _calculateDistancesForTriangle(i);
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
    for (int i = 0; i < layer.size(); i++) {
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

    for (int i = 2; layers[i-2].size() > 0; i++) {
        std::vector<GridIndex> q;
        layers.push_back(q);
        _getLayerCells(i, layers[i - 2], layers[i - 1], layerGrid);
    }
    layers.pop_back();
}

void LevelSet::_convertToQueues(std::vector<std::vector<GridIndex>> &layers,
                      std::vector<std::queue<GridIndex>> &queues) {
    for (int j = 0; j < layers.size(); j++) {
        std::queue<GridIndex> q;
        queues.push_back(q);
        for (int i = 0; i < layers[j].size(); i++) {
            queues[j].push(layers[j][i]);
        }
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

void LevelSet::_calculateUnsignedDistanceForQueue(std::queue<GridIndex> &q) {

    GridIndex g, n;
    GridIndex ns[6];
    glm::vec3 p, d;
    glm::vec3 tri[3];
    double dist;

    while (!q.empty()) {
        g = q.front();
        q.pop();

        _getNeighbourGridIndices6(g, ns);
        for (int j = 0; j < 6; j++) {
            n = ns[j];
            if (_isCellIndexInRange(n) && _isDistanceSet(n)) {
                int tidx = _indexGrid(n);
                _surfaceMesh.getTrianglePosition(tidx, tri);
                p = _gridIndexToCellCenter(g);
                d = Collision::findClosestPointOnTriangle(p, tri[0], tri[1], tri[2]);
                dist = glm::length(d - p);

                if (!_isDistanceSet(g) || dist < _signedDistance(g)) {
                    _setLevelSetCell(g, dist, tidx);
                }

                if (dist < _signedDistance(n)) {
                    _resetLevelSetCell(n);
                    q.push(n);
                }
            }
        }

        if (!_isDistanceSet(g)) {
            q.push(g);
        }
    }
}

void LevelSet::_calculateUnsignedDistance() {
    std::vector<std::vector<GridIndex>> cellLayers;
    _getCellLayers(cellLayers);

    std::vector<std::queue<GridIndex>> cellQueues;
    _convertToQueues(cellLayers, cellQueues);
    cellLayers.clear();

    for (int i = 0; i < cellQueues.size(); i++) {
        _calculateUnsignedDistanceForQueue(cellQueues[i]);
    }
}

void LevelSet::_updateCellSign(GridIndex g) {
    // Use this convention:
    //     outside mesh, signed distance is negative
    //     inside mesh, signed distance is positive

    int tidx = _indexGrid(g);
    glm::vec3 p = _gridIndexToCellCenter(g);
    glm::vec3 ct = _surfaceMesh.getTriangleCenter(tidx);
    glm::vec3 n = _surfaceMesh.getTriangleNormal(tidx);
    glm::vec3 v = ct - p;

    if (glm::dot(v, n) < 0) {
        _signedDistance.set(g, -1.0*_signedDistance(g));
    }
}

void LevelSet::_calculateDistanceFieldSigns() {
    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                if (_isDistanceSet(i, j, k)) {
                    _updateCellSign(GridIndex(i, j, k));
                }
            }
        }
    }
}

void LevelSet::calculateSignedDistanceField() {
    _resetSignedDistanceField();
    _calculateUnsignedSurfaceDistance();
    _calculateUnsignedDistance();
    _calculateDistanceFieldSigns();
}