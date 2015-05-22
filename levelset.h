#pragma once

#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <queue>

#include "glm/glm.hpp"
#include "Array3d.h"
#include "Collision.h"
#include "trianglemesh.h"
#include "macvelocityfield.h"
#include "levelsetfield.h"

class LevelSet
{
public:
    LevelSet();
    LevelSet(int i, int j, int k, double dx);
    ~LevelSet();

    void setSurfaceMesh(TriangleMesh mesh);
    void calculateSignedDistanceField();
    void calculateSignedDistanceField(int numLayers);
    Array3d<double> getSignedDistanceField() { return _signedDistance; }

    void advectSignedDistanceField(MACVelocityField &velocityGrid, double dt);

private:
    void _resetSignedDistanceField();
    void _calculateUnsignedSurfaceDistanceSquared();
    void _calculateDistancesSquaredForTriangle(int triangleIndex);
    void _getTriangleGridCellOverlap(Triangle t, std::vector<GridIndex> &cells);
    void _calculateUnsignedDistanceSquared();
    void _getCellLayers(std::vector<std::vector<GridIndex>> &layers);
    void _getNeighbourGridIndices6(GridIndex g, GridIndex n[6]);
    void _getLayerCells(int idx, std::vector<GridIndex> &layer, 
                                 std::vector<GridIndex> &nextLayer,
                                 Array3d<int> &layerGrid);
    void _convertToQueues(std::vector<std::vector<GridIndex>> &layers,
                          std::vector<std::queue<GridIndex>> &queues);
    void _calculateUnsignedDistanceSquaredForQueue(std::queue<GridIndex> &q);
    void _setLevelSetCell(GridIndex g, double dist, int tidx);
    void _resetLevelSetCell(GridIndex g);
    void _squareRootDistanceField();
    void _calculateDistanceFieldSigns();
    void _updateCellSign(GridIndex g, std::vector<glm::vec3> &triangleCenters, 
                                      std::vector<glm::vec3> &triangleDirections);

    void _advectCell(MACVelocityField &vgrid, Array3d<double> &tempGrid, GridIndex g, double dt);
    glm::vec3 _findClosestPointOnSurface(GridIndex g);
    glm::vec3 _findClosestPointOnSurface(glm::vec3 p);
    glm::vec3 _RK4(MACVelocityField &vgrid, glm::vec3 p0, glm::vec3 v0, double dt);
    glm::vec3 _evaluateVelocityAtPosition(MACVelocityField &vgrid, glm::vec3 p);
    glm::vec3 _evaluateVelocityAtGridIndex(MACVelocityField &vgrid, GridIndex g);
    double _minDistToTriangleSquared(glm::vec3 p, int tidx);
    double _minDistToTriangleSquared(glm::vec3 p, int tidx, glm::vec3 *point);
    glm::vec3 _backwardsAdvectVelocity(MACVelocityField &vgrid, glm::vec3 p0, glm::vec3 v0, double dt);
    bool _integrateVelocity(MACVelocityField &vgrid, 
                            glm::vec3 p0, glm::vec3 v0, double dt, glm::vec3 *p1);

    inline bool _isPointInsideSurface(glm::vec3 p) {
        return _distanceField.getFieldValue(p) > 0.0;
    }
    inline bool _isCellInsideSurface(GridIndex g) {
        assert(_isDistanceSet(g));
        return _signedDistance(g) > 0.0;
    }

    GridIndex _positionToGridIndex(glm::vec3 p) {
        double invdx = 1.0 / _dx;
        return GridIndex((int)floor(p.x*invdx),
                         (int)floor(p.y*invdx),
                         (int)floor(p.z*invdx));
    }
    glm::vec3 _gridIndexToPosition(GridIndex g) {
        assert(_isCellIndexInRange(g));
        return glm::vec3((double)g.i*_dx, (double)g.j*_dx, (double)g.k*_dx);
    }
    glm::vec3 _gridIndexToCellCenter(GridIndex g) {
        assert(_isCellIndexInRange(g));
        double hd = 0.5*_dx;
        return glm::vec3((double)g.i*_dx + hd, (double)g.j*_dx + hd, (double)g.k*_dx + hd);
    }
    inline bool _isCellIndexInRange(GridIndex g) {
        return g.i >= 0 && g.j >= 0 && g.k >= 0 && 
               g.i < _isize && g.j < _jsize && g.k < _ksize;
    }

    double _dx = 0.0;
    double _isize = 0;
    double _jsize = 0;
    double _ksize = 0;
    double _numLayers;

    TriangleMesh _surfaceMesh;

    Array3d<double> _signedDistance;
    Array3d<int> _indexGrid;
    Array3d<bool> _isDistanceSet;

    LevelSetField _distanceField;
};

