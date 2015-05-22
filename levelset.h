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

class LevelSet
{
public:
    LevelSet();
    LevelSet(int i, int j, int k, double dx);
    ~LevelSet();

    void setSurfaceMesh(TriangleMesh mesh);
    void calculateSignedDistanceField();
    Array3d<double> getSignedDistanceField() { return _signedDistance; }

private:
    void _resetSignedDistanceField();
    void _calculateUnsignedSurfaceDistance();
    void _calculateDistancesForTriangle(int triangleIndex);
    void _getTriangleGridCellOverlap(Triangle t, std::vector<GridIndex> &cells);
    void _calculateUnsignedDistance();
    void _getCellLayers(std::vector<std::vector<GridIndex>> &layers);
    void _getNeighbourGridIndices6(GridIndex g, GridIndex n[6]);
    void _getLayerCells(int idx, std::vector<GridIndex> &layer, 
                                 std::vector<GridIndex> &nextLayer,
                                 Array3d<int> &layerGrid);
    void _convertToQueues(std::vector<std::vector<GridIndex>> &layers,
                          std::vector<std::queue<GridIndex>> &queues);
    void _calculateUnsignedDistanceForQueue(std::queue<GridIndex> &q);
    void _setLevelSetCell(GridIndex g, double dist, int tidx);
    void _resetLevelSetCell(GridIndex g);
    void _calculateDistanceFieldSigns();
    void _updateCellSign(GridIndex g);



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

    TriangleMesh _surfaceMesh;

    Array3d<double> _signedDistance;
    Array3d<int> _indexGrid;
    Array3d<bool> _isDistanceSet;
};

