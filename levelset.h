#pragma once

#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <queue>

#include "glm/glm.hpp"
#include "array3d.h"
#include "grid3d.h"
#include "collision.h"
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
    void calculateSurfaceCurvature();
    double getSurfaceCurvature(glm::vec3 p);
    double getSurfaceCurvature(unsigned int tidx);
    Array3d<double> getSignedDistanceField() { return _signedDistance; }
    glm::vec3 getClosestPointOnSurface(glm::vec3 p);
    double getDistance(glm::vec3 p);

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
    void _calculateUnsignedDistanceSquaredForLayer(std::vector<GridIndex> &q);
    void _setLevelSetCell(GridIndex g, double dist, int tidx);
    void _resetLevelSetCell(GridIndex g);
    void _squareRootDistanceField();
    void _calculateDistanceFieldSigns();
    void _updateCellSign(GridIndex g, std::vector<glm::vec3> &triangleCenters, 
                                      std::vector<glm::vec3> &triangleDirections);

    glm::vec3 _findClosestPointOnSurface(GridIndex g);
    glm::vec3 _findClosestPointOnSurface(glm::vec3 p);
    glm::vec3 _findClosestPointOnSurface(glm::vec3 p, int *tidx);
    glm::vec3 _evaluateVelocityAtPosition(MACVelocityField &vgrid, glm::vec3 p);
    glm::vec3 _evaluateVelocityAtGridIndex(MACVelocityField &vgrid, GridIndex g);
    double _minDistToTriangleSquared(glm::vec3 p, int tidx);
    double _minDistToTriangleSquared(GridIndex g, int tidx);
    double _minDistToTriangleSquared(glm::vec3 p, int tidx, glm::vec3 *point);

    double _calculateCurvatureAtVertex(int idx);
    void _getCurvatureSamplePoints(int vidx, std::vector<glm::vec3> &points,
                                             std::vector<int> &tris);
    void _getTrianglePatch(int vertexIndex, double *area, std::vector<int> &tris);
    int _getRandomTriangle(std::vector<int> &tris, std::vector<double> &distribution);
    glm::vec3 _getRandomPointInTriangle(int tidx);

    double _interpolateSignedDistance(glm::vec3 p);
    double _trilinearInterpolate(double points[8], 
                                 double ix, double iy, double iz);

    inline bool _isPointInsideSurface(glm::vec3 p) {
        return _distanceField.getFieldValue(p) > 0.0;
    }
    inline bool _isCellInsideSurface(GridIndex g) {
        assert(_isDistanceSet(g));
        return _signedDistance(g) > 0.0;
    }
    inline bool _isCellInsideSurface(int i, int j, int k) {
        assert(_isDistanceSet(i, j, k));
        return _signedDistance(i, j, k) > 0.0;
    }

    glm::vec3 _gridIndexToPosition(GridIndex g) {
        assert(Grid3d::isGridIndexInRange(g, _isize, _jsize, _ksize));
        return Grid3d::GridIndexToPosition(g, _dx);
    }
    glm::vec3 _gridIndexToCellCenter(GridIndex g) {
        assert(Grid3d::isGridIndexInRange(g, _isize, _jsize, _ksize));
        return Grid3d::GridIndexToCellCenter(g, _dx);
    }
    glm::vec3 _gridIndexToCellCenter(int i, int j, int k) {
        assert(Grid3d::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize));
        return Grid3d::GridIndexToCellCenter(i, j, k, _dx);
    }

    double _dx = 0.0;
    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;
    int _numLayers;

    TriangleMesh _surfaceMesh;

    Array3d<double> _signedDistance;
    Array3d<int> _indexGrid;
    Array3d<bool> _isDistanceSet;

    LevelSetField _distanceField;

    std::vector<double> _vertexCurvatures;
    double _surfaceCurvatureSampleRadius = 3.0;  // radius in # of cells
    int _maxSurfaceCurvatureSamples = 40;
    Array3d<bool> _triangleHash;
    
};

