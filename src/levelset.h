/*
Copyright (c) 2016 Ryan L. Guy

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
#ifndef LEVELSET_H
#define LEVELSET_H

#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <queue>

#include "vmath.h"
#include "array3d.h"
#include "grid3d.h"
#include "interpolation.h"
#include "collision.h"
#include "trianglemesh.h"
#include "macvelocityfield.h"
#include "gridindexvector.h"
#include "fluidsimassert.h"

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
    double getSurfaceCurvature(vmath::vec3 p);
    double getSurfaceCurvature(vmath::vec3 p, vmath::vec3 *normal);
    double getSurfaceCurvature(unsigned int tidx);
    Array3d<float> getSignedDistanceField() { return _signedDistance; }
    vmath::vec3 getClosestPointOnSurface(vmath::vec3 p);
    vmath::vec3 getClosestPointOnSurface(vmath::vec3 p, int *tidx);
    double getDistance(vmath::vec3 p);
    double getSignedDistance(vmath::vec3 p);
    double getDistance(GridIndex g);
    double getSignedDistance(GridIndex g);
    bool isPointInInsideCell(vmath::vec3 p);

private:
    void _resetSignedDistanceField();
    void _calculateUnsignedSurfaceDistanceSquared();
    void _calculateDistancesSquaredForTriangle(int triangleIndex);
    void _getTriangleGridCellOverlap(Triangle t, GridIndexVector &cells);
    void _calculateUnsignedDistanceSquared();
    void _getCellLayers(std::vector<GridIndexVector> &layers);
    void _getNeighbourGridIndices6(GridIndex g, GridIndex n[6]);
    void _getLayerCells(int idx, GridIndexVector &layer, 
                                 GridIndexVector &nextLayer,
                                 Array3d<int> &layerGrid);
    void _calculateUnsignedDistanceSquaredForLayer(GridIndexVector &q);
    void _setLevelSetCell(GridIndex g, double dist, int tidx);
    void _resetLevelSetCell(GridIndex g);
    void _squareRootDistanceField();
    void _calculateDistanceFieldSigns();
    void _updateCellSign(GridIndex g, std::vector<vmath::vec3> &triangleCenters, 
                                      std::vector<vmath::vec3> &triangleDirections);
    void _floodFillMissingSignedDistances();
    void _floodFillWithDistance(GridIndex g, double val);

    vmath::vec3 _findClosestPointOnSurface(GridIndex g);
    vmath::vec3 _findClosestPointOnSurface(vmath::vec3 p);
    vmath::vec3 _findClosestPointOnSurface(vmath::vec3 p, int *tidx);
    vmath::vec3 _evaluateVelocityAtPosition(MACVelocityField &vgrid, vmath::vec3 p);
    vmath::vec3 _evaluateVelocityAtGridIndex(MACVelocityField &vgrid, GridIndex g);
    double _minDistToTriangleSquared(vmath::vec3 p, int tidx);
    double _minDistToTriangleSquared(GridIndex g, int tidx);
    double _minDistToTriangleSquared(vmath::vec3 p, int tidx, vmath::vec3 *point);

    double _calculateCurvatureAtVertex(int idx);
    void _getCurvatureSamplePoints(int vidx, std::vector<vmath::vec3> &points,
                                             std::vector<int> &tris);
    void _getTrianglePatch(int vertexIndex, double *area, std::vector<int> &tris);
    int _getRandomTriangle(std::vector<int> &tris, std::vector<double> &distribution);
    vmath::vec3 _getRandomPointInTriangle(int tidx);

    double _linearInterpolateSignedDistance(vmath::vec3 p);
    double _cubicInterpolateSignedDistance(vmath::vec3 p);

    bool _isPointInsideSurface(vmath::vec3 p);
    bool _isCellInsideSurface(GridIndex g);
    bool _isCellInsideSurface(int i, int j, int k);

    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;
    double _dx = 0.0;
    int _numLayers;

    TriangleMesh _surfaceMesh;

    Array3d<float> _signedDistance;
    Array3d<int> _indexGrid;
    Array3d<bool> _isDistanceSet;

    std::vector<double> _vertexCurvatures;
    double _surfaceCurvatureSampleRadius = 6.0;  // radius in # of cells
    int _maxSurfaceCurvatureSamples = 40;
    Array3d<bool> _triangleHash;
    
};

#endif
