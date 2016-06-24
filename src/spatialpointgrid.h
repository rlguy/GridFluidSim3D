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
#ifndef SPATIALPOINTGRID_H
#define SPATIALPOINTGRID_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <algorithm>

#include "array3d.h"
#include "aabb.h"
#include "fragmentedvector.h"
#include "grid3d.h"
#include "vmath.h"
#include "fluidsimassert.h"

struct GridPointReference {
    int id;

    GridPointReference() : id(-1) {}
    GridPointReference(int n) : id(n) {}

    bool operator==(const GridPointReference &other) const { 
        return id == other.id;
    }
};

struct GridPoint {
    vmath::vec3 position;
    GridPointReference ref;

    GridPoint() {}
    GridPoint(vmath::vec3 p, GridPointReference r) : position(p), ref(r) {} 
    GridPoint(vmath::vec3 p, unsigned int id) : position(p), ref(id) {}
};

class SpatialPointGrid
{
public:
    SpatialPointGrid();
    SpatialPointGrid(int isize, int jsize, int ksize, double _dx);
    ~SpatialPointGrid();

    void clear();
    std::vector<GridPointReference> insert(std::vector<vmath::vec3> &points);
    std::vector<GridPointReference> insert(FragmentedVector<vmath::vec3> &points);
    void queryPointsInsideSphere(vmath::vec3 p, double r, std::vector<vmath::vec3> &points);
    void queryPointsInsideSphere(GridPointReference ref, double r, std::vector<vmath::vec3> &points);
    void queryPointsInsideSphere(vmath::vec3 p, double r, std::vector<bool> &exclusions, 
                                                        std::vector<vmath::vec3> &points);
    void queryPointsInsideSphere(GridPointReference ref, double r, 
                                 std::vector<bool> &exclusions, std::vector<vmath::vec3> &points);
    void queryPointReferencesInsideSphere(vmath::vec3 p, double r, 
                                          std::vector<GridPointReference> &refs);
    void queryPointReferencesInsideSphere(GridPointReference ref, double r, 
                                          std::vector<GridPointReference> &refs);
    void queryPointReferencesInsideSphere(vmath::vec3 p, double r, 
                                          std::vector<bool> &exclusions, 
                                          std::vector<GridPointReference> &refs);
    void queryPointReferencesInsideSphere(GridPointReference ref, double r, 
                                          std::vector<bool> &exclusions,
                                          std::vector<GridPointReference> &refs);

    void queryPointsInsideAABB(AABB bbox, std::vector<vmath::vec3> &points);
    void queryPointReferencesInsideAABB(AABB bbox, std::vector<GridPointReference> &refs);

    void getConnectedPoints(vmath::vec3 seed, double radius, std::vector<vmath::vec3> &points);
    void getConnectedPointReferences(vmath::vec3 seed, double radius, std::vector<GridPointReference> &refs);
    void getConnectedPoints(GridPointReference seed, double radius, std::vector<vmath::vec3> &points);
    void getConnectedPointReferences(GridPointReference seed, double radius, std::vector<GridPointReference> &refs);
    void getConnectedPointComponents(double radius, std::vector<std::vector<vmath::vec3> > &points);
    void getConnectedPointReferenceComponents(double radius, std::vector<std::vector<GridPointReference> > &refs);

    vmath::vec3 getPointFromReference(GridPointReference ref);

private:

    struct CellNode {
        int start;
        int count;

        CellNode() : start(-1), count(-1) {}
        CellNode(int startIndex, int numPoints) : start(startIndex), count(numPoints) {}
    };

    inline unsigned int _getFlatIndex(int i, int j, int k) {
        return (unsigned int)i + (unsigned int)_isize *
               ((unsigned int)j + (unsigned int)_jsize * (unsigned int)k);
    }

    inline unsigned int _getFlatIndex(GridIndex g) {
        return (unsigned int)g.i + (unsigned int)_isize *
               ((unsigned int)g.j + (unsigned int)_jsize * (unsigned int)g.k);
    }

    void _sortGridPointsByGridIndex(std::vector<vmath::vec3> &points,
                                    std::vector<GridPoint> &sortedPoints,
                                    std::vector<GridPointReference> &refList);
    void _updateRefIDToGridPointIndexTable();
    void _insertCellNodesIntoGrid();
    void _queryPointsInsideSphere(vmath::vec3 p, double r, int refID, std::vector<vmath::vec3> &points);
    void _queryPointsInsideSphere(vmath::vec3 p, double r, 
                                  std::vector<bool> &exclusions, 
                                  std::vector<vmath::vec3> &points);
    void _queryPointReferencesInsideSphere(vmath::vec3 p, double r, int refID, 
                                           std::vector<GridPointReference> &refs);
    void _queryPointReferencesInsideSphere(vmath::vec3 p, double r, std::vector<bool> &exclusions, 
                                           std::vector<GridPointReference> &refs);

    void _getConnectedPoints(GridPointReference seed, double radius, 
                             std::vector<vmath::vec3> &points);
    void _getConnectedPointReferences(GridPointReference seed, double radius, 
                                      std::vector<GridPointReference> &refs);

    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;
    double _dx = 0.0;

    std::vector<GridPoint> _gridPoints;
    std::vector<int> _refIDToGridPointIndexTable;
    Array3d<CellNode> _grid;
    AABB _bbox;
};

#endif