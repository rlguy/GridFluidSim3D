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
#ifndef GRID3D_H
#define GRID3D_H

#include <stdio.h>
#include <iostream>

#include "glm/glm.hpp"
#include "array3d.h"
#include "aabb.h"

namespace Grid3d {
    
    extern void positionToGridIndex(double x, double y, double z, double dx,
                                    int *i, int *j, int *k);
    extern void positionToGridIndex(glm::vec3 p, double dx,
                                    int *i, int *j, int *k);
    extern GridIndex positionToGridIndex(double x, double y, double z, double dx);
    extern GridIndex positionToGridIndex(glm::vec3 p, double dx);

    extern void GridIndexToPosition(int i, int j, int k, double dx, 
                                    double *x, double *y, double *z);
    extern void GridIndexToPosition(GridIndex g, double dx, 
                                    double *x, double *y, double *z);
    extern glm::vec3 GridIndexToPosition(int i, int j, int k, double dx);
    extern glm::vec3 GridIndexToPosition(GridIndex g, double dx);

    extern void GridIndexToCellCenter(int i, int j, int k, double dx, 
                                      double *x, double *y, double *z);
    extern void GridIndexToCellCenter(GridIndex g, double dx, 
                                      double *x, double *y, double *z);
    extern glm::vec3 GridIndexToCellCenter(int i, int j, int k, double dx);
    extern glm::vec3 GridIndexToCellCenter(GridIndex g, double dx);

    extern bool isPositionInGrid(double x, double y, double z, double dx,
                                 int i, int j, int k);
    extern bool isPositionInGrid(glm::vec3 p, double dx,
                                 int i, int j, int k);
    extern bool isPositionInGrid(double x, double y, double z, double dx, GridIndex g);
    extern bool isPositionInGrid(glm::vec3 p, double dx, GridIndex g);

    extern bool isGridIndexInRange(int i, int j, int k, int imax, int jmax, int kmax);
    extern bool isGridIndexInRange(GridIndex g, int imax, int jmax, int kmax);
    extern bool isGridIndexInRange(int i, int j, int k, GridIndex gmax);
    extern bool isGridIndexInRange(GridIndex g, GridIndex gmax);

    extern bool isGridIndicesNeighbours(int i1, int j1, int k1, int i2, int j2, int k2);
    extern bool isGridIndicesNeighbours(GridIndex g1, int i2, int j2, int k2);
    extern bool isGridIndicesNeighbours(int i1, int j1, int k1, GridIndex g2);
    extern bool isGridIndicesNeighbours(GridIndex g1, GridIndex g2);

    extern bool isGridIndexOnBorder(int i, int j, int k, int imax, int jmax, int kmax);
    extern bool isGridIndexOnBorder(GridIndex g, int imax, int jmax, int kmax);
    extern bool isGridIndexOnBorder(int i, int j, int k, GridIndex gmax);
    extern bool isGridIndexOnBorder(GridIndex g, GridIndex gmax);

    extern void getNeighbourGridIndices6(int i, int j, int k, GridIndex n[6]);
    extern void getNeighbourGridIndices6(GridIndex g, GridIndex n[6]);
    extern void getNeighbourGridIndices26(int i, int j, int k, GridIndex n[26]);
    extern void getNeighbourGridIndices26(GridIndex g, GridIndex n[26]);
    extern void getNeighbourGridIndices124(int i, int j, int k, GridIndex n[124]);
    extern void getNeighbourGridIndices124(GridIndex g, GridIndex n[124]);

    extern void getSubdividedGridIndices(int i, int j, int k, int subdivisions, GridIndex *n);
    extern void getSubdividedGridIndices(GridIndex g, int subdivisions, GridIndex *n);

    extern void getGridIndexVertices(int i, int j, int k, GridIndex v[8]);
    extern void getGridIndexVertices(GridIndex g, GridIndex v[8]);

    extern void getVertexGridIndexNeighbours(int i, int j, int k, GridIndex n[8]);
    extern void getVertexGridIndexNeighbours(GridIndex v, GridIndex n[8]);

    extern void getGridIndexBounds(glm::vec3 p, double r, double dx, 
                                   int imax, int jmax, int kmax, 
                                   GridIndex *g1, GridIndex *g2);
    extern void getGridIndexBounds(glm::vec3 p, double r, double dx, GridIndex gmax, 
                                   GridIndex *g1, GridIndex *g2);
    extern void getGridIndexBounds(glm::vec3 p, double r, glm::mat3 G, double dx, 
                                   int imax, int jmax, int kmax, 
                                   GridIndex *g1, GridIndex *g2);
    extern void getGridIndexBounds(glm::vec3 p, double r, glm::mat3 G, double dx, GridIndex gmax, 
                                   GridIndex *g1, GridIndex *g2);
    extern void getGridIndexBounds(AABB bbox, double dx, 
                                   int imax, int jmax, int kmax,
                                   GridIndex *g1, GridIndex *g2);
    extern void getGridIndexBounds(AABB bbox, double dx, GridIndex gmax,
                                   GridIndex *g1, GridIndex *g2);

    extern AABB fitAABBtoGrid(AABB bbox, double dx, int imax, int jmax, int kmax);
    extern AABB fitAABBtoGrid(AABB bbox, double dx, GridIndex gmax);
}

#endif
