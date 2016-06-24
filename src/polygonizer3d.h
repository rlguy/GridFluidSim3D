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
#ifndef POLYGONIZER3D_H
#define POLYGONIZER3D_H

#include <stdio.h>
#include <iostream>
#include <vector>
#include <queue>
#include <sstream>
#include <fstream>

#include "scalarfield.h"
#include "array3d.h"
#include "grid3d.h"
#include "trianglemesh.h"
#include "vmath.h"
#include "gridindexvector.h"
#include "stopwatch.h"
#include "fluidsimassert.h"

class Polygonizer3d
{
public:
    Polygonizer3d();
    Polygonizer3d(ScalarField *scalarField);

    ~Polygonizer3d();

    void setSurfaceCellMask(Array3d<bool> *mask);
    TriangleMesh polygonizeSurface();

private:
    struct EdgeGrid {
        Array3d<int> U;         // store index to vertex
        Array3d<int> V;
        Array3d<int> W;

        EdgeGrid() : U(Array3d<int>(0, 0, 0)),
                     V(Array3d<int>(0, 0, 0)),
                     W(Array3d<int>(0, 0, 0)) {}

        EdgeGrid(int i, int j, int k) : 
                     U(Array3d<int>(i, j + 1, k + 1, -1)),
                     V(Array3d<int>(i + 1, j, k + 1, -1)),
                     W(Array3d<int>(i + 1, j + 1, k, -1)) {}
    };

    vmath::vec3 _getVertexPosition(GridIndex v);
    double _getVertexFieldValue(GridIndex v);
    bool _isCellOnSurface(GridIndex g);
    void _polygonizeCell(GridIndex g, EdgeGrid &edges, TriangleMesh &mesh);
    int _calculateCubeIndex(GridIndex g);
    void _calculateVertexList(GridIndex g,
                              int cubeIndex, 
                              EdgeGrid &edges, 
                              std::vector<vmath::vec3> &meshVertices, 
                              int vertList[12]);
    vmath::vec3 _vertexInterp(vmath::vec3 p1, vmath::vec3 p2, double valp1, double valp2);
    void _calculateSurfaceTriangles(GridIndexVector &surfaceCells, TriangleMesh &mesh);
    void _findSurfaceCells(GridIndexVector &surfaceCells);

    static const int _edgeTable[256];
    static const int _triTable[256][16];

    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;
    double _dx = 0.0;

    double _surfaceThreshold = 0.5;

    ScalarField *_scalarField;
    bool _isScalarFieldSet = false;

    Array3d<bool> *_surfaceCellMask;
    bool _isSurfaceCellMaskSet = false;

};

#endif
