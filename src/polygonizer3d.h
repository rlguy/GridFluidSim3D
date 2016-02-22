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
#include <assert.h>

#include "implicitsurfacescalarfield.h"
#include "array3d.h"
#include "grid3d.h"
#include "trianglemesh.h"
#include "vmath.h"
#include "gridindexvector.h"
#include "stopwatch.h"

class Polygonizer3d
{
public:
    Polygonizer3d();
    Polygonizer3d(ImplicitSurfaceScalarField *scalarField);

    ~Polygonizer3d();

    void setSurfaceCellMask(Array3d<bool> *mask);
    void polygonizeSurface();
    
    TriangleMesh getTriangleMesh() { return _surface; };
    void writeSurfaceToOBJ(std::string filename);

private:
    struct EdgeGrid {
        Array3d<int> U;         // store index to vertex
        Array3d<int> V;
        Array3d<int> W;
        Array3d<bool> isSetU;
        Array3d<bool> isSetV;
        Array3d<bool> isSetW;

        EdgeGrid() : U(Array3d<int>(0, 0, 0)),
                     V(Array3d<int>(0, 0, 0)),
                     W(Array3d<int>(0, 0, 0)),
                     isSetU(Array3d<bool>(0, 0, 0)),
                     isSetV(Array3d<bool>(0, 0, 0)), 
                     isSetW(Array3d<bool>(0, 0, 0)) {}

        EdgeGrid(int i, int j, int k) : 
                     U(Array3d<int>(i, j + 1, k + 1)),
                     V(Array3d<int>(i + 1, j, k + 1)),
                     W(Array3d<int>(i + 1, j + 1, k)),
                     isSetU(Array3d<bool>(i, j + 1, k + 1, false)),
                     isSetV(Array3d<bool>(i + 1, j, k + 1, false)),
                     isSetW(Array3d<bool>(i + 1, j + 1, k, false)) {}
    };

    void _getCellVertexPositions(GridIndex g, vmath::vec3 positions[8]);
    vmath::vec3 _getVertexPosition(GridIndex v);
    double _getVertexFieldValue(GridIndex v);
    bool _isCellOutsideSurface(GridIndex g);
    bool _isCellInsideSurface(GridIndex g);
    bool _isCellOnSurface(GridIndex g);
    int _getCellSurfaceStatus(GridIndex g);
    void _polygonizeCell(GridIndex g, double isolevel, EdgeGrid &edges);
    int _calculateCubeIndex(GridIndex g, double isolevel);
    void _calculateVertexList(GridIndex g, double isolevel, int cubeIndex, int vertList[12], EdgeGrid &edges);
    vmath::vec3 _vertexInterp(double isolevel, vmath::vec3 p1, vmath::vec3 p2, double valp1, double valp2);
    void _calculateSurfaceTriangles();

    GridIndexVector _findSurfaceCells();

    static const int _edgeTable[256];
    static const int _triTable[256][16];

    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;
    double _dx = 0.0;

    double _surfaceThreshold = 0.5;
    bool _isScalarFieldSet = false;

    ImplicitSurfaceScalarField *_scalarField;
    Array3d<bool> *_surfaceCellMask;
    bool _isSurfaceCellMaskSet = false;

    GridIndexVector _surfaceCells;
    TriangleMesh _surface;
};

#endif
