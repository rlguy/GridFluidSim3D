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
#ifndef SPARSEPOLYGONIZER3D_H
#define SPARSEPOLYGONIZER3D_H

#include <stdio.h>
#include <iostream>
#include <vector>
#include <queue>
#include <sstream>
#include <fstream>
#include <assert.h>

#include "levelsetfield.h"
#include "sparseimplicitsurfacescalarfield.h"
#include "sparsearray3d.h"
#include "grid3d.h"
#include "trianglemesh.h"
#include "vmath.h"
#include "gridindexvector.h"

#pragma once

class SparsePolygonizer3d
{
public:
    SparsePolygonizer3d();
    SparsePolygonizer3d(SparseImplicitSurfaceScalarField &scalarField);

    ~SparsePolygonizer3d();

    void setSurfaceThreshold(double val) { _field->setSurfaceThreshold(val); }
    void setSurfaceCellIndices(GridIndexVector &indices);
    void setScalarField(SparseImplicitSurfaceScalarField &field);
    void polygonizeSurface();

    TriangleMesh getTriangleMesh() { return _surface; };
    void writeSurfaceToOBJ(std::string filename);

private:
    struct EdgeGrid {
        SparseArray3d<int> U;         // store index to vertex
        SparseArray3d<int> V;
        SparseArray3d<int> W;
        SparseArray3d<bool> isSetU;
        SparseArray3d<bool> isSetV;
        SparseArray3d<bool> isSetW;

        EdgeGrid() {}

        EdgeGrid(int i, int j, int k) : 
                     U(i, j + 1, k + 1, 0),
                     V(i + 1, j, k + 1, 0),
                     W(i + 1, j + 1, k, 0),
                     isSetU(i, j + 1, k + 1, false),
                     isSetV(i + 1, j, k + 1, false),
                     isSetW(i + 1, j + 1, k, false) {}
    };

    void _getCellVertexPositions(GridIndex g, vmath::vec3 positions[8]);
    void _getCellVertexValues(GridIndex g, double values[8]);
    vmath::vec3 _getVertexPosition(GridIndex v);
    bool _isCellOutsideSurface(GridIndex g);
    bool _isCellInsideSurface(GridIndex g);
    bool _isCellOnSurface(GridIndex g);
    bool _isCellDataAvailable(GridIndex g);
    int _getCellSurfaceStatus(GridIndex g);
    void _polygonizeCell(GridIndex g, double isolevel, EdgeGrid &edges);
    int _calculateCubeIndex(GridIndex g, double isolevel);
    void _calculateVertexList(GridIndex g, double isolevel, int cubeIndex, int vertList[12], EdgeGrid &edges);
    vmath::vec3 _vertexInterp(double isolevel, vmath::vec3 p1, vmath::vec3 p2, double valp1, double valp2);
    void _calculateSurfaceTriangles();

    GridIndexVector _findSurfaceCells();
    void _resetVertexValues();
    GridIndexVector _processSeedCell(GridIndex seed, SparseArray3d<bool> &isCellDone);

    static const int _edgeTable[256];
    static const int _triTable[256][16];

    bool _isInitialized = false;
    int _isize, _jsize, _ksize;
    double _dx;

    SurfaceField *_field;
    SparseArray3d<float> _vertexValues;
    SparseArray3d<bool> _isCellDone;
    double _surfaceThreshold = 0.5;

    // cell indices that are fully or partially within the iso surface
    GridIndexVector _surfaceIndices;
    GridIndexVector _surfaceCells;
    TriangleMesh _surface;

    float MISSING_DATA_VALUE = -1.0f;
};

#endif
