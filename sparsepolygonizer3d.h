#include <stdio.h>
#include <iostream>
#include <vector>
#include <queue>
#include <sstream>
#include <fstream>
#include <assert.h>

#include "implicitsurfacefield.h"
#include "sparseimplicitsurfacescalarfield.h"
#include "sparsearray3d.h"
#include "grid3d.h"
#include "trianglemesh.h"
#include "glm/glm.hpp"

#include "stopwatch.h"

#pragma once

class SparsePolygonizer3d
{
public:
    SparsePolygonizer3d();
    SparsePolygonizer3d(SparseImplicitSurfaceScalarField &scalarField);

    ~SparsePolygonizer3d();

    void setSurfaceThreshold(double val) { _field->setSurfaceThreshold(val); }
    void setSurfaceCellIndices(std::vector<GridIndex> indices);
    void setScalarField(SparseImplicitSurfaceScalarField &field);
    void polygonizeSurface();

    std::vector<GridIndex> getSurfaceCells() { return _surfaceCells; }
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
                     U(i, j + 1, k + 1, 0.0),
                     V(i + 1, j, k + 1, 0.0),
                     W(i + 1, j + 1, k, 0.0),
                     isSetU(i, j + 1, k + 1, false),
                     isSetV(i + 1, j, k + 1, false),
                     isSetW(i + 1, j + 1, k, false) {}
    };

    void _getCellVertexPositions(GridIndex g, glm::vec3 positions[8]);
    glm::vec3 _getVertexPosition(GridIndex v);
    double _getVertexFieldValue(GridIndex v);
    bool _isCellOutsideSurface(GridIndex g);
    bool _isCellInsideSurface(GridIndex g);
    bool _isCellOnSurface(GridIndex g);
    bool _isCellDataAvailable(GridIndex g);
    int _getCellSurfaceStatus(GridIndex g);
    void _polygonizeCell(GridIndex g, double isolevel, EdgeGrid &edges);
    int _calculateCubeIndex(GridIndex g, double isolevel);
    void _calculateVertexList(GridIndex g, double isolevel, int cubeIndex, int vertList[12], EdgeGrid &edges);
    glm::vec3 _vertexInterp(double isolevel, glm::vec3 p1, glm::vec3 p2, double valp1, double valp2);
    void _calculateSurfaceTriangles();

    std::vector<GridIndex> _findSurfaceCells();
    void _resetVertexValues();
    std::vector<GridIndex> _processSeedCell(GridIndex seed, SparseArray3d<bool> &isCellDone);

    static const int _edgeTable[256];
    static const int _triTable[256][16];

    bool _isInitialized = false;
    int _isize, _jsize, _ksize;
    double _dx;

    SurfaceField *_field;
    SparseArray3d<double> _vertexValues;
    SparseArray3d<bool> _isCellDone;
    double _surfaceThreshold = 0.5;

    // cell indices that are fully or partially within the iso surface
    std::vector<GridIndex> _surfaceIndices;
    std::vector<GridIndex> _surfaceCells;
    TriangleMesh _surface;

    double MISSING_DATA_VALUE = -1.0;
};

