#include <stdio.h>
#include <iostream>
#include <vector>
#include <queue>
#include <sstream>
#include <fstream>
#include <assert.h>

#include "implicitfield.h"
#include "array3d.h"
#include "trianglemesh.h"
#include "glm/glm.hpp"

#pragma once

class Polygonizer3d
{
public:
    Polygonizer3d();
    Polygonizer3d(int i_width, int j_height, int k_depth, double cellsize, 
                  ImplicitField *field);

    ~Polygonizer3d();

    void setInsideCellIndices(std::vector<GridIndex> indices);
    void polygonizeSurface();

    std::vector<GridIndex> getSurfaceCells() { return _surfaceCells; }
    TriangleMesh* getTriangleMesh() { return &_surface; };
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

    void _getVertexCellNeighbours(GridIndex v, GridIndex cells[8]);
    void _getCellVertexIndices(GridIndex g, GridIndex vertices[8]);
    void _getCellVertexPositions(GridIndex g, glm::vec3 positions[8]);
    glm::vec3 _getVertexPosition(GridIndex v);
    double _getVertexFieldValue(GridIndex v);
    bool _isCellOutsideSurface(GridIndex g);
    bool _isCellInsideSurface(GridIndex g);
    bool _isCellOnSurface(GridIndex g);
    int _getCellSurfaceStatus(GridIndex g);
    void _polygonizeCell(GridIndex g, double isolevel, EdgeGrid &edges);
    int _calculateCubeIndex(GridIndex g, double isolevel);
    void _calculateVertexList(GridIndex g, double isolevel, int cubeIndex, int vertList[12], EdgeGrid &edges);
    glm::vec3 _vertexInterp(double isolevel, glm::vec3 p1, glm::vec3 p2, double valp1, double valp2);
    void _calculateSurfaceTriangles();

    std::vector<GridIndex> _findSurfaceCells();
    void _resetVertexValues();
    std::vector<GridIndex> _processSeedCell(GridIndex seed, Array3d<bool> &isCellDone);
    void _getNeighbourGridIndices6(GridIndex cell, GridIndex n[6]);

    inline bool _isCellIndexInRange(int i, int j, int k) {
        return i >= 0 && j >= 0 && k >= 0 && i < _isize && j < _jsize && k < _ksize;
    }
    inline bool _isCellIndexInRange(GridIndex g) {
        return g.i >= 0 && g.j >= 0 && g.k >= 0 && g.i < _isize && g.j < _jsize && g.k < _ksize;
    }

    static const int edgeTable[256];
    static const int triTable[256][16];

    bool _isInitialized = false;
    int _isize, _jsize, _ksize;
    double _dx;

    ImplicitField *_field;
    Array3d<double> _vertexValues;
    Array3d<bool> _isVertexSet;

    // cell indices that are fully or partially within the iso surface
    std::vector<GridIndex> _insideIndices;
    std::vector<GridIndex> _surfaceCells;
    TriangleMesh _surface;
};

