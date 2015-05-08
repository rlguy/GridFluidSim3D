#include <stdio.h>
#include <iostream>
#include <vector>
#include <queue>
#include <assert.h>

#include "implicitfield.h"
#include "array3d.h"
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

private:
    void _getCellVertexIndices(GridIndex g, GridIndex vertices[8]);
    glm::vec3 _getVertexPosition(GridIndex v);
    double _getVertexFieldValue(GridIndex v);
    bool _isCellOutsideSurface(GridIndex g);
    bool _isCellInsideSurface(GridIndex g);
    bool _isCellOnSurface(GridIndex g);
    int _getCellSurfaceStatus(GridIndex g);

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

    bool _isInitialized = false;
    int _isize, _jsize, _ksize;
    double _dx;

    ImplicitField *_field;
    Array3d<double> _vertexValues;
    Array3d<bool> _isVertexSet;

    // cell indices that are fully or partially within the iso surface
    std::vector<GridIndex> _insideIndices;
    std::vector<GridIndex> _surfaceCells;
};

