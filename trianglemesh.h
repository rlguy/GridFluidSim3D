#include <queue>
#include <sstream>
#include <fstream>
#include <assert.h>

#include "triangle.h"
#include "implicitfield.h"
#include "array3d.h"
#include "aabb.h"
#include "glm/glm.hpp"

#pragma once

class TriangleMesh
{
public:
    TriangleMesh();
    ~TriangleMesh();

    int numVertices();
    int numFaces();
    int numTriangles() { return numFaces(); }
    void clear();
    void writeMeshToOBJ(std::string filename);
    void updateVertexNormals();
    void getFaceNeighbours(Triangle t, std::vector<int> &n);
    bool isNeighbours(Triangle t1, Triangle t2);
    void getCellsInsideMesh(std::vector<GridIndex> &cells);
    void getSurfaceCells();

    void setGridDimensions(int i, int j, int k, double dx) {
        _gridi = i; _gridj = j; _gridk = k; _dx = dx;
    }

    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;
    std::vector<Triangle> triangles;

private:
    void _updateVertexTriangles();
    bool _trianglesEqual(Triangle &t1, Triangle &t2);
    bool _isPointInsideMesh(glm::vec3 p);
    void _updateTriangleGrid();
    void _getTriangleGridCellOverlap(Triangle t, std::vector<GridIndex> &cells);
    void _getSurfaceCells(std::vector<GridIndex> &cells);

    GridIndex _positionToGridIndex(glm::vec3 p);
    glm::vec3 _gridIndexToPosition(GridIndex g) {
        assert(_isCellIndexInRange(g));
        return glm::vec3((double)g.i*_dx, (double)g.j*_dx, (double)g.k*_dx);
    }
    inline bool _isCellIndexInRange(GridIndex g) {
        return g.i >= 0 && g.j >= 0 && g.k >= 0 && 
               g.i < _gridi && g.j < _gridj && g.k < _gridk;
    }

    double _gridi = 0;
    double _gridj = 0;
    double _gridk = 0;
    double _dx = 0;

    std::vector <std::vector<int> > _vertexTriangles;

    Array3d<std::vector<int>> triGrid;
};

