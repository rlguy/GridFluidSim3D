#pragma once

#define _CRT_SECURE_NO_WARNINGS

#include <queue>
#include <vector>
#include <sstream>
#include <fstream>
#include <assert.h>

#include "triangle.h"
#include "surfacefield.h"
#include "array3d.h"
#include "aabb.h"
#include "collision.h"
#include "glm/glm.hpp"

class TriangleMesh
{
public:
    TriangleMesh();
    ~TriangleMesh();

    bool loadOBJ(std::string OBJFilename) {
        return loadOBJ(OBJFilename, glm::vec3(0.0, 0.0, 0.0), 1.0);
    }
    bool loadOBJ(std::string OBJFilename, glm::vec3 offset) {
        return loadOBJ(OBJFilename, offset, 1.0);
    }
    bool loadOBJ(std::string OBJFilename, double scale) {
        return loadOBJ(OBJFilename, glm::vec3(0.0, 0.0, 0.0), scale);
    }
    bool loadOBJ(std::string OBJFilename, glm::vec3 offset, double scale);

    int numVertices();
    int numFaces();
    int numTriangles() { return numFaces(); }
    void clear();
    void writeMeshToOBJ(std::string filename);
    void removeDuplicateTriangles();
    void updateVertexNormals();
    void getFaceNeighbours(Triangle t, std::vector<int> &n);
    bool isNeighbours(Triangle t1, Triangle t2);
    void getCellsInsideMesh(std::vector<GridIndex> &cells);
    void getTrianglePosition(unsigned int index, glm::vec3 tri[3]);
    glm::vec3 getTriangleNormal(unsigned int index);
    glm::vec3 getTriangleFaceDirection(unsigned int index);
    glm::vec3 getTriangleCenter(unsigned int index);

    void setGridDimensions(int i, int j, int k, double dx) {
        _gridi = i; _gridj = j; _gridk = k; _dx = dx;
    }

    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;
    std::vector<Triangle> triangles;

private:
    void _updateVertexTriangles();
    bool _trianglesEqual(Triangle &t1, Triangle &t2);
    bool _isOnTriangleEdge(double u, double v);
    bool _isTriangleInVector(int index, std::vector<int> &tris);
    bool _isIntInVector(int i, std::vector<int> &ints);
    int _getIntersectingTrianglesInCell(GridIndex g, glm::vec3 p, glm::vec3 dir, 
                                        std::vector<int> &tris, bool *success);
    bool _isCellInsideMesh(const GridIndex g);
    void _floodfill(GridIndex g, Array3d<bool> &cells);
    void _updateTriangleGrid();
    void _destroyTriangleGrid();
    void _getTriangleGridCellOverlap(Triangle t, std::vector<GridIndex> &cells);
    void _getSurfaceCells(std::vector<GridIndex> &cells);

    void _getNeighbourGridIndices6(GridIndex g, GridIndex n[6]);
    GridIndex _positionToGridIndex(glm::vec3 p);
    glm::vec3 _gridIndexToPosition(GridIndex g) {
        assert(_isCellIndexInRange(g));
        return glm::vec3((double)g.i*_dx, (double)g.j*_dx, (double)g.k*_dx);
    }
    inline bool _isCellIndexInRange(GridIndex g) {
        return g.i >= 0 && g.j >= 0 && g.k >= 0 && 
               g.i < _gridi && g.j < _gridj && g.k < _gridk;
    }
    inline double _randomFloat(double min, double max) {
        return min + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max - min)));
    }

    double _gridi = 0;
    double _gridj = 0;
    double _gridk = 0;
    double _dx = 0;

    std::vector <std::vector<int> > _vertexTriangles;

    Array3d<std::vector<int>> _triGrid;
};

