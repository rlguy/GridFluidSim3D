#pragma once

#include <queue>
#include <vector>
#include <sstream>
#include <fstream>
#include <assert.h>

#include "triangle.h"
#include "surfacefield.h"
#include "array3d.h"
#include "grid3d.h"
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
    void writeMeshToSTL(std::string filename);
    void writeMeshToPLY(std::string filename);
    void removeDuplicateTriangles();
    void updateVertexNormals();
    void updateVertexTriangles();
    void clearVertexTriangles();
    void updateTriangleAreas();
    void clearTriangleAreas();
    void smooth(double value, int iterations);
    void getFaceNeighbours(unsigned int tidx, std::vector<int> &n);
    void getFaceNeighbours(Triangle t, std::vector<int> &n);
    double getTriangleArea(int tidx);
    void getVertexNeighbours(unsigned int vidx, std::vector<int> &n);
    bool isNeighbours(Triangle t1, Triangle t2);
    void getCellsInsideMesh(std::vector<GridIndex> &cells);
    void getTrianglePosition(unsigned int index, glm::vec3 tri[3]);
    glm::vec3 getTriangleNormal(unsigned int index);
    glm::vec3 getTriangleNormalSmooth(unsigned int index, glm::vec3 p);
    glm::vec3 getTriangleFaceDirection(unsigned int index);
    glm::vec3 getTriangleCenter(unsigned int index);
    glm::vec3 getBarycentricCoordinates(unsigned int index, glm::vec3 p);

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
    void _smoothTriangleMesh(double value);
    int _numDigitsInInteger(int num);

    inline double _randomFloat(double min, double max) {
        return min + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max - min)));
    }

    int _gridi = 0;
    int _gridj = 0;
    int _gridk = 0;
    double _dx = 0;

    std::vector<std::vector<int> > _vertexTriangles;
    std::vector<double> _triangleAreas;

    Array3d<std::vector<int>> _triGrid;
};

