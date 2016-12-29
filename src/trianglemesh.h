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
#ifndef TRIANGLEMESH_H
#define TRIANGLEMESH_H

#include <stdlib.h>
#include <queue>
#include <vector>
#include <sstream>
#include <fstream>
#include <string.h>
#include <algorithm>

#include "triangle.h"
#include "array3d.h"
#include "grid3d.h"
#include "aabb.h"
#include "collision.h"
#include "vmath.h"
#include "gridindexvector.h"
#include "spatialpointgrid.h"
#include "fluidsimassert.h"

enum class TriangleMeshFormat : char { 
    ply   = 0x00, 
    bobj  = 0x01
};

class TriangleMesh
{
public:
    TriangleMesh();
    ~TriangleMesh();

    bool loadPLY(std::string PLYFilename);
    bool loadBOBJ(std::string BOBJFilename);
    void writeMeshToPLY(std::string filename);
    void writeMeshToBOBJ(std::string filename);
    static std::string getFileExtension(TriangleMeshFormat fmt);

    int numVertices();
    int numFaces();
    int numTriangles() { return numFaces(); }
    void clear();
    void removeDuplicateTriangles();
    void updateVertexNormals();
    void updateVertexTriangles();
    void clearVertexTriangles();
    void updateTriangleAreas();
    void clearTriangleAreas();
    void smooth(double value, int iterations);
    void smooth(double value, int iterations, std::vector<int> &vertices);
    void getFaceNeighbours(unsigned int tidx, std::vector<int> &n);
    void getFaceNeighbours(Triangle t, std::vector<int> &n);
    double getTriangleArea(int tidx);
    void getVertexNeighbours(unsigned int vidx, std::vector<int> &n);
    bool isNeighbours(Triangle t1, Triangle t2);
    void getTrianglePosition(unsigned int index, vmath::vec3 tri[3]);
    vmath::vec3 getTriangleNormal(unsigned int index);
    vmath::vec3 getTriangleNormalSmooth(unsigned int index, vmath::vec3 p);
    vmath::vec3 getTriangleFaceDirection(unsigned int index);
    vmath::vec3 getTriangleCenter(unsigned int index);
    vmath::vec3 getBarycentricCoordinates(unsigned int index, vmath::vec3 p);
    void removeMinimumTriangleCountPolyhedra(int count);
    void removeMinimumVolumePolyhedra(double volume);
    void removeHoles();
    void removeTriangles(std::vector<int> &triangles);
    void removeExtraneousVertices();
    void translate(vmath::vec3 trans);
    void append(TriangleMesh &mesh);
    void join(TriangleMesh &mesh);
    void join(TriangleMesh &mesh, double tolerance);
    void removeDuplicateVertices(int i, int j, int k, double dx);

    void setGridDimensions(int i, int j, int k, double dx) {
        _gridi = i; _gridj = j; _gridk = k; _dx = dx;
    }

    std::vector<vmath::vec3> vertices;
    std::vector<vmath::vec3> vertexcolors;  // r, g, b values in range [0.0, 1.0]
    std::vector<vmath::vec3> normals;
    std::vector<Triangle> triangles;

private:
    bool _getPLYHeader(std::ifstream *file, std::string *header);
    bool _getElementNumberInPlyHeader(std::string &header, 
                                      std::string &element, int *n);
    bool _getNumVerticesInPLYHeader(std::string &header, int *n);
    bool _getNumFacesInPLYHeader(std::string &header, int *n);
    bool _isVertexColorsEnabledInPLYHeader(std::string &header);
    bool _loadPLYVertexData(std::ifstream *file, std::string &header);
    bool _loadPLYTriangleData(std::ifstream *file, std::string &header);

    void _updateVertexTriangles();
    bool _trianglesEqual(Triangle &t1, Triangle &t2);
    bool _isOnTriangleEdge(double u, double v);
    bool _isTriangleInVector(int index, std::vector<int> &tris);
    bool _isIntInVector(int i, std::vector<int> &ints);
    void _smoothTriangleMesh(double value, std::vector<bool> &isSmooth);
    void _getBoolVectorOfSmoothedVertices(std::vector<int> &verts, 
                                          std::vector<bool> &isSmooth);
    int _numDigitsInInteger(int num);

    void _getPolyhedra(std::vector<std::vector<int> > &polyList);
    void _getPolyhedronFromTriangle(int triangle, 
                                    std::vector<bool> &visitedTriangles,
                                    std::vector<int> &polyhedron);
    double _getSignedTriangleVolume(unsigned int tidx);
    double _getPolyhedronVolume(std::vector<int> &polyhedron);
    bool _isPolyhedronHole(std::vector<int> &poly);
    AABB _getMeshVertexIntersectionAABB(std::vector<vmath::vec3> verts1,
                                        std::vector<vmath::vec3> verts2, 
                                        double tolerance);
    void _findDuplicateVertexPairs(int i, int j, int k, double dx, 
                                   std::vector<std::pair<int, int> > &pairs);
    void _findDuplicateVertexPairs(std::vector<int> &verts1, 
                                   std::vector<int> &verts2, 
                                   AABB bbox,
                                   double tolerance, 
                                   std::vector<std::pair<int, int> > &pairs);

    template<class T>
    std::string _toString(T item) {
        std::ostringstream sstream;
        sstream << item;

        return sstream.str();
    }

    int _gridi = 0;
    int _gridj = 0;
    int _gridk = 0;
    double _dx = 0;

    std::vector<std::vector<int> > _vertexTriangles;
    std::vector<double> _triangleAreas;

    Array3d<std::vector<int>> _triGrid;
};

#endif
