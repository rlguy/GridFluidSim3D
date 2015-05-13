#include <queue>
#include <sstream>
#include <fstream>
#include <assert.h>

#include "implicitfield.h"
#include "array3d.h"
#include "glm/glm.hpp"

#pragma once

struct Triangle {
    int tri[3];     // indices to a vertex

    Triangle() {
        tri[0] = 0;
        tri[1] = 0;
        tri[2] = 0;
    }

    Triangle(int p1, int p2, int p3) {
        tri[0] = p1;
        tri[1] = p2;
        tri[2] = p3;
    }
};

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

    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;
    std::vector<Triangle> triangles;

private:
    std::vector <std::vector<int> > vertexTriangles;
};

