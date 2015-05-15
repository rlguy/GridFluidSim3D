#include "trianglemesh.h"


TriangleMesh::TriangleMesh()
{
}


TriangleMesh::~TriangleMesh()
{
}

int TriangleMesh::numVertices() {
    return (int)vertices.size();
}

int TriangleMesh::numFaces() {
    return (int)triangles.size();
}

void TriangleMesh::clear() {
    vertices.clear();
    normals.clear();
    triangles.clear();
    _vertexTriangles.clear();
}

void TriangleMesh::writeMeshToOBJ(std::string filename) {
    assert(normals.size() == vertices.size());

    std::ostringstream str;

    str << "# OBJ file format with ext .obj" << std::endl;
    str << "# vertex count = " << vertices.size() << std::endl;
    str << "# face count = " << triangles.size() << std::endl;

    glm::vec3 p;
    for (int i = 0; i < vertices.size(); i++) {
        p = vertices[i];
        str << "v " << p.x << " " << p.y << " " << p.z << std::endl;
    }

    glm::vec3 n;
    for (int i = 0; i < normals.size(); i++) {
        n = normals[i];
        str << "vn " << n.x << " " << n.y << " " << n.z << std::endl;
    }

    Triangle t;
    int v1, v2, v3;
    for (int i = 0; i < triangles.size(); i++) {
        t = triangles[i];
        v1 = t.tri[0] + 1;
        v2 = t.tri[1] + 1;
        v3 = t.tri[2] + 1;

        str << "f " << v1 << "//" << v1 << " " <<
            v2 << "//" << v2 << " " <<
            v3 << "//" << v3 << std::endl;
    }

    std::ofstream out(filename);
    out << str.str();
    out.close();
}

void TriangleMesh::updateVertexNormals() {
    normals.clear();
    _updateVertexTriangles();
    
    std::vector<glm::vec3> facenormals;
    facenormals.reserve(triangles.size());
    Triangle t;
    glm::vec3 v1, v2;
    for (int i = 0; i < triangles.size(); i++) {
        t = triangles[i];

        v1 = vertices[t.tri[1]] - vertices[t.tri[0]];
        v2 = vertices[t.tri[2]] - vertices[t.tri[0]];
        facenormals.push_back(glm::normalize(glm::cross(v1, v2)));
    }

    glm::vec3 n;
    for (int i = 0; i < _vertexTriangles.size(); i++) {
        n = glm::vec3(0.0, 0.0, 0.0);
        for (int j = 0; j < _vertexTriangles[i].size(); j++) {
            n += facenormals[_vertexTriangles[i][j]];
        }

        n = glm::normalize(n / (float)_vertexTriangles[i].size());
        normals.push_back(n);
    }
}

void TriangleMesh::getFaceNeighbours(Triangle t, std::vector<int> &n) {
    std::vector<int> vn;
    for (int i = 1; i < 3; i++) {
        vn = _vertexTriangles[t.tri[i]];
        n.insert(n.end(), vn.begin(), vn.end());
    }
}

bool TriangleMesh::_trianglesEqual(Triangle &t1, Triangle &t2) {
    return t1.tri[0] == t2.tri[0] ||
           t1.tri[1] == t2.tri[1] ||
           t1.tri[2] == t2.tri[2];
}

bool TriangleMesh::isNeighbours(Triangle t1, Triangle t2) {
    std::vector<int> n;
    getFaceNeighbours(t1, n);
    for (int i = 0; i < n.size(); i++) {
        if (_trianglesEqual(triangles[n[i]], t2)) {
            return true;
        }
    }

    return false;
}

void TriangleMesh::_updateVertexTriangles() {
    _vertexTriangles.reserve(vertices.size());

    for (int i = 0; i < vertices.size(); i++) {
        std::vector<int> triangles;
        triangles.reserve(14);  // 14 is the maximum number of adjacent triangles
                                // to a vertex
        _vertexTriangles.push_back(triangles);
    }

    std::vector<glm::vec3> facenormals;
    facenormals.reserve(triangles.size());
    Triangle t;
    for (int i = 0; i < triangles.size(); i++) {
        t = triangles[i];
        _vertexTriangles[t.tri[0]].push_back(i);
        _vertexTriangles[t.tri[1]].push_back(i);
        _vertexTriangles[t.tri[2]].push_back(i);
    }
}

GridIndex TriangleMesh::_positionToGridIndex(glm::vec3 p) {
    double invdx = 1.0 / _dx;
    return GridIndex((int)floor(p.x*invdx),
        (int)floor(p.y*invdx),
        (int)floor(p.z*invdx));
}

bool TriangleMesh::_isPointInsideMesh(glm::vec3 p) {
    // count how many intersections between point and edge of grid
    // even intersections: outside
    // odd intersections: inside

    glm::vec3 rightdir = glm::vec3(1.0, 0.0, 0.0);
    glm::vec3 leftdir = glm::vec3(-1.0, 0.0, 0.0);

    return false;
}

void TriangleMesh::_getTriangleGridCellOverlap(Triangle t, std::vector<GridIndex> &cells) {
    std::vector<GridIndex> testcells;
    AABB tbbox = AABB(t, vertices);
    tbbox.getOverlappingGridCells(_dx, testcells);

    AABB cbbox = AABB(glm::vec3(0.0, 0.0, 0.0), _dx, _dx, _dx);
    for (int i = 0; i < testcells.size(); i++) {
        cbbox.position = _gridIndexToPosition(testcells[i]);
        if (cbbox.isOverlappingTriangle(t, vertices)) {
            cells.push_back(testcells[i]);
        }
    }
}

void TriangleMesh::_updateTriangleGrid() {
    triGrid = Array3d<std::vector<int>>(_gridi, _gridj, _gridk);

    std::vector<GridIndex> cells;
    std::vector<int> *triVector;
    Triangle t;
    GridIndex g;
    for (int i = 0; i < triangles.size(); i++) {
        t = triangles[i];
        cells.clear();
        _getTriangleGridCellOverlap(t, cells);

        for (int j = 0; j < cells.size(); j++) {
            g = cells[j];
            triVector = triGrid.getPointer(g);
            triVector->push_back(i);
        }
    }
}

void TriangleMesh::_getSurfaceCells(std::vector<GridIndex> &cells) {
    std::vector<int> *tris;
    for (int k = 0; k < triGrid.depth; k++) {
        for (int j = 0; j < triGrid.height; j++) {
            for (int i = 0; i < triGrid.width; i++) {
                tris = triGrid.getPointer(i, j, k);
                if (tris->size() > 0) {
                    cells.push_back(GridIndex(i, j, k));
                }
            }
        }
    }
}

void TriangleMesh::getCellsInsideMesh(std::vector<GridIndex> &cells) {
    if (_gridi == 0 || _gridj == 0 || _gridk == 0) {
        return;
    }

    _updateTriangleGrid();

    std::vector<GridIndex> surfaceCells;
    _getSurfaceCells(surfaceCells);

    triGrid = Array3d<std::vector<int> >();
}