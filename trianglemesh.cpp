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
    vertexTriangles.clear();
}

void TriangleMesh::writeMeshToOBJ(std::string filename) {
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

    vertexTriangles.clear();
    vertexTriangles.reserve(vertices.size());

    for (int i = 0; i < vertices.size(); i++) {
        std::vector<int> triangles;
        triangles.reserve(14);  // 14 is the maximum number of adjacent triangles
                                // to a vertex
        vertexTriangles.push_back(triangles);
    }
    
    std::vector<glm::vec3> facenormals;
    facenormals.reserve(triangles.size());
    Triangle t;
    glm::vec3 v1, v2;
    for (int i = 0; i < triangles.size(); i++) {
        t = triangles[i];
        vertexTriangles[t.tri[0]].push_back(i);
        vertexTriangles[t.tri[1]].push_back(i);
        vertexTriangles[t.tri[2]].push_back(i);

        v1 = vertices[t.tri[1]] - vertices[t.tri[0]];
        v2 = vertices[t.tri[2]] - vertices[t.tri[0]];
        facenormals.push_back(glm::normalize(glm::cross(v1, v2)));
    }

    glm::vec3 n;
    for (int i = 0; i < vertexTriangles.size(); i++) {
        n = glm::vec3(0.0, 0.0, 0.0);
        for (int j = 0; j < vertexTriangles[i].size(); j++) {
            n += facenormals[vertexTriangles[i][j]];
        }

        n = glm::normalize(n / (float)vertexTriangles[i].size());
        normals.push_back(n);
    }
}