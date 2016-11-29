#include <stdio.h>
#include <iostream>

#include "../trianglemesh.h"
#include "../triangle.h"
#include "../vmath.h"
#include "../aabb.h"
#include "../grid3d.h"
#include "../array3d.h"
#include "../collision.h"

#include "cbindings.h"
#include "gridindex_c.h"

#ifdef _WIN32
    #define EXPORTDLL __declspec(dllexport)
#else
    #define EXPORTDLL
#endif

typedef struct TriangleMesh_t {
    float *vertices;
    int *triangles;
    int numVertices;
    int numTriangles;
} TriangleMesh_t;

namespace utils {

void structToTriangleMesh(TriangleMesh_t &mesh_data, TriangleMesh &mesh) {
    mesh.vertices.reserve(mesh_data.numVertices);
    mesh.triangles.reserve(mesh_data.numTriangles);

    vmath::vec3 v;
    for (int i = 0; i < mesh_data.numVertices; i++) {
        v.x = mesh_data.vertices[3*i + 0];
        v.y = mesh_data.vertices[3*i + 1];
        v.z = mesh_data.vertices[3*i + 2];
        mesh.vertices.push_back(v);
    }

    Triangle t;
    for (int i = 0; i < mesh_data.numTriangles; i++) {
        t.tri[0] = mesh_data.triangles[3*i + 0];
        t.tri[1] = mesh_data.triangles[3*i + 1];
        t.tri[2] = mesh_data.triangles[3*i + 2];
        mesh.triangles.push_back(t);
    }
}

void _getTriangleGridZ(
        TriangleMesh &m, double dx, Array3d<std::vector<int> > &ztrigrid) {
    AABB tbbox;
    GridIndex gmin, gmax;
    std::vector<int> *tris;
    for (unsigned int tidx = 0; tidx < m.triangles.size(); tidx++) {
        tbbox = AABB(m.triangles[tidx], m.vertices);
        Grid3d::getGridIndexBounds(
            tbbox, dx, ztrigrid.width, ztrigrid.height, 1, &gmin, &gmax
        );

        for (int j = gmin.j; j <= gmax.j; j++) {
            for (int i = gmin.i; i <= gmax.i; i++) {
                tris = ztrigrid.getPointer(i, j, 0);
                tris->push_back(tidx);
            }
        }
    }
}

void _getTriangleCollisionsZ(
        vmath::vec3 origin, std::vector<int> &indices, TriangleMesh &m,
        std::vector<double> &collisions) {

    vmath::vec3 dir(0.0, 0.0, 1.0);
    vmath::vec3 v1, v2, v3, coll;
    Triangle t;
    for (unsigned int i = 0; i < indices.size(); i++) {
        t = m.triangles[indices[i]];
        v1 = m.vertices[t.tri[0]];
        v2 = m.vertices[t.tri[1]];
        v3 = m.vertices[t.tri[2]];
        if (Collision::lineIntersectsTriangle(origin, dir, v1, v2, v3, &coll)) {
            collisions.push_back(coll.z);
        }
    }
}

double _randomDouble(double min, double max) {
    return min + (double)rand() / ((double)RAND_MAX / (max - min));
}

void _getCollisionGridZ(
        TriangleMesh &m, double dx, Array3d<std::vector<double> > &zcollisions) {

    Array3d<std::vector<int> > ztrigrid(zcollisions.width, zcollisions.height, 1);
    _getTriangleGridZ(m, dx, ztrigrid);

    /* Triangles that align perfectly with grid cell centers may produce 
       imperfect collision results due to an edge case where a line-mesh 
       intersection can report two collisions when striking an edge that is 
       shared by two triangles. To reduce the chance of this occurring, a random
       jitter will be added to the position of the grid cell centers. 
    */
    double jit = 0.05*dx;
    vmath::vec3 jitter(_randomDouble(jit, -jit), 
                       _randomDouble(jit, -jit), 
                       _randomDouble(jit, -jit));

    std::vector<double> *zvals;
    vmath::vec3 gp;
    std::vector<int> *tris;
    for (int j = 0; j < ztrigrid.height; j++) {
        for (int i = 0; i < ztrigrid.width; i++) {
            tris = ztrigrid.getPointer(i, j, 0);
            if (tris->size() == 0) {
                continue;
            }

            zvals = zcollisions.getPointer(i, j, 0);
            zvals->reserve(tris->size());
            gp = Grid3d::GridIndexToCellCenter(i, j, -1, dx) + jitter;
            _getTriangleCollisionsZ(gp, *tris, m, *zvals);
        }
    }
}

void getCellsInsideTriangleMesh(
        TriangleMesh &m, int isize, int jsize, int ksize, double dx,
        std::vector<GridIndex> &cells) {

    Array3d<std::vector<double> > zcollisions(isize, jsize, 1);
    _getCollisionGridZ(m, dx, zcollisions);

    std::vector<double> *zvals;
    for (int k = 0; k < ksize; k++) {
        for (int j = 0; j < jsize; j++) {
            for (int i = 0; i < isize; i++) {
                zvals = zcollisions.getPointer(i, j, 0);
                if (zvals->size() % 2 != 0) {
                    continue;
                }

                double z = Grid3d::GridIndexToCellCenter(i, j, k, dx).z;
                int numless = 0;
                for (unsigned int zidx = 0; zidx < zvals->size(); zidx++) {
                    if (zvals->at(zidx) < z) {
                        numless++;
                    }
                }

                if (numless % 2 == 1) {
                    cells.push_back(GridIndex(i, j, k));
                }
            }
        }
    }

}

}

extern "C" {

    EXPORTDLL int utils_get_triangle_mesh_cells_storage_size(
            TriangleMesh_t *mesh_data, double dx, int *err) {

        TriangleMesh mesh;
        utils::structToTriangleMesh(*mesh_data, mesh);
        AABB bbox(mesh.vertices);

        GridIndex goffset = Grid3d::positionToGridIndex(bbox.position, dx);
        vmath::vec3 offset = Grid3d::GridIndexToPosition(goffset, dx);

        bbox.position -= offset;
        int inf = std::numeric_limits<int>::max();
        GridIndex gmin, gmax;
        Grid3d::getGridIndexBounds(bbox, dx, inf, inf, inf, &gmin, &gmax);
        int size = (gmax.i + 1) * (gmax.j + 1) * (gmax.k + 1);

        *err = CBindings::SUCCESS;
        return size;
    }
    
    EXPORTDLL void utils_get_triangle_mesh_cells(
            TriangleMesh_t *mesh_data, double dx, 
            GridIndex_t *output, int outsize, int *numcells, int *err) {

        TriangleMesh mesh;
        utils::structToTriangleMesh(*mesh_data, mesh);
        AABB bbox(mesh.vertices);

        GridIndex goffset = Grid3d::positionToGridIndex(bbox.position, dx);
        vmath::vec3 offset = Grid3d::GridIndexToPosition(goffset, dx);
        mesh.translate(-offset);
        bbox.position -= offset;

        int inf = std::numeric_limits<int>::max();
        GridIndex gmin, gmax;
        Grid3d::getGridIndexBounds(bbox, dx, inf, inf, inf, &gmin, &gmax);

        std::vector<GridIndex> cells;
        cells.reserve(outsize);
        utils::getCellsInsideTriangleMesh(
            mesh, gmax.i + 1, gmax.j + 1, gmax.k + 1, dx, cells
        );

        GridIndex g;
        for (unsigned int i = 0; i < cells.size(); i++) {
            g = cells[i];
            output[i] = GridIndex_t{ g.i + goffset.i, 
                                     g.j + goffset.j, 
                                     g.k + goffset.k };
        }

        *numcells = (int)cells.size();
        *err = CBindings::SUCCESS;
    }
    
}
