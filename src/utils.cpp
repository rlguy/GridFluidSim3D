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
#include "utils.h"

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

void _getCollisionGridZSubd2(TriangleMesh &m, double dx, 
                             Array3d<std::vector<double> > &zcollisions,
                             Array3d<std::vector<double> > &zsubcollisions) {

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
    for (int j = 0; j < zcollisions.height; j++) {
        for (int i = 0; i < zcollisions.width; i++) {
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

    jit = 0.05*0.5*dx;
    jitter = vmath::vec3(_randomDouble(jit, -jit), 
                         _randomDouble(jit, -jit), 
                         _randomDouble(jit, -jit));
    for (int j = 0; j < zsubcollisions.height; j++) {
        for (int i = 0; i < zsubcollisions.width; i++) {
            int trigridi = (int)floor(0.5*i);
            int trigridj = (int)floor(0.5*j);

            tris = ztrigrid.getPointer(trigridi, trigridj, 0);
            if (tris->size() == 0) {
                continue;
            }

            zvals = zsubcollisions.getPointer(i, j, 0);
            zvals->reserve(tris->size());
            gp = Grid3d::GridIndexToCellCenter(i, j, -1, 0.5*dx) + jitter;
            _getTriangleCollisionsZ(gp, *tris, m, *zvals);
        }
    }

}

bool isCellInside(double z, std::vector<double> *zvals) {
    if (zvals->size() % 2 != 0) {
        return false;
    }

    int numless = 0;
    for (unsigned int zidx = 0; zidx < zvals->size(); zidx++) {
        if (zvals->at(zidx) < z) {
            numless++;
        }
    }

    return numless % 2 == 1;
}

bool isCellInside(int i, int j, int k, double dx, std::vector<double> *zvals) {
    double z = Grid3d::GridIndexToCellCenter(i, j, k, dx).z;
    return isCellInside(z, zvals);
}

void sortInsideBorderCells(Array3d<bool> &cellgrid, 
                           std::vector<GridIndex> &insideCells, 
                           std::vector<GridIndex> &borderCells) {
    
    int isize = cellgrid.width;
    int jsize = cellgrid.height;
    int ksize = cellgrid.depth;
    Array3d<bool> isCellProcessed(isize, jsize, ksize, false);
    for (int k = 0; k < ksize; k++) {
        for (int j = 0; j < jsize; j++) {
            for (int i = 0; i < isize; i++) {
                if (cellgrid(i, j, k)) {
                    isCellProcessed.set(i, j, k, true);
                }
            }
        }
    }

    GridIndex nbs6[6];
    GridIndex nbs26[26];
    GridIndex g;
    for (int k = 0; k < ksize; k++) {
        for (int j = 0; j < jsize; j++) {
            for (int i = 0; i < isize; i++) {
                if (!cellgrid(i, j, k)) {
                    continue;
                }

                Grid3d::getNeighbourGridIndices6(i, j, k, nbs6);
                bool isInside = true;
                for (int nidx = 0; nidx < 6; nidx++) {
                    g = nbs6[nidx];
                    if (!Grid3d::isGridIndexInRange(g, isize, jsize, ksize) || 
                            !cellgrid(g)) {
                        isInside = false;
                        break;
                    }
                }

                if (isInside) {
                    insideCells.push_back(GridIndex(i, j, k));
                    continue;
                }

                borderCells.push_back(GridIndex(i, j, k));
                Grid3d::getNeighbourGridIndices26(i, j, k, nbs26);
                for (int nidx = 0; nidx < 26; nidx++) {
                    g = nbs26[nidx];
                    if (!Grid3d::isGridIndexInRange(g, isize, jsize, ksize) ||
                            isCellProcessed(g)) {
                        continue;
                    }

                    borderCells.push_back(g);
                    isCellProcessed.set(g, true);
                }
            }
        }
    }

}

unsigned char getCellFillMask(GridIndex g, double dx,
                              Array3d<std::vector<double> > &zsubcollisions) {
    unsigned char mask = 0;
    GridIndex subg(2 * g.i, 2 * g.j, 2 * g.k);

    std::vector<double> *zvals = zsubcollisions.getPointer(subg.i, subg.j, 0);
    if (isCellInside(subg.i, subg.j, subg.k, dx, zvals)) {
        mask |= 1;
    }

    zvals = zsubcollisions.getPointer(subg.i + 1, subg.j, 0);
    if (isCellInside(subg.i + 1, subg.j, subg.k, dx, zvals)) {
        mask |= 2;
    }

    zvals = zsubcollisions.getPointer(subg.i, subg.j + 1, 0);
    if (isCellInside(subg.i, subg.j + 1, subg.k, dx, zvals)) {
        mask |= 4;
    }

    zvals = zsubcollisions.getPointer(subg.i + 1, subg.j + 1, 0);
    if (isCellInside(subg.i + 1, subg.j + 1, subg.k, dx, zvals)) {
        mask |= 8;
    }

    zvals = zsubcollisions.getPointer(subg.i, subg.j, 0);
    if (isCellInside(subg.i, subg.j, subg.k + 1, dx, zvals)) {
        mask |= 16;
    }

    zvals = zsubcollisions.getPointer(subg.i + 1, subg.j, 0);
    if (isCellInside(subg.i + 1, subg.j, subg.k + 1, dx, zvals)) {
        mask |= 32;
    }

    zvals = zsubcollisions.getPointer(subg.i, subg.j + 1, 0);
    if (isCellInside(subg.i, subg.j + 1, subg.k + 1, dx, zvals)) {
        mask |= 64;
    }

    zvals = zsubcollisions.getPointer(subg.i + 1, subg.j + 1, 0);
    if (isCellInside(subg.i + 1, subg.j + 1, subg.k + 1, dx, zvals)) {
        mask |= 128;
    }

    return mask;
}

void getCellsInsideTriangleMeshSubd2(
        TriangleMesh &m, int isize, int jsize, int ksize, double dx,
        std::vector<GridIndex> &cells, std::vector<unsigned char> &cell_masks) {

    Array3d<std::vector<double> > zcollisions(isize, jsize, 1);
    Array3d<std::vector<double> > zsubcollisions(2*isize, 2*jsize, 1);
    _getCollisionGridZSubd2(m, dx, zcollisions, zsubcollisions);

    Array3d<bool> cellgrid(isize, jsize, ksize, false);
    std::vector<double> *zvals;
    int cellCount = 0;
    for (int k = 0; k < ksize; k++) {
        for (int j = 0; j < jsize; j++) {
            for (int i = 0; i < isize; i++) {
                zvals = zcollisions.getPointer(i, j, 0);
                if (isCellInside(i, j, k, dx, zvals)) {
                    cellgrid.set(i, j, k, true);
                    cellCount++;
                }
            }
        }
    }

    std::vector<GridIndex> insideCells;
    std::vector<GridIndex> borderCells;
    insideCells.reserve(cellCount / 2);
    borderCells.reserve(cellCount / 2);
    sortInsideBorderCells(cellgrid, insideCells, borderCells);

    Array3d<unsigned char> maskGrid(isize, jsize, ksize, 0);
    for (unsigned int i = 0; i < insideCells.size(); i++) {
        maskGrid.set(insideCells[i], 255);
    }

    double hdx = 0.5 * dx;
    for (unsigned int i = 0; i < borderCells.size(); i++) {
        unsigned char mask = getCellFillMask(borderCells[i], hdx, zsubcollisions);
        maskGrid.set(borderCells[i], mask);
    }

    for (int k = 0; k < ksize; k++) {
        for (int j = 0; j < jsize; j++) {
            for (int i = 0; i < isize; i++) {
                if (maskGrid(i, j, k) != 0) {
                    cells.push_back(GridIndex(i, j, k));
                    cell_masks.push_back(maskGrid(i, j, k));
                }
            }
        }
    }

}

void getCellsInsideTriangleMeshSubd2(
        TriangleMesh mesh, double dx,
        std::vector<GridIndex> &cells, std::vector<unsigned char> &cell_masks) {

    AABB bbox(mesh.vertices);
    GridIndex goffset = Grid3d::positionToGridIndex(bbox.position, dx);
    vmath::vec3 offset = Grid3d::GridIndexToPosition(goffset, dx);
    mesh.translate(-offset);
    bbox.position -= offset;

    int inf = std::numeric_limits<int>::max();
    GridIndex gmin, gmax;
    Grid3d::getGridIndexBounds(bbox, dx, inf, inf, inf, &gmin, &gmax);

    utils::getCellsInsideTriangleMeshSubd2(
        mesh, gmax.i + 1, gmax.j + 1, gmax.k + 1, dx, cells, cell_masks
    );

    for (unsigned int i = 0; i < cells.size(); i++) {
        cells[i].i = cells[i].i + goffset.i;
        cells[i].j = cells[i].j + goffset.j;
        cells[i].k = cells[i].k + goffset.k;
    }
}

void getCellsInsideTriangleMesh(
        TriangleMesh mesh, double dx, std::vector<GridIndex> &cells) {

    AABB bbox(mesh.vertices);
    GridIndex goffset = Grid3d::positionToGridIndex(bbox.position, dx);
    vmath::vec3 offset = Grid3d::GridIndexToPosition(goffset, dx);
    mesh.translate(-offset);
    bbox.position -= offset;

    int inf = std::numeric_limits<int>::max();
    GridIndex gmin, gmax;
    Grid3d::getGridIndexBounds(bbox, dx, inf, inf, inf, &gmin, &gmax);

    utils::getCellsInsideTriangleMesh(
        mesh, gmax.i + 1, gmax.j + 1, gmax.k + 1, dx, cells
    );

    for (unsigned int i = 0; i < cells.size(); i++) {
        cells[i].i = cells[i].i + goffset.i;
        cells[i].j = cells[i].j + goffset.j;
        cells[i].k = cells[i].k + goffset.k;
    }
}

}