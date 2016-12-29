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
#ifndef UTILS_H
#define UTILS_H

#include <vector>

#include "trianglemesh.h"
#include "array3d.h"

namespace utils {
    typedef struct TriangleMesh_t {
        float *vertices;
        int *triangles;
        int numVertices;
        int numTriangles;
    } TriangleMesh_t;

    void structToTriangleMesh(TriangleMesh_t &mesh_data, TriangleMesh &mesh);

    void _getTriangleGridZ(
        TriangleMesh &m, double dx, Array3d<std::vector<int> > &ztrigrid);

    void _getTriangleCollisionsZ(
        vmath::vec3 origin, std::vector<int> &indices, TriangleMesh &m,
        std::vector<double> &collisions);

    double _randomDouble(double min, double max);

    void _getCollisionGridZ(
        TriangleMesh &m, double dx, Array3d<std::vector<double> > &zcollisions);

    void getCellsInsideTriangleMesh(
        TriangleMesh &m, int isize, int jsize, int ksize, double dx,
        std::vector<GridIndex> &cells);

    void getCellsInsideTriangleMesh(
        TriangleMesh mesh, double dx, std::vector<GridIndex> &cells);

    void _getCollisionGridZSubd2(
        TriangleMesh &m, double dx, 
        Array3d<std::vector<double> > &zcollisions,
        Array3d<std::vector<double> > &zsubcollisions);

    bool isCellInside(double z, std::vector<double> *zvals);

    bool isCellInside(int i, int j, int k, double dx, std::vector<double> *zvals);

    void sortInsideBorderCells(
        Array3d<bool> &cellgrid, 
        std::vector<GridIndex> &insideCells, 
        std::vector<GridIndex> &borderCells);

    unsigned char getCellFillMask(
        GridIndex g, double dx, Array3d<std::vector<double> > &zsubcollisions);


    void getCellsInsideTriangleMeshSubd2(
        TriangleMesh &m, int isize, int jsize, int ksize, double dx,
        std::vector<GridIndex> &cells, std::vector<unsigned char> &cell_masks);

    void getCellsInsideTriangleMeshSubd2(
        TriangleMesh mesh, double dx,
        std::vector<GridIndex> &cells, std::vector<unsigned char> &cell_masks);

}

#endif