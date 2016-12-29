#include <stdio.h>
#include <iostream>

#include "../utils.h"

#include "cbindings.h"
#include "gridindex_c.h"

#ifdef _WIN32
    #define EXPORTDLL __declspec(dllexport)
#else
    #define EXPORTDLL
#endif

extern "C" {

    EXPORTDLL int utils_get_triangle_mesh_cells_storage_size(
            utils::TriangleMesh_t *mesh_data, double dx, int *err) {

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
            utils::TriangleMesh_t *mesh_data, double dx, 
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

    EXPORTDLL void utils_get_triangle_mesh_cells_subd2(
        utils::TriangleMesh_t *mesh_data, double dx, 
        GridIndex_t *output_cells, unsigned char *output_cell_masks, 
        int outsize, int *numcells, int *err) {

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
        std::vector<unsigned char> cell_masks;
        cells.reserve(outsize);
        cell_masks.reserve(outsize);

        utils::getCellsInsideTriangleMeshSubd2(
            mesh, gmax.i + 1, gmax.j + 1, gmax.k + 1, dx, cells, cell_masks
        );

        GridIndex g;
        for (unsigned int i = 0; i < cells.size(); i++) {
            g = cells[i];
            output_cells[i] = GridIndex_t{ g.i + goffset.i, 
                                           g.j + goffset.j, 
                                           g.k + goffset.k };
            output_cell_masks[i] = cell_masks[i];
        }

        *numcells = cells.size();
        *err = CBindings::SUCCESS;
    }
    
}
