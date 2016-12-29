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
#include "../../fluidsimulation.h"
#include "../../trianglemesh.h"
#include "../../array3d.h"
#include "../../utils.h"
#include "../../vmath.h"

#include <fstream>

bool file_exists(std::string filename) {
    std::ifstream infile(filename);
    return infile.good();
}

bool add_mesh_obstacle(std::string meshpath, FluidSimulation *fluidsim) {

    if (!file_exists(meshpath)) {
        std::cout << "ERROR: Could not open file [" << meshpath << "]" << std::endl;
        return false;
    }

    TriangleMesh mesh;
    bool success = mesh.loadPLY(meshpath);
    if (!success) {
        std::cout << "ERROR: Could not load PLY file [" << meshpath << "]" << std::endl;
        return false;
    }

    std::cout << "Loaded PLY mesh with " << mesh.vertices.size() << " vertices " <<
                 "and " << mesh.triangles.size() << " triangles." << std::endl;

    // uncomment to translate mesh
    //mesh.translate(vmath::vec3(4.0, 0.0, 0.0));

    int isize, jsize, ksize;
    fluidsim->getGridDimensions(&isize, &jsize, &ksize);
    double dx = fluidsim->getCellSize();

    std::vector<GridIndex> cells;
    utils::getCellsInsideTriangleMesh(mesh, dx, cells);

    // Filter out cells that lie outside of the simulation domain
    std::vector<GridIndex> cellsInsideDomain;
    for (size_t i = 0; i < cells.size(); i++) {
        GridIndex g = cells[i];
        if (g.i >= 0 && g.j >= 0 && g.k >= 0 && g.i < isize && g.j < jsize && g.k < ksize) {
            cellsInsideDomain.push_back(g);
        }
    }

    std::cout << "Converted mesh into " << cellsInsideDomain.size() << " grid cells." << std::endl;

    fluidsim->addSolidCells(cellsInsideDomain);

    return true;
}

void example_import_mesh_obstacle() {

    /*
	   This example will demonstrate how to convert a triangle mesh to a set
       of grid cells that can be added to the fluid domain as a solid object.
      
       Notes: - the triangle mesh must be watertight (no holes) to be accurately
                converted into solid cells

              - all faces in the triangle mesh must be triangles

              - the triangle mesh must be in the stanford .PLY file format
                (http://paulbourke.net/dataformats/ply/)

              - the .PLY mesh header must be in the following format:

                    ply
                    format binary_little_endian 1.0
                    element vertex REPLACE_WITH_NUM_VERTICES
                    property float x
                    property float y
                    property float z
                    element face REPLACE_WITH_NUM_FACES
                    property list uchar int vertex_index
                    end_header
    */

    int isize = 64;
    int jsize = 64;
    int ksize = 64;
    double dx = 0.125;
    FluidSimulation fluidsim(isize, jsize, ksize, dx);

    double width, height, depth;
    fluidsim.getSimulationDimensions(&width, &height, &depth);
    fluidsim.addImplicitFluidPoint(width/2, height/2, depth/2, 7.0);
    
    fluidsim.addBodyForce(0.0, 0.0, -25.0);
    fluidsim.initialize();

    std::string meshpath = "PATH_TO_PLY_FILE.ply";
    bool success = add_mesh_obstacle(meshpath, &fluidsim);
    if (!success) {
        return;
    }

    double timestep = 1.0 / 30.0;
    for (;;) {
        fluidsim.update(timestep);
    }
    
}