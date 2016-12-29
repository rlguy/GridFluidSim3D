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
#include <math.h>

#include "../../fluidsimulation.h"
#include "../../grid3d.h"
#include "../../vmath.h"

void example_inflow_outflow() {

	// This example will add an inflow fluid source whose
    // position oscillates between two points.
    //
    // An outflow fluid source will be placed at one end of
    // the simulation domain to drain fluid.
    //
    // A solid pillar will be placed in the center of the
    // simulation domain.

    int isize = 128;
    int jsize = 64;
    int ksize = 64;
    double dx = 0.125;
    FluidSimulation fluidsim(isize, jsize, ksize, dx);

    double width, height, depth;
    fluidsim.getSimulationDimensions(&width, &height, &depth);

    // Initialize fluid sources
    AABB inflowAABB;
    inflowAABB.position = vmath::vec3(0.0, 0.0, 0.0);
    inflowAABB.width = 5*dx;
    inflowAABB.height = 15*dx;
    inflowAABB.depth = 30*dx;
    vmath::vec3 inflowVelocity = vmath::vec3(10.0, 0.0, 0.0);
    CuboidFluidSource inflow(inflowAABB, inflowVelocity);

    AABB outflowAABB;
    outflowAABB.position = vmath::vec3(width - 5*dx, 0.0, 0.0);
    outflowAABB.width = 10*dx;
    outflowAABB.height = height;
    outflowAABB.depth = depth;
    CuboidFluidSource outflow(outflowAABB);
    outflow.setAsOutflow();

    fluidsim.addCuboidFluidSource(&inflow);
    fluidsim.addCuboidFluidSource(&outflow);

    // Create a pillar of solid cells in the center of the domain
    std::vector<GridIndex> solidCells;
    vmath::vec3 center(0.5*width, 0.5*height, 0.5*depth);
    double pillarRadius = 10*dx;
    double rsq = pillarRadius * pillarRadius;
    for (int k = 0; k < ksize; k++) {
        for (int j = 0; j < jsize; j++) {
            for (int i = 0; i < isize; i++) {
                vmath::vec3 gpos = Grid3d::GridIndexToCellCenter(i, j, k, dx);
                vmath::vec3 v = gpos - center;
                double distsq = v.x*v.x + v.z*v.z;

                if (distsq < rsq) {
                    solidCells.push_back(GridIndex(i, j, k));
                }
            }
        }
    }
    fluidsim.addSolidCells(solidCells);

    fluidsim.addBodyForce(0.0, -25.0, 0.0);
    fluidsim.initialize();

    double timestep = 1.0 / 30.0;
    double runtime = 0.0;
    for (;;) {
        runtime += timestep;

        // Oscillate position of inflow source between two points
        double ocspeed = 0.5*3.14159;
        double sinval = 0.5 + 0.5*sin(runtime*ocspeed);

        vmath::vec3 p1(0.1*width, 0.15*height, 0.5*depth);
        vmath::vec3 p2(0.1*width, 0.85*height, 0.5*depth);
        vmath::vec3 p12 = p2 - p1;
        vmath::vec3 sourcepos = p1 + sinval*p12;
        inflow.setCenter(sourcepos);

        fluidsim.update(timestep);
    }
    
}