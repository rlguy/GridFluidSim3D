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

void example_sphere_drop() {

	// This example will drop a ball of fluid in the center
    // of a rectangular fluid simulation domain.

    int isize = 256;
    int jsize = 128;
    int ksize = 128;
    double dx = 0.0625;
    FluidSimulation fluidsim(isize, jsize, ksize, dx);

    // Increase subdivision level to increase the resolution that
    // the output meshes are generated at.
    int subdivisionLevel = 1;
    fluidsim.setSurfaceSubdivisionLevel(subdivisionLevel);

    if (subdivisionLevel >= 2) {
    	// Helps reduce output filesize by removing polyhedrons
    	// that do not meet a minimum triangle count threshold.
    	fluidsim.setMinPolyhedronTriangleCount(64);
    }

    double width, height, depth;
    fluidsim.getSimulationDimensions(&width, &height, &depth);
    fluidsim.addImplicitFluidPoint(width/2, height/2, depth/2, 7.0);
    
    fluidsim.addBodyForce(0.0, -25.0, 0.0);
    fluidsim.initialize();

    double timestep = 1.0 / 30.0;
    for (;;) {
        fluidsim.update(timestep);
    }
    
}