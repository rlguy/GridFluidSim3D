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

void example_lego_sphere_drop() {

	// This example will drop a ball of fluid to a pool
    // of resting fluid. The output surface mesh will be generated
    // as LEGO bricks.
    //
    // The brick surface reconstruction method requires data from
    // three consecutive frames, so data output will not be written
    // to disk until the third frame.

    int isize = 128;
    int jsize = 128;
    int ksize = 128;
    double dx = 0.0625;
    FluidSimulation fluidsim(isize, jsize, ksize, dx);

    fluidsim.disableIsotropicSurfaceReconstruction();

    double brickWidth = 3*dx;
    double brickHeight = 1.2*brickWidth;
    double brickDepth = brickWidth;
    fluidsim.enableBrickOutput(brickWidth, brickHeight, brickDepth);

    double width, height, depth;
    fluidsim.getSimulationDimensions(&width, &height, &depth);
    fluidsim.addImplicitFluidPoint(width/2, height/2, depth/2, 5.0);

    fluidsim.addFluidCuboid(0.0, 0.0, 0.0, width, 0.125*height, depth);
    
    fluidsim.addBodyForce(0.0, -25.0, 0.0);
    fluidsim.initialize();

    double timestep = 1.0 / 30.0;
    for (;;) {
        fluidsim.update(timestep);
    }
    
}