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
#include "../../vmath.h"

void example_dambreak() {

	// This example will run a dambreak scenario where
	// a cuboid of fluid is released at one side of the 
	// simulation domain.

    int isize = 128;
    int jsize = 64;
    int ksize = 64;
    double dx = 0.125;
    FluidSimulation fluidsim(isize, jsize, ksize, dx);

    double width, height, depth;
    fluidsim.getSimulationDimensions(&width, &height, &depth);

    AABB bbox;
    bbox.position = vmath::vec3(0, 0, 0);
    bbox.width = 0.25*width;
    bbox.height = 0.75*height;
    bbox.depth = depth;

    fluidsim.addFluidCuboid(bbox);
    
    fluidsim.addBodyForce(0.0, -25.0, 0.0);
    fluidsim.initialize();

    double timestep = 1.0 / 30.0;
    for (;;) {
        fluidsim.update(timestep);
    }
    
}