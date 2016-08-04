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
#include <string>

#include "../../fluidsimulation.h"
#include "../../fluidsimulationsavestate.h"
#include "../../config.h"
#include "../../fluidsimassert.h"

void example_save_state(std::string filename) {

    int isize = 64;
    int jsize = 64;
    int ksize = 64;
    double dx = 0.125;
    FluidSimulation fluidsim(isize, jsize, ksize, dx);

    fluidsim.disableAutosave();

    double width, height, depth;
    fluidsim.getSimulationDimensions(&width, &height, &depth);
    fluidsim.addImplicitFluidPoint(width/2, 0.0, depth/2, 7.0);
    
    fluidsim.addBodyForce(0.0, -25.0, 0.0);
    fluidsim.initialize();

    double timestep = 1.0 / 30.0;
    int numFrames = 5;
    while (fluidsim.getCurrentFrame() < numFrames) {
        fluidsim.update(timestep);
    }
    
    fluidsim.saveState(filename);
}

void example_load_state(std::string filename) {

    FluidSimulationSaveState state;
    bool success = state.loadState(filename);
    FLUIDSIM_ASSERT(success);

    FluidSimulation fluidsim(state);

    /*
        After constructing the FluidSimulation object, the save state is
        no longer needed and can be closed. Closing the state will
        clean up any temporary files that were created during loading
        the state and initializing the fluid simulation.
    */
    state.closeState();

    /*
        The save state feature only saves the following data:
            - grid dimensions and cell size
            - current frame
            - marker particles
            - diffuse particles
            - solid cells
            - FluidBrickGrid data

        The save state will not load any changed settings, body forces,
        or fluid sources, so these attributes/objects will need to be re-added
        manually if desired once the state is loaded.

        The initialize() method does not need to be called when loading a simulation
        from a save state. The simulation is initialized within the constructor.
    */

    fluidsim.disableAutosave();
    fluidsim.addBodyForce(0.0, -25.0, 0.0);

    double timestep = 1.0 / 30.0;
    int numFrames = 5;
    int maxFrameNo = fluidsim.getCurrentFrame() + numFrames;
    while (fluidsim.getCurrentFrame() < maxFrameNo) {
        fluidsim.update(timestep);
    }
}

void example_save_load_state() {
    
    // The fluid simulator generates an autosave state
    // at the start of each frame by default.
    //
    // This example will turn off the autosave feature and demonstrate
    // how to manually save and load a savestate.

    std::string dir = Config::getSavestatesDirectory();
    std::string filename = dir + "/mySaveState.state";
    example_save_state(filename);
    example_load_state(filename);
}





