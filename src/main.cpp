#include "main.h"

int main(int argc, char* args[]) {

    //FluidSimulationSaveState state;
    //bool success = state.loadState("savestates/autosave01.state");
    //assert(success);
    
    int xvoxels = 128;
    int yvoxels = 128;
    int zvoxels = 128;
    double cellwidth = 0.0625;
    FluidSimulation fluidsim(xvoxels, yvoxels, zvoxels, cellwidth);
    //FluidSimulation fluidsim(state);

    fluidsim.enableAnisotropicSurfaceReconstruction();
    fluidsim.disableSaveState();

    double x, y, z;
    fluidsim.getSimulationDimensions(&x, &y, &z);

    fluidsim.addImplicitFluidPoint(x/2, y/2-1.0, z/2, 7.28);
    fluidsim.addBodyForce(0.0, -40.0, 0.0);

    fluidsim.run();

    double timestep = 1.0 / 30.0;
    while (true) {
        fluidsim.update(timestep);
    }
    
	return 0;
}





























