#include "main.h"

int main(int argc, char* args[]) {

    //FluidSimulationSaveState state;
    //bool success = state.loadState("savestates/autosave01.state");
    //assert(success);

    int xvoxels = 64;
    int yvoxels = 64;
    int zvoxels = 64;
    double cellwidth = 0.125;
    FluidSimulation fluidsim(xvoxels, yvoxels, zvoxels, cellwidth);
    //FluidSimulation fluidsim(state);

    fluidsim.enableAnisotropicSurfaceReconstruction();

    double x, y, z;
    fluidsim.getSimulationDimensions(&x, &y, &z);

    fluidsim.addImplicitFluidPoint(x/2, y/2, z/2, 4.2838);
    fluidsim.addBodyForce(0.0, -40.0, 0.0);

    fluidsim.run();

    double timestep = 1.0 / 30.0;
    while (true) {
        fluidsim.update(timestep);
    }

	return 0;
}





























