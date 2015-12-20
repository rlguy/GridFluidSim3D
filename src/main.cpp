#include "main.h"

int main(int argc, char* args[]) {

    //FluidSimulationSaveState state;
    //bool success = state.loadState("savestates/autosave01.state");
    //assert(success);
    
    int xvoxels = 64;
    int yvoxels = 64;
    int zvoxels = 64;
    double dx = 0.05333;
    FluidSimulation fluidsim(xvoxels, yvoxels, zvoxels, dx);
    //FluidSimulation fluidsim(state);

    fluidsim.enableAnisotropicSurfaceReconstruction();
    //fluidsim.disableSaveState();

    double x, y, z;
    fluidsim.getSimulationDimensions(&x, &y, &z);

    double height = 5*dx;
    AABB bbox1(vmath::vec3(2*dx, 1.0+dx, 5*dx), 1*dx, height, z - 10*dx);
    AABB bbox2(vmath::vec3(x - 1*dx - 2*dx, 1.0+dx, 2*dx), 1*dx, height, z - 10*dx);
    //fluidsim.addCuboidFluidSource(bbox1, vmath::vec3(10.0, 8.0, 0.0));
    //fluidsim.addCuboidFluidSource(bbox2, vmath::vec3(-10.0, 8.0, 0.0));

    double minr = 0.5;
    double maxr = 1.5;
    int n = 0;
    double pad = 1.5;

    for (int i = 0; i < n; i++) {
        double px = pad + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (x - 2*pad)));
        double py = pad + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (y - 2*pad)));
        double pz = pad + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (z - 2*pad)));
        double pr = minr + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (maxr - minr)));
        fluidsim.addImplicitFluidPoint(px, py, pz, pr);
    }

    fluidsim.addImplicitFluidPoint(x/2, y/2, z/2, 3.0);
    fluidsim.addBodyForce(0.0, -40.0, 0.0);

    fluidsim.run();

    double timestep = 1.0 / 60.0;
    while (true) {
        fluidsim.update(timestep);
    }
    
    
	return 0;
}





























