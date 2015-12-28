#include "main.h"

#include "FragmentedVector.h"

int main(int argc, char* args[]) {

    
    FluidSimulationSaveState state;
    bool success = state.loadState("savestates/autosave02.state");
    assert(success);

    
    int xvoxels = 150;
    int yvoxels = 150;
    int zvoxels = 150;
    double dx = 0.0535;
    //FluidSimulation fluidsim(xvoxels, yvoxels, zvoxels, dx);
    FluidSimulation fluidsim(state);

    fluidsim.enableAnisotropicSurfaceReconstruction();
    //fluidsim.enableBrickOutput(0.25, 0.25, 0.25);
    //fluidsim.disableSaveState();

    double x, y, z;
    fluidsim.getSimulationDimensions(&x, &y, &z);

    double height = 0.3*x;
    AABB bbox1(vmath::vec3(2*dx, 2*dx, dx), 3*dx, height, z - 2*dx);
    fluidsim.addCuboidFluidSource(bbox1, vmath::vec3(8.0, 0.0, 0.0));

    double minr = 0.5;
    double maxr = 1.5;
    int n = 0;
    double pad = 1.5;

    for (int i = 0; i < n; i++) {
        double px = pad + (float)(rand()) / ((float)(RAND_MAX / (x - 2*pad)));
        double py = pad + (float)(rand()) / ((float)(RAND_MAX / (y - 2*pad)));
        double pz = pad + (float)(rand()) / ((float)(RAND_MAX / (z - 2*pad)));
        double pr = minr + (float)(rand()) / ((float)(RAND_MAX / (maxr - minr)));
        fluidsim.addImplicitFluidPoint(px, py, pz, pr);
    }

    //fluidsim.addImplicitFluidPoint(x/2, y/2, z/2, 7.0);
    fluidsim.addBodyForce(0.0, -40.0, 0.0);

    
    double starti = 128;
    for (int k = 64 - 10; k <= 64 + 10; k++) {
        for (int j = 1; j <= 30; j++) {
            for (int i = starti; i <= starti + 8; i++) {
                fluidsim.addSolidCell(i, j, k);
            }
        }
    }

    for (int k = 32 - 10; k <= 32 + 10; k++) {
        for (int j = 1; j <= 20; j++) {
            for (int i = starti; i <= starti + 5; i++) {
                fluidsim.addSolidCell(i, j, k);
            }
        }
    }

    for (int k = 96 - 10; k <= 96 + 10; k++) {
        for (int j = 1; j <= 20; j++) {
            for (int i = starti; i <= starti + 5; i++) {
                fluidsim.addSolidCell(i, j, k);
            }
        }
    }
    

    fluidsim.run();

    double timestep = 1.0 / 300.0;
    while (true) {
        fluidsim.update(timestep);
    }
    
    
	return 0;
}









