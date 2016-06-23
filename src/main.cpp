#include "main.h"

int main() {
    
    // This example will drop a ball of fluid in the center
    // of the fluid simulation domain.

    int isize = 64;
    int jsize = 64;
    int ksize = 64;
    double dx = 0.125;
    FluidSimulation fluidsim(isize, jsize, ksize, dx);

    fluidsim.setSurfaceSubdivisionLevel(2);

    double x, y, z;
    fluidsim.getSimulationDimensions(&x, &y, &z);
    fluidsim.addImplicitFluidPoint(x/2, y/2, z/2, 7.0);
    
    fluidsim.addBodyForce(0.0, -25.0, 0.0);
    fluidsim.initialize();

    double timestep = 1.0 / 30.0;
    for (;;) {
        fluidsim.update(timestep);
    }
    
    return 0;
}
