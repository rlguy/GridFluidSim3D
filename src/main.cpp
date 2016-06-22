#include "main.h"

int main(int argc, char* args[]) {
    
    // This example will drop a ball of fluid in the center
    // of the fluid simulation domain.

    std::cout << "Starting Program" << std::endl;

    int isize = 32;
    int jsize = 32;
    int ksize = 32;
    double dx = 0.25;
    FluidSimulation fluidsim(isize, jsize, ksize, dx);

    std::cout << "constructor" << std::endl;

    //fluidsim.setSurfaceSubdivisionLevel(2);

    double x, y, z;
    fluidsim.getSimulationDimensions(&x, &y, &z);
    fluidsim.addImplicitFluidPoint(x/2, y/2, z/2, 5.0);

    std::cout << "add point" << std::endl;
    
    fluidsim.addBodyForce(0.0, -25.0, 0.0);
    fluidsim.initialize();

    std::cout << "initialize" << std::endl;

    double timestep = 1.0 / 30.0;
    while (true) {
        fluidsim.update(timestep);
    }
    
    return 0;
}





