#include "../fluidsimulation.h"

#ifdef _WIN32
    #define EXPORTDLL __declspec(dllexport)
#else
    #define EXPORTDLL
#endif

extern "C" {
	EXPORTDLL FluidSimulation* FluidSimulation_new(){ 
	    std::cout << "Creating FluidSimulation object" << std::endl;
		return new FluidSimulation(); 
	}

	EXPORTDLL void FluidSimulation_destroy(FluidSimulation* obj) {
	    
	    std::cout << "Destroying FluidSimulation object" << std::endl;
	    delete obj;
	}
}