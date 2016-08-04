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
#include <math.h>

#include "../../fluidsimulation.h"
#include "../../grid3d.h"
#include "../../vmath.h"

std::vector<GridIndex> getCellsWithinRadius(vmath::vec3 position, double radius, FluidSimulation *fluidsim) {
 
    int isize, ksize, jsize;
    fluidsim->getGridDimensions(&isize, &jsize, &ksize);
    double dx = fluidsim->getCellSize();
 
    std::vector<GridIndex> cells;
    double rsq = radius*radius;
    for (int k = 0; k < ksize; k++) {
        for (int j = 0; j < jsize; j++) {
            for (int i = 0; i < isize; i++) {
                vmath::vec3 v = Grid3d::GridIndexToCellCenter(i, j, k, dx) - position;
                if (v.lengthsq() < rsq) {
                    cells.push_back(GridIndex(i, j, k));
                }
            }
        }  
    }
 
    return cells;
}
 
vmath::vec3 rotateY(vmath::vec3 v, float rads) {
    float x = v.z*sin(rads) + v.x*cos(rads);
    float y = v.y;
    float z = v.z*cos(rads) - v.x*sin(rads);
 
    return vmath::vec3(x, y, z);
}
 
int ISIZE = 256;
int JSIZE = 256;
int KSIZE = 256;
double DX = 0.0625;
vmath::vec3 gravityForceField(vmath::vec3 p) {
    vmath::vec3 center(0.5*(double)ISIZE*DX,
                       0.5*(double)JSIZE*DX,
                       0.5*(double)KSIZE*DX);
 
    double ming = 1.0;
    double maxg = 25.0;
    double mindsq = 1.0;
    double maxdsq = 8.0*8.0;
 
    vmath::vec3 v = center - p;
    double distsq = v.lengthsq();
 
    double eps = 1e-6;
    if (distsq < eps) {
        return vmath::vec3();
    }
 
    double distfactor = 1.0 - (distsq - mindsq) / (maxdsq - mindsq);
    double gstrength = ming + distfactor*(maxg - ming);
 
    return (float)gstrength*v.normalize();
}
 
void example_gravity_field() {
    
    // This example will recreate this fluid simulation:
    //      https://gfycat.com/SphericalHandsomeIndochinesetiger
    //
    // Fluid is emitted from a spherical fluid source that oscillates
    // its emission direction.
    //
    // The fluid in the simulation domain will be under the influence
    // of a variable gravity force field. The force field generates a gravity
    // vector towards a sphere in the center of the fluid domain with force
    // strength falling off proportional to radius^2.

    int isize = ISIZE;
    int jsize = JSIZE;
    int ksize = KSIZE;
    double dx = DX;
    FluidSimulation fluidsim(isize, jsize, ksize, dx);
 
    fluidsim.setSurfaceSubdivisionLevel(2);
    fluidsim.setNumPolygonizerSlices(4);
 
    double width, height, depth;
    fluidsim.getSimulationDimensions(&width, &height, &depth);
 
    vmath::vec3 center(0.5*width, 0.5*height, 0.5*depth);
    double radius = 2.0;
    std::vector<GridIndex> solidCells = getCellsWithinRadius(center, radius, &fluidsim);
    fluidsim.addSolidCells(solidCells);
   
    vmath::vec3 emitterPosition(0.5*width, 0.9*height, 0.5*depth);
    vmath::vec3 emitterVelocity(8.0, 0.0, 0.0);
    double emitterRadius = 1.0;
    SphericalFluidSource emitter(emitterPosition, emitterRadius, emitterVelocity);
    fluidsim.addSphericalFluidSource(&emitter);

    fluidsim.addBodyForce(gravityForceField);
 
    fluidsim.initialize();
 
    double timestep = 1.0/60.0;
    double simulationTime = 0.0;
    for (;;) {
 
        int frameno = fluidsim.getCurrentFrame();
        if (frameno >= 200) {
            emitter.deactivate();
        }
 
        double pi = 3.14159265;
        double minAngle = -0.25*pi;
        double maxAngle = 0.25*pi;
        double rotationSpeed = 0.75*pi;
        double rotationFactor = sin(rotationSpeed*simulationTime);
        double rads = minAngle + rotationFactor*(maxAngle - minAngle);
        vmath::vec3 rotatedVelocity = rotateY(emitterVelocity, rads);
        emitter.setVelocity(rotatedVelocity);
 
        fluidsim.update(timestep);
 
        simulationTime += timestep;
    }
   
}