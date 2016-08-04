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

#include <string>
#include <sstream>    // std::ostringstream
#include <fstream>    // std::ofstream, std::ifstream

#include "../../trianglemesh.h"
#include "../../vmath.h"
#include "../../config.h"

// convert number to a string and pad to a width of npad with zeros
std::string numberToPaddedString(int number, int npad) {
    std::ostringstream ss;
    ss << number;
    std::string numstr = ss.str();
    numstr.insert(numstr.begin(), npad - numstr.size(), '0');
    return numstr;
}

/*
    This method will export particle positions as a Stanford .PLY mesh containing
    only vertices that represent particle positions.
*/
void exportParticlesAsPLY(std::vector<vmath::vec3> &particles, std::string framestr) {
    std::string bakefilespath = Config::getBakefilesDirectory();
    std::string filename = bakefilespath + "/particles" + framestr + ".ply";

    TriangleMesh mesh;
    mesh.vertices = particles;
    mesh.writeMeshToPLY(filename);
}

/*
    This method will import particle data written by exportParticlesAsPLY
*/
std::vector<vmath::vec3> importPLYParticles(std::string filename){
    TriangleMesh mesh;
    mesh.loadPLY(filename);

    return mesh.vertices;
}

/*
    This method will export particle positions as a binary file containing xyz coordinates
    for each particle as floats. For example, particles p0 to pn will be written in the form:
   
        [p0x, p0y, p0z, p1x, p1y, p1z, ..., pnx, pny, pnz]
  
    with each number stored as a float.
*/
void exportParticlesAsBinary(std::vector<vmath::vec3> &particles, std::string framestr) {
    std::string bakefilespath = Config::getBakefilesDirectory();
    std::string filename = bakefilespath + "/particles" + framestr + ".data";

    int particleByteSize = 3 * sizeof(float);
    unsigned int dataByteSize = particles.size() * particleByteSize;
    std::ofstream outfile(filename.c_str(), std::ios::out | std::ios::binary);

    char *dataptr = (char*)&(particles[0]);
    outfile.write(dataptr, dataByteSize);
}

/*
    This method will import particle data written by exportParticlesAsBinary
*/
std::vector<vmath::vec3> importBinaryParticles(std::string filename){
    std::ifstream infile(filename.c_str(), std::ios::in | std::ios::binary);

    infile.seekg (0, infile.end);
    unsigned int fileByteSize = infile.tellg();
    infile.seekg (0, infile.beg);

    int particleByteSize = 3 * sizeof(float);
    unsigned int numParticles = fileByteSize / particleByteSize;
    std::vector<vmath::vec3> particles(numParticles);

    char *dataptr = (char*)&(particles[0]);
    infile.read(dataptr, fileByteSize);

    return particles;
}

/*
    This method will export particle positions as a plaintext file containing xyz coordinates
    for each particle. Each number will be delimited by a newline character (\n). For example, 
    particles p0 to pn will be written in the form:

        p0x
        p0y
        p0z
        p1x
        p1y
        p1z
        ...
        pnx
        pny
        pnz
*/
void exportParticlesAsPlaintext(std::vector<vmath::vec3> &particles, std::string framestr) {
    std::string bakefilespath = Config::getBakefilesDirectory();
    std::string filename = bakefilespath + "/particles" + framestr + ".txt";

    std::ostringstream writestr;
    for (unsigned int i = 0; i < particles.size(); i++) {
        writestr << particles[i].x << "\n" << 
                    particles[i].y << "\n" << 
                    particles[i].z << "\n";
    }

    std::ofstream outfile(filename);
    outfile << writestr.str();
}


/*
    This method will import particle data written by exportParticlesAsPlaintext
*/
std::vector<vmath::vec3> importPlaintextParticles(std::string filename){
    std::ifstream infile(filename);

    std::vector<vmath::vec3> particles;
    vmath::vec3 p;
    while (!infile.eof()) {
        infile >> p.x >> p.y >> p.z;
        particles.push_back(p);
    }

    return particles;
}

void example_export_particle_positions() {

    //  Exporting the simulation particle position data can be useful if you
    //  want to import the particles into an external program.
    //
	//  This example will demonstrate how to export fluid simulation
    //  particle positions in threee different forms: as a .ply mesh of
    //  vertices, as a binary file of xyz coordinates, and as a plain text
    //  file of coordinates.
    //
    //  This example also provides methods on how to import the written
    //  particle data back into a std::vector<vmath::vec3>

    int isize = 32;
    int jsize = 32;
    int ksize = 32;
    double dx = 0.25;
    FluidSimulation fluidsim(isize, jsize, ksize, dx);

    // Disable writing the surface mesh to disk.
    fluidsim.disableSurfaceMeshOutput();

    double width, height, depth;
    fluidsim.getSimulationDimensions(&width, &height, &depth);
    fluidsim.addImplicitFluidPoint(width/2, height/2, depth/2, 6.0);
    
    fluidsim.addBodyForce(0.0, -25.0, 0.0);
    fluidsim.initialize();

    double timestep = 1.0 / 30.0;
    for (;;) {

        // Create a string from the current frame number to be used
        // for saving files as a numbered sequence
        int frameno = fluidsim.getCurrentFrame();
        int npad = 6;
        std::string framestring = numberToPaddedString(frameno, npad);

        std::vector<vmath::vec3> particles = fluidsim.getMarkerParticlePositions();

        // Uncomment method to choose which data format to export
        exportParticlesAsPLY(particles, framestring);
        //exportParticlesAsBinary(particles, framestring);
        //exportParticlesAsPlaintext(particles, framestring);

        fluidsim.update(timestep);
    }
    
}
