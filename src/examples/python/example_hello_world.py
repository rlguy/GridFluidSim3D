# Copyright (c) 2016 Ryan L. Guy
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
# 
# 1. The origin of this software must not be misrepresented; you must not
#    claim that you wrote the original software. If you use this software
#    in a product, an acknowledgement in the product documentation would be
#    appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be
#    misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.

import sys, os

if len(sys.argv) >= 2: 
    sys.path.append(os.path.abspath(sys.argv[1]))
else:
    filepath = os.path.dirname(__file__)
    guesspath = os.path.abspath(os.path.join(filepath, os.pardir))
    guesspath = os.path.abspath(os.path.join(guesspath, os.pardir))
    sys.path.append(guesspath)

try:
    import pyfluid
except ImportError:
    errmsg = ("Could not find the pyfluid package. Pass the directory that contains the " +
              "pyfluid package as a command line argument and try again. " + 
              "For example, if the package is located at 'build/fluidsim/pyfluid', " +
              "then pass the directory 'build/fluidsim/'\n\n" +
              "Usage:\tpython example_hello_world.py path/to/directory\n")
    raise ImportError(errmsg)


# This is a very basic example of how to use the FluidSimulation class to 
# run a simulation. The simulation in this example will drop a ball of 
# fluid in the center of a cube shaped fluid domain. This example is 
# relatively quick to compute and can be used to test if the simulation 
# program is running correctly.

from pyfluid import FluidSimulation


# The fluid simulator performs its computations on a 3D grid, and because of 
# this the simulation domain is shaped like a rectangular prism. The 
# FluidSimulation class can be initialized with four parameters: the number 
# of grid cells in each direction x, y, and z, and the width of a grid cell.

isize = 32
jsize = 32
ksize = 32
dx = 0.25
fluidsim = FluidSimulation(isize, jsize, ksize, dx)

# We want to add a ball of fluid to the center of the fluid domain, so we will 
# need to get the dimensions of the domain by calling getSimulationDimensions. 
# Alternatively, the dimensions can be calculated by multiplying the cell 
# width by the corresponding number of cells in a direction 
# (e.g. width = dx*isize).

width, height, depth = fluidsim.get_simulation_dimensions()

# Now that we have the dimensions of the simulation domain, we can calculate 
# the center, and add a ball of fluid by calling addImplicitFluidPoint which 
# takes the x, y, and z position and radius as parameters.

centerx = width / 2;
centery = height / 2;
centerz = depth / 2;
radius = 6.0;
fluidsim.add_implicit_fluid_point(centerx, centery, centerz, radius)

# An important note to make about addImplicitFluidPoint is that it will not 
# add a sphere with the specified radius, it will add a sphere with half of 
# the specified radius. An implicit fluid point is represented as a field of 
# values on the simulation grid. The strength of the field values are 1 at the
# point center and falls off towards 0 as distance from the point increases. 
# When the simulation is initialized, fluid particles will be created in 
# regions where the field values are greater than 0.5. This means that if you 
# add a fluid point with a radius of 6.0, the ball of fluid in the simulation 
# will actually be of radius 3.0 since field values will be less than 0.5 at 
# a distance greater than half of the specified radius.

# The FluidSimulation object now has a domain containing some fluid, but the 
# current simulation will not be very interesting as there are no forces 
# acting upon the fluid. We can add the force of gravity by making a call to 
# addBodyForce which takes three values representing a force vector as 
# parameters. We will set the force of gravity to point downwards with a value 
# of 25.0.

gx = 0.0;
gy = -25.0;
gz = 0.0;
fluidsim.add_body_force(gx, gy, gz)

# Now we have a simulation domain with some fluid, and a force acting on the 
# fluid. Before we run the simulation, a call to initialize must be made. Note 
# that any calls to addImplicitFluidPoint must be made before initialize is 
# called.

fluidsim.initialize();

# We will now run the simulation for a total of 30 animation frames at a rate 
# of 30 frames per second by repeatedly making calls to the update function. 
# The update function advances the state of the simulation by a specified 
# period of time. To update the simulation at a rate of 30 frames per second, 
# each call to update will need to be supplied with a time value of 1.0/30.0. 
# Each call to update will generate a triangle mesh that represents the fluid 
# surface. The mesh files will be saved in the output/bakefiles/ directory as 
# a numbered sequence of files stored in the Stanford .PLY file format.

timestep = 1.0 / 30.0;
numframes = 30;
for i in range(numframes):
    fluidsim.update(timestep)

# As this loop runs, the program should output simulation stats and timing 
# metrics to the terminal. After the loop completes, the output/bakefiles/ 
# directory should contain 30 .PLY triangle meshes numbered in sequence from 
# 0 to 29: 000000.ply, 000001.ply, 000002.ply, ..., 000028.ply, 000029.ply.

# The fluid simulation in this example is quick to compute, but of low quality 
# due to the low resolution of the simulation grid. The quality of this 
# simulation can be improved by increasing the simulation dimensions while 
# decreasing the cell size. For example, try simulating on a grid of 
# resolution 64 x 64 x 64 with a cell size of 0.125, or even better, on a grid 
# of resolution 128 x 128 x 128 with a cell size of 0.0625.
