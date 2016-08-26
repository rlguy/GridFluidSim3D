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
              "Usage:\tpython example_lego_sphere_drop.py path/to/directory\n")
    raise ImportError(errmsg)

# This example will drop a ball of fluid to a pool of resting fluid. The output 
# surface mesh will be generated as LEGO bricks.
#
# The brick surface reconstruction method requires data from three consecutive 
# frames, so data output will not be written to disk until the third frame.

from pyfluid import FluidSimulation, AABB

isize = 128
jsize = 128
ksize = 128
dx = 0.0625
fluidsim = FluidSimulation(isize, jsize, ksize, dx)

fluidsim.enable_isotropic_surface_reconstruction = False

brick = AABB()
brick.width = 3*dx
brick.height = 1.2*brick.width
brick.depth = brick.width
fluidsim.enable_brick_output = (True, brick)

width, height, depth = fluidsim.get_simulation_dimensions()
fluidsim.add_implicit_fluid_point(width/2, height/2, depth/2, 5.0)

cuboid = AABB(0.0, 0.0, 0.0, width, 0.125*height, depth);
fluidsim.add_fluid_cuboid(cuboid)

fluidsim.add_body_force(0.0, -25.0, 0.0)
fluidsim.initialize()

timestep = 1.0/30.0
while True:
    fluidsim.update(timestep)
