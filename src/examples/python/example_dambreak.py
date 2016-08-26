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
              "Usage:\tpython example_dambreak.py path/to/directory\n")
    raise ImportError(errmsg)

# This example will run a dambreak scenario where a cuboid of fluid is released 
# at one side of the simulation domain.
from pyfluid import FluidSimulation, AABB, Vector3

isize = 128
jsize = 64
ksize = 64
dx = 0.125
fluidsim = FluidSimulation(isize, jsize, ksize, dx)

width, height, depth = fluidsim.get_simulation_dimensions()
cuboid = AABB(Vector3(0, 0, 0), 0.25*width, 0.75*height, depth)
fluidsim.add_fluid_cuboid(cuboid)

fluidsim.add_body_force(0.0, -25.0, 0.0)
fluidsim.initialize()

timestep = 1.0/30.0
while True:
    fluidsim.update(timestep)
