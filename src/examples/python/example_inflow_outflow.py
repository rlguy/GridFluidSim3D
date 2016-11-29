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
              "Usage:\tpython example_inflow_outflow.py path/to/directory\n")
    raise ImportError(errmsg)

# This example will add an inflow fluid source whose
# position oscillates between two points.
#
# An outflow fluid source will be placed at one end of
# the simulation domain to drain fluid.
#
# A solid pillar will be placed in the center of the
# simulation domain.
import math
from pyfluid import FluidSimulation, CuboidFluidSource, AABB, Vector3, GridIndex

def get_pillar_cells(position, radius, isize, jsize, ksize, dx):
    rsq = radius*radius
    cells = []
    for k in range(ksize):
        for j in range(jsize):
            for i in range(isize):
                vx = (i + 0.5)*dx - position.x
                vz = (k + 0.5)*dx - position.z
                distsq = vx*vx + vz*vz
                if distsq < rsq:
                    cells.append(GridIndex(i, j, k))

    return cells


isize = 128
jsize = 64
ksize = 64
dx = 0.125
fluidsim = FluidSimulation(isize, jsize, ksize, dx)

width, height, depth = fluidsim.get_simulation_dimensions()

# initialize fluid sources
inflow_bbox = AABB()
inflow_bbox.position = Vector3(0.0, 0.0, 0.0)
inflow_bbox.width = 5*dx
inflow_bbox.height = 15*dx
inflow_bbox.depth = 30*dx
inflow_velocity = Vector3(10.0, 0.0, 0.0)
fluid_inflow = CuboidFluidSource(inflow_bbox, inflow_velocity)
fluid_inflow.is_inflow = True

outflow_bbox = AABB()
outflow_bbox.position = Vector3(width - 5*dx, 0.0, 0.0)
outflow_bbox.width = 10*dx
outflow_bbox.height = height
outflow_bbox.depth = depth
fluid_outflow = CuboidFluidSource(outflow_bbox)
fluid_outflow.is_outflow = True

fluidsim.add_cuboid_fluid_source(fluid_inflow)
fluidsim.add_cuboid_fluid_source(fluid_outflow)

# Create a pillar of solid cells in the center of the domain
center = Vector3(0.5*width, 0.5*height, 0.5*depth)
radius = 10*dx
pillar_cells = get_pillar_cells(center, radius, isize, jsize, ksize, dx)
fluidsim.add_solid_cells(pillar_cells)

fluidsim.add_body_force(0.0, -25.0, 0.0)
fluidsim.initialize()

timestep = 1.0/30.0
runtime = 0.0
while True:
    # Oscillate position of inflow source between two points
    runtime += timestep
    occspeed = 0.5*3.14159
    sinval = 0.5 + 0.5*math.sin(runtime*occspeed)

    p1 = Vector3(0.1*width, 0.15*height, 0.5*depth)
    p2 = Vector3(0.1*width, 0.85*height, 0.5*depth)
    p12 = p2 - p1
    sourcepos = p1 + sinval*p12
    fluid_inflow.center = sourcepos

    fluidsim.update(timestep)
    