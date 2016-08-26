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
              "Usage:\tpython example_save_load_state.py path/to/directory\n")
    raise ImportError(errmsg)


# The fluid simulator generates an autosave state at the start of each frame 
# by default.
#
# This example will turn off the autosave feature and demonstrate how to 
# manually save and load a savestate.

from pyfluid import FluidSimulation, FluidSimulationSaveState
from pyfluid import config

def save_state(filename):
    isize = 64
    jsize = 64
    ksize = 64
    dx = 0.125
    fluidsim = FluidSimulation(isize, jsize, ksize, dx)

    width, height, depth = fluidsim.get_simulation_dimensions()
    fluidsim.add_implicit_fluid_point(width/2, 0.0, depth/2, 7.0)

    fluidsim.add_body_force(0.0, -25.0, 0.0)
    fluidsim.enable_autosave = False
    fluidsim.initialize()

    timestep = 1.0 / 30.0
    num_frames = 5
    for i in range(num_frames):
        fluidsim.update(timestep)

    fluidsim.save_state(filename)

def load_state(filename):
    state = FluidSimulationSaveState();
    success = state.load_state(filename)

    if not success:
        raise RuntimeError("Unable to load savestate: " + filename)

    fluidsim = FluidSimulation.from_save_state(state)

     # After constructing the FluidSimulation object, the save state is
     # no longer needed and can be closed. Closing the state will
     # clean up any temporary files that were created during loading
     # the state and initializing the fluid simulation.
    state.close_state()

    # The save state feature only saves the following data:
    #     - grid dimensions and cell size
    #     - current frame
    #     - marker particles
    #     - diffuse particles
    #     - solid cells
    #     - FluidBrickGrid data
    # 
    # The save state will not load any changed settings, body forces,
    # or fluid sources, so these attributes/objects will need to be re-added
    # manually if desired once the state is loaded.

    # The initialize() method does not need to be called when loading a simulation
    # from a save state. The simulation is initialized within the constructor.

    fluidsim.add_body_force(0.0, -25.0, 0.0)
    fluidsim.enable_autosave = False
    
    timestep = 1.0 / 30.0
    num_frames = 5
    for i in range(num_frames):
        fluidsim.update(timestep)


def main():
    directory = config.get_savestates_directory()
    filename = directory + "/mySaveState.state"
    save_state(filename)
    load_state(filename)

if __name__ == "__main__":
    main()