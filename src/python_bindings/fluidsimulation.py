import ctypes
from fluidlib import lib

class FluidSimulation(object):
    def __init__(self):
        self._obj = lib.FluidSimulation_new()

    def __del__(self):
        lib.FluidSimulation_destroy(self._obj)
