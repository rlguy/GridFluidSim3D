import ctypes
from fluidlib import lib
from vector3 import Vector3, Vector3_t
from gridindex import GridIndex, GridIndex_t

def _check_load_state_initialized(func):
    def wrapper(*args, **kwargs):
        self = args[0]
        if isinstance(self, FluidSimulationSaveState):
            self._check_load_state()
        return func(*args, **kwargs)

    return wrapper

class FluidSimulationSaveState(object):
    def __init__(self):
        self._obj = lib.FluidSimulationSaveState_new()

    def __del__(self):
        lib.FluidSimulationSaveState_destroy(self._obj)

    def __call__(self):
        return self._obj

    def save_state(self, filename, fluidsimulation):
    	cfilename = ctypes.c_char_p(filename)
        lib.FluidSimulationSaveState_save_state(self(), 
                                                cfilename, 
                                                fluidsimulation())

    def load_state(self, filename):
        cfilename = ctypes.c_char_p(filename)
        success = lib.FluidSimulationSaveState_load_state(self(), cfilename)
        return bool(success)

    def close_state(self):
        lib.FluidSimulationSaveState_close_state(self())

    @_check_load_state_initialized
    def get_grid_dimensions(self):
        i = ctypes.c_int()
        j = ctypes.c_int()
        k = ctypes.c_int()
        lib.FluidSimulationSaveState_get_grid_dimensions(self(), 
                                                         ctypes.byref(i), 
                                                         ctypes.byref(j), 
                                                         ctypes.byref(k))
        return GridIndex(i.value, j.value, k.value)

    @_check_load_state_initialized
    def get_cell_size(self):
        return float(lib.FluidSimulationSaveState_get_cell_size(self()))

    @_check_load_state_initialized
    def get_current_frame(self):
        return int(lib.FluidSimulationSaveState_get_current_frame(self()))

    @_check_load_state_initialized
    def get_num_marker_particles(self):
        return int(lib.FluidSimulationSaveState_get_num_marker_particles(self()))
        
    @_check_load_state_initialized
    def get_num_diffuse_particles(self):
        return int(lib.FluidSimulationSaveState_get_num_diffuse_particles(self()))
        
    @_check_load_state_initialized
    def get_num_solid_cells(self):
        return int(lib.FluidSimulationSaveState_get_num_solid_cells(self()))

    @_check_load_state_initialized
    def get_marker_particle_positions(self, startidx = None, endidx = None):
        nparticles = self.get_num_marker_particles()
        startidx, endidx = self._check_range(startidx, endidx, 0, nparticles)
        n = endidx - startidx
        out = (Vector3_t * n)()
        lib.FluidSimulationSaveState_get_marker_particle_positions(self(), 
                                                                   startidx,
                                                                   endidx,
                                                                   out)
        return out

    @_check_load_state_initialized
    def get_marker_particle_velocities(self, startidx = None, endidx = None):
        nparticles = self.get_num_marker_particles()
        startidx, endidx = self._check_range(startidx, endidx, 0, self.get_num_marker_particles())
        n = endidx - startidx
        out = (Vector3_t * n)()
        lib.FluidSimulationSaveState_get_marker_particle_velocities(self(), 
                                                                    startidx,
                                                                    endidx,
                                                                    out)
        return out

    @_check_load_state_initialized
    def get_diffuse_particle_positions(self, startidx = None, endidx = None):
        nparticles = self.get_num_diffuse_particles()
        startidx, endidx = self._check_range(startidx, endidx, 0,  nparticles)
        n = endidx - startidx
        out = (Vector3_t * n)()
        lib.FluidSimulationSaveState_get_diffuse_particle_positions(self(), 
                                                                    startidx,
                                                                    endidx,
                                                                    out)
        return out

    @_check_load_state_initialized
    def get_diffuse_particle_velocities(self, startidx = None, endidx = None):
        nparticles = self.get_num_diffuse_particles()
        startidx, endidx = self._check_range(startidx, endidx, 0,  nparticles)
        n = endidx - startidx
        out = (Vector3_t * n)()
        lib.FluidSimulationSaveState_get_diffuse_particle_velocities(self(), 
                                                                     startidx,
                                                                     endidx,
                                                                     out)
        return out

    @_check_load_state_initialized
    def get_diffuse_particle_lifetimes(self, startidx = None, endidx = None):
        nparticles = self.get_num_diffuse_particles()
        startidx, endidx = self._check_range(startidx, endidx, 0,  nparticles)
        n = endidx - startidx
        out = (ctypes.c_float * n)()
        lib.FluidSimulationSaveState_get_diffuse_particle_lifetimes(self(), 
                                                                    startidx,
                                                                    endidx,
                                                                    out)
        lifetimes = [0.0]*n
        for i in xrange(n):
            lifetimes[i] = out[i]
        return lifetimes

    @_check_load_state_initialized
    def get_diffuse_particle_types(self, startidx = None, endidx = None):
        nparticles = self.get_num_diffuse_particles()
        startidx, endidx = self._check_range(startidx, endidx, 0, nparticles)
        n = endidx - startidx
        out = (ctypes.c_char * n)()
        lib.FluidSimulationSaveState_get_diffuse_particle_types(self(), 
                                                                startidx,
                                                                endidx,
                                                                out)
        types = [0]*n
        for i in xrange(n):
            types[i] = ord(out[i])
        return types

    @_check_load_state_initialized
    def get_solid_cells(self, startidx = None, endidx = None):
        ncells = self.get_num_solid_cells()
        startidx, endidx = self._check_range(startidx, endidx, 0, ncells)
        n = endidx - startidx
        out = (GridIndex_t * n)()
        lib.FluidSimulationSaveState_get_solid_cells(self(), 
                                                     startidx,
                                                     endidx,
                                                     out)
        return out

    @_check_load_state_initialized
    def is_fluid_brick_grid_enabled(self):
        return bool(lib.FluidSimulationSaveState_is_fluid_brick_grid_enabled(self()))

    def is_load_state_initialized(self):
        return bool(lib.FluidSimulationSaveState_is_load_state_initialized(self()))

    def _check_range(self, startidx, endidx, minidx, maxidx):
        if startidx is None:
            startidx = minidx
        if endidx is None:
            endidx = maxidx

        if not isinstance(startidx, int) or not isinstance(endidx, int):
            raise TypeError("Index range must be integers")
        if startidx < minidx:
            raise IndexError("startidx out of range: " + str(startidx))
        if endidx > maxidx:
            raise IndexError("endidx out of range: " + str(endidx))
        if endidx < startidx:
            endidx = startidx

        return startidx, endidx

    def _check_load_state(self):
        if not self.is_load_state_initialized():
            raise RuntimeError("Savestate must be loaded to use this method.")


def __init__(lib):
    lib.FluidSimulationSaveState_new.argtypes = []
    lib.FluidSimulationSaveState_new.restype = ctypes.c_void_p

    lib.FluidSimulationSaveState_destroy.argtypes = [ctypes.c_void_p]
    lib.FluidSimulationSaveState_destroy.restype = None

    lib.FluidSimulationSaveState_save_state.argtypes = [ctypes.c_void_p, 
                                                        ctypes.c_char_p, 
                                                        ctypes.c_void_p]
    lib.FluidSimulationSaveState_save_state.restype = None

    lib.FluidSimulationSaveState_load_state.argtypes = [ctypes.c_void_p, 
                                                        ctypes.c_char_p]
    lib.FluidSimulationSaveState_load_state.restype = ctypes.c_int

    lib.FluidSimulationSaveState_close_state.argtypes = [ctypes.c_void_p]
    lib.FluidSimulationSaveState_close_state.restype = None

    lib.FluidSimulationSaveState_get_grid_dimensions.argtypes = [ctypes.c_void_p,
                                                                 ctypes.c_void_p,
                                                                 ctypes.c_void_p,
                                                                 ctypes.c_void_p,]
    lib.FluidSimulationSaveState_get_grid_dimensions.restype = None

    lib.FluidSimulationSaveState_get_cell_size.argtypes = [ctypes.c_void_p]
    lib.FluidSimulationSaveState_get_cell_size.restype = ctypes.c_double

    lib.FluidSimulationSaveState_get_current_frame.argtypes = [ctypes.c_void_p]
    lib.FluidSimulationSaveState_get_current_frame.restype = ctypes.c_int

    lib.FluidSimulationSaveState_get_num_marker_particles.argtypes = [ctypes.c_void_p]
    lib.FluidSimulationSaveState_get_num_marker_particles.restype = ctypes.c_int

    lib.FluidSimulationSaveState_get_num_diffuse_particles.argtypes = [ctypes.c_void_p]
    lib.FluidSimulationSaveState_get_num_diffuse_particles.restype = ctypes.c_int

    lib.FluidSimulationSaveState_get_num_solid_cells.argtypes = [ctypes.c_void_p]
    lib.FluidSimulationSaveState_get_num_solid_cells.restype = ctypes.c_int

    lib.FluidSimulationSaveState_get_marker_particle_positions.argtypes = [ctypes.c_void_p,
                                                                           ctypes.c_int,
                                                                           ctypes.c_int,
                                                                           ctypes.c_void_p]
    lib.FluidSimulationSaveState_get_marker_particle_positions.restype = None

    lib.FluidSimulationSaveState_get_marker_particle_velocities.argtypes = [ctypes.c_void_p,
                                                                            ctypes.c_int,
                                                                            ctypes.c_int,
                                                                            ctypes.c_void_p]
    lib.FluidSimulationSaveState_get_marker_particle_velocities.restype = None

    lib.FluidSimulationSaveState_get_diffuse_particle_positions.argtypes = [ctypes.c_void_p,
                                                                            ctypes.c_int,
                                                                            ctypes.c_int,
                                                                            ctypes.c_void_p]
    lib.FluidSimulationSaveState_get_diffuse_particle_positions.restype = None

    lib.FluidSimulationSaveState_get_diffuse_particle_velocities.argtypes = [ctypes.c_void_p,
                                                                             ctypes.c_int,
                                                                             ctypes.c_int,
                                                                             ctypes.c_void_p]
    lib.FluidSimulationSaveState_get_diffuse_particle_velocities.restype = None

    lib.FluidSimulationSaveState_get_diffuse_particle_lifetimes.argtypes = [ctypes.c_void_p,
                                                                            ctypes.c_int,
                                                                            ctypes.c_int,
                                                                            ctypes.c_void_p]
    lib.FluidSimulationSaveState_get_diffuse_particle_lifetimes.restype = None

    lib.FluidSimulationSaveState_get_diffuse_particle_types.argtypes = [ctypes.c_void_p,
                                                                        ctypes.c_int,
                                                                        ctypes.c_int,
                                                                        ctypes.c_void_p]
    lib.FluidSimulationSaveState_get_diffuse_particle_types.restype = None

    lib.FluidSimulationSaveState_get_solid_cells.argtypes = [ctypes.c_void_p,
                                                             ctypes.c_int,
                                                             ctypes.c_int,
                                                             ctypes.c_void_p]
    lib.FluidSimulationSaveState_get_solid_cells.restype = None

    lib.FluidSimulationSaveState_is_fluid_brick_grid_enabled.argtypes = [ctypes.c_void_p]
    lib.FluidSimulationSaveState_is_fluid_brick_grid_enabled.restype = ctypes.c_int

    lib.FluidSimulationSaveState_is_load_state_initialized.argtypes = [ctypes.c_void_p]
    lib.FluidSimulationSaveState_is_load_state_initialized.restype = ctypes.c_int

__init__(lib)
