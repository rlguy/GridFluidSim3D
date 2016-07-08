import ctypes
from fluidlib import lib
from fluidsimulationsavestate import FluidSimulationSaveState
from vector3 import Vector3
from gridindex import GridIndex
import numbers

def _check_simulation_initialized(func):
    def check_initialized_wrapper(*args, **kwargs):
        self = args[0]
        if isinstance(self, FluidSimulation) and not self.is_initialized():
            errmsg = "FluidSimulation must be initialized before calling this method"
            raise RuntimeError(errmsg)
        return func(*args, **kwargs)

    return check_initialized_wrapper

def _check_simulation_not_initialized(func):
    def check_not_initialized_wrapper(*args, **kwargs):
        self = args[0]
        if isinstance(self, FluidSimulation) and self.is_initialized():
            errmsg = "This method must be called before FluidSimulation is initialized"
            raise RuntimeError(errmsg)
        return func(*args, **kwargs)

    return check_not_initialized_wrapper

def _xyz_or_vector_and_radius(func):
    def xyz_or_vector_wrapper(*args, **kwargs):
        self = args[0]
        if not isinstance(self, FluidSimulation):
            return func(*args, **kwargs)

        x = y = z = radius = None
        if len(args) >= 3 and isinstance(args[1], Vector3):
            v = args[1]
            x, y, z = v.x, v.y, v.z
            radius = args[2]
        elif len(args) == 5:
            x, y, z, radius = args[1], args[2], args[3], args[4]

        return func(self, x, y, z, radius)

    return xyz_or_vector_wrapper

class FluidSimulation(object):

    def __init__(self, isize = None, jsize = None, ksize = None, dx = None):
        is_empty_constructor = all(x == None for x in (isize, jsize, ksize, dx))
        is_dimensions_constructor = (isinstance(isize, int) and
                                     isinstance(jsize, int) and
                                     isinstance(ksize, int) and
                                     isinstance(dx, numbers.Real))

        if is_empty_constructor:
            self._init_from_empty()
        elif is_dimensions_constructor:
            self._init_from_dimensions(isize, jsize, ksize, dx)
        else:
            errmsg = "FluidSimulation must be initialized with types:\n"
            errmsg += "isize:\t" + (str(int) + "\n" + 
                      "jsize:\t" + str(int) + "\n" + 
                      "ksize:\t" + str(int) + "\n" + 
                      "dx:\t" + str(float))
            raise TypeError(errmsg)

    def _init_from_empty(self):
        success = ctypes.c_int()
        self._obj = lib.FluidSimulation_new_from_empty(ctypes.byref(success))
        if not success:
            errmsg = "FluidSimulation_new_from_empty - "
            errmsg += lib.FluidSimulation_get_error_message()
            raise RuntimeError(errmsg)

    def _init_from_dimensions(self, isize, jsize, ksize, dx):
        if not all(x > 0 for x in (isize, jsize, ksize, dx)):
            raise ValueError("FluidSimulation dimensions must be greater than zero")

        success = ctypes.c_int()
        self._obj = lib.FluidSimulation_new_from_dimensions(
                        isize, jsize, ksize, dx, ctypes.byref(success))
        if not success:
            errmsg = "FluidSimulation_new_from_dimensions - "
            errmsg += lib.FluidSimulation_get_error_message()
            raise RuntimeError(errmsg)

    @classmethod
    def from_save_state(cls, savestate):
        if isinstance(savestate, FluidSimulationSaveState):
            self = cls()
            lib.FluidSimulation_destroy(self._obj)

            success = ctypes.c_int()
            self._obj = lib.FluidSimulation_new_from_save_state(
                            savestate(), ctypes.byref(success))
            if not success:
                errmsg = "FluidSimulation_new_from_save_state - "
                errmsg += lib.FluidSimulation_get_error_message()
                raise RuntimeError(errmsg)

            return self
        else:
            raise TypeError("savestate must be of type FluidSimulationSaveState")

    def initialize(self):
        if self.is_initialized():
            return
        success = ctypes.c_int()
        lib.FluidSimulation_initialize(self(), ctypes.byref(success))
        if not success:
            errmsg = "FluidSimulation_initialize - "
            errmsg += lib.FluidSimulation_get_error_message()
            raise RuntimeError(errmsg)

    def is_initialized(self):
        success = ctypes.c_int()
        result = lib.FluidSimulation_is_initialized(self(), ctypes.byref(success))
        if not success:
            errmsg = "FluidSimulation_is_initialized - "
            errmsg += lib.FluidSimulation_get_error_message()
            raise RuntimeError(errmsg)
        return bool(result)

    @_check_simulation_initialized
    def update(self, dt):
        deltatime = ctypes.c_double(dt)
        success = ctypes.c_int()
        lib.FluidSimulation_update(self(), deltatime, ctypes.byref(success))
        if not success:
            errmsg = "FluidSimulation_update - "
            errmsg += lib.FluidSimulation_get_error_message()
            raise RuntimeError(errmsg)

    @_check_simulation_initialized
    def save_state(self, filename):
        cfilename = ctypes.c_char_p(filename)
        success = ctypes.c_int()
        lib.FluidSimulation_save_state(self(), cfilename, ctypes.byref(success))
        if not success:
            errmsg = "FluidSimulation_save_state - "
            errmsg += lib.FluidSimulation_get_error_message()
            raise RuntimeError(errmsg)

    def get_current_frame(self):
        success = ctypes.c_int()
        frameno = lib.FluidSimulation_get_current_frame(self(), 
                                                        ctypes.byref(success))
        if not success:
            errmsg = "FluidSimulation_get_current_frame - "
            errmsg += lib.FluidSimulation_get_error_message()
            raise RuntimeError(errmsg)
        return int(frameno)

    def is_current_frame_finished(self):
        success = ctypes.c_int()
        result = lib.FluidSimulation_is_current_frame_finished(self(), 
                                                               ctypes.byref(success))
        if not success:
            errmsg = "FluidSimulation_is_current_frame_finished - "
            errmsg += lib.FluidSimulation_get_error_message()
            raise RuntimeError(errmsg)
        return bool(result)

    def get_cell_size(self):
        success = ctypes.c_int()
        dx = lib.FluidSimulation_get_cell_size(self(), ctypes.byref(success))
        if not success:
            errmsg = "FluidSimulation_get_cell_size - "
            errmsg += lib.FluidSimulation_get_error_message()
            raise RuntimeError(errmsg)
        return float(dx)

    def get_grid_dimensions(self):
        isize = ctypes.c_int()
        jsize = ctypes.c_int()
        ksize = ctypes.c_int()
        success = ctypes.c_int()
        lib.FluidSimulation_get_grid_dimensions(self(), ctypes.byref(isize),
                                                        ctypes.byref(jsize),
                                                        ctypes.byref(ksize),
                                                        ctypes.byref(success))
        if not success:
            errmsg = "FluidSimulation_get_grid_dimensions - "
            errmsg += lib.FluidSimulation_get_error_message()
            raise RuntimeError(errmsg)
        return GridIndex(isize.value, jsize.value, ksize.value)

    def get_grid_width(self):
        success = ctypes.c_int()
        isize = lib.FluidSimulation_get_grid_width(self(), ctypes.byref(success))
        if not success:
            errmsg = "FluidSimulation_get_grid_width - "
            errmsg += lib.FluidSimulation_get_error_message()
            raise RuntimeError(errmsg)
        return int(isize)

    def get_grid_height(self):
        success = ctypes.c_int()
        jsize = lib.FluidSimulation_get_grid_height(self(), ctypes.byref(success))
        if not success:
            errmsg = "FluidSimulation_get_grid_height - "
            errmsg += lib.FluidSimulation_get_error_message()
            raise RuntimeError(errmsg)
        return int(jsize)

    def get_grid_depth(self):
        success = ctypes.c_int()
        ksize = lib.FluidSimulation_get_grid_depth(self(), ctypes.byref(success))
        if not success:
            errmsg = "FluidSimulation_get_grid_depth - "
            errmsg += lib.FluidSimulation_get_error_message()
            raise RuntimeError(errmsg)
        return int(ksize)

    def get_simulation_dimensions(self):
        width = ctypes.c_double()
        height = ctypes.c_double()
        depth = ctypes.c_double()
        success = ctypes.c_int()
        lib.FluidSimulation_get_simulation_dimensions(self(), ctypes.byref(width),
                                                              ctypes.byref(height),
                                                              ctypes.byref(depth),
                                                              ctypes.byref(success))
        if not success:
            errmsg = "FluidSimulation_get_simulation_dimensions - "
            errmsg += lib.FluidSimulation_get_error_message()
            raise RuntimeError(errmsg)
        return Vector3(width.value, height.value, depth.value)

    def get_simulation_width(self):
        success = ctypes.c_int()
        width = lib.FluidSimulation_get_simulation_width(self(), ctypes.byref(success))
        if not success:
            errmsg = "FluidSimulation_get_simulation_width - "
            errmsg += lib.FluidSimulation_get_error_message()
            raise RuntimeError(errmsg)
        return float(width)

    def get_simulation_height(self):
        success = ctypes.c_int()
        height = lib.FluidSimulation_get_simulation_height(self(), ctypes.byref(success))
        if not success:
            errmsg = "FluidSimulation_get_simulation_height - "
            errmsg += lib.FluidSimulation_get_error_message()
            raise RuntimeError(errmsg)
        return float(height)

    def get_simulation_depth(self):
        success = ctypes.c_int()
        depth = lib.FluidSimulation_get_simulation_depth(self(), ctypes.byref(success))
        if not success:
            errmsg = "FluidSimulation_get_simulation_depth - "
            errmsg += lib.FluidSimulation_get_error_message()
            raise RuntimeError(errmsg)
        return float(depth)

    @_check_simulation_not_initialized
    @_xyz_or_vector_and_radius
    def add_implicit_fluid_point(self, x, y, z, radius):
        if self.is_initialized():
            errmsg = ("implicit fluid point must be added before before" +
                      "simulation is initialized")
            raise RuntimeError(errmsg)

        x = ctypes.c_double(x)
        y = ctypes.c_double(y)
        z = ctypes.c_double(z)
        radius = ctypes.c_double(radius)
        success = ctypes.c_int()
        lib.FluidSimulation_add_implicit_fluid_point(self(), 
                                                     x, y, z, radius, 
                                                     ctypes.byref(success))
        if not success:
            errmsg = "FluidSimulation_add_implicit_fluid_point - "
            errmsg += lib.FluidSimulation_get_error_message()
            raise RuntimeError(errmsg)

    def __del__(self):
        try:
            lib.FluidSimulation_destroy(self._obj)
        except:
            pass

    def __call__(self):
        return self._obj

def __init__(lib):
    lib.FluidSimulation_new_from_empty.argtypes = [ctypes.c_void_p]
    lib.FluidSimulation_new_from_empty.restype = ctypes.c_void_p

    lib.FluidSimulation_new_from_dimensions.argtypes = [ctypes.c_int,
                                                        ctypes.c_int,
                                                        ctypes.c_int,
                                                        ctypes.c_double,
                                                        ctypes.c_void_p]
    lib.FluidSimulation_new_from_dimensions.restype = ctypes.c_void_p

    lib.FluidSimulation_new_from_save_state.argtypes = [ctypes.c_void_p,
                                                        ctypes.c_void_p]
    lib.FluidSimulation_new_from_save_state.restype = ctypes.c_void_p

    lib.FluidSimulation_destroy.argtypes = [ctypes.c_void_p]
    lib.FluidSimulation_destroy.restype = None

    lib.FluidSimulation_initialize.argtypes = [ctypes.c_void_p, 
                                               ctypes.c_void_p]
    lib.FluidSimulation_initialize.restype = None

    lib.FluidSimulation_is_initialized.argtypes = [ctypes.c_void_p, 
                                               ctypes.c_void_p]
    lib.FluidSimulation_is_initialized.restype = ctypes.c_int

    lib.FluidSimulation_update.argtypes = [ctypes.c_void_p, 
                                           ctypes.c_double,
                                           ctypes.c_void_p]
    lib.FluidSimulation_update.restype = None

    lib.FluidSimulation_save_state.argtypes = [ctypes.c_void_p, 
                                               ctypes.c_char_p, 
                                               ctypes.c_void_p]
    lib.FluidSimulation_save_state.restype = None

    lib.FluidSimulation_get_current_frame.argtypes = [ctypes.c_void_p,
                                                      ctypes.c_void_p]
    lib.FluidSimulation_get_current_frame.restype = ctypes.c_int

    lib.FluidSimulation_is_current_frame_finished.argtypes = [ctypes.c_void_p,
                                                              ctypes.c_void_p]
    lib.FluidSimulation_is_current_frame_finished.restype = ctypes.c_int

    lib.FluidSimulation_get_cell_size.argtypes = [ctypes.c_void_p,
                                                  ctypes.c_void_p]
    lib.FluidSimulation_get_cell_size.restype = ctypes.c_double

    lib.FluidSimulation_get_grid_dimensions.argtypes = [ctypes.c_void_p,
                                                        ctypes.c_void_p,
                                                        ctypes.c_void_p,
                                                        ctypes.c_void_p,
                                                        ctypes.c_void_p]
    lib.FluidSimulation_get_grid_dimensions.restype = None

    lib.FluidSimulation_get_grid_width.argtypes = [ctypes.c_void_p,
                                                   ctypes.c_void_p]
    lib.FluidSimulation_get_grid_width.restype = ctypes.c_int

    lib.FluidSimulation_get_grid_height.argtypes = [ctypes.c_void_p,
                                                   ctypes.c_void_p]
    lib.FluidSimulation_get_grid_height.restype = ctypes.c_int

    lib.FluidSimulation_get_grid_depth.argtypes = [ctypes.c_void_p,
                                                   ctypes.c_void_p]
    lib.FluidSimulation_get_grid_depth.restype = ctypes.c_int

    lib.FluidSimulation_get_simulation_dimensions.argtypes = [ctypes.c_void_p,
                                                              ctypes.c_void_p,
                                                              ctypes.c_void_p,
                                                              ctypes.c_void_p,
                                                              ctypes.c_void_p]
    lib.FluidSimulation_get_simulation_dimensions.restype = None

    lib.FluidSimulation_get_simulation_width.argtypes = [ctypes.c_void_p,
                                                         ctypes.c_void_p]
    lib.FluidSimulation_get_simulation_width.restype = ctypes.c_double

    lib.FluidSimulation_get_simulation_height.argtypes = [ctypes.c_void_p,
                                                          ctypes.c_void_p]
    lib.FluidSimulation_get_simulation_height.restype = ctypes.c_double

    lib.FluidSimulation_get_simulation_depth.argtypes = [ctypes.c_void_p,
                                                         ctypes.c_void_p]
    lib.FluidSimulation_get_simulation_depth.restype = ctypes.c_double

    lib.FluidSimulation_add_implicit_fluid_point.argtypes = [ctypes.c_void_p, 
                                                             ctypes.c_double,
                                                             ctypes.c_double,
                                                             ctypes.c_double,
                                                             ctypes.c_double,
                                                             ctypes.c_void_p]
    lib.FluidSimulation_add_implicit_fluid_point.restype = None

    lib.FluidSimulation_get_error_message.argtypes = []
    lib.FluidSimulation_get_error_message.restype = ctypes.c_char_p

    

__init__(lib)