from abc import ABCMeta, abstractmethod
from ctypes import c_void_p, c_char_p, c_int, c_double, byref

from .pyfluid import pyfluid as lib
from . import pybindings as pb
from . import method_decorators as decorators
from .vector3 import Vector3, Vector3_t
from .aabb import AABB, AABB_t

class FluidSource:
    __metaclass__ = ABCMeta

    @abstractmethod
    def __init__():
        pass

    def __call__(self):
        return self._obj

    @property
    def position(self):
        libfunc = lib.FluidSource_get_position
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], Vector3_t)
        cvect = pb.execute_lib_func(libfunc, [self()])
        return Vector3.from_struct(cvect)

    @position.setter
    def position(self, pos):
        libfunc = lib.FluidSource_set_position
        pb.init_lib_func(libfunc, [c_void_p, Vector3_t, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), pos.to_struct()])

    @property
    def velocity(self):
        libfunc = lib.FluidSource_get_velocity
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], Vector3_t)
        cvect = pb.execute_lib_func(libfunc, [self()])
        return Vector3.from_struct(cvect)

    @velocity.setter
    def velocity(self, vel):
        libfunc = lib.FluidSource_set_velocity
        pb.init_lib_func(libfunc, [c_void_p, Vector3_t, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), vel.to_struct()])

    @property
    def direction(self):
        libfunc = lib.FluidSource_get_direction
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], Vector3_t)
        cvect = pb.execute_lib_func(libfunc, [self()])
        return Vector3.from_struct(cvect)

    @direction.setter
    def direction(self, direction):
        libfunc = lib.FluidSource_set_direction
        pb.init_lib_func(libfunc, [c_void_p, Vector3_t, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), direction.to_struct()])

    @property
    def is_inflow(self):
        libfunc = lib.FluidSource_is_inflow
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self()]))

    @is_inflow.setter
    def is_inflow(self, boolval):
        if boolval:
            libfunc = lib.FluidSource_set_as_inflow
        else:
            libfunc = lib.FluidSource_set_as_outflow
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self()])

    @property
    def is_outflow(self):
        libfunc = lib.FluidSource_is_outflow
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self()]))

    @is_outflow.setter
    def is_outflow(self, boolval):
        if boolval:
            libfunc = lib.FluidSource_set_as_outflow
        else:
            libfunc = lib.FluidSource_set_as_inflow
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self()])

    @property
    def is_active(self):
        libfunc = lib.FluidSource_is_active
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self()]))

    @is_active.setter
    def is_active(self, boolval):
        if boolval:
            libfunc = lib.FluidSource_activate
        else:
            libfunc = lib.FluidSource_deactivate
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], None)
        pb.execute_lib_func(libfunc, [self()])

    @property
    def id(self):
        libfunc = lib.FluidSource_get_id
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_int)
        return pb.execute_lib_func(libfunc, [self()])

    def get_AABB(self):
        libfunc = lib.FluidSource_get_AABB
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], AABB_t)
        cbbox = pb.execute_lib_func(libfunc, [self()])
        return AABB.from_struct(cbbox)

    @decorators.xyz_or_vector
    def contains_point(self, px, py, pz):
        cpos = Vector3_t(px, py, pz)
        libfunc = lib.FluidSource_contains_point
        pb.init_lib_func(libfunc, [c_void_p, Vector3_t, c_void_p], c_int)
        return bool(pb.execute_lib_func(libfunc, [self(), cpos]))

    

class CuboidFluidSource(FluidSource):

    def __init__(self, bbox = None, velocity = None):
        if bbox is None:
            bbox = AABB()
        if velocity is None:
            velocity = Vector3()

        libfunc = lib.CuboidFluidSource_new
        pb.init_lib_func(libfunc, [AABB_t, Vector3_t, c_void_p], c_void_p)
        self._obj = pb.execute_lib_func(libfunc, [bbox.to_struct(), 
                                                  velocity.to_struct()])

    def __del__(self):
        libfunc = lib.CuboidFluidSource_destroy
        pb.init_lib_func(libfunc, [c_void_p], None)
        try:
            libfunc(self._obj)
        except:
            pass

    @property
    def width(self):
        libfunc = lib.CuboidFluidSource_get_width
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return pb.execute_lib_func(libfunc, [self()])

    @width.setter
    def width(self, width):
        libfunc = lib.CuboidFluidSource_set_width
        pb.init_lib_func(libfunc, [c_void_p, c_double, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), width])

    @property
    def height(self):
        libfunc = lib.CuboidFluidSource_get_height
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return pb.execute_lib_func(libfunc, [self()])

    @height.setter
    def height(self, height):
        libfunc = lib.CuboidFluidSource_set_height
        pb.init_lib_func(libfunc, [c_void_p, c_double, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), height])

    @property
    def depth(self):
        libfunc = lib.CuboidFluidSource_get_depth
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return pb.execute_lib_func(libfunc, [self()])

    @depth.setter
    def depth(self, depth):
        libfunc = lib.CuboidFluidSource_set_depth
        pb.init_lib_func(libfunc, [c_void_p, c_double, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), depth])

    def set_AABB(self, bbox):
        libfunc = lib.CuboidFluidSource_set_AABB
        pb.init_lib_func(libfunc, [c_void_p, AABB_t, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), bbox.to_struct()])

    @property
    def AABB(self):
        return self.get_AABB()

    @AABB.setter
    def AABB(self, bbox):
        self.set_AABB(bbox)

    @property
    def center(self):
        libfunc = lib.CuboidFluidSource_get_center
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], Vector3_t)
        cvect = pb.execute_lib_func(libfunc, [self()])
        return Vector3.from_struct(cvect)

    @center.setter
    def center(self, center):
        libfunc = lib.CuboidFluidSource_set_center
        pb.init_lib_func(libfunc, [c_void_p, Vector3_t, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), center.to_struct()])

    def expand(self, val):
        libfunc = lib.CuboidFluidSource_expand
        pb.init_lib_func(libfunc, [c_void_p, c_double, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), val])


class SphericalFluidSource(FluidSource):
    
    def __init__(self, position = None, radius = None, velocity = None):
        if position is None:
            position = Vector3()
        if radius is None:
            radius = 0.0
        if velocity is None:
            velocity = Vector3()

        libfunc = lib.SphericalFluidSource_new
        pb.init_lib_func(libfunc, 
                         [Vector3_t, c_double, Vector3_t, c_void_p], c_void_p)
        self._obj = pb.execute_lib_func(libfunc, [position.to_struct(), 
                                                  radius,
                                                  velocity.to_struct()])

    def __del__(self):
        libfunc = lib.SphericalFluidSource_destroy
        pb.init_lib_func(libfunc, [c_void_p], None)
        try:
            libfunc(self._obj)
        except:
            pass

    @property
    def radius(self):
        libfunc = lib.SphericalFluidSource_get_radius
        pb.init_lib_func(libfunc, [c_void_p, c_void_p], c_double)
        return pb.execute_lib_func(libfunc, [self()])

    @radius.setter
    def radius(self, radius):
        libfunc = lib.SphericalFluidSource_set_radius
        pb.init_lib_func(libfunc, [c_void_p, c_double, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), radius])

    def expand(self, val):
        libfunc = lib.SphericalFluidSource_expand
        pb.init_lib_func(libfunc, [c_void_p, c_double, c_void_p], None)
        pb.execute_lib_func(libfunc, [self(), val])
