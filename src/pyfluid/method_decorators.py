import numbers

from .fluidsimulationsavestate import FluidSimulationSaveState
from .vector3 import Vector3
from .gridindex import GridIndex

def ijk_or_gridindex(func):
    def ijk_or_gridindex_wrapper(self, *args):
        try:
            i, j, k = args
        except:
            i, j, k = args[0]
        return func(self, i, j, k)
    return ijk_or_gridindex_wrapper

def ijk_or_gridindex_and_value(func):
    def ijk_or_gridindex_and_value_wrapper(self, *args):
        try:
            return func(self, *args)
        except:
            i, j, k = args[0]
            return func(self, i, j, k, args[1])
    return ijk_or_gridindex_and_value_wrapper

def xyz_or_vector(func):
    def xyz_or_vector_wrapper(self, *args):
        try:
            return func(self, *args)
        except:
            return func(self, *args[0])
    return xyz_or_vector_wrapper

def xyz_or_vector_and_radius(func):
    def xyz_or_vector_wrapper(self, *args):
        try:
            return func(self, *args)
        except:
            x, y, z = args[0]
            return func(self, x, y, z, args[1])
    return xyz_or_vector_wrapper

def check_gt_zero(func):
    def check_values(self, *args):
        for arg in args:
            if isinstance(arg, numbers.Real) and arg <= 0:
                raise ValueError("Value must be greater than zero")
        return func(self, *args)
    return check_values

def check_ge_zero(func):
    def check_values(self, *args):
        for arg in args:
            if isinstance(arg, numbers.Real) and arg < 0:
                raise ValueError("Value must be greater than or equal to zero")
        return func(self, *args)
    return check_values

def check_gt(value):
    def check_gt_decorator(func):
        def check_gt_wrapper(self, *args):
            for arg in args:
                if isinstance(arg, numbers.Real) and arg <= value:
                    raise ValueError("Value must be greater than " + str(value))
            return func(self, *args)
        return check_gt_wrapper
    return check_gt_decorator

def check_ge(value):
    def check_ge_decorator(func):
        def check_ge_wrapper(self, *args):
            for arg in args:
                if isinstance(arg, numbers.Real) and arg < value:
                    raise ValueError("Value must be greater than or equal to " + str(value))
            return func(self, *args)
        return check_ge_wrapper
    return check_ge_decorator

def check_type(argtype):
    def check_type_decorator(func):
        def check_type_wrapper(self, *args):
            for arg in args:
                if not isinstance(arg, argtype):
                    raise TypeError("Argument must be of type " + str(argtype))
            return func(self, *args)
        return check_type_wrapper
    return check_type_decorator