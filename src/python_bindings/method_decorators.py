import numbers

from fluidsimulationsavestate import FluidSimulationSaveState
from vector3 import Vector3
from gridindex import GridIndex

def ijk_or_gridindex(func):
    def ijk_or_gridindex_wrapper(self, *args):
        if len(args) == 1 and isinstance(args[0], GridIndex):
            return func(self, *args[0])
        return func(self, *args)
    return ijk_or_gridindex_wrapper

def xyz_or_vector(func):
    def xyz_or_vector_wrapper(self, *args):
        if len(args) == 1 and isinstance(args[0], Vector3):
            return func(self, *args[0])
        return func(self, *args)
    return xyz_or_vector_wrapper

def xyz_or_vector_and_radius(func):
    def xyz_or_vector_wrapper(self, *args):
        if len(args) == 2 and isinstance(args[0], Vector3):
            return func(self, radius=args[1], *args[0])
        elif len(args) == 4:
            return func(self, *args)
        return func(self, x, y, z, radius)
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