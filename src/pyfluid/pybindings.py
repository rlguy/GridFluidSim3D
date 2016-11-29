from .pyfluid import pyfluid as lib
from ctypes import c_char_p, c_int, byref

def check_success(success, errprefix):
    libfunc = lib.CBindings_get_error_message
    init_lib_func(libfunc, [], c_char_p)
    if not success:
        raise RuntimeError(errprefix + libfunc())

def init_lib_func(libfunc, argtypes, restype):
    if libfunc.argtypes is None:
        libfunc.argtypes = argtypes
        libfunc.restype = restype

def execute_lib_func(libfunc, params):
    args = []
    for idx, arg in enumerate(params):
        try:
            cval = libfunc.argtypes[idx](arg)
        except:
            cval = arg
        args.append(cval)
    success = c_int();
    args.append(byref(success))

    result = None
    if libfunc.restype:
        funcresult = libfunc(*args)
        try:
            return libfunc.restype(funcresult).value
        except:
            return funcresult
    else:
        libfunc(*args)

    check_success(success, libfunc.__name__ + " - ")
    return result