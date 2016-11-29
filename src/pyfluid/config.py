import ctypes
import os

from .pyfluid import pyfluid as lib
from . import pybindings as pb

def get_executable_directory():
    return _get_directory(lib.Config_get_executable_directory)

def get_output_directory():
    return _get_directory(lib.Config_get_output_directory)

def get_bakefiles_directory():
    return _get_directory(lib.Config_get_bakefiles_directory)

def get_logs_directory():
    return _get_directory(lib.Config_get_logs_directory)

def get_savestates_directory():
    return _get_directory(lib.Config_get_savestates_directory)

def get_temp_directory():
    return _get_directory(lib.Config_get_temp_directory)

def _get_directory(libfunc):
    directory = (ctypes.c_char * 4096)()
    pb.init_lib_func(libfunc, [ctypes.c_char_p], None)
    libfunc(directory)
    return directory.value

def set_output_directory(directory):
    _set_directory(lib.Config_set_output_directory, directory)

def set_bakefiles_directory(directory):
    _set_directory(lib.Config_set_bakefiles_directory, directory)

def set_logs_directory(directory):
    _set_directory(lib.Config_set_logs_directory, directory)

def set_savestates_directory(directory):
    _set_directory(lib.Config_set_savestates_directory, directory)

def set_temp_directory(directory):
    _set_directory(lib.Config_set_temp_directory, directory)

def _set_directory(libfunc, directory):
    directory = os.path.abspath(directory)
    if not os.path.exists(directory):
        os.makedirs(directory)

    cdir = ctypes.c_char_p(directory.encode('utf-8'))
    pb.init_lib_func(libfunc, [ctypes.c_char_p], None)
    libfunc(cdir)