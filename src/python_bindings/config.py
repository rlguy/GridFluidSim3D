import ctypes

from pyfluid import pyfluid as lib
import pybindings as pb

def get_executable_directory():
	return _get_directory(lib.Config_get_executable_directory)

def get_resources_directory():
	return _get_directory(lib.Config_get_resources_directory)

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