import ctypes
from ctypes import c_void_p, c_char_p, c_char, c_int, c_float, c_double, byref

from .pyfluid import pyfluid as lib
from .gridindex import GridIndex_t
from . import pybindings as pb

def get_triangle_mesh_cells(mesh, dx):
	mesh_struct = mesh.to_struct()

	libfunc = lib.utils_get_triangle_mesh_cells_storage_size
	pb.init_lib_func(libfunc, [c_void_p, c_double, c_void_p], c_int)
	storage_size = pb.execute_lib_func(libfunc, [byref(mesh_struct), dx])

	indices = (GridIndex_t * storage_size)()
	num_cells = c_int()

	libfunc = lib.utils_get_triangle_mesh_cells
	args = [c_void_p, c_double, c_void_p, c_int, c_void_p, c_void_p]
	pb.init_lib_func(libfunc, args, None)
	pb.execute_lib_func(
		libfunc, [byref(mesh_struct), dx, indices, storage_size, byref(num_cells)]
	)

	output = []
	for i in range(num_cells.value):
		output.append(indices[i])

	return output


def get_triangle_mesh_cells_subd2(mesh, dx):
	mesh_struct = mesh.to_struct()

	libfunc = lib.utils_get_triangle_mesh_cells_storage_size
	pb.init_lib_func(libfunc, [c_void_p, c_double, c_void_p], c_int)
	storage_size = pb.execute_lib_func(libfunc, [byref(mesh_struct), dx])

	indices = (GridIndex_t * storage_size)()
	cell_masks = (c_char * storage_size)()
	num_cells = c_int()

	libfunc = lib.utils_get_triangle_mesh_cells_subd2
	args = [c_void_p, c_double, c_void_p, c_char_p, c_int, c_void_p, c_void_p]
	pb.init_lib_func(libfunc, args, None)

	params = [byref(mesh_struct), dx, indices, cell_masks, storage_size, byref(num_cells)]
	pb.execute_lib_func(libfunc, params)

	output = []
	output_masks = []
	for i in range(num_cells.value):
		output.append(indices[i])
		output_masks.append(ord(cell_masks[i]))

	return output, output_masks