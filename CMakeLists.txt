cmake_minimum_required(VERSION 2.8.9)

function(set_output_directories DIR)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${DIR} PARENT_SCOPE)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${DIR} PARENT_SCOPE)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${DIR} PARENT_SCOPE)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${DIR} PARENT_SCOPE)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${DIR} PARENT_SCOPE)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${DIR} PARENT_SCOPE)
endfunction(set_output_directories)

project(fluidsim)
set(CMAKE_BUILD_TYPE Release)

if (MSVC)
    add_definitions(-DCL_USE_DEPRECATED_OPENCL_1_1_APIS -DNOMINMAX -D_CRT_SECURE_NO_WARNINGS)
endif()

if (APPLE)
    set(CMAKE_MACOSX_RPATH ON)
endif()

if (MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O2 /W4 /EHsc /wd4244")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /LARGEADDRESSAWARE")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /LARGEADDRESSAWARE")
elseif(MINGW)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -Wall")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--enable-stdcall-fixup -Wl,--large-address-aware")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--enable-stdcall-fixup -Wl,--large-address-aware")
elseif(APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -Wall")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -Wall -fPIC")
endif()

find_package(OpenCL)

if (NOT OpenCL_FOUND)
    message(FATAL_ERROR "Error: OpenCL was not found on your system.\nPlease install an OpenCL SDK specific to your GPU vender (AMD, NVIDIA, Intel, etc.) and try again.")
endif()

if (APPLE)
    set(CL_CPP_BINDINGS_DIR "${OpenCL_INCLUDE_DIRS}/Versions/A/Headers")
else()
    set(CL_CPP_BINDINGS_DIR "${OpenCL_INCLUDE_DIRS}/CL")
endif()

if (NOT EXISTS "${CL_CPP_BINDINGS_DIR}/cl.hpp") 
    message(FATAL_ERROR "Error: The OpenCL C++ bindings were not found on your system.\nPlease download the 'cl.hpp' header from https://www.khronos.org/registry/cl/ and place the file in the '${CL_CPP_BINDINGS_DIR}' directory and try again.")
endif()

include_directories(src ${OpenCL_INCLUDE_DIRS})
file(GLOB SOURCES "src/*.cpp" "src/c_bindings/*.cpp" "src/kernels/*.cpp")

add_library(objects OBJECT ${SOURCES})

set(EXECUTABLE_DIR ${CMAKE_BINARY_DIR}/fluidsim)
set_output_directories(${EXECUTABLE_DIR})
add_executable(fluidsim $<TARGET_OBJECTS:objects>)
target_link_libraries(fluidsim ${OpenCL_LIBRARY})

set(PYTHON_MODULE_DIR ${CMAKE_BINARY_DIR}/fluidsim/pyfluid)
set(PYTHON_MODULE_LIB_DIR ${CMAKE_BINARY_DIR}/fluidsim/pyfluid/lib)
set_output_directories(${PYTHON_MODULE_LIB_DIR})
add_library(pyfluid SHARED $<TARGET_OBJECTS:objects>)
target_link_libraries(pyfluid ${OpenCL_LIBRARY})

file(MAKE_DIRECTORY "${EXECUTABLE_DIR}/output/bakefiles")
file(MAKE_DIRECTORY "${EXECUTABLE_DIR}/output/logs")
file(MAKE_DIRECTORY "${EXECUTABLE_DIR}/output/savestates")
file(MAKE_DIRECTORY "${EXECUTABLE_DIR}/output/temp")
file(COPY "${CMAKE_SOURCE_DIR}/src/pyfluid/" DESTINATION "${PYTHON_MODULE_DIR}")
file(COPY "${CMAKE_SOURCE_DIR}/src/examples/python/" DESTINATION "${PYTHON_MODULE_DIR}/examples")

set(CONFIG_EXECUTABLE_DIR  ${EXECUTABLE_DIR})
set(CONFIG_OUTPUT_DIR      ${EXECUTABLE_DIR}/output)
set(CONFIG_BAKEFILES_DIR   ${EXECUTABLE_DIR}/output/bakefiles)
set(CONFIG_LOGS_DIR        ${EXECUTABLE_DIR}/output/logs)
set(CONFIG_SAVESTATES_DIR  ${EXECUTABLE_DIR}/output/savestates)
set(CONFIG_TEMP_DIR        ${EXECUTABLE_DIR}/output/temp)
configure_file(
  "${PROJECT_SOURCE_DIR}/src/config.h.in"
  "${PROJECT_SOURCE_DIR}/src/config.h"
)

file(READ "src/kernels/tricubicinterpolate.cl" TRICUBIC_INTERPOLATE_KERNEL_STRING)
file(READ "src/kernels/scalarfield.cl" SCALAR_FIELD_KERNEL_STRING)
set(TRICUBIC_INTERPOLATE_KERNEL_STRING  ${TRICUBIC_INTERPOLATE_KERNEL_STRING})
set(SCALAR_FIELD_KERNEL_STRING  ${SCALAR_FIELD_KERNEL_STRING})
configure_file(
  "${PROJECT_SOURCE_DIR}/src/kernels/kernels.cpp.in"
  "${PROJECT_SOURCE_DIR}/src/kernels/kernels.cpp"
)
