cmake_minimum_required(VERSION 2.8.9)

function(set_output_directories DIR)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${DIR} PARENT_SCOPE )
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${DIR} PARENT_SCOPE )
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${DIR} PARENT_SCOPE )
endfunction(set_output_directories)

project(fluidsim)
set(CMAKE_BUILD_TYPE Release)

if (MSVC)
    add_definitions(-DCL_USE_DEPRECATED_OPENCL_1_1_APIS -DNOMINMAX -D_CRT_SECURE_NO_WARNINGS)
endif()

if (MSVC)
    set(CMAKE_CXX_FLAGS "/O2 /W4 /EHsc /wd4244")
    set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} /LARGEADDRESSAWARE")
    set(CMAKE_SHARED_LINKER_FLAGS   "${CMAKE_EXE_LINKER_FLAGS} /LARGEADDRESSAWARE")
else()
    set(CMAKE_CXX_FLAGS "-O3 -std=c++11 -Wall")
    set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -Wl,--enable-stdcall-fixup -Wl,--large-address-aware")
    set(CMAKE_SHARED_LINKER_FLAGS   "${CMAKE_EXE_LINKER_FLAGS} -Wl,--enable-stdcall-fixup -Wl,--large-address-aware")
endif()

find_package(OpenCL)

if (NOT OpenCL_FOUND)
    message(FATAL_ERROR "Error: OpenCL was not found on your system.\nPlease install an OpenCL SDK specific to your GPU vender (AMD, NVIDIA, Intel, etc.) and try again.")
endif()

if (${APPLE})
    set(CL_CPP_BINDINGS_DIR "${OpenCL_INCLUDE_DIRS}/OpenCL")
else()
    set(CL_CPP_BINDINGS_DIR "${OpenCL_INCLUDE_DIRS}/CL")
endif()

if (NOT EXISTS "${CL_CPP_BINDINGS_DIR}/cl.hpp") 
    message(FATAL_ERROR "Error: The OpenCL C++ bindings were not found on your system.\nPlease download the 'cl.hpp' header from https://www.khronos.org/registry/cl/ and place the file in the '${CL_CPP_BINDINGS_DIR}' directory and try again.")
endif()

include_directories(src ${OpenCL_INCLUDE_DIRS})
file(GLOB SOURCES "src/*.cpp" "src/c_bindings/*.cpp")

add_library(objects OBJECT ${SOURCES})

set(EXECUTABLE_DIR ${CMAKE_BINARY_DIR}/fluidsim)
set_output_directories(${EXECUTABLE_DIR})
add_executable(fluidsim $<TARGET_OBJECTS:objects>)
target_link_libraries(fluidsim ${OpenCL_LIBRARY})

set(PYTHON_MODULE_DIR ${CMAKE_BINARY_DIR}/fluidsim/python_module)
set_output_directories(${PYTHON_MODULE_DIR})
add_library(fluidcore SHARED $<TARGET_OBJECTS:objects>)
target_link_libraries(fluidcore ${OpenCL_LIBRARY})

file(MAKE_DIRECTORY "${EXECUTABLE_DIR}/bakefiles")
file(MAKE_DIRECTORY "${EXECUTABLE_DIR}/logs")
file(MAKE_DIRECTORY "${EXECUTABLE_DIR}/savestates")
file(COPY "src/kernels" DESTINATION "${EXECUTABLE_DIR}/resources")
file(COPY "${CMAKE_SOURCE_DIR}/src/python_bindings/" DESTINATION "${PYTHON_MODULE_DIR}")
