cmake_minimum_required(VERSION 2.8.9)

project(fluidsim)
set(CMAKE_BUILD_TYPE Release)

set(PROGRAM_DIR ${CMAKE_BINARY_DIR}/fluidsim)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${PROGRAM_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG  ${PROGRAM_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE  ${PROGRAM_DIR})

if (MSVC)
    add_definitions(-DCL_USE_DEPRECATED_OPENCL_1_1_APIS -DNOMINMAX -D_CRT_SECURE_NO_WARNINGS)
endif()

if (MSVC)
    set(CMAKE_CXX_FLAGS "/O2 /W4 /EHsc /wd4244")
    set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} /LARGEADDRESSAWARE")
else()
    set(CMAKE_CXX_FLAGS "-O3 -std=c++11 -Wall")
    set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -Wl,--enable-stdcall-fixup -Wl,--large-address-aware")
endif()

find_package(OpenCL)

if (NOT OpenCL_FOUND)
    message(FATAL_ERROR "\
Error: OpenCL was not found on your system.\n\
Please install an OpenCL SDK specific to your GPU vender (AMD, NVIDIA, Intel, etc.) and try again.")
endif()

if (${APPLE})
    set(CL_CPP_BINDINGS_DIR "${OpenCL_INCLUDE_DIRS}/OpenCL")
else()
    set(CL_CPP_BINDINGS_DIR "${OpenCL_INCLUDE_DIRS}/CL")
endif()

if (NOT EXISTS "${CL_CPP_BINDINGS_DIR}/cl.hpp") 
    message(FATAL_ERROR "\
Error: The OpenCL C++ bindings were not found on your system.\n\
Please download the 'cl.hpp' header from https://www.khronos.org/registry/cl/ and place the file in the '${CL_CPP_BINDINGS_DIR}' directory and try again.")
endif()

include_directories(${OpenCL_INCLUDE_DIRS})

include_directories(src)
file(GLOB SOURCES "src/*.cpp")

add_executable(fluidsim ${SOURCES})

target_include_directories(fluidsim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(fluidsim ${OpenCL_LIBRARY})

file(MAKE_DIRECTORY "${PROGRAM_DIR}/bakefiles")
file(MAKE_DIRECTORY "${PROGRAM_DIR}/logs")
file(MAKE_DIRECTORY "${PROGRAM_DIR}/savestates")

file(MAKE_DIRECTORY "${PROGRAM_DIR}/resources")
file(COPY "src/kernels" DESTINATION "${PROGRAM_DIR}/resources")
