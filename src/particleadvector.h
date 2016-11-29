/*
Copyright (c) 2016 Ryan L. Guy

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgement in the product documentation would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef PARTICLEADVECTOR_H
#define PARTICLEADVECTOR_H

#ifdef __GNUC__
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#ifdef _MSC_VER 
    #pragma warning(push)
    #pragma warning(disable : 4996 4512 4510 4512 4610 )
#endif

#if defined(__APPLE__) || defined(__MACOSX)
    #include <OpenCL/cl.hpp>
#else
    #include <CL/cl.hpp>
#endif

#ifdef _MSC_VER 
    #pragma warning(pop)
#endif

#ifdef __GNUC__
    #pragma GCC diagnostic pop
#endif

#include <vector>
#include <fstream>
#include <algorithm>
#include <string>

#include "macvelocityfield.h"
#include "array3d.h"
#include "arrayview3d.h"
#include "grid3d.h"
#include "stopwatch.h"
#include "config.h"
#include "fluidsimassert.h"
#include "kernels/kernels.h"

class ParticleAdvector
{
public:
    ParticleAdvector();

    bool initialize();
    void setDevicePreference(std::string devtype);
    void setDevicePreferenceGPU();
    void setDevicePreferenceCPU();

    void printDeviceInfo();
    std::string getDeviceInfo();
    void printKernelInfo();
    std::string getKernelInfo();
    bool isUsingGPU();
    bool isUsingCPU();
    void disableOpenCL();
    void enableOpenCL();
    bool isOpenCLEnabled();
    int getKernelWorkLoadSize();
    void setKernelWorkLoadSize(int n);

    void advectParticlesRK4(std::vector<vmath::vec3> &particles,
                            MACVelocityField *vfield,
                            double dt,
                            std::vector<vmath::vec3> &output);

    void advectParticlesRK3(std::vector<vmath::vec3> &particles,
                            MACVelocityField *vfield,
                            double dt,
                            std::vector<vmath::vec3> &output);

    void advectParticlesRK2(std::vector<vmath::vec3> &particles,
                            MACVelocityField *vfield,
                            double dt,
                            std::vector<vmath::vec3> &output);

    void advectParticlesRK1(std::vector<vmath::vec3> &particles,
                            MACVelocityField *vfield,
                            double dt,
                            std::vector<vmath::vec3> &output);

    void tricubicInterpolate(std::vector<vmath::vec3> &particles,
                             MACVelocityField *vfield,
                             std::vector<vmath::vec3> &output);

    // method will overwrite particles with output data
    void tricubicInterpolate(std::vector<vmath::vec3> &particles,
                             MACVelocityField *vfield);

private:

    struct CLDeviceInfo {
        char cl_device_name[4096];
        char cl_device_vendor[4096];
        char cl_device_version[4096];
        char cl_driver_version[4096];
        char cl_device_opencl_c_version[4096];

        cl_device_type device_type;
        cl_uint cl_device_max_clock_frequency;
        cl_ulong cl_device_global_mem_size;
        cl_ulong cl_device_local_mem_size;
        cl_ulong cl_device_max_mem_alloc_size;
        size_t cl_device_max_work_group_size;
        GridIndex cl_device_max_work_item_sizes;
    };

    struct CLKernelInfo {
        char cl_kernel_function_name[4096];
        char cl_kernel_attributes[4096];

        cl_ulong cl_kernel_num_args;
        size_t cl_kernel_work_group_size;
        cl_ulong cl_kernel_local_mem_size;
        cl_ulong cl_kernel_private_mem_size;
        size_t cl_kernel_preferred_work_group_size_multiple;
    };

    struct ParticleChunk {
        std::vector<vmath::vec3> particles;
        std::vector<int> references;
    };

    struct DataChunkParameters {
        std::vector<vmath::vec3>::iterator particlesBegin;
        std::vector<vmath::vec3>::iterator particlesEnd;
        std::vector<int>::iterator referencesBegin;
        std::vector<int>::iterator referencesEnd;

        ArrayView3d<float> ufieldview;
        ArrayView3d<float> vfieldview;
        ArrayView3d<float> wfieldview;

        GridIndex chunkOffset;
        GridIndex indexOffset;
        vmath::vec3 positionOffset;
    };

    struct DataBuffer {
        std::vector<vmath::vec3> positionDataH;
        std::vector<float> vfieldDataH;
        std::vector<GridIndex> offsetDataH;

        cl::Buffer positionDataCL;
        cl::Buffer vfieldDataCL;
        cl::Buffer offsetDataCL;
    };

    void _checkError(cl_int err, const char * name);
    cl::Context _getCLContext(cl_int *err);
    cl::Device _getCLDevice(cl::Context &context, cl_int *err);
    CLDeviceInfo _initializeDeviceInfo(cl::Device &device);
    cl_int _initializeCLKernel();
    CLKernelInfo _initializeKernelInfo(cl::Kernel &kernel);
    std::string _getProgramString(std::string filename);
    cl_int _initializeCLCommandQueue();

    void _getParticleChunkGrid(double cwidth, double cheight, double cdepth,
                               std::vector<vmath::vec3> &particles,
                               Array3d<ParticleChunk> &grid);
    void _getDataChunkParameters(MACVelocityField *vfield,
                                 Array3d<ParticleChunk> &particleChunkGrid,
                                 std::vector<DataChunkParameters> &chunkParameters);
    void _getDataChunkParametersForChunkIndex(GridIndex index,
                                              MACVelocityField *vfield,
                                              ParticleChunk *particleChunk,
                                              std::vector<DataChunkParameters> &chunkParameters);
    
    int _getWorkGroupSize(CLDeviceInfo &info);
    int _getChunkPositionDataSize();
    int _getChunkVelocityDataSize();
    int _getChunkOffsetDataSize();
    int _getChunkTotalDataSize();
    int _getMaxChunksPerComputation();

    void _tricubicInterpolateChunks(std::vector<DataChunkParameters> &chunks,
                                    std::vector<vmath::vec3> &output);
    void _initializeDataBuffer(std::vector<DataChunkParameters> &chunks,
                               DataBuffer &buffer);
    void _getHostPositionDataBuffer(std::vector<DataChunkParameters> &chunks,
                                    std::vector<vmath::vec3> &buffer);
    void _getHostVelocityDataBuffer(std::vector<DataChunkParameters> &chunks,
                                    std::vector<float> &buffer);
    void _appendChunkVelocityDataToBuffer(DataChunkParameters &chunk, 
                                          std::vector<float> &buffer);
    void _getHostChunkOffsetDataBuffer(std::vector<DataChunkParameters> &chunks,
                                       std::vector<GridIndex> &buffer);
    void _setCLKernelArgs(DataBuffer &buffer, double dx);
    void _setOutputData(std::vector<DataChunkParameters> &chunks,
                        DataBuffer &buffer,
                        std::vector<vmath::vec3> &output);
    
    vmath::vec3 _RK4(vmath::vec3 p0, double dt, MACVelocityField *vfield);
    vmath::vec3 _RK3(vmath::vec3 p0, double dt, MACVelocityField *vfield);
    vmath::vec3 _RK2(vmath::vec3 p0, double dt, MACVelocityField *vfield);
    vmath::vec3 _RK1(vmath::vec3 p0, double dt, MACVelocityField *vfield);
    void _advectParticlesRK4NoCL(std::vector<vmath::vec3> &particles,
                                 MACVelocityField *vfield, 
                                 double dt,
                                 std::vector<vmath::vec3> &output);
    void _advectParticlesRK3NoCL(std::vector<vmath::vec3> &particles,
                                 MACVelocityField *vfield, 
                                 double dt,
                                 std::vector<vmath::vec3> &output);
    void _advectParticlesRK2NoCL(std::vector<vmath::vec3> &particles,
                                 MACVelocityField *vfield, 
                                 double dt,
                                 std::vector<vmath::vec3> &output);
    void _advectParticlesRK1NoCL(std::vector<vmath::vec3> &particles,
                                 MACVelocityField *vfield, 
                                 double dt,
                                 std::vector<vmath::vec3> &output);
    void _tricubicInterpolateNoCL(std::vector<vmath::vec3> &particles,
                                 MACVelocityField *vfield, 
                                 std::vector<vmath::vec3> &output);
    void _validateOutput(std::vector<vmath::vec3> &output);


    bool _isInitialized = false;

    cl_device_type _devicePreference1 = CL_DEVICE_TYPE_GPU;
    cl_device_type _devicePreference2 = CL_DEVICE_TYPE_CPU;

    CLDeviceInfo _deviceInfo;
    CLKernelInfo _kernelInfo;
    cl::Context _CLContext;
    cl::Device _CLDevice;
    cl::Kernel _CLKernel;
    cl::CommandQueue _CLQueue;

    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;
    double _dx = 0.0;

    int _maxItemsPerWorkGroup = 512;
    int _dataChunkWidth = 5;
    int _dataChunkHeight = 5;
    int _dataChunkDepth = 5;
    int _maxChunksPerComputation = 15000;
    int _kernelWorkLoadSize = 1000;
    bool _isOpenCLEnabled = true;
    
};

#endif
