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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#ifndef CLSCALARFIELD_H
#define CLSCALARFIELD_H

#if defined(__APPLE__) || defined(__MACOSX)
    #include <OpenCL/cl.hpp>
#else
    #include <CL/cl.hpp>
#endif

#include <vector>
#include <fstream>
#include <algorithm>
#include <string>

#include "macvelocityfield.h"
#include "implicitsurfacescalarfield.h"
#include "array3d.h"
#include "arrayview3d.h"
#include "grid3d.h"
#include "collision.h"
#include "stopwatch.h"

class CLScalarField
{
public:
    CLScalarField();

    bool initialize();
    void addPoints(std::vector<vmath::vec3> &points, 
                   double radius,
                   vmath::vec3 offset,
                   double dx,
                   Array3d<float> *field);
    void addPoints(std::vector<vmath::vec3> &points, 
                   double radius,
                   vmath::vec3 offset,
                   double dx,
                   ImplicitSurfaceScalarField &field);
    void addPoints(std::vector<vmath::vec3> &points, 
                   ImplicitSurfaceScalarField &field);

    void addPointValues(std::vector<vmath::vec3> &points, 
                        std::vector<float> &values,
                        double radius,
                        vmath::vec3 offset,
                        double dx,
                        Array3d<float> *field);
    void addPointValues(std::vector<vmath::vec3> &points, 
                        std::vector<float> &values,
                        double radius,
                        vmath::vec3 offset,
                        double dx,
                        Array3d<float> *scalarfield,
                        Array3d<float> *weightfield);
    void addPointValues(std::vector<vmath::vec3> &points, 
                        std::vector<float> &values,
                        double radius,
                        vmath::vec3 offset,
                        double dx,
                        ImplicitSurfaceScalarField &field);
    void addPointValues(std::vector<vmath::vec3> &points, 
                        std::vector<float> &values,
                        ImplicitSurfaceScalarField &field);

    void setMaxScalarFieldValueThreshold(float val);
    void setMaxScalarFieldValueThreshold();
    bool isMaxScalarFieldValueThresholdSet();
    double getMaxScalarFieldValueThreshold();

    void setDevicePreference(std::string devtype);
    void setDevicePreferenceGPU();
    void setDevicePreferenceCPU();

    void printDeviceInfo();
    bool isUsingGPU();
    bool isUsingCPU();

private:

    struct CLDeviceInfo {
        std::string cl_device_name;
        std::string cl_device_vendor;
        std::string cl_device_version;
        std::string cl_driver_version;
        std::string cl_device_opencl_c_version;

        cl_device_type device_type;
        cl_uint cl_device_max_clock_frequency;
        cl_ulong cl_device_global_mem_size;
        cl_ulong cl_device_local_mem_size;
        cl_ulong cl_device_max_mem_alloc_size;
        cl_uint cl_device_max_work_group_size;
        GridIndex cl_device_max_work_item_sizes;
    };

    struct DataBuffer {
        std::vector<float> pointDataH;
        std::vector<float> scalarFieldDataH;
        std::vector<GridIndex> offsetDataH;

        cl::Buffer positionDataCL;
        cl::Buffer scalarFieldDataCL;
        cl::Buffer offsetDataCL;
    };

    struct PointValue {
        PointValue() {}
        PointValue(vmath::vec3 p, float v) : position(p), value(v) {}
        vmath::vec3 position;
        float value;
    };

    struct WorkGroup {
        std::vector<PointValue> particles;

        ArrayView3d<float> fieldview;
        ArrayView3d<float> weightfieldview;

        GridIndex chunkOffset;
        GridIndex indexOffset;
        vmath::vec3 positionOffset;

        float minScalarFieldValue = 0.0;
    };

    struct WorkChunk {
        GridIndex workGroupIndex;

        std::vector<PointValue>::iterator particlesBegin;
        std::vector<PointValue>::iterator particlesEnd;
    };

    void _checkError(cl_int err, const char * name);
    cl::Context _getCLContext(cl_int *err);
    cl::Device _getCLDevice(cl::Context &context, cl_int *err);
    CLDeviceInfo _initializeDeviceInfo(cl::Device &device);
    cl_int _initializeChunkDimensions();
    cl_int _initializeCLKernels();
    std::string _getProgramString(std::string filename);
    cl_int _initializeCLCommandQueue();

    vmath::vec3 _getInternalOffset();
    void _initializePointValues(std::vector<vmath::vec3> &points,
                                std::vector<PointValue> &pvs);
    void _initializePointValues(std::vector<vmath::vec3> &points,
                                std::vector<float> &values,
                                std::vector<PointValue> &pvs);
    GridIndex _getWorkGroupGridDimensions();
    void _initializeWorkGroupGrid(std::vector<PointValue> &points,
                                  Array3d<float> *scalarfield,
                                  Array3d<WorkGroup> &grid);
    void _initializeWorkGroupGrid(std::vector<PointValue> &points,
                                  Array3d<float> *scalarfield,
                                  Array3d<float> *weightfield,
                                  Array3d<WorkGroup> &grid);
    void _initializeWorkGroupParameters(Array3d<WorkGroup> &grid,
                                        Array3d<float> *scalarfield);
    void _initializeWorkGroupParameters(Array3d<WorkGroup> &grid,
                                        Array3d<float> *scalarfield,
                                        Array3d<float> *weightfield);
    void _reserveWorkGroupGridParticleMemory(Array3d<WorkGroup> &grid,
                                             Array3d<int> &countGrid);
    void _getWorkGroupParticleCounts(std::vector<PointValue> &points,
                                     Array3d<int> &countGrid);
    void _insertParticlesIntoWorkGroupGrid(std::vector<PointValue> &points,
                                           Array3d<WorkGroup> &grid);
    void _initializeWorkChunks(Array3d<WorkGroup> &grid,
                               std::vector<WorkChunk> &chunks);
    void _getWorkChunksFromWorkGroup(WorkGroup *group, 
                                     std::vector<WorkChunk> &chunks);
    static bool _compareWorkChunkByNumParticles(const WorkChunk &c1, 
                                                const WorkChunk &c2);
    void _getNextWorkChunksToProcess(std::vector<WorkChunk> &queue,
                                     Array3d<WorkGroup> &grid,
                                     std::vector<WorkChunk> &chunks,
                                     int n);

    int _getChunkPointDataSize();
    int _getChunkPointValueDataSize();
    int _getChunkScalarFieldDataSize();
    int _getChunkScalarWeightFieldDataSize();
    int _getChunkOffsetDataSize();
    int _getMaxChunksPerPointComputation();
    int _getMaxChunksPerPointValueComputation();
    int _getMaxChunksPerWeightPointValueComputation();
    int _getMaxChunkLimit(int pointDataSize, int fieldDataSize, int offsetDataSize);

    void _computePointScalarField(std::vector<WorkChunk> &chunks,
                                  Array3d<WorkGroup> &workGroupGrid);
    void _computePointValueScalarField(std::vector<WorkChunk> &chunks,
                                       Array3d<WorkGroup> &workGroupGrid);
    void _computePointValueScalarWeightField(std::vector<WorkChunk> &chunks,
                                             Array3d<WorkGroup> &workGroupGrid);
    int _getMaxNumParticlesInChunk(std::vector<WorkChunk> &chunks);
    void _initializePointComputationDataBuffer(std::vector<WorkChunk> &chunks,
                                               Array3d<WorkGroup> &workGroupGrid,
                                               int numParticles,
                                               DataBuffer &buffer);
    void _initializePointValueComputationDataBuffer(std::vector<WorkChunk> &chunks,
                                                    Array3d<WorkGroup> &workGroupGrid,
                                                    int numParticles,
                                                    DataBuffer &buffer);
    void _initializeWeightPointValueComputationDataBuffer(std::vector<WorkChunk> &chunks,
                                                          Array3d<WorkGroup> &workGroupGrid,
                                                          int numParticles,
                                                          DataBuffer &buffer);
    void _getHostPointDataBuffer(std::vector<WorkChunk> &chunks,
                                 Array3d<WorkGroup> &grid,
                                 int numParticles,
                                 std::vector<float> &buffer);
    void _getHostPointValueDataBuffer(std::vector<WorkChunk> &chunks,
                                      Array3d<WorkGroup> &grid,
                                      int numParticles,
                                      std::vector<float> &buffer);
    void _getHostScalarFieldDataBuffer(std::vector<WorkChunk> &chunks,
                                       Array3d<WorkGroup> &grid,
                                       std::vector<float> &buffer);
    void _getHostScalarWeightFieldDataBuffer(std::vector<WorkChunk> &chunks,
                                             Array3d<WorkGroup> &grid,
                                             std::vector<float> &buffer);
    void _getHostChunkOffsetDataBuffer(std::vector<WorkChunk> &chunks,
                                       std::vector<GridIndex> &buffer);
    void _initializeCLDataBuffers(DataBuffer &buffer);
    void _setPointComputationCLKernelArgs(DataBuffer &buffer, int numParticles, double dx);
    void _setPointValueComputationCLKernelArgs(DataBuffer &buffer, int numParticles, double dx);
    void _setWeightPointValueComputationCLKernelArgs(DataBuffer &buffer, int numParticles, double dx);
    void _setKernelArgs(cl::Kernel &kernel, 
                        DataBuffer &buffer, 
                        int localDataBytes,
                        int numParticles, 
                        double radius, 
                        double dx);
    void _launchKernel(cl::Kernel &kernel, int numWorkItems, int workGroupSize);
    void _readCLBuffer(cl::Buffer &sourceCL, std::vector<float> &destH, int dataSize);
    void _setPointComputationOutputFieldData(std::vector<float> &buffer, 
                                             std::vector<WorkChunk> &chunks,
                                             Array3d<WorkGroup> &workGroupGrid);
    void _setPointValueComputationOutputFieldData(std::vector<float> &buffer, 
                                                  std::vector<WorkChunk> &chunks,
                                                  Array3d<WorkGroup> &workGroupGrid);
    void _setWeightPointValueComputationOutputFieldData(std::vector<float> &buffer, 
                                                        std::vector<WorkChunk> &chunks,
                                                        Array3d<WorkGroup> &workGroupGrid);
    void _updateWorkGroupMinimumValues(Array3d<WorkGroup> &grid);
    float _getWorkGroupMinimumValue(WorkGroup *g);

    bool _isInitialized = false;

    cl_device_type _devicePreference1 = CL_DEVICE_TYPE_GPU;
    cl_device_type _devicePreference2 = CL_DEVICE_TYPE_CPU;

    CLDeviceInfo _deviceInfo;
    cl::Context _CLContext;
    cl::Device _CLDevice;
    cl::Kernel _CLKernelPoints;
    cl::Kernel _CLKernelPointValues;
    cl::Kernel _CLKernelWeightPointValues;
    cl::CommandQueue _CLQueue;

    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;
    double _dx = 0.0;
    double _radius = 0.0;
    vmath::vec3 _offset;    // this offset value should not be used
                            // for internal calculations. Use 
                            // _getInternalOffset() for correct value.

    int _workGroupSize = 0;
    int _chunkWidth = 0;
    int _chunkHeight = 0;
    int _chunkDepth = 0;

    int _maxWorkGroupSize = 256;
    int _minWorkGroupSize = 32;
    int _maxParticlesPerChunk = 1000;
    int _maxChunksPerComputation = 5000;

    bool _isMaxScalarFieldValueThresholdSet = false;
    float _maxScalarFieldValueThreshold = 1.0;
    
};

#endif

#pragma GCC diagnostic pop