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

#ifdef __GNUC__
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#include "clscalarfield.h"

CLScalarField::CLScalarField() {
}

bool CLScalarField::initialize() {
    cl_int err;
    cl::Context context = _getCLContext(&err);
    if (err != CL_SUCCESS) {
        return false;
    }

    cl::Device device = _getCLDevice(context, &err);
    if (err != CL_SUCCESS) {
        return false;
    }
    _CLContext = context;
    _CLDevice = device;
    _deviceInfo = _initializeDeviceInfo(device);

    err = _initializeChunkDimensions();
    if (err != CL_SUCCESS) {
        return false;
    }
    
    err = _initializeCLKernels();
    if (err != CL_SUCCESS) {
        return false;
    }
    _kernelPointsInfo = _initializeKernelInfo(_CLKernelPoints);
    _kernelPointValuesInfo = _initializeKernelInfo(_CLKernelPointValues);
    _kernelWeightPointValuesInfo = _initializeKernelInfo(_CLKernelWeightPointValues);

    err = _initializeCLCommandQueue();
    if (err != CL_SUCCESS) {
        return false;
    }

    _isInitialized = true;
    return true;
}

void CLScalarField::addPoints(std::vector<vmath::vec3> &points, 
                              double radius,
                              vmath::vec3 offset,
                              double dx,
                              Array3d<float> *field) {
    FLUIDSIM_ASSERT(_isInitialized);

    if (!_isOpenCLEnabled) {
        _addPointsNoCL(points, radius, offset, dx, field);
    }

    _isize = field->width;
    _jsize = field->height;
    _ksize = field->depth;
    _dx = dx;
    _radius = radius;
    _offset = offset;

    bool isOutOfRangeValueSet = field->isOutOfRangeValueSet();
    if (!isOutOfRangeValueSet) {
        field->setOutOfRangeValue(0.0);
    }

    std::vector<PointValue> pointValues;
    _initializePointValues(points, pointValues);

    GridIndex workGroupDims = _getWorkGroupGridDimensions();
    Array3d<WorkGroup> workGroupGrid(workGroupDims.i, workGroupDims.j, workGroupDims.k);

    _initializeWorkGroupGrid(pointValues, field, workGroupGrid);

    std::vector<WorkChunk> workChunkQueue;
    _initializeWorkChunks(workGroupGrid, workChunkQueue);

    int maxChunks = _getMaxChunksPerPointValueComputation();
    
    std::vector<WorkChunk> chunks;
    while (!workChunkQueue.empty()) {
        _updateWorkGroupMinimumValues(workGroupGrid);

        chunks.clear();

        _getNextWorkChunksToProcess(workChunkQueue, 
                                    workGroupGrid, 
                                    chunks, 
                                    maxChunks);

        _computePointScalarField(chunks, workGroupGrid);
    }

    if (!isOutOfRangeValueSet) {
        field->setOutOfRangeValue();
    }
    
}

void CLScalarField::addPoints(std::vector<vmath::vec3> &points, 
                              double radius,
                              vmath::vec3 offset,
                              double dx,
                              ScalarField &isfield) {

    Array3d<float> *field = isfield.getPointerToScalarField();
    addPoints(points, radius, offset, dx, field);
}

void CLScalarField::addPoints(std::vector<vmath::vec3> &points,
                              ScalarField &isfield) {

    double r = isfield.getPointRadius();
    vmath::vec3 offset = isfield.getOffset();
    double dx = isfield.getCellSize();
    Array3d<float> *field = isfield.getPointerToScalarField();
    addPoints(points, r, offset, dx, field);
}

void CLScalarField::addPointValues(std::vector<vmath::vec3> &points, 
                                   std::vector<float> &values,
                                   double radius,
                                   vmath::vec3 offset,
                                   double dx,
                                   Array3d<float> *field) {
    
    FLUIDSIM_ASSERT(_isInitialized);
    FLUIDSIM_ASSERT(points.size() == values.size());

    if (!_isOpenCLEnabled) {
        _addPointValuesNoCL(points, values, radius, offset, dx, field);
        return;
    }

    _isize = field->width;
    _jsize = field->height;
    _ksize = field->depth;
    _dx = dx;
    _radius = radius;
    _offset = offset;

    bool isOutOfRangeValueSet = field->isOutOfRangeValueSet();
    if (!isOutOfRangeValueSet) {
        field->setOutOfRangeValue(0.0);
    }

    std::vector<PointValue> pointValues;
    _initializePointValues(points, values, pointValues);

    GridIndex workGroupDims = _getWorkGroupGridDimensions();
    Array3d<WorkGroup> workGroupGrid(workGroupDims.i, workGroupDims.j, workGroupDims.k);
    _initializeWorkGroupGrid(pointValues, field, workGroupGrid);
    
    std::vector<WorkChunk> workChunkQueue;
    _initializeWorkChunks(workGroupGrid, workChunkQueue);

    int maxChunks = _getMaxChunksPerPointValueComputation();

    std::vector<WorkChunk> chunks;
    while (!workChunkQueue.empty()) {
        _updateWorkGroupMinimumValues(workGroupGrid);

        chunks.clear();
        _getNextWorkChunksToProcess(workChunkQueue, 
                                    workGroupGrid, 
                                    chunks, 
                                    maxChunks);
        _computePointValueScalarField(chunks, workGroupGrid);
    }

    if (!isOutOfRangeValueSet) {
        field->setOutOfRangeValue();
    }
}

void CLScalarField::addPointValues(std::vector<vmath::vec3> &points, 
                                   std::vector<float> &values,
                                   double radius,
                                   vmath::vec3 offset,
                                   double dx,
                                   Array3d<float> *scalarfield,
                                   Array3d<float> *weightfield) {
    FLUIDSIM_ASSERT(_isInitialized);
    FLUIDSIM_ASSERT(points.size() == values.size());
    FLUIDSIM_ASSERT(scalarfield->width == weightfield->width &&
                    scalarfield->height == weightfield->height &&
                    scalarfield->depth == weightfield->depth);

    if (!_isOpenCLEnabled) {
        _addPointValuesNoCL(points, values, radius, offset, dx, 
                            scalarfield, weightfield);
        return;
    }

    _isize = scalarfield->width;
    _jsize = scalarfield->height;
    _ksize = scalarfield->depth;
    _dx = dx;
    _radius = radius;
    _offset = offset;

    bool isScalarFieldOutOfRangeValueSet = scalarfield->isOutOfRangeValueSet();
    bool isWeightFieldOutOfRangeValueSet = weightfield->isOutOfRangeValueSet();
    if (!isScalarFieldOutOfRangeValueSet) {
        scalarfield->setOutOfRangeValue(0.0);
    }
    if (!isWeightFieldOutOfRangeValueSet) {
        weightfield->setOutOfRangeValue(0.0);
    }

    std::vector<PointValue> pointValues;
    _initializePointValues(points, values, pointValues);

    GridIndex workGroupDims = _getWorkGroupGridDimensions();
    Array3d<WorkGroup> workGroupGrid(workGroupDims.i, workGroupDims.j, workGroupDims.k);

    _initializeWorkGroupGrid(pointValues, scalarfield, weightfield, workGroupGrid);
    
    std::vector<WorkChunk> workChunkQueue;
    _initializeWorkChunks(workGroupGrid, workChunkQueue);

    int maxChunks = _getMaxChunksPerWeightPointValueComputation();

    std::vector<WorkChunk> chunks;
    while (!workChunkQueue.empty()) {
        _updateWorkGroupMinimumValues(workGroupGrid);

        chunks.clear();
        _getNextWorkChunksToProcess(workChunkQueue, 
                                    workGroupGrid, 
                                    chunks, 
                                    maxChunks);
        _computePointValueScalarWeightField(chunks, workGroupGrid);
    }

    if (!isScalarFieldOutOfRangeValueSet) {
        scalarfield->setOutOfRangeValue();
    }
    if (!isWeightFieldOutOfRangeValueSet) {
        weightfield->setOutOfRangeValue();
    }
    
}

void CLScalarField::addPointValues(std::vector<vmath::vec3> &points, 
                                   std::vector<float> &values,
                                   double radius,
                                   vmath::vec3 offset,
                                   double dx,
                                   ScalarField &isfield) {

    Array3d<float> *field = isfield.getPointerToScalarField();

    if (isfield.isWeightFieldEnabled()) {
        Array3d<float> *weightfield = isfield.getPointerToWeightField();
        addPointValues(points, values, radius, offset, dx, field, weightfield);
    } else {
        addPointValues(points, values, radius, offset, dx, field);
    }
}

void CLScalarField::addPointValues(std::vector<vmath::vec3> &points, 
                                   std::vector<float> &values,
                                   ScalarField &isfield) {

    double r = isfield.getPointRadius();
    vmath::vec3 offset = isfield.getOffset();
    double dx = isfield.getCellSize();
    Array3d<float> *field = isfield.getPointerToScalarField();

    if (isfield.isWeightFieldEnabled()) {
        Array3d<float> *weightfield = isfield.getPointerToWeightField();
        addPointValues(points, values, r, offset, dx, field, weightfield);
    } else {
        addPointValues(points, values, r, offset, dx, field);
    }
}

void CLScalarField::setMaxScalarFieldValueThreshold(float val) {
    _isMaxScalarFieldValueThresholdSet = true;
    _maxScalarFieldValueThreshold = val;
}

void CLScalarField::setMaxScalarFieldValueThreshold() {
    _isMaxScalarFieldValueThresholdSet = false;
}

bool CLScalarField::isMaxScalarFieldValueThresholdSet() {
    return _isMaxScalarFieldValueThresholdSet;
}

double CLScalarField::getMaxScalarFieldValueThreshold() {
    return _maxScalarFieldValueThreshold;
}

void CLScalarField::setDevicePreference(std::string devtype) {
    std::transform(devtype.begin(), devtype.end(), devtype.begin(), ::tolower);

    if (devtype == "gpu") {
        setDevicePreferenceGPU();
    } else if (devtype == "cpu") {
        setDevicePreferenceCPU();
    }
}

void CLScalarField::setDevicePreferenceGPU() {
    _devicePreference1 = CL_DEVICE_TYPE_GPU;
    _devicePreference2 = CL_DEVICE_TYPE_CPU;
}

void CLScalarField::setDevicePreferenceCPU() {
    _devicePreference1 = CL_DEVICE_TYPE_CPU;
    _devicePreference2 = CL_DEVICE_TYPE_GPU;
}

void CLScalarField::printDeviceInfo() {
    if (!_isInitialized) {
        return;
    }

    std::cout << getDeviceInfo();
}

std::string CLScalarField::getDeviceInfo() {
    std::ostringstream ss;
    if (!_isInitialized) {
        return ss.str();
    }

    ss << "CL_DEVICE_NAME:                " << 
          _deviceInfo.cl_device_name << std::endl;
    ss << "CL_DEVICE_VENDOR:              " << 
          _deviceInfo.cl_device_vendor << std::endl;
    ss << "CL_DEVICE_VERSION:             " << 
          _deviceInfo.cl_device_version << std::endl;
    ss << "CL_DRIVER_VERSION:             " << 
           _deviceInfo.cl_driver_version << std::endl;
    ss << "CL_DEVICE_OPENCL_C_VERSION:    " << 
          _deviceInfo.cl_device_opencl_c_version << std::endl;

    std::string type;
    switch (_deviceInfo.device_type) {
        case CL_DEVICE_TYPE_CPU:
            type = "CPU";
            break;
        case CL_DEVICE_TYPE_GPU:
            type = "GPU";
            break;
        case CL_DEVICE_TYPE_ACCELERATOR:
            type = "ACCELERATOR";
            break;
        case CL_DEVICE_TYPE_DEFAULT:
            type = "DEFAULT";
            break;
        default:
            break;
    }
    ss << "CL_DEVICE_TYPE:                " << 
          type << std::endl;
    ss << "CL_DEVICE_MAX_CLOCK_FREQUENCY: " << 
          _deviceInfo.cl_device_max_clock_frequency << "MHz" << std::endl;
    ss << "CL_DEVICE_GLOBAL_MEM_SIZE:     " << 
          _deviceInfo.cl_device_global_mem_size << std::endl;
    ss << "CL_DEVICE_LOCAL_MEM_SIZE:      " << 
          _deviceInfo.cl_device_local_mem_size << std::endl;
    ss << "CL_DEVICE_MAX_MEM_ALLOC_SIZE:  " << 
          _deviceInfo.cl_device_max_mem_alloc_size << std::endl;
    ss << "CL_DEVICE_MAX_WORK_GROUP_SIZE: " << 
          _deviceInfo.cl_device_max_work_group_size << std::endl;

    GridIndex g = _deviceInfo.cl_device_max_work_item_sizes;
    ss << "CL_DEVICE_MAX_WORK_ITEM_SIZES: " << g.i << " x " << 
                                               g.j << " x " << 
                                               g.k << std::endl;
    return ss.str();
}

void CLScalarField::printKernelInfo() {
    if (!_isInitialized) {
        return;
    }

    std::cout << getKernelInfo();
}

std::string CLScalarField::getKernelInfo() {
    std::string k1 = _getKernelInfo(_kernelPointsInfo);
    std::string k2 = _getKernelInfo(_kernelPointValuesInfo);
    std::string k3 = _getKernelInfo(_kernelWeightPointValuesInfo);

    return k1 + "\n" + k2 + "\n" + k3;
}

std::string CLScalarField::_getKernelInfo(CLKernelInfo &info) {
    std::ostringstream ss;
    if (!_isInitialized) {
        return ss.str();
    }

    ss << "CL_KERNEL_FUNCTION_NAME:                      " << 
                 info.cl_kernel_function_name << std::endl;
    ss << "CL_KERNEL_ATTRIBUTES:                        " << 
                 info.cl_kernel_attributes << std::endl;

    ss << "CL_KERNEL_NUM_ARGS:                           " << 
                 info.cl_kernel_num_args << std::endl;
    ss << "CL_KERNEL_WORK_GROUP_SIZE:                    " << 
                 info.cl_kernel_work_group_size << std::endl;
    ss << "CL_KERNEL_LOCAL_MEM_SIZE:                     " << 
                 info.cl_kernel_local_mem_size << std::endl;
    ss << "CL_KERNEL_PRIVATE_MEM_SIZE:                   " << 
                 info.cl_kernel_private_mem_size << std::endl;
    ss << "CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE: " << 
                 info.cl_kernel_preferred_work_group_size_multiple << std::endl;

    return ss.str();
}

cl_int CLScalarField::_initializeChunkDimensions() {
    unsigned int groupsize = (unsigned int)_deviceInfo.cl_device_max_work_group_size;
    groupsize = fmin(groupsize, _maxWorkGroupSize);

    if (groupsize < (unsigned int)_minWorkGroupSize) {
        return (cl_int)-1;
    }

    std::vector<unsigned int> validsizes;
    int size = _minWorkGroupSize;
    while (size <= _maxWorkGroupSize) {
        validsizes.push_back(size);
        size *= 2;
    }

    bool isValidSize = false;
    for (unsigned int i = 0; i < validsizes.size(); i++) {
        if (groupsize == validsizes[i]) {
            isValidSize = true;
            break;
        }
    }

    if (!isValidSize) {
        for (int i = (int)validsizes.size() - 1; i >= 0; i--) {
            if (groupsize > validsizes[i]) {
                groupsize = validsizes[i];
                break;
            }
        }
    }

    int chunksize = floor(cbrt(groupsize));

    _workGroupSize = groupsize;
    _chunkWidth = chunksize;
    _chunkHeight = chunksize;
    _chunkDepth = chunksize;

    return CL_SUCCESS;
}

bool CLScalarField::isUsingGPU() {
    if (!_isInitialized) {
        return false;
    }
    return _deviceInfo.device_type == CL_DEVICE_TYPE_GPU;
}

bool CLScalarField::isUsingCPU() {
    if (!_isInitialized) {
        return false;
    }
    return _deviceInfo.device_type == CL_DEVICE_TYPE_CPU;
}

void CLScalarField::disableOpenCL() {
    _isOpenCLEnabled = false;
}

void CLScalarField::enableOpenCL() {
    _isOpenCLEnabled = true;
}

bool CLScalarField::isOpenCLEnabled() {
    return _isOpenCLEnabled;
}

int CLScalarField::getKernelWorkLoadSize() {
    return _kernelWorkLoadSize;
}

void CLScalarField::setKernelWorkLoadSize(int n) {
    _kernelWorkLoadSize = n;
}

void CLScalarField::_checkError(cl_int err, const char * name) {
    if (err != CL_SUCCESS) {
        std::cerr << "ERROR: " << name  << " (" << err << ")" << std::endl;
        FLUIDSIM_ASSERT(err == CL_SUCCESS);
    }
}

cl::Context CLScalarField::_getCLContext(cl_int *err) {
    cl::Context context;

    std::vector< cl::Platform > platforms;
    cl::Platform::get(&platforms);

    if (platforms.size() == 0) {
        *err = -1;
        return context;
    }

    // Try to find a platform with first device preference
    for (unsigned int i = 0; i < platforms.size(); i++) {
        cl_context_properties p = (cl_context_properties)(platforms[i]());

        cl_context_properties cprops[3] = {CL_CONTEXT_PLATFORM, p, 0};
        context = cl::Context(_devicePreference1, cprops, NULL, NULL, err);

        if (*err == CL_SUCCESS) {
            return context;
        }
    }

    // If first preference device not found, try to find a platform with 
    // second device preference.
    for (unsigned int i = 0; i < platforms.size(); i++) {
        cl_context_properties p = (cl_context_properties)(platforms[i]());

        cl_context_properties cprops[3] = {CL_CONTEXT_PLATFORM, p, 0};
        context = cl::Context(_devicePreference2, cprops, NULL, NULL, err);

        if (*err == CL_SUCCESS) {
            return context;
        }
    }

    *err = -1;

    return context;
}

cl::Device CLScalarField::_getCLDevice(cl::Context &context, cl_int *err) {
    std::vector<cl::Device> devices;
    devices = context.getInfo<CL_CONTEXT_DEVICES>();

    if (devices.size() == 0) {
        *err = -1;
        return cl::Device();
    }

    *err = CL_SUCCESS;
    return devices[0];
}

CLScalarField::CLDeviceInfo CLScalarField::_initializeDeviceInfo(cl::Device &device) {
    CLDeviceInfo info;

    device.getInfo(CL_DEVICE_NAME, &(info.cl_device_name));
    device.getInfo(CL_DEVICE_VENDOR, &(info.cl_device_vendor));
    device.getInfo(CL_DEVICE_VERSION, &(info.cl_device_version));
    device.getInfo(CL_DRIVER_VERSION, &(info.cl_driver_version));
    device.getInfo(CL_DEVICE_OPENCL_C_VERSION, &(info.cl_device_opencl_c_version));

    clGetDeviceInfo(device(), CL_DEVICE_TYPE, 
                    sizeof(cl_device_type), &(info.device_type), NULL);
    clGetDeviceInfo(device(), CL_DEVICE_MAX_CLOCK_FREQUENCY, 
                    sizeof(cl_uint), &(info.cl_device_max_clock_frequency), NULL);
    clGetDeviceInfo(device(), CL_DEVICE_GLOBAL_MEM_SIZE, 
                    sizeof(cl_ulong), &(info.cl_device_global_mem_size), NULL);
    clGetDeviceInfo(device(), CL_DEVICE_LOCAL_MEM_SIZE, 
                    sizeof(cl_ulong), &(info.cl_device_local_mem_size), NULL);
    clGetDeviceInfo(device(), CL_DEVICE_MAX_MEM_ALLOC_SIZE, 
                    sizeof(cl_ulong), &(info.cl_device_max_mem_alloc_size), NULL);
    clGetDeviceInfo(device(), CL_DEVICE_MAX_WORK_GROUP_SIZE, 
                    sizeof(size_t), &(info.cl_device_max_work_group_size), NULL);

    std::vector<size_t> workItemSizes;
    device.getInfo(CL_DEVICE_MAX_WORK_ITEM_SIZES, &workItemSizes);

    GridIndex groupdims(1, 1, 1);
    if (workItemSizes.size() >= 1) {
        groupdims.i = (int)workItemSizes[0];
    }
    if (workItemSizes.size() >= 2) {
        groupdims.j = (int)workItemSizes[1];
    }
    if (workItemSizes.size() >= 3) {
        groupdims.k = (int)workItemSizes[2];
    }
    info.cl_device_max_work_item_sizes = groupdims;

    return info;
}

CLScalarField::CLKernelInfo CLScalarField::_initializeKernelInfo(cl::Kernel &kernel) {
    CLKernelInfo info;

    kernel.getInfo(CL_KERNEL_FUNCTION_NAME, &(info.cl_kernel_function_name));
    kernel.getInfo(CL_KERNEL_ATTRIBUTES, &(info.cl_kernel_attributes));

    clGetKernelInfo (kernel(), CL_KERNEL_NUM_ARGS,
                     sizeof(cl_ulong), &(info.cl_kernel_num_args), NULL);
    clGetKernelWorkGroupInfo(kernel(), _CLDevice(), CL_KERNEL_WORK_GROUP_SIZE,
                             sizeof(size_t), &(info.cl_kernel_work_group_size), NULL);
    clGetKernelWorkGroupInfo(kernel(), _CLDevice(), CL_KERNEL_LOCAL_MEM_SIZE,
                             sizeof(cl_ulong), &(info.cl_kernel_local_mem_size), NULL);
    clGetKernelWorkGroupInfo(kernel(), _CLDevice(), CL_KERNEL_PRIVATE_MEM_SIZE,
                             sizeof(cl_ulong), &(info.cl_kernel_private_mem_size), NULL);
    clGetKernelWorkGroupInfo(kernel(), _CLDevice(), CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE,
                             sizeof(size_t), &(info.cl_kernel_preferred_work_group_size_multiple), NULL);
    return info;
}

cl_int CLScalarField::_initializeCLKernels() {
    std::string prog = Kernels::scalarfieldCL;
    cl::Program::Sources source(1, std::make_pair(prog.c_str(), prog.length()+1));
    cl::Program program(_CLContext, source);

    std::vector<cl::Device> devices = _CLContext.getInfo<CL_CONTEXT_DEVICES>();

    cl_int err = program.build(devices, "");
    if (err != CL_SUCCESS) {
        return err;
    }

    cl::Kernel kernel(program, "compute_scalar_field_points", &err);
    if (err != CL_SUCCESS) {
        return err;
    }
    _CLKernelPoints = kernel;

    kernel = cl::Kernel(program, "compute_scalar_field_point_values", &err);
    if (err != CL_SUCCESS) {
        return err;
    }
    _CLKernelPointValues = kernel;

    kernel = cl::Kernel(program, "compute_scalar_weight_field_point_values", &err);
    if (err != CL_SUCCESS) {
        return err;
    }
    _CLKernelWeightPointValues = kernel;

    return CL_SUCCESS;
}

std::string CLScalarField::_getProgramString(std::string filename) {
    std::ifstream file(filename);
    _checkError(file.is_open() ? CL_SUCCESS : -1, filename.c_str());
    
    std::string prog(std::istreambuf_iterator<char>(file), (std::istreambuf_iterator<char>()));

    return prog;
}

cl_int CLScalarField::_initializeCLCommandQueue() {
    cl_int err;
    cl::CommandQueue queue(_CLContext, _CLDevice, 0, &err);
    if (err != CL_SUCCESS) {
        return err;
    }

    _CLQueue = queue;

    return CL_SUCCESS;
}

/*  
    The scalarfield.cl kernels calculate field values at cell centers. We want
    values to be calculated at minimal cell corners to match the convention of
    the ScalarField class. To do this, <0.5dx, 0.5dx, 0.5dx> is
    subtracted from the offset that the user sets.
*/ 
vmath::vec3 CLScalarField::_getInternalOffset() {
    return _offset - vmath::vec3(0.5*_dx, 0.5*_dx, 0.5*_dx);
}

void CLScalarField::_initializePointValues(std::vector<vmath::vec3> &points,
                                           std::vector<PointValue> &pvs) {
    pvs.reserve(points.size());
    float defaultValue = 0.0;
    vmath::vec3 offset = _getInternalOffset();
    for (unsigned int i = 0; i < points.size(); i++) {
        pvs.push_back(PointValue(points[i] - offset, defaultValue));
    }
}

void CLScalarField::_initializePointValues(std::vector<vmath::vec3> &points,
                                           std::vector<float> &values,
                                           std::vector<PointValue> &pvs) {
    FLUIDSIM_ASSERT(points.size() == values.size());

    vmath::vec3 offset = _getInternalOffset();
    pvs.reserve(points.size());
    for (unsigned int i = 0; i < points.size(); i++) {
        pvs.push_back(PointValue(points[i] - offset, values[i]));
    }
}

GridIndex CLScalarField::_getWorkGroupGridDimensions() {
    int igrid = ceil((double)_isize / (double)_chunkWidth);
    int jgrid = ceil((double)_jsize / (double)_chunkHeight);
    int kgrid = ceil((double)_ksize / (double)_chunkDepth);
    return GridIndex(igrid, jgrid, kgrid);
}

void CLScalarField::_initializeWorkGroupGrid(std::vector<PointValue> &points,
                                             Array3d<float> *scalarfield,
                                             Array3d<WorkGroup> &grid) {
    _initializeWorkGroupParameters(grid, scalarfield);
    Array3d<int> countGrid(grid.width, grid.height, grid.depth, 0);
    //_getWorkGroupParticleCounts(points, countGrid);       // Uses less memory at the cost
                                                            // of speed if uncommented
    _reserveWorkGroupGridParticleMemory(grid, countGrid);
    _insertParticlesIntoWorkGroupGrid(points, grid);
}

void CLScalarField::_initializeWorkGroupGrid(std::vector<PointValue> &points,
                                             Array3d<float> *scalarfield,
                                             Array3d<float> *weightfield,
                                             Array3d<WorkGroup> &grid) {
    _initializeWorkGroupParameters(grid, scalarfield, weightfield);
    Array3d<int> countGrid(grid.width, grid.height, grid.depth, 0);
    //_getWorkGroupParticleCounts(points, countGrid);       // Uses less memory at the cost
                                                            // of speed if uncommented
    _reserveWorkGroupGridParticleMemory(grid, countGrid);
    _insertParticlesIntoWorkGroupGrid(points, grid);
}

void CLScalarField::_initializeWorkGroupParameters(Array3d<WorkGroup> &grid,
                                                   Array3d<float> *scalarfield) {
    GridIndex chunkOffset, indexOffset;
    vmath::vec3 positionOffset;
    WorkGroup *group;
    for (int k = 0; k < grid.depth; k++) {
        for (int j = 0; j < grid.height; j++) {
            for (int i = 0; i < grid.width; i++) {
                group = grid.getPointer(i, j, k);

                chunkOffset = GridIndex(i, j, k);
                indexOffset = GridIndex(i*_chunkWidth, 
                                        j*_chunkHeight,
                                        k*_chunkDepth);
                positionOffset = Grid3d::GridIndexToPosition(indexOffset, _dx);

                group->fieldview = ArrayView3d<float>(_chunkWidth, _chunkHeight, _chunkDepth,
                                                      indexOffset,
                                                      scalarfield);
                group->chunkOffset = chunkOffset;
                group->indexOffset = indexOffset;
                group->positionOffset = positionOffset;
            }
        }
    }
}

void CLScalarField::_initializeWorkGroupParameters(Array3d<WorkGroup> &grid,
                                                   Array3d<float> *scalarfield,
                                                   Array3d<float> *weightfield) {
    GridIndex chunkOffset, indexOffset;
    vmath::vec3 positionOffset;
    WorkGroup *group;
    for (int k = 0; k < grid.depth; k++) {
        for (int j = 0; j < grid.height; j++) {
            for (int i = 0; i < grid.width; i++) {
                group = grid.getPointer(i, j, k);

                chunkOffset = GridIndex(i, j, k);
                indexOffset = GridIndex(i*_chunkWidth, 
                                        j*_chunkHeight,
                                        k*_chunkDepth);
                positionOffset = Grid3d::GridIndexToPosition(indexOffset, _dx);

                group->fieldview = ArrayView3d<float>(_chunkWidth, _chunkHeight, _chunkDepth,
                                                      indexOffset,
                                                      scalarfield);
                group->weightfieldview = ArrayView3d<float>(_chunkWidth, _chunkHeight, _chunkDepth,
                                                            indexOffset,
                                                            weightfield);
                group->chunkOffset = chunkOffset;
                group->indexOffset = indexOffset;
                group->positionOffset = positionOffset;
            }
        }
    }
}

void CLScalarField::_reserveWorkGroupGridParticleMemory(Array3d<WorkGroup> &grid,
                                                        Array3d<int> &countGrid) {
    WorkGroup *group;
    for (int k = 0; k < grid.depth; k++) {
        for (int j = 0; j < grid.height; j++) {
            for (int i = 0; i < grid.width; i++) {
                group = grid.getPointer(i, j, k);
                group->particles.reserve(countGrid(i, j, k));
            }
        }
    }
}

void CLScalarField::_getWorkGroupParticleCounts(std::vector<PointValue> &points,
                                                Array3d<int> &countGrid) {
    double chunkdx = _chunkWidth * _dx;
    double chunkdy = _chunkHeight * _dx;
    double chunkdz = _chunkDepth * _dx;
    double invchunkdx = 1.0 / chunkdx;
    double invchunkdy = 1.0 / chunkdy;
    double invchunkdz = 1.0 / chunkdz;

    GridIndex gmax(countGrid.width, countGrid.height, countGrid.depth);

    AABB cbbox(vmath::vec3(), chunkdx - 2 * _radius, 
                              chunkdy - 2 * _radius, 
                              chunkdz - 2 * _radius);

    AABB pbbox(vmath::vec3(), 2 * _radius, 2 * _radius, 2 * _radius);

    vmath::vec3 p, minp, maxp;
    int mini, minj, mink, maxi, maxj, maxk;
    for (unsigned int i = 0; i < points.size(); i++) {
        p = points[i].position;

        int ci = floor(p.x * invchunkdx);
        int cj = floor(p.y * invchunkdy);
        int ck = floor(p.z * invchunkdz);
        double cx = (double)ci * chunkdx;
        double cy = (double)cj * chunkdy;
        double cz = (double)ck * chunkdz;

        cbbox.position = vmath::vec3(cx + _radius, cy + _radius, cz + _radius);
        if (cbbox.isPointInside(p) && Grid3d::isGridIndexInRange(ci, cj, ck, gmax)) {
            // sphere is contained within one grid cell
            countGrid.add(ci, cj, ck, 1);
            continue;
        }

        // sphere is contained within at least 2 grid cells
        minp = vmath::vec3(p.x - _radius, p.y - _radius, p.z - _radius);
        maxp = vmath::vec3(p.x + _radius, p.y + _radius, p.z + _radius);
        mini = fmax(floor(minp.x * invchunkdx), 0);
        minj = fmax(floor(minp.y * invchunkdy), 0);
        mink = fmax(floor(minp.z * invchunkdz), 0);
        maxi = fmin(floor(maxp.x * invchunkdx), gmax.i - 1);
        maxj = fmin(floor(maxp.y * invchunkdy), gmax.j - 1);
        maxk = fmin(floor(maxp.z * invchunkdz), gmax.k - 1);

        for (int ck = mink; ck <= maxk; ck++) {
            for (int cj = minj; cj <= maxj; cj++) {
                for (int ci = mini; ci <= maxi; ci++) {
                    countGrid.add(ci, cj, ck, 1);
                }
            }
        }
    }
}

void CLScalarField::_insertParticlesIntoWorkGroupGrid(std::vector<PointValue> &points,
                                                      Array3d<WorkGroup> &grid) {

    double chunkdx = _chunkWidth * _dx;
    double chunkdy = _chunkHeight * _dx;
    double chunkdz = _chunkDepth * _dx;
    double invchunkdx = 1.0 / chunkdx;
    double invchunkdy = 1.0 / chunkdy;
    double invchunkdz = 1.0 / chunkdz;

    GridIndex gmax(grid.width, grid.height, grid.depth);

    AABB cbbox(vmath::vec3(), chunkdx - 2 * _radius, 
                              chunkdy - 2 * _radius, 
                              chunkdz - 2 * _radius);

    AABB pbbox(vmath::vec3(), 2 * _radius, 2 * _radius, 2 * _radius);

    PointValue pv;
    vmath::vec3 p, minp, maxp;
    int mini, minj, mink, maxi, maxj, maxk;
    WorkGroup *group;
    for (unsigned int i = 0; i < points.size(); i++) {
        pv = points[i];
        p = pv.position;

        int ci = floor(p.x * invchunkdx);
        int cj = floor(p.y * invchunkdy);
        int ck = floor(p.z * invchunkdz);
        double cx = (double)ci * chunkdx;
        double cy = (double)cj * chunkdy;
        double cz = (double)ck * chunkdz;

        cbbox.position = vmath::vec3(cx + _radius, cy + _radius, cz + _radius);
        if (cbbox.isPointInside(p) && Grid3d::isGridIndexInRange(ci, cj, ck, gmax)) {
            // sphere is contained within one grid cell
            group = grid.getPointer(ci, cj, ck);
            group->particles.push_back(pv);
            continue;
        }

        // sphere is contained within at least 2 grid cells
        minp = vmath::vec3(p.x - _radius, p.y - _radius, p.z - _radius);
        maxp = vmath::vec3(p.x + _radius, p.y + _radius, p.z + _radius);
        mini = fmax(floor(minp.x * invchunkdx), 0);
        minj = fmax(floor(minp.y * invchunkdy), 0);
        mink = fmax(floor(minp.z * invchunkdz), 0);
        maxi = fmin(floor(maxp.x * invchunkdx), gmax.i - 1);
        maxj = fmin(floor(maxp.y * invchunkdy), gmax.j - 1);
        maxk = fmin(floor(maxp.z * invchunkdz), gmax.k - 1);

        for (int ck = mink; ck <= maxk; ck++) {
            for (int cj = minj; cj <= maxj; cj++) {
                for (int ci = mini; ci <= maxi; ci++) {
                    group = grid.getPointer(ci, cj, ck);
                    group->particles.push_back(pv);
                }
            }
        }
    }
}


bool CLScalarField::_compareWorkChunkByNumParticles(const WorkChunk &c1, const WorkChunk &c2) {
    return c1.particlesEnd - c1.particlesBegin < c2.particlesEnd - c2.particlesBegin;
}

void CLScalarField::_initializeWorkChunks(Array3d<WorkGroup> &grid,
                                          std::vector<WorkChunk> &chunks) {
    WorkGroup *group;
    for (int k = 0; k < grid.depth; k++) {
        for (int j = 0; j < grid.height; j++) {
            for (int i = 0; i < grid.width; i++) {
                group = grid.getPointer(i, j, k);
                _getWorkChunksFromWorkGroup(group, chunks);
            }
        }
    }
    chunks.shrink_to_fit();

    std::sort(chunks.begin(), chunks.end(), _compareWorkChunkByNumParticles);
}

void CLScalarField::_getWorkChunksFromWorkGroup(WorkGroup *group, 
                                                std::vector<WorkChunk> &chunks) {
    if (group->particles.size() == 0) {
        return;
    }

    GridIndex groupidx = group->chunkOffset; 
    int size = (int)group->particles.size();
    int chunksize = _maxParticlesPerChunk;

    for (int i = 0; i < size; i += chunksize) {
        WorkChunk c;
        c.workGroupIndex = groupidx;

        int begidx = i;
        int endidx = begidx + chunksize;
        if (endidx > size) {
            endidx = size;
        }

        c.particlesBegin = group->particles.begin() + begidx;
        c.particlesEnd = group->particles.begin() + endidx;

        chunks.push_back(c);
    }
}

void CLScalarField::_getNextWorkChunksToProcess(std::vector<WorkChunk> &queue,
                                                Array3d<WorkGroup> &grid,
                                                std::vector<WorkChunk> &chunks,
                                                int n) {
    WorkChunk c;
    WorkGroup *g;
    while ((int)chunks.size() < n && !queue.empty()) {
        c = queue.back();
        queue.pop_back();

        if (_isMaxScalarFieldValueThresholdSet) {
            g = grid.getPointer(c.workGroupIndex);
            float minval = g->minScalarFieldValue;

            if (minval < _maxScalarFieldValueThreshold) {
                chunks.push_back(c);
            }
        } else {
            chunks.push_back(c);
        }
    }
}

int CLScalarField::_getChunkPointDataSize() {
    return 3*_maxParticlesPerChunk*sizeof(float);
}

int CLScalarField::_getChunkPointValueDataSize() {
    return 4*_maxParticlesPerChunk*sizeof(float);
}

int CLScalarField::_getChunkScalarFieldDataSize() {
    return _chunkWidth * _chunkHeight * _chunkDepth * sizeof(float);
}

int CLScalarField::_getChunkScalarWeightFieldDataSize() {
    return 2 * _getChunkScalarFieldDataSize();
}

int CLScalarField::_getChunkOffsetDataSize() {
    return 3*sizeof(int);
}

int CLScalarField::_getMaxChunksPerPointComputation() {
    int pointDataSize = _getChunkPointDataSize();
    int fieldDataSize = _getChunkScalarFieldDataSize();
    int offsetDataSize = _getChunkOffsetDataSize();

    return _getMaxChunkLimit(pointDataSize, fieldDataSize, offsetDataSize);
}

int CLScalarField::_getMaxChunksPerPointValueComputation() {
    int pointDataSize = _getChunkPointValueDataSize();
    int fieldDataSize = _getChunkScalarFieldDataSize();
    int offsetDataSize = _getChunkOffsetDataSize();

    return _getMaxChunkLimit(pointDataSize, fieldDataSize, offsetDataSize);
}

int CLScalarField::_getMaxChunksPerWeightPointValueComputation() {
    int pointDataSize = _getChunkPointValueDataSize();
    int fieldDataSize = _getChunkScalarWeightFieldDataSize();
    int offsetDataSize = _getChunkOffsetDataSize();

    return _getMaxChunkLimit(pointDataSize, fieldDataSize, offsetDataSize);
}

int CLScalarField::_getMaxChunkLimit(int pointDataSize, int fieldDataSize, int offsetDataSize) {
    cl_ulong maxGlobalMem = _deviceInfo.cl_device_global_mem_size;
    cl_ulong maxAlloc = _deviceInfo.cl_device_max_mem_alloc_size;

    int numPositionAllocItems = floor((double)maxAlloc / (double)pointDataSize);
    int numFieldAllocItems = floor((double)maxAlloc / (double)fieldDataSize);
    int numOffsetAllocItems = floor((double)maxAlloc / (double)offsetDataSize);

    int allocLimitCount = fmin(fmin(numPositionAllocItems, 
                                    numFieldAllocItems),
                                    numOffsetAllocItems);

    int totalDataSize = pointDataSize + fieldDataSize + offsetDataSize;
    int globalMemLimitCount = floor((double)maxGlobalMem / (double)totalDataSize);

    int hardwareLimit = fmin(allocLimitCount, globalMemLimitCount);
    int softwareLimit = _maxChunksPerComputation;

    return fmin(hardwareLimit, softwareLimit);
}

void CLScalarField::_computePointScalarField(std::vector<WorkChunk> &chunks,
                                             Array3d<WorkGroup> &workGroupGrid) {
    
    int numParticles = _getMaxNumParticlesInChunk(chunks);

    DataBuffer buffer;
    _initializePointComputationDataBuffer(chunks, workGroupGrid, numParticles, buffer);
    _setPointComputationCLKernelArgs(buffer, numParticles);

    int numWorkItems = (int)chunks.size() * _workGroupSize;
    _launchKernel(_CLKernelPoints, numWorkItems, _workGroupSize);

    int dataSize = (int)chunks.size() * _getChunkScalarFieldDataSize();
    _readCLBuffer(buffer.scalarFieldDataCL, buffer.scalarFieldDataH, dataSize);
    _setPointComputationOutputFieldData(buffer.scalarFieldDataH, chunks, workGroupGrid);
}

void CLScalarField::_computePointValueScalarField(std::vector<WorkChunk> &chunks,
                                                  Array3d<WorkGroup> &workGroupGrid) {
    
    
    int numParticles = _getMaxNumParticlesInChunk(chunks);

    DataBuffer buffer;
    _initializePointValueComputationDataBuffer(chunks, workGroupGrid, numParticles, buffer);
    _setPointValueComputationCLKernelArgs(buffer, numParticles);

    int numWorkItems = (int)chunks.size() * _workGroupSize;
    _launchKernel(_CLKernelPointValues, numWorkItems, _workGroupSize);

    int dataSize = (int)chunks.size() * _getChunkScalarFieldDataSize();
    _readCLBuffer(buffer.scalarFieldDataCL, buffer.scalarFieldDataH, dataSize);
    _setPointValueComputationOutputFieldData(buffer.scalarFieldDataH, chunks, workGroupGrid);
}

void CLScalarField::_computePointValueScalarWeightField(std::vector<WorkChunk> &chunks,
                                                        Array3d<WorkGroup> &workGroupGrid) {

    int numParticles = _getMaxNumParticlesInChunk(chunks);

    DataBuffer buffer;
    _initializeWeightPointValueComputationDataBuffer(chunks, workGroupGrid, numParticles, buffer);
    _setWeightPointValueComputationCLKernelArgs(buffer, numParticles);

    int numWorkItems = (int)chunks.size() * _workGroupSize;
    _launchKernel(_CLKernelWeightPointValues, numWorkItems, _workGroupSize);

    int dataSize = (int)chunks.size() * _getChunkScalarWeightFieldDataSize();
    _readCLBuffer(buffer.scalarFieldDataCL, buffer.scalarFieldDataH, dataSize);
    _setWeightPointValueComputationOutputFieldData(buffer.scalarFieldDataH, 
                                                   chunks, 
                                                   workGroupGrid);
}

int CLScalarField::_getMaxNumParticlesInChunk(std::vector<WorkChunk> &chunks) {
    int maxParticles = 0;
    for (unsigned int i = 0; i < chunks.size(); i++) {
        int n = chunks[i].particlesEnd - chunks[i].particlesBegin;
        if (n > maxParticles) {
            maxParticles = n;
        }
    }

    return maxParticles;
}

void CLScalarField::_initializePointComputationDataBuffer(std::vector<WorkChunk> &chunks,
                                                          Array3d<WorkGroup> &workGroupGrid,
                                                          int numParticles,
                                                          DataBuffer &buffer) {
    _getHostPointDataBuffer(chunks, workGroupGrid, numParticles, buffer.pointDataH);
    _getHostScalarFieldDataBuffer(chunks, buffer.scalarFieldDataH);
    _getHostChunkOffsetDataBuffer(chunks, buffer.offsetDataH);
    _initializeCLDataBuffers(buffer);
}

void CLScalarField::_initializePointValueComputationDataBuffer(std::vector<WorkChunk> &chunks,
                                                               Array3d<WorkGroup> &workGroupGrid,
                                                               int numParticles,
                                                               DataBuffer &buffer) {
    _getHostPointValueDataBuffer(chunks, workGroupGrid, numParticles, buffer.pointDataH);
    _getHostScalarFieldDataBuffer(chunks, buffer.scalarFieldDataH);
    _getHostChunkOffsetDataBuffer(chunks, buffer.offsetDataH);
    _initializeCLDataBuffers(buffer);
}

void CLScalarField::_initializeWeightPointValueComputationDataBuffer(std::vector<WorkChunk> &chunks,
                                                                     Array3d<WorkGroup> &workGroupGrid,
                                                                     int numParticles,
                                                                     DataBuffer &buffer) {
    _getHostPointValueDataBuffer(chunks, workGroupGrid, numParticles, buffer.pointDataH);
    _getHostScalarWeightFieldDataBuffer(chunks, buffer.scalarFieldDataH);
    _getHostChunkOffsetDataBuffer(chunks, buffer.offsetDataH);
    _initializeCLDataBuffers(buffer);
}

void CLScalarField::_initializeCLDataBuffers(DataBuffer &buffer) {
    size_t pointDataBytes = buffer.pointDataH.size() * sizeof(float);
    size_t scalarFieldDataBytes = buffer.scalarFieldDataH.size() * sizeof(float);
    size_t offsetDataBytes = buffer.offsetDataH.size() * sizeof(GridIndex);

    cl_int err;
    buffer.positionDataCL = cl::Buffer(_CLContext, 
                                       CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, 
                                       pointDataBytes, 
                                       (void*)&(buffer.pointDataH[0]), 
                                       (cl_int*)&err);
    _checkError(err, "Creating position data buffer");

    buffer.scalarFieldDataCL = cl::Buffer(_CLContext, 
                                          CL_MEM_WRITE_ONLY | CL_MEM_USE_HOST_PTR, 
                                          scalarFieldDataBytes, 
                                          (void*)&(buffer.scalarFieldDataH[0]), 
                                          &err);
    _checkError(err, "Creating scalar field data buffer");

    buffer.offsetDataCL = cl::Buffer(_CLContext, 
                                     CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, 
                                     offsetDataBytes, 
                                     (void*)&(buffer.offsetDataH[0]), 
                                     &err);
    _checkError(err, "Creating chunk offset data buffer");
}

void CLScalarField::_getHostPointDataBuffer(std::vector<WorkChunk> &chunks,
                                            Array3d<WorkGroup> &grid,
                                            int numParticles,
                                            std::vector<float> &buffer) {
    int numElements = (int)chunks.size() * 3 * numParticles;
    buffer.reserve(numElements);

    // Dummy position that is far away enough from the scalar field that it
    // will not affect any scalar field values
    vmath::vec3 outOfRangePos(grid.width * _chunkWidth * _dx + 2 * _radius,
                              grid.height * _chunkHeight * _dx + 2 * _radius,
                              grid.depth * _chunkDepth * _dx + 2 * _radius);
    WorkChunk c;
    vmath::vec3 p;
    std::vector<PointValue>::iterator beg;
    std::vector<PointValue>::iterator end;
    for (unsigned int i = 0; i < chunks.size(); i++) {
        c = chunks[i];

        int numPoints = c.particlesEnd - c.particlesBegin;
        int numPad = numParticles - numPoints;

        beg = c.particlesBegin;
        end = c.particlesEnd;
        for (std::vector<PointValue>::iterator it = beg; it != end; ++it) {
            p = (*it).position;
            buffer.push_back(p.x);
            buffer.push_back(p.y);
            buffer.push_back(p.z);
        }

        for (int i = 0; i < numPad; i++) {
            buffer.push_back(outOfRangePos.x);
            buffer.push_back(outOfRangePos.y);
            buffer.push_back(outOfRangePos.z);
        }
    }
}

void CLScalarField::_getHostPointValueDataBuffer(std::vector<WorkChunk> &chunks,
                                                 Array3d<WorkGroup> &grid,
                                                 int numParticles,
                                                 std::vector<float> &buffer) {
    int numElements = (int)chunks.size() * 4 * numParticles;
    buffer.reserve(numElements);

    // Dummy position that is far away enough from the scalar field that it
    // will not affect any scalar field values
    vmath::vec3 outOfRangePos(grid.width * _chunkWidth * _dx + 2 * _radius,
                              grid.height * _chunkHeight * _dx + 2 * _radius,
                              grid.depth * _chunkDepth * _dx + 2 * _radius);
    float outOfRangeValue = 0.0f;

    WorkChunk c;
    vmath::vec3 p;
    float v;
    std::vector<PointValue>::iterator beg;
    std::vector<PointValue>::iterator end;
    for (unsigned int i = 0; i < chunks.size(); i++) {
        c = chunks[i];

        int numPoints = c.particlesEnd - c.particlesBegin;
        int numPad = numParticles - numPoints;

        beg = c.particlesBegin;
        end = c.particlesEnd;
        for (std::vector<PointValue>::iterator it = beg; it != end; ++it) {
            p = (*it).position;
            v = (*it).value;
            buffer.push_back(p.x);
            buffer.push_back(p.y);
            buffer.push_back(p.z);
            buffer.push_back(v);
        }

        for (int i = 0; i < numPad; i++) {
            buffer.push_back(outOfRangePos.x);
            buffer.push_back(outOfRangePos.y);
            buffer.push_back(outOfRangePos.z);
            buffer.push_back(outOfRangeValue);
        }
    }
}

void CLScalarField::_getHostScalarFieldDataBuffer(std::vector<WorkChunk> &chunks,
                                                  std::vector<float> &buffer) {
    int numElements = (int)chunks.size() * _chunkWidth * _chunkHeight * _chunkWidth;
    buffer.reserve(numElements);
    for (int i = 0; i < numElements; i++) {
        buffer.push_back(0.0);
    }
}

void CLScalarField::_getHostScalarWeightFieldDataBuffer(std::vector<WorkChunk> &chunks,
                                                        std::vector<float> &buffer) {
    int numElements = 2 * (int)chunks.size() * _chunkWidth * _chunkHeight * _chunkWidth;
    buffer.reserve(numElements);
    for (int i = 0; i < numElements; i++) {
        buffer.push_back(0.0);
    }
}

void CLScalarField::_getHostChunkOffsetDataBuffer(std::vector<WorkChunk> &chunks,
                                                  std::vector<GridIndex> &buffer) {
    buffer.reserve(chunks.size());
    for (unsigned int i = 0; i < chunks.size(); i++) {
        buffer.push_back(chunks[i].workGroupIndex);
    }
}

void CLScalarField::_setPointComputationCLKernelArgs(DataBuffer &buffer, 
                                                     int numParticles) {
    int localDataBytes = numParticles * 3 * sizeof(float);
    _setKernelArgs(_CLKernelPoints, 
                   buffer, localDataBytes, numParticles, _radius, _dx);
}

void CLScalarField::_setPointValueComputationCLKernelArgs(DataBuffer &buffer, 
                                                          int numParticles) {
    int localDataBytes = numParticles * 4 * sizeof(float);
    _setKernelArgs(_CLKernelPointValues, 
                   buffer, localDataBytes, numParticles, _radius, _dx);
}

void CLScalarField::_setWeightPointValueComputationCLKernelArgs(DataBuffer &buffer, 
                                                                int numParticles) {
    int localDataBytes = numParticles * 4 * sizeof(float);
    _setKernelArgs(_CLKernelWeightPointValues, 
                   buffer, localDataBytes, numParticles, _radius, _dx);
}

void CLScalarField::_setKernelArgs(cl::Kernel &kernel, 
                                   DataBuffer &buffer, 
                                   int localDataBytes,
                                   int numParticles, 
                                   double radius, 
                                   double dx) {
    cl_int err = kernel.setArg(0, buffer.positionDataCL);
    _checkError(err, "Kernel::setArg() - position data");

    err = kernel.setArg(1, buffer.scalarFieldDataCL);
    _checkError(err, "Kernel::setArg() - scalar field data");

    err = kernel.setArg(2, buffer.offsetDataCL);
    _checkError(err, "Kernel::setArg() - chunk offset data");

    FLUIDSIM_ASSERT((unsigned int)localDataBytes <= _deviceInfo.cl_device_local_mem_size);
    err = kernel.setArg(3, cl::__local(localDataBytes));
    _checkError(err, "Kernel::setArg() - local position data");

    err = kernel.setArg(4, numParticles);
    _checkError(err, "Kernel::setArg() - num particles");

    int numGroups = buffer.offsetDataH.size();
    err = kernel.setArg(5, numGroups);
    _checkError(err, "Kernel::setArg() - num groups");

    err = kernel.setArg(6, (float)radius);
    _checkError(err, "Kernel::setArg() - radius");

    err = kernel.setArg(7, (float)dx);
    _checkError(err, "Kernel::setArg() - dx");
}

void CLScalarField::_launchKernel(cl::Kernel &kernel, int numWorkItems, int workGroupSize) {
    int numChunks = numWorkItems / workGroupSize;
    int loadSize = _kernelWorkLoadSize;
    int numComputations = ceil((double)numChunks / (double)loadSize);

    cl::Event event;
    cl_int err;
    for (int i = 0; i < numComputations; i++) {
        int offset = i * loadSize * workGroupSize;
        int items = (int)fmin(numWorkItems - offset, loadSize * workGroupSize);
        
        err = _CLQueue.enqueueNDRangeKernel(kernel, 
                                            cl::NDRange(offset), 
                                            cl::NDRange(items), 
                                            cl::NDRange(workGroupSize), 
                                            NULL, 
                                            &event);    
        _checkError(err, "CommandQueue::enqueueNDRangeKernel()");
    }

    event.wait();
}

void CLScalarField::_readCLBuffer(cl::Buffer &sourceCL, std::vector<float> &destH, int dataSize) {
    FLUIDSIM_ASSERT((int)(destH.size() * sizeof(float)) >= dataSize);
    cl_int err = _CLQueue.enqueueReadBuffer(sourceCL, CL_TRUE, 0, dataSize, (void*)&(destH[0]));
    _checkError(err, "CommandQueue::enqueueReadBuffer()");
}

void CLScalarField::_setPointComputationOutputFieldData(std::vector<float> &buffer, 
                                                        std::vector<WorkChunk> &chunks,
                                                        Array3d<WorkGroup> &workGroupGrid) {
    int elementsPerChunk = _chunkWidth * _chunkHeight * _chunkDepth;
    FLUIDSIM_ASSERT(buffer.size() == chunks.size() * elementsPerChunk);

    GridIndex cg;
    ArrayView3d<float> fieldview;
    int bufferidx = 0;
    for (unsigned int cidx = 0; cidx < chunks.size(); cidx++) {
        cg = chunks[cidx].workGroupIndex;
        fieldview = workGroupGrid(cg).fieldview;

        for (int k = 0; k < fieldview.depth; k++) {
            for (int j = 0; j < fieldview.height; j++) {
                for (int i = 0; i < fieldview.width; i++) {
                    fieldview.add(i, j, k, buffer[bufferidx]);
                    bufferidx++;
                }
            }
        }
    }
}

void CLScalarField::_setPointValueComputationOutputFieldData(std::vector<float> &buffer, 
                                                             std::vector<WorkChunk> &chunks,
                                                             Array3d<WorkGroup> &workGroupGrid) {
    _setPointComputationOutputFieldData(buffer, chunks, workGroupGrid);
}

void CLScalarField::_setWeightPointValueComputationOutputFieldData(std::vector<float> &buffer, 
                                                                   std::vector<WorkChunk> &chunks,
                                                                   Array3d<WorkGroup> &workGroupGrid) {
    int elementsPerChunk = _chunkWidth * _chunkHeight * _chunkDepth;
    FLUIDSIM_ASSERT(buffer.size() == 2 * chunks.size() * elementsPerChunk);

    GridIndex cg;
    WorkGroup *group;
    ArrayView3d<float> scalarfieldview;
    ArrayView3d<float> weightfieldview;
    int bufferidx = 0;
    int weightfieldoffset = (int)chunks.size() * elementsPerChunk;
    for (unsigned int cidx = 0; cidx < chunks.size(); cidx++) {
        cg = chunks[cidx].workGroupIndex;
        group = workGroupGrid.getPointer(cg);
        scalarfieldview = group->fieldview;
        weightfieldview = group->weightfieldview;

        for (int k = 0; k < scalarfieldview.depth; k++) {
            for (int j = 0; j < scalarfieldview.height; j++) {
                for (int i = 0; i < scalarfieldview.width; i++) {
                    scalarfieldview.add(i, j, k, buffer[bufferidx]);
                    weightfieldview.add(i, j, k, buffer[bufferidx + weightfieldoffset]);
                    bufferidx++;
                }
            }
        }
    }
}

void CLScalarField::_updateWorkGroupMinimumValues(Array3d<WorkGroup> &grid) {
    WorkGroup *g;
    for (int k = 0; k < grid.depth; k++) {
        for (int j = 0; j < grid.height; j++) {
            for (int i = 0; i < grid.width; i++) {
                g = grid.getPointer(i, j, k);
                g->minScalarFieldValue = _getWorkGroupMinimumValue(g);
            }
        }
    }
}

float CLScalarField::_getWorkGroupMinimumValue(WorkGroup *g) {
    float minval = std::numeric_limits<float>::infinity();
    ArrayView3d<float> view = g->fieldview;
    for (int k = 0; k < view.depth; k++) {
        for (int j = 0; j < view.height; j++) {
            for (int i = 0; i < view.width; i++) {
                if (view.isIndexInParent(i, j, k) && view(i, j, k) < minval) {
                    minval = view(i, j, k);
                }
            }
        }
    }

    return minval;
}

void CLScalarField::_addPointsNoCL(std::vector<vmath::vec3> &points, 
                                   double radius,
                                   vmath::vec3 offset,
                                   double dx,
                                   Array3d<float> *field) {

    ScalarField calcfield(field->width, field->height, field->depth, dx);
    calcfield.setPointRadius(radius);
    calcfield.setOffset(offset);
    for (unsigned int i = 0; i < points.size(); i++) {
        calcfield.addPoint(points[i]);
    }

    Array3d<float>* calcfieldp = calcfield.getPointerToScalarField();
    _copyField(calcfieldp, field);

}

void CLScalarField::_addPointValuesNoCL(std::vector<vmath::vec3> &points, 
                                        std::vector<float> &values,
                                        double radius,
                                        vmath::vec3 offset,
                                        double dx,
                                        Array3d<float> *field) {

    ScalarField calcfield(field->width, field->height, field->depth, dx);
    calcfield.setPointRadius(radius);
    calcfield.setOffset(offset);
    for (unsigned int i = 0; i < points.size(); i++) {
        calcfield.addPointValue(points[i], values[i]);
    }

    Array3d<float>* calcfieldp = calcfield.getPointerToScalarField();
    _copyField(calcfieldp, field);
}

void CLScalarField::_addPointValuesNoCL(std::vector<vmath::vec3> &points, 
                                        std::vector<float> &values,
                                        double radius,
                                        vmath::vec3 offset,
                                        double dx,
                                        Array3d<float> *scalarfield,
                                        Array3d<float> *weightfield) {

    ScalarField calcfield(scalarfield->width, scalarfield->height, scalarfield->depth, dx);
    calcfield.enableWeightField();
    calcfield.setPointRadius(radius);
    calcfield.setOffset(offset);
    for (unsigned int i = 0; i < points.size(); i++) {
        calcfield.addPointValue(points[i], values[i]);
    }

    Array3d<float>* calcfieldp = calcfield.getPointerToScalarField();
    Array3d<float>* calcweightfieldp = calcfield.getPointerToWeightField();
    _copyField(calcfieldp, scalarfield);
    _copyField(calcweightfieldp, weightfield);
}

void CLScalarField::_copyField(Array3d<float> *src, Array3d<float> *dest) {
    for (int k = 0; k < dest->depth; k++) {
        for (int j = 0; j < dest->height; j++) {
            for (int i = 0; i < dest->width; i++) {
                dest->set(i, j, k, src->get(i, j, k));
            }
        }
    }
}

#ifdef __GNUC__
    #pragma GCC diagnostic pop
#endif