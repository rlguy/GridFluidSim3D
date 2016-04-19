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
    assert(_isInitialized);

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
                              ImplicitSurfaceScalarField &isfield) {

    Array3d<float> *field = isfield.getPointerToScalarField();
    addPoints(points, radius, offset, dx, field);
}

void CLScalarField::addPoints(std::vector<vmath::vec3> &points,
                              ImplicitSurfaceScalarField &isfield) {

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
    
    assert(_isInitialized);
    assert(points.size() == values.size());

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
                                   ImplicitSurfaceScalarField &isfield) {

    Array3d<float> *field = isfield.getPointerToScalarField();
    addPointValues(points, values, radius, offset, dx, field);
}

void CLScalarField::addPointValues(std::vector<vmath::vec3> &points, 
                                   std::vector<float> &values,
                                   ImplicitSurfaceScalarField &isfield) {

    double r = isfield.getPointRadius();
    vmath::vec3 offset = isfield.getOffset();
    double dx = isfield.getCellSize();
    Array3d<float> *field = isfield.getPointerToScalarField();
    addPointValues(points, values, r, offset, dx, field);
}

void CLScalarField::setMaxScalarFieldValueThreshold(float val) {
    _isMaxScalarFieldValueThresholdSet = true;
    _maxScalarFieldValueThreshold = val;
}

void CLScalarField::setMaxScalarFieldValueThreshold() {
    _isMaxScalarFieldValueThresholdSet = false;
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

    std::cout << "CL_DEVICE_NAME:                " << 
                 _deviceInfo.cl_device_name << std::endl;
    std::cout << "CL_DEVICE_VENDOR:              " << 
                 _deviceInfo.cl_device_vendor << std::endl;
    std::cout << "CL_DEVICE_VERSION:             " << 
                 _deviceInfo.cl_device_version << std::endl;
    std::cout << "CL_DRIVER_VERSION:             " << 
                 _deviceInfo.cl_driver_version << std::endl;
    std::cout << "CL_DEVICE_OPENCL_C_VERSION:    " << 
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
    std::cout << "CL_DEVICE_TYPE:                " << 
                 type << std::endl;
    std::cout << "CL_DEVICE_MAX_CLOCK_FREQUENCY: " << 
                 _deviceInfo.cl_device_max_clock_frequency << "MHz" << std::endl;
    std::cout << "CL_DEVICE_GLOBAL_MEM_SIZE:     " << 
                 _deviceInfo.cl_device_global_mem_size << std::endl;
    std::cout << "CL_DEVICE_LOCAL_MEM_SIZE:      " << 
                 _deviceInfo.cl_device_local_mem_size << std::endl;
    std::cout << "CL_DEVICE_MAX_MEM_ALLOC_SIZE:  " << 
                 _deviceInfo.cl_device_max_mem_alloc_size << std::endl;
    std::cout << "CL_DEVICE_MAX_WORK_GROUP_SIZE: " << 
                 _deviceInfo.cl_device_max_work_group_size << std::endl;

    GridIndex g = _deviceInfo.cl_device_max_work_item_sizes;
    std::cout << "CL_DEVICE_MAX_WORK_ITEM_SIZES: " << g.i << " x " << 
                                                      g.j << " x " << 
                                                      g.k << std::endl;
}

cl_int CLScalarField::_initializeChunkDimensions() {
    int groupsize = (int)_deviceInfo.cl_device_max_work_group_size;
    groupsize = fmin(groupsize, _maxWorkGroupSize);

    if (groupsize < _minWorkGroupSize) {
        return (cl_int)-1;
    }

    std::vector<int> validsizes;
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
        for (unsigned int i = validsizes.size() - 1; i >= 0; i--) {
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

void CLScalarField::_checkError(cl_int err, const char * name) {
    if (err != CL_SUCCESS) {
        std::cerr << "ERROR: " << name  << " (" << err << ")" << std::endl;
        assert(err == CL_SUCCESS);
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
                    sizeof(cl_uint), &(info.cl_device_max_work_group_size), NULL);

    std::vector<int> workItemSizes;
    device.getInfo(CL_DEVICE_MAX_WORK_ITEM_SIZES, &workItemSizes);

    GridIndex groupdims(1, 1, 1);
    if (workItemSizes.size() >= 1) {
        groupdims.i = workItemSizes[0];
    }
    if (workItemSizes.size() >= 2) {
        groupdims.j = workItemSizes[1];
    }
    if (workItemSizes.size() >= 3) {
        groupdims.k = workItemSizes[2];
    }
    info.cl_device_max_work_item_sizes = groupdims;

    return info;
}

cl_int CLScalarField::_initializeCLKernels() {
    std::string prog = _getProgramString("src/kernels/scalarfield.cl");

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

void CLScalarField::_initializePointValues(std::vector<vmath::vec3> &points,
                                           std::vector<PointValue> &pvs) {
    pvs.reserve(points.size());
    float defaultValue = 0.0;
    for (unsigned int i = 0; i < points.size(); i++) {
        pvs.push_back(PointValue(points[i], defaultValue));
    }
}

void CLScalarField::_initializePointValues(std::vector<vmath::vec3> &points,
                                           std::vector<float> &values,
                                           std::vector<PointValue> &pvs) {
    assert(points.size() == values.size());

    pvs.reserve(points.size());
    for (unsigned int i = 0; i < points.size(); i++) {
        pvs.push_back(PointValue(points[i], values[i]));
    }
}

GridIndex CLScalarField::_getWorkGroupGridDimensions() {
    int igrid = ceil((double)_isize / (double)_chunkWidth);
    int jgrid = ceil((double)_jsize / (double)_chunkHeight);
    int kgrid = ceil((double)_ksize / (double)_chunkDepth);
    return GridIndex(igrid, jgrid, kgrid);
}

void CLScalarField::_initializeWorkGroupGrid(std::vector<PointValue> &points,
                                             Array3d<float> *scalarField,
                                             Array3d<WorkGroup> &grid) {
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

                group->fieldview = ArrayView3d<float>(_chunkWidth,
                                                      _chunkHeight,
                                                      _chunkDepth,
                                                      indexOffset,
                                                      scalarField);
                group->chunkOffset = chunkOffset;
                group->indexOffset = indexOffset;
                group->positionOffset = positionOffset;
            }
        }
    }

    Array3d<int> countGrid(grid.width, grid.height, grid.width, 0);
    //_getWorkGroupParticleCounts(points, countGrid);       // Uses less memory at the cost
                                                            // of speed if uncommented
    for (int k = 0; k < grid.depth; k++) {
        for (int j = 0; j < grid.height; j++) {
            for (int i = 0; i < grid.width; i++) {
                group = grid.getPointer(i, j, k);
                group->particles.reserve(countGrid(i, j, k));
            }
        }
    }

    _insertParticlesIntoWorkGroupGrid(points, grid);
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
        p = points[i].position - _offset;

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
        maxj = fmin(floor(maxp.y * invchunkdx), gmax.j - 1);
        maxk = fmin(floor(maxp.z * invchunkdx), gmax.k - 1);

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
        p = pv.position - _offset;

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
        maxj = fmin(floor(maxp.y * invchunkdx), gmax.j - 1);
        maxk = fmin(floor(maxp.z * invchunkdx), gmax.k - 1);

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
    int size = group->particles.size();
    int chunksize = _maxParticlesPerChunk;

    for (int i = 0; i <= size; i += chunksize) {
        WorkChunk c;
        c.workGroupIndex = groupidx;

        c.particlesBegin = group->particles.begin() + i;
        c.particlesEnd = c.particlesBegin + chunksize;

        if (group->particles.end() - c.particlesEnd < 0) {
            c.particlesEnd = group->particles.end();
        }

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

int CLScalarField::_getChunkOffsetDataSize() {
    return 3*sizeof(int);
}

int CLScalarField::_getPointChunkTotalDataSize() {
    return _getChunkPointDataSize() + 
           _getChunkScalarFieldDataSize() + 
           _getChunkOffsetDataSize();
}

int CLScalarField::_getPointValueChunkTotalDataSize() {
    return _getChunkPointValueDataSize() + 
           _getChunkScalarFieldDataSize() + 
           _getChunkOffsetDataSize();
}

int CLScalarField::_getMaxChunksPerPointComputation() {
    int positionSize = _getChunkPointDataSize();
    int fieldSize = _getChunkScalarFieldDataSize();
    int offsetSize = _getChunkOffsetDataSize();
    int totalSize = _getPointChunkTotalDataSize();

    int maxGlobalMem = _deviceInfo.cl_device_global_mem_size;
    int maxAlloc = _deviceInfo.cl_device_max_mem_alloc_size;

    int numPositionAllocItems = floor((double)maxAlloc / (double)positionSize);
    int numFieldAllocItems = floor((double)maxAlloc / (double)fieldSize);
    int numOffsetAllocItems = floor((double)maxAlloc / (double)offsetSize);

    int allocLimitCount = fmin(fmin(numPositionAllocItems, 
                                    numFieldAllocItems),
                                    numOffsetAllocItems);

    int globalMemLimitCount = floor((double)maxGlobalMem / (double)totalSize);

    int hardwareLimit = fmin(allocLimitCount, globalMemLimitCount);
    int softwareLimit = _maxChunksPerComputation;

    return fmin(hardwareLimit, softwareLimit);
}

int CLScalarField::_getMaxChunksPerPointValueComputation() {
    int positionSize = _getChunkPointValueDataSize();
    int fieldSize = _getChunkScalarFieldDataSize();
    int offsetSize = _getChunkOffsetDataSize();
    int totalSize = _getPointValueChunkTotalDataSize();

    int maxGlobalMem = _deviceInfo.cl_device_global_mem_size;
    int maxAlloc = _deviceInfo.cl_device_max_mem_alloc_size;

    int numPositionAllocItems = floor((double)maxAlloc / (double)positionSize);
    int numFieldAllocItems = floor((double)maxAlloc / (double)fieldSize);
    int numOffsetAllocItems = floor((double)maxAlloc / (double)offsetSize);

    int allocLimitCount = fmin(fmin(numPositionAllocItems, 
                                    numFieldAllocItems),
                                    numOffsetAllocItems);

    int globalMemLimitCount = floor((double)maxGlobalMem / (double)totalSize);

    int hardwareLimit = fmin(allocLimitCount, globalMemLimitCount);
    int softwareLimit = _maxChunksPerComputation;

    return fmin(hardwareLimit, softwareLimit);
}

void CLScalarField::_computePointScalarField(std::vector<WorkChunk> &chunks,
                                             Array3d<WorkGroup> &workGroupGrid) {
    
    int numParticles = _getMaxNumParticlesInChunk(chunks);

    DataBuffer buffer;
    _initializePointComputationDataBuffer(chunks, workGroupGrid, numParticles, buffer);
    _setPointComputationCLKernelArgs(buffer, numParticles, _dx);

    int numWorkItems = chunks.size() * _workGroupSize;

    cl::Event event;
    cl_int err = _CLQueue.enqueueNDRangeKernel(_CLKernelPoints, 
                                               cl::NullRange, 
                                               cl::NDRange(numWorkItems), 
                                               cl::NDRange(_workGroupSize), 
                                               NULL, 
                                               &event);    
    _checkError(err, "CommandQueue::enqueueNDRangeKernel()");

    event.wait();

    int dataSize = chunks.size() * _getChunkScalarFieldDataSize();
    err = _CLQueue.enqueueReadBuffer(buffer.scalarFieldDataCL, 
                                     CL_TRUE, 0, 
                                     dataSize, 
                                     (void*)&(buffer.scalarFieldDataH[0]));
    _checkError(err, "CommandQueue::enqueueReadBuffer()");

    _setPointComputationOutputScalarFieldData(buffer.scalarFieldDataH, chunks, workGroupGrid);
}

void CLScalarField::_computePointValueScalarField(std::vector<WorkChunk> &chunks,
                                                  Array3d<WorkGroup> &workGroupGrid) {
    
    
    int numParticles = _getMaxNumParticlesInChunk(chunks);

    DataBuffer buffer;
    _initializePointValueComputationDataBuffer(chunks, workGroupGrid, numParticles, buffer);

    _setPointValueComputationCLKernelArgs(buffer, numParticles, _dx);

    int numWorkItems = chunks.size() * _workGroupSize;

    cl::Event event;
    cl_int err = _CLQueue.enqueueNDRangeKernel(_CLKernelPointValues, 
                                               cl::NullRange, 
                                               cl::NDRange(numWorkItems), 
                                               cl::NDRange(_workGroupSize), 
                                               NULL, 
                                               &event);    
    _checkError(err, "CommandQueue::enqueueNDRangeKernel()");

    event.wait();

    int dataSize = chunks.size() * _getChunkScalarFieldDataSize();
    err = _CLQueue.enqueueReadBuffer(buffer.scalarFieldDataCL, 
                                     CL_TRUE, 0, 
                                     dataSize, 
                                     (void*)&(buffer.scalarFieldDataH[0]));
    _checkError(err, "CommandQueue::enqueueReadBuffer()");

    _setPointValueComputationOutputScalarFieldData(buffer.scalarFieldDataH, chunks, workGroupGrid);
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
    _getHostScalarFieldDataBuffer(chunks, workGroupGrid, buffer.scalarFieldDataH);
    _getHostChunkOffsetDataBuffer(chunks, buffer.offsetDataH);
    _initializeCLDataBuffers(buffer);
}

void CLScalarField::_initializePointValueComputationDataBuffer(std::vector<WorkChunk> &chunks,
                                                               Array3d<WorkGroup> &workGroupGrid,
                                                               int numParticles,
                                                               DataBuffer &buffer) {
    _getHostPointValueDataBuffer(chunks, workGroupGrid, numParticles, buffer.pointDataH);
    _getHostScalarFieldDataBuffer(chunks, workGroupGrid, buffer.scalarFieldDataH);
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
    int numElements = chunks.size() * 3 * numParticles;
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
    int numElements = chunks.size() * 4 * numParticles;
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
                                                  Array3d<WorkGroup> &grid,
                                                  std::vector<float> &buffer) {
    int numElements = chunks.size() * _chunkWidth * _chunkHeight * _chunkWidth;
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
                                                     int numParticles, 
                                                     double dx) {
    
    cl_int err = _CLKernelPoints.setArg(0, buffer.positionDataCL);
    _checkError(err, "Kernel::setArg() - position data");

    err = _CLKernelPoints.setArg(1, buffer.scalarFieldDataCL);
    _checkError(err, "Kernel::setArg() - scalar field data");

    err = _CLKernelPoints.setArg(2, buffer.offsetDataCL);
    _checkError(err, "Kernel::setArg() - chunk offset data");

    int localPointDataBytes = numParticles * 3 * sizeof(float);
    assert((unsigned int)localPointDataBytes <= _deviceInfo.cl_device_local_mem_size);

    err = _CLKernelPoints.setArg(3, cl::__local(localPointDataBytes));
    _checkError(err, "Kernel::setArg() - local position data");

    err = _CLKernelPoints.setArg(4, numParticles);
    _checkError(err, "Kernel::setArg() - num particles");

    err = _CLKernelPoints.setArg(5, (float)_radius);
    _checkError(err, "Kernel::setArg() - radius");

    err = _CLKernelPoints.setArg(6, (float)dx);
    _checkError(err, "Kernel::setArg() - dx");
    
}

void CLScalarField::_setPointValueComputationCLKernelArgs(DataBuffer &buffer, 
                                                          int numParticles, 
                                                          double dx) {
    cl_int err = _CLKernelPointValues.setArg(0, buffer.positionDataCL);
    _checkError(err, "Kernel::setArg() - position data");

    err = _CLKernelPointValues.setArg(1, buffer.scalarFieldDataCL);
    _checkError(err, "Kernel::setArg() - scalar field data");

    err = _CLKernelPointValues.setArg(2, buffer.offsetDataCL);
    _checkError(err, "Kernel::setArg() - chunk offset data");

    int localPointDataBytes = numParticles * 4 * sizeof(float);
    assert((unsigned int)localPointDataBytes <= _deviceInfo.cl_device_local_mem_size);

    err = _CLKernelPointValues.setArg(3, cl::__local(localPointDataBytes));
    _checkError(err, "Kernel::setArg() - local position data");

    err = _CLKernelPointValues.setArg(4, numParticles);
    _checkError(err, "Kernel::setArg() - num particles");

    err = _CLKernelPointValues.setArg(5, (float)_radius);
    _checkError(err, "Kernel::setArg() - radius");

    err = _CLKernelPointValues.setArg(6, (float)dx);
    _checkError(err, "Kernel::setArg() - dx");
}

void CLScalarField::_setPointComputationOutputScalarFieldData(std::vector<float> &buffer, 
                                                              std::vector<WorkChunk> &chunks,
                                                              Array3d<WorkGroup> &workGroupGrid) {
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

void CLScalarField::_setPointValueComputationOutputScalarFieldData(std::vector<float> &buffer, 
                                                                   std::vector<WorkChunk> &chunks,
                                                                   Array3d<WorkGroup> &workGroupGrid) {
    _setPointComputationOutputScalarFieldData(buffer, chunks, workGroupGrid);
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

#pragma GCC diagnostic pop