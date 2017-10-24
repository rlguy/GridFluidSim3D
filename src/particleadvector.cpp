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

#include "particleadvector.h"

ParticleAdvector::ParticleAdvector() {
}

bool ParticleAdvector::initialize() {
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
    
    err = _initializeCLKernel();
    if (err != CL_SUCCESS) {
        return false;
    }
    _kernelInfo = _initializeKernelInfo(_CLKernel);

    err = _initializeCLCommandQueue();
    if (err != CL_SUCCESS) {
        return false;
    }

    _isInitialized = true;
    return true;
}

void ParticleAdvector::setDevicePreference(std::string devtype) {
    std::transform(devtype.begin(), devtype.end(), devtype.begin(), ::tolower);

    if (devtype == "gpu") {
        setDevicePreferenceGPU();
    } else if (devtype == "cpu") {
        setDevicePreferenceCPU();
    }
}

void ParticleAdvector::setDevicePreferenceGPU() {
    _devicePreference1 = CL_DEVICE_TYPE_GPU;
    _devicePreference2 = CL_DEVICE_TYPE_CPU;
}

void ParticleAdvector::setDevicePreferenceCPU() {
    _devicePreference1 = CL_DEVICE_TYPE_CPU;
    _devicePreference2 = CL_DEVICE_TYPE_GPU;
}

void ParticleAdvector::printDeviceInfo() {
    if (!_isInitialized) {
        return;
    }

    std::cout << getDeviceInfo();
}

std::string ParticleAdvector::getDeviceInfo() {
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

void ParticleAdvector::printKernelInfo() {
    if (!_isInitialized) {
        return;
    }

    std::cout << getKernelInfo();
}

std::string ParticleAdvector::getKernelInfo() {
    std::ostringstream ss;
    if (!_isInitialized) {
        return ss.str();
    }

    ss << "CL_KERNEL_FUNCTION_NAME:                      " << 
          _kernelInfo.cl_kernel_function_name << std::endl;
    ss << "CL_KERNEL_ATTRIBUTES:                        " << 
          _kernelInfo.cl_kernel_attributes << std::endl;

    ss << "CL_KERNEL_NUM_ARGS:                           " << 
          _kernelInfo.cl_kernel_num_args << std::endl;
    ss << "CL_KERNEL_WORK_GROUP_SIZE:                    " << 
          _kernelInfo.cl_kernel_work_group_size << std::endl;
    ss << "CL_KERNEL_LOCAL_MEM_SIZE:                     " << 
          _kernelInfo.cl_kernel_local_mem_size << std::endl;
    ss << "CL_KERNEL_PRIVATE_MEM_SIZE:                   " << 
          _kernelInfo.cl_kernel_private_mem_size << std::endl;
    ss << "CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE: " << 
          _kernelInfo.cl_kernel_preferred_work_group_size_multiple << std::endl;

    return ss.str();
}

bool ParticleAdvector::isUsingGPU() {
    if (!_isInitialized) {
        return false;
    }
    return _deviceInfo.device_type == CL_DEVICE_TYPE_GPU;
}

bool ParticleAdvector::isUsingCPU() {
    if (!_isInitialized) {
        return false;
    }
    return _deviceInfo.device_type == CL_DEVICE_TYPE_CPU;
}

void ParticleAdvector::disableOpenCL() {
    _isOpenCLEnabled = false;
}

void ParticleAdvector::enableOpenCL() {
    _isOpenCLEnabled = true;
}

bool ParticleAdvector::isOpenCLEnabled() {
    return _isOpenCLEnabled;
}

int ParticleAdvector::getKernelWorkLoadSize() {
    return _kernelWorkLoadSize;
}

void ParticleAdvector::setKernelWorkLoadSize(int n) {
    _kernelWorkLoadSize = n;
}

void ParticleAdvector::advectParticlesRK4(std::vector<vmath::vec3> &particles,
                                          MACVelocityField *vfield, 
                                          double dt,
                                          std::vector<vmath::vec3> &output) {
    if (!_isOpenCLEnabled) {
        _advectParticlesRK4NoCL(particles, vfield, dt, output);
        return;
    }

    FLUIDSIM_ASSERT(_isInitialized);

    /*
        // Classic fourth-order method
        vmath::vec3 RK4(vmath::vec3 p0, double dt) {
            vmath::vec3 k1 = getVelocityAtPosition(p0);
            vmath::vec3 k2 = getVelocityAtPosition(p0 + (float)(0.5*dt)*k1);
            vmath::vec3 k3 = getVelocityAtPosition(p0 + (float)(0.5*dt)*k2);
            vmath::vec3 k4 = getVelocityAtPosition(p0 + (float)dt*k3);
            
            vmath::vec3 p1 = p0 + (float)(dt/6.0f)*(k1 + 2.0f*k2 + 2.0f*k3 + k4);

            return p1;
        }
    */
    // The following code is a vectorized version of the above code.

    output.clear();
    output.reserve(particles.size());
    for (unsigned int i = 0; i < particles.size(); i++) {
        output.push_back(particles[i]);
    }

    std::vector<vmath::vec3> tempdata;
    tempdata.reserve(particles.size());

    tricubicInterpolate(particles, vfield, tempdata);

    float scale = (float)dt / 6.0f;
    vmath::vec3 v;
    for (unsigned int i = 0; i < tempdata.size(); i++) {
        output[i] += scale * tempdata[i];
        tempdata[i] = particles[i] + (float)(0.5*dt) * tempdata[i];
    }

    tricubicInterpolate(tempdata, vfield);

    for (unsigned int i = 0; i < tempdata.size(); i++) {
        output[i] += scale * 2.0f * tempdata[i];
        tempdata[i] = particles[i] + (float)(0.5*dt) * tempdata[i];
    }

    tricubicInterpolate(tempdata, vfield);

    for (unsigned int i = 0; i < tempdata.size(); i++) {
        output[i] += scale * 2.0f * tempdata[i];
        tempdata[i] = particles[i] + (float)dt * tempdata[i];
    }

    tricubicInterpolate(tempdata, vfield);

    for (unsigned int i = 0; i < tempdata.size(); i++) {
        output[i] += scale * tempdata[i];
    }
}

void ParticleAdvector::advectParticlesRK3(std::vector<vmath::vec3> &particles,
                                          MACVelocityField *vfield, 
                                          double dt,
                                          std::vector<vmath::vec3> &output) {
    if (!_isOpenCLEnabled) {
        _advectParticlesRK3NoCL(particles, vfield, dt, output);
        return;
    }

    FLUIDSIM_ASSERT(_isInitialized);

    /*
        // Ralston's third order method (Ralston 62)
        vmath::vec3 RK3(vmath::vec3 p0, double dt) {
            vmath::vec3 k1 = getVelocityAtPosition(p0);
            vmath::vec3 k2 = getVelocityAtPosition(p0 + (float)(0.5*dt)*k1);
            vmath::vec3 k3 = getVelocityAtPosition(p0 + (float)(0.75*dt)*k2);
            vmath::vec3 p1 = p0 + (float)(dt/9.0f)*(2.0f*k1 + 3.0f*k2 + 4.0f*k3);

            return p1;
        }
    */
    // The following code is a vectorized version of the above code.

    output.clear();
    output.reserve(particles.size());
    for (unsigned int i = 0; i < particles.size(); i++) {
        output.push_back(particles[i]);
    }

    std::vector<vmath::vec3> tempdata;
    tempdata.reserve(particles.size());

    tricubicInterpolate(particles, vfield, tempdata);

    float scale = (float)dt / 9.0f;
    vmath::vec3 v;
    for (unsigned int i = 0; i < tempdata.size(); i++) {
        output[i] += scale * 2.0f * tempdata[i];
        tempdata[i] = particles[i] + (float)(0.5*dt) * tempdata[i];
    }

    tricubicInterpolate(tempdata, vfield);

    for (unsigned int i = 0; i < tempdata.size(); i++) {
        output[i] += scale * 3.0f * tempdata[i];
        tempdata[i] = particles[i] + (float)(0.75*dt) * tempdata[i];
    }

    tricubicInterpolate(tempdata, vfield);

    for (unsigned int i = 0; i < tempdata.size(); i++) {
        output[i] += scale * 4.0f * tempdata[i];
    }

}

void ParticleAdvector::advectParticlesRK2(std::vector<vmath::vec3> &particles,
                                          MACVelocityField *vfield, 
                                          double dt,
                                          std::vector<vmath::vec3> &output) {
    if (!_isOpenCLEnabled) {
        _advectParticlesRK2NoCL(particles, vfield, dt, output);
        return;
    }

    FLUIDSIM_ASSERT(_isInitialized);

    /*
        // Midpoint method
        vmath::vec3 RK2(vmath::vec3 p0, double dt) {
            vmath::vec3 k1 = getVelocityAtPosition(p0);
            vmath::vec3 k2 = getVelocityAtPosition(p0 + (float)(0.5*dt)*k1);
            vmath::vec3 p1 = p0 + (float)dt*k2;

            return p1;
        }
    */
    // The following code is a vectorized version of the above code.

    std::vector<vmath::vec3> tempdata;
    tempdata.reserve(particles.size());

    tricubicInterpolate(particles, vfield, tempdata);

    for (unsigned int i = 0; i < tempdata.size(); i++) {
        tempdata[i] = particles[i] + (float)(0.5*dt) * tempdata[i];
    }

    tricubicInterpolate(tempdata, vfield);

    output.clear();
    output.reserve(particles.size());
    for (unsigned int i = 0; i < tempdata.size(); i++) {
        output.push_back(particles[i] + (float)dt * tempdata[i]);
    }

}

void ParticleAdvector::advectParticlesRK1(std::vector<vmath::vec3> &particles,
                                          MACVelocityField *vfield, 
                                          double dt,
                                          std::vector<vmath::vec3> &output) {
    if (!_isOpenCLEnabled) {
        _advectParticlesRK1NoCL(particles, vfield, dt, output);
        return;
    }

    FLUIDSIM_ASSERT(_isInitialized);

    /*  // Forward Euler
        vmath::vec3 RK1(vmath::vec3 p0, double dt) {
            vmath::vec3 k1 = getVelocityAtPosition(p0);
            vmath::vec3 p1 = p0 + (float)dt*k1;

            return p1;
        }
    */
    // The following code is a vectorized version of the above code.

    tricubicInterpolate(particles, vfield, output);

    for (unsigned int i = 0; i < output.size(); i++) {
        output[i] = particles[i] + (float)dt * output[i];
    }
}

void ParticleAdvector::tricubicInterpolate(std::vector<vmath::vec3> &particles,
                                           MACVelocityField *vfield,
                                           std::vector<vmath::vec3> &output) {
    if (!_isOpenCLEnabled) {
        _tricubicInterpolateNoCL(particles, vfield, output);
        return;
    }

    FLUIDSIM_ASSERT(_isInitialized);

    vfield->getGridDimensions(&_isize, &_jsize, &_ksize);
    _dx = vfield->getGridCellSize();

    int chunki = _dataChunkWidth;
    int chunkj = _dataChunkHeight;
    int chunkk = _dataChunkDepth;
    int chunkgridi = ceil((double)_isize / (double)(chunki));
    int chunkgridj = ceil((double)_jsize / (double)(chunkj));
    int chunkgridk = ceil((double)_ksize / (double)(chunkk));

    Array3d<ParticleChunk> particleGrid(chunkgridi, chunkgridj, chunkgridk);

    _getParticleChunkGrid(chunki*_dx, chunkj*_dx, chunkk*_dx, 
                          particles, particleGrid);

    std::vector<DataChunkParameters> chunkParams;
    _getDataChunkParameters(vfield, particleGrid, chunkParams);

    int maxChunks = _getMaxChunksPerComputation();
    int numComputations = ceil((double)chunkParams.size() / (double) maxChunks);

    output.reserve(particles.size());
    for (size_t i = output.size(); i < particles.size(); i++) {
        output.push_back(vmath::vec3());
    }

    std::vector<DataChunkParameters> chunks;
    for (int i = 0; i < numComputations; i++) {
        int begidx = i*maxChunks;
        int endidx = begidx + maxChunks;
        if (endidx > (int)chunkParams.size()) {
            endidx = (int)chunkParams.size();
        }

        std::vector<DataChunkParameters>::iterator beg = chunkParams.begin() + begidx;
        std::vector<DataChunkParameters>::iterator end = chunkParams.begin() + endidx;

        chunks.clear();
        chunks.insert(chunks.begin(), beg, end);

        _tricubicInterpolateChunks(chunks, output);
    }

    _validateOutput(output);
}

void ParticleAdvector::tricubicInterpolate(std::vector<vmath::vec3> &particles,
                                           MACVelocityField *vfield) {
    if (!_isOpenCLEnabled) {
        _tricubicInterpolateNoCL(particles, vfield, particles);
        return;
    }

    tricubicInterpolate(particles, vfield, particles);
}

void ParticleAdvector::_checkError(cl_int err, const char * name) {
    if (err != CL_SUCCESS) {
        std::cerr << "ERROR: " << name  << " (" << err << ")" << std::endl;
        FLUIDSIM_ASSERT(err == CL_SUCCESS);
    }
}

cl::Context ParticleAdvector::_getCLContext(cl_int *err) {
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

cl::Device ParticleAdvector::_getCLDevice(cl::Context &context, cl_int *err) {
    std::vector<cl::Device> devices;
    devices = context.getInfo<CL_CONTEXT_DEVICES>();

    if (devices.size() == 0) {
        *err = -1;
        return cl::Device();
    }

    *err = CL_SUCCESS;
    return devices[0];
}

ParticleAdvector::CLDeviceInfo ParticleAdvector::_initializeDeviceInfo(cl::Device &device) {
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

ParticleAdvector::CLKernelInfo ParticleAdvector::_initializeKernelInfo(cl::Kernel &kernel) {
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

cl_int ParticleAdvector::_initializeCLKernel() {
    std::string prog = Kernels::tricubicinterpolateCL;
    cl::Program::Sources source(1, std::make_pair(prog.c_str(), prog.length()+1));
    cl::Program program(_CLContext, source);

    std::vector<cl::Device> devices = _CLContext.getInfo<CL_CONTEXT_DEVICES>();

    cl_int err = program.build(devices, "");
    if (err != CL_SUCCESS) {
        return err;
    }

    cl::Kernel kernel(program, "tricubic_interpolate_kernel", &err);
    if (err != CL_SUCCESS) {
        return err;
    }

    _CLKernel = kernel;

    return CL_SUCCESS;
}

std::string ParticleAdvector::_getProgramString(std::string filename) {
    std::ifstream file(filename);
    _checkError(file.is_open() ? CL_SUCCESS : -1, filename.c_str());
    
    std::string prog(std::istreambuf_iterator<char>(file), (std::istreambuf_iterator<char>()));

    return prog;
}

cl_int ParticleAdvector::_initializeCLCommandQueue() {
    cl_int err;
    cl::CommandQueue queue(_CLContext, _CLDevice, 0, &err);
    if (err != CL_SUCCESS) {
        return err;
    }

    _CLQueue = queue;

    return CL_SUCCESS;
}

void ParticleAdvector::_getParticleChunkGrid(double cwidth, double cheight, double cdepth,
                                             std::vector<vmath::vec3> &particles,
                                             Array3d<ParticleChunk> &grid) {

    double bwidth = grid.width * cwidth;
    double bheight = grid.height * cheight;
    double bdepth = grid.depth * cdepth;

    double eps = 1e-6;
    double bboxeps = 0.01 * _dx;

    // The grid boundary dimensions are reduced to keep particles away from
    // the edge. Numerical error may cause particle locations to be calculated
    // to be outside of the grid if they lie on the grid boundary.
    AABB bbox(vmath::vec3(0.0, 0.0, 0.0), bwidth, bheight, bdepth);
    bbox.expand(-bboxeps);

    Array3d<int> countGrid(grid.width, grid.height, grid.depth, 0);
    vmath::vec3 p;
    for (unsigned int i = 0; i < particles.size(); i++) {
        p = particles[i];

        if (!bbox.isPointInside(p)) {
            p = bbox.getNearestPointInsideAABB(p, eps);
        }

        int pi = (int)(p.x / cwidth);
        int pj = (int)(p.y / cheight);
        int pk = (int)(p.z / cdepth);

        FLUIDSIM_ASSERT(Grid3d::isGridIndexInRange(pi, pj, pk, grid.width, grid.height, grid.depth));

        countGrid.add(pi, pj, pk, 1);
    }

    ParticleChunk *pc;
    for (int k = 0; k < grid.depth; k++) {
        for (int j = 0; j < grid.height; j++) {
            for (int i = 0; i < grid.width; i++) {
                pc = grid.getPointer(i, j, k);
                pc->particles.reserve(countGrid(i, j, k));
            }
        }
    }

    for (int k = 0; k < grid.depth; k++) {
        for (int j = 0; j < grid.height; j++) {
            for (int i = 0; i < grid.width; i++) {
                pc = grid.getPointer(i, j, k);
                pc->references.reserve(countGrid(i, j, k));
            }
        }
    }

    for (unsigned int i = 0; i < particles.size(); i++) {
        p = particles[i];

        if (!bbox.isPointInside(p)) {
            p = bbox.getNearestPointInsideAABB(p, eps);
        }

        int pi = (int)(p.x / cwidth);
        int pj = (int)(p.y / cheight);
        int pk = (int)(p.z / cdepth);

        pc = grid.getPointer(pi, pj, pk);
        pc->particles.push_back(p);
        pc->references.push_back(i);
    }

    /*
       Move particles away from the boundaries of a chunk. Due to reduced
       precision by using float32 in the OpenCL kernel, if a particle is very 
       close to the boundary of a chunk, its location could be calculated to be 
       in a different chunk from what is calculated in this method.
    */
    for (int k = 0; k < grid.depth; k++) {
        for (int j = 0; j < grid.height; j++) {
            for (int i = 0; i < grid.width; i++) {
                bbox = AABB(i*cwidth, j*cheight, k*cdepth, cwidth, cheight, cdepth);
                bbox.expand(-bboxeps);

                pc = grid.getPointer(i, j, k);
                for (unsigned int pidx = 0; pidx < pc->particles.size(); pidx++) {
                    p = pc->particles[pidx];
                    if (!bbox.isPointInside(p)) {
                        p = bbox.getNearestPointInsideAABB(p, eps);
                        pc->particles[pidx] = p;
                    }
                }
            }
        }
    }
}

void ParticleAdvector::_getDataChunkParametersForChunkIndex(GridIndex cindex,
                                                            MACVelocityField *vfield,
                                                            ParticleChunk *particleChunk,
                                                            std::vector<DataChunkParameters> &chunkParameters) {

    if (particleChunk->particles.size() == 0) {
        return;
    }

    GridIndex indexOffset = GridIndex(cindex.i*_dataChunkWidth, 
                                      cindex.j*_dataChunkHeight, 
                                      cindex.k*_dataChunkDepth);

    double dx = vfield->getGridCellSize();
    vmath::vec3 positionOffset = Grid3d::GridIndexToPosition(indexOffset.i,
                                                             indexOffset.j,
                                                             indexOffset.k, dx);

    Array3d<float> *ugrid = vfield->getArray3dU();
    Array3d<float> *vgrid = vfield->getArray3dV();
    Array3d<float> *wgrid = vfield->getArray3dW();

    GridIndex ugridOffset(indexOffset.i - 1, indexOffset.j - 2, indexOffset.k - 2);
    GridIndex vgridOffset(indexOffset.i - 2, indexOffset.j - 1, indexOffset.k - 2);
    GridIndex wgridOffset(indexOffset.i - 2, indexOffset.j - 2, indexOffset.k - 1);

    ArrayView3d<float> ugridview(_dataChunkWidth + 3, _dataChunkHeight + 4, _dataChunkDepth + 4,
                                 ugridOffset, ugrid);
    ArrayView3d<float> vgridview(_dataChunkWidth + 4, _dataChunkHeight + 3, _dataChunkDepth + 4,
                                 vgridOffset, vgrid);
    ArrayView3d<float> wgridview(_dataChunkWidth + 4, _dataChunkHeight + 4, _dataChunkDepth + 3,
                                 wgridOffset, wgrid);

    int groupSize = _getWorkGroupSize(_deviceInfo);
    int numDataChunks = ceil((double)particleChunk->particles.size() / (double)groupSize);

    for (int i = 0; i < numDataChunks; i++) {
        DataChunkParameters params;

        int begidx = i*groupSize;
        int endidx = begidx + groupSize;
        if (endidx > (int)particleChunk->particles.size()) {
            endidx = (int)particleChunk->particles.size();
        }

        params.particlesBegin = particleChunk->particles.begin() + begidx;
        params.referencesBegin = particleChunk->references.begin() + begidx;

        params.particlesEnd = particleChunk->particles.begin() + endidx;
        params.referencesEnd = particleChunk->references.begin() + endidx;

        params.ufieldview = ugridview;
        params.vfieldview = vgridview;
        params.wfieldview = wgridview;

        params.chunkOffset = cindex;
        params.indexOffset = indexOffset;
        params.positionOffset = positionOffset;

        chunkParameters.push_back(params);
    }
}

void ParticleAdvector::_getDataChunkParameters(MACVelocityField *vfield,
                                               Array3d<ParticleChunk> &particleGrid,
                                               std::vector<DataChunkParameters> &chunkParameters) {

    GridIndex cindex;
    ParticleChunk *pc;
    for (int k = 0; k < particleGrid.depth; k++) {
        for (int j = 0; j < particleGrid.height; j++) {
            for (int i = 0; i < particleGrid.width; i++) {
                cindex = GridIndex(i, j, k);
                pc = particleGrid.getPointer(cindex);
                _getDataChunkParametersForChunkIndex(cindex, vfield, pc, chunkParameters);
            }
        }
    }
}

int ParticleAdvector::_getWorkGroupSize(CLDeviceInfo &info) {
    return fmin(info.cl_device_max_work_group_size, _maxItemsPerWorkGroup);
}

int ParticleAdvector::_getChunkPositionDataSize() {
    // <x, y, z> position data size
    return 3*sizeof(float)*_getWorkGroupSize(_deviceInfo);
}

int ParticleAdvector::_getChunkVelocityDataSize() {
    // u, v, w Velocity field data size
    int cw = _dataChunkWidth;
    int ch = _dataChunkHeight;
    int cd = _dataChunkDepth;

    int usize = sizeof(float)*(cw + 3)*(ch + 4)*(cd + 4);
    int vsize = sizeof(float)*(cw + 4)*(ch + 3)*(cd + 4);
    int wsize = sizeof(float)*(cw + 4)*(ch + 4)*(cd + 3);

    return usize + vsize + wsize;
}

int ParticleAdvector::_getChunkOffsetDataSize() {
    // <i, j, k> chunk index offset data size
    return 3*sizeof(int);
}

int ParticleAdvector::_getChunkTotalDataSize() {
    return _getChunkPositionDataSize() + 
           _getChunkVelocityDataSize() + 
           _getChunkOffsetDataSize();
}

int ParticleAdvector::_getMaxChunksPerComputation() {
    int positionSize = _getChunkPositionDataSize();
    int vfieldSize = _getChunkVelocityDataSize();
    int offsetSize = _getChunkOffsetDataSize();
    int totalSize = _getChunkTotalDataSize();

    cl_ulong maxGlobalMem = _deviceInfo.cl_device_global_mem_size;
    cl_ulong maxAlloc = _deviceInfo.cl_device_max_mem_alloc_size;

    int numPositionAllocItems = floor((double)maxAlloc / (double)positionSize);
    int numVelocityAllocItems = floor((double)maxAlloc / (double)vfieldSize);
    int numOffsetAllocItems = floor((double)maxAlloc / (double)offsetSize);

    int allocLimitCount = fmin(fmin(numPositionAllocItems, 
                                    numVelocityAllocItems),
                                    numOffsetAllocItems);

    int globalMemLimitCount = floor((double)maxGlobalMem / (double)totalSize);

    int hardwareLimit = fmin(allocLimitCount, globalMemLimitCount);
    int softwareLimit = _maxChunksPerComputation;

    return fmin(hardwareLimit, softwareLimit);
}

void ParticleAdvector::_tricubicInterpolateChunks(std::vector<DataChunkParameters> &chunks,
                                                  std::vector<vmath::vec3> &output) {
    DataBuffer buffer;
    _initializeDataBuffer(chunks, buffer);
    _setCLKernelArgs(buffer, _dx);

    int loadSize = _kernelWorkLoadSize;
    int workGroupSize = _getWorkGroupSize(_deviceInfo);
    int numWorkItems = (int)chunks.size()*workGroupSize;
    int numComputations = ceil((double)chunks.size() / (double)loadSize);

    cl::Event event;
    cl_int err;
    for (int i = 0; i < numComputations; i++) {
        int offset = i * loadSize * workGroupSize;
        int items = (int)fmin(numWorkItems - offset, loadSize * workGroupSize);
        
        err = _CLQueue.enqueueNDRangeKernel(_CLKernel, 
                                            cl::NDRange(offset), 
                                            cl::NDRange(items), 
                                            cl::NDRange(workGroupSize), 
                                            NULL, 
                                            &event);    
        _checkError(err, "CommandQueue::enqueueNDRangeKernel()");
    }

    event.wait();

    int dataSize = (int)chunks.size() * _getChunkPositionDataSize();
    err = _CLQueue.enqueueReadBuffer(buffer.positionDataCL, 
                                     CL_TRUE, 0, 
                                     dataSize, 
                                     (void*)&(buffer.positionDataH[0]));
    _checkError(err, "CommandQueue::enqueueReadBuffer()");

    _setOutputData(chunks, buffer, output);
}

void ParticleAdvector::_initializeDataBuffer(std::vector<DataChunkParameters> &chunks,
                                             DataBuffer &buffer) {

    _getHostPositionDataBuffer(chunks, buffer.positionDataH);
    _getHostVelocityDataBuffer(chunks, buffer.vfieldDataH);
    _getHostChunkOffsetDataBuffer(chunks, buffer.offsetDataH);

    size_t positionDataBytes = buffer.positionDataH.size()*sizeof(vmath::vec3);
    size_t vfieldDataBytes = buffer.vfieldDataH.size()*sizeof(float);
    size_t offsetDataBytes = buffer.offsetDataH.size()*sizeof(GridIndex);

    cl_int err;
    buffer.positionDataCL = cl::Buffer(_CLContext, 
                                       CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, 
                                       positionDataBytes, 
                                       (void*)&(buffer.positionDataH[0]), 
                                       (cl_int*)&err);
    _checkError(err, "Creating position data buffer");

    buffer.vfieldDataCL = cl::Buffer(_CLContext, 
                                      CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, 
                                      vfieldDataBytes, 
                                      (void*)&(buffer.vfieldDataH[0]), 
                                      &err);
    _checkError(err, "Creating velocity field data buffer");

    buffer.offsetDataCL = cl::Buffer(_CLContext, 
                                     CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, 
                                     offsetDataBytes, 
                                     (void*)&(buffer.offsetDataH[0]), 
                                     &err);
    _checkError(err, "Creating chunk offset data buffer");
}

void ParticleAdvector::_getHostPositionDataBuffer(std::vector<DataChunkParameters> &chunks,
                                                  std::vector<vmath::vec3> &buffer) {

    int groupSize = _getWorkGroupSize(_deviceInfo);
    int numElements = (int)chunks.size()*groupSize;
    buffer.reserve(numElements);

    DataChunkParameters c;
    vmath::vec3 p;
    for (unsigned int i = 0; i < chunks.size(); i++) {
        c = chunks[i];

        int numPoints = c.particlesEnd - c.particlesBegin;
        int numPad = groupSize - numPoints;
        vmath::vec3 defaultPosition = c.positionOffset;

        buffer.insert(buffer.end(), c.particlesBegin, c.particlesEnd);
        for (int i = 0; i < numPad; i++) {
            buffer.push_back(defaultPosition);
        }
    }
}

void ParticleAdvector::_getHostVelocityDataBuffer(std::vector<DataChunkParameters> &chunks,
                                                  std::vector<float> &buffer) {

    int numElements = _getChunkVelocityDataSize() / sizeof(float);
    buffer.reserve(numElements);
    for (unsigned int i = 0; i < chunks.size(); i++) {
        _appendChunkVelocityDataToBuffer(chunks[i], buffer);
    }
}

void ParticleAdvector::_appendChunkVelocityDataToBuffer(DataChunkParameters &chunk, 
                                                        std::vector<float> &buffer) {

    for (int k = 0; k < chunk.ufieldview.depth; k++) {
        for (int j = 0; j < chunk.ufieldview.height; j++) {
            for (int i = 0; i < chunk.ufieldview.width; i++) {
                buffer.push_back(chunk.ufieldview(i, j, k));
            }
        }
    }

    for (int k = 0; k < chunk.vfieldview.depth; k++) {
        for (int j = 0; j < chunk.vfieldview.height; j++) {
            for (int i = 0; i < chunk.vfieldview.width; i++) {
                buffer.push_back(chunk.vfieldview(i, j, k));
            }
        }
    }

    for (int k = 0; k < chunk.wfieldview.depth; k++) {
        for (int j = 0; j < chunk.wfieldview.height; j++) {
            for (int i = 0; i < chunk.wfieldview.width; i++) {
                buffer.push_back(chunk.wfieldview(i, j, k));
            }
        }
    }
}

void ParticleAdvector::_getHostChunkOffsetDataBuffer(std::vector<DataChunkParameters> &chunks,
                                                     std::vector<GridIndex> &buffer) {
    buffer.reserve(chunks.size());
    for (unsigned int i = 0; i < chunks.size(); i++) {
        buffer.push_back(chunks[i].chunkOffset);
    }
}

void ParticleAdvector::_setCLKernelArgs(DataBuffer &buffer, double dx) {
    cl_int err = _CLKernel.setArg(0, buffer.positionDataCL);
    _checkError(err, "Kernel::setArg() - position data");

    err = _CLKernel.setArg(1, buffer.vfieldDataCL);
    _checkError(err, "Kernel::setArg() - velocity field data");

    err = _CLKernel.setArg(2, buffer.offsetDataCL);
    _checkError(err, "Kernel::setArg() - chunk offset data");

    int vfieldLocalBytes = _getChunkVelocityDataSize();
    FLUIDSIM_ASSERT((unsigned int)vfieldLocalBytes <= _deviceInfo.cl_device_local_mem_size);

    err = _CLKernel.setArg(3, cl::__local(vfieldLocalBytes));
    _checkError(err, "Kernel::setArg() - local vfield data");

    err = _CLKernel.setArg(4, (float)dx);
    _checkError(err, "Kernel::setArg() - dx");
}

void ParticleAdvector::_setOutputData(std::vector<DataChunkParameters> &chunks,
                                      DataBuffer &buffer,
                                      std::vector<vmath::vec3> &output) {

    int workGroupSize = _getWorkGroupSize(_deviceInfo);

    DataChunkParameters chunk;
    for (unsigned int gidx = 0; gidx < chunks.size(); gidx++) {
        int hostOffset = gidx*workGroupSize;
        int dataOffset = 0;

        chunk = chunks[gidx];
        std::vector<int>::iterator begin = chunk.referencesBegin;
        std::vector<int>::iterator end = chunk.referencesEnd;
        for (std::vector<int>::iterator it = begin; it != end; ++it) {
            output[*it] = buffer.positionDataH[hostOffset + dataOffset];
            dataOffset++;
        }
    }
}

vmath::vec3 ParticleAdvector::_RK4(vmath::vec3 p0, double dt, MACVelocityField *vfield) {
    vmath::vec3 k1 = vfield->evaluateVelocityAtPosition(p0);
    vmath::vec3 k2 = vfield->evaluateVelocityAtPosition(p0 + (float)(0.5*dt)*k1);
    vmath::vec3 k3 = vfield->evaluateVelocityAtPosition(p0 + (float)(0.5*dt)*k2);
    vmath::vec3 k4 = vfield->evaluateVelocityAtPosition(p0 + (float)dt*k3);
    
    vmath::vec3 p1 = p0 + (float)(dt/6.0f)*(k1 + 2.0f*k2 + 2.0f*k3 + k4);

    return p1;
}

vmath::vec3 ParticleAdvector::_RK3(vmath::vec3 p0, double dt, MACVelocityField *vfield) {
    vmath::vec3 k1 = vfield->evaluateVelocityAtPosition(p0);
    vmath::vec3 k2 = vfield->evaluateVelocityAtPosition(p0 + (float)(0.5*dt)*k1);
    vmath::vec3 k3 = vfield->evaluateVelocityAtPosition(p0 + (float)(0.75*dt)*k2);
    vmath::vec3 p1 = p0 + (float)(dt/9.0f)*(2.0f*k1 + 3.0f*k2 + 4.0f*k3);

    return p1;
}

vmath::vec3 ParticleAdvector::_RK2(vmath::vec3 p0, double dt, MACVelocityField *vfield) {
    vmath::vec3 k1 = vfield->evaluateVelocityAtPosition(p0);
    vmath::vec3 k2 = vfield->evaluateVelocityAtPosition(p0 + (float)(0.5*dt)*k1);
    vmath::vec3 p1 = p0 + (float)dt*k2;

    return p1;
}

vmath::vec3 ParticleAdvector::_RK1(vmath::vec3 p0, double dt, MACVelocityField *vfield) {
    vmath::vec3 k1 = vfield->evaluateVelocityAtPosition(p0);
    vmath::vec3 p1 = p0 + (float)dt*k1;

    return p1;
}

void ParticleAdvector::_advectParticlesRK4NoCL(std::vector<vmath::vec3> &particles,
                             MACVelocityField *vfield, 
                             double dt,
                             std::vector<vmath::vec3> &output) {
    output.clear();
    output.reserve(particles.size());
    for (size_t i = 0; i < particles.size(); i++) {
        output.push_back(_RK4(particles[i], dt, vfield));
    }
}

void ParticleAdvector::_advectParticlesRK3NoCL(std::vector<vmath::vec3> &particles,
                             MACVelocityField *vfield, 
                             double dt,
                             std::vector<vmath::vec3> &output) {
    output.clear();
    output.reserve(particles.size());
    for (size_t i = 0; i < particles.size(); i++) {
        output.push_back(_RK3(particles[i], dt, vfield));
    }
}

void ParticleAdvector::_advectParticlesRK2NoCL(std::vector<vmath::vec3> &particles,
                             MACVelocityField *vfield, 
                             double dt,
                             std::vector<vmath::vec3> &output) {
    output.clear();
    output.reserve(particles.size());
    for (size_t i = 0; i < particles.size(); i++) {
        output.push_back(_RK2(particles[i], dt, vfield));
    }
}

void ParticleAdvector::_advectParticlesRK1NoCL(std::vector<vmath::vec3> &particles,
                             MACVelocityField *vfield, 
                             double dt,
                             std::vector<vmath::vec3> &output) {
    output.clear();
    output.reserve(particles.size());
    for (size_t i = 0; i < particles.size(); i++) {
        output.push_back(_RK1(particles[i], dt, vfield));
    }
}

void ParticleAdvector::_tricubicInterpolateNoCL(std::vector<vmath::vec3> &particles,
                                                MACVelocityField *vfield, 
                                                std::vector<vmath::vec3> &output) {
    output.reserve(particles.size());
    for (size_t i = output.size(); i < particles.size(); i++) {
        output.push_back(vmath::vec3());
    }

    for (size_t i = 0; i < particles.size(); i++) {
        output[i] = vfield->evaluateVelocityAtPosition(particles[i]);
    }

    _validateOutput(output);
}

void ParticleAdvector::_validateOutput(std::vector<vmath::vec3> &output) {
    vmath::vec3 v;
    for (unsigned int i = 0; i < output.size(); i++) {
        v = output[i];
        if (std::isinf(v.x) || std::isnan(v.x) || 
                std::isinf(v.y) || std::isnan(v.y) ||
                std::isinf(v.z) || std::isnan(v.z)) {
            output[i] = vmath::vec3(0.0, 0.0, 0.0);
        }
    }
}

#ifdef __GNUC__
    #pragma GCC diagnostic pop
#endif