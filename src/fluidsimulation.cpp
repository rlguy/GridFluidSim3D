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
#include "fluidsimulation.h"


FluidSimulation::FluidSimulation() {
    _initializeLogFile();
}

FluidSimulation::FluidSimulation(int isize, int jsize, int ksize, double dx) :
                                _isize(isize), _jsize(jsize), _ksize(ksize), _dx(dx)
{
    _initializeLogFile();
    _initializeSimulationGrids(_isize, _jsize, _ksize, _dx);
    _initializeSimulationVectors(_isize, _jsize, _ksize);
}

FluidSimulation::FluidSimulation(FluidSimulationSaveState &state) {
    _initializeLogFile();
    FLUIDSIM_ASSERT(state.isLoadStateInitialized());
    _initializeSimulationFromSaveState(state);
}

FluidSimulation::~FluidSimulation() {
}

/*******************************************************************************
    PUBLIC
********************************************************************************/

void FluidSimulation::initialize() {
    if (!_isSimulationInitialized) {
        _logfile.log(std::ostringstream().flush() << 
                     _logfile.getTime() << " initialize" << std::endl);
        _initializeSimulation();
    }
}

bool FluidSimulation::isInitialized() {
    return _isSimulationInitialized;
}

void FluidSimulation::saveState(std::string filename) {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " saveState: " << filename << std::endl);
    FluidSimulationSaveState state;
    state.saveState(filename, this);
}

int FluidSimulation::getCurrentFrame() {
    return _currentFrame;
}

bool FluidSimulation::isCurrentFrameFinished() { 
    return _isCurrentFrameFinished; 
}

double FluidSimulation::getCellSize() { 
    return _dx; 
}

void FluidSimulation::getGridDimensions(int *i, int *j, int *k) { 
    *i = _isize; *j = _jsize; *k = _ksize; 
}

int FluidSimulation::getGridWidth() {
    return _isize;
}

int FluidSimulation::getGridHeight() {
    return _jsize;
}

int FluidSimulation::getGridDepth() {
    return _ksize;
}

void FluidSimulation::getSimulationDimensions(double *w, double *h, double *d) { 
    *w = (double)_isize*_dx;
    *h = (double)_jsize*_dx;
    *d = (double)_ksize*_dx;
}

double FluidSimulation::getSimulationWidth() {  
    return (double)_isize*_dx; 
}

double FluidSimulation::getSimulationHeight() { 
    return (double)_jsize*_dx; 
}

double FluidSimulation::getSimulationDepth() {  
    return (double)_ksize*_dx; 
}

double FluidSimulation::getDensity() { 
    return _density; 
}

void FluidSimulation::setDensity(double p) { 
    if (p <= 0.0) {
        std::string msg = "Error: density must be greater than 0.\n";
        msg += "density: " + _toString(p) + "\n";
        throw std::domain_error(msg);
    }

    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " setDensity: " << p << std::endl);

    _density = p; 
}

Material FluidSimulation::getMaterial(int i, int j, int k) { 
    if (!Grid3d::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize)) {
        std::string msg = "Error: material index out of range.\n";
        msg += "i: " + _toString(i) + " j: " + _toString(j) + " k: " + _toString(k) + "\n";
        throw std::out_of_range(msg);
    }

    return _materialGrid(i, j, k); 
}

Material FluidSimulation::getMaterial(GridIndex g) { 
    return getMaterial(g.i, g.j, g.k); 
}

void FluidSimulation::setMarkerParticleScale(double s) { 
    if (s < 0.0) {
        std::string msg = "Error: marker particle scale must be greater than or equal to 0.\n";
        msg += "scale: " + _toString(s) + "\n";
        throw std::domain_error(msg);
    }

    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " setMarkerParticleScale: " << s << std::endl);

    _markerParticleScale = s; 
}

double FluidSimulation::getMarkerParticleScale() {
    return _markerParticleScale;
}

int FluidSimulation::getSurfaceSubdivisionLevel() {
    return _outputFluidSurfaceSubdivisionLevel;
}

void FluidSimulation::setSurfaceSubdivisionLevel(int n) {
    if (n < 1) {
        std::string msg = "Error: subdivision level must be greater than or equal to 1.\n";
        msg += "subdivision level: " + _toString(n) + "\n";
        throw std::domain_error(msg);
    }

    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " setSurfaceSubdivisionLevel: " << n << std::endl);

    _outputFluidSurfaceSubdivisionLevel = n;
}

int FluidSimulation::getNumPolygonizerSlices() {
    return _numSurfaceReconstructionPolygonizerSlices;
}

void FluidSimulation::setNumPolygonizerSlices(int n) {
    if (n < 1) {
        std::string msg = "Error: number of polygonizer slices must be greater than or equal to 1.\n";
        msg += "polygonizer slices: " + _toString(n) + "\n";
        throw std::domain_error(msg);
    }

    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " setNumPolygonizerSlices: " << n << std::endl);

    _numSurfaceReconstructionPolygonizerSlices = n;
}

int FluidSimulation::getMinPolyhedronTriangleCount() {
    return _minimumSurfacePolyhedronTriangleCount;
}

void FluidSimulation::setMinPolyhedronTriangleCount(int n) {
    if (n < 0) {
        std::string msg = "Error: minimum polyhedron triangle count must be greater than or equal to 0.\n";
        msg += "triangle count: " + _toString(n) + "\n";
        throw std::domain_error(msg);
    }

    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " setMinPolyhedronTriangleCount: " << n << std::endl);

    _minimumSurfacePolyhedronTriangleCount = n;
}

void FluidSimulation::setDomainOffset(double x, double y, double z) {
    setDomainOffset(vmath::vec3(x, y, z));
}

void FluidSimulation::setDomainOffset(vmath::vec3 offset) {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " setDomainOffset: " << 
                 offset.x << " " << offset.y << " " << offset.z << std::endl);

    _domainOffset = offset;
}

vmath::vec3 FluidSimulation::getDomainOffset() {
    return _domainOffset;
}

void FluidSimulation::setMeshOutputFormatAsPLY() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " setMeshOutputFormatAsPLY" << std::endl);

    _meshOutputFormat = TriangleMeshFormat::ply;
}

void FluidSimulation::setMeshOutputFormatAsBOBJ() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " setMeshOutputFormatAsBOBJ" << std::endl);

    _meshOutputFormat = TriangleMeshFormat::bobj;
}

void FluidSimulation::enableSurfaceMeshOutput() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " enableSurfaceMeshOutput" << std::endl);

    _isSurfaceMeshOutputEnabled = true;
}

void FluidSimulation::disableSurfaceMeshOutput() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " disableSurfaceMeshOutput" << std::endl);

    _isSurfaceMeshOutputEnabled = false;
}

bool FluidSimulation::isSurfaceMeshOutputEnabled() {
    return _isSurfaceMeshOutputEnabled;
}

void FluidSimulation::enableIsotropicSurfaceReconstruction() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " enableIsotropicSurfaceReconstruction" << std::endl);

    _isIsotropicSurfaceMeshReconstructionEnabled = true;
}

void FluidSimulation::disableIsotropicSurfaceReconstruction() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " disableIsotropicSurfaceReconstruction" << std::endl);

    _isIsotropicSurfaceMeshReconstructionEnabled = false;
}

bool FluidSimulation::isIsotropicSurfaceReconstructionEnabled() {
    return _isIsotropicSurfaceMeshReconstructionEnabled;
}

void FluidSimulation::enablePreviewMeshOutput(double cellsize) {
    if (cellsize <= 0.0) {
        std::string msg = "Error: cell size must be greater than 0.0.\n";
        msg += "cellsize: " + _toString(cellsize) + "\n";
        throw std::domain_error(msg);
    }

     _logfile.log(std::ostringstream().flush() << 
             _logfile.getTime() << " enablePreviewMeshOutput: " << cellsize << std::endl);

    _isPreviewSurfaceMeshEnabled = true;
    _previewdx = cellsize;
}

void FluidSimulation::disablePreviewMeshOutput() {
     _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " disablePreviewMeshOutput" << std::endl);

    _isPreviewSurfaceMeshEnabled = false;
}

bool FluidSimulation::isPreviewMeshOutputEnabled() {
    return _isPreviewSurfaceMeshEnabled;
}

void FluidSimulation::enableAnisotropicSurfaceReconstruction() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " enableAnisotropicSurfaceReconstruction" << std::endl);

    _isAnisotropicSurfaceMeshReconstructionEnabled = true;
}

void FluidSimulation::disableAnisotropicSurfaceReconstruction() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " disableAnisotropicSurfaceReconstruction" << std::endl);

    _isAnisotropicSurfaceMeshReconstructionEnabled = false;
}

bool FluidSimulation::isAnisotropicSurfaceReconstructionEnabled() {
    return _isAnisotropicSurfaceMeshReconstructionEnabled;
}

void FluidSimulation::enableDiffuseMaterialOutput() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " enableDiffuseMaterialOutput" << std::endl);

    _isDiffuseMaterialOutputEnabled = true;
}

void FluidSimulation::disableDiffuseMaterialOutput() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " disableDiffuseMaterialOutput" << std::endl);

    _isDiffuseMaterialOutputEnabled = false;
}

bool FluidSimulation::isDiffuseMaterialOutputEnabled() {
    return _isDiffuseMaterialOutputEnabled;
}

void FluidSimulation::enableBubbleDiffuseMaterial() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " enableBubbleDiffuseMaterial" << std::endl);

    _isBubbleDiffuseMaterialEnabled = true;
    _isDiffuseMaterialOutputEnabled = true;
}

void FluidSimulation::enableSprayDiffuseMaterial() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " enableSprayDiffuseMaterial" << std::endl);

    _isSprayDiffuseMaterialEnabled = true;
    _isDiffuseMaterialOutputEnabled = true;
}

void FluidSimulation::enableFoamDiffuseMaterial() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " enableFoamDiffuseMaterial" << std::endl);

    _isFoamDiffuseMaterialEnabled = true;
    _isDiffuseMaterialOutputEnabled = true;
}

void FluidSimulation::disableBubbleDiffuseMaterial() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " disableBubbleDiffuseMaterial" << std::endl);

    _isBubbleDiffuseMaterialEnabled = false;
}

void FluidSimulation::disableSprayDiffuseMaterial() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " disableSprayDiffuseMaterial" << std::endl);

    _isSprayDiffuseMaterialEnabled = false;
}

void FluidSimulation::disableFoamDiffuseMaterial() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " disableFoamDiffuseMaterial" << std::endl);

    _isFoamDiffuseMaterialEnabled = false;
}

bool FluidSimulation::isBubbleDiffuseMaterialEnabled() {
    return _isBubbleDiffuseMaterialEnabled;
}

bool FluidSimulation::isSprayDiffuseMaterialEnabled() {
    return _isSprayDiffuseMaterialEnabled;
}

bool FluidSimulation::isFoamDiffuseMaterialEnabled() {
    return _isFoamDiffuseMaterialEnabled;
}

void FluidSimulation::outputDiffuseMaterialAsSeparateFiles() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " outputDiffuseMaterialAsSeparateFiles" << std::endl);

    _isDiffuseMaterialOutputEnabled = true;
    _isDiffuseMaterialFilesSeparated = true;
}

void FluidSimulation::outputDiffuseMaterialAsSingleFile() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " outputDiffuseMaterialAsSingleFile" << std::endl);

    _isDiffuseMaterialOutputEnabled = true;
    _isDiffuseMaterialFilesSeparated = false;
}

int FluidSimulation::getMaxNumDiffuseParticles() {
    return _diffuseMaterial.getMaxNumDiffuseParticles();
}

void FluidSimulation::setMaxNumDiffuseParticles(int n) {
    if (n < 0) {
        std::string msg = "Error: maxNumDiffuseParticles must be greater than or equal to 0.\n";
        msg += "n: " + _toString(n) + "\n";
        throw std::domain_error(msg);
    }

    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " setMaxNumDiffuseParticles: " << n << std::endl);

    _diffuseMaterial.setMaxNumDiffuseParticles(n);
}

double FluidSimulation::getMaxDiffuseParticleLifetime() {
    return _diffuseMaterial.getMaxDiffuseParticleLifetime();
}   

void FluidSimulation::setMaxDiffuseParticleLifetime(double lifetime) {
    if (lifetime < 0) {
        std::string msg = "Error: lifetime must be greater than 0.\n";
        msg += "lifetime: " + _toString(lifetime) + "\n";
        throw std::domain_error(msg);
    }

    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " setMaxDiffuseParticleLifetime: " << lifetime << std::endl);

    _diffuseMaterial.setMaxDiffuseParticleLifetime(lifetime);
}

double FluidSimulation::getDiffuseParticleWavecrestEmissionRate() {
    return _diffuseMaterial.getDiffuseParticleWavecrestEmissionRate();
}

void FluidSimulation::setDiffuseParticleWavecrestEmissionRate(double r) {
    if (r < 0) {
        std::string msg = "Error: wavecrest emission rate must be greater than or equal to 0.\n";
        msg += "rate: " + _toString(r) + "\n";
        throw std::domain_error(msg);
    }

    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " setDiffuseParticleWavecrestEmissionRate: " << r << std::endl);

    _diffuseMaterial.setDiffuseParticleWavecrestEmissionRate(r);
}

double FluidSimulation::getDiffuseParticleTurbulenceEmissionRate() {
    return _diffuseMaterial.getDiffuseParticleTurbulenceEmissionRate();
}

void FluidSimulation::setDiffuseParticleTurbulenceEmissionRate(double r) {
    if (r < 0) {
        std::string msg = "Error: turbulence emission rate must be greater than or equal to 0.\n";
        msg += "rate: " + _toString(r) + "\n";
        throw std::domain_error(msg);
    }

    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " setDiffuseParticleTurbulenceEmissionRate: " << r << std::endl);

    _diffuseMaterial.setDiffuseParticleTurbulenceEmissionRate(r);
}

void FluidSimulation::getDiffuseParticleEmissionRates(double *rwc, 
                                                      double *rt) {
    _diffuseMaterial.getDiffuseParticleEmissionRates(rwc, rt);
}

void FluidSimulation::setDiffuseParticleEmissionRates(double r) {
    if (r < 0) {
        std::string msg = "Error: emission rate must be greater than or equal to 0.\n";
        msg += "rate: " + _toString(r) + "\n";
        throw std::domain_error(msg);
    }

    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " setDiffuseParticleEmissionRates: " << r << std::endl);

    _diffuseMaterial.setDiffuseParticleEmissionRates(r);
}

void FluidSimulation::setDiffuseParticleEmissionRates(double rwc, 
                                                      double rt) {
    if (rwc < 0 || rt < 0) {
        std::string msg = "Error: emission rates must be greater than or equal to 0.\n";
        msg += "wavecrest emission rate: " + _toString(rwc) + "\n";
        msg += "turbulence emission rate: " + _toString(rt) + "\n";
        throw std::domain_error(msg);
    }

    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " setDiffuseParticleEmissionRates: " << 
                 rwc << " " << rt << std::endl);

    _diffuseMaterial.setDiffuseParticleEmissionRates(rwc, rt);
}

void FluidSimulation::enableBrickOutput(double width, double height, double depth) {
    if (!(width > 0.0 && height > 0.0 && depth > 0.0)) {
        std::string msg = "Error: brick dimensions must be greater than 0.\n";
        msg += "width: " + _toString(width) + 
               " height: " + _toString(height) + 
               " depth: " + _toString(depth) + "\n";
        throw std::domain_error(msg);
    }

    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " enableBrickOutput: " << 
                 width << " " << height << " " << depth << std::endl);

    AABB brick = AABB(vmath::vec3(), width, height, depth);

    int i, j, k;
    _fluidBrickGrid.getGridDimensions(&i, &j, &k);
    if (i != _isize || j != _jsize || k != _ksize) {
        _fluidBrickGrid = FluidBrickGrid(_isize, _jsize, _ksize, _dx, brick);
    }
    _fluidBrickGrid.setBrickDimensions(brick);

    _isBrickOutputEnabled = true;
}

void FluidSimulation::enableBrickOutput(AABB brickbbox) {
    enableBrickOutput(brickbbox.width, brickbbox.height, brickbbox.depth);
}

void FluidSimulation::disableBrickOutput() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " disableBrickOutput" << std::endl);

    _isBrickOutputEnabled = false;
}

bool FluidSimulation::isBrickOutputEnabled() {
    return _isBrickOutputEnabled;
}

bool FluidSimulation::isFluidBrickGridInitialized() {
    return _fluidBrickGrid.isInitialized();
}

AABB FluidSimulation::getBrickAABB() {
    if (!isFluidBrickGridInitialized()) {
        return AABB();
    }
    
    return _fluidBrickGrid.getBrickAABB();
}

void FluidSimulation::enableAutosave() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " enableAutosave" << std::endl);

    _isAutosaveEnabled = true;
}

void FluidSimulation::disableAutosave() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " disableAutosave" << std::endl);

    _isAutosaveEnabled = false;
}

bool FluidSimulation::isAutosaveEnabled() {
    return _isAutosaveEnabled;
}

void FluidSimulation::enableOpenCLParticleAdvection() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " enableOpenCLParticleAdvection" << std::endl);

    _particleAdvector.enableOpenCL();
}

void FluidSimulation::disableOpenCLParticleAdvection() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " disableOpenCLParticleAdvection" << std::endl);

    _particleAdvector.disableOpenCL();
}

bool FluidSimulation::isOpenCLParticleAdvectionEnabled() {
    return _particleAdvector.isOpenCLEnabled();
}

void FluidSimulation::enableOpenCLScalarField() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " enableOpenCLScalarField" << std::endl);

    _scalarFieldAccelerator.enableOpenCL();
}

void FluidSimulation::disableOpenCLScalarField() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " disableOpenCLScalarField" << std::endl);

    _scalarFieldAccelerator.disableOpenCL();
}

bool FluidSimulation::isOpenCLScalarFieldEnabled() {
    return _scalarFieldAccelerator.isOpenCLEnabled();
}

int FluidSimulation::getParticleAdvectionKernelWorkLoadSize() {
    return _particleAdvector.getKernelWorkLoadSize();
}

void FluidSimulation::setParticleAdvectionKernelWorkLoadSize(int n) {
    if (n < 1) {
        std::string msg = "Error: work load size must be greater than or equal to 1.\n";
        msg += "size: " + _toString(n) + "\n";
        throw std::domain_error(msg);
    }

    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << 
                 " setParticleAdvectionKernelWorkLoadSize: " << n << std::endl);

    _particleAdvector.setKernelWorkLoadSize(n);
}

int FluidSimulation::getScalarFieldKernelWorkLoadSize() {
    return _scalarFieldAccelerator.getKernelWorkLoadSize();
}

void FluidSimulation::setScalarFieldKernelWorkLoadSize(int n) {
    if (n < 1) {
        std::string msg = "Error: work load size must be greater than or equal to 1.\n";
        msg += "size: " + _toString(n) + "\n";
        throw std::domain_error(msg);
    }

    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << 
                 " setScalarFieldKernelWorkLoadSize: " << n << std::endl);

    _scalarFieldAccelerator.setKernelWorkLoadSize(n);
}

void FluidSimulation::addBodyForce(double fx, double fy, double fz) { 
    addBodyForce(vmath::vec3(fx, fy, fz)); 
}

void FluidSimulation::addBodyForce(vmath::vec3 f) {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " addBodyForce: " <<
                 f.x << " " << f.y << " " << f.z << std::endl);

    _constantBodyForces.push_back(f);
}

void FluidSimulation::addBodyForce(vmath::vec3 (*fieldFunction)(vmath::vec3)) {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " addBodyForce: " << fieldFunction << std::endl);

    _variableBodyForces.push_back(fieldFunction);
}

vmath::vec3 FluidSimulation::getConstantBodyForce() {
    return _getConstantBodyForce();
}

vmath::vec3 FluidSimulation::getVariableBodyForce(double px, double py, double pz) {
    return _getVariableBodyForce(px, py, pz);
}

vmath::vec3 FluidSimulation::getVariableBodyForce(vmath::vec3 p) {
    return getVariableBodyForce(p.x, p.y, p.z);
}

vmath::vec3 FluidSimulation::getTotalBodyForce(double px, double py, double pz) {
    return getConstantBodyForce() + getVariableBodyForce(px, py, pz);
}

vmath::vec3 FluidSimulation::getTotalBodyForce(vmath::vec3 p) {
    return getTotalBodyForce(p.y, p.y, p.z);
}

void FluidSimulation::resetBodyForce() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " resetBodyForce" << std::endl);

    _constantBodyForces.clear();
    _variableBodyForces.clear();
}

void FluidSimulation::addImplicitFluidPoint(double x, double y, double z, double r) {
    addImplicitFluidPoint(vmath::vec3(x, y, z), r);
}

void FluidSimulation::addImplicitFluidPoint(vmath::vec3 p, double r) {
    if (r < 0.0) {
        std::string msg = "Error: implicit fluid point radius must be greater than or equal to 0.\n";
        msg += "radius: " + _toString(r) + "\n";
        throw std::domain_error(msg);
    }

    if (_isSimulationInitialized) {
        std::string msg = "Error: implicit fluid point must be added before before simulation is initialized.\n";
        throw std::runtime_error(msg);
    }

    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " addImplicitFluidPoint: " << 
                 p.x << " " << p.y << " " << p.z << " " << r << std::endl);

    _fluidPoints.push_back(FluidPoint(p, r));
}

void FluidSimulation::addFluidCuboid(double x, double y, double z, 
                                     double w, double h, double d) {
    addFluidCuboid(vmath::vec3(x, y, z), w, h, d);
}

void FluidSimulation::addFluidCuboid(vmath::vec3 p1, vmath::vec3 p2) {
    vmath::vec3 minp = vmath::vec3(fmin(p1.x, p2.x),
                               fmin(p1.y, p2.y), 
                               fmin(p1.z, p2.z));
    double width = fabs(p2.x - p1.x);
    double height = fabs(p2.y - p1.y);
    double depth = fabs(p2.z - p1.z);

    addFluidCuboid(minp, width, height, depth);
}

void FluidSimulation::addFluidCuboid(AABB bbox) {
    addFluidCuboid(bbox.position, bbox.width, bbox.height, bbox.depth);
}

void FluidSimulation::addFluidCuboid(vmath::vec3 p, double w, double h, double d) {
    if (!(w >= 0.0 && h >= 0.0 && d >= 0.0)) {
        std::string msg = "Error: Fluid cuboid dimensions must be greater than or equal to 0.\n";
        msg += "width: " + _toString(w) + 
               " height: " + _toString(h) + 
               " depth: " + _toString(d) + "\n";
        throw std::domain_error(msg);
    }

    if (_isSimulationInitialized) {
        std::string msg = "Error: fluid cuboid must be added before simulation is initialized.\n";
        throw std::runtime_error(msg);
    }

    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " addFluidCuboid: " << 
                 p.x << " " << p.y << " " << p.z << " " << w << " " << h << " " << d << std::endl);

    _fluidCuboids.push_back(FluidCuboid(p, w, h, d));
}

void FluidSimulation::addSphericalFluidSource(SphericalFluidSource *source) {
    for (unsigned int i = 0; i < _sphericalFluidSources.size(); i++) {
        if (source->getID() == _sphericalFluidSources[i]->getID()) {
            std::string msg = "Error: Fluid source has already been added.\n";
            throw std::runtime_error(msg);
        }
    }

    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " addSphericalFluidSource: " << source << std::endl);

    _fluidSources.push_back(source);
    _sphericalFluidSources.push_back(source);
}

void FluidSimulation::addCuboidFluidSource(CuboidFluidSource *source) {
    for (unsigned int i = 0; i < _cuboidFluidSources.size(); i++) {
        if (source->getID() == _cuboidFluidSources[i]->getID()) {
            std::string msg = "Error: Fluid source has already been added.\n";
            throw std::runtime_error(msg);
        }
    }

    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " addCuboidFluidSource: " << source << std::endl);

    _fluidSources.push_back(source);
    _cuboidFluidSources.push_back(source);
}

void FluidSimulation::removeFluidSource(FluidSource *source) {
    bool isFound = false;
    for (unsigned int i = 0; i < _fluidSources.size(); i++) {
        if (source->getID() == _fluidSources[i]->getID()) {
            _fluidSources.erase(_fluidSources.begin() + i);
            isFound = true;
            break;
        }
    }

    if (!isFound) {
        std::string msg = "Error: could not find fluid source to remove.\n";
        msg += "fluid source: " + _toString(source) + "\n";
        throw std::invalid_argument(msg);
    }

    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " removeFluidSource: " << source << std::endl);

    isFound = false;
    for (unsigned int i = 0; i < _sphericalFluidSources.size(); i++) {
        if (source->getID() == _sphericalFluidSources[i]->getID()) {
            _sphericalFluidSources.erase(_sphericalFluidSources.begin() + i);
            isFound = true;
            break;
        }
    }

    for (unsigned int i = 0; i < _cuboidFluidSources.size(); i++) {
        if (source->getID() == _cuboidFluidSources[i]->getID()) {
            _cuboidFluidSources.erase(_cuboidFluidSources.begin() + i);
            isFound = true;
            break;
        }
    }

    FLUIDSIM_ASSERT(isFound);
}

void FluidSimulation::removeFluidSources() {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " removeFluidSources" << std::endl);

    _fluidSources.clear();
    _sphericalFluidSources.clear();
    _cuboidFluidSources.clear();
}

void FluidSimulation::addSolidCells(std::vector<GridIndex> &indices) {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " addSolidCells: " << indices.size() << std::endl);

    GridIndex g;
    for (unsigned int i = 0; i < indices.size(); i++) {
        g = indices[i];
        if (!Grid3d::isGridIndexInRange(g, _isize, _jsize, _ksize)) {
            std::string msg = "Error: solid cell index out of range.\n";
            msg += "i: " + _toString(g.i) + " j: " + _toString(g.j) + " k: " + _toString(g.k) + "\n";
            throw std::out_of_range(msg);
        }


        _materialGrid.setSolid(g);
    }
}

void FluidSimulation::removeSolidCells(std::vector<GridIndex> &indices) {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " removeSolidCells: " << indices.size() << std::endl);

    GridIndex g;
    for (unsigned int i = 0; i < indices.size(); i++) {
        g = indices[i];
        if (!Grid3d::isGridIndexInRange(g, _isize, _jsize, _ksize)) {
            std::string msg = "Error: solid cell index out of range.\n";
            msg += "i: " + _toString(g.i) + " j: " + _toString(g.j) + " k: " + _toString(g.k) + "\n";
            throw std::out_of_range(msg);
        }

        // Cannot remove border cells
        if (Grid3d::isGridIndexOnBorder(g, _isize, _jsize, _ksize)) { 
            continue; 
        }

        if (_materialGrid.isCellSolid(g)) {
            _materialGrid.setAir(g);
        }
    }
}

void FluidSimulation::addFluidCells(std::vector<GridIndex> &indices, vmath::vec3 velocity) {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " addFluidCells: " << indices.size() << std::endl);

    GridCellGroup group(_isize, _jsize, _ksize, velocity);
    GridIndex g;
    for (unsigned int i = 0; i < indices.size(); i++) {
        g = indices[i];
        if (!Grid3d::isGridIndexInRange(g, _isize, _jsize, _ksize)) {
            std::string msg = "Error: fluid cell index out of range.\n";
            msg += "i: " + _toString(g.i) + " j: " + _toString(g.j) + " k: " + _toString(g.k) + "\n";
            throw std::out_of_range(msg);
        }

        if (_materialGrid.isCellAir(g)) {
            group.indices.push_back(g);
        }
    }

    if (!group.indices.empty()) {
        _addedFluidCellQueue.push_back(group);
    }
}

void FluidSimulation::addFluidCells(std::vector<GridIndex> &indices, 
                       double vx, double vy, double vz) {
    addFluidCells(indices, vmath::vec3(vx, vy, vz));
}

void FluidSimulation::addFluidCells(std::vector<GridIndex> &indices) {
    addFluidCells(indices, vmath::vec3(0.0, 0.0, 0.0));
}

void FluidSimulation::removeFluidCells(std::vector<GridIndex> &indices) {
    _logfile.log(std::ostringstream().flush() << 
                 _logfile.getTime() << " removeFluidCells: " << indices.size() << std::endl);

    GridCellGroup group(_isize, _jsize, _ksize);
    GridIndex g;
    for (unsigned int i = 0; i < indices.size(); i++) {
        g = indices[i];
        if (!Grid3d::isGridIndexInRange(g, _isize, _jsize, _ksize)) {
            std::string msg = "Error: fluid cell index out of range.\n";
            msg += "i: " + _toString(g.i) + " j: " + _toString(g.j) + " k: " + _toString(g.k) + "\n";
            throw std::out_of_range(msg);
        }

        group.indices.push_back(g);
    }

    if (!group.indices.empty()) {
        _removedFluidCellQueue.push_back(group);
    }
}

unsigned int FluidSimulation::getNumMarkerParticles() {
    return _markerParticles.size();
}

std::vector<MarkerParticle> FluidSimulation::getMarkerParticles() {
    return getMarkerParticles(0, _markerParticles.size());
}

std::vector<MarkerParticle> FluidSimulation::getMarkerParticles(int startidx, int endidx) {
    int size = _markerParticles.size();
    if (!(startidx >= 0 && startidx <= size) || !(endidx >= 0 && endidx <= size)) {
        std::string msg = "Error: invalid index range.\n";
        msg += "start index: " + _toString(startidx) + " end index: " + _toString(endidx) + "\n";
        throw std::out_of_range(msg);
    }

    std::vector<MarkerParticle> particles;
    particles.reserve(endidx - startidx);

    for (int i = startidx; i < endidx; i++) {
        particles.push_back(_markerParticles[i]);
    }

    return particles;
}

std::vector<vmath::vec3> FluidSimulation::getMarkerParticlePositions() {
    return getMarkerParticlePositions(0, _markerParticles.size());
}

std::vector<vmath::vec3> FluidSimulation::getMarkerParticlePositions(int startidx, int endidx) {
    int size = _markerParticles.size();
    if (!(startidx >= 0 && startidx <= size) || !(endidx >= 0 && endidx <= size)) {
        std::string msg = "Error: invalid index range.\n";
        msg += "start index: " + _toString(startidx) + " end index: " + _toString(endidx) + "\n";
        throw std::out_of_range(msg);
    }

    std::vector<vmath::vec3> particles;
    particles.reserve(endidx - startidx);

    for (int i = startidx; i < endidx; i++) {
        particles.push_back(_markerParticles[i].position);
    }

    return particles;
}

std::vector<vmath::vec3> FluidSimulation::getMarkerParticleVelocities() {
    return getMarkerParticleVelocities(0, _markerParticles.size());
}

std::vector<vmath::vec3> FluidSimulation::getMarkerParticleVelocities(int startidx, int endidx) {
    int size = _markerParticles.size();
    if (!(startidx >= 0 && startidx <= size) || !(endidx >= 0 && endidx <= size)) {
        std::string msg = "Error: invalid index range.\n";
        msg += "start index: " + _toString(startidx) + " end index: " + _toString(endidx) + "\n";
        throw std::out_of_range(msg);
    }

    std::vector<vmath::vec3> velocities;
    velocities.reserve(endidx - startidx);

    for (int i = startidx; i < endidx; i++) {
        velocities.push_back(_markerParticles[i].velocity);
    }

    return velocities;
}

unsigned int FluidSimulation::getNumDiffuseParticles() {
    return _diffuseMaterial.getNumDiffuseParticles();
}

std::vector<DiffuseParticle> FluidSimulation::getDiffuseParticles() {
    return getDiffuseParticles(0, _markerParticles.size());
}

std::vector<DiffuseParticle> FluidSimulation::getDiffuseParticles(int startidx, int endidx) {
    int size = getNumDiffuseParticles();
    if (!(startidx >= 0 && startidx <= size) || !(endidx >= 0 && endidx <= size)) {
        std::string msg = "Error: invalid index range.\n";
        msg += "start index: " + _toString(startidx) + " end index: " + _toString(endidx) + "\n";
        throw std::out_of_range(msg);
    }

    std::vector<DiffuseParticle> particles;
    particles.reserve(endidx - startidx);

    FragmentedVector<DiffuseParticle> *dps = _diffuseMaterial.getDiffuseParticles();
    for (int i = startidx; i < endidx; i++) {
        particles.push_back(dps->at(i));
    }

    return particles;
}

std::vector<vmath::vec3> FluidSimulation::getDiffuseParticlePositions() {
    int size = getNumDiffuseParticles();
    return getDiffuseParticlePositions(0, size);
}

std::vector<vmath::vec3> FluidSimulation::getDiffuseParticlePositions(int startidx, int endidx) {
    int size = getNumDiffuseParticles();
    if (!(startidx >= 0 && startidx <= size) || !(endidx >= 0 && endidx <= size)) {
        std::string msg = "Error: invalid index range.\n";
        msg += "start index: " + _toString(startidx) + " end index: " + _toString(endidx) + "\n";
        throw std::out_of_range(msg);
    }

    std::vector<vmath::vec3> particles;
    particles.reserve(endidx - startidx);

    FragmentedVector<DiffuseParticle> *dps = _diffuseMaterial.getDiffuseParticles();
    for (int i = startidx; i < endidx; i++) {
        particles.push_back(dps->at(i).position);
    }

    return particles;
}

std::vector<vmath::vec3> FluidSimulation::getDiffuseParticleVelocities() {
    int size = getNumDiffuseParticles();
    return getDiffuseParticleVelocities(0, size);
}

std::vector<vmath::vec3> FluidSimulation::getDiffuseParticleVelocities(int startidx, int endidx) {
    int size = getNumDiffuseParticles();
    if (!(startidx >= 0 && startidx <= size) || !(endidx >= 0 && endidx <= size)) {
        std::string msg = "Error: invalid index range.\n";
        msg += "start index: " + _toString(startidx) + " end index: " + _toString(endidx) + "\n";
        throw std::out_of_range(msg);
    }

    std::vector<vmath::vec3> velocities;
    velocities.reserve(endidx - startidx);

    FragmentedVector<DiffuseParticle> *dps = _diffuseMaterial.getDiffuseParticles();
    for (int i = startidx; i < endidx; i++) {
        velocities.push_back(dps->at(i).velocity);
    }

    return velocities;
}

std::vector<float> FluidSimulation::getDiffuseParticleLifetimes() {
    int size = getNumDiffuseParticles();
    return getDiffuseParticleLifetimes(0, size);
}

std::vector<float> FluidSimulation::getDiffuseParticleLifetimes(int startidx, int endidx) {
    int size = getNumDiffuseParticles();
    if (!(startidx >= 0 && startidx <= size) || !(endidx >= 0 && endidx <= size)) {
        std::string msg = "Error: invalid index range.\n";
        msg += "start index: " + _toString(startidx) + " end index: " + _toString(endidx) + "\n";
        throw std::out_of_range(msg);
    }

    std::vector<float> lifetimes;
    lifetimes.reserve(endidx - startidx);

    FragmentedVector<DiffuseParticle> *dps = _diffuseMaterial.getDiffuseParticles();
    for (int i = startidx; i < endidx; i++) {
        lifetimes.push_back(dps->at(i).lifetime);
    }

    return lifetimes;
}

std::vector<char> FluidSimulation::getDiffuseParticleTypes() {
    int size = getNumDiffuseParticles();
    return getDiffuseParticleTypes(0, size);
}

std::vector<char> FluidSimulation::getDiffuseParticleTypes(int startidx, int endidx) {
    int size = getNumDiffuseParticles();
    if (!(startidx >= 0 && startidx <= size) || !(endidx >= 0 && endidx <= size)) {
        std::string msg = "Error: invalid index range.\n";
        msg += "start index: " + _toString(startidx) + " end index: " + _toString(endidx) + "\n";
        throw std::out_of_range(msg);
    }

    std::vector<char> types;
    types.reserve(endidx - startidx);

    FragmentedVector<DiffuseParticle> *dps = _diffuseMaterial.getDiffuseParticles();
    for (int i = startidx; i < endidx; i++) {
        types.push_back((char)(dps->at(i).type));
    }

    return types;
}

MACVelocityField* FluidSimulation::getVelocityField() { 
    return &_MACVelocity; 
}

LevelSet* FluidSimulation::getLevelSet() { 
    return &_levelset; 
};

FluidBrickGrid* FluidSimulation::getFluidBrickGrid() {
    return &_fluidBrickGrid;
};

/********************************************************************************
    Initializing the Fluid Simulator
********************************************************************************/
void FluidSimulation::_initializeLogFile() {
    _logfile.setPath(Config::getLogsDirectory());
}

void FluidSimulation::_initializeSimulationGrids(int isize, int jsize, int ksize, double dx) {
    _logfile.separator();
    _logfile.timestamp();
    _logfile.newline();
    _logfile.log(std::ostringstream().flush() << 
                 "Initializing Simulation Grids:" << std::endl <<
                 "\tGrid Dimensions: " << isize << " x " << 
                                        jsize << " x " << 
                                        ksize << std::endl <<
                 "\tCell Size:       " << dx << std::endl);

    StopWatch t;
    t.start();
    _MACVelocity = MACVelocityField(isize, jsize, ksize, dx);
    t.stop();

    _logfile.log("Constructing MACVelocityField: \t", t.getTime(), 4, 1);

    t.reset();
    t.start();
    _materialGrid = FluidMaterialGrid(isize, jsize, ksize);
    t.stop();

    _logfile.log("Constructing FluidMaterialGrid:\t", t.getTime(), 4, 1);

    t.reset();
    t.start();
    _levelset = LevelSet(isize, jsize, ksize, dx);
    t.stop();

    _logfile.log("Constructing LevelSet:         \t", t.getTime(), 4, 1);
}

void FluidSimulation::_initializeSimulationVectors(int isize, int jsize, int ksize) {
    _fluidCellIndices = GridIndexVector(isize, jsize, ksize);
}

void FluidSimulation::_initializeSolidCells() {
    // fill borders with solid cells
    for (int j = 0; j < _jsize; j++) {
        for (int i = 0; i < _isize; i++) {
            _materialGrid.setSolid(i, j, 0);
            _materialGrid.setSolid(i, j, _ksize-1);
        }
    }

    for (int k = 0; k < _ksize; k++) {
        for (int i = 0; i < _isize; i++) {
            _materialGrid.setSolid(i, 0, k);
            _materialGrid.setSolid(i, _jsize-1, k);
        }
    }

    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            _materialGrid.setSolid(0, j, k);
            _materialGrid.setSolid(_isize-1, j, k);
        }
    }
}

void FluidSimulation::_addMarkerParticlesToCell(GridIndex g) {
    _addMarkerParticlesToCell(g, vmath::vec3());
}

double FluidSimulation::_getMarkerParticleJitter() {
    return 0.25 * _markerParticleJitterFactor * _dx;
}

void FluidSimulation::_addMarkerParticlesToCell(GridIndex g, vmath::vec3 velocity) {
    double q = 0.25*_dx;
    vmath::vec3 c = Grid3d::GridIndexToCellCenter(g, _dx);

    vmath::vec3 points[] = {
        vmath::vec3(c.x - q, c.y - q, c.z - q),
        vmath::vec3(c.x + q, c.y - q, c.z - q),
        vmath::vec3(c.x + q, c.y - q, c.z + q),
        vmath::vec3(c.x - q, c.y - q, c.z + q),
        vmath::vec3(c.x - q, c.y + q, c.z - q),
        vmath::vec3(c.x + q, c.y + q, c.z - q),
        vmath::vec3(c.x + q, c.y + q, c.z + q),
        vmath::vec3(c.x - q, c.y + q, c.z + q)
    };

    double jitter = _getMarkerParticleJitter();
    for (int idx = 0; idx < 8; idx++) {
        vmath::vec3 jit = vmath::vec3(_randomDouble(-jitter, jitter),
                                  _randomDouble(-jitter, jitter),
                                  _randomDouble(-jitter, jitter));

        vmath::vec3 p = points[idx] + jit;
        _markerParticles.push_back(MarkerParticle(p, velocity));
    }
}

void FluidSimulation::_addMarkerParticle(vmath::vec3 p) {
    _addMarkerParticle(p, vmath::vec3());
}

void FluidSimulation::_addMarkerParticle(vmath::vec3 p, vmath::vec3 velocity) {
    GridIndex g = Grid3d::positionToGridIndex(p, _dx);
    if (Grid3d::isGridIndexInRange(g, _isize, _jsize, _ksize) &&
        !_materialGrid.isCellSolid(g)) {
        _markerParticles.push_back(MarkerParticle(p, velocity));
    }
}

void FluidSimulation::_addMarkerParticles(std::vector<vmath::vec3> particles) {
    for (unsigned int i = 0; i < particles.size(); i++) {
        _addMarkerParticle(particles[i], vmath::vec3());
    }
}

void FluidSimulation::_calculateInitialFluidSurfaceScalarField(ScalarField &field) {
    FluidPoint fp;
    for (unsigned int i = 0; i < _fluidPoints.size(); i++) {
        fp = _fluidPoints[i];
        field.addPoint(fp.position, fp.radius);
    }

    FluidCuboid fc;
    for (unsigned int i = 0; i < _fluidCuboids.size(); i++) {
        fc = _fluidCuboids[i];
        field.addCuboid(fc.bbox.position, fc.bbox.width, fc.bbox.height, fc.bbox.depth);
    }
}

void FluidSimulation::_getInitialFluidCellsFromScalarField(ScalarField &field,
                                                           GridIndexVector &fluidCells) {
    field.setMaterialGrid(_materialGrid);
    double threshold = field.getSurfaceThreshold();

    vmath::vec3 c;
    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                if (!_materialGrid.isCellSolid(i, j, k) &&
                    field.getScalarFieldValueAtCellCenter(i, j, k) > threshold) {
                    fluidCells.push_back(GridIndex(i, j, k));
                }
            }
        }
    }
}

void FluidSimulation::_getFullAndPartiallyFullFluidCells(GridIndexVector &fluidCells,
                                                         GridIndexVector &fullFluidCells,
                                                         GridIndexVector &partialFluidCells) {
    FluidMaterialGrid mgrid = _materialGrid;
    for (unsigned int i = 0; i < fluidCells.size(); i++) {
        mgrid.setFluid(fluidCells[i]);
    }

    GridIndex g;
    for (unsigned int i = 0; i < fluidCells.size(); i++) {
        g = fluidCells[i];
        if (mgrid.isCellNeighbouringAir(g)) {
            partialFluidCells.push_back(g);
        } else {
            fullFluidCells.push_back(g);
        }
    }

    for (int k = 0; k < mgrid.depth; k++) {
        for (int j = 0; j < mgrid.height; j++) {
            for (int i = 0; i < mgrid.width; i++) {
                if (mgrid.isCellAir(i, j, k) &&
                    mgrid.isCellNeighbouringFluid(i, j, k)) {
                    partialFluidCells.push_back(i, j, k);
                }
            }
        }
    }
}

void FluidSimulation::_getPartiallyFullFluidCellParticles(GridIndexVector &partialFluidCells,
                                                          ScalarField &field,
                                                          std::vector<vmath::vec3> &partialParticles) {
    FluidMaterialGrid submgrid(_isize, _jsize, _ksize);
    for (unsigned int i = 0; i < partialFluidCells.size(); i++) {
        submgrid.setFluid(partialFluidCells[i]);
    }
    submgrid.setSubdivisionLevel(2);
    double subdx = 0.5*_dx;

    double jitter = _getMarkerParticleJitter();
    vmath::vec3 jit;
    vmath::vec3 c;
    for (int k = 0; k < submgrid.depth; k++) {
        for (int j = 0; j < submgrid.height; j++) {
            for (int i = 0; i < submgrid.width; i++) {
                if (!submgrid.isCellFluid(i, j, k)) {
                    continue;
                }

                c = Grid3d::GridIndexToCellCenter(i, j, k, subdx);
                if (field.isPointInside(c)) {
                    jit = vmath::vec3(_randomDouble(-jitter, jitter),
                                      _randomDouble(-jitter, jitter),
                                      _randomDouble(-jitter, jitter));
                    partialParticles.push_back(c + jit);
                }
            }
        }
    }
}

void FluidSimulation::_initializeMarkerParticles(GridIndexVector &fullFluidCells,
                                                 std::vector<vmath::vec3> &partialParticles) {
    _markerParticles.reserve((unsigned int)(8*fullFluidCells.size() + partialParticles.size()));
    GridIndex g;
    for (int i = 0; i < (int)fullFluidCells.size(); i++) {
        g = fullFluidCells[i];
        _addMarkerParticlesToCell(g);
    }
    _addMarkerParticles(partialParticles);
}

void FluidSimulation::_initializeFluidCellIndices() {
    GridIndex g;
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        g = Grid3d::positionToGridIndex(_markerParticles[i].position, _dx);
        if (!_materialGrid.isCellFluid(g)) {
            _materialGrid.setFluid(g);
            _fluidCellIndices.push_back(g);
        }
    }
}

void FluidSimulation::_initializeFluidMaterial() {
    ScalarField field = ScalarField(_isize + 1, _jsize + 1, _ksize + 1, _dx);
    _calculateInitialFluidSurfaceScalarField(field);

    GridIndexVector fluidCells(_isize, _jsize, _ksize);
    _getInitialFluidCellsFromScalarField(field, fluidCells);

    GridIndexVector fullFluidCells(_isize, _jsize, _ksize);
    GridIndexVector partialFluidCells(_isize, _jsize, _ksize);
    _getFullAndPartiallyFullFluidCells(fluidCells, fullFluidCells, partialFluidCells);

    std::vector<vmath::vec3> partialParticles;
    _getPartiallyFullFluidCellParticles(partialFluidCells, field, partialParticles);

    _initializeMarkerParticles(fullFluidCells, partialParticles);
    _initializeFluidCellIndices();
}

void FluidSimulation::_initializeMarkerParticleRadius() {
    double volume = _dx*_dx*_dx / 8.0;
    double pi = 3.141592653;
    _markerParticleRadius = pow(3*volume / (4*pi), 1.0/3.0);
}

void FluidSimulation::_initializeSimulation() {
    _logfile.newline();
    _logfile.log(std::ostringstream().flush() << 
                 "Initializing Simulation:" << std::endl);

    StopWatch t;
    t.start();
    _initializeSolidCells();
    t.stop();

    _logfile.log("Initializing Solid Cells:    \t", t.getTime(), 4, 1);

    t.reset();
    t.start();
    _initializeFluidMaterial();
    t.stop();

    _logfile.log("Initializing Fluid Material: \t", t.getTime(), 4, 1);

    _initializeMarkerParticleRadius();

    t.reset();
    t.start();
    _initializeCLObjects();
    t.stop();

    _logfile.log("Initializing OpenCL Objects: \t", t.getTime(), 4, 1);
    _logOpenCLInfo();

    _isSimulationInitialized = true;
}

void FluidSimulation::_logOpenCLInfo() {
    _logfile.newline();
    _logfile.separator();
    _logfile.newline();
    _logfile.log(std::ostringstream().flush() << 
                 "OpenCL ParticleAdvector Device Info:" << std::endl);
    std::string deviceInfo = _particleAdvector.getDeviceInfo();
    _logfile.log(std::ostringstream().flush() << deviceInfo << std::endl);

    _logfile.log(std::ostringstream().flush() << 
                 "OpenCL ParticleAdvector Kernel Info:" << std::endl);
    std::string kernelInfo = _particleAdvector.getKernelInfo();
    _logfile.log(std::ostringstream().flush() << kernelInfo << std::endl);

    _logfile.separator();
    _logfile.newline();
    _logfile.log(std::ostringstream().flush() << 
                 "OpenCL CLScalarField Device Info:" << std::endl);
    deviceInfo = _scalarFieldAccelerator.getDeviceInfo();
    _logfile.log(std::ostringstream().flush() << deviceInfo << std::endl);

    _logfile.log(std::ostringstream().flush() << 
                 "OpenCL CLScalarField Kernel Info:" << std::endl);
    kernelInfo = _scalarFieldAccelerator.getKernelInfo();
    _logfile.log(std::ostringstream().flush() << kernelInfo << std::endl);
}

void FluidSimulation::_initializeFluidMaterialParticlesFromSaveState() {
    MarkerParticle p;
    GridIndex g;
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        p = _markerParticles[i];
        g = Grid3d::positionToGridIndex(p.position, _dx);
        FLUIDSIM_ASSERT(!_materialGrid.isCellSolid(g));
        _materialGrid.setFluid(g);
    }

    _fluidCellIndices.clear();
    for (int k = 0; k < _materialGrid.depth; k++) {
        for (int j = 0; j < _materialGrid.height; j++) {
            for (int i = 0; i < _materialGrid.width; i++) {
                if (_materialGrid.isCellFluid(i, j, k)) {
                    _fluidCellIndices.push_back(i, j, k);
                }
            }
        }
    }
}

void FluidSimulation::_initializeMarkerParticlesFromSaveState(
                                        FluidSimulationSaveState &state) {

    int n = state.getNumMarkerParticles();
    _markerParticles.reserve(n);

    int chunksize = _loadStateReadChunkSize;
    int numRead = 0;

    std::vector<vmath::vec3> vectors;
    while (numRead < n) {
        int startidx = numRead;
        int endidx = numRead + chunksize;
        if (endidx > n) {
            endidx = n;
        }

        vectors = state.getMarkerParticlePositions(startidx, endidx);
        for (unsigned int i = 0; i < vectors.size(); i++) {
            _markerParticles.push_back(MarkerParticle(vectors[i]));
        }

        numRead += (int)vectors.size();
    }

    numRead = 0;
    while (numRead < n) {
        int startidx = numRead;
        int endidx = numRead + chunksize;
        if (endidx > n) {
            endidx = n;
        }

        vectors = state.getMarkerParticleVelocities(startidx, endidx);
        for (unsigned int i = 0; i < vectors.size(); i++) {
            _markerParticles[startidx + i].velocity = vectors[i];
        }

        numRead += (int)vectors.size();
    }

}

void FluidSimulation::_initializeDiffuseParticlesFromSaveState(
                                        FluidSimulationSaveState &state) {
    FragmentedVector<DiffuseParticle> diffuseParticles;

    int n = state.getNumDiffuseParticles();
    diffuseParticles.reserve(n);

    int chunksize = _loadStateReadChunkSize;
    int numRead = 0;

    std::vector<vmath::vec3> vectors;
    while (numRead < n) {
        int startidx = numRead;
        int endidx = numRead + chunksize;
        if (endidx > n) {
            endidx = n;
        }

        vectors = state.getDiffuseParticlePositions(startidx, endidx);
        for (unsigned int i = 0; i < vectors.size(); i++) {
            diffuseParticles.push_back(DiffuseParticle());
            diffuseParticles[startidx + i].position = vectors[i];
        }

        numRead += (int)vectors.size();
    }

    numRead = 0;
    while (numRead < n) {
        int startidx = numRead;
        int endidx = numRead + chunksize;
        if (endidx > n) {
            endidx = n;
        }

        vectors = state.getDiffuseParticleVelocities(startidx, endidx);
        for (unsigned int i = 0; i < vectors.size(); i++) {
            diffuseParticles[startidx + i].velocity = vectors[i];
        }

        numRead += (int)vectors.size();
    }
    vectors.clear();
    vectors.shrink_to_fit();

    numRead = 0;
    std::vector<float> lifetimes;
    while (numRead < n) {
        int startidx = numRead;
        int endidx = numRead + chunksize;
        if (endidx > n) {
            endidx = n;
        }

        lifetimes = state.getDiffuseParticleLifetimes(startidx, endidx);
        for (unsigned int i = 0; i < lifetimes.size(); i++) {
            diffuseParticles[startidx + i].lifetime = lifetimes[i];
        }

        numRead += (int)lifetimes.size();
    }

    numRead = 0;
    std::vector<char> types;
    while (numRead < n) {
        int startidx = numRead;
        int endidx = numRead + chunksize;
        if (endidx > n) {
            endidx = n;
        }

        types = state.getDiffuseParticleTypes(startidx, endidx);
        for (unsigned int i = 0; i < types.size(); i++) {
            diffuseParticles[startidx + i].type = (DiffuseParticleType)types[i];
        }

        numRead += (int)types.size();
    }

    _diffuseMaterial.setDiffuseParticles(diffuseParticles);
}

void FluidSimulation::_initializeSolidCellsFromSaveState(FluidSimulationSaveState &state) {
    int n = state.getNumSolidCells();
    int chunksize = _loadStateReadChunkSize;
    int numRead = 0;

    std::vector<GridIndex> indices;
    while (numRead < n) {
        int startidx = numRead;
        int endidx = numRead + chunksize;
        if (endidx > n) {
            endidx = n;
        }

        indices = state.getSolidCells(startidx, endidx);
        addSolidCells(indices);
        numRead += (int)indices.size();
    }
}

void FluidSimulation::_initializeFluidBrickGridFromSaveState(FluidSimulationSaveState &state) {
    FluidBrickGridSaveState brickstate;
    state.getFluidBrickGridSaveState(brickstate);
    _fluidBrickGrid = FluidBrickGrid(brickstate);
}

void FluidSimulation::_initializeSimulationFromSaveState(FluidSimulationSaveState &state) {
    state.getGridDimensions(&_isize, &_jsize, &_ksize);
    _dx = state.getCellSize();
    _initializeSimulationGrids(_isize, _jsize, _ksize, _dx);
    _initializeSimulationVectors(_isize, _jsize, _ksize);

    _logfile.newline();
    _logfile.log(std::ostringstream().flush() << 
                 "Initializing Simulation From Savestate:" << std::endl);

    _currentFrame = state.getCurrentFrame();
    _currentBrickMeshFrame = fmax(_currentFrame + _brickMeshFrameOffset, 0);

    _logfile.log(std::ostringstream().flush() << 
                 "\tCurrentFrame: " << _currentFrame << std::endl);

    StopWatch t;
    t.start();
    _initializeSolidCellsFromSaveState(state);
    t.stop();

    _logfile.log("Loading Solid Cells:         \t", t.getTime(), 4, 1);
    
    t.reset();
    t.start();
    _initializeMarkerParticlesFromSaveState(state);
    t.stop();

    _logfile.log("Loading Marker Particles:    \t", t.getTime(), 4, 1);

    t.reset();
    t.start();
    _initializeDiffuseParticlesFromSaveState(state);
    t.stop();

    _logfile.log("Loading Diffuse Particles:   \t", t.getTime(), 4, 1);

    t.reset();
    t.start();
    _initializeFluidMaterialParticlesFromSaveState();
    t.stop();

    _logfile.log("Initializing Fluid Material: \t", t.getTime(), 4, 1);

    _initializeMarkerParticleRadius();

    if (state.isFluidBrickGridEnabled()) {
        t.reset();
        t.start();
        _initializeFluidBrickGridFromSaveState(state);
        t.stop();

        _logfile.log("Loading Brick Grid:          \t", t.getTime(), 4, 1);
    }

    t.reset();
    t.start();
    _initializeCLObjects();
    t.stop();

    _logfile.log("Initializing OpenCL Objects: \t", t.getTime(), 4, 1);

    _logOpenCLInfo();

    _isSimulationInitialized = true;
}

void FluidSimulation::_initializeCLObjects() {
    bool success = _particleAdvector.initialize();
    FLUIDSIM_ASSERT(success);

    success = _scalarFieldAccelerator.initialize();
    FLUIDSIM_ASSERT(success);
}

/********************************************************************************
    1. Update Fluid Material
********************************************************************************/

void FluidSimulation::_removeMarkerParticlesFromCells(Array3d<bool> &isRemovalCell) {
    std::vector<bool> isRemoved;
    isRemoved.reserve(_markerParticles.size());

    MarkerParticle p;
    GridIndex g;
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        p = _markerParticles[i];
        g = Grid3d::positionToGridIndex(p.position, _dx);
        isRemoved.push_back(isRemovalCell(g));
    }

    _removeItemsFromVector(_markerParticles, isRemoved);
}

void FluidSimulation::_removeDiffuseParticlesFromCells(Array3d<bool> &isRemovalCell) {
    std::vector<bool> isRemoved;
    isRemoved.reserve(getNumDiffuseParticles());

    FragmentedVector<DiffuseParticle> *diffuseParticles = _diffuseMaterial.getDiffuseParticles();

    DiffuseParticle p;
    GridIndex g;
    for (unsigned int i = 0; i < diffuseParticles->size(); i++) {
        p = diffuseParticles->at(i);
        g = Grid3d::positionToGridIndex(p.position, _dx);
        isRemoved.push_back(isRemovalCell(g));
    }

    _removeItemsFromVector(*diffuseParticles, isRemoved);
}

void FluidSimulation::_addNewFluidCells(GridIndexVector &cells, 
                                        vmath::vec3 velocity) {

    _markerParticles.reserve((unsigned int)(_markerParticles.size() + 8*cells.size()));
    for (int i = 0; i < (int)cells.size(); i++) {
        if (_materialGrid.isCellAir(cells[i])) {
            _addMarkerParticlesToCell(cells[i], velocity);
            _materialGrid.setFluid(cells[i]);
        }
    }
}

void FluidSimulation::_addNewFluidParticles(std::vector<vmath::vec3> &particles, vmath::vec3 velocity) {
    _markerParticles.reserve((unsigned int)(_markerParticles.size() + particles.size()));
    GridIndex g;
    for (unsigned int i = 0; i < particles.size(); i++) {
        g = Grid3d::positionToGridIndex(particles[i], _dx);
        _addMarkerParticle(particles[i], velocity);
        _materialGrid.setFluid(g);
    }
}

void FluidSimulation::_getNewFluidParticles(FluidSource *source, std::vector<vmath::vec3> &particles) {
    AABB bbox = source->getAABB();
    bbox = Grid3d::fitAABBtoGrid(bbox, _dx, _isize, _jsize, _ksize);

    GridIndex gmin, gmax;
    Grid3d::getGridIndexBounds(bbox, _dx, _isize, _jsize, _ksize, &gmin, &gmax);
    int iwidth = gmax.i - gmin.i + 1;
    int jheight = gmax.j - gmin.j + 1;
    int kdepth = gmax.k - gmin.k + 1;
    
    Array3d<bool> isInvalidCell = Array3d<bool>(iwidth, jheight, kdepth, true);
    GridIndexVector sourceCells = source->getFluidOrAirCells(_materialGrid, _dx);
    GridIndex g;
    for (unsigned int i = 0; i < sourceCells.size(); i++) {
        g = sourceCells[i];
        g = GridIndex(g.i - gmin.i, g.j - gmin.j, g.k - gmin.k);
        isInvalidCell.set(g, false);
    }

    GridIndex subgridOffsets[8] = {
        GridIndex(0, 0, 0), GridIndex(0, 0, 1), GridIndex(0, 1, 0), GridIndex(0, 1, 1),
        GridIndex(1, 0, 0), GridIndex(1, 0, 1), GridIndex(1, 1, 0), GridIndex(1, 1, 1)
    };

    Array3d<bool> newParticleGrid = Array3d<bool>(2*iwidth, 2*jheight, 2*kdepth, true);
    GridIndex subg;
    for (int k = 0; k < isInvalidCell.depth; k++) {
        for (int j = 0; j < isInvalidCell.height; j++) {
            for (int i = 0; i < isInvalidCell.width; i++) {
                if (isInvalidCell(i, j, k)) {
                    for (int idx = 0; idx < 8; idx++) {
                        subg = subgridOffsets[idx];
                        newParticleGrid.set(2*i + subg.i, 2*j + subg.j, 2*k + subg.k, false);
                    }
                }
            }
        }
    }

    vmath::vec3 offset = Grid3d::GridIndexToPosition(gmin, _dx);
    vmath::vec3 p;
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        if (bbox.isPointInside(_markerParticles[i].position)) {
            p = _markerParticles[i].position - offset;
            subg = Grid3d::positionToGridIndex(p, 0.5*_dx);
            newParticleGrid.set(subg, false);
        }
    }

    double jitter = _getMarkerParticleJitter();
    vmath::vec3 jit;
    for (int k = 0; k < newParticleGrid.depth; k++) {
        for (int j = 0; j < newParticleGrid.height; j++) {
            for (int i = 0; i < newParticleGrid.width; i++) {
                if (newParticleGrid(i, j, k)) {
                    jit = vmath::vec3(_randomDouble(-jitter, jitter),
                                    _randomDouble(-jitter, jitter),
                                    _randomDouble(-jitter, jitter));

                    p = Grid3d::GridIndexToCellCenter(i, j, k, 0.5*_dx);
                    particles.push_back(p + offset + jit);
                }
            }
        }
    }
}

void FluidSimulation::_updateInflowFluidSource(FluidSource *source) {
    FLUIDSIM_ASSERT(source->isInflow());

    GridIndexVector newCells = source->getAirCells(_materialGrid, _dx);
    vmath::vec3 velocity = source->getVelocity();
    if (newCells.size() > 0) {
        _addNewFluidCells(newCells, velocity);
    }

    std::vector<vmath::vec3> newParticles;
    _getNewFluidParticles(source, newParticles);
    if (newParticles.size() > 0) {
        _addNewFluidParticles(newParticles, velocity);
    }
}

void FluidSimulation::_updateFluidSources() {

    Array3d<bool> isOutflowCell(_isize, _jsize, _ksize, false);
    bool isOutflowCellInSimulation = false;

    FluidSource *source;
    for (unsigned int i = 0; i < _fluidSources.size(); i++) {
        source = _fluidSources[i];

        if (source->isInflow()) {
            _updateInflowFluidSource(_fluidSources[i]);
        } else if (source->isOutflow()) {
            GridIndexVector cells = source->getFluidCells(_materialGrid, _dx);

            for (unsigned int cidx = 0; cidx < cells.size(); cidx++) {
                isOutflowCell.set(cells[cidx], true);
                isOutflowCellInSimulation = true;
            }
        }
    }

    if (isOutflowCellInSimulation) {
        _removeMarkerParticlesFromCells(isOutflowCell);
        _removeDiffuseParticlesFromCells(isOutflowCell);
    }
}

void FluidSimulation::_updateAddedFluidCellQueue() {
    if (_addedFluidCellQueue.empty()) {
        return;
    }

    for (unsigned int i = 0; i < _addedFluidCellQueue.size(); i++) {
        _addNewFluidCells(_addedFluidCellQueue[i].indices, 
                          _addedFluidCellQueue[i].velocity);
    }
    _addedFluidCellQueue.clear();
    _addedFluidCellQueue.shrink_to_fit();
}

void FluidSimulation::_updateRemovedFluidCellQueue() {
    if (_removedFluidCellQueue.empty()) {
        return;
    }

    Array3d<bool> isRemovalCell(_isize, _jsize, _ksize, false);
    GridIndexVector *group;
    for (unsigned int i = 0; i < _removedFluidCellQueue.size(); i++) {
        group = &(_removedFluidCellQueue[i].indices);
        for (unsigned int gidx = 0; gidx < group->size(); gidx++) {
            isRemovalCell.set(group->at(gidx), true);
        }
    }
    _removedFluidCellQueue.clear();
    _removedFluidCellQueue.shrink_to_fit();

    std::vector<bool> isRemoved;
    isRemoved.reserve(_markerParticles.size());

    bool isParticlesInRemovalCell = false;
    MarkerParticle p;
    GridIndex g;
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        p = _markerParticles[i];
        g = Grid3d::positionToGridIndex(p.position, _dx);

        bool isInRemovalCell = isRemovalCell(g);
        if (isInRemovalCell) {
            isParticlesInRemovalCell = true;
        }

        isRemoved.push_back(isInRemovalCell);
    }

    if (isParticlesInRemovalCell) {
        _removeItemsFromVector(_markerParticles, isRemoved);
    }
}

void FluidSimulation::_removeMarkerParticlesInSolidCells() {
    std::vector<bool> isRemoved;
    isRemoved.reserve(_markerParticles.size());

    bool isParticlesInSolidCell = false;
    MarkerParticle p;
    GridIndex g;
    int count = 0;
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        p = _markerParticles[i];
        g = Grid3d::positionToGridIndex(p.position, _dx);

        bool isInSolidCell = _materialGrid.isCellSolid(g);
        if (isInSolidCell) {
            isParticlesInSolidCell = true;
            count++;
        }

        isRemoved.push_back(isInSolidCell);
    }

    if (isParticlesInSolidCell) {
        _removeItemsFromVector(_markerParticles, isRemoved);
    }
}

void FluidSimulation::_removeDiffuseParticlesInSolidCells() {
    FragmentedVector<DiffuseParticle> *diffuseParticles = _diffuseMaterial.getDiffuseParticles();

    std::vector<bool> isRemoved;
    isRemoved.reserve(diffuseParticles->size());

    bool isParticlesInSolidCell = false;
    DiffuseParticle p;
    GridIndex g;
    for (unsigned int i = 0; i < diffuseParticles->size(); i++) {
        p = diffuseParticles->at(i);
        g = Grid3d::positionToGridIndex(p.position, _dx);

        bool isInSolidCell = _materialGrid.isCellSolid(g);
        if (isInSolidCell) {
            isParticlesInSolidCell = true;
        }

        isRemoved.push_back(_materialGrid.isCellSolid(g));
    }

    if (isParticlesInSolidCell) {
        _removeItemsFromVector(*diffuseParticles, isRemoved);
    }
}

void FluidSimulation::_removeParticlesInSolidCells() {
    _removeMarkerParticlesInSolidCells();
    _removeDiffuseParticlesInSolidCells();
}

void FluidSimulation::_updateFluidCells() {
    _removeParticlesInSolidCells();
    _updateAddedFluidCellQueue();
    _updateRemovedFluidCellQueue();
    _updateFluidSources();

    //_materialGrid.setAir(_fluidCellIndices);

    for (int k = 1; k < _materialGrid.depth - 1; k++) {
        for (int j = 1; j < _materialGrid.height - 1; j++) {
            for (int i = 1; i < _materialGrid.width - 1; i++) {
                if (_materialGrid.isCellFluid(i, j, k)) {
                    _materialGrid.setAir(i, j, k);
                }
            }
        }
    }

    _fluidCellIndices.clear();
    
    MarkerParticle p;
    GridIndex g;
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        p = _markerParticles[i];
        g = Grid3d::positionToGridIndex(p.position, _dx);
        FLUIDSIM_ASSERT(!_materialGrid.isCellSolid(g));
        _materialGrid.setFluid(g);
    }

    int count = 0;
    for (int k = 0; k < _materialGrid.depth; k++) {
        for (int j = 0; j < _materialGrid.height; j++) {
            for (int i = 0; i < _materialGrid.width; i++) {
                if (_materialGrid.isCellFluid(i, j, k)) {
                    count++;
                }
            }
        }
    }

    _fluidCellIndices.reserve(count);
    for (int k = 0; k < _materialGrid.depth; k++) {
        for (int j = 0; j < _materialGrid.height; j++) {
            for (int i = 0; i < _materialGrid.width; i++) {
                if (_materialGrid.isCellFluid(i, j, k)) {
                    _fluidCellIndices.push_back(i, j, k);
                }
            }
        }
    }

}

/********************************************************************************
    2. Reconstruct Internal Fluid Surface
********************************************************************************/

void FluidSimulation::_polygonizeInternalSurface(TriangleMesh &surface, 
                                                 TriangleMesh &preview) {
    IsotropicParticleMesher mesher(_isize, _jsize, _ksize, _dx);

    double r = _markerParticleRadius*_markerParticleScale;
    mesher.setScalarFieldAccelerator(&_scalarFieldAccelerator);

    if (_isPreviewSurfaceMeshEnabled) {
        mesher.enablePreviewMesher(_previewdx);
    }

    surface = mesher.meshParticles(_markerParticles, _materialGrid, r);
    if (_isPreviewSurfaceMeshEnabled) {
        preview = mesher.getPreviewMesh();
    }
}

bool FluidSimulation::_isInternalFluidSurfaceNeeded() {
    bool isNeeded = _isIsotropicSurfaceMeshReconstructionEnabled && 
                    _outputFluidSurfaceSubdivisionLevel == 1;
    isNeeded |= _isAnisotropicSurfaceMeshReconstructionEnabled;
    isNeeded |= _isBrickOutputEnabled;
    isNeeded |= _isDiffuseMaterialOutputEnabled;
    isNeeded |= _isPreviewSurfaceMeshEnabled && 
                !_isIsotropicSurfaceMeshReconstructionEnabled;

    return isNeeded;
}

void FluidSimulation::_reconstructInternalFluidSurface() {
    if (!_isInternalFluidSurfaceNeeded()) {
        return;
    }

    _polygonizeInternalSurface(_surfaceMesh, _previewMesh);
    _surfaceMesh.removeMinimumTriangleCountPolyhedra(
                        _minimumSurfacePolyhedronTriangleCount);
}

/********************************************************************************
    3. Compute LevelSet Signed Distance Field
********************************************************************************/

bool FluidSimulation::_isLevelSetNeeded() {
    return _isAnisotropicSurfaceMeshReconstructionEnabled ||
           _isBrickOutputEnabled ||
           _isDiffuseMaterialOutputEnabled;
}

void FluidSimulation::_updateLevelSetSignedDistanceField() {
    if (!_isLevelSetNeeded()) {
        return;
    }

    _levelset.setSurfaceMesh(_surfaceMesh);
    int numLayers = _CFLConditionNumber + 2;
    _levelset.calculateSignedDistanceField(numLayers);
}

/********************************************************************************
    4.  Reconstruct Output Fluid Surface
********************************************************************************/

void FluidSimulation::_writeDiffuseMaterialToFile(std::string bubblefile,
                                                  std::string foamfile,
                                                  std::string sprayfile) {
    TriangleMesh bubbleMesh;
    TriangleMesh foamMesh;
    TriangleMesh sprayMesh;

    bubbleMesh.vertices.reserve(_diffuseMaterial.getNumBubbleParticles());
    foamMesh.vertices.reserve(_diffuseMaterial.getNumFoamParticles());
    sprayMesh.vertices.reserve(_diffuseMaterial.getNumSprayParticles());

    FragmentedVector<DiffuseParticle> *dps = _diffuseMaterial.getDiffuseParticles();

    DiffuseParticle dp;
    for (unsigned int i = 0; i < dps->size(); i++) {
        dp = dps->at(i);

        if (dp.type == DiffuseParticleType::bubble && _isBubbleDiffuseMaterialEnabled) {
            bubbleMesh.vertices.push_back(dp.position);
        } else if (dp.type == DiffuseParticleType::foam && _isFoamDiffuseMaterialEnabled) {
            foamMesh.vertices.push_back(dp.position);
        } else if (dp.type == DiffuseParticleType::spray && _isSprayDiffuseMaterialEnabled) {
            sprayMesh.vertices.push_back(dp.position);
        }
    }

    if (_isBubbleDiffuseMaterialEnabled) {
        bubbleMesh.translate(_domainOffset);
        _writeTriangleMeshToFile(bubbleMesh, bubblefile);
    }
    if (_isFoamDiffuseMaterialEnabled) {
        foamMesh.translate(_domainOffset);
        _writeTriangleMeshToFile(foamMesh, foamfile);
    }
    if (_isSprayDiffuseMaterialEnabled) {
        sprayMesh.translate(_domainOffset);
        _writeTriangleMeshToFile(sprayMesh, sprayfile);
    }
}

void FluidSimulation::_writeDiffuseMaterialToFile(std::string diffusefile) {
    FragmentedVector<DiffuseParticle> *dps = _diffuseMaterial.getDiffuseParticles();

    TriangleMesh diffuseMesh;
    diffuseMesh.vertices.reserve(dps->size());

    DiffuseParticle dp;
    for (unsigned int i = 0; i < dps->size(); i++) {
        dp = dps->at(i);

        if (dp.type == DiffuseParticleType::bubble && _isBubbleDiffuseMaterialEnabled) {
            diffuseMesh.vertices.push_back(dp.position);
        } else if (dp.type == DiffuseParticleType::foam && _isFoamDiffuseMaterialEnabled) {
            diffuseMesh.vertices.push_back(dp.position);
        } else if (dp.type == DiffuseParticleType::spray && _isSprayDiffuseMaterialEnabled) {
            diffuseMesh.vertices.push_back(dp.position);
        }
    }

    diffuseMesh.translate(_domainOffset);
    _writeTriangleMeshToFile(diffuseMesh, diffusefile);
}

void FluidSimulation::_writeBrickColorListToFile(TriangleMesh &mesh, 
                                                     std::string filename) {
    int binsize = 3*sizeof(unsigned char)*(int)mesh.vertexcolors.size();
    char *storage = new char[binsize];

    vmath::vec3 c;
    for (unsigned int i = 0; i < mesh.vertexcolors.size(); i++) {
        c = mesh.vertexcolors[i];
        storage[3*i] = (unsigned char)(c.x*255.0);
        storage[3*i + 1] = (unsigned char)(c.y*255.0);
        storage[3*i + 2] = (unsigned char)(c.z*255.0);
    }
    
    std::ofstream erasefile;
    erasefile.open(filename, std::ofstream::out | std::ofstream::trunc);
    erasefile.close();

    std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
    file.write(storage, binsize);
    file.close();

    delete[] storage;
}

void FluidSimulation::_writeBrickTextureToFile(TriangleMesh &mesh, 
                                               std::string filename) {

    int bisize, bjsize, bksize;
    _fluidBrickGrid.getBrickGridDimensions(&bisize, &bjsize, &bksize);

    double bx = _fluidBrickGrid.getBrickAABB().width;
    double by = _fluidBrickGrid.getBrickAABB().height;
    double bz = _fluidBrickGrid.getBrickAABB().depth;

    Array3d<unsigned char> colorGrid(bisize, bjsize, bksize, (char)0);

    vmath::vec3 p, c;
    for (unsigned int i = 0; i < mesh.vertices.size(); i++) {
        p = mesh.vertices[i];
        c = mesh.vertexcolors[i];

        int bi = (int)(p.x / bx);
        int bj = (int)(p.y / by);
        int bk = (int)(p.z / bz);

        int coloridx = (int)(c.x*255);
        colorGrid.set(bi, bj, bk, (unsigned char)coloridx);
    }
    
    int binsize = sizeof(unsigned char)*bisize*bjsize*bksize;
    unsigned char *storage = new unsigned char[binsize];

    int offset = 0;
    for (int k = 0; k < colorGrid.depth; k++) {
        for (int j = 0; j < colorGrid.height; j++) {
            for (int i = 0; i < colorGrid.width; i++) {
                storage[offset] = colorGrid(i, j, k);
                offset++;
            }
        }
    }
    
    std::ofstream erasefile;
    erasefile.open(filename, std::ofstream::out | std::ofstream::trunc);
    erasefile.close();

    std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
    file.write((char*)storage, binsize);
    file.close();

    delete[] storage;
}

void FluidSimulation::_writeBrickMaterialToFile(std::string brickfile,
                                                std::string colorfile,
                                                std::string texturefile) {
    TriangleMesh brickmesh;
    _fluidBrickGrid.getBrickMesh(brickmesh);

    brickmesh.translate(_domainOffset);
    _writeTriangleMeshToFile(brickmesh, brickfile);
    _writeBrickColorListToFile(brickmesh, colorfile);
    _writeBrickTextureToFile(brickmesh, texturefile);
}

void FluidSimulation::_writeTriangleMeshToFile(TriangleMesh &mesh, std::string filename) {
    if (_meshOutputFormat == TriangleMeshFormat::ply) {
        mesh.writeMeshToPLY(filename);
    } else if (_meshOutputFormat == TriangleMeshFormat::bobj) {
        mesh.writeMeshToBOBJ(filename);
    }
}

std::string FluidSimulation::_numberToString(int number) {
    std::ostringstream ss;
    ss << number;
    return ss.str();
}

std::string FluidSimulation::_getFrameString(int number) {
    std::string currentFrame = _numberToString(number);
    currentFrame.insert(currentFrame.begin(), 6 - currentFrame.size(), '0');
    return currentFrame;
}

bool FluidSimulation::_isVertexNearSolid(vmath::vec3 v, double eps) {
    GridIndex g = Grid3d::positionToGridIndex(v, _dx);
    if (_materialGrid.isCellSolid(g)) {
        return true;
    }
    
    // is v near the solid boundary?
    vmath::vec3 e = vmath::vec3(eps, eps, eps);
    if (g.i == 1 || g.i == _isize - 2 || 
        g.j == 1 || g.j == _jsize - 2 || 
        g.k == 1 || g.k == _ksize - 2) {

        vmath::vec3 min = Grid3d::GridIndexToPosition(1, 1, 1, _dx);
        vmath::vec3 max = Grid3d::GridIndexToPosition(_isize - 2, 
                                                    _jsize - 2, 
                                                    _ksize - 2, _dx);
        AABB bbox = AABB(min + e, max + vmath::vec3(_dx, _dx, _dx) - e);
        if (!bbox.isPointInside(v)) {
            return true;
        }
    }

    // is v near a solid cell?
    vmath::vec3 gp = Grid3d::GridIndexToPosition(g, _dx);
    AABB bbox = AABB(gp + e, gp + vmath::vec3(_dx, _dx, _dx) - e);
    if (bbox.isPointInside(v)) {
        return false;
    }

    vmath::vec3 ex(eps, 0.0, 0.0);
    vmath::vec3 ey(0.0, eps, 0.0);
    vmath::vec3 ez(0.0, 0.0, eps);

    vmath::vec3 points[26] {
        v-ex, v+ex, v-ey, v+ey, v-ez, v+ez, v-ex-ey, v-ex+ey,
        v+ex-ey, v+ex+ey, v-ex-ez, v-ex+ez, v+ex-ez, v+ex+ez, 
        v-ey-ez, v-ey+ez, v+ey-ez, v+ey+ez, v-ex-ey-ez,
        v-ex-ey+ez, v-ex+ey-ez, v-ex+ey+ez, v+ex-ey-ez,
        v+ex-ey+ez, v+ex+ey-ez, v+ex+ey+ez};
    
    int i, j, k;
    for (int idx = 0; idx < 26; idx++) {
        Grid3d::positionToGridIndex(points[idx], _dx, &i, &j, &k);
        if (_materialGrid.isCellSolid(i, j, k)) {
            return true;
        }
    }

    return false;
}

void FluidSimulation::_getSmoothVertices(TriangleMesh &mesh,
                                         std::vector<int> &smoothVertices) {
    double eps = 0.02*_dx;
    vmath::vec3 v;
    for (unsigned int i = 0; i < mesh.vertices.size(); i++) {
        v = mesh.vertices[i];
        if (!_isVertexNearSolid(v, eps)) {
            smoothVertices.push_back(i);
        }
    }
}

void FluidSimulation::_smoothSurfaceMesh(TriangleMesh &mesh) {
    std::vector<int> smoothVertices;
    _getSmoothVertices(mesh, smoothVertices);

    mesh.smooth(_surfaceReconstructionSmoothingValue, 
                _surfaceReconstructionSmoothingIterations,
                smoothVertices);
}

void FluidSimulation::_polygonizeIsotropicOutputSurface(TriangleMesh &surface, 
                                                        TriangleMesh &preview) {
    int slices = _numSurfaceReconstructionPolygonizerSlices;
    double r = _markerParticleRadius*_markerParticleScale;

    IsotropicParticleMesher mesher(_isize, _jsize, _ksize, _dx);
    mesher.setScalarFieldAccelerator(&_scalarFieldAccelerator);
    mesher.setSubdivisionLevel(_outputFluidSurfaceSubdivisionLevel);
    mesher.setNumPolygonizationSlices(slices);

    bool isGeneratingPreview = _isPreviewSurfaceMeshEnabled &&
                               !_isInternalFluidSurfaceNeeded();
    if (isGeneratingPreview) {
        mesher.enablePreviewMesher(_previewdx);
    }

    surface = mesher.meshParticles(_markerParticles, _materialGrid, r);
    if (isGeneratingPreview) {
        preview = mesher.getPreviewMesh();
    }

}

TriangleMesh FluidSimulation::_polygonizeAnisotropicOutputSurface() {
    int slices = _numSurfaceReconstructionPolygonizerSlices;
    double r = _markerParticleRadius;

    AnisotropicParticleMesher mesher(_isize, _jsize, _ksize, _dx);
    mesher.setSubdivisionLevel(_outputFluidSurfaceSubdivisionLevel);
    mesher.setNumPolygonizationSlices(slices);

    return mesher.meshParticles(_markerParticles, _levelset, _materialGrid, r);
}

void FluidSimulation::_updateBrickGrid(double dt) {
    std::vector<vmath::vec3> points;
    points.reserve(_markerParticles.size());
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        points.push_back(_markerParticles[i].position);
    }

    _fluidBrickGrid.update(_levelset, _materialGrid, points, dt);
}

void FluidSimulation::_outputIsotropicSurfaceMesh() {
    if (!_isSurfaceMeshOutputEnabled) { return; }
    if (!_isIsotropicSurfaceMeshReconstructionEnabled) { return; }

    TriangleMesh isomesh, previewmesh;
    if (_outputFluidSurfaceSubdivisionLevel == 1) {
        isomesh = _surfaceMesh;
        previewmesh = _previewMesh;
    } else {
        _polygonizeIsotropicOutputSurface(isomesh, previewmesh);
        isomesh.removeMinimumTriangleCountPolyhedra(
            _minimumSurfacePolyhedronTriangleCount
        );
    }

    _smoothSurfaceMesh(isomesh);
    _smoothSurfaceMesh(previewmesh);
    isomesh.translate(_domainOffset);
    previewmesh.translate(_domainOffset);

    std::string framestr = _getFrameString(_currentFrame);
    std::string bakedir = Config::getBakefilesDirectory();
    std::string ext = "." + TriangleMesh::getFileExtension(_meshOutputFormat);
    std::string isofile = bakedir + "/" + framestr + ext;
    _writeTriangleMeshToFile(isomesh, isofile);

    if (_isPreviewSurfaceMeshEnabled) {
        std::string previewfile = bakedir + "/preview" + framestr + ext;
        _writeTriangleMeshToFile(previewmesh, previewfile);
    }
}

void FluidSimulation::_outputAnisotropicSurfaceMesh() {
    if (!_isSurfaceMeshOutputEnabled) { return; }
    if (!_isAnisotropicSurfaceMeshReconstructionEnabled) { return; }

    TriangleMesh anisomesh = _polygonizeAnisotropicOutputSurface();
    anisomesh.removeMinimumTriangleCountPolyhedra(
        _minimumSurfacePolyhedronTriangleCount
    );

    _smoothSurfaceMesh(anisomesh);
    anisomesh.translate(_domainOffset);

    std::string framestr = _getFrameString(_currentFrame);
    std::string bakedir = Config::getBakefilesDirectory();
    std::string ext = "." + TriangleMesh::getFileExtension(_meshOutputFormat);
    std::string anisofile = bakedir + "/anisotropic" + framestr + ext;
    _writeTriangleMeshToFile(anisomesh, anisofile);
}

void FluidSimulation::_outputDiffuseMaterial() {
    if (!_isDiffuseMaterialOutputEnabled) { return; }

    std::string framestr = _getFrameString(_currentFrame);
    std::string bakedir = Config::getBakefilesDirectory();
    std::string ext = "." + TriangleMesh::getFileExtension(_meshOutputFormat);

    if (_isDiffuseMaterialFilesSeparated) {
        _writeDiffuseMaterialToFile(bakedir + "/bubble" + framestr + ext,
                                    bakedir + "/foam" + framestr + ext,
                                    bakedir + "/spray" + framestr + ext);
    } else {
        _writeDiffuseMaterialToFile(bakedir + "/diffuse" + framestr + ext);
    }
}

void FluidSimulation::_outputBrickMesh(double dt) {
    if (!_isBrickOutputEnabled) { return; }

    _updateBrickGrid(dt);

    if (!_fluidBrickGrid.isBrickMeshReady()) { return; }

    std::string framestr = _getFrameString(_currentBrickMeshFrame);
    std::string bakedir = Config::getBakefilesDirectory();
    std::string ext = "." + TriangleMesh::getFileExtension(_meshOutputFormat);

    _writeBrickMaterialToFile(bakedir + "/brick" + framestr + ext, 
                              bakedir + "/brickcolor" + framestr + ".data",
                              bakedir + "/bricktexture" + framestr + ".data");
    _currentBrickMeshFrame++;
}

void FluidSimulation::_reconstructOutputFluidSurface(double dt) {
    _outputIsotropicSurfaceMesh();
    _outputAnisotropicSurfaceMesh();
    _outputDiffuseMaterial();
    _outputBrickMesh(dt);
}

/********************************************************************************
    5.  Advect Velocity Field
********************************************************************************/

void FluidSimulation::_applyFluidSourceToVelocityField(FluidSource *source,
                                                       int dir,
                                                       Array3d<bool> &isValueSet,
                                                       Array3d<float> &field) {
    int U = 0; int V = 1; int W = 2;

    vmath::vec3 (*getFacePosition)(int, int, int, double);
    if (dir == U) {
        getFacePosition = &Grid3d::FaceIndexToPositionU;
    } else if (dir == V) {
        getFacePosition = &Grid3d::FaceIndexToPositionV;
    } else if (dir == W) {
        getFacePosition = &Grid3d::FaceIndexToPositionW;
    } else {
        return;
    }

    double speed = source->getVelocity()[dir];
    vmath::vec3 p;
    for (int k = 0; k < field.depth; k++) {
        for (int j = 0; j < field.height; j++) {
            for (int i = 0; i < field.width; i++) {

                if (!isValueSet(i, j, k)) {
                    continue;
                }

                p = getFacePosition(i, j, k, _dx);
                if (source->containsPoint(p)) {
                    field.set(i, j, k, speed);
                }
            }
        }
    }

}

void FluidSimulation::_computeVelocityScalarField(Array3d<float> &field, 
                                                  Array3d<bool> &isValueSet,
                                                  int dir) {
    int U = 0; int V = 1; int W = 2;

    ScalarField grid = ScalarField(field.width,
                                                                 field.height,
                                                                 field.depth, _dx);
    grid.enableWeightField();
    grid.setPointRadius(_dx); 

    vmath::vec3 offset;
    if (dir == U) {
        offset = vmath::vec3(0.0, 0.5*_dx, 0.5*_dx);
    } else if (dir == V) {
        offset = vmath::vec3(0.5*_dx, 0.0, 0.5*_dx);
    } else if (dir == W) {
        offset = vmath::vec3(0.5*_dx, 0.5*_dx, 0.0);
    } else {
        return;
    }
    grid.setOffset(offset);

    int n = _maxParticlesPerVelocityAdvection;
    std::vector<vmath::vec3> positions;
    std::vector<float> values;
    positions.reserve(fmin(n, _markerParticles.size()));
    values.reserve(fmin(n, _markerParticles.size()));

    for (int startidx = 0; startidx < (int)_markerParticles.size(); startidx += n) {
        int endidx = startidx + n - 1;
        if (endidx >= (int)_markerParticles.size()) {
            endidx = _markerParticles.size() - 1;
        }

        positions.clear();
        values.clear();
        for (int i = startidx; i <= endidx; i++) {
            positions.push_back(_markerParticles[i].position);
            values.push_back(_markerParticles[i].velocity[dir]);
        }

        _scalarFieldAccelerator.addPointValues(positions, values, grid);
    }
    grid.applyWeightField();

    Array3d<float> *scalarfield = grid.getPointerToScalarField();
    Array3d<float> *weightfield = grid.getPointerToWeightField();

    double eps = 1e-9;
    for (int k = 0; k < field.depth; k++) {
        for (int j = 0; j < field.height; j++) {
            for (int i = 0; i < field.width; i++) {
                field.set(i, j, k, scalarfield->get(i, j, k));

                if (weightfield->get(i, j, k) > eps) {
                    isValueSet.set(i, j, k, true);
                }
            }
        }
    }

    FluidSource *source;
    for (unsigned int i = 0; i < _fluidSources.size(); i++) {
        source = _fluidSources[i];
        if (source->isInflow() && source->isActive()) {
            _applyFluidSourceToVelocityField(source, dir, isValueSet, field);
        }
    }
}

void FluidSimulation::_advectVelocityFieldU() {
    _MACVelocity.clearU();

    Array3d<float> ugrid = Array3d<float>(_isize + 1, _jsize, _ksize, 0.0f);
    Array3d<bool> isValueSet = Array3d<bool>(_isize + 1, _jsize, _ksize, false);
    _computeVelocityScalarField(ugrid, isValueSet, 0);

    GridIndexVector extrapolationIndices(_isize + 1, _jsize, _ksize);
    for (int k = 0; k < ugrid.depth; k++) {
        for (int j = 0; j < ugrid.height; j++) {
            for (int i = 0; i < ugrid.width; i++) {
                if (_materialGrid.isFaceBorderingFluidU(i, j, k)) {
                    if (!isValueSet(i, j, k)) {
                        extrapolationIndices.push_back(i, j, k);
                    } else {
                        _MACVelocity.setU(i, j, k, ugrid(i, j, k));
                    }
                }
            }
        }
    }

    GridIndex g, n;
    GridIndex nb[26];
    double avg, weight;
    for (unsigned int i = 0; i < extrapolationIndices.size(); i++) {
        g = extrapolationIndices[i];
        Grid3d::getNeighbourGridIndices26(g, nb);

        avg = 0.0;
        weight = 0.0;
        for (int idx = 0; idx < 26; idx++) {
            n = nb[idx];
            if (ugrid.isIndexInRange(n) && fabs(ugrid(n)) > 0.0) {
                avg += ugrid(n);
                weight += 1.0;
            }
        }

        if (weight > 0.0) {
            _MACVelocity.setU(g, avg / weight);
        }
    }
}

void FluidSimulation::_advectVelocityFieldV() {
    _MACVelocity.clearV();

    Array3d<float> vgrid = Array3d<float>(_isize, _jsize + 1, _ksize, 0.0f);
    Array3d<bool> isValueSet = Array3d<bool>(_isize, _jsize + 1, _ksize, false);
    _computeVelocityScalarField(vgrid, isValueSet, 1);
    
    GridIndexVector extrapolationIndices(_isize, _jsize + 1, _ksize);
    for (int k = 0; k < vgrid.depth; k++) {
        for (int j = 0; j < vgrid.height; j++) {
            for (int i = 0; i < vgrid.width; i++) {
                if (_materialGrid.isFaceBorderingFluidV(i, j, k)) {
                    if (!isValueSet(i, j, k)) {
                        extrapolationIndices.push_back(i, j, k);
                    } else {
                        _MACVelocity.setV(i, j, k, vgrid(i, j, k));
                    }
                }
            }
        }
    }

    GridIndex g, n;
    GridIndex nb[26];
    double avg, weight;
    for (unsigned int i = 0; i < extrapolationIndices.size(); i++) {
        g = extrapolationIndices[i];
        Grid3d::getNeighbourGridIndices26(g, nb);

        avg = 0.0;
        weight = 0.0;
        for (int idx = 0; idx < 26; idx++) {
            n = nb[idx];
            if (vgrid.isIndexInRange(n) && isValueSet(n)) {
                avg += vgrid(n);
                weight += 1.0;
            }
        }

        if (weight > 0.0) {
            _MACVelocity.setV(g, avg / weight);
        }
    }
}

void FluidSimulation::_advectVelocityFieldW() {
    _MACVelocity.clearW();

    Array3d<float> wgrid = Array3d<float>(_isize, _jsize, _ksize + 1, 0.0f);
    Array3d<bool> isValueSet = Array3d<bool>(_isize, _jsize, _ksize + 1, 0.0f);
    _computeVelocityScalarField(wgrid, isValueSet, 2);
    
    GridIndexVector extrapolationIndices(_isize, _jsize, _ksize + 1);
    for (int k = 0; k < wgrid.depth; k++) {
        for (int j = 0; j < wgrid.height; j++) {
            for (int i = 0; i < wgrid.width; i++) {
                if (_materialGrid.isFaceBorderingFluidW(i, j, k)) {
                    if (!isValueSet(i, j, k)) {
                        extrapolationIndices.push_back(i, j, k);
                    } else {
                        _MACVelocity.setW(i, j, k, wgrid(i, j, k));
                    }
                }
            }
        }
    }

    GridIndex g, n;
    GridIndex nb[26];
    double avg, weight;
    for (unsigned int i = 0; i < extrapolationIndices.size(); i++) {
        g = extrapolationIndices[i];
        Grid3d::getNeighbourGridIndices26(g, nb);

        avg = 0.0;
        weight = 0.0;
        for (int idx = 0; idx < 26; idx++) {
            n = nb[idx];
            if (wgrid.isIndexInRange(n) && isValueSet(n)) {
                avg += wgrid(n);
                weight += 1.0;
            }
        }

        if (weight > 0.0) {
            _MACVelocity.setW(g, avg / weight);
        }
    }
}

void FluidSimulation::_advectVelocityField() {
    _advectVelocityFieldU();
    _advectVelocityFieldV();
    _advectVelocityFieldW();
}

/********************************************************************************
    6. Apply Body Forces
********************************************************************************/

vmath::vec3 FluidSimulation::_getConstantBodyForce() {
    vmath::vec3 bf;
    for (unsigned int i = 0; i < _constantBodyForces.size(); i++) {
        bf += _constantBodyForces[i];
    }

    return bf;
}

vmath::vec3 FluidSimulation::_getVariableBodyForce(double px, double py, double pz) {
    return _getVariableBodyForce(vmath::vec3(px, py, pz));
}

vmath::vec3 FluidSimulation::_getVariableBodyForce(vmath::vec3 p) {
    vmath::vec3 fsum;
    vmath::vec3 (*fieldFunction)(vmath::vec3);
    for (unsigned int i = 0; i < _variableBodyForces.size(); i++) {
        fieldFunction = _variableBodyForces[i];
        fsum += fieldFunction(p);
    }

    return fsum;
}

void FluidSimulation::_applyConstantBodyForces(double dt) {

    vmath::vec3 bodyForce = _getConstantBodyForce();

    if (fabs(bodyForce.x) > 0.0) {
        for (int k = 0; k < _ksize; k++) {
            for (int j = 0; j < _jsize; j++) {
                for (int i = 0; i < _isize + 1; i++) {
                    if (_materialGrid.isFaceBorderingFluidU(i, j, k)) {
                        _MACVelocity.addU(i, j, k, bodyForce.x * dt);
                    }
                }
            }
        }
    }

    if (fabs(bodyForce.y) > 0.0) {
        for (int k = 0; k < _ksize; k++) {
            for (int j = 0; j < _jsize + 1; j++) {
                for (int i = 0; i < _isize; i++) {
                    if (_materialGrid.isFaceBorderingFluidV(i, j, k)) {
                        _MACVelocity.addV(i, j, k, bodyForce.y * dt);
                    }
                }
            }
        }
    }

    if (fabs(bodyForce.z) > 0.0) {
        for (int k = 0; k < _ksize + 1; k++) {
            for (int j = 0; j < _jsize; j++) {
                for (int i = 0; i < _isize; i++) {
                    if (_materialGrid.isFaceBorderingFluidW(i, j, k)) {
                        _MACVelocity.addW(i, j, k, bodyForce.z * dt);
                    }
                }
            }
        }
    }
}

void FluidSimulation::_applyVariableBodyForce(vmath::vec3 (*fieldFunction)(vmath::vec3),
                                              double dt) {

    vmath::vec3 p;
    vmath::vec3 bodyForce;
    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize + 1; i++) {
                if (_materialGrid.isFaceBorderingFluidU(i, j, k)) {
                    p = Grid3d::FaceIndexToPositionU(i, j, k, _dx);
                    bodyForce = fieldFunction(p);
                    _MACVelocity.addU(i, j, k, bodyForce.x * dt);
                }
            }
        }
    }


    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize + 1; j++) {
            for (int i = 0; i < _isize; i++) {
                if (_materialGrid.isFaceBorderingFluidV(i, j, k)) {
                    p = Grid3d::FaceIndexToPositionV(i, j, k, _dx);
                    bodyForce = fieldFunction(p);
                    _MACVelocity.addV(i, j, k, bodyForce.y * dt);
                }
            }
        }
    }


    for (int k = 0; k < _ksize + 1; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                if (_materialGrid.isFaceBorderingFluidW(i, j, k)) {
                    p = Grid3d::FaceIndexToPositionW(i, j, k, _dx);
                    bodyForce = fieldFunction(p);
                    _MACVelocity.addW(i, j, k, bodyForce.z * dt);
                }
            }
        }
    }

}

void FluidSimulation::_applyVariableBodyForces(double dt) {
    vmath::vec3 (*fieldFunction)(vmath::vec3);
    for (unsigned int i = 0; i < _variableBodyForces.size(); i++) {
        fieldFunction = _variableBodyForces[i];
        _applyVariableBodyForce(fieldFunction, dt);
    }
}

void FluidSimulation::_applyBodyForcesToVelocityField(double dt) {
    _applyConstantBodyForces(dt);
    _applyVariableBodyForces(dt);
}


/********************************************************************************
    7. Pressure Solve
********************************************************************************/

void FluidSimulation::_updatePressureGrid(Array3d<float> &pressureGrid, double dt) {
    PressureSolverParameters params;
    params.cellwidth = _dx;
    params.density = _density;
    params.deltaTime = dt;
    params.fluidCells = &_fluidCellIndices;
    params.materialGrid = &_materialGrid;
    params.velocityField = &_MACVelocity;
    params.logfile = &_logfile;

    VectorXd pressures((int)_fluidCellIndices.size());
    PressureSolver solver;
    solver.solve(params, pressures);

    GridIndex g;
    for (unsigned int idx = 0; idx < _fluidCellIndices.size(); idx++) {
        g = _fluidCellIndices[idx];
        pressureGrid.set(g, (float)pressures[idx]);
    }
}

/********************************************************************************
    8. Apply Pressure
********************************************************************************/

void FluidSimulation::_applyPressureToFaceU(int i, int j, int k, 
                                            Array3d<float> &pressureGrid,
                                            MACVelocityField &tempMACVelocity, double dt) {
    double usolid = 0.0;   // solids are stationary
    double scale = dt / (_density * _dx);
    double invscale = 1.0 / scale;

    int ci = i - 1; int cj = j; int ck = k;

    double p0, p1;
    if (!_materialGrid.isCellSolid(ci, cj, ck) && !_materialGrid.isCellSolid(ci + 1, cj, ck)) {
        p0 = pressureGrid(ci, cj, ck);
        p1 = pressureGrid(ci + 1, cj, ck);
    } else if (_materialGrid.isCellSolid(ci, cj, ck)) {
        p0 = pressureGrid(ci + 1, cj, ck) - 
                invscale*(_MACVelocity.U(i, j, k) - usolid);
        p1 = pressureGrid(ci + 1, cj, ck);
    } else {
        p0 = pressureGrid(ci, cj, ck);
        p1 = pressureGrid(ci, cj, ck) +
                invscale*(_MACVelocity.U(i, j, k) - usolid);
    }

    double unext = _MACVelocity.U(i, j, k) - scale*(p1 - p0);
    tempMACVelocity.setU(i, j, k, unext);
}

void FluidSimulation::_applyPressureToFaceV(int i, int j, int k, 
                                            Array3d<float> &pressureGrid,
                                            MACVelocityField &tempMACVelocity, double dt) {
    double usolid = 0.0;   // solids are stationary
    double scale = dt / (_density * _dx);
    double invscale = 1.0 / scale;

    int ci = i; int cj = j - 1; int ck = k;

    double p0, p1;
    if (!_materialGrid.isCellSolid(ci, cj, ck) && !_materialGrid.isCellSolid(ci, cj + 1, ck)) {
        p0 = pressureGrid(ci, cj, ck);
        p1 = pressureGrid(ci, cj + 1, ck);
    }
    else if (_materialGrid.isCellSolid(ci, cj, ck)) {
        p0 = pressureGrid(ci, cj + 1, ck) -
            invscale*(_MACVelocity.V(i, j, k) - usolid);
        p1 = pressureGrid(ci, cj + 1, ck);
    }
    else {
        p0 = pressureGrid(ci, cj, ck);
        p1 = pressureGrid(ci, cj, ck) +
            invscale*(_MACVelocity.V(i, j, k) - usolid);
    }

    double vnext = _MACVelocity.V(i, j, k) - scale*(p1 - p0);
    tempMACVelocity.setV(i, j, k, vnext);
}

void FluidSimulation::_applyPressureToFaceW(int i, int j, int k, 
                                            Array3d<float> &pressureGrid,
                                            MACVelocityField &tempMACVelocity, double dt) {
    double usolid = 0.0;   // solids are stationary
    double scale = dt / (_density * _dx);
    double invscale = 1.0 / scale;

    int ci = i; int cj = j; int ck = k - 1;

    double p0, p1;
    if (!_materialGrid.isCellSolid(ci, cj, ck) && !_materialGrid.isCellSolid(ci, cj, ck + 1)) {
        p0 = pressureGrid(ci, cj, ck);
        p1 = pressureGrid(ci, cj, ck + 1);
    }
    else if (_materialGrid.isCellSolid(ci, cj, ck)) {
        p0 = pressureGrid(ci, cj, ck + 1) -
                invscale*(_MACVelocity.W(i, j, k) - usolid);
        p1 = pressureGrid(ci, cj, ck + 1);
    }
    else {
        p0 = pressureGrid(ci, cj, ck);
        p1 = pressureGrid(ci, cj, ck) +
                invscale*(_MACVelocity.W(i, j, k) - usolid);
    }

    double wnext = _MACVelocity.W(i, j, k) - scale*(p1 - p0);
    tempMACVelocity.setW(i, j, k, wnext);
}

void FluidSimulation::_commitTemporaryVelocityFieldValues(MACVelocityField &tempMACVelocity) {
    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize + 1; i++) {
                if (_materialGrid.isFaceBorderingFluidU(i, j, k)) {
                    _MACVelocity.setU(i, j, k, tempMACVelocity.U(i, j, k));
                }
            }
        }
    }

    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize + 1; j++) {
            for (int i = 0; i < _isize; i++) {
                if (_materialGrid.isFaceBorderingFluidV(i, j, k)) {
                    _MACVelocity.setV(i, j, k, tempMACVelocity.V(i, j, k));
                }
            }
        }
    }

    for (int k = 0; k < _ksize + 1; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                if (_materialGrid.isFaceBorderingFluidW(i, j, k)) {
                    _MACVelocity.setW(i, j, k, tempMACVelocity.W(i, j, k));
                }
            }
        }
    }
}

void FluidSimulation::_applyPressureToVelocityField(Array3d<float> &pressureGrid, double dt) {
    MACVelocityField tempMACVelocity = MACVelocityField(_isize, _jsize, _ksize, _dx);

    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize + 1; i++) {
                if (_materialGrid.isFaceBorderingSolidU(i, j, k)) {
                    tempMACVelocity.setU(i, j, k, 0.0);
                }

                if (_materialGrid.isFaceBorderingFluidU(i, j, k) &&
                    !_materialGrid.isFaceBorderingSolidU(i, j, k)) {
                    _applyPressureToFaceU(i, j, k, pressureGrid, tempMACVelocity, dt);
                }
            }
        }
    }

    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize + 1; j++) {
            for (int i = 0; i < _isize; i++) {
                if (_materialGrid.isFaceBorderingSolidV(i, j, k)) {
                    tempMACVelocity.setV(i, j, k, 0.0);
                }

                if (_materialGrid.isFaceBorderingFluidV(i, j, k) &&
                    !_materialGrid.isFaceBorderingSolidV(i, j, k)) {
                    _applyPressureToFaceV(i, j, k, pressureGrid, tempMACVelocity, dt);
                }
            }
        }
    }

    for (int k = 0; k < _ksize + 1; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                if (_materialGrid.isFaceBorderingSolidW(i, j, k)) {
                    tempMACVelocity.setW(i, j, k, 0.0);
                }

                if (_materialGrid.isFaceBorderingFluidW(i, j, k) &&
                    !_materialGrid.isFaceBorderingSolidW(i, j, k)) {
                    _applyPressureToFaceW(i, j, k, pressureGrid, tempMACVelocity, dt);
                }
            }
        }
    }

    _commitTemporaryVelocityFieldValues(tempMACVelocity);
}

/********************************************************************************
    9. Extrapolate Velocity Field
********************************************************************************/

void FluidSimulation::_extrapolateFluidVelocities(MACVelocityField &MACGrid) {
    int numLayers = (int)ceil(_CFLConditionNumber + 2);
    MACGrid.extrapolateVelocityField(_materialGrid, numLayers);
}

/********************************************************************************
    10. Update Diffuse Particle Simulation
********************************************************************************/

void FluidSimulation::_updateDiffuseMaterial(double dt) {

    vmath::vec3 bodyForce = _getConstantBodyForce();

    _diffuseMaterial.update(_isize, _jsize, _ksize, _dx,
                            &_markerParticles,
                            &_MACVelocity, 
                            &_levelset, 
                            &_materialGrid,
                            &_particleAdvector,
                            bodyForce,
                            dt);

    int spraycount, bubblecount, foamcount;
    _diffuseMaterial.getDiffuseParticleTypeCounts(&spraycount, 
                                                  &bubblecount, 
                                                  &foamcount);

    _logfile.log("Num Diffuse Particles: ", (int)getNumDiffuseParticles(), 1);
    _logfile.log("NUM SPRAY:  ", spraycount, 2);
    _logfile.log("NUM BUBBLE: ", bubblecount, 2);
    _logfile.log("NUM FOAM:   ", foamcount, 2);
}

/********************************************************************************
    11. Update MarkerParticle Velocities
********************************************************************************/

void FluidSimulation::_updateRangeOfMarkerParticleVelocities(int startIdx, int endIdx) {
    int size = endIdx - startIdx + 1;
    std::vector<vmath::vec3> positions, vnew, vold;
    positions.reserve(size);
    vnew.reserve(size);
    vold.reserve(size);

    for (int i = startIdx; i <= endIdx; i++) {
        positions.push_back(_markerParticles[i].position);
    }

    _particleAdvector.tricubicInterpolate(positions, &_MACVelocity, vnew);
    _particleAdvector.tricubicInterpolate(positions, &_savedVelocityField, vold);

    vmath::vec3 vPIC, vFLIP, v;
    MarkerParticle mp;
    for (unsigned int i = 0; i < positions.size(); i++) {
        mp = _markerParticles[startIdx + i];

        vPIC = vnew[i];
        vFLIP = mp.velocity + vnew[i] - vold[i];

        v = (float)_ratioPICFLIP * vPIC + (float)(1 - _ratioPICFLIP) * vFLIP;
        _markerParticles[startIdx + i].velocity = v;
    }
}

void FluidSimulation::_updateMarkerParticleVelocities() {
    int n = _maxParticlesPerPICFLIPUpdate;
    for (int startidx = 0; startidx < (int)_markerParticles.size(); startidx += n) {
        int endidx = startidx + n - 1;
        endidx = fmin(endidx, _markerParticles.size() - 1);

        _updateRangeOfMarkerParticleVelocities(startidx, endidx);
    }
}

/********************************************************************************
    12. Advance MarkerParticles
********************************************************************************/

vmath::vec3 FluidSimulation::_resolveParticleSolidCellCollision(vmath::vec3 p0, vmath::vec3 p1) {
    
    GridIndex g1 = Grid3d::positionToGridIndex(p0, _dx);
    GridIndex g2 = Grid3d::positionToGridIndex(p1, _dx);
    FLUIDSIM_ASSERT(!_materialGrid.isCellSolid(g1));
    FLUIDSIM_ASSERT(_materialGrid.isCellSolid(g2));

    GridIndex voxel;
    bool foundVoxel = Collision::getLineSegmentVoxelIntersection(p0, p1, _dx,
                                                                 _materialGrid, 
                                                                 &voxel);
    if (!foundVoxel) {
        return p0;
    }

    vmath::vec3 raynorm = vmath::normalize(p1 - p0);
    vmath::vec3 vpos = Grid3d::GridIndexToPosition(voxel, _dx);
    AABB bbox(vpos, _dx, _dx, _dx);

    vmath::vec3 cpoint;
    bool foundCollision = Collision::rayIntersectsAABB(p0, raynorm, bbox, &cpoint);

    if (!foundCollision) {
        return p0;
    }

    vmath::vec3 resolvedPosition = cpoint - (float)(0.05*_dx)*raynorm;

    GridIndex gr = Grid3d::positionToGridIndex(resolvedPosition, _dx);
    if (_materialGrid.isCellSolid(gr)) {
        return p0;
    }
    
    return resolvedPosition;
}

void FluidSimulation::_advanceRangeOfMarkerParticles(int startIdx, int endIdx, 
                                                     double dt) {
    FLUIDSIM_ASSERT(startIdx <= endIdx);

    std::vector<vmath::vec3> positions;
    positions.reserve(endIdx - startIdx + 1);
    for (int i = startIdx; i <= endIdx; i++) {
        positions.push_back(_markerParticles[i].position);
    }

    std::vector<vmath::vec3> output;
    _particleAdvector.advectParticlesRK4(positions, &_MACVelocity, dt, 
                                         output);

    MarkerParticle mp;
    vmath::vec3 nextp;
    GridIndex g;
    for (unsigned int i = 0; i < output.size(); i++) {
        mp = _markerParticles[startIdx + i];
        nextp = output[i];

        g = Grid3d::positionToGridIndex(nextp, _dx);
        if (_materialGrid.isCellSolid(g)) {
            nextp = _resolveParticleSolidCellCollision(mp.position, nextp);
        }

        _markerParticles[startIdx + i].position = nextp;
    }
}

void FluidSimulation::_shuffleMarkerParticleOrder() {
    MarkerParticle mi;
    for (int i = _markerParticles.size() - 2; i >= 0; i--) {
        int j = (rand() % (int)(i - 0 + 1));
        mi = _markerParticles[i];
        _markerParticles[i] = _markerParticles[j];
        _markerParticles[j] = mi;
    }
}

void FluidSimulation::_removeMarkerParticles() {
    Array3d<int> countGrid = Array3d<int>(_isize, _jsize, _ksize, 0);
    _shuffleMarkerParticleOrder();

    std::vector<bool> isRemoved;
    isRemoved.reserve(_markerParticles.size());

    MarkerParticle mp;
    GridIndex g;
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        mp = _markerParticles[i];
        g = Grid3d::positionToGridIndex(mp.position, _dx);
        if (countGrid(g) >= _maxMarkerParticlesPerCell) {
            isRemoved.push_back(true);
            continue;
        }
        countGrid.add(g, 1);

        isRemoved.push_back(false);
    }

    _removeItemsFromVector(_markerParticles, isRemoved);
}

void FluidSimulation::_advanceMarkerParticles(double dt) {

    int n = _maxParticlesPerParticleAdvection;
    for (int startidx = 0; startidx < (int)_markerParticles.size(); startidx += n) {
        int endidx = startidx + n - 1;
        endidx = fmin(endidx, _markerParticles.size() - 1);

        _advanceRangeOfMarkerParticles(startidx, endidx, dt);
    }

    _removeMarkerParticles();
}

/********************************************************************************
    TIME STEP
********************************************************************************/

void FluidSimulation::_stepFluid(double dt) {

    _simulationTime += dt;

    _logfile.separator();
    _logfile.timestamp();
    _logfile.newline();
    _logfile.log("Frame: ", _currentFrame, 0);
    _logfile.log("Step time: ", dt, 4);
    _logfile.newline();

    std::vector<StopWatch> timers(13);
    timers[0].start();

    timers[1].start();
    _updateFluidCells();
    timers[1].stop();

    _logfile.log("Update Fluid Cells:          \t", timers[1].getTime(), 4);
    _logfile.log("Num Fluid Cells: \t", (int)_fluidCellIndices.size(), 4, 1);
    _logfile.log("Num Marker Particles: \t", (int)_markerParticles.size(), 4, 1);

    timers[2].start();
    _reconstructInternalFluidSurface();
    timers[2].stop();

    _logfile.log("Reconstruct Fluid Surface:  \t", timers[2].getTime(), 4);

    timers[3].start();
    _updateLevelSetSignedDistanceField();
    timers[3].stop();

    _logfile.log("Update Level set:           \t", timers[3].getTime(), 4);

    timers[4].start();
    if (_isFirstTimeStepForFrame) {
        _reconstructOutputFluidSurface(_currentFrameTimeStep);
    }
    timers[4].stop();

    _logfile.log("Reconstruct Output Surface: \t", timers[4].getTime(), 4);

    timers[5].start();
    _advectVelocityField();
    _savedVelocityField = _MACVelocity;
    _extrapolateFluidVelocities(_savedVelocityField);
    timers[5].stop();

    _logfile.log("Advect Velocity Field:       \t", timers[5].getTime(), 4);

    timers[6].start();
    _applyBodyForcesToVelocityField(dt);
    timers[6].stop();

    _logfile.log("Apply Body Forces:           \t", timers[6].getTime(), 4);

    {
        timers[7].start();
        Array3d<float> pressureGrid = Array3d<float>(_isize, _jsize, _ksize, 0.0f);
        _updatePressureGrid(pressureGrid, dt);
        timers[7].stop();

        _logfile.log("Update Pressure Grid:        \t", timers[7].getTime(), 4);

        timers[8].start();
        _applyPressureToVelocityField(pressureGrid, dt);
        timers[8].stop();

        _logfile.log("Apply Pressure:              \t", timers[8].getTime(), 4);
    }

    timers[9].start();
    _extrapolateFluidVelocities(_MACVelocity);
    timers[9].stop();

    _logfile.log("Extrapolate Fluid Velocities:\t", timers[9].getTime(), 4);

    timers[10].start();
    if (_isDiffuseMaterialOutputEnabled) {
        _updateDiffuseMaterial(dt);
    }
    timers[10].stop();

    _logfile.log("Update Diffuse Material:     \t", timers[10].getTime(), 4);

    timers[11].start();
    _updateMarkerParticleVelocities();
    _savedVelocityField = MACVelocityField();
    
    timers[11].stop();

    _logfile.log("Update PIC/FLIP Velocities:  \t", timers[11].getTime(), 4);

    timers[12].start();
    _advanceMarkerParticles(dt);
    timers[12].stop();

    _logfile.log("Advance Marker Particles:    \t", timers[12].getTime(), 4);

    timers[0].stop();

    double totalTime = floor(timers[0].getTime()*1000.0) / 1000.0;
    _realTime += totalTime;
    _logfile.newline();

    std::vector<double> percentages(timers.size(), 0.0);
    for (unsigned int i = 0; i < timers.size(); i++) {
        percentages[i] = floor(1000 * timers[i].getTime() / totalTime) / 10.0;
    }

    _logfile.log("---Percentage Breakdown---", "");
    _logfile.log("Update Fluid Cells:          \t", percentages[1], 3);
    _logfile.log("Reconstruct Fluid Surface:   \t", percentages[2], 3);
    _logfile.log("Update Level Set:            \t", percentages[3], 3);
    _logfile.log("Reconstruct Output Surface:  \t", percentages[4], 3);
    _logfile.log("Advect Velocity Field:       \t", percentages[5], 3);
    _logfile.log("Apply Body Forces:           \t", percentages[6], 3);
    _logfile.log("Update Pressure Grid:        \t", percentages[7], 3);
    _logfile.log("Apply Pressure:              \t", percentages[8], 3);
    _logfile.log("Extrapolate Fluid Velocities:\t", percentages[9], 3);
    _logfile.log("Update Diffuse Material:     \t", percentages[10], 3);
    _logfile.log("Update PIC/FLIP Velocities:  \t", percentages[11], 3);
    _logfile.log("Advance Marker Particles:    \t", percentages[12], 3);
    _logfile.newline();

    _logfile.log("Update time:   ", totalTime, 3);
    _logfile.log("Total time:    ", _realTime, 3);
    _logfile.newline();
}

double FluidSimulation::_getMaximumMarkerParticleSpeed() {
    double maxsq = 0.0;
    MarkerParticle mp;
    for (unsigned int i = 0; i < _markerParticles.size(); i++) {
        double distsq = vmath::dot(mp.velocity, mp.velocity);
        if (distsq > maxsq) {
            maxsq = distsq;
        }
    }

    return sqrt(maxsq);
}

double FluidSimulation::_calculateNextTimeStep() {
    double maxu = _getMaximumMarkerParticleSpeed();
    double timeStep = _CFLConditionNumber*_dx / maxu;

    return timeStep;
}

void FluidSimulation::_autosave() {
    std::string dir = Config::getSavestatesDirectory();
    saveState(dir + "/autosave.state");
}

void FluidSimulation::update(double dt) {
    if (!_isSimulationInitialized) {
        std::string msg = "Error: FluidSimulation must be initialized before update.\n";
        throw std::runtime_error(msg);
    }

    if (dt < 0.0) {
        std::string msg = "Error: delta time must be greater than or equal to 0.\n";
        msg += "delta time: " + _toString(dt) + "\n";
        throw std::domain_error(msg);
    }

    _isCurrentFrameFinished = false;

    _currentFrameTimeStep = dt;

    if (_isAutosaveEnabled) {
        _autosave();
    }

    _currentTimeStep = 0;
    double timeleft = dt;
    while (timeleft > 0.0) {
        double timestep = _calculateNextTimeStep();
        if (timeleft - timestep < 0.0) {
            timestep = timeleft;
        }
        timeleft -= timestep;

        _isFirstTimeStepForFrame = _currentTimeStep == 0;

        _stepFluid(timestep);

        _currentTimeStep++;
    }
    _currentFrame++;

    _isCurrentFrameFinished = true;
}