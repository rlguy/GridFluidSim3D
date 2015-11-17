/*
Copyright (c) 2015 Ryan L. Guy

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

#include "particlemesher.h"


ParticleMesher::ParticleMesher() {
}

ParticleMesher::ParticleMesher(int isize, int jsize, int ksize, double dx) :
                                    _isize(isize), _jsize(jsize), _ksize(ksize), _dx(dx) {
}

ParticleMesher::~ParticleMesher() {
}

TriangleMesh ParticleMesher::meshParticles(std::vector<glm::vec3> &particles, 
                                           LevelSet &levelset,
                                           double particleRadius) {
    _clear();
    _setParticleRadius(particleRadius);

    StopWatch t;

    t.start();
    std::vector<glm::vec3> insideParticles;
    std::vector<glm::vec3> surfaceParticles;
    std::vector<int> nearSurfaceParticles;
    std::vector<int> farSurfaceParticles;
    _sortParticlesBySurfaceDistance(particles, 
                                    insideParticles, 
                                    surfaceParticles, nearSurfaceParticles, farSurfaceParticles,
                                    levelset);

    std::cout << "\tINSIDE: " << insideParticles.size() << std::endl;
    std::cout << "\tSURFACE: " << surfaceParticles.size() << std::endl;
    std::cout << "\tNEAR: " << nearSurfaceParticles.size() << std::endl;
    std::cout << "\tFAR: " << farSurfaceParticles.size() << std::endl;

    t.stop();

    std::cout << "\tSORT PARTICLES " << t.getTime() << "\n" << std::endl;

    t.reset();
    t.start();
    _initializeSurfaceParticleSpatialGrid(surfaceParticles);
    _updateNearFarSurfaceParticleReferences(nearSurfaceParticles, farSurfaceParticles);
    t.stop();

    std::cout << "\tINITIALIZE GRID " << t.getTime() << "\n" << std::endl;

    t.reset();
    t.start();
    _updateSurfaceParticleComponentIDs();
    t.stop();

    std::cout << "\tUPDATE COMPONENTS " << t.getTime() << "\n" << std::endl;

    t.reset();
    t.start();
    _smoothSurfaceParticlePositions();
    t.stop();

    std::cout << "\tSMOOTH POSITIONS " << t.getTime() << "\n" << std::endl;

    t.reset();
    t.start();
    std::vector<glm::mat3x3> anisotropyMatrices;
    _computeAnisotropyMatrices(anisotropyMatrices);
    t.stop();

    std::cout << "\tCOMPUTE ANISOTROPY " << t.getTime() << "\n" << std::endl;

    return TriangleMesh();
}

void ParticleMesher::_clear() {
    _surfaceParticles.clear();
    _nearSurfaceParticleRefs.clear();
    _farSurfaceParticleRefs.clear();
    _pointGrid = SpatialPointGrid();
    _smoothedPositions.clear();
    _covarianceMatrices.clear();
}

void ParticleMesher::_sortParticlesBySurfaceDistance(std::vector<glm::vec3> &allParticles,
                                                     std::vector<glm::vec3> &insideParticles,
                                                     std::vector<glm::vec3> &surfaceParticles,
                                                     std::vector<int> &nearSurfaceParticles,
                                                     std::vector<int> &farSurfaceParticles,
                                                     LevelSet &levelset) {

    double supportRadius = _particleRadius*_supportRadiusFactor;
    double maxNearSurfaceDepth = _dx*_maxParticleToSurfaceDepth;
    double maxSurfaceDepth = maxNearSurfaceDepth + supportRadius;
    glm::vec3 p;
    for (unsigned int i = 0; i < allParticles.size(); i++) {
        p = allParticles[i];

        double dist = levelset.getSignedDistance(p);
        if (dist < 0.0) {
            surfaceParticles.push_back(p);
            nearSurfaceParticles.push_back(surfaceParticles.size()-1);
        } else {
            if (dist < maxSurfaceDepth) {
                surfaceParticles.push_back(p);
                if (dist < maxNearSurfaceDepth) {
                    nearSurfaceParticles.push_back(surfaceParticles.size()-1);
                } else {
                    farSurfaceParticles.push_back(surfaceParticles.size()-1);
                }
            } else {
                insideParticles.push_back(p);
            }
        }
    }
}

void ParticleMesher::_initializeSurfaceParticleSpatialGrid(std::vector<glm::vec3> &particles) {
    _pointGrid = SpatialPointGrid(_isize, _jsize, _ksize, _dx);
    std::vector<GridPointReference> refs = _pointGrid.insert(particles);

    SurfaceParticle sp;
    for (unsigned int i = 0; i < particles.size(); i++) {
        sp = SurfaceParticle(particles[i], refs[i]);
        _surfaceParticles.push_back(sp);
    }
}

void ParticleMesher::_updateNearFarSurfaceParticleReferences(std::vector<int> nearParticles,
                                             std::vector<int> farParticles) {
    SurfaceParticle sp;
    for (unsigned int i = 0; i < nearParticles.size(); i++) {
        sp = _surfaceParticles[nearParticles[i]];
        _nearSurfaceParticleRefs.push_back(sp.ref);
    }

    for (unsigned int i = 0; i < farParticles.size(); i++) {
        sp = _surfaceParticles[farParticles[i]];
        _farSurfaceParticleRefs.push_back(sp.ref);
    }
}

void ParticleMesher::_updateSurfaceParticleComponentIDs() {
    double r = _particleRadius*_connectedComponentRadiusFactor;
    std::vector<std::vector<GridPointReference> > components;
    _pointGrid.getConnectedPointReferenceComponents(r, components);

    for (unsigned int cid = 0; cid < components.size(); cid++) {
        for (unsigned int idx = 0; idx < components[cid].size(); idx++) {
            int spidx = components[cid][idx].id;
            _surfaceParticles[spidx].componentID = cid;
        }
    }

    // debug
    int count = 0;
    for (unsigned int i = 0; i < components.size(); i++) {
        if (components[i].size() > 2) {
            count++;
        }
    }

    std::cout << "\t\tNUM COMPONENTS: " << components.size() << " " << count << std::endl;
}

void ParticleMesher::_smoothSurfaceParticlePositions() {
    _computeSmoothedNearSurfaceParticlePositions();

    GridPointReference ref;
    for (unsigned int i = 0; i < _nearSurfaceParticleRefs.size(); i++) {
        ref = _nearSurfaceParticleRefs[i];
        _surfaceParticles[ref.id].position = _smoothedPositions[i];
    }

    _smoothedPositions.clear();
    _smoothedPositions.shrink_to_fit();
}

void ParticleMesher::_computeSmoothedNearSurfaceParticlePositions() {
    int size = (int)_nearSurfaceParticleRefs.size();

    std::vector<int> startIndices;
    std::vector<int> endIndices;

    int numThreads = _numThreads;
    int chunksize = (int)floor(size / numThreads);
    for (int i = 0; i < numThreads; i++) {
        int startIdx = (i == 0) ? 0 : endIndices[i - 1] + 1;
        int endIdx = (i == numThreads - 1) ? size - 1 : startIdx + chunksize - 1;

        startIndices.push_back(startIdx);
        endIndices.push_back(endIdx);
    }

    double supportRadius = _supportRadiusFactor*_particleRadius;
    _setKernelRadius(supportRadius);
    _smoothedPositions = std::vector<glm::vec3>(_nearSurfaceParticleRefs.size(), 
                                                    glm::vec3(0.0, 0.0, 0.0));
    
    std::vector<std::thread> threads;
    for (int i = 0; i < numThreads; i++) {
        threads.push_back(std::thread(&ParticleMesher::_smoothRangeOfSurfaceParticlePositions,
                                      this,
                                      startIndices[i],
                                      endIndices[i],
                                      supportRadius));
    }

    for (int i = 0; i < numThreads; i++) {
        threads[i].join();
    }
    
}

void ParticleMesher::_smoothRangeOfSurfaceParticlePositions(int startidx, int endidx, double radius) {
    std::vector<GridPointReference> neighbourRefs;
    GridPointReference ref;
    glm::vec3 newp;
    for (int i = startidx; i <= endidx; i++) {
        ref = _nearSurfaceParticleRefs[i];
        newp = _getSmoothedParticlePosition(ref, radius, neighbourRefs);
        _smoothedPositions[i] = newp;
    }
}

glm::vec3 ParticleMesher::_getSmoothedParticlePosition(GridPointReference ref,
                                                       double radius,
                                                       std::vector<GridPointReference> &neighbourRefs) {

    neighbourRefs.clear();
    _pointGrid.queryPointReferencesInsideSphere(ref, radius, neighbourRefs);
    glm::vec3 mean = _getWeightedMeanParticlePosition(ref, neighbourRefs);

    SurfaceParticle spi = _surfaceParticles[ref.id]; 
    float k = (float)_smoothingConstant;
    return (1.0f - k)*spi.position + k*mean;
}

glm::vec3 ParticleMesher::_getWeightedMeanParticlePosition(GridPointReference ref,
                                                           std::vector<GridPointReference> &neighbours) {
    SurfaceParticle spi = _surfaceParticles[ref.id]; 
    SurfaceParticle spj;

    glm::vec3 posSum = glm::vec3(0.0, 0.0, 0.0);
    double weightSum = 0.0;
    double kernalVal;
    glm::vec3 v;

    GridPointReference refj;
    double eps = 1e-9;
    for (unsigned int i = 0; i < neighbours.size(); i++) {
        refj = neighbours[i];
        spj = _surfaceParticles[refj.id];

        kernalVal = _evaluateKernel(spi, spj);
        posSum += (float)kernalVal*spj.position;
        weightSum += kernalVal;
    }

    if (weightSum < eps) {
        return spi.position;
    }

    return posSum / (float)weightSum;
}

void ParticleMesher::_computeAnisotropyMatrices(std::vector<glm::mat3x3> &matrices) {
    _computeCovarianceMatrices();

    _covarianceMatrices.clear();
    _covarianceMatrices.shrink_to_fit();
}

void ParticleMesher::_computeCovarianceMatrices() {
    int size = (int)_nearSurfaceParticleRefs.size();

    std::vector<int> startIndices;
    std::vector<int> endIndices;

    int numThreads = _numThreads;
    int chunksize = (int)floor(size / numThreads);
    for (int i = 0; i < numThreads; i++) {
        int startIdx = (i == 0) ? 0 : endIndices[i - 1] + 1;
        int endIdx = (i == numThreads - 1) ? size - 1 : startIdx + chunksize - 1;

        startIndices.push_back(startIdx);
        endIndices.push_back(endIdx);
    }

    double radius = _supportRadiusFactor*_particleRadius;
    _setKernelRadius(radius);
    _covarianceMatrices = std::vector<glm::mat3>(size, glm::mat3());
    
    std::vector<std::thread> threads;
    for (int i = 0; i < numThreads; i++) {
        threads.push_back(std::thread(&ParticleMesher::_computeRangeOfCovarianceMatrices,
                                      this,
                                      startIndices[i],
                                      endIndices[i],
                                      radius));
    }

    for (int i = 0; i < numThreads; i++) {
        threads[i].join();
    }
}

void ParticleMesher::_computeRangeOfCovarianceMatrices(int startidx, int endidx, double radius) {
    std::vector<GridPointReference> neighbours;
    GridPointReference ref;
    glm::mat3 cmat;
    for (int i = startidx; i <= endidx; i++) {
        ref = _nearSurfaceParticleRefs[i];
        _covarianceMatrices[i] = _computeCovarianceMatrix(ref, radius, neighbours);
    }
}

glm::mat3 ParticleMesher::_computeCovarianceMatrix(GridPointReference ref, double radius,
                                                   std::vector<GridPointReference> &neighbours) {
    neighbours.clear();
    _pointGrid.queryPointReferencesInsideSphere(ref, radius, neighbours);
    glm::vec3 meanpos = _getWeightedMeanParticlePosition(ref, neighbours);

    SurfaceParticle meansp = SurfaceParticle(meanpos);
    meansp.componentID = _surfaceParticles[ref.id].componentID;

    double weightSum = 0.0;
    float sum00 = 0.0;
    float sum11 = 0.0;
    float sum22 = 0.0;
    float sum01 = 0.0;
    float sum02 = 0.0;
    float sum12 = 0.0;
    GridPointReference refj;
    SurfaceParticle spj;
    glm::vec3 v;
    double kernelVal;
    double eps = 1e-9;
    for (unsigned int i = 0; i < neighbours.size(); i++) {
        refj = neighbours[i];
        spj = _surfaceParticles[refj.id];

        kernelVal = _evaluateKernel(meansp, spj);

        if (kernelVal < eps) {
            continue;
        }

        v = spj.position - meansp.position;
        sum00 += (float)kernelVal*v[0]*v[0];
        sum11 += (float)kernelVal*v[1]*v[1];
        sum22 += (float)kernelVal*v[2]*v[2];
        sum01 += (float)kernelVal*v[0]*v[1];
        sum02 += (float)kernelVal*v[0]*v[2];
        sum12 += (float)kernelVal*v[1]*v[2];
        weightSum += kernelVal;
    }

    if (weightSum < eps) {
        return glm::mat3(1.0);
    }

    return glm::mat3(sum00, sum01, sum02,
                     sum01, sum11, sum12,
                     sum02, sum12, sum22) / (float) weightSum;
}

void ParticleMesher::_setParticleRadius(double r) {
    _particleRadius = r;
}

void ParticleMesher::_setKernelRadius(double r) {
    assert(r > 0.0);

    _kernelRadius = r;
    _invKernelRadius = 1 / r;
}

double ParticleMesher::_evaluateKernel(SurfaceParticle &pi, SurfaceParticle &pj) {
    if (pj.componentID != pi.componentID) {
        return 0.0;
    }

    double dist = glm::length(pj.position - pi.position);
    if (dist >= _kernelRadius) {
        return 0.0;
    }

    double v = dist * _invKernelRadius;
    return 1.0 - v*v*v;
}