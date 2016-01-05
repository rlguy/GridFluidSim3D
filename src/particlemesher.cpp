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

TriangleMesh ParticleMesher::meshParticles(FragmentedVector<MarkerParticle> &particles, 
                                           LevelSet &levelset,
                                           FluidMaterialGrid &materialGrid,
                                           double particleRadius) {

    _setParticleRadius(particleRadius);

    FragmentedVector<vmath::vec3> filteredParticles;
    _filterHighDensityParticles(particles, filteredParticles);

    _initializeSurfaceParticles(filteredParticles, levelset);
    _smoothSurfaceParticlePositions();

    _computeScalarField(materialGrid, filteredParticles, levelset);
   
    Polygonizer3d polygonizer = Polygonizer3d(_scalarField);
    polygonizer.polygonizeSurface();

    return polygonizer.getTriangleMesh();
}

void ParticleMesher::_filterHighDensityParticles(FragmentedVector<MarkerParticle> &particles,
                                                 FragmentedVector<vmath::vec3> &filtered) {
    filtered.reserve(particles.size());

    Array3d<int> countGrid = Array3d<int>(_isize, _jsize, _ksize, 0);

    vmath::vec3 p;
    GridIndex g;
    for (unsigned int i = 0; i < particles.size(); i++) {
        p = particles[i].position;
        g = Grid3d::positionToGridIndex(p, _dx);

        if (countGrid(g) >= _maxParticlesPerCell) {
            continue;
        }
        countGrid.add(g, 1);

        filtered.push_back(particles[i].position);
    }

}

void ParticleMesher::_clear() {
    _surfaceParticles.clear();
    _nearSurfaceParticleRefs.clear();
    _farSurfaceParticleRefs.clear();
    _pointGrid = SpatialPointGrid();
    _smoothedPositions.clear();
}

void ParticleMesher::_initializeSurfaceParticles(FragmentedVector<vmath::vec3> &particles, 
                                                 LevelSet &levelset) {

    vmath::vec3 p;
    for (unsigned int i = 0; i < particles.size(); i++) {
        p = particles[i];
        if (_isSurfaceParticle(p, levelset)) {
            _surfaceParticles.push_back(SurfaceParticle(p));
        }
    }

    _initializeSurfaceParticleSpatialGrid();
    _updateNearFarSurfaceParticleReferences(levelset);
    _updateSurfaceParticleComponentIDs();
}

void ParticleMesher::_initializeSurfaceParticleSpatialGrid() {
    _pointGrid = SpatialPointGrid(_isize, _jsize, _ksize, _dx);

    FragmentedVector<vmath::vec3> points;
    for (unsigned int i = 0; i < _surfaceParticles.size(); i++) {
        points.push_back(_surfaceParticles[i].position);
    }

    std::vector<GridPointReference> refs = _pointGrid.insert(points);

    for (unsigned int i = 0; i < _surfaceParticles.size(); i++) {
        _surfaceParticles[i].ref = refs[i];
    }
}

ParticleMesher::ParticleLocation ParticleMesher::_getParticleLocationType(vmath::vec3 p, 
                                                                          LevelSet &levelset) {
    double supportRadius = _particleRadius*_supportRadiusFactor;
    double maxNearSurfaceDepth = _dx*_maxParticleToSurfaceDepth;
    double maxSurfaceDepth = maxNearSurfaceDepth + supportRadius;
    double dist = levelset.getSignedDistance(p);

    if (dist < 0.0) {
        return ParticleLocation::NearSurface;
    } else {
        if (dist < maxSurfaceDepth) {
            if (dist < maxNearSurfaceDepth) {
                return ParticleLocation::NearSurface;
            } else {
                return ParticleLocation::FarSurface;
            }
        } else {
            return ParticleLocation::Inside;
        }
    }

}


void ParticleMesher::_updateNearFarSurfaceParticleReferences(LevelSet &levelset) {
    SurfaceParticle sp;
    ParticleLocation type;
    for (unsigned int i = 0; i < _surfaceParticles.size(); i++) {
        sp = _surfaceParticles[i];
        type = _getParticleLocationType(sp.position, levelset);

        if (type == ParticleLocation::NearSurface) {
            _nearSurfaceParticleRefs.push_back(sp.ref);
        } else {
            _farSurfaceParticleRefs.push_back(sp.ref);
        }
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

void *ParticleMesher::_startSmoothRangeOfSurfaceParticlePositionsThread(void *threadarg) {
    Threading::IndexRangeThreadParams *params = (Threading::IndexRangeThreadParams *)threadarg;
    int start = params->startIndex;
    int end = params->endIndex;
    ((ParticleMesher *)(params->obj))->_smoothRangeOfSurfaceParticlePositions(start, end);

    return NULL;
}

void ParticleMesher::_computeSmoothedNearSurfaceParticlePositions() {
    
    double supportRadius = _supportRadiusFactor*_particleRadius;
    _setKernelRadius(supportRadius);

    int numElements = _nearSurfaceParticleRefs.size();
    _smoothedPositions = FragmentedVector<vmath::vec3>(numElements);
    
    Threading::splitIndexRangeWorkIntoThreads(numElements, _numThreads, (void *)this, 
                                              ParticleMesher::_startSmoothRangeOfSurfaceParticlePositionsThread);
}

void ParticleMesher::_smoothRangeOfSurfaceParticlePositions(int startidx, int endidx) {
    std::vector<GridPointReference> neighbourRefs;
    GridPointReference ref;
    vmath::vec3 newp;
    for (int i = startidx; i <= endidx; i++) {
        ref = _nearSurfaceParticleRefs[i];
        newp = _getSmoothedParticlePosition(ref, _kernelRadius, neighbourRefs);
        _smoothedPositions[i] = newp;
    }
}

vmath::vec3 ParticleMesher::_getSmoothedParticlePosition(GridPointReference ref,
                                                         double radius,
                                                         std::vector<GridPointReference> &neighbourRefs) {
    neighbourRefs.clear();
    _pointGrid.queryPointReferencesInsideSphere(ref, radius, neighbourRefs);
    vmath::vec3 mean = _getWeightedMeanParticlePosition(ref, neighbourRefs);

    SurfaceParticle spi = _surfaceParticles[ref.id]; 
    float k = (float)_smoothingConstant;
    return (1.0f - k)*spi.position + k*mean;
}

vmath::vec3 ParticleMesher::_getWeightedMeanParticlePosition(GridPointReference ref,
                                                             std::vector<GridPointReference> &neighbours) {
    SurfaceParticle spi = _surfaceParticles[ref.id]; 
    SurfaceParticle spj;

    double xsum = 0.0;
    double ysum = 0.0;
    double zsum = 0.0;
    double weightSum = 0.0;
    double kernalVal;
    vmath::vec3 v;

    GridPointReference refj;
    double eps = 1e-9;
    for (unsigned int i = 0; i < neighbours.size(); i++) {
        refj = neighbours[i];
        spj = _surfaceParticles[refj.id];

        kernalVal = _evaluateKernel(spi, spj);
        xsum += kernalVal*spj.position.x;
        ysum += kernalVal*spj.position.y;
        zsum += kernalVal*spj.position.z;
        weightSum += kernalVal;
    }

    if (weightSum < eps) {
        return spi.position;
    }

    return vmath::vec3(xsum, ysum, zsum) / (float)weightSum;
}

void ParticleMesher::_computeScalarField(FluidMaterialGrid &materialGrid,
                                         FragmentedVector<vmath::vec3> &particles,
                                         LevelSet &levelset) {
    _initializeScalarField(materialGrid);
    _initializeProducerConsumerStacks();

    _addAnisotropicParticlesToScalarField();
    _addIsotropicParticlesToScalarField(particles, levelset);

}

void ParticleMesher::_initializeScalarField(FluidMaterialGrid &materialGrid) {
    _scalarField = ImplicitSurfaceScalarField(_isize + 1, _jsize + 1, _ksize + 1, _dx);
    _scalarField.setMaterialGrid(materialGrid);
}

void ParticleMesher::_initializeProducerConsumerStacks() {
    _unprocessedAnisotropicParticleStack = _nearSurfaceParticleRefs;
    _processedAnisotropicParticleStack = ProducerConsumerStack<AnisotropicParticle>(_consumerStackSize);
    _numAnisotropicParticles = _unprocessedAnisotropicParticleStack.size();
}

void ParticleMesher::_addAnisotropicParticlesToScalarField() {
    double r = _particleRadius*_anisotropicParticleScale;
    _scalarField.setPointRadius(r);

    std::vector<pthread_t> producers(_numThreads);
    pthread_t consumer;

    pthread_attr_t attr = Threading::createJoinableThreadAttribute();

    Threading::createThread(&consumer, 
                            &attr, 
                            ParticleMesher::_startAnisotropicParticleConsumerThread, 
                            (void *)this);

    for (int i = 0; i < _numThreads; i++) {
        Threading::createThread(&producers[i], 
                                &attr, 
                                ParticleMesher::_startAnisotropicParticleProducerThread, 
                                (void *)this);
    }

    Threading::destroyThreadAttribute(&attr);

    Threading::joinThreads(producers);
    Threading::joinThread(consumer);
}

void ParticleMesher::_addAnisotropicParticleToScalarField(AnisotropicParticle &aniso) {
    double r = _particleRadius*_anisotropicParticleScale;
    vmath::vec3 p = aniso.position;
    vmath::mat3 G = (float)r*aniso.anisotropy;

    _scalarField.addEllipsoidValue(p, G, _anisotropicParticleFieldScale);
}

void ParticleMesher::_addIsotropicParticlesToScalarField(FragmentedVector<vmath::vec3> &particles, LevelSet &levelset) {
    double r = _particleRadius*_isotropicParticleScale;
    _scalarField.setPointRadius(r);

    vmath::vec3 p;
    for (unsigned int i = 0; i < particles.size(); i++) {
        p = particles[i];

        if (_isInsideParticle(p, levelset)) {
            _scalarField.addPoint(p);
        }
    }
}

ParticleMesher::AnisotropicParticle ParticleMesher::_computeAnisotropicParticle(GridPointReference ref) {
    std::vector<GridPointReference> neighbours;
    vmath::mat3 covariance = _computeCovarianceMatrix(ref, _kernelRadius, neighbours);

    SVD svd;
    _covarianceMatrixToSVD(covariance, svd);
    vmath::mat3 G = _SVDToAnisotropicMatrix(svd);

    vmath::vec3 p = _surfaceParticles[ref.id].position;
    return AnisotropicParticle(p, G);
}

void *ParticleMesher::_startAnisotropicParticleProducerThread(void *q) {
    ParticleMesher *obj;
    obj = (ParticleMesher *) q;
    obj->_anisotropicParticleProducerThread();

    return NULL;
}

void *ParticleMesher::_startAnisotropicParticleConsumerThread(void *q) {
    ParticleMesher *obj;
    obj = (ParticleMesher *) q;
    obj->_anisotropicParticleConsumerThread();

    return NULL;
}

void ParticleMesher::_anisotropicParticleProducerThread() {

    int chunksize = _producerStackSize;

    _anisotropicParticleStackMutex.lock();
    bool isEmpty = _unprocessedAnisotropicParticleStack.empty();
    _anisotropicParticleStackMutex.unlock();

    std::vector<GridPointReference> refs;
    std::vector<AnisotropicParticle> anisotropicParticles;
    while (!isEmpty) {

        refs.clear();
        anisotropicParticles.clear();

        _getUnprocessedParticlesFromStack(chunksize, refs);

        _anisotropicParticleStackMutex.lock();
        isEmpty = _unprocessedAnisotropicParticleStack.empty();
        _anisotropicParticleStackMutex.unlock();

        for (unsigned int i = 0; i < refs.size(); i++) {
            anisotropicParticles.push_back(_computeAnisotropicParticle(refs[i]));
        }

        if (anisotropicParticles.empty()) {
            break;
        }

        _processedAnisotropicParticleStack.pushAll(anisotropicParticles);
    }
}

void ParticleMesher::_getUnprocessedParticlesFromStack(int chunksize, std::vector<GridPointReference> &refs) {
    _anisotropicParticleStackMutex.lock();

    int stackSize = (int)_unprocessedAnisotropicParticleStack.size();
    int n = chunksize <= stackSize ? chunksize : stackSize;
    for (int i = 0; i < n; i++) {
        refs.push_back(_unprocessedAnisotropicParticleStack.back());
        _unprocessedAnisotropicParticleStack.pop_back();
    }

    _anisotropicParticleStackMutex.unlock();
}

void ParticleMesher::_anisotropicParticleConsumerThread() {
    std::vector<AnisotropicParticle> anisoParticles;

    int itemsProcessed = 0;
    while (itemsProcessed < _numAnisotropicParticles) {
        anisoParticles.clear();
        _processedAnisotropicParticleStack.popAll(anisoParticles);

        for (unsigned int i = 0; i < anisoParticles.size(); i++) {
            _addAnisotropicParticleToScalarField(anisoParticles[i]);
            itemsProcessed++;
        }
    }
}

vmath::mat3 ParticleMesher::_computeCovarianceMatrix(GridPointReference ref, double radius,
                                                   std::vector<GridPointReference> &neighbours) {

    neighbours.clear();
    _pointGrid.queryPointReferencesInsideSphere(ref, radius, neighbours);

    if ((int)neighbours.size() <= _minAnisotropicParticleNeighbourThreshold) {
        return vmath::mat3(1.0);
    }

    vmath::vec3 meanpos = _getWeightedMeanParticlePosition(ref, neighbours);

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
    vmath::vec3 v;
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
        return vmath::mat3(1.0);
    }

    return vmath::mat3(sum00, sum01, sum02,
                     sum01, sum11, sum12,
                     sum02, sum12, sum22) / (float) weightSum;
}

void ParticleMesher::_covarianceMatrixToSVD(vmath::mat3 &covariance, SVD &svd) {
    vmath::quat q = _diagonalizeMatrix(covariance);
    vmath::mat3 Q = vmath::mat3_cast(q);
    vmath::mat3 D = vmath::transpose(Q) * covariance * Q;

    double d0 = D[0][0];
    double d1 = D[1][1];
    double d2 = D[2][2];
    int k0 = 0;
    int k1 = 1;
    int k2 = 2;

    if (d0 > d1 && d0 > d2) {
        if (d1 > d2) {
            k0 = 0; k1 = 1; k2 = 2;
        } else {
            k0 = 0; k1 = 2; k2 = 1;
        }
    } else if (d1 > d0 && d1 > d2) {
        if (d0 > d2) {
            k0 = 1; k1 = 0; k2 = 2;
        } else {
            k0 = 1; k1 = 2; k2 = 0;
        }
    } else if (d2 > d0 && d2 > d1) {
        if (d0 > d1) {
            k0 = 2; k1 = 0; k2 = 1;
        } else {
            k0 = 2; k1 = 1; k2 = 0;
        }
    }

    double kr = _maxEigenvalueRatio;
    double sigma0 = (double)D[k0][k0];
    double sigma1 = std::max((double)D[k1][k1], sigma0 / kr);
    double sigma2 = std::max((double)D[k2][k2], sigma0 / kr);;

    double ks = cbrt(1.0/(sigma0*sigma1*sigma2));          // scale so that det(covariance) == 1
    svd.rotation = vmath::mat3(Q[k0], Q[k1], Q[k2]);
    svd.diag = (float)ks*vmath::vec3(sigma0, sigma1, sigma2);
}

/*
    A must be a symmetric matrix.
	returns quaternion q such that its corresponding matrix Q 
	can be used to Diagonalize A
	Diagonal matrix D = Transpose(Q) * A * Q;  and  A = Q * D * Transpose(Q)
	The columns of Q are the eigenvectors, D's diagonal is the eigenvalues

    Method adapted from:
        www.melax.com/diag.html?attredirects=0
*/
vmath::quat ParticleMesher::_diagonalizeMatrix(vmath::mat3 A) {

    int maxsteps = 24;
    vmath::quat q(1.0, vmath::vec3());
    int i = 0;

    for (i = 0; i < maxsteps; i++) {
        vmath::mat3 Q = vmath::mat3_cast(q);
        vmath::mat3 D = vmath::transpose(Q) * A * Q;
        vmath::vec3 offdiag(D[2][1], D[2][0], D[1][0]);
        vmath::vec3 om(fabsf(offdiag.x), fabsf(offdiag.y), fabsf(offdiag.z));
        int k = (om.x > om.y && om.x > om.z)?0: (om.y > om.z)? 1 : 2;
        int k1 = (k+1)%3;
		int k2 = (k+2)%3;

        if(offdiag[k]==0.0f) {
            break;
        }

        float thet = (D[k2][k2]-D[k1][k1])/(2.0f*offdiag[k]);
		float sgn = (thet > 0.0f)?1.0f:-1.0f;
		thet *= sgn;
        float t = sgn /(thet +((thet < 1.E6f)?sqrtf(thet*thet+1.0f):thet));
		float c = 1.0f/sqrtf(t*t+1.0f);

        if(c==1.0f) { 
            break; 
        }

        float jrtemp[3] = {0.0f, 0.0f, 0.0f};
        jrtemp[k] = sgn*sqrtf((1.0f-c)/2.0f);
        jrtemp[k] *= -1.0f;

        vmath::quat jr(sqrtf(1.0f - jrtemp[k]*jrtemp[k]), 
                     vmath::vec3(jrtemp[0], jrtemp[1], jrtemp[2]));

        if(jr.w==1.0f) {
            break; 
        }

        q = vmath::cross(q,jr); 
		q = vmath::normalize(q);
    }

    return q;
}

vmath::mat3 ParticleMesher::_SVDToAnisotropicMatrix(SVD &svd) {
    vmath::mat3 invD = vmath::mat3(vmath::vec3(1.0 / svd.diag.x, 0.0, 0.0),
                               vmath::vec3(0.0, 1.0 / svd.diag.y, 0.0),
                               vmath::vec3(0.0, 0.0, 1.0 / svd.diag.z));

    return svd.rotation * invD * vmath::transpose(svd.rotation);
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

    double dist = vmath::length(pj.position - pi.position);
    if (dist >= _kernelRadius) {
        return 0.0;
    }

    double v = dist * _invKernelRadius;
    return 1.0 - v*v*v;
}