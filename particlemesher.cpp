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
                                           Array3d<int> &materialGrid,
                                           double particleRadius) {
    _clear();
    _setParticleRadius(particleRadius);

    std::vector<IsotropicParticle> isoParticles;
    std::vector<AnisotropicParticle> anisoParticles;
    _computeSurfaceReconstructionParticles(particles, levelset,
                                           isoParticles, anisoParticles);

    TriangleMesh mesh = _reconstructSurface(isoParticles, anisoParticles,
                                            materialGrid);
    return mesh;
}

void ParticleMesher::_clear() {
    _surfaceParticles.clear();
    _nearSurfaceParticleRefs.clear();
    _farSurfaceParticleRefs.clear();
    _pointGrid = SpatialPointGrid();
    _smoothedPositions.clear();
    _covarianceMatrices.clear();
    _SVDMatrices.clear();
}

void ParticleMesher::_computeSurfaceReconstructionParticles(
                                          std::vector<glm::vec3> &particles, 
                                          LevelSet &levelset,
                                          std::vector<IsotropicParticle> &iso,
                                          std::vector<AnisotropicParticle> &aniso) {
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
    nearSurfaceParticles.clear();
    nearSurfaceParticles.shrink_to_fit();
    farSurfaceParticles.clear();
    farSurfaceParticles.shrink_to_fit();
    surfaceParticles.clear();
    surfaceParticles.shrink_to_fit();
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

    _initializeSurfaceReconstructionParticles(iso, insideParticles,
                                              aniso, anisotropyMatrices);

    _pointGrid = SpatialPointGrid();

    anisotropyMatrices.clear();
    insideParticles.clear();
    _nearSurfaceParticleRefs.clear();
    _farSurfaceParticleRefs.clear();

    anisotropyMatrices.shrink_to_fit();
    insideParticles.shrink_to_fit();
    _nearSurfaceParticleRefs.shrink_to_fit();
    _farSurfaceParticleRefs.shrink_to_fit();
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

void ParticleMesher::_computeAnisotropyMatrices(std::vector<glm::mat3x3> &anisoMatrices) {
    _computeCovarianceMatrices();
    _computeSVDMatrices();

    _covarianceMatrices.clear();
    _covarianceMatrices.shrink_to_fit();

    SVD svd;
    anisoMatrices.reserve(_SVDMatrices.size());
    for (unsigned int i = 0; i < _SVDMatrices.size(); i++) {
        svd = _SVDMatrices[i];
        anisoMatrices.push_back(_SVDToAnisotropicMatrix(svd));
    }

    _SVDMatrices.clear();
    _SVDMatrices.shrink_to_fit();
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

    if (neighbours.size() <= _minAnisotropicParticleNeighbourThreshold) {
        return glm::mat3(1.0);
    }

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

void ParticleMesher::_computeSVDMatrices() {
    _SVDMatrices = std::vector<SVD>(_covarianceMatrices.size(), SVD());

    for (unsigned int i = 0; i < _covarianceMatrices.size(); i++) {
        SVD svd;
        _covarianceMatrixToSVD(_covarianceMatrices[i], svd);
        _SVDMatrices[i] = svd;
    }
}

void ParticleMesher::_covarianceMatrixToSVD(glm::mat3 &covariance, SVD &svd) {
    glm::quat q = _diagonalizeMatrix(covariance);
    glm::mat3 Q = glm::mat3_cast(q);
    glm::mat3 D = glm::transpose(Q) * covariance * Q;

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

    double ks = _eigenvalueScalingFactor;
    svd.rotation = glm::mat3(Q[k0], Q[k1], Q[k2]);
    svd.diag = (float)ks*glm::vec3(sigma0, sigma1, sigma2);
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
glm::quat ParticleMesher::_diagonalizeMatrix(glm::mat3 A) {

    int maxsteps = 24;
    glm::quat q(1.0, glm::vec3(0.0, 0.0, 0.0));
    int i = 0;

    for (i = 0; i < maxsteps; i++) {
        glm::mat3 Q = glm::mat3_cast(q);
        glm::mat3 D = glm::transpose(Q) * A * Q;
        glm::vec3 offdiag(D[2][1], D[2][0], D[1][0]);
        glm::vec3 om(fabsf(offdiag.x), fabsf(offdiag.y), fabsf(offdiag.z));
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

        glm::quat jr(sqrtf(1.0f - jrtemp[k]*jrtemp[k]), 
                     glm::vec3(jrtemp[0], jrtemp[1], jrtemp[2]));

        if(jr.w==1.0f) {
            break; 
        }

        q = glm::cross(q,jr); 
		q = glm::normalize(q);
    }

    return q;
}

glm::mat3 ParticleMesher::_SVDToAnisotropicMatrix(SVD &svd) {
    glm::mat3 invD = glm::mat3(glm::vec3(1.0 / svd.diag.x, 0.0, 0.0),
                               glm::vec3(0.0, 1.0 / svd.diag.y, 0.0),
                               glm::vec3(0.0, 0.0, 1.0 / svd.diag.z));

    return svd.rotation * invD * glm::transpose(svd.rotation);
}

void ParticleMesher::_initializeSurfaceReconstructionParticles(
                                    std::vector<IsotropicParticle> &iso,
                                    std::vector<glm::vec3> &insideParticles,
                                    std::vector<AnisotropicParticle> &aniso,
                                    std::vector<glm::mat3> &anisoMatrices) {

    iso.reserve(_nearSurfaceParticleRefs.size());
    aniso.reserve(insideParticles.size() + _farSurfaceParticleRefs.size());

    glm::vec3 p;
    GridPointReference ref;
    for (unsigned int i = 0; i < _farSurfaceParticleRefs.size(); i++) {
        ref = _farSurfaceParticleRefs[i];
        p = _surfaceParticles[ref.id].position;
        iso.push_back(IsotropicParticle(p));
    }

    for (unsigned int i = 0; i < insideParticles.size(); i++) {
        iso.push_back(insideParticles[i]);
    }

    for (unsigned int i = 0; i < _nearSurfaceParticleRefs.size(); i++) {
        ref = _nearSurfaceParticleRefs[i];
        p = _surfaceParticles[ref.id].position;
        aniso.push_back(AnisotropicParticle(p, anisoMatrices[i]));
    }
}

TriangleMesh ParticleMesher::_reconstructSurface(std::vector<IsotropicParticle> &iso,
                                                 std::vector<AnisotropicParticle> &aniso,
                                                 Array3d<int> &materialGrid) {

    ImplicitSurfaceScalarField field = ImplicitSurfaceScalarField(_isize + 1, 
                                                                  _jsize + 1, 
                                                                  _ksize + 1, _dx);
    field.setMaterialGrid(materialGrid);

    double r = _particleRadius*_anisotropicParticleScale;
    field.setPointRadius(r);

    glm::vec3 p, v;
    glm::mat3 G;
    for (unsigned int i = 0; i < aniso.size(); i++) {
        p = aniso[i].position;
        G = (float)r*aniso[i].anisotropy;
        field.addEllipsoid(p, G);
    }

    r = _particleRadius*_isotropicParticleScale;
    field.setPointRadius(r);
    for (unsigned int i = 0; i < iso.size(); i++) {
        p = iso[i].position;
        field.addPoint(p);
    }

    Polygonizer3d polygonizer = Polygonizer3d(field);

    polygonizer.polygonizeSurface();
    return polygonizer.getTriangleMesh();
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