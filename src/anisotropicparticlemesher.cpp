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
#include "anisotropicparticlemesher.h"

AnisotropicParticleMesher::AnisotropicParticleMesher() {
}

AnisotropicParticleMesher::AnisotropicParticleMesher(int isize, int jsize, int ksize, double dx) :
                                                        _isize(isize), _jsize(jsize), _ksize(ksize), 
                                                        _dx(dx) {

}

AnisotropicParticleMesher::~AnisotropicParticleMesher() {
}

void AnisotropicParticleMesher::setSubdivisionLevel(int n) {
    FLUIDSIM_ASSERT(n >= 1);
    _subdivisionLevel = n;
}

void AnisotropicParticleMesher::setNumPolygonizationSlices(int n) {
    FLUIDSIM_ASSERT(n >= 1);

    if (n > _isize) {
        n = _isize;
    }

    _numPolygonizationSlices = n;
}

TriangleMesh AnisotropicParticleMesher::meshParticles(FragmentedVector<MarkerParticle> &particles, 
                                                      LevelSet &levelset,
                                                      FluidMaterialGrid &materialGrid,
                                                      double particleRadius) {

    FLUIDSIM_ASSERT(materialGrid.width == _isize &&
           materialGrid.height == _jsize &&
           materialGrid.depth == _ksize);
    FLUIDSIM_ASSERT(particleRadius > 0.0);

    _setParticleRadius(particleRadius);

    FragmentedVector<vmath::vec3> filteredParticles;
    _filterHighDensityParticles(particles, filteredParticles);

    _initializeSurfaceParticles(filteredParticles, levelset);

    if (_numPolygonizationSlices == 1) {
        return _polygonizeAll(filteredParticles, levelset, materialGrid);
    }

    return _polygonizeSlices(filteredParticles, levelset, materialGrid);
}

void AnisotropicParticleMesher::_filterHighDensityParticles(FragmentedVector<MarkerParticle> &particles,
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

void AnisotropicParticleMesher::_clear() {
    _surfaceParticles.clear();
    _nearSurfaceParticleRefs.clear();
    _farSurfaceParticleRefs.clear();
    _pointGrid = SpatialPointGrid();
    _smoothedPositions.clear();
}

void AnisotropicParticleMesher::_initializeSurfaceParticles(FragmentedVector<vmath::vec3> &particles, 
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
    _smoothSurfaceParticlePositions();
}

void AnisotropicParticleMesher::_initializeSurfaceParticleSpatialGrid() {
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

AnisotropicParticleMesher::ParticleLocation AnisotropicParticleMesher::_getParticleLocationType(
                                                                    vmath::vec3 p, LevelSet &levelset) {
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

void AnisotropicParticleMesher::_updateNearFarSurfaceParticleReferences(LevelSet &levelset) {
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


void AnisotropicParticleMesher::_updateSurfaceParticleComponentIDs() {
    double r = _particleRadius*_connectedComponentRadiusFactor;
    std::vector<std::vector<GridPointReference> > components;
    _pointGrid.getConnectedPointReferenceComponents(r, components);


    for (unsigned int cid = 0; cid < components.size(); cid++) {

        int componentID = cid;
        if ((int)components[cid].size() < _minComponentParticleCount) {
            componentID = -1;
        }

        for (unsigned int idx = 0; idx < components[cid].size(); idx++) {
            int spidx = components[cid][idx].id;
            _surfaceParticles[spidx].componentID = componentID;
        }
    }
}


void AnisotropicParticleMesher::_smoothSurfaceParticlePositions() {
    _computeSmoothedNearSurfaceParticlePositions();

    GridPointReference ref;
    for (unsigned int i = 0; i < _nearSurfaceParticleRefs.size(); i++) {
        ref = _nearSurfaceParticleRefs[i];
        _surfaceParticles[ref.id].position = _smoothedPositions[i];
    }

    _smoothedPositions.clear();
    _smoothedPositions.shrink_to_fit();
}

void AnisotropicParticleMesher::_computeSmoothedNearSurfaceParticlePositions() {
    
    double supportRadius = _supportRadiusFactor*_particleRadius;
    _setKernelRadius(supportRadius);

    int numElements = _nearSurfaceParticleRefs.size();
    _smoothedPositions = FragmentedVector<vmath::vec3>(numElements);
    
    _smoothRangeOfSurfaceParticlePositions(0, _nearSurfaceParticleRefs.size() - 1);
}

void AnisotropicParticleMesher::_smoothRangeOfSurfaceParticlePositions(int startidx, int endidx) {
    std::vector<GridPointReference> neighbourRefs;
    GridPointReference ref;
    vmath::vec3 newp;
    for (int i = startidx; i <= endidx; i++) {
        ref = _nearSurfaceParticleRefs[i];
        newp = _getSmoothedParticlePosition(ref, _kernelRadius, neighbourRefs);
        _smoothedPositions[i] = newp;
    }
}

vmath::vec3 AnisotropicParticleMesher::_getSmoothedParticlePosition(GridPointReference ref,
                                                                    double radius,
                                                                    std::vector<GridPointReference> &neighbourRefs) {
    neighbourRefs.clear();
    _pointGrid.queryPointReferencesInsideSphere(ref, radius, neighbourRefs);
    vmath::vec3 mean = _getWeightedMeanParticlePosition(ref, neighbourRefs);

    SurfaceParticle spi = _surfaceParticles[ref.id]; 
    float k = (float)_smoothingConstant;
    return (1.0f - k)*spi.position + k*mean;
}

vmath::vec3 AnisotropicParticleMesher::_getWeightedMeanParticlePosition(GridPointReference ref,
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

TriangleMesh AnisotropicParticleMesher::_polygonizeAll(FragmentedVector<vmath::vec3> &particles, 
                                                       LevelSet &levelset,
                                                       FluidMaterialGrid &materialGrid) {
    _computeScalarField(materialGrid, particles, levelset);
   
    Polygonizer3d polygonizer = Polygonizer3d(&_scalarField);

    return polygonizer.polygonizeSurface();
}

TriangleMesh AnisotropicParticleMesher::_polygonizeSlices(FragmentedVector<vmath::vec3> &particles, 
                                                          LevelSet &levelset,
                                                          FluidMaterialGrid &materialGrid) {

    int width, height, depth;
    double dx;
    _getSubdividedGridDimensions(&width, &height, &depth, &dx);

    int sliceWidth = ceil((double)width / (double)_numPolygonizationSlices);
    int numSlices = ceil((double)width / (double)sliceWidth);

    if (numSlices == 1) {
        return _polygonizeAll(particles, levelset, materialGrid);
    }

    TriangleMesh mesh;
    for (int i = 0; i < numSlices; i++) {
        int startidx = i*sliceWidth;
        int endidx = startidx + sliceWidth - 1;
        endidx = endidx < width ? endidx : width - 1;

        TriangleMesh sliceMesh = _polygonizeSlice(startidx, endidx, particles,
                                                levelset, materialGrid);

        vmath::vec3 offset = _getSliceGridPositionOffset(startidx, endidx);
        sliceMesh.translate(offset);

        mesh.join(sliceMesh);
    }

    return mesh;
}

TriangleMesh AnisotropicParticleMesher::_polygonizeSlice(int startidx, int endidx, 
                                                         FragmentedVector<vmath::vec3> &particles, 
                                                         LevelSet &levelset,
                                                         FluidMaterialGrid &materialGrid) {
    int width, height, depth;
    double dx;
    _getSubdividedGridDimensions(&width, &height, &depth, &dx);

    bool isStartSlice = startidx == 0;
    bool isEndSlice = endidx == width - 1;
    bool isMiddleSlice = !isStartSlice && !isEndSlice;

    int gridWidth = endidx - startidx + 1;
    int gridHeight = height;
    int gridDepth = depth;
    if (isStartSlice || isEndSlice) {
        gridWidth++;
    } else if (isMiddleSlice) {
        gridWidth += 2;
    }

    _computeSliceScalarField(startidx, endidx, particles, levelset, materialGrid);

    Array3d<bool> mask(gridWidth, gridHeight, gridDepth);
    _getSliceMask(startidx, endidx, mask);

    Polygonizer3d polygonizer(&_scalarField);
    polygonizer.setSurfaceCellMask(&mask);

    return polygonizer.polygonizeSurface();
}

void AnisotropicParticleMesher::_getSubdividedGridDimensions(int *i, int *j, int *k, double *dx) {
    *i = _isize*_subdivisionLevel;
    *j = _jsize*_subdivisionLevel;
    *k = _ksize*_subdivisionLevel;
    *dx = _dx / (double)_subdivisionLevel;
}

vmath::vec3 AnisotropicParticleMesher::_getSliceGridPositionOffset(int startidx, int endidx) {
    (void)endidx;
    int width, height, depth;
    double dx;
    _getSubdividedGridDimensions(&width, &height, &depth, &dx);

    bool isStartSlice = startidx == 0;

    double offx;
    if (isStartSlice) {
        offx = startidx*dx;
    } else {
        offx = (startidx - 1)*dx;
    }

    return vmath::vec3(offx, 0.0, 0.0); 
}

void AnisotropicParticleMesher::_getSliceMaterialGrid(int startidx, int endidx,
                                                      FluidMaterialGrid &materialGrid,
                                                      FluidMaterialGrid &sliceMaterialGrid) {
    (void)endidx;
    int origsubd = materialGrid.getSubdivisionLevel();
    materialGrid.setSubdivisionLevel(_subdivisionLevel);
    
    Material m;
    for (int k = 0; k < sliceMaterialGrid.depth; k++) {
        for (int j = 0; j < sliceMaterialGrid.height; j++) {
            for (int i = 0; i < sliceMaterialGrid.width; i++) {
                m = materialGrid(startidx + i, j, k);
                sliceMaterialGrid.set(i, j, k, m);
            }
        }
    }

    materialGrid.setSubdivisionLevel(origsubd);
}

AABB AnisotropicParticleMesher::_getSliceAABB(int startidx, int endidx) {
    int width, height, depth;
    double dx;
    _getSubdividedGridDimensions(&width, &height, &depth, &dx);

    bool isStartSlice = startidx == 0;
    bool isEndSlice = endidx == width - 1;
    bool isMiddleSlice = !isStartSlice && !isEndSlice;

    double gridWidth = (endidx - startidx + 1)*dx;
    double gridHeight = height*dx;
    double gridDepth = depth*dx;
    if (isStartSlice || isEndSlice) {
        gridWidth += dx;
    } else if (isMiddleSlice) {
        gridWidth += 2.0*dx;
    }

    vmath::vec3 offset = _getSliceGridPositionOffset(startidx, endidx);

    double isopad = _particleRadius*_isotropicParticleScale;
    double anisopad = _particleRadius*_anisotropicParticleScale*_maxEigenvalueRatio;
    double pad = fmax(isopad, anisopad);
    AABB bbox(offset, gridWidth, gridHeight, gridDepth);
    bbox.expand(2.0*pad);

    return bbox;
}

void AnisotropicParticleMesher::_updateScalarFieldSeam(int startidx, int endidx) {
    int width, height, depth;
    double dx;
    _getSubdividedGridDimensions(&width, &height, &depth, &dx);

    bool isStartSlice = startidx == 0;
    bool isEndSlice = endidx == width - 1;

    if (!isStartSlice) {
        _applyScalarFieldSliceSeamData();
    }
    if (!isEndSlice) {
        _saveScalarFieldSliceSeamData();
    }
}

void AnisotropicParticleMesher::_applyScalarFieldSliceSeamData() {
    int width, height, depth;
    _scalarField.getGridDimensions(&width, &height, &depth);

    for (int k = 0; k < depth; k++) {
        for (int j = 0; j < height; j++) {
            for (int i = 0; i <= 2; i++) {
                _scalarField.setScalarFieldValue(i, j, k, _scalarFieldSeamData(i, j, k));
            }
        }
    }
}

void AnisotropicParticleMesher::_saveScalarFieldSliceSeamData() {
    int width, height, depth;
    _scalarField.getGridDimensions(&width, &height, &depth);

    _scalarFieldSeamData = Array3d<float>(3, height, depth);
    for (int k = 0; k < depth; k++) {
        for (int j = 0; j < height; j++) {
            for (int i = 0; i <= 2; i++) {
                _scalarFieldSeamData.set(i, j, k, _scalarField.getRawScalarFieldValue(i + width - 3, j, k));
            }
        }
    }
}

void AnisotropicParticleMesher::_getSliceMask(int startidx, int endidx, Array3d<bool> &mask) {
    mask.fill(true);

    int width, height, depth;
    double dx;
    _getSubdividedGridDimensions(&width, &height, &depth, &dx);

    bool isStartSlice = startidx == 0;
    bool isEndSlice = endidx == width - 1;

    if (!isStartSlice) {
        int idx = 0;
        for (int k = 0; k < mask.depth; k++) {
            for (int j = 0; j < mask.height; j++) {
                mask.set(idx, j, k, false);
            }
        }
    }

    if (!isEndSlice) {
        int idx = mask.width - 1;
        for (int k = 0; k < mask.depth; k++) {
            for (int j = 0; j < mask.height; j++) {
                mask.set(idx, j, k, false);
            }
        }
    }
}

void AnisotropicParticleMesher::_computeScalarField(FluidMaterialGrid &materialGrid,
                                                    FragmentedVector<vmath::vec3> &particles,
                                                    LevelSet &levelset) {
    _initializeScalarField(materialGrid);
    _addAnisotropicParticlesToScalarField();
    _addIsotropicParticlesToScalarField(particles, levelset);
}

void AnisotropicParticleMesher::_getSliceParticles(int startidx, int endidx,
                                                   FragmentedVector<vmath::vec3> &particles,
                                                   FragmentedVector<vmath::vec3> &sliceParticles) {
    AABB bbox = _getSliceAABB(startidx, endidx);
    for (unsigned int i = 0; i < particles.size(); i++) {
        if (bbox.isPointInside(particles[i])) {
            sliceParticles.push_back(particles[i]);
        }
    }
}

void AnisotropicParticleMesher::_computeSliceScalarField(int startidx, int endidx,
                                                         FragmentedVector<vmath::vec3> &particles,
                                                         LevelSet &levelset,
                                                         FluidMaterialGrid &materialGrid) {
    _initializeSliceScalarField(startidx, endidx, materialGrid);
    _addAnisotropicParticlesToSliceScalarField(startidx, endidx);

    FragmentedVector<vmath::vec3> sliceParticles;
    _getSliceParticles(startidx, endidx, particles, sliceParticles);
    _addIsotropicParticlesToScalarField(sliceParticles, levelset);


    _updateScalarFieldSeam(startidx, endidx);
}

void AnisotropicParticleMesher::_initializeScalarField(FluidMaterialGrid &materialGrid) {
    int subd = _subdivisionLevel;
    int width = _isize*subd;
    int height = _jsize*subd;
    int depth = _ksize*subd;
    double dx = _dx / (double)subd;

    _scalarField = ScalarField(width + 1, height + 1, depth + 1, dx);
    
    int origsubd = materialGrid.getSubdivisionLevel();
    materialGrid.setSubdivisionLevel(subd);
    _scalarField.setMaterialGrid(materialGrid);
    materialGrid.setSubdivisionLevel(origsubd);
}

void AnisotropicParticleMesher::_initializeSliceScalarField(int startidx, int endidx,
                                                            FluidMaterialGrid &materialGrid) {
    int subd = _subdivisionLevel;
    int width = _isize*subd;
    int height = _jsize*subd;
    int depth = _ksize*subd;
    double dx = _dx / (double)subd;

    bool isStartSlice = startidx == 0;
    bool isEndSlice = endidx == width - 1;
    bool isMiddleSlice = !isStartSlice && !isEndSlice;

    int gridWidth = endidx - startidx + 1;
    int gridHeight = height;
    int gridDepth = depth;
    if (isStartSlice || isEndSlice) {
        gridWidth++;
    } else if (isMiddleSlice) {
        gridWidth += 2;
    }


    _scalarField = ScalarField(gridWidth + 1, gridHeight + 1, gridDepth + 1, dx);

    FluidMaterialGrid sliceMaterialGrid(gridWidth, gridHeight, gridDepth);
    _getSliceMaterialGrid(startidx, endidx, materialGrid, sliceMaterialGrid);

    _scalarField.setMaterialGrid(sliceMaterialGrid);

    vmath::vec3 fieldOffset = _getSliceGridPositionOffset(startidx, endidx);
    _scalarField.setOffset(fieldOffset);
}

void AnisotropicParticleMesher::_computeRangeOfAnisotropicParticles(int startidx, int endidx, 
                                                                    std::vector<AnisotropicParticle> &particles) {
    GridPointReference ref;
    for (int i = startidx; i <= endidx; i++) {
        ref = _nearSurfaceParticleRefs[i];

        if (_surfaceParticles[ref.id].componentID != -1) {
            particles.push_back(_computeAnisotropicParticle(ref));
        }
    }
}

void AnisotropicParticleMesher::_computeRangeOfSliceAnisotropicParticles(int refstartidx, int refendidx, 
                                                                         int slicestartidx, int sliceendidx,
                                                                         std::vector<AnisotropicParticle> &particles) {
    AABB bbox = _getSliceAABB(slicestartidx, sliceendidx);

    GridPointReference ref;
    for (int i = refstartidx; i <= refendidx; i++) {
        ref = _nearSurfaceParticleRefs[i];

        if (_surfaceParticles[ref.id].componentID == -1) {
            continue;
        }

        if (bbox.isPointInside(_surfaceParticles[ref.id].position)) {
            particles.push_back(_computeAnisotropicParticle(ref));
        }
    }
}

void AnisotropicParticleMesher::_addAnisotropicParticlesToScalarField() {
    
    double r = _particleRadius*_anisotropicParticleScale;
    _scalarField.setPointRadius(r);

    int n = _anisotropicParticleChunkSize;
    std::vector<AnisotropicParticle> particles;
    particles.reserve(n);

    for (int startidx = 0; startidx < (int)_nearSurfaceParticleRefs.size(); startidx += n) {
        int endidx = startidx + n - 1;
        endidx = fmin(endidx, _nearSurfaceParticleRefs.size() - 1);

        particles.clear();
        _computeRangeOfAnisotropicParticles(startidx, endidx, particles);

        for (unsigned int pidx = 0; pidx < particles.size(); pidx++) {
            _addAnisotropicParticleToScalarField(particles[pidx]);
        }
    }
}

void AnisotropicParticleMesher::_addAnisotropicParticlesToSliceScalarField(int slicestartidx,
                                                                           int sliceendidx) {
    
    double r = _particleRadius*_anisotropicParticleScale;
    _scalarField.setPointRadius(r);

    int n = _anisotropicParticleChunkSize;
    std::vector<AnisotropicParticle> particles;
    particles.reserve(n);

    for (int refstartidx = 0; refstartidx < (int)_nearSurfaceParticleRefs.size(); refstartidx += n) {
        int refendidx = refstartidx + n - 1;
        refendidx = fmin(refendidx, _nearSurfaceParticleRefs.size() - 1);

        particles.clear();
        _computeRangeOfSliceAnisotropicParticles(refstartidx, refendidx, 
                                                 slicestartidx, sliceendidx, particles);

        for (unsigned int pidx = 0; pidx < particles.size(); pidx++) {
            _addAnisotropicParticleToScalarField(particles[pidx]);
        }
    }
}

void AnisotropicParticleMesher::_addAnisotropicParticleToScalarField(AnisotropicParticle &aniso) {
    vmath::vec3 p = aniso.position;
    vmath::mat3 G = aniso.anisotropy;
    double scale = _anisotropicParticleFieldScale;
    _scalarField.addEllipsoidValue(p, G, scale);

}

void AnisotropicParticleMesher::_addIsotropicParticlesToScalarField(FragmentedVector<vmath::vec3> &particles, 
                                                                    LevelSet &levelset) {
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

AnisotropicParticleMesher::AnisotropicParticle AnisotropicParticleMesher::_computeAnisotropicParticle(
                                                                            GridPointReference ref) {
    std::vector<GridPointReference> neighbours;
    vmath::mat3 covariance = _computeCovarianceMatrix(ref, _kernelRadius, neighbours);

    SVD svd;
    _covarianceMatrixToSVD(covariance, svd);
    vmath::mat3 G = _SVDToAnisotropicMatrix(svd);

    vmath::vec3 p = _surfaceParticles[ref.id].position;
    return AnisotropicParticle(p, G);
}

vmath::mat3 AnisotropicParticleMesher::_computeCovarianceMatrix(GridPointReference ref, double radius,
                                                                std::vector<GridPointReference> &neighbours) {

    neighbours.clear();
    _pointGrid.queryPointReferencesInsideSphere(ref, radius, neighbours);

    if ((int)neighbours.size() <= _minAnisotropicParticleNeighbourThreshold) {
        return vmath::mat3();
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
    double scale = _particleRadius*_anisotropicParticleScale;
    double eps = 1e-9;
    for (unsigned int i = 0; i < neighbours.size(); i++) {
        refj = neighbours[i];
        spj = _surfaceParticles[refj.id];

        kernelVal = _evaluateKernel(meansp, spj)*scale;

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

void AnisotropicParticleMesher::_covarianceMatrixToSVD(vmath::mat3 &covariance, SVD &svd) {
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
    double sigma2 = std::max((double)D[k2][k2], sigma0 / kr);

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
vmath::quat AnisotropicParticleMesher::_diagonalizeMatrix(vmath::mat3 A) {

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

vmath::mat3 AnisotropicParticleMesher::_SVDToAnisotropicMatrix(SVD &svd) {
    vmath::mat3 invD = vmath::mat3(vmath::vec3(1.0 / svd.diag.x, 0.0, 0.0),
                               vmath::vec3(0.0, 1.0 / svd.diag.y, 0.0),
                               vmath::vec3(0.0, 0.0, 1.0 / svd.diag.z));

    return svd.rotation * invD * vmath::transpose(svd.rotation);
}

void AnisotropicParticleMesher::_setParticleRadius(double r) {
    _particleRadius = r;
}

void AnisotropicParticleMesher::_setKernelRadius(double r) {
    FLUIDSIM_ASSERT(r > 0.0);

    _kernelRadius = r;
    _invKernelRadius = 1 / r;
}

double AnisotropicParticleMesher::_evaluateKernel(SurfaceParticle &pi, SurfaceParticle &pj) {
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