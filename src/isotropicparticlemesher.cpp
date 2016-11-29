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
#include "isotropicparticlemesher.h"

IsotropicParticleMesher::IsotropicParticleMesher() {
}

IsotropicParticleMesher::IsotropicParticleMesher(int isize, int jsize, int ksize, double dx) :
                                                    _isize(isize), _jsize(jsize), _ksize(ksize), _dx(dx) {

}

IsotropicParticleMesher::~IsotropicParticleMesher() {

}

void IsotropicParticleMesher::setSubdivisionLevel(int n) {
    FLUIDSIM_ASSERT(n >= 1);
    _subdivisionLevel = n;
}

void IsotropicParticleMesher::setNumPolygonizationSlices(int n) {
    FLUIDSIM_ASSERT(n >= 1);

    if (n > _isize) {
        n = _isize;
    }

    _numPolygonizationSlices = n;
}

TriangleMesh IsotropicParticleMesher::meshParticles(FragmentedVector<MarkerParticle> &particles, 
                                                    FluidMaterialGrid &materialGrid,
                                                    double particleRadius) {

    FLUIDSIM_ASSERT(materialGrid.width == _isize &&
           materialGrid.height == _jsize &&
           materialGrid.depth == _ksize);
    FLUIDSIM_ASSERT(particleRadius > 0.0);

    _particleRadius = particleRadius;

    if (_numPolygonizationSlices == 1) {
        return _polygonizeAll(particles, materialGrid);
    }

    return _polygonizeSlices(particles, materialGrid);
}

void IsotropicParticleMesher::setScalarFieldAccelerator(CLScalarField *accelerator) {
    _scalarFieldAccelerator = accelerator;
    _isScalarFieldAcceleratorSet = true;
}

void IsotropicParticleMesher::setScalarFieldAccelerator() {
    _isScalarFieldAcceleratorSet = false;
}

void IsotropicParticleMesher::enablePreviewMesher(double dx) {
    _initializePreviewMesher(dx);
    _isPreviewMesherEnabled = true;
}

void IsotropicParticleMesher::disablePreviewMesher() {
    _isPreviewMesherEnabled = false;
}

TriangleMesh IsotropicParticleMesher::getPreviewMesh(FluidMaterialGrid &materialGrid) {
    if (!_isPreviewMesherEnabled) {
        return TriangleMesh();
    }

    FluidMaterialGrid pmgrid(_pisize, _pjsize, _pksize);
    _getPreviewMaterialGrid(materialGrid, pmgrid);
    _pfield.setMaterialGrid(pmgrid);

    Polygonizer3d polygonizer(&_pfield);
    return polygonizer.polygonizeSurface();
}

TriangleMesh IsotropicParticleMesher::getPreviewMesh() {
    if (!_isPreviewMesherEnabled) {
        return TriangleMesh();
    }

    ScalarField field = _pfield;
    _setScalarFieldSolidBorders(field);

    Polygonizer3d polygonizer(&field);
    return polygonizer.polygonizeSurface();
}

TriangleMesh IsotropicParticleMesher::_polygonizeAll(FragmentedVector<MarkerParticle> &particles, 
                                                     FluidMaterialGrid &materialGrid) {
    int subd = _subdivisionLevel;
    int width = _isize*subd;
    int height = _jsize*subd;
    int depth = _ksize*subd;
    double dx = _dx / (double)subd;

    ScalarField field(width + 1, height + 1, depth + 1, dx);

    int origsubd = materialGrid.getSubdivisionLevel();
    materialGrid.setSubdivisionLevel(subd);
    field.setMaterialGrid(materialGrid);
    materialGrid.setSubdivisionLevel(origsubd);

    field.setPointRadius(_particleRadius);
    _addPointsToScalarField(particles, field);

    if (_isPreviewMesherEnabled) {
        _addScalarFieldToPreviewField(field);
    }

    Polygonizer3d polygonizer(&field);

    return polygonizer.polygonizeSurface();
}

TriangleMesh IsotropicParticleMesher::_polygonizeSlices(FragmentedVector<MarkerParticle> &particles, 
                                                        FluidMaterialGrid &materialGrid) {
    int width, height, depth;
    double dx;
    _getSubdividedGridDimensions(&width, &height, &depth, &dx);

    int sliceWidth = ceil((double)width / (double)_numPolygonizationSlices);
    int numSlices = ceil((double)width / (double)sliceWidth);

    if (numSlices == 1) {
        return _polygonizeAll(particles, materialGrid);
    }

    TriangleMesh mesh;
    for (int i = 0; i < numSlices; i++) {
        int startidx = i*sliceWidth;
        int endidx = startidx + sliceWidth - 1;
        endidx = endidx < width ? endidx : width - 1;

        TriangleMesh sliceMesh = _polygonizeSlice(startidx, endidx, particles, materialGrid);

        vmath::vec3 offset = _getSliceGridPositionOffset(startidx, endidx);
        sliceMesh.translate(offset);
        mesh.join(sliceMesh);
    }

    return mesh;
}

TriangleMesh IsotropicParticleMesher::_polygonizeSlice(int startidx, int endidx, 
                                                        FragmentedVector<MarkerParticle> &particles, 
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

    ScalarField field(gridWidth + 1, gridHeight + 1, gridDepth + 1, dx);
    _computeSliceScalarField(startidx, endidx, particles, materialGrid, field);

    if (_isPreviewMesherEnabled) {
        _addScalarFieldSliceToPreviewField(startidx, endidx, field);
    }

    Array3d<bool> mask(gridWidth, gridHeight, gridDepth);
    _getSliceMask(startidx, endidx, mask);

    Polygonizer3d polygonizer(&field);
    polygonizer.setSurfaceCellMask(&mask);

    return polygonizer.polygonizeSurface();
}

void IsotropicParticleMesher::_getSubdividedGridDimensions(int *i, int *j, int *k, double *dx) {
    *i = _isize*_subdivisionLevel;
    *j = _jsize*_subdivisionLevel;
    *k = _ksize*_subdivisionLevel;
    *dx = _dx / (double)_subdivisionLevel;
}

void IsotropicParticleMesher::_computeSliceScalarField(int startidx, int endidx, 
                                                       FragmentedVector<MarkerParticle> &markerParticles,
                                                       FluidMaterialGrid &materialGrid,
                                                       ScalarField &field) {
    
    FragmentedVector<vmath::vec3> sliceParticles;
    _getSliceParticles(startidx, endidx, markerParticles, sliceParticles);

    int width, height, depth;
    field.getGridDimensions(&width, &height, &depth);

    FluidMaterialGrid sliceMaterialGrid(width - 1, height - 1, depth - 1);
    _getSliceMaterialGrid(startidx, endidx, materialGrid, sliceMaterialGrid);

    field.setMaterialGrid(sliceMaterialGrid);

    vmath::vec3 fieldOffset = _getSliceGridPositionOffset(startidx, endidx);
    field.setOffset(fieldOffset);
    field.setPointRadius(_particleRadius);

    _addPointsToScalarField(sliceParticles, field);
    _updateScalarFieldSeam(startidx, endidx, field);
}

vmath::vec3 IsotropicParticleMesher::_getSliceGridPositionOffset(int startidx, int endidx) {
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

void IsotropicParticleMesher::_getSliceParticles(int startidx, int endidx, 
                                                 FragmentedVector<MarkerParticle> &markerParticles,
                                                 FragmentedVector<vmath::vec3> &sliceParticles) {
    AABB bbox = _getSliceAABB(startidx, endidx);
    for (unsigned int i = 0; i < markerParticles.size(); i++) {
        if (bbox.isPointInside(markerParticles[i].position)) {
            sliceParticles.push_back(markerParticles[i].position);
        }
    }
}

void IsotropicParticleMesher::_getSliceMaterialGrid(int startidx, int endidx,
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

AABB IsotropicParticleMesher::_getSliceAABB(int startidx, int endidx) {
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

    AABB bbox(offset, gridWidth, gridHeight, gridDepth);
    bbox.expand(2.0*_particleRadius);

    return bbox;
}

void IsotropicParticleMesher::_addPointsToScalarField(FragmentedVector<vmath::vec3> &points,
                                                      ScalarField &field) {
    
    if (_isScalarFieldAcceleratorSet) {
        _addPointsToScalarFieldAccelerator(points, field);
    } else {
        for (unsigned int i = 0; i < points.size(); i++) {
            field.addPoint(points[i]);
        }
    }
}

void IsotropicParticleMesher::_addPointsToScalarField(FragmentedVector<MarkerParticle> &points,
                                                      ScalarField &field) {
    if (_isScalarFieldAcceleratorSet) {
        _addPointsToScalarFieldAccelerator(points, field);
    } else {
        for (unsigned int i = 0; i < points.size(); i++) {
            field.addPoint(points[i].position);
        }
    }
}

void IsotropicParticleMesher::_addPointsToScalarFieldAccelerator(FragmentedVector<vmath::vec3> &points,
                                                                 ScalarField &field) {
    bool isThresholdSet = _scalarFieldAccelerator->isMaxScalarFieldValueThresholdSet();
    double origThreshold = _scalarFieldAccelerator->getMaxScalarFieldValueThreshold();
    _scalarFieldAccelerator->setMaxScalarFieldValueThreshold(_maxScalarFieldValueThreshold);

    int n = _maxParticlesPerScalarFieldAddition;
    std::vector<vmath::vec3> positions;
    positions.reserve(fmin(n, points.size()));

    for (int startidx = 0; startidx < (int)points.size(); startidx += n) {
        int endidx = startidx + n - 1;
        if (endidx >= (int)points.size()) {
            endidx = points.size() - 1;
        }

        positions.clear();
        for (int i = startidx; i <= endidx; i++) {
            positions.push_back(points[i]);
        }

        _scalarFieldAccelerator->addPoints(positions, field);
    }

    if (!isThresholdSet) {
        _scalarFieldAccelerator->setMaxScalarFieldValueThreshold();
    } else {
        _scalarFieldAccelerator->setMaxScalarFieldValueThreshold(origThreshold);
    }
}

void IsotropicParticleMesher::_addPointsToScalarFieldAccelerator(FragmentedVector<MarkerParticle> &points,
                                                                 ScalarField &field) {
    bool isThresholdSet = _scalarFieldAccelerator->isMaxScalarFieldValueThresholdSet();
    double origThreshold = _scalarFieldAccelerator->getMaxScalarFieldValueThreshold();
    _scalarFieldAccelerator->setMaxScalarFieldValueThreshold(_maxScalarFieldValueThreshold);

    int n = _maxParticlesPerScalarFieldAddition;
    std::vector<vmath::vec3> positions;
    positions.reserve(fmin(n, points.size()));

    for (int startidx = 0; startidx < (int)points.size(); startidx += n) {
        int endidx = startidx + n - 1;
        if (endidx >= (int)points.size()) {
            endidx = points.size() - 1;
        }

        positions.clear();
        for (int i = startidx; i <= endidx; i++) {
            positions.push_back(points[i].position);
        }

        _scalarFieldAccelerator->addPoints(positions, field);
    }

    if (!isThresholdSet) {
        _scalarFieldAccelerator->setMaxScalarFieldValueThreshold();
    } else {
        _scalarFieldAccelerator->setMaxScalarFieldValueThreshold(origThreshold);
    }
}

void IsotropicParticleMesher::_updateScalarFieldSeam(int startidx, int endidx,
                                                     ScalarField &field) {
    int width, height, depth;
    double dx;
    _getSubdividedGridDimensions(&width, &height, &depth, &dx);

    bool isStartSlice = startidx == 0;
    bool isEndSlice = endidx == width - 1;

    if (!isStartSlice) {
        _applyScalarFieldSliceSeamData(field);
    }
    if (!isEndSlice) {
        _saveScalarFieldSliceSeamData(field);
    }
}

void IsotropicParticleMesher::_applyScalarFieldSliceSeamData(ScalarField &field) {
    int width, height, depth;
    field.getGridDimensions(&width, &height, &depth);

    for (int k = 0; k < depth; k++) {
        for (int j = 0; j < height; j++) {
            for (int i = 0; i <= 2; i++) {
                field.setScalarFieldValue(i, j, k, _scalarFieldSeamData(i, j, k));
            }
        }
    }
}

void IsotropicParticleMesher::_saveScalarFieldSliceSeamData(ScalarField &field) {
    int width, height, depth;
    field.getGridDimensions(&width, &height, &depth);

    _scalarFieldSeamData = Array3d<float>(3, height, depth);
    for (int k = 0; k < depth; k++) {
        for (int j = 0; j < height; j++) {
            for (int i = 0; i <= 2; i++) {
                _scalarFieldSeamData.set(i, j, k, field.getRawScalarFieldValue(i + width - 3, j, k));
            }
        }
    }
}

void IsotropicParticleMesher::_getSliceMask(int startidx, int endidx, Array3d<bool> &mask) {
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

void IsotropicParticleMesher::_initializePreviewMesher(double pdx) {
    double width = _isize * _dx;
    double height = _jsize * _dx;
    double depth = _ksize * _dx;

    _pisize = fmax(ceil(width / pdx), 1);
    _pjsize = fmax(ceil(height / pdx), 1);
    _pksize = fmax(ceil(depth / pdx), 1);
    _pdx = pdx;

    _pfield = ScalarField(_pisize + 1, _pjsize + 1, _pksize + 1, _pdx);
}

void IsotropicParticleMesher::_addScalarFieldToPreviewField(ScalarField &field) {
    vmath::vec3 pv;
    for (int k = 0; k < _pksize + 1; k++) {
        for (int j = 0; j < _pjsize + 1; j++) {
            for (int i = 0; i < _pisize + 1; i++) {
                pv = Grid3d::GridIndexToPosition(i, j, k, _pdx);
                double fval = field.tricubicInterpolation(pv);
                _pfield.setScalarFieldValue(i, j, k, fval);
            }
        }
    }
}

void IsotropicParticleMesher::_addScalarFieldSliceToPreviewField(
        int startidx, int endidx, ScalarField &field) {

    int isize, jsize, ksize;
    field.getGridDimensions(&isize, &jsize, &ksize);

    double width = isize * _dx;
    double height = jsize * _dx;
    double depth = ksize * _dx;
    vmath::vec3 offset = _getSliceGridPositionOffset(startidx, endidx);
    AABB bbox(offset, width, height, depth);

    vmath::vec3 pv;
    for (int k = 0; k < _pksize + 1; k++) {
        for (int j = 0; j < _pjsize + 1; j++) {
            for (int i = 0; i < _pisize + 1; i++) {
                pv = Grid3d::GridIndexToPosition(i, j, k, _pdx);
                if (!bbox.isPointInside(pv)) {
                    continue;
                }

                double fval = field.tricubicInterpolation(pv - offset);
                _pfield.setScalarFieldValue(i, j, k, fval);
            }
        }
    }
}

void IsotropicParticleMesher::_getPreviewMaterialGrid(
        FluidMaterialGrid &materialGrid, FluidMaterialGrid &previewGrid) {
    
    for (int j = 0; j < _pjsize; j++) {
        for (int i = 0; i < _pisize; i++) {
            previewGrid.setSolid(i, j, 0);
            previewGrid.setSolid(i, j, _pksize-1);
        }
    }

    for (int k = 0; k < _pksize; k++) {
        for (int i = 0; i < _pisize; i++) {
            previewGrid.setSolid(i, 0, k);
            previewGrid.setSolid(i, _pjsize-1, k);
        }
    }

    for (int k = 0; k < _pksize; k++) {
        for (int j = 0; j < _pjsize; j++) {
            previewGrid.setSolid(0, j, k);
            previewGrid.setSolid(_pisize-1, j, k);
        }
    }

    vmath::vec3 c;
    GridIndex g;
    for (int k = 0; k < _pksize; k++) {
        for (int j = 0; j < _pjsize; j++) {
            for (int i = 0; i < _pisize; i++) {
                c = Grid3d::GridIndexToCellCenter(i, j, k, _pdx);
                g = Grid3d::positionToGridIndex(c, _dx);

                if (!Grid3d::isGridIndexInRange(g, _isize, _jsize, _ksize)) {
                    continue;
                }

                if (materialGrid.isCellSolid(g)) {
                    previewGrid.setSolid(i, j, k);
                }
            }
        }
    }
}

void IsotropicParticleMesher::_setScalarFieldSolidBorders(ScalarField &field) {
    double eps = 1e-3;
    double thresh = field.getSurfaceThreshold() - eps;
    int si, sj, sk;
    field.getGridDimensions(&si, &sj, &sk);

    for (int j = 0; j < sj; j++) {
        for (int i = 0; i < si; i++) {
            field.setScalarFieldValue(i, j, 0, thresh);
            field.setScalarFieldValue(i, j, sk-1, thresh);
        }
    }

    for (int k = 0; k < sk; k++) {
        for (int i = 0; i < si; i++) {
            field.setScalarFieldValue(i, 0, k, thresh);
            field.setScalarFieldValue(i, sj-1, k, thresh);
        }
    }

    for (int k = 0; k < sk; k++) {
        for (int j = 0; j < sj; j++) {
            field.setScalarFieldValue(0, j, k, thresh);
            field.setScalarFieldValue(si-1, j, k, thresh);
        }
    }
}