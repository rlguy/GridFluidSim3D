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
#ifndef ANISOTROPICPARTICLEMESHER_H
#define ANISOTROPICPARTICLEMESHER_H

#include <stdio.h>
#include <iostream>

#include "array3d.h"
#include "grid3d.h"
#include "spatialpointgrid.h"
#include "aabb.h"
#include "levelset.h"
#include "trianglemesh.h"
#include "scalarfield.h"
#include "polygonizer3d.h"
#include "stopwatch.h"
#include "vmath.h"
#include "fluidmaterialgrid.h"
#include "fragmentedvector.h"
#include "markerparticle.h"

class AnisotropicParticleMesher
{
public:
    AnisotropicParticleMesher();
    AnisotropicParticleMesher(int isize, int jsize, int ksize, double dx);
    ~AnisotropicParticleMesher();

    void setSubdivisionLevel(int n);
    void setNumPolygonizationSlices(int n);

    TriangleMesh meshParticles(FragmentedVector<MarkerParticle> &particles, 
                               LevelSet &levelset,
                               FluidMaterialGrid &materialGrid,
                               double particleRadius);
private: 

    struct SurfaceParticle {
        vmath::vec3 position;
        GridPointReference ref;
        int componentID;

        SurfaceParticle() : ref(-1),
                            componentID(-1) {}
        SurfaceParticle(vmath::vec3 p) : position(p), 
                                         ref(-1),
                                         componentID(-1) {}
        SurfaceParticle(vmath::vec3 p, GridPointReference r) : 
                                         position(p), 
                                         ref(r),
                                         componentID(-1) {}
        SurfaceParticle(vmath::vec3 p, GridPointReference r, int compID) : 
                                         position(p), 
                                         ref(r),
                                         componentID(compID) {}
    };

    struct IsotropicParticle {
        vmath::vec3 position;

        IsotropicParticle() {}
        IsotropicParticle(vmath::vec3 p) : position(p) {}
    };

    struct AnisotropicParticle {
        vmath::vec3 position;
        vmath::mat3 anisotropy;

        AnisotropicParticle() {}
        AnisotropicParticle(vmath::vec3 p) : position(p), 
                                           anisotropy(vmath::mat3(1.0)) {}
        AnisotropicParticle(vmath::vec3 p, vmath::mat3 G) : position(p), 
                                                        anisotropy(G) {}
    };

    struct SVD {
        vmath::mat3 rotation;
        vmath::vec3 diag;

        SVD() {}
        SVD(vmath::mat3 rot, vmath::vec3 d) : rotation(rot), diag(d) {}
        SVD(vmath::vec3 d, vmath::mat3 rot) : rotation(rot), diag(d) {}
    };

    enum class ParticleLocation : char { 
        Inside   = 0x00, 
        NearSurface = 0x01, 
        FarSurface = 0x02
    };

    void _clear();

    void _initializeSurfaceParticles(FragmentedVector<vmath::vec3> &particles, 
                                     LevelSet &levelset);
    void _filterHighDensityParticles(FragmentedVector<MarkerParticle> &particles, 
                                     FragmentedVector<vmath::vec3> &filtered);
    ParticleLocation _getParticleLocationType(vmath::vec3 p, LevelSet &levelset);

    inline bool _isNearSurfaceParticle(vmath::vec3 p, LevelSet &levelset) {
        return _getParticleLocationType(p, levelset) == ParticleLocation::NearSurface;
    }
    inline bool _isFarSurfaceParticle(vmath::vec3 p, LevelSet &levelset) {
        return _getParticleLocationType(p, levelset) == ParticleLocation::FarSurface;
    }
    inline bool _isInsideParticle(vmath::vec3 p, LevelSet &levelset) {
        return _getParticleLocationType(p, levelset) == ParticleLocation::Inside;
    }
    inline bool _isSurfaceParticle(vmath::vec3 p, LevelSet &levelset) {
        ParticleLocation type = _getParticleLocationType(p, levelset);
        return  type == ParticleLocation::FarSurface || type == ParticleLocation::NearSurface;
    }

    void _updateNearFarSurfaceParticleReferences(LevelSet &levelset);

    void _initializeSurfaceParticleSpatialGrid();
    void _updateSurfaceParticleComponentIDs();
    void _smoothSurfaceParticlePositions();
    void _computeSmoothedNearSurfaceParticlePositions();
    void _smoothRangeOfSurfaceParticlePositions(int startidx, int endidx);
    vmath::vec3 _getSmoothedParticlePosition(GridPointReference ref,
                                           double radius,
                                           std::vector<GridPointReference> &refs);
    vmath::vec3 _getWeightedMeanParticlePosition(GridPointReference ref,
                                               std::vector<GridPointReference> &neighbours);
    TriangleMesh _polygonizeAll(FragmentedVector<vmath::vec3> &particles, 
                                LevelSet &levelset,
                                FluidMaterialGrid &materialGrid);
    TriangleMesh _polygonizeSlices(FragmentedVector<vmath::vec3> &particles, 
                                   LevelSet &levelset,
                                   FluidMaterialGrid &materialGrid);
    TriangleMesh _polygonizeSlice(int startidx, int endidx, 
                                  FragmentedVector<vmath::vec3> &particles, 
                                  LevelSet &levelset,
                                  FluidMaterialGrid &materialGrid);

    void _getSubdividedGridDimensions(int *i, int *j, int *k, double *dx);
    void _computeSliceScalarField(int startidx, int endidx, 
                                  FragmentedVector<vmath::vec3> &particles,
                                  LevelSet &levelset,
                                  FluidMaterialGrid &materialGrid);
    vmath::vec3 _getSliceGridPositionOffset(int startidx, int endidx);
    void _getSliceParticles(int startidx, int endidx, 
                            FragmentedVector<vmath::vec3> &markerParticles,
                            FragmentedVector<vmath::vec3> &sliceParticles);
    void _getSliceMaterialGrid(int startidx, int endidx,
                               FluidMaterialGrid &materialGrid,
                               FluidMaterialGrid &sliceMaterialGrid);
    AABB _getSliceAABB(int startidx, int endidx);
    void _updateScalarFieldSeam(int startidx, int endidx);
    void _applyScalarFieldSliceSeamData();
    void _saveScalarFieldSliceSeamData();
    void _getSliceMask(int startidx, int endidx, Array3d<bool> &mask);

    void _computeScalarField(FluidMaterialGrid &materialGrid,
                             FragmentedVector<vmath::vec3> &particles,
                             LevelSet &levelset);
    void _initializeScalarField(FluidMaterialGrid &materialGrid);
    void _initializeSliceScalarField(int startidx, int endidx, 
                                     FluidMaterialGrid &materialGrid);
    void _addAnisotropicParticlesToScalarField();
    void _addAnisotropicParticlesToSliceScalarField(int startidx, int endidx);
    void _computeRangeOfAnisotropicParticles(int startidx, int endidx, 
                                             std::vector<AnisotropicParticle> &particles);
    void _computeRangeOfSliceAnisotropicParticles(int refstartidx, int refendidx, 
                                                  int slicestartidx, int sliceendidx,
                                                  std::vector<AnisotropicParticle> &particles);
    void _addAnisotropicParticleToScalarField(AnisotropicParticle &aniso);
    void _addIsotropicParticlesToScalarField(FragmentedVector<vmath::vec3> &particles, LevelSet &levelset);
    AnisotropicParticle _computeAnisotropicParticle(GridPointReference ref);
    void _getUnprocessedParticlesFromStack(int num, std::vector<GridPointReference> &refs);
    vmath::mat3 _computeCovarianceMatrix(GridPointReference ref, double radius,
                                       std::vector<GridPointReference> &neighbours);
    void _covarianceMatrixToSVD(vmath::mat3 &covariance, SVD &svd);
    vmath::quat _diagonalizeMatrix(vmath::mat3 A);
    vmath::mat3 _SVDToAnisotropicMatrix(SVD &svd);

    void _setParticleRadius(double r);
    void _setKernelRadius(double r);
    double _evaluateKernel(SurfaceParticle &pi, SurfaceParticle &pj);

    double _particleRadius = 0.0;
    double _anisotropicParticleScale = 4.0;
    double _anisotropicParticleFieldScale = 1.0;
    double _isotropicParticleScale = 3.5;
    double _kernelRadius = 0.0;
    double _invKernelRadius = 1.0;

    double _supportRadiusFactor = 6.0;              // in number of _particleRadius
    double _connectedComponentRadiusFactor = 3.0;   // in number of _particleRadius
    int _minComponentParticleCount = 5;
    double _smoothingConstant = 0.95;               // in range [0.0,1.0]

    int _minAnisotropicParticleNeighbourThreshold = 6;
    double _maxEigenvalueRatio = 5.5;

    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;
    double _dx = 0;

    int _maxParticlesPerCell = 16;
    double _maxParticleToSurfaceDepth = 2.0; // in number of cells

    SpatialPointGrid _pointGrid;
    FragmentedVector<SurfaceParticle> _surfaceParticles;
    FragmentedVector<GridPointReference> _nearSurfaceParticleRefs;
    FragmentedVector<GridPointReference> _farSurfaceParticleRefs;
    FragmentedVector<vmath::vec3> _smoothedPositions;

    ScalarField _scalarField;

    int _anisotropicParticleChunkSize = 1000000;  // Max number of particles to add to scalar
                                                  // field at once.

    int _subdivisionLevel = 1;
    int _numPolygonizationSlices = 1;

    Array3d<float> _scalarFieldSeamData;
};

#endif
