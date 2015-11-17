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

#pragma once

#include <stdio.h>
#include <iostream>
#include <thread>

#include "array3d.h"
#include "grid3d.h"
#include "spatialpointgrid.h"
#include "aabb.h"
#include "levelset.h"
#include "trianglemesh.h"
#include "stopwatch.h"
#include "glm/glm.hpp"

class ParticleMesher
{
public:
    ParticleMesher();
    ParticleMesher(int isize, int jsize, int ksize, double dx);
    ~ParticleMesher();

    TriangleMesh meshParticles(std::vector<glm::vec3> &particles, 
                               LevelSet &levelset,
                               double particleRadius);

private: 

    struct SurfaceParticle {
        glm::vec3 position;
        GridPointReference ref;
        int componentID;

        SurfaceParticle() : position(glm::vec3(0.0, 0.0, 0.0)), 
                            ref(-1),
                            componentID(-1) {}
        SurfaceParticle(glm::vec3 p) : position(p), 
                                       ref(-1),
                                       componentID(-1) {}
        SurfaceParticle(glm::vec3 p, GridPointReference r) : 
                                       position(p), 
                                       ref(r),
                                       componentID(-1) {}
        SurfaceParticle(glm::vec3 p, GridPointReference r, int compID) : 
                                       position(p), 
                                       ref(r),
                                       componentID(compID) {}
    };

    void _clear();
    void _sortParticlesBySurfaceDistance(std::vector<glm::vec3> &allParticles,
                                         std::vector<glm::vec3> &insideParticles,
                                         std::vector<glm::vec3> &surfaceParticles,
                                         std::vector<int> &nearSurfaceParticles,
                                         std::vector<int> &farSurfaceParticles,
                                         LevelSet &levelset);
    void _initializeSurfaceParticleSpatialGrid(std::vector<glm::vec3> &particles);
    void _updateNearFarSurfaceParticleReferences(std::vector<int> nearParticles,
                                                 std::vector<int> farParticles);
    void _updateSurfaceParticleComponentIDs();
    void _smoothSurfaceParticlePositions();
    void _computeSmoothedNearSurfaceParticlePositions();
    void _smoothRangeOfSurfaceParticlePositions(int startidx, int endidx, double radius);
    glm::vec3 _getSmoothedParticlePosition(GridPointReference ref,
                                           double radius,
                                           std::vector<GridPointReference> &refs);
    glm::vec3 _getWeightedMeanParticlePosition(GridPointReference ref,
                                               std::vector<GridPointReference> &neighbours);

    void _computeAnisotropyMatrices(std::vector<glm::mat3x3> &matrices);
    void _computeCovarianceMatrices();
    void _computeRangeOfCovarianceMatrices(int startidx, int endidx, double radius);
    glm::mat3 _computeCovarianceMatrix(GridPointReference ref, double radius,
                                         std::vector<GridPointReference> &neighbours);

    void _setParticleRadius(double r);
    void _setKernelRadius(double r);
    double _evaluateKernel(SurfaceParticle &pi, SurfaceParticle &pj);

    double _particleRadius = 0.0;
    double _kernelRadius = 0.0;
    double _invKernelRadius = 1.0;

    double _supportRadiusFactor = 4;              // in number of _particleRadius
    double _connectedComponentRadiusFactor = 2;   // in number of _particleRadius
    double _smoothingConstant = 0.95;             // in range [0.0,1.0]
    int _numThreads = 8;

    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;
    double _dx = 0;

    double _maxParticleToSurfaceDepth = 3.0; // in number of cells

    SpatialPointGrid _pointGrid;
    std::vector<SurfaceParticle> _surfaceParticles;
    std::vector<GridPointReference> _nearSurfaceParticleRefs;
    std::vector<GridPointReference> _farSurfaceParticleRefs;
    std::vector<glm::vec3> _smoothedPositions;
    std::vector<glm::mat3> _covarianceMatrices;
};

