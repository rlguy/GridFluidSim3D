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
#include "diffuseparticlesimulation.h"

DiffuseParticleSimulation::DiffuseParticleSimulation() {
}

DiffuseParticleSimulation::~DiffuseParticleSimulation() {
}

void DiffuseParticleSimulation::update(int isize, int jsize, int ksize, double dx,
                                       FragmentedVector<MarkerParticle> *markerParticles,
                                       MACVelocityField *vfield,
                                       LevelSet *levelset,
                                       FluidMaterialGrid *mgrid,
                                       ParticleAdvector *particleAdvector,
                                       vmath::vec3 bodyForce,
                                       double dt) {

    _isize = isize;
    _jsize = jsize;
    _ksize = ksize;
    _dx = dx;

    // radius of sphere with 1/8th the volume of a grid cell
    _markerParticleRadius = pow(3*(_dx*_dx*_dx / 8.0) / (4*3.141592653), 1.0/3.0);

    _markerParticles = markerParticles;
	_vfield = vfield;
    _levelset = levelset;
    _materialGrid = mgrid;
    _particleAdvector = particleAdvector;
    _bodyForce = bodyForce;

	std::vector<DiffuseParticleEmitter> emitters;
	_getDiffuseParticleEmitters(emitters);
    _emitDiffuseParticles(emitters, dt);

    if (_diffuseParticles.size() == 0.0) {
        return;
    }

    _updateDiffuseParticleTypes();
    _updateDiffuseParticleLifetimes(dt);
    _advanceDiffuseParticles(dt);
    _removeDiffuseParticles();
}

void DiffuseParticleSimulation::
        getDiffuseParticleTypeCounts(int *numspray, int *numbubble, int *numfoam) {
    _getDiffuseParticleTypeCounts(numspray, numbubble, numfoam);
}

int DiffuseParticleSimulation::getNumSprayParticles() {
    return _getNumSprayParticles();
}

int DiffuseParticleSimulation::getNumBubbleParticles() {
    return _getNumBubbleParticles();
}

int DiffuseParticleSimulation::getNumFoamParticles() {
    return _getNumFoamParticles();
}

FragmentedVector<DiffuseParticle>* DiffuseParticleSimulation::getDiffuseParticles() {
    return &_diffuseParticles;
}

int DiffuseParticleSimulation::getNumDiffuseParticles() {
    return _diffuseParticles.size();
}

void DiffuseParticleSimulation::
        setDiffuseParticles(std::vector<DiffuseParticle> &particles) {
    _diffuseParticles.clear();
    _diffuseParticles.shrink_to_fit();
    _diffuseParticles.reserve((unsigned int)particles.size());
    for (size_t i = 0; i < particles.size(); i++) {
        _diffuseParticles.push_back(particles[i]);
    }
}

void DiffuseParticleSimulation::
        setDiffuseParticles(FragmentedVector<DiffuseParticle> &particles) {
    _diffuseParticles.clear();
    _diffuseParticles.shrink_to_fit();
    _diffuseParticles.reserve(particles.size());
    for (unsigned int i = 0; i < particles.size(); i++) {
        _diffuseParticles.push_back(particles[i]);
    }
}

void DiffuseParticleSimulation::
        addDiffuseParticles(std::vector<DiffuseParticle> &particles) {
    _diffuseParticles.reserve((unsigned int)(_diffuseParticles.size() + particles.size()));
    for (size_t i = 0; i < particles.size(); i++) {
        _diffuseParticles.push_back(particles[i]);
    }
}

void DiffuseParticleSimulation::
        addDiffuseParticles(FragmentedVector<DiffuseParticle> &particles) {
    _diffuseParticles.reserve(_diffuseParticles.size() + particles.size());
    for (unsigned int i = 0; i < particles.size(); i++) {
        _diffuseParticles.push_back(particles[i]);
    }
}

int DiffuseParticleSimulation::getMaxNumDiffuseParticles() {
    return _maxNumDiffuseParticles;
}

void DiffuseParticleSimulation::setMaxNumDiffuseParticles(int n) {
    FLUIDSIM_ASSERT(n >= 0);
    _maxNumDiffuseParticles = n;
}

double DiffuseParticleSimulation::getMaxDiffuseParticleLifetime() {
    return _maxDiffuseParticleLifetime;
}

void DiffuseParticleSimulation::setMaxDiffuseParticleLifetime(double lifetime) {
    FLUIDSIM_ASSERT(lifetime >= 0);
    _maxDiffuseParticleLifetime = lifetime;
}

double DiffuseParticleSimulation::getDiffuseParticleWavecrestEmissionRate() {
    return _wavecrestEmissionRate;
}

void DiffuseParticleSimulation::setDiffuseParticleWavecrestEmissionRate(double r) {
    FLUIDSIM_ASSERT(r >= 0);
    _wavecrestEmissionRate = r;
}

double DiffuseParticleSimulation::getDiffuseParticleTurbulenceEmissionRate() {
    return _turbulenceEmissionRate;
}

void DiffuseParticleSimulation::setDiffuseParticleTurbulenceEmissionRate(double r) {
    FLUIDSIM_ASSERT(r >= 0);
    _turbulenceEmissionRate = r;
}

void DiffuseParticleSimulation::getDiffuseParticleEmissionRates(double *rwc, 
                                                                  double *rt) {
    *rwc = _wavecrestEmissionRate;
    *rt  = _turbulenceEmissionRate;
}

void DiffuseParticleSimulation::setDiffuseParticleEmissionRates(double r) {
    setDiffuseParticleWavecrestEmissionRate(r);
    setDiffuseParticleTurbulenceEmissionRate(r);
}

void DiffuseParticleSimulation::setDiffuseParticleEmissionRates(double rwc, 
                                                                double rt) {
    setDiffuseParticleWavecrestEmissionRate(rwc);
    setDiffuseParticleTurbulenceEmissionRate(rt);
}

void DiffuseParticleSimulation::
		_getDiffuseParticleEmitters(std::vector<DiffuseParticleEmitter> &emitters) {

	_levelset->calculateSurfaceCurvature();
    _turbulenceField.calculateTurbulenceField(_vfield, *_materialGrid);

    std::vector<vmath::vec3> surfaceParticles;
    std::vector<vmath::vec3> insideParticles;
    _sortMarkerParticlePositions(surfaceParticles, insideParticles);
    _getSurfaceDiffuseParticleEmitters(surfaceParticles, emitters);
    _getInsideDiffuseParticleEmitters(insideParticles, emitters);
    _shuffleDiffuseParticleEmitters(emitters);
}

void DiffuseParticleSimulation::
        _sortMarkerParticlePositions(std::vector<vmath::vec3> &surface, 
                                     std::vector<vmath::vec3> &inside) {
    vmath::vec3 p;
    double width = _diffuseSurfaceNarrowBandSize * _dx;
    for (unsigned int i = 0; i < _markerParticles->size(); i++) {
        p = _markerParticles->at(i).position;
        if (_levelset->getDistance(p) < width) {
            surface.push_back(p);
        } else if (_levelset->isPointInInsideCell(p)) {
            inside.push_back(p);
        }
    }
}

void DiffuseParticleSimulation::
        _getSurfaceDiffuseParticleEmitters(std::vector<vmath::vec3> &surface, 
                                           std::vector<DiffuseParticleEmitter> &emitters) {
    
    std::vector<vmath::vec3> velocities;
    _particleAdvector->tricubicInterpolate(surface, _vfield, velocities);

    vmath::vec3 p, v;
    for (unsigned int i = 0; i < surface.size(); i++) {
        p = surface[i];
        v = velocities[i];

        double Iwc = _getWavecrestPotential(p, v);
        double It = 0.0;

        if (Iwc > 0.0 || It > 0.0) {
            double Ie = _getEnergyPotential(v);
            if (Ie > 0.0) {
                emitters.push_back(DiffuseParticleEmitter(p, v, Ie, Iwc, It));
            }
        }
    }
    
}

double DiffuseParticleSimulation::
        _getWavecrestPotential(vmath::vec3 p, vmath::vec3 v) {

    GridIndex g = Grid3d::positionToGridIndex(p, _dx);
    if (!_materialGrid->isCellAir(g) && !_materialGrid->isCellNeighbouringAir(g)) {
        return 0.0;
    }

    vmath::vec3 normal;
    double k = _levelset->getSurfaceCurvature(p, &normal);

    if (vmath::dot(vmath::normalize(v), normal) < 0.6) {
        return 0.0;
    }

    k = fmax(k, _minWavecrestCurvature);
    k = fmin(k, _maxWavecrestCurvature);

    return (k - _minWavecrestCurvature) / 
           (_maxWavecrestCurvature - _minWavecrestCurvature);
}

double DiffuseParticleSimulation::
        _getTurbulencePotential(vmath::vec3 p, TurbulenceField &tfield) {

    GridIndex g = Grid3d::positionToGridIndex(p, _dx);
    if (!_materialGrid->isCellFluid(g) && !_materialGrid->isCellNeighbouringFluid(g)) {
        return 0.0;
    }

    double t = tfield.evaluateTurbulenceAtPosition(p);

    t = fmax(t, _minTurbulence);
    t = fmin(t, _maxTurbulence);

    return (t - _minTurbulence) / 
           (_maxTurbulence - _minTurbulence);
}

double DiffuseParticleSimulation::
        _getEnergyPotential(vmath::vec3 velocity) {

    double e = 0.5*vmath::dot(velocity, velocity);
    e = fmax(e, _minParticleEnergy);
    e = fmin(e, _maxParticleEnergy);

    return (e - _minParticleEnergy) / (_maxParticleEnergy - _minParticleEnergy);
}

void DiffuseParticleSimulation::
        _getInsideDiffuseParticleEmitters(std::vector<vmath::vec3> &inside, 
                                          std::vector<DiffuseParticleEmitter> &emitters) {
    
    std::vector<vmath::vec3> velocities;
    _particleAdvector->tricubicInterpolate(inside, _vfield, velocities);

    vmath::vec3 p, v;
    for (unsigned int i = 0; i < inside.size(); i++) {
        p = inside[i];
        v = velocities[i];
        double It = _getTurbulencePotential(p, _turbulenceField);

        if (It > 0.0) {
            double Ie = _getEnergyPotential(v);
            if (Ie > 0.0) {
                emitters.push_back(DiffuseParticleEmitter(p, v, Ie, 0.0, It));
            }
        }
    }
}

void DiffuseParticleSimulation::
        _shuffleDiffuseParticleEmitters(std::vector<DiffuseParticleEmitter> &emitters) {
    DiffuseParticleEmitter em;
    for (int i = (int)emitters.size() - 2; i >= 0; i--) {
        int j = (rand() % (int)(i - 0 + 1));
        em = emitters[i];
        emitters[i] = emitters[j];
        emitters[j] = em;
    }
}

void DiffuseParticleSimulation::
        _emitDiffuseParticles(std::vector<DiffuseParticleEmitter> &emitters,
                              double dt) {

    std::vector<DiffuseParticle> newdps;
    for (unsigned int i = 0; i < emitters.size(); i++) {
        if (_diffuseParticles.size() >= _maxNumDiffuseParticles) {
            return;
        }

        _emitDiffuseParticles(emitters[i], dt, newdps);
    }

    _computeNewDiffuseParticleVelocities(newdps);

    _diffuseParticles.reserve((unsigned int)(_diffuseParticles.size() + newdps.size()));
    for (size_t i = 0; i < newdps.size(); i++) {
        _diffuseParticles.push_back(newdps[i]);
    }
}

void DiffuseParticleSimulation::
        _emitDiffuseParticles(DiffuseParticleEmitter &emitter, 
                              double dt,
                              std::vector<DiffuseParticle> &particles) {
    int n = _getNumberOfEmissionParticles(emitter, dt);

    if (_diffuseParticles.size() + n >= _maxNumDiffuseParticles) {
        n = _maxNumDiffuseParticles - _diffuseParticles.size();
    }

    if (n <= 0) {
        return;
    }

    float particleRadius = 4.0f*(float)_markerParticleRadius;
    vmath::vec3 axis = vmath::normalize(emitter.velocity);

    float eps = 10e-6f;
    vmath::vec3 e1;
    if (fabs(axis.x) - 1.0 < eps && fabs(axis.y) < eps && fabs(axis.z) < eps) {
        e1 = vmath::normalize(vmath::cross(axis, vmath::vec3(1.0, 0.0, 0.0)));
    } else {
        e1 = vmath::normalize(vmath::cross(axis, vmath::vec3(0.0, 1.0, 0.0)));
    }

    e1 = e1*(float)particleRadius;
    vmath::vec3 e2 = vmath::normalize(vmath::cross(axis, e1)) * (float)particleRadius;

    float Xr, Xt, Xh, r, theta, h, sinval, cosval, lifetime;
    vmath::vec3 p;
    vmath::vec3 v(0.0, 0.0, 0.0); // velocities will computed in bulk by ParticleAdvector
    GridIndex g;
    for (int i = 0; i < n; i++) {
        Xr = (float)(rand()) / (float)RAND_MAX;
        Xt = (float)(rand()) / (float)RAND_MAX;
        Xh = (float)(rand()) / (float)RAND_MAX;

        r = particleRadius*sqrt(Xr);
        theta = Xt*2.0f*3.141592653f;
        h = Xh*vmath::length((float)dt*emitter.velocity);
        sinval = sin(theta);
        cosval = cos(theta);

        p = emitter.position + r*cosval*e1 + r*sinval*e2 + h*axis;
        g = Grid3d::positionToGridIndex(p, _dx);
        if (!Grid3d::isGridIndexInRange(g, _isize, _jsize, _ksize) || 
                _materialGrid->isCellSolid(g)) {
            continue;
        }

        lifetime = (float)(emitter.energyPotential*_maxDiffuseParticleLifetime);
        lifetime = _randomDouble(0.5*lifetime, lifetime);
        particles.push_back(DiffuseParticle(p, v, lifetime));
    }
}

int DiffuseParticleSimulation::
        _getNumberOfEmissionParticles(DiffuseParticleEmitter &emitter,
                                      double dt) {
    double wc = _wavecrestEmissionRate*emitter.wavecrestPotential;
    double t = _turbulenceEmissionRate*emitter.turbulencePotential;
    double n = emitter.energyPotential*(wc + t)*dt;

    if (n < 0.0) {
        return 0;
    }

    return (int)(n + 0.5);
}

void DiffuseParticleSimulation::
        _computeNewDiffuseParticleVelocities(std::vector<DiffuseParticle> &particles) {
    std::vector<vmath::vec3> data;
    data.reserve(particles.size());
    for (unsigned int i = 0; i < particles.size(); i++) {
        data.push_back(particles[i].position);
    }

    _particleAdvector->tricubicInterpolate(data, _vfield);

    for (unsigned int i = 0; i < particles.size(); i++) {
        particles[i].velocity = data[i];
    }
}

void DiffuseParticleSimulation::_updateDiffuseParticleTypes() {
    DiffuseParticle dp;
    DiffuseParticleType type;
    for (unsigned int i = 0; i < _diffuseParticles.size(); i++) {
        dp = _diffuseParticles[i];
        type = _getDiffuseParticleType(dp);
        _diffuseParticles[i].type = type;
    }
}

DiffuseParticleType DiffuseParticleSimulation::
        _getDiffuseParticleType(DiffuseParticle &dp) {
    double foamDist = _maxFoamToSurfaceDistance*_dx;
    double dist = _levelset->getSignedDistance(dp.position);

    DiffuseParticleType type;
    if (dist > 0.0) {       // inside surface
        type = DiffuseParticleType::bubble;
    } else {                // outside surface
        if (fabs(dist) < foamDist) {
            type = DiffuseParticleType::foam;
        } else {
            type = DiffuseParticleType::spray;
        }
    }

    if (type == DiffuseParticleType::foam || type == DiffuseParticleType::spray) {
        GridIndex g = Grid3d::positionToGridIndex(dp.position, _dx);
        
        if (!_materialGrid->isCellAir(g) && !_materialGrid->isCellNeighbouringAir(g)) {
            type = DiffuseParticleType::bubble;
        }
    }

    return type;
}

void DiffuseParticleSimulation::_updateDiffuseParticleLifetimes(double dt) {
    double maxDist = _maxSprayToSurfaceDistance*_dx;

    DiffuseParticle dp;
    for (unsigned int i = 0; i < _diffuseParticles.size(); i++) {
        dp = _diffuseParticles[i];

        double modifier = 0.0;
        if (dp.type == DiffuseParticleType::spray) {
            modifier = _sprayParticleLifetimeModifier;
            if (_levelset->getDistance(dp.position) > maxDist) {
                modifier = _sprayParticleMaxDistanceLifetimeModifier;
            }
        } else if (dp.type == DiffuseParticleType::bubble) {
            modifier = _bubbleParticleLifetimeModifier;
        } else if (dp.type == DiffuseParticleType::foam) {
            modifier = _foamParticleLifetimeModifier;
        }

        _diffuseParticles[i].lifetime = dp.lifetime - (float)(modifier*dt);
    }
}

void DiffuseParticleSimulation::_advanceDiffuseParticles(double dt) {
    _advanceSprayParticles(dt);
    _advanceBubbleParticles(dt);
    _advanceFoamParticles(dt);
}

void DiffuseParticleSimulation::_advanceSprayParticles(double dt) {

    DiffuseParticle dp;
    vmath::vec3 nextv, nextp;
    GridIndex g;
    for (unsigned int i = 0; i < _diffuseParticles.size(); i++) {
        dp = _diffuseParticles[i];

        if (dp.type != DiffuseParticleType::spray) {
            continue;
        }

        nextv = dp.velocity + _bodyForce * (float)dt;
        nextp = dp.position + nextv * (float)dt;
        
        g = Grid3d::positionToGridIndex(nextp, _dx);
        if (_materialGrid->isCellSolid(g)) {
            nextp = _resolveParticleSolidCellCollision(dp.position, nextp);
        }

        _diffuseParticles[i].position = nextp;
        _diffuseParticles[i].velocity = nextv;
    }
}

void DiffuseParticleSimulation::_advanceBubbleParticles(double dt) {

    int bubblecount = _getNumBubbleParticles();
    if (bubblecount == 0) {
        return;
    }

    std::vector<vmath::vec3> data;
    data.reserve(bubblecount);
    for (unsigned int i = 0; i < _diffuseParticles.size(); i++) {
        if (_diffuseParticles[i].type == DiffuseParticleType::bubble) {
            data.push_back(_diffuseParticles[i].position);
        }
    }

    _particleAdvector->tricubicInterpolate(data, _vfield);

    DiffuseParticle dp;
    vmath::vec3 vmac, vbub, bouyancyVelocity, dragVelocity;
    vmath::vec3 nextv, nextp;
    GridIndex g;
    int dataidx = 0;
    for (unsigned int i = 0; i < _diffuseParticles.size(); i++) {
        dp = _diffuseParticles[i];
        
        if (dp.type != DiffuseParticleType::bubble) {
            continue;
        }

        vmac = data[dataidx];
        vbub = dp.velocity;
        bouyancyVelocity = (float)-_bubbleBouyancyCoefficient * _bodyForce;
        dragVelocity = (float)_bubbleDragCoefficient*(vmac - vbub) / (float)dt;

        nextv = dp.velocity + (float)dt*(bouyancyVelocity + dragVelocity);
        nextp = dp.position + nextv * (float)dt;

        g = Grid3d::positionToGridIndex(nextp, _dx);
        if (_materialGrid->isCellSolid(g)) {
            nextp = _resolveParticleSolidCellCollision(dp.position, nextp);
        }

        _diffuseParticles[i].position = nextp;
        _diffuseParticles[i].velocity = nextv;

        dataidx++;
    }
}

void DiffuseParticleSimulation::_advanceFoamParticles(double dt) {

    int foamcount = _getNumFoamParticles();
    if (foamcount == 0) {
        return;
    }

    std::vector<vmath::vec3> positions;
    positions.reserve(foamcount);
    for (unsigned int i = 0; i < _diffuseParticles.size(); i++) {
        if (_diffuseParticles[i].type == DiffuseParticleType::foam) {
            positions.push_back(_diffuseParticles[i].position);
        }
    }

    std::vector<vmath::vec3> nextpositions;
    _particleAdvector->advectParticlesRK2(positions, 
                                          _vfield,
                                          dt,
                                          nextpositions);

    DiffuseParticle dp;
    vmath::vec3 nextp;
    GridIndex g;
    int sprayidx = 0;
    for (unsigned int i = 0; i < _diffuseParticles.size(); i++) {
        dp = _diffuseParticles[i];
        
        if (dp.type != DiffuseParticleType::foam) {
            continue;
        }

        nextp = nextpositions[sprayidx];

        g = Grid3d::positionToGridIndex(nextp, _dx);
        if (_materialGrid->isCellSolid(g)) {
            nextp = _resolveParticleSolidCellCollision(dp.position, nextp);
        }

        _diffuseParticles[i].position = nextp;

        sprayidx++;
    }
}

vmath::vec3 DiffuseParticleSimulation::
        _resolveParticleSolidCellCollision(vmath::vec3 p0, 
                                           vmath::vec3 p1) {
    
    GridIndex g1 = Grid3d::positionToGridIndex(p0, _dx);
    GridIndex g2 = Grid3d::positionToGridIndex(p1, _dx);
    FLUIDSIM_ASSERT(!_materialGrid->isCellSolid(g1));
    FLUIDSIM_ASSERT(_materialGrid->isCellSolid(g2));

    GridIndex voxel;
    bool foundVoxel = Collision::getLineSegmentVoxelIntersection(p0, p1, _dx,
                                                                 *_materialGrid, 
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
    if (_materialGrid->isCellSolid(gr)) {
        return p0;
    }
    
    return resolvedPosition;
}

void DiffuseParticleSimulation::
        _getDiffuseParticleTypeCounts(int *numspray, int *numbubble, int *numfoam) {
    int spray = 0;
    int bubble = 0;
    int foam = 0;
    for (unsigned int i = 0; i < _diffuseParticles.size(); i++) {
        DiffuseParticleType type = _diffuseParticles[i].type;
        if (type == DiffuseParticleType::spray) {
            spray++;
        } else if (type == DiffuseParticleType::bubble) {
            bubble++;
        } else if (type == DiffuseParticleType::foam) {
            foam++;
        }
    }

    *numspray = spray;
    *numbubble = bubble;
    *numfoam = foam;
}

int DiffuseParticleSimulation::_getNumSprayParticles() {
    int spraycount = 0;
    for (unsigned int i = 0; i < _diffuseParticles.size(); i++) {
        if (_diffuseParticles[i].type == DiffuseParticleType::spray) {
            spraycount++;
        }
    }

    return spraycount;
}

int DiffuseParticleSimulation::_getNumBubbleParticles() {
    int bubblecount = 0;
    for (unsigned int i = 0; i < _diffuseParticles.size(); i++) {
        if (_diffuseParticles[i].type == DiffuseParticleType::bubble) {
            bubblecount++;
        }
    }

    return bubblecount;
}

int DiffuseParticleSimulation::_getNumFoamParticles() {
    int foamcount = 0;
    for (unsigned int i = 0; i < _diffuseParticles.size(); i++) {
        if (_diffuseParticles[i].type == DiffuseParticleType::foam) {
            foamcount++;
        }
    }

    return foamcount;
}

void DiffuseParticleSimulation::_removeDiffuseParticles() {
    Array3d<int> countGrid = Array3d<int>(_isize, _jsize, _ksize, 0);

    std::vector<bool> isRemoved;
    isRemoved.reserve(_diffuseParticles.size());

    DiffuseParticle dp;
    GridIndex g;
    for (unsigned int i = 0; i < _diffuseParticles.size(); i++) {
        dp = _diffuseParticles[i];

        if (dp.lifetime <= 0.0) {
            isRemoved.push_back(true);
            continue;
        }

        g = Grid3d::positionToGridIndex(dp.position, _dx);
        if (countGrid(g) >= _maxDiffuseParticlesPerCell) {
            isRemoved.push_back(true);
            continue;
        }
        countGrid.add(g, 1);

        isRemoved.push_back(false);
    }

    _removeItemsFromVector(_diffuseParticles, isRemoved);
}