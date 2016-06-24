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

#ifndef DIFFUSEPARTICLESIMULATION_H
#define DIFFUSEPARTICLESIMULATION_H

#include <vector>

#include "fragmentedvector.h"
#include "macvelocityfield.h"
#include "levelset.h"
#include "fluidmaterialgrid.h"
#include "turbulencefield.h"
#include "particleadvector.h"
#include "markerparticle.h"
#include "diffuseparticle.h"
#include "vmath.h"
#include "grid3d.h"
#include "collision.h"
#include "fluidsimassert.h"

class DiffuseParticleSimulation
{

public:
	DiffuseParticleSimulation();
	~DiffuseParticleSimulation();

	void update(int isize, int jsize, int ksize, double dx,
              FragmentedVector<MarkerParticle> *markerParticles,
              MACVelocityField *vfield,
              LevelSet *levelset,
              FluidMaterialGrid *mgrid,
              ParticleAdvector *particleAdvector,
              vmath::vec3 bodyForce,
              double dt);

  void getDiffuseParticleTypeCounts(int *numspray, 
                                    int *numbubble, 
                                    int *numfoam);
  int getNumSprayParticles();
  int getNumBubbleParticles();
  int getNumFoamParticles();

  FragmentedVector<DiffuseParticle>* getDiffuseParticles();
  int getNumDiffuseParticles();
  void setDiffuseParticles(std::vector<DiffuseParticle> &particles);
  void setDiffuseParticles(FragmentedVector<DiffuseParticle> &particles);
  void addDiffuseParticles(std::vector<DiffuseParticle> &particles);
  void addDiffuseParticles(FragmentedVector<DiffuseParticle> &particles);

  int getMaxNumDiffuseParticles();
  void setMaxNumDiffuseParticles(int n);
  double getMaxDiffuseParticleLifetime();
  void setMaxDiffuseParticleLifetime(double lifetime);
  double getDiffuseParticleWavecrestEmissionRate();
  void setDiffuseParticleWavecrestEmissionRate(double r);
  double getDiffuseParticleTurbulenceEmissionRate();
  void setDiffuseParticleTurbulenceEmissionRate(double r);
  void getDiffuseParticleEmissionRates(double *rwc, double *rt);
  void setDiffuseParticleEmissionRates(double r);
  void setDiffuseParticleEmissionRates(double rwc, double rt);

private:

    struct DiffuseParticleEmitter {
        vmath::vec3 position;
        vmath::vec3 velocity;
        double energyPotential;
        double wavecrestPotential;
        double turbulencePotential;

        DiffuseParticleEmitter() : energyPotential(0.0),
                                   wavecrestPotential(0.0),
                                   turbulencePotential(0.0) {}

        DiffuseParticleEmitter(vmath::vec3 p, vmath::vec3 v, 
                               double e, double wc, double t) : 
                                   position(p),
                                   velocity(v),
                                   energyPotential(e),
                                   wavecrestPotential(wc),
                                   turbulencePotential(t) {}
    };    

    void _getDiffuseParticleEmitters(std::vector<DiffuseParticleEmitter> &emitters);
    void _sortMarkerParticlePositions(std::vector<vmath::vec3> &surface, 
                                      std::vector<vmath::vec3> &inside);
    void _getSurfaceDiffuseParticleEmitters(std::vector<vmath::vec3> &surface, 
                                            std::vector<DiffuseParticleEmitter> &emitters);
    double _getWavecrestPotential(vmath::vec3 p, vmath::vec3 v);
    double _getTurbulencePotential(vmath::vec3 p, TurbulenceField &tfield);
    double _getEnergyPotential(vmath::vec3 velocity);
    void _getInsideDiffuseParticleEmitters(std::vector<vmath::vec3> &inside, 
                                           std::vector<DiffuseParticleEmitter> &emitters);
    void _shuffleDiffuseParticleEmitters(std::vector<DiffuseParticleEmitter> &emitters);

    void _emitDiffuseParticles(std::vector<DiffuseParticleEmitter> &emitters, double dt);
    void _emitDiffuseParticles(DiffuseParticleEmitter &emitter, 
                               double dt,
                               std::vector<DiffuseParticle> &particles);
    int _getNumberOfEmissionParticles(DiffuseParticleEmitter &emitter,
                                      double dt);
    void _computeNewDiffuseParticleVelocities(std::vector<DiffuseParticle> &particles);

    void _updateDiffuseParticleTypes();
    DiffuseParticleType _getDiffuseParticleType(DiffuseParticle &p);

    void _updateDiffuseParticleLifetimes(double dt);

    void _advanceDiffuseParticles(double dt);
    void _advanceSprayParticles(double dt);
    void _advanceBubbleParticles(double dt);
    void _advanceFoamParticles(double dt);
    vmath::vec3 _resolveParticleSolidCellCollision(vmath::vec3 p0, 
                                                   vmath::vec3 p1);
    void _getDiffuseParticleTypeCounts(int *numspray, 
                                      int *numbubble, 
                                      int *numfoam);
    int _getNumSprayParticles();
    int _getNumBubbleParticles();
    int _getNumFoamParticles();

    void _removeDiffuseParticles();

    template<class T>
    void _removeItemsFromVector(FragmentedVector<T> &items, std::vector<bool> &isRemoved) {
        FLUIDSIM_ASSERT(items.size() == isRemoved.size());

        int currentidx = 0;
        for (unsigned int i = 0; i < items.size(); i++) {
            if (!isRemoved[i]) {
                items[currentidx] = items[i];
                currentidx++;
            }
        }

        for (unsigned int i = 0; i < items.size() - currentidx; i++) {
            items.pop_back();
        }
        items.shrink_to_fit();
    }

    inline double _randomDouble(double min, double max) {
        return min + (double)rand() / ((double)RAND_MAX / (max - min));
    }

    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;
    double _dx;

    double _markerParticleRadius = 0;

    double _diffuseSurfaceNarrowBandSize = 0.25;  // size in # of cells
    double _minWavecrestCurvature = 0.20;
    double _maxWavecrestCurvature = 1.0;
    double _minParticleEnergy = 0.0;
    double _maxParticleEnergy = 60.0;
    double _minTurbulence = 100.0;
    double _maxTurbulence = 200.0;
    unsigned int _maxNumDiffuseParticles = 10e6;
    double _maxDiffuseParticleLifetime = 2.8;
    double _wavecrestEmissionRate = 175;
    double _turbulenceEmissionRate = 175;
    double _maxFoamToSurfaceDistance = 2.0;    // in number of grid cells
    double _maxSprayToSurfaceDistance = 12.0;  // in number of grid cells
    double _sprayParticleLifetimeModifier = 2.0;
    double _sprayParticleMaxDistanceLifetimeModifier = 15.0;
    double _bubbleParticleLifetimeModifier = 0.333;
    double _foamParticleLifetimeModifier = 1.0;
    double _bubbleBouyancyCoefficient = 4.0;
    double _bubbleDragCoefficient = 1.0;
    int _maxDiffuseParticlesPerCell = 250;

    FragmentedVector<MarkerParticle> *_markerParticles;
    MACVelocityField *_vfield;
    LevelSet *_levelset;
    FluidMaterialGrid *_materialGrid;
    ParticleAdvector *_particleAdvector;
    vmath::vec3 _bodyForce;

    TurbulenceField _turbulenceField;
    FragmentedVector<DiffuseParticle> _diffuseParticles;
};

#endif