#pragma once
#pragma warning( disable : 4996 )

#include <stdio.h>
#include <iostream>
#include <time.h>
#include <assert.h>

#include "array3d.h"
#include "glm/glm.hpp" 

class MACVelocityField
{
public:
    MACVelocityField();
    MACVelocityField(int x_voxels, int y_voxels, int z_voxels, double cell_size);
    ~MACVelocityField();

    double U(int i, int j, int k);
    double V(int i, int j, int k);
    double W(int i, int j, int k);
    double U(GridIndex g);
    double V(GridIndex g);
    double W(GridIndex g);
    double tempU(int i, int j, int k);
    double tempV(int i, int j, int k);
    double tempW(int i, int j, int k);

    void setU(int i, int j, int k, double val);
    void setV(int i, int j, int k, double val);
    void setW(int i, int j, int k, double val);
    void setU(Array3d<double> &ugrid);
    void setV(Array3d<double> &vgrid);
    void setW(Array3d<double> &wgrid);
    void addU(int i, int j, int k, double val);
    void addV(int i, int j, int k, double val);
    void addW(int i, int j, int k, double val);
    void setTempU(int i, int j, int k, double val);
    void setTempV(int i, int j, int k, double val);
    void setTempW(int i, int j, int k, double val);
    void addTempU(int i, int j, int k, double val);
    void addTempV(int i, int j, int k, double val);
    void addTempW(int i, int j, int k, double val);
    void resetTemporaryVelocityField();
    void commitTemporaryVelocityFieldValues();

    double* getRawArrayU();
    double* getRawArrayV();
    double* getRawArrayW();

    void clear();
    void clearU();
    void clearV();
    void clearW();

    void randomizeValues();
    void randomizeValues(double min, double max);

    inline bool isIndexInRangeU(int i, int j, int k) {
        return i >= 0 && j >= 0 && k >= 0 && 
               i < _i_voxels + 1 && j < _j_voxels && k < _k_voxels;
    }
    inline bool isIndexInRangeV(int i, int j, int k) {
        return i >= 0 && j >= 0 && k >= 0 && 
               i < _i_voxels && j < _j_voxels + 1 && k < _k_voxels;
    }
    inline bool isIndexInRangeW(int i, int j, int k) {
        return i >= 0 && j >= 0 && k >= 0 && 
               i < _i_voxels && j < _j_voxels && k < _k_voxels + 1;
    }
    inline bool isIndexInRangeU(GridIndex g) {
        return g.i >= 0 && g.j >= 0 && g.k >= 0 && 
               g.i < _i_voxels + 1 && g.j < _j_voxels && g.k < _k_voxels;
    }
    inline bool isIndexInRangeV(GridIndex g) {
        return g.i >= 0 && g.j >= 0 && g.k >= 0 && 
               g.i < _i_voxels && g.j < _j_voxels + 1 && g.k < _k_voxels;
    }
    inline bool isIndexInRangeW(GridIndex g) {
        return g.i >= 0 && g.j >= 0 && g.k >= 0 && 
               g.i < _i_voxels && g.j < _j_voxels && g.k < _k_voxels + 1;
    }

    glm::vec3 evaluateVelocityAtCellCenter(int i, int j, int k);
    double evaluateVelocityMagnitudeAtCellCenter(int i, int j, int k);
    double evaluateVelocityMagnitudeSquaredAtCellCenter(int i, int j, int k);
    double evaluateMaximumVelocityMagnitude();

    glm::vec3 evaluateVelocityAtFaceCenterU(int i, int j, int k);
    glm::vec3 evaluateVelocityAtFaceCenterV(int i, int j, int k);
    glm::vec3 evaluateVelocityAtFaceCenterW(int i, int j, int k);

    glm::vec3 evaluateVelocityAtPosition(double x, double y, double z);
    glm::vec3 evaluateVelocityAtPosition(glm::vec3 pos);

    glm::vec3 velocityIndexToPositionU(int i, int j, int k);
    glm::vec3 velocityIndexToPositionV(int i, int j, int k);
    glm::vec3 velocityIndexToPositionW(int i, int j, int k);

private:
    void _initializeVelocityGrids();

    inline bool _isCellIndexInRange(int i, int j, int k) {
        return i >= 0 && j >= 0 && k >= 0 && i < _i_voxels && j < _j_voxels && k < _k_voxels;
    }
    inline bool _isPositionInGrid(double x, double y, double z) {
        return x >= 0 && y >= 0 && z >= 0 && 
               x < _dx*_i_voxels && y < _dx*_j_voxels && z < _dx*_k_voxels;
    }

    double _default_out_of_range_value = 0.0;

    inline double _randomFloat(double min, double max) {
         return min + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max - min))); 
    }

    double _cubicInterpolate(double p[4], double x);
    double _bicubicInterpolate(double p[4][4], double x, double y);
    double _tricubicInterpolate(double p[4][4][4], double x, double y, double z);
    double _fasttricubicInterpolate(double p[4][4][4], double x, double y, double z);

    double _interpolateU(double x, double y, double z);
    double _interpolateV(double x, double y, double z);
    double _interpolateW(double x, double y, double z);

    void _positionToGridIndex(double x, double y, double z, int *i, int *j, int *k);
    void _gridIndexToPosition(int i, int j, int k, double *x, double *y, double *z);

    double _dx = 0.1;
    int _i_voxels = 10;
    int _j_voxels = 10;
    int _k_voxels = 10;

    Array3d<double> _u;
    Array3d<double> _v;
    Array3d<double> _w;
    Array3d<double> _temp_u;
    Array3d<double> _temp_v;
    Array3d<double> _temp_w;
    Array3d<bool> _is_set_u;
    Array3d<bool> _is_set_v;
    Array3d<bool> _is_set_w;
};

