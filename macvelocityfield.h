#pragma once
#pragma warning( disable : 4996 )

#include <stdio.h>
#include <iostream>
#include <time.h>
#include <assert.h>

#include "array3d.h"
#include "grid3d.h"
#include "glm/glm.hpp" 

class MACVelocityField
{
public:
    MACVelocityField();
    MACVelocityField(int x_voxels, int y_voxels, int z_voxels, double cell_size);
    ~MACVelocityField();

    void getGridDimensions(int *i, int *j, int *k);
    double getGridCellSize();

    float U(int i, int j, int k);
    float V(int i, int j, int k);
    float W(int i, int j, int k);
    float U(GridIndex g);
    float V(GridIndex g);
    float W(GridIndex g);

    void setU(int i, int j, int k, double val);
    void setV(int i, int j, int k, double val);
    void setW(int i, int j, int k, double val);
    void setU(GridIndex g, double val);
    void setV(GridIndex g, double val);
    void setW(GridIndex g, double val);
    void setU(Array3d<float> &ugrid);
    void setV(Array3d<float> &vgrid);
    void setW(Array3d<float> &wgrid);
    void addU(int i, int j, int k, double val);
    void addV(int i, int j, int k, double val);
    void addW(int i, int j, int k, double val);

    float* getRawArrayU();
    float* getRawArrayV();
    float* getRawArrayW();

    void clear();
    void clearU();
    void clearV();
    void clearW();

    inline bool isIndexInRangeU(int i, int j, int k) {
        return Grid3d::isGridIndexInRange(i, j, k, _isize + 1, _jsize, _ksize);
    }
    inline bool isIndexInRangeV(int i, int j, int k) {
        return Grid3d::isGridIndexInRange(i, j, k, _isize, _jsize + 1, _ksize);
    }
    inline bool isIndexInRangeW(int i, int j, int k) {
        return Grid3d::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize + 1);
    }
    inline bool isIndexInRangeU(GridIndex g) {
        return Grid3d::isGridIndexInRange(g, _isize + 1, _jsize, _ksize);
    }
    inline bool isIndexInRangeV(GridIndex g) {
        return Grid3d::isGridIndexInRange(g, _isize, _jsize + 1, _ksize);
    }
    inline bool isIndexInRangeW(GridIndex g) {
        return Grid3d::isGridIndexInRange(g, _isize, _jsize, _ksize + 1);
    }

    glm::vec3 evaluateVelocityAtCellCenter(int i, int j, int k);
    float evaluateVelocityMagnitudeAtCellCenter(int i, int j, int k);
    float evaluateVelocityMagnitudeSquaredAtCellCenter(int i, int j, int k);
    float evaluateMaximumVelocityMagnitude();

    glm::vec3 evaluateVelocityAtFaceCenterU(int i, int j, int k);
    glm::vec3 evaluateVelocityAtFaceCenterV(int i, int j, int k);
    glm::vec3 evaluateVelocityAtFaceCenterW(int i, int j, int k);

    glm::vec3 evaluateVelocityAtPosition(double x, double y, double z);
    glm::vec3 evaluateVelocityAtPosition(glm::vec3 pos);
    glm::vec3 evaluateVelocityAtPositionLinear(double x, double y, double z);
    glm::vec3 evaluateVelocityAtPositionLinear(glm::vec3 pos);
    glm::vec3 evaluateChangeInVelocityAtPosition(glm::vec3 pos, MACVelocityField &savedField);

    glm::vec3 velocityIndexToPositionU(int i, int j, int k);
    glm::vec3 velocityIndexToPositionV(int i, int j, int k);
    glm::vec3 velocityIndexToPositionW(int i, int j, int k);

private:
    void _initializeVelocityGrids();

    float _default_out_of_range_value = 0.0f;

    inline double _randomFloat(double min, double max) {
         return min + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max - min))); 
    }

    double _cubicInterpolate(double p[4], double x);
    double _bicubicInterpolate(double p[4][4], double x, double y);
    double _tricubicInterpolate(double p[4][4][4], double x, double y, double z);
    double _trilinearInterpolate(double p[8], double x, double y, double z);

    double _interpolateU(double x, double y, double z);
    double _interpolateV(double x, double y, double z);
    double _interpolateW(double x, double y, double z);
    double _interpolateLinearU(double x, double y, double z);
    double _interpolateLinearV(double x, double y, double z);
    double _interpolateLinearW(double x, double y, double z);
    double _interpolateDeltaVelocityU(double x, double y, double z, MACVelocityField &savedField);
    double _interpolateDeltaVelocityV(double x, double y, double z, MACVelocityField &savedField);
    double _interpolateDeltaVelocityW(double x, double y, double z, MACVelocityField &savedField);

    double _dx = 0.1;
    int _isize = 10;
    int _jsize = 10;
    int _ksize = 10;

    Array3d<float> _u;
    Array3d<float> _v;
    Array3d<float> _w;
};

