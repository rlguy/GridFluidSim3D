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
#ifndef MACVELOCITYFIELD_H
#define MACVELOCITYFIELD_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <limits>
#include <time.h>

#include "fluidmaterialgrid.h"
#include "array3d.h"
#include "grid3d.h"
#include "interpolation.h"
#include "vmath.h" 
#include "fluidsimassert.h"

class MACVelocityField
{
public:
    MACVelocityField();
    MACVelocityField(int isize, int jsize, int ksize, double dx);
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

    Array3d<float>* getArray3dU();
    Array3d<float>* getArray3dV();
    Array3d<float>* getArray3dW();

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

    vmath::vec3 evaluateVelocityAtCellCenter(int i, int j, int k);
    float evaluateVelocityMagnitudeAtCellCenter(int i, int j, int k);
    float evaluateVelocityMagnitudeSquaredAtCellCenter(int i, int j, int k);
    float evaluateMaximumVelocityMagnitude();

    vmath::vec3 evaluateVelocityAtFaceCenterU(int i, int j, int k);
    vmath::vec3 evaluateVelocityAtFaceCenterV(int i, int j, int k);
    vmath::vec3 evaluateVelocityAtFaceCenterW(int i, int j, int k);

    vmath::vec3 evaluateVelocityAtPosition(double x, double y, double z);
    vmath::vec3 evaluateVelocityAtPosition(vmath::vec3 pos);
    vmath::vec3 evaluateVelocityAtPositionLinear(double x, double y, double z);
    vmath::vec3 evaluateVelocityAtPositionLinear(vmath::vec3 pos);

    vmath::vec3 velocityIndexToPositionU(int i, int j, int k);
    vmath::vec3 velocityIndexToPositionV(int i, int j, int k);
    vmath::vec3 velocityIndexToPositionW(int i, int j, int k);

    void extrapolateVelocityField(FluidMaterialGrid &materialGrid, int numLayers);

private:
    void _initializeVelocityGrids();

    float _default_out_of_range_value = 0.0f;

    inline double _randomFloat(double min, double max) {
         return min + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max - min))); 
    }

    double _interpolateU(double x, double y, double z);
    double _interpolateV(double x, double y, double z);
    double _interpolateW(double x, double y, double z);
    double _interpolateLinearU(double x, double y, double z);
    double _interpolateLinearV(double x, double y, double z);
    double _interpolateLinearW(double x, double y, double z);

    void _resetExtrapolatedFluidVelocities(FluidMaterialGrid &matGrid);
    void _updateExtrapolationLayers(FluidMaterialGrid &matGrid, 
                                    Array3d<int> &layerGrid);
    void _updateExtrapolationLayer(int layerIndex, 
                                   FluidMaterialGrid &matGrid,
                                   Array3d<int> &layerGrid);
    void _extrapolateVelocitiesForLayerIndex(int idx, 
                                             FluidMaterialGrid &matGrid,
                                             Array3d<int> &layerGrid);
    void _extrapolateVelocitiesForLayerIndexU(int idx, 
                                              FluidMaterialGrid &matGrid,
                                              Array3d<int> &layerGrid);
    void _extrapolateVelocitiesForLayerIndexV(int idx, 
                                              FluidMaterialGrid &matGrid,
                                              Array3d<int> &layerGrid);
    void _extrapolateVelocitiesForLayerIndexW(int idx, 
                                              FluidMaterialGrid &matGrid,
                                              Array3d<int> &layerGrid);
    double _getExtrapolatedVelocityForFaceU(int i, int j, int k, int layerIdx,
                                            Array3d<int> &layerGrid);
    double _getExtrapolatedVelocityForFaceV(int i, int j, int k, int layerIdx,
                                            Array3d<int> &layerGrid);
    double _getExtrapolatedVelocityForFaceW(int i, int j, int k, int layerIdx,
                                            Array3d<int> &layerGrid);

    inline bool _isFaceBorderingGridValueU(int i, int j, int k, int value, Array3d<int> &grid) {
        if (i == grid.width) { return grid(i - 1, j, k) == value; }
        else if (i > 0) { return grid(i, j, k) == value || grid(i - 1, j, k) == value; }
        else { return grid(i, j, k) == value; }
    }
    inline bool _isFaceBorderingGridValueV(int i, int j, int k, int value, Array3d<int> &grid) {
        if (j == grid.height) { return grid(i, j - 1, k) == value; }
        else if (j > 0) { return grid(i, j, k) == value || grid(i, j - 1, k) == value; }
        else { return grid(i, j, k) == value; }
    }
    inline bool _isFaceBorderingGridValueW(int i, int j, int k, int value, Array3d<int> &grid) {
        if (k == grid.depth) { return grid(i, j, k - 1) == value; }
        else if (k > 0) { return grid(i, j, k) == value || grid(i, j, k - 1) == value; }
        else { return grid(i, j, k) == value; }
    }

    inline bool _isFaceBorderingGridValueU(GridIndex g, int value, Array3d<int> &grid) {
        return _isFaceBorderingGridValueU(g.i, g.j, g.k, value, grid);
    }
    inline bool _isFaceBorderingGridValueV(GridIndex g, int value, Array3d<int> &grid) {
        return _isFaceBorderingGridValueV(g.i, g.j, g.k, value, grid);
    }
    inline bool _isFaceBorderingGridValueW(GridIndex g, int value, Array3d<int> &grid) {
        return _isFaceBorderingGridValueW(g.i, g.j, g.k, value, grid);
    }

    inline bool _isFaceBorderingLayerIndexU(int i, int j, int k, int layer, Array3d<int> &layerGrid) {
        return _isFaceBorderingGridValueU(i, j, k, layer, layerGrid);
    }
    inline bool _isFaceBorderingLayerIndexV(int i, int j, int k, int layer, Array3d<int> &layerGrid) {
        return _isFaceBorderingGridValueV(i, j, k, layer, layerGrid);
    }
    inline bool _isFaceBorderingLayerIndexW(int i, int j, int k, int layer, Array3d<int> &layerGrid) {
        return _isFaceBorderingGridValueW(i, j, k, layer, layerGrid);
    }
    inline bool _isFaceBorderingLayerIndexU(GridIndex g, int layer, Array3d<int> &layerGrid) {
        return _isFaceBorderingGridValueU(g, layer, layerGrid);
    }
    inline bool _isFaceBorderingLayerIndexV(GridIndex g, int layer, Array3d<int> &layerGrid) {
        return _isFaceBorderingGridValueV(g, layer, layerGrid);
    }
    inline bool _isFaceBorderingLayerIndexW(GridIndex g, int layer, Array3d<int> &layerGrid) {
        return _isFaceBorderingGridValueW(g, layer, layerGrid);
    }

    inline bool _isFaceVelocityExtrapolatedU(int i, int j, int k, Array3d<int> &layerGrid) {
        if (i == _isize) {  return layerGrid(i - 1, j, k) >= 1.0; }
        else if (i > 0) { return layerGrid(i, j, k) >= 1.0 || layerGrid(i - 1, j, k) >= 1.0; }
        else { return layerGrid(i, j, k) >= 1.0; }
    }

    inline bool _isFaceVelocityExtrapolatedV(int i, int j, int k, Array3d<int> &layerGrid) {
        if (j == _jsize) { return layerGrid(i, j - 1, k) >= 1.0; }
        else if (j > 0) { return layerGrid(i, j, k) >= 1.0 || layerGrid(i, j - 1, k) >= 1.0; }
        else { return layerGrid(i, j, k) >= 1.0; }
    }

    inline bool _isFaceVelocityExtrapolatedW(int i, int j, int k, Array3d<int> &layerGrid) {
        if (k == _ksize) { return layerGrid(i, j, k - 1) >= 1.0; }
        else if (k > 0) { return layerGrid(i, j, k) >= 1.0 || layerGrid(i, j, k - 1) >= 1.0; }
        else { return layerGrid(i, j, k) >= 1.0; }
    }

    int _isize = 10;
    int _jsize = 10;
    int _ksize = 10;
    double _dx = 0.1;

    Array3d<float> _u;
    Array3d<float> _v;
    Array3d<float> _w;

    int _numExtrapolationLayers = 0;
};

#endif
