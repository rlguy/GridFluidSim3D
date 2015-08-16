#pragma once
#include "surfacefield.h"
#include "stopwatch.h"


class LevelSetField : public SurfaceField
{
public:
    LevelSetField();
    LevelSetField(int width, int height, int depth, double dx);
    ~LevelSetField();

    virtual void clear();
    virtual double getFieldValue(glm::vec3 p);

    void setSignedDistanceField(Array3d<float> distField);
    bool isCellInside(GridIndex g) { return _distanceField(g) > 0.0; };
    double getFieldValueAtCellCenter(GridIndex g) { return _distanceField(g); };

private:

    double _fasttricubicInterpolate(double p[4][4][4], double x, double y, double z);

    double _surfaceThreshold = 0.0;
    Array3d<float> _distanceField;
};

