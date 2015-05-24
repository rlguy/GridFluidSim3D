#pragma once
#include "surfacefield.h"


class LevelSetField : public SurfaceField
{
public:
    LevelSetField();
    LevelSetField(int width, int height, int depth, double dx);
    ~LevelSetField();

    virtual void clear();
    virtual double getFieldValue(glm::vec3 p);

    void setSignedDistanceField(Array3d<double> distField);
    bool isCellInside(GridIndex g) { return _distanceField(g) > 0.0; };
    double getFieldValueAtCellCenter(GridIndex g) { return _distanceField(g); };

private:

    double _fasttricubicInterpolate(double p[4][4][4], double x, double y, double z);

    GridIndex _positionToGridIndex(glm::vec3 p);
    glm::vec3 _gridIndexToPosition(GridIndex g);
    glm::vec3 _gridIndexToCellCenter(GridIndex g);

    inline bool _isCellIndexInRange(GridIndex g) {
        return g.i >= 0 && g.j >= 0 && g.k >= 0 && g.i < i_width && g.j < j_height && g.k < k_depth;
    }


    Array3d<double> _distanceField;
};

