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
#ifndef LEVELSETFIELD_H
#define LEVELSETFIELD_H

#include <limits>

#include "surfacefield.h"
#include "interpolation.h"


class LevelSetField : public SurfaceField
{
public:
    LevelSetField();
    LevelSetField(int width, int height, int depth, double dx);
    ~LevelSetField();

    virtual void clear();
    virtual double getFieldValue(vmath::vec3 p);

    void setSignedDistanceField(Array3d<float> distField);
    bool isCellInside(GridIndex g) { return _distanceField(g) > 0.0; };
    double getFieldValueAtCellCenter(GridIndex g) { return _distanceField(g); };

private:

    double _surfaceThreshold = 0.0;
    Array3d<float> _distanceField;
};

#endif
