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

#include <assert.h>

#include "array3d.h"
#include "grid3d.h"
#include "interpolation.h"
#include "macvelocityfield.h"
#include "vmath.h"

class TurbulenceField
{
public:
    TurbulenceField();
    ~TurbulenceField();

    void calculateTurbulenceField(MACVelocityField *vfield,
                                  std::vector<GridIndex> &fluidCells);
    void destroyTurbulenceField();
    double evaluateTurbulenceAtPosition(vmath::vec3 p);

private:
    
    void _getVelocityGrid(MACVelocityField *macfield, 
                          Array3d<vmath::vec3> &vgrid);
    double _calculateTurbulenceAtGridCell(int i, int j, int k, 
                                          Array3d<vmath::vec3> &vgrid);

    int M_FLUID = 1;

    Array3d<float> _field;

    int _isize, _jsize, _ksize;
    double _dx;
    double _radius;

    GridIndex cellNeighbours[124];
};

