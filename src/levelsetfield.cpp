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
#include "levelsetfield.h"


LevelSetField::LevelSetField(int i, int j, int k, double dx) :
                             SurfaceField(i, j, k, dx),
                             _distanceField(Array3d<float>(i, j, k, 1.0f))
{
    setSurfaceThreshold(_surfaceThreshold);
}

LevelSetField::LevelSetField()
{
}

LevelSetField::~LevelSetField()
{
}

void LevelSetField::clear() {
    _distanceField.fill(0.0);
}

double LevelSetField::getFieldValue(vmath::vec3 p) {
    GridIndex g = Grid3d::positionToGridIndex(p, dx);
    vmath::vec3 gp = Grid3d::GridIndexToPosition(g, dx);

    double refx = gp.x - 0.5*dx;
    double refy = gp.y - 0.5*dx;
    double refz = gp.z - 0.5*dx;
    int refi = g.i - 1; 
    int refj = g.j - 1; 
    int refk = g.k - 1;

    vmath::vec3 offset = p - gp;
    if (offset.x >= 0.5*dx) {
        refi++;
        refx += dx;
    }
    if (offset.y >= 0.5*dx) {
        refj++;
        refy += dx;
    }
    if (offset.z >= 0.5*dx) {
        refk++;
        refz += dx;
    }

    double invdx = 1 / dx;
    double ix = (p.x - refx)*invdx;
    double iy = (p.y - refy)*invdx;
    double iz = (p.z - refz)*invdx;

    g = GridIndex(refi - 1, refj - 1, refk - 1);

    double min = std::numeric_limits<double>::infinity();
    double max = -std::numeric_limits<double>::infinity();
    double points[4][4][4];
    GridIndex c;
    for (int pk = 0; pk < 4; pk++) {
        for (int pj = 0; pj < 4; pj++) {
            for (int pi = 0; pi < 4; pi++) {
                c = GridIndex(pi + g.i, pj + g.j, pk + g.k);
                if (Grid3d::isGridIndexInRange(c, i_width, j_height, k_depth)) {
                    points[pi][pj][pk] = _distanceField(c);

                    if (points[pi][pj][pk] < min) {
                        min = points[pi][pj][pk];
                    } else if (points[pi][pj][pk] > max) {
                        max = points[pi][pj][pk];
                    }
                } else {
                    points[pi][pj][pk] = 0.0;
                }
            }
        }
    }

    double val = Interpolation::tricubicInterpolate(points, ix, iy, iz);
    if (val < min) {
        val = min;
    } else if (val > max) {
        val = max;
    }

    double eps = 10e-6;
    if (isMaterialGridSet && val > surfaceThreshold && _isPointNearSolid(p)) {
        val = surfaceThreshold - eps;
    }

    return val;
}

void LevelSetField::setSignedDistanceField(Array3d<float> distField) {
    _distanceField = distField;
}
