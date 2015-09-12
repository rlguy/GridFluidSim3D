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

#include <SDL.h>
#include <gl\glew.h>
#include <SDL_opengl.h>
#include <gl\glu.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>

#include "fluidsimulation.h"
#include "polygonizer3d.h"
#include "surfacefield.h"
#include "array3d.h"
#include "grid3d.h"
#include "trianglemesh.h"
#include "game/camera/camera3d.h"
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"

#include "aabb.h"

class FluidRenderer
{
public:
    FluidRenderer();
    FluidRenderer(FluidSimulation *sim);
    ~FluidRenderer();

    void update(float dt);
    void draw();

    void setScale(double s) { _scale = s; }
    void setDrawOffset(double x, double y, double z) { _tx = x; _ty = y; _tz = z; }

    void drawGrid();
    void drawGridBoundingBox();
    void drawSolidCells();
    void drawFluidCells();
    void drawAirCells();
    void drawMarkerParticles();
    void drawBillboardTextures(GLuint tex, double width, Camera3d *cam);
    void drawSurfaceTriangles();
    void drawDensityGrid();
private:
    int M_AIR = 0;
    int M_FLUID = 1;
    int M_SOLID = 2;

    void _drawWireframeCube(glm::vec3 p, double size);
    void _drawFluidMaterialType(int mType);

    void _setTransforms();
    void _unsetTransforms();

    FluidSimulation *_fluidsim;

    double _scale = 1.0;
    double _tx = 0.0;
    double _ty = 0.0;
    double _tz = 0.0;

};

