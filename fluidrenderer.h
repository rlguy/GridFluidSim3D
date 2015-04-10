#pragma once

#include <SDL.h>
#include <gl\glew.h>
#include <SDL_opengl.h>
#include <gl\glu.h>
#include <stdio.h>
#include <iostream>
#include <string>

#include "fluidsimulation.h"
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"

class FluidRenderer
{
public:
    FluidRenderer();
    FluidRenderer(FluidSimulation &sim);
    ~FluidRenderer();

    void update(float dt);
    void draw();

    void setScale(double s) { scale = s; }
    void setDrawOffset(double x, double y, double z) { tx = x; ty = y; tz = z; }

    void drawGrid();
    void drawGridBoundingBox();
    void drawSolidCells();
    void drawFluidCells();
    void drawAirCells();

private:
    int M_AIR = 0;
    int M_FLUID = 1;
    int M_SOLID = 2;

    void _drawWireframeCube(glm::vec3 p, double size);
    void _drawFluidMaterialType(int mType);

    void _setTransforms();
    void _unsetTransforms();

    FluidSimulation fluidsim;

    double scale = 1.0;
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;

};

