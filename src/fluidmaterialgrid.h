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
#ifndef FLUIDMATERIALGRID_H
#define FLUIDMATERIALGRID_H

#include <vector>

#include "subdividedarray3d.h"
#include "grid3d.h"
#include "gridindexvector.h"

enum class Material : char { 
    air   = 0x00, 
    fluid = 0x01, 
    solid = 0x02
};

class FluidMaterialGrid {

public:

    FluidMaterialGrid();
    FluidMaterialGrid(int i, int j, int k);
    ~FluidMaterialGrid();

    Material operator()(int i, int j, int k);
    Material operator()(GridIndex g);

    void fill(Material m);

    void set(int i, int j, int k, Material m);
    void set(GridIndex g, Material m);
    void set(GridIndexVector &cells, Material m);
    void setAir(int i, int j, int k);
    void setAir(GridIndex g);
    void setAir(GridIndexVector &cells);
    void setFluid(int i, int j, int k);
    void setFluid(GridIndex g);
    void setFluid(GridIndexVector &cells);
    void setSolid(int i, int j, int k);
    void setSolid(GridIndex g);
    void setSolid(GridIndexVector &cells);

    bool isCellAir(int i, int j, int k);
    bool isCellAir(GridIndex g);
    bool isCellFluid(int i, int j, int k);
    bool isCellFluid(GridIndex g);
    bool isCellSolid(int i, int j, int k);
    bool isCellSolid(GridIndex g);

    bool isFaceBorderingMaterialU(int i, int j, int k, Material m);
    bool isFaceBorderingMaterialU(GridIndex g, Material m);
    bool isFaceBorderingMaterialV(int i, int j, int k, Material m);
    bool isFaceBorderingMaterialV(GridIndex g, Material m);
    bool isFaceBorderingMaterialW(int i, int j, int k, Material m);
    bool isFaceBorderingMaterialW(GridIndex g, Material m);
    bool isFaceBorderingAirU(int i, int j, int k);
    bool isFaceBorderingAirU(GridIndex g);
    bool isFaceBorderingFluidU(int i, int j, int k);
    bool isFaceBorderingFluidU(GridIndex g);
    bool isFaceBorderingSolidU(int i, int j, int k);
    bool isFaceBorderingSolidU(GridIndex g);
    bool isFaceBorderingAirV(int i, int j, int k);
    bool isFaceBorderingAirV(GridIndex g);
    bool isFaceBorderingFluidV(int i, int j, int k);
    bool isFaceBorderingFluidV(GridIndex g);
    bool isFaceBorderingSolidV(int i, int j, int k);
    bool isFaceBorderingSolidV(GridIndex g);
    bool isFaceBorderingAirW(int i, int j, int k);
    bool isFaceBorderingAirW(GridIndex g);
    bool isFaceBorderingFluidW(int i, int j, int k);
    bool isFaceBorderingFluidW(GridIndex g);
    bool isFaceBorderingSolidW(int i, int j, int k);
    bool isFaceBorderingSolidW(GridIndex g);

    bool isCellNeighbouringMaterial(int i, int j, int k, Material m);
    bool isCellNeighbouringMaterial(GridIndex g, Material m);
    bool isCellNeighbouringAir(int i, int j, int k);
    bool isCellNeighbouringAir(GridIndex g);
    bool isCellNeighbouringFluid(int i, int j, int k);
    bool isCellNeighbouringFluid(GridIndex g);
    bool isCellNeighbouringSolid(int i, int j, int k);
    bool isCellNeighbouringSolid(GridIndex g);

    void setSubdivisionLevel(int n);
    int getSubdivisionLevel();

    int width = 0;
    int height = 0;
    int depth = 0;

private: 

    SubdividedArray3d<Material> _grid;


};

#endif
