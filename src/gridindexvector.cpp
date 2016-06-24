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
#include "gridindexvector.h"

GridIndexVector::GridIndexVector() {
}

GridIndexVector::GridIndexVector(int i, int j, int k) : 
                                    width(i), height(j), depth(k) {
}

GridIndexVector::~GridIndexVector() {
}


GridIndex GridIndexVector::operator[](int i) {
    FLUIDSIM_ASSERT(i >= 0 && i < (int)_indices.size());
    return _getUnflattenedIndex(_indices[i]);
}

void GridIndexVector::insert(std::vector<GridIndex> &indices) {
    reserve(_indices.size() + indices.size());
    for (unsigned int i = 0; i < indices.size(); i++) {
        push_back(indices[i]);
    }
}

void GridIndexVector::insert(GridIndexVector &indices) {
    FLUIDSIM_ASSERT(width == indices.width && height == indices.height && depth == indices.depth);

    reserve(_indices.size() + indices.size());
    int maxidx = width*height*depth - 1;
    for (unsigned int i = 0; i < indices.size(); i++) {
        int flatidx = indices.getFlatIndex(i);
        FLUIDSIM_ASSERT(flatidx >= 0 && flatidx <= maxidx);
        _indices.push_back(flatidx);
    }
}

std::vector<GridIndex> GridIndexVector::getVector() {
    std::vector<GridIndex> vector;
    vector.reserve(size());
    
    for (unsigned int i = 0; i < size(); i++) {
        vector.push_back((*this)[i]);
    }

    return vector;
}

void GridIndexVector::getVector(std::vector<GridIndex> &vector) {
    vector.reserve(size());
    
    for (unsigned int i = 0; i < size(); i++) {
        vector.push_back((*this)[i]);
    }
}