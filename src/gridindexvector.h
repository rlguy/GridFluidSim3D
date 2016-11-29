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
#ifndef GRIDINDEXVECTOR_H
#define GRIDINDEXVECTOR_H

#include <vector>
#include <stdio.h>
#include <iostream>

#include "array3d.h"
#include "fluidsimassert.h"

class GridIndexVector
{
public:
    GridIndexVector();
    GridIndexVector(int i, int j, int k);
    ~GridIndexVector();

    inline size_t size() {
        return _indices.size();
    }

    inline bool empty() {
        return _indices.empty();
    }

    inline void reserve(size_t n) {
        _indices.reserve(n);
    }

    inline void shrink_to_fit() {
        _indices.shrink_to_fit();
    }

    GridIndex operator[](int i);

    inline GridIndex at(int i) {
        FLUIDSIM_ASSERT(i >= 0 && i < (int)_indices.size());
        return _getUnflattenedIndex(_indices[i]);
    }

    inline GridIndex get(int i) {
        FLUIDSIM_ASSERT(i >= 0 && i < (int)_indices.size());
        return _getUnflattenedIndex(_indices[i]);
    }

    inline unsigned int getFlatIndex(int i) {
        FLUIDSIM_ASSERT(i >= 0 && i < (int)_indices.size());
        return _indices[i];
    }

    inline GridIndex front() {
        FLUIDSIM_ASSERT(!_indices.empty());
        return _getUnflattenedIndex(_indices.front());
    }

    inline GridIndex back() {
        FLUIDSIM_ASSERT(!_indices.empty());
        return _getUnflattenedIndex(_indices.back());
    }

    inline void push_back(GridIndex g) {
        FLUIDSIM_ASSERT(g.i >= 0 && g.j >= 0 && g.k >= 0 && g.i < width && g.j < height && g.k < depth);
        _indices.push_back(_getFlatIndex(g));
    }

    inline void push_back(int i, int j, int k) {
        FLUIDSIM_ASSERT(i >= 0 && j >= 0 && k >= 0 && i < width && j < height && k < depth);
        _indices.push_back(_getFlatIndex(i, j, k));
    }

    void insert(std::vector<GridIndex> &indices);
    void insert(GridIndexVector &indices);

    inline void pop_back() {
        FLUIDSIM_ASSERT(!_indices.empty());
        _indices.pop_back();
    }

    inline void clear() {
        _indices.clear();
    }

    std::vector<GridIndex> getVector();
    void getVector(std::vector<GridIndex> &vector);

    int width = 0;
    int height = 0;
    int depth = 0;

private:

    inline unsigned int _getFlatIndex(int i, int j, int k) {
        return (unsigned int)i + (unsigned int)width *
               ((unsigned int)j + (unsigned int)height * (unsigned int)k);
    }

    inline unsigned int _getFlatIndex(GridIndex g) {
        return (unsigned int)g.i + (unsigned int)width *
               ((unsigned int)g.j + (unsigned int)height * (unsigned int)g.k);
    }

    inline GridIndex _getUnflattenedIndex(unsigned int flatidx) {
        int i = flatidx % width;
        int j = (flatidx / width) % height;
        int k = flatidx / (width * height);

        return GridIndex(i, j, k);
    }

    std::vector<int> _indices;

};

#endif