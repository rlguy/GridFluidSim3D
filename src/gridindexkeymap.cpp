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
#include "gridindexkeymap.h"

GridIndexKeyMap::GridIndexKeyMap() {
}

GridIndexKeyMap::GridIndexKeyMap(int i, int j, int k) : _isize(i), _jsize(j), _ksize(k) {
    _indices = std::vector<int>(i*j*k, _notFoundValue);
}

GridIndexKeyMap::~GridIndexKeyMap() {
}

void GridIndexKeyMap::clear() {
    for (unsigned int i = 0; i < _indices.size(); i++) {
        _indices[i] = _notFoundValue;
    }
}

void GridIndexKeyMap::insert(GridIndex g, int key) {
    insert(g.i, g.j, g.k, key);
}

void GridIndexKeyMap::insert(int i, int j, int k, int key) {
    FLUIDSIM_ASSERT(Grid3d::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize));

    int flatidx = _getFlatIndex(i, j, k);
    _indices[flatidx] = key;
}

int GridIndexKeyMap::find(GridIndex g) {
    return find(g.i, g.j, g.k);
}

int GridIndexKeyMap::find(int i, int j, int k) {
    FLUIDSIM_ASSERT(Grid3d::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize));

    if (_indices.size() == 0) {
        return _notFoundValue;
    }

    int flatidx = _getFlatIndex(i, j, k);
    return _indices[flatidx];
}
