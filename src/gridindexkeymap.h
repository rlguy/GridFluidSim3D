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
#ifndef GRIDINDEXKEYMAP_H
#define GRIDINDEXKEYMAP_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <limits>
#include <algorithm>

#include "grid3d.h"
#include "array3d.h"
#include "fluidsimassert.h"

class GridIndexKeyMap
{
public:
    GridIndexKeyMap();
    GridIndexKeyMap(int i, int j, int k);
    ~GridIndexKeyMap();

    void clear();
    void insert(GridIndex g, int key);
    void insert(int i, int j, int k, int key);
    int find(GridIndex g);
    int find(int i, int j, int k);

private:

    inline unsigned int _getFlatIndex(int i, int j, int k) {
        return (unsigned int)i + (unsigned int)_isize *
               ((unsigned int)j + (unsigned int)_jsize * (unsigned int)k);
    }

    inline unsigned int _getFlatIndex(GridIndex g) {
        return (unsigned int)g.i + (unsigned int)_isize *
               ((unsigned int)g.j + (unsigned int)_jsize * (unsigned int)g.k);
    }

    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;

    std::vector<int> _indices;
    int _notFoundValue = -1;

};

#endif