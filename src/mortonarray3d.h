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
#ifndef MORTONARRAY3D_H
#define MORTONARRAY3D_H

#include <vector>

#include "grid3d.h"
#include "gridindexvector.h"
#include "array3d.h"
#include "fluidsimassert.h"

template <class T>
class MortonArray3d
{
public:
    MortonArray3d() {
        _initializeGrid();
    }

    MortonArray3d(int i, int j, int k) : width(i), height(j), depth(k), 
                                         _numElements(i*j*k) {
        _initializeGrid();
    }

    MortonArray3d(int i, int j, int k, T fillValue) : width(i), height(j), depth(k), 
                                                      _numElements(i*j*k) {
        _initializeGrid();
        fill(fillValue);
    }

    MortonArray3d(const MortonArray3d &obj) {
        width = obj.width;
        height = obj.height;
        depth = obj.depth;
        _numElements = obj._numElements;

        _initializeGrid();

        for (int i = 0; i < _gridSize; i++) {
            _grid[i] = obj._grid[i];
        }

        if (obj._isOutOfRangeValueSet) {
            _outOfRangeValue = obj._outOfRangeValue;
            _isOutOfRangeValueSet = true;
        }
    }

    MortonArray3d operator=(const MortonArray3d &rhs) {
        delete[] _grid;

        width = rhs.width;
        height = rhs.height;
        depth = rhs.depth;
        _numElements = rhs._numElements;

        _initializeGrid();

        for (int i = 0; i < _gridSize; i++) {
            _grid[i] = rhs._grid[i];
        }

        if (rhs._isOutOfRangeValueSet) {
            _outOfRangeValue = rhs._outOfRangeValue;
            _isOutOfRangeValueSet = true;
        }

        return *this;
    }

    ~MortonArray3d() {
        delete[] _grid;
    }

    void fill(T value) {
        for (int i = 0; i < _gridSize; i++) {
            _grid[i] = value;
        }
    }

    T operator()(int i, int j, int k) {
        bool isInRange = _isIndexInRange(i, j, k);
        if (!isInRange && _isOutOfRangeValueSet) {
            return _outOfRangeValue;
        }
        FLUIDSIM_ASSERT(isInRange);

        return _grid[_getMortonIndex(i, j, k)];
    }

    T operator()(GridIndex g) {
        bool isInRange = _isIndexInRange(g.i, g.j, g.k);
        if (!isInRange && _isOutOfRangeValueSet) {
            return _outOfRangeValue;
        }
        FLUIDSIM_ASSERT(isInRange);

        return _grid[_getMortonIndex(g)];;
    }

    T get(int i, int j, int k) {
        bool isInRange = _isIndexInRange(i, j, k);
        if (!isInRange && _isOutOfRangeValueSet) {
            return _outOfRangeValue;
        }
        FLUIDSIM_ASSERT(isInRange);

        return _grid[_getMortonIndex(i, j, k)];
    }

    T get(GridIndex g) {
        bool isInRange = _isIndexInRange(g.i, g.j, g.k);
        if (!isInRange && _isOutOfRangeValueSet) {
            return _outOfRangeValue;
        }
        FLUIDSIM_ASSERT(isInRange);

        return _grid[_getMortonIndex(g)];;
    }

    void set(int i, int j, int k, T value) {
        FLUIDSIM_ASSERT(_isIndexInRange(i, j, k));
        _grid[_getMortonIndex(i, j, k)] = value;
    }

    void set(GridIndex g, T value) {
        FLUIDSIM_ASSERT(_isIndexInRange(g.i, g.j, g.k));
        _grid[_getMortonIndex(g)] = value;
    }

    void set(std::vector<GridIndex> &cells, T value) {
        for (unsigned int i = 0; i < cells.size(); i++) {
            set(cells[i], value);
        }
    }

    void add(int i, int j, int k, T value) {
        FLUIDSIM_ASSERT(_isIndexInRange(i, j, k));
        _grid[_getMortonIndex(i, j, k)] += value;
    }

    void add(GridIndex g, T value) {
        FLUIDSIM_ASSERT(_isIndexInRange(g.i, g.j, g.k));
        _grid[_getMortonIndex(g)] += value;
    }

    T *getPointer(int i, int j, int k) {
        bool isInRange = _isIndexInRange(i, j, k);
        if (!isInRange && _isOutOfRangeValueSet) {
            return &_outOfRangeValue;
        }

        FLUIDSIM_ASSERT(isInRange);
        return &_grid[_getMortonIndex(i, j, k)];
    }

    T *getPointer(GridIndex g) {
        bool isInRange = _isIndexInRange(g.i, g.j, g.k);
        if (!isInRange && _isOutOfRangeValueSet) {
            return &_outOfRangeValue;
        }

        FLUIDSIM_ASSERT(isInRange);
        return &_grid[_getMortonIndex(g)];
    }

    T *getRawArray() {
        return _grid;
    }

    int getNumElements() {
        return _numElements;
    }

    void setOutOfRangeValue() {
        _isOutOfRangeValueSet = false;
    }
    void setOutOfRangeValue(T val) {
        _outOfRangeValue = val;
        _isOutOfRangeValueSet = true;
    }

    bool isOutOfRangeValueSet() {
        return _isOutOfRangeValueSet;
    }
    T getOutOfRangeValue() {
        return _outOfRangeValue;
    }

    inline bool isIndexInRange(int i, int j, int k) {
        return i >= 0 && j >= 0 && k >= 0 && i < width && j < height && k < depth;
    }

    inline bool isIndexInRange(GridIndex g) {
        return g.i >= 0 && g.j >= 0 && g.k >= 0 && g.i < width && g.j < height && g.k < depth;
    }

    int width = 0;
    int height = 0;
    int depth = 0;

private:
    void _initializeGrid() {
        _bwidth = ceil((double)width / (double)_blockWidth);
        _bheight = ceil((double)height / (double)_blockWidth);
        _bdepth = ceil((double)depth / (double)_blockWidth);
        _gridSize = _bwidth*_bheight*_bdepth*_blockSize;

        _grid = new T[_gridSize];
    }

    inline bool _isIndexInRange(int i, int j, int k) {
        return i >= 0 && j >= 0 && k >= 0 && i < width && j < height && k < depth;
    }

    const int _morton16x[16] = {
        0x000, 0x001, 0x008, 0x009,
        0x040, 0x041, 0x048, 0x049,
        0x200, 0x201, 0x208, 0x209,
        0x240, 0x241, 0x248, 0x249
    };

    const int _morton16y[16] = {
        0x000, 0x002, 0x010, 0x012,
        0x080, 0x082, 0x090, 0x092,
        0x400, 0x402, 0x410, 0x412,
        0x480, 0x482, 0x490, 0x492
    };

    const int _morton16z[16] = {
        0x000, 0x004, 0x020, 0x024,
        0x100, 0x104, 0x120, 0x124,
        0x800, 0x804, 0x820, 0x824,
        0x900, 0x904, 0x920, 0x924
    };

    inline int _mortonEncode(int i, int j, int k) {
        return _morton16x[i] | _morton16y[j] | _morton16z[k];
    }

    inline GridIndex _mortonDecode(int m) {
        int i = m & 0x249;
        int j = m & 0x492;
        int k = m & 0x924;

        return GridIndex(i, j, k);
    }

    inline unsigned int _getMortonIndex(int i, int j, int k) {
        unsigned int bi = i >> _p2;                 // i / (2^_p2)
        unsigned int bj = j >> _p2;
        unsigned int bk = k >> _p2;
        unsigned int flatidx = Grid3d::getFlatIndex(bi, bj, bk, _bwidth, _bheight);
        unsigned int blockOffset = flatidx*_blockSize;

        unsigned int mi = i & (_blockWidth - 1);    // i % (2^p2) 
        unsigned int mj = j & (_blockWidth - 1);
        unsigned int mk = k & (_blockWidth - 1);
        unsigned int mortonOffset = _mortonEncode(mi, mj, mk);

        return blockOffset + mortonOffset;
    }

    inline unsigned int _getMortonIndex(GridIndex g) {
        return _getMortonIndex(g.i, g.j, g.k);
    }

    unsigned int _p2 = 4;
    unsigned int _blockWidth = 1 << _p2;    // 2^_p2
    unsigned int _blockSize = _blockWidth * _blockWidth * _blockWidth;

    int _bwidth = 0;
    int _bheight = 0;
    int _bdepth = 0;
    int _gridSize = 0;

    T *_grid;

    bool _isOutOfRangeValueSet = false;
    T _outOfRangeValue;
    int _numElements = 0;
};

#endif
