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
#ifndef SPARSEARRAY3D_H
#define SPARSEARRAY3D_H

#include <vector>
#include <unordered_map>
#include <assert.h>

#include "array3d.h"

template <class T>
class SparseArray3d
{
public:
    SparseArray3d() : width(0), height(0), depth(0) {
    }

    SparseArray3d(int i, int j, int k, T defVal) : width(i), height(j), depth(k),
                                                   _defaultValue(defVal) {
    }

    SparseArray3d(const SparseArray3d &obj) {
        width = obj.width;
        height = obj.height;
        depth = obj.depth;

        _defaultValue = obj._defaultValue;
        _grid = GridHash(obj._grid);

        if (obj._isOutOfRangeValueSet) {
            _outOfRangeValue = obj._outOfRangeValue;
            _isOutOfRangeValueSet = true;
        }
    }

    SparseArray3d operator=(const SparseArray3d &rhs)
    {
        width = rhs.width;
        height = rhs.height;
        depth = rhs.depth;

        _defaultValue = rhs._defaultValue;
        _grid = GridHash(rhs._grid);

        if (rhs._isOutOfRangeValueSet) {
            _outOfRangeValue = rhs._outOfRangeValue;
            _isOutOfRangeValueSet = true;
        }

        return *this;
    }

    ~SparseArray3d() {
    }

    
    void clear() {
        _grid.clear();
        _sparseIndices.clear();
    }
    
    T operator()(int i, int j, int k)
    {
        bool isInRange = _isIndexInRange(i, j, k);
        if (!isInRange && _isOutOfRangeValueSet) {
            return _outOfRangeValue;
        }
        assert(isInRange);

        GridIndex g = GridIndex(i, j, k);
        T value;
        if (_isKeyInGrid(g, &value)) {
            return value;
        }

        return _defaultValue;
    }

    T operator()(GridIndex g)
    {
        bool isInRange = _isIndexInRange(g);
        if (!isInRange && _isOutOfRangeValueSet) {
            return _outOfRangeValue;
        }
        assert(isInRange);

        T value;
        if (_isKeyInGrid(g, &value)) {
            return value;
        }

        return _defaultValue;
    }

    void set(int i, int j, int k, T value) {
        assert(_isIndexInRange(i, j, k));

        GridIndex g = GridIndex(i, j, k);
        GridHashIterator it = _grid.find(g);
        if (it != _grid.end()) {
            it->second = value;
            return;
        }

        std::pair<GridIndex,T> kv(g, value);
        _grid.insert(kv);

        if (_isTrackingSparseIndices) {
            _sparseIndices.push_back(g);
        }
    }

    void set(GridIndex g, T value) {
        assert(_isIndexInRange(g.i, g.j, g.k));

        GridHashIterator it = _grid.find(g);
        if (it != _grid.end()) {
            it->second = value;
            return;
        }

        std::pair<GridIndex,T> kv(g, value);
        _grid.insert(kv);

        if (_isTrackingSparseIndices) {
            _sparseIndices.push_back(g);
        }
    }
    
    void set(std::vector<GridIndex> cells, T value) {
        for (unsigned int i = 0; i < cells.size(); i++) {
            set(cells[i], value);
        }
    }

    void add(int i, int j, int k, T value) {
        assert(_isIndexInRange(i, j, k));

        GridIndex g = GridIndex(i, j, k);
        GridHashIterator it = _grid.find(g);
        if (it != _grid.end()) {
            it->second += value;
            return;
        }

        std::pair<GridIndex,T> kv(g, _defaultValue + value);
        _grid.insert(kv);

        if (_isTrackingSparseIndices) {
            _sparseIndices.push_back(g);
        }
    }

    void add(GridIndex g, T value) {
        assert(_isIndexInRange(g.i, g.j, g.k));

        GridHashIterator it = _grid.find(g);
        if (it != _grid.end()) {
            it->second += value;
            return;
        }

        std::pair<GridIndex,T> kv(g, _defaultValue + value);
        _grid.insert(kv);

        if (_isTrackingSparseIndices) {
            _sparseIndices.push_back(g);
        }
    }

    T *getPointer(int i, int j, int k) {
        bool isInRange = _isIndexInRange(i, j, k);
        if (!isInRange && _isOutOfRangeValueSet) {
            return &_outOfRangeValue;
        }
        assert(isInRange);

        GridIndex g(i, j, k);
        if (this->_IsKeyInGrid(g)) {
            return &_grid[g];
        }

        std::pair<GridIndex,T> kv(g, _defaultValue);
        _grid.insert(kv);

        if (_isTrackingSparseIndices) {
            _sparseIndices.push_back(g);
        }

        return &_grid[g];
    }

    T *getPointer(GridIndex g) {
        bool isInRange = _isIndexInRange(g.i, g.j, g.k);
        if (!isInRange && _isOutOfRangeValueSet) {
            return &_outOfRangeValue;
        }
        assert(isInRange);

        if (this->_IsKeyInGrid(g)) {
            return &_grid[g];
        }

        std::pair<GridIndex,T> kv(g, _defaultValue);
        _grid.insert(kv);

        if (_isTrackingSparseIndices) {
            _sparseIndices.push_back(g);
        }

        return &_grid[g];
    }
    

    void reserve(int n) {
        _grid.reserve(n);
    }

    std::vector<GridIndex> getSparseIndices() {
        return _sparseIndices;
    }

    int getNumElements() {
        return _grid.size();
    }

    T getDefaultValue() {
        return _defaultValue;
    }

    void setDefaultValue(T val) {
        _defaultValue = val;
    }

    std::unordered_map<GridIndex, T, GridIndexHasher> getUnorderedMap() {
        return _grid;
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

    void enableSparseIndexTracking() {
        _isTrackingSparseIndices = true;
    }

    void disableSparseIndexTracking() {
        _isTrackingSparseIndices = false;
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

    inline bool _isIndexInRange(int i, int j, int k) {
        return i >= 0 && j >= 0 && k >= 0 && i < width && j < height && k < depth;
    }
    inline bool _isIndexInRange(GridIndex g) {
        return g.i >= 0 && g.j >= 0 && g.k >= 0 && g.i < width && g.j < height && g.k < depth;
    }
    inline bool _isKeyInGrid(GridIndex g) {
        return _grid.find(g) != _grid.end();
    }
    inline bool _isKeyInGrid(GridIndex g, T *value) {
        GridHashConstIterator it = _grid.find(g);
        if (it != _grid.end()) {
            *value = it->second;
            return true;
        }

        return false;
    }

    typedef typename std::unordered_map<GridIndex, T, GridIndexHasher> GridHash;
    typedef typename std::unordered_map<GridIndex, T, GridIndexHasher>::iterator GridHashIterator;
    typedef typename std::unordered_map<GridIndex, T, GridIndexHasher>::const_iterator GridHashConstIterator;

    GridHash _grid;
    std::vector<GridIndex> _sparseIndices;

    bool _isOutOfRangeValueSet = false;
    bool _isTrackingSparseIndices = false;
    T _outOfRangeValue;
    T _defaultValue;
};

#endif
