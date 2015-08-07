#pragma once

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

    SparseArray3d(SparseArray3d &obj) {
        width = obj.width;
        height = obj.height;
        depth = obj.depth;

        _defaultValue = obj.getDefaultValue();
        _grid = obj.getUnorderedMap();

        if (obj.isOutOfRangeValueSet()) {
            _outOfRangeValue = obj.getOutOfRangeValue();
            _isOutOfRangeValueSet = true;
        }
    }

    SparseArray3d operator=(SparseArray3d &rhs)
    {
        width = rhs.width;
        height = rhs.height;
        depth = rhs.depth;

        _defaultValue = rhs.getDefaultValue();
        _grid = rhs.getUnorderedMap();

        if (rhs.isOutOfRangeValueSet()) {
            _outOfRangeValue = rhs.getOutOfRangeValue();
            _isOutOfRangeValueSet = true;
        }

        return *this;
    }

    ~SparseArray3d() {
    }

    
    void clear() {
        _grid.clear();
    }
    
    T& operator()(int i, int j, int k)
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

    T& operator()(GridIndex g)
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
        if (_isKeyInGrid(g)) {
            _grid[g] = value;
            return;
        }

        std::pair<GridIndex,T> kv(g, value);
        _grid.insert(kv);
    }

    void set(GridIndex g, T value) {
        assert(_isIndexInRange(g.i, g.j, g.k));

        if (_isKeyInGrid(g)) {
            _grid[g] = value;
            return;
        }

        std::pair<GridIndex,T> kv(g, value);
        _grid.insert(kv);
    }
    
    void set(std::vector<GridIndex> cells, T value) {
        for (int i = 0; i < (int)cells.size(); i++) {
            set(cells[i], value);
        }
    }

    void add(int i, int j, int k, T value) {
        assert(_isIndexInRange(i, j, k));

        GridIndex g = GridIndex(i, j, k);
        if (_isKeyInGrid(g)) {
            _grid[g] += value;
            return;
        }

        std::pair<GridIndex,T> kv(g, _defaultValue + value);
        _grid.insert(kv);
    }

    void add(GridIndex g, T value) {
        assert(_isIndexInRange(g.i, g.j, g.k));

        if (_isKeyInGrid(g)) {
            _grid[g] += value;
            return;
        }

        std::pair<GridIndex,T> kv(g, _defaultValue + value);
        _grid.insert(kv);
    }

    T *getPointer(int i, int j, int k) {
        bool isInRange = _isIndexInRange(i, j, k);
        if (!isInRange && _isOutOfRangeValueSet) {
            return &_outOfRangeValue;
        }
        assert(isInRange);

        GridIndex g(i, j, k);
        if (_IsKeyInGrid(g)) {
            return &_grid[g];
        }

        std::pair<GridIndex,T> kv(g, _defaultValue);
        _grid.insert(kv);

        return &_grid[g];
    }

    T *getPointer(GridIndex g) {
        bool isInRange = _isIndexInRange(g.i, g.j, g.k);
        if (!isInRange && _isOutOfRangeValueSet) {
            return &_outOfRangeValue;
        }
        assert(isInRange);

        if (_IsKeyInGrid(g)) {
            return &_grid[g];
        }

        std::pair<GridIndex,T> kv(g, _defaultValue);
        _grid.insert(kv);

        return &_grid[g];
    }
    

    void reserve(int n) {
        _grid.reserve(n);
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
        _isOutOfrangeValueSet = false;
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
        GridHash::const_iterator it = _grid.find(g);
        if (it != _grid.end()) {
            *value = it->second;
            return true;
        }

        return false;
    }

    typedef std::unordered_map<GridIndex, T, GridIndexHasher> GridHash;

    GridHash _grid;

    bool _isOutOfRangeValueSet = false;
    T _outOfRangeValue;
    T _defaultValue;
};

