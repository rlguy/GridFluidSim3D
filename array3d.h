#pragma once

#include <vector>
#include <assert.h>

struct GridIndex {
    int i, j, k;

    GridIndex() : i(0), j(0), k(0) {}
    GridIndex(int ii, int jj, int kk) : i(ii), j(jj), k(kk) {}
};

template <class T>
class Array3d
{
public:
    Array3d() : width(0), height(0), depth(0)
    {
        _initializeGrid();
    }

    Array3d(int i, int j, int k) : width(i), height(j), depth(k)
    {
        _initializeGrid();
    }

    Array3d(int i, int j, int k, T fillValue) : width(i), height(j), depth(k)
    {
        _initializeGrid();
        fill(fillValue);
    }

    Array3d(Array3d &obj)
    {
        width = obj.width;
        height = obj.height;
        depth = obj.depth;

        _initializeGrid();

        for (int k = 0; k < depth; k++) {
            for (int j = 0; j < height; j++) {
                for (int i = 0; i < width; i++) {
                    set(i, j, k, obj(i, j, k));
                }
            }
        }

        if (obj.isOutOfRangeValueSet()) {
            _outOfRangeValue = obj.getOutOfRangeValue();
            _isOutOfRangeValueSet = true;
        }
    }

    Array3d operator=(Array3d & rhs)
    {
        delete[] _grid;

        width = rhs.width;
        height = rhs.height;
        depth = rhs.depth;

        _initializeGrid();

        for (int k = 0; k < depth; k++) {
            for (int j = 0; j < height; j++) {
                for (int i = 0; i < width; i++) {
                    set(i, j, k, rhs(i, j, k));
                }
            }
        }

        if (rhs.isOutOfRangeValueSet()) {
            _outOfRangeValue = rhs.getOutOfRangeValue();
            _isOutOfRangeValueSet = true;
        }

        return *this;
    }

    ~Array3d()
    {
        delete[] _grid;
    }

    void fill(T value) {
        for (int idx = 0; idx < width*height*depth; idx++) {
            _grid[idx] = value;
        }
    }

    T& operator()(int i, int j, int k)
    {
        bool isInRange = _isIndexInRange(i, j, k);
        if (!isInRange && _isOutOfRangeValueSet) {
            return _outOfRangeValue;
        }
        assert(isInRange);

        return _grid[_getFlatIndex(i, j, k)];
    }

    T& operator()(GridIndex g)
    {
        bool isInRange = _isIndexInRange(g.i, g.j, g.k);
        if (!isInRange && _isOutOfRangeValueSet) {
            return _outOfRangeValue;
        }
        assert(isInRange);

        return _grid[_getFlatIndex(g)];;
    }

    void set(int i, int j, int k, T value) {
        assert(_isIndexInRange(i, j, k));
        _grid[_getFlatIndex(i, j, k)] = value;
    }

    void set(GridIndex g, T value) {
        assert(_isIndexInRange(g.i, g.j, g.k));
        _grid[_getFlatIndex(g)] = value;
    }

    void set(std::vector<GridIndex> cells, T value) {
        for (int i = 0; i < (int)cells.size(); i++) {
            set(cells[i], value);
        }
    }

    void add(int i, int j, int k, T value) {
        assert(_isIndexInRange(i, j, k));
        _grid[_getFlatIndex(i, j, k)] += value;
    }

    void add(GridIndex g, T value) {
        assert(_isIndexInRange(g.i, g.j, g.k));
        _grid[_getFlatIndex(g)] += value;
    }

    T *getPointer(int i, int j, int k) {
        bool isInRange = _isIndexInRange(i, j, k);
        if (!isInRange && _isOutOfRangeValueSet) {
            return &_outOfRangeValue;
        }

        assert(isInRange);
        return &_grid[_getFlatIndex(i, j, k)];
    }

    T *getPointer(GridIndex g) {
        bool isInRange = _isIndexInRange(g.i, g.j, g.k);
        if (!isInRange && _isOutOfRangeValueSet) {
            return &_outOfRangeValue;
        }

        assert(isInRange);
        return &_grid[_getFlatIndex(g)];
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
        return i >= 0 && j >= 0 && k >= 0 || i < width || j < height || k < depth;
    }

    inline bool isIndexInRange(GridIndex g) {
        return g.i >= 0 && g.j >= 0 && g.k >= 0 || g.i < width || g.j < height || g.k < depth;
    }

    int width = 0;
    int height = 0;
    int depth = 0;

private:
    void _initializeGrid() {
        _grid = new T[width*height*depth];
    }

    inline bool _isIndexInRange(int i, int j, int k) {
        return i >= 0 && j >= 0 && k >= 0 && i < width && j < height && k < depth;
    }

    inline unsigned int _getFlatIndex(int i, int j, int k) {
        return (unsigned int)i + (unsigned int)width *
               ((unsigned int)j + (unsigned int)height * (unsigned int)k);
    }

    inline unsigned int _getFlatIndex(GridIndex g) {
        return (unsigned int)g.i + (unsigned int)width *
               ((unsigned int)g.j + (unsigned int)height * (unsigned int)g.k);
    }

    T *_grid;

    bool _isOutOfRangeValueSet = false;
    T _outOfRangeValue;
};

