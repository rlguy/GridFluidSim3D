#pragma once

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
    Array3d()
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
    }

    Array3d operator=(Array3d & rhs)
    {
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

        return *this;
    }

    ~Array3d()
    {
        for (int k = 0; k < depth; k++) {
            for (int j = 0; j < height; j++)
                delete[] grid[k][j];

            delete[] grid[k];
        }
        delete[] grid;
    }

    void fill(T value) {
        for (int k = 0; k < depth; k++) {
            for (int j = 0; j < height; j++) {
                for (int i = 0; i < width; i++) {
                    set(i, j, k, value);
                }
            }
        }
    }

    T& operator()(int i, int j, int k)
    {
        assert(_isIndexInRange(i, j, k));
        return grid[k][j][i];
    }

    void set(int i, int j, int k, T value) {
        assert(_isIndexInRange(i, j, k));
        grid[k][j][i] = value;
    }

    int width = 0;
    int height = 0;
    int depth = 0;

private:
    void _initializeGrid() {
        grid = new T**[depth];
        for (int k = 0; k < depth; k++) {
            grid[k] = new T*[height];

            for (int j = 0; j < height; j++) {
                grid[k][j] = new T[width];
            }
        }
    }

    inline bool _isIndexInRange(int i, int j, int k) {
        return i >= 0 && j >= 0 && k >= 0 && i < width && j < height && k < depth;
    }

    T ***grid;
};

