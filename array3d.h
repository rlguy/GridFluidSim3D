#pragma once

#include <assert.h>

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

    Array3d(const Array3d &obj)
    {
        width = obj.width;
        height = obj.height;
        depth = obj.depth;

        _initializeGrid();
    }

    Array3d operator=(const Array3d & rhs)
    {
        width = rhs.width;
        height = rhs.height;
        depth = rhs.depth;

        _initializeGrid();

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

    int width = 1;
    int height = 1;
    int depth = 1;

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

