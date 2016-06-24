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
#ifndef SUBDIVIDEDARRAY3D_H
#define SUBDIVIDEDARRAY3D_H

#include <vector>
#include <stdexcept>
#include <sstream>

#include "gridindexvector.h"
#include "array3d.h"

template <class T>
class SubdividedArray3d
{
public:

    SubdividedArray3d() {
    }

    SubdividedArray3d(int i, int j, int k) : width(i), height(j), depth(k),
                                             _isize(i), _jsize(j), _ksize(k),
                                             _grid(i, j, k) {
    }

    SubdividedArray3d(int i, int j, int k, T fillValue) : width(i), height(j), depth(k),
                                                          _isize(i), _jsize(j), _ksize(k),
                                                          _grid(i, j, k, fillValue) {
    }

    ~SubdividedArray3d() {
    }

    void setSubdivisionLevel(int level) {
        if (level <= 0) {
            std::string msg = "Error: subdivision level must be greater than or equal to 1.\n";
            msg += "level: " + _toString(level) + "\n";
            throw std::domain_error(msg);
        }

        width  = level * _isize;
        height = level * _jsize;
        depth  = level * _ksize;

        _sublevel = level;
        _invsublevel = 1.0 / level;
    }

    int getSubdivisionLevel() {
        return _sublevel;
    }

    int getUnsubdividedWidth() { return _isize; }
    int getUnsubdividedHeight() { return _ksize; }
    int getUnsubdividedDepth() { return _jsize; }

    void getUnsubdividedDimensions(int *i, int *j, int *k) {
        *i = _isize; *j = _jsize; *k = _ksize;
    }

    void fill(T value) {
        _grid.fill(value);
    }

    T operator()(int i, int j, int k) {
        i = (int)(i * _invsublevel);
        j = (int)(j * _invsublevel);
        k = (int)(k * _invsublevel);

        return _grid(i, j, k);
    }

    T operator()(GridIndex g) {
        g.i = (int)(g.i * _invsublevel);
        g.j = (int)(g.j * _invsublevel);
        g.k = (int)(g.k * _invsublevel);

        return _grid(g);
    }

    T get(int i, int j, int k) {
        i = (int)(i * _invsublevel);
        j = (int)(j * _invsublevel);
        k = (int)(k * _invsublevel);

        return _grid(i, j, k);
    }

    T get(GridIndex g) {
        g.i = (int)(g.i * _invsublevel);
        g.j = (int)(g.j * _invsublevel);
        g.k = (int)(g.k * _invsublevel);

        return _grid(g);
    }

    void set(int i, int j, int k, T value) {
        _grid.set(i, j, k, value);
    }

    void set(GridIndex g, T value) {
        _grid.set(g, value);
    }

    void set(std::vector<GridIndex> &cells, T value) {
        for (unsigned int i = 0; i < cells.size(); i++) {
            _grid.set(cells[i], value);
        }
    }

    void set(GridIndexVector &cells, T value) {
        for (unsigned int i = 0; i < cells.size(); i++) {
            _grid.set(cells[i], value);
        }
    }

    void add(int i, int j, int k, T value) {
        _grid.add(i, j, k, value);
    }

    void add(GridIndex g, T value) {
        _grid.add(g, value);
    }

    T *getPointer(int i, int j, int k) {
        return _grid.getPointer(i, j, k);
    }

    T *getPointer(GridIndex g) {
        return _grid.getPointer(g);
    }

    T *getRawArray() {
        return _grid.getRawArray();
    }

    void setOutOfRangeValue() {
        _grid.setOutOfRangeValue();
    }

    void setOutOfRangeValue(T val) {
        _grid.setOutOfRangeValue(val);
    }

    bool isOutOfRangeValueSet() {
        return _grid.isOutOfRangeValueSet;
    }
    T getOutOfRangeValue() {
        return _grid.getOutOfRangeValue();
    }

    int width = 0;
    int height = 0;
    int depth = 0;

private:

    template<class S>
    std::string _toString(S item) {
        std::ostringstream sstream;
        sstream << item;

        return sstream.str();
    }

    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;

    Array3d<T> _grid;
    unsigned int _sublevel = 1;
     double _invsublevel = 1;

};

#endif
