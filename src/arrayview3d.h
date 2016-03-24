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
#ifndef ARRAYVIEW3D_H
#define ARRAYVIEW3D_H

#include <vector>
#include <assert.h>

#include "array3d.h"

template <class T>
class ArrayView3d
{
public:

    ArrayView3d() {
        setDimensions(0, 0, 0);
        setOffset(0, 0, 0);
        setArray3d(&_dummyGrid);
    }

    ArrayView3d(Array3d<T> *grid) {
        setDimensions(0, 0, 0);
        setOffset(0, 0, 0);
        setArray3d(grid);
    }

    ArrayView3d(int isize, int jsize, int ksize, Array3d<T> *grid) {
        setDimensions(isize, jsize, ksize);
        setOffset(0, 0, 0);
        setArray3d(grid);
    }

    ArrayView3d(int isize, int jsize, int ksize, 
                int offi, int offj, int offk, Array3d<T> *grid) {
        setDimensions(isize, jsize, ksize);
        setOffset(offi, offj, offk);
        setArray3d(grid);
    }

    ArrayView3d(int isize, int jsize, int ksize, 
                GridIndex offset, Array3d<T> *grid) {
        setDimensions(isize, jsize, ksize);
        setOffset(offset);
        setArray3d(grid);
    }

    ArrayView3d(const ArrayView3d &obj) {
        width = obj.width;
        height = obj.height;
        depth = obj.depth;

        _ioffset = obj._ioffset;
        _joffset = obj._joffset;
        _koffset = obj._koffset;

        _dummyGrid = Array3d<T>();

        if (obj._parent == &obj._dummyGrid) {
            _parent = &_dummyGrid;
        } else {
            _parent = obj._parent;
        }
    }

    ArrayView3d operator=(const ArrayView3d &rhs) {
        width = rhs.width;
        height = rhs.height;
        depth = rhs.depth;

        _ioffset = rhs._ioffset;
        _joffset = rhs._joffset;
        _koffset = rhs._koffset;

        _dummyGrid = Array3d<T>();

        if (rhs._parent == &rhs._dummyGrid) {
            _parent = &_dummyGrid;
        } else {
            _parent = rhs._parent;
        }

        return *this;
    }

    ~ArrayView3d() {
    }

    void setDimensions(int isize, int jsize, int ksize) {
        assert(_isDimensionsValid(isize, jsize, ksize));
        width = isize;
        height = jsize;
        depth = ksize;
    }

    void getDimensions(int *isize, int *jsize, int *ksize) {
        *isize = width;
        *ksize = height;
        *jsize = depth;
    }

    GridIndex getDimensions() {
        return GridIndex(width, height, depth);
    }

    void setOffset(int offi, int offj, int offk) {
        _ioffset = offi;
        _joffset = offj;
        _koffset = offk;
    }

    void setOffset(GridIndex offset) {
        _ioffset = offset.i;
        _joffset = offset.j;
        _koffset = offset.k;
    }

    void getOffset(int *offi, int *offj, int *offk) {
        *offi = _ioffset;
        *offj = _joffset;
        *offk = _koffset;
    }

    GridIndex getOffset() {
        return GridIndex(_ioffset, _joffset, _koffset);
    }

    void setArray3d(Array3d<T> *grid) {
        _parent = grid;
    }

    Array3d<T> *getArray3d() {
        return _parent;
    }

    Array3d<T> getViewAsArray3d() {
        Array3d<T> view(width, height, depth);

        for (int k = 0; k < depth; k++) {
            for (int j = 0; j < height; j++) {
                for (int i = 0; i < width; i++) {
                    view.set(i, j, k, get(i, j, k));
                }
            }
        }

        return view;
    }

    void getViewAsArray3d(Array3d<T> &view) {
        assert(view.width == width && view.height == height && view.depth = depth);

        for (int k = 0; k < depth; k++) {
            for (int j = 0; j < height; j++) {
                for (int i = 0; i < width; i++) {
                    view.set(i, j, k, get(i, j, k));
                }
            }
        }

        return view;
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

    T get(int i, int j, int k) {
        assert(_isIndexInView(i, j, k));

        GridIndex pidx = _viewToParentIndex(i, j, k);
        bool isInRange = _parent->isIndexInRange(pidx);
        if (!isInRange && _parent->isOutOfRangeValueSet()) {
            return _parent->getOutOfRangeValue();
        }
        assert(isInRange);

        return _parent->get(pidx);
    }

    T get(GridIndex g) {
        assert(_isIndexInView(g));
        
        GridIndex pidx = _viewToParentIndex(g);
        bool isInRange = _parent->isIndexInRange(pidx);
        if (!isInRange && _parent->isOutOfRangeValueSet()) {
            return _parent->getOutOfRangeValue();
        }
        assert(isInRange);

        return _parent->get(pidx);   
    }

    T operator()(int i, int j, int k) {
        return get(i, j, k);
    }

    T operator()(GridIndex g) {
        return get(g);  
    }

    void set(int i, int j, int k, T value) {
        assert(_isIndexInView(i, j, k));

        GridIndex pidx = _viewToParentIndex(i, j, k);
        if (_parent->isIndexInRange(pidx)) {
            _parent->set(pidx, value);
        }
    }

    void set(GridIndex g, T value) {
        assert(_isIndexInView(g));

        GridIndex pidx = _viewToParentIndex(g);
        if (_parent->isIndexInRange(pidx)) {
            _parent->set(pidx, value);
        }
    }

    void set(std::vector<GridIndex> &cells, T value) {
        for (unsigned int i = 0; i < cells.size(); i++) {
            set(cells[i], value);
        }
    }

    void add(int i, int j, int k, T value) {
        assert(_isIndexInView(i, j, k));

        GridIndex pidx = _viewToParentIndex(i, j, k);
        if (_parent->isIndexInRange(pidx)) {
            _parent->add(pidx, value);
        }   
    }

    void add(GridIndex g, T value) {
        assert(_isIndexInView(g));

        GridIndex pidx = _viewToParentIndex(g);
        if (_parent->isIndexInRange(pidx)) {
            _parent->add(pidx, value);
        }   
    }

    T *getPointer(int i, int j, int k) {
        assert(_isIndexInView(i, j, k));
        _parent->getPointer(_viewToParentIndex(i, j, k));
    }

    T *getPointer(GridIndex g) {
        assert(_isIndexInView(g));
        _parent->getPointer(_viewToParentIndex(g));
    }

    inline bool isIndexInView(int i, int j, int k) {
        return _isIndexInView(i, j, k);
    }

    inline bool isIndexInView(GridIndex g) {
        return _isIndexInView(g);
    }

    int width = 0;
    int height = 0;
    int depth = 0;

private:

    inline bool _isDimensionsValid(int isize, int jsize, int ksize) {
        return isize >= 0 && jsize >= 0 && ksize >= 0;
    }

    inline bool _isIndexInView(int i, int j, int k) {
        return i >= 0 && j >= 0 && k >= 0 && i < width && j < height && k < depth;
    }

    inline bool _isIndexInView(GridIndex g) {
        return g.i >= 0 && g.j >= 0 && g.k >= 0 && g.i < width && g.j < height && g.k < depth;
    }

    inline GridIndex _viewToParentIndex(int i, int j, int k) {
        return GridIndex(i + _ioffset, j + _joffset, k + _koffset);
    }

    inline GridIndex _viewToParentIndex(GridIndex g) {
        return GridIndex(g.i + _ioffset, g.j + _joffset, g.k + _koffset);
    }

    int _ioffset = 0;
    int _joffset = 0;
    int _koffset = 0;

    Array3d<T> *_parent;
    Array3d<T> _dummyGrid;

};

#endif
