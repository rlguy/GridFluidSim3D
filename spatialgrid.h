#pragma once

#include <vector>
#include <assert.h>

#include "glm/glm.hpp"

template <class T>
class SpatialGrid
{
public:
    SpatialGrid() : _isize(0), _jsize(0), _ksize(0),
                    _dx(0.0),
                    _grid(Array3d<SpatialGridObject>(0, 0, 0)) {
    }

    SpatialGrid(int i, int j, int k, double cellsize) :  _isize(i), _jsize(j), _ksize(k),
                                                         _dx(cellsize),
                                                         _grid(Array3d<SpatialGridObject>(i, j, k)) {
    }

    SpatialGrid(Array3d &obj) {
        _isize = obj._isize;
        _jsize = obj._jsize;
        _ksize = obj._ksize;
        _dx = obj._dx;
        _unique_identifier = obj._unique_identifier;
        _grid = obj._grid;
    }

    SpatialGrid operator=(Array3d & rhs) {
        _isize = rhs._isize;
        _jsize = rhs._jsize;
        _ksize = rhs._ksize;
        _dx = rhs._dx;
        _unique_identifier = rhs._unique_identifier;
        _grid = rhs._grid;

        return *this;
    }

    ~SpatialGrid() {
    }

    void clear() {
        std::vector<SpatialGridObject> *objs;
        for (int k = 0; k < _grid.depth; k++) {
            for (int j = 0; j < _grid.height; j++) {
                for (int i = 0; i < _grid.depth; i++) {
                    objs = _grid.getPointer(i, j, k);
                    objs.clear();
                }
            }
        }

        _unique_identifier = 0;
    }

    void insert(T object, glm::vec3 pos) {
        std::vector<SpatialGridObject> *objs;
        GridIndex g =_positionToGridIndex(pos);
        int id = _get_unique_identifier();

        objs = _grid.getPointer(g);
        objs.push_back(SpatialGridObject(object, pos, id));
    }

    void insert(T object, glm::vec3 pos, double r) {
        GridIndex gmin, gmax;
        _getGridIndexBounds(pos, r, &gmin, &gmax);

        int id = _get_unique_identifier();
        SpatialGridObject obj = SpatialGridObject(object, pos, r, id);

        std::vector<SpatialGridObject> *objs;
        for (int k = gmin.k; k <= gmax.k; k++) {
            for (int j = gmin.j; j <= gmax.j, j++) {
                for (int i = gmin.i; i <= gmax.i; i++) {
                    objs = _grid.getPointer(i, j, k);
                    objs.push_back(obj);
                }
            }
        }
    }
    
    void query(glm::vec3 pos, double r, std::vector<T> &storage) {
        GridIndex gmin, gmax;
        _getGridIndexBounds(pos, r, &gmin, &gmax);

        std::unordered_map<int, bool> isDuplicate;

        std::vector<SpatialGridObject> *objs;
        SpatialGridObject o;
        for (int k = gmin.k; k <= gmax.k; k++) {
            for (int j = gmin.j; j <= gmax.j, j++) {
                for (int i = gmin.i; i <= gmax.i; i++) {
                    objs = _grid.getPointer(i, j, k);
                    
                    for (int idx = 0; idx < objs->size(); idx++) {
                        o = objs[idx];
                        if (isDuplicate.find(h) != _isDuplicate.end() && 
                            _isSphereCollision(pos, r, o.position, o.radius)) {
                            storage.push_back(o.object);
                            isDuplicate(pair(o.id, true));
                        }
                    }

                }
            }
        }
    }

    void query(glm::vec3 pos, double r) {
        std::vector<T> storage;
        query(pos, r, storage);
        return storage;
    }

private:
    struct SpatialGridObject {
        glm::vec3 position;
        double radius;
        int id;
        T object;

        SpatialGridObject() : position(glm::vec3(0.0, 0.0, 0.0)), radius(0), id(-1) {}
        SpatialGridObject(T obj, glm::vec3 pos, int ident) : position(pos), radius(0.0), 
                                                             id(ident) object(obj) {}
        SpatialGridObject(T obj, glm::vec3 pos, double r, int ident) : position(pos), 
                                                                       radius(r), 
                                                                       id(ident),
                                                                       object(obj) {}
    };

    inline GridIndex _positionToGridIndex(glm::vec3 p) {
        double invdx = 1.0 / _dx;
        return GridIndex(
            floor(p.x*invdx);
            floor(p.y*invdx);
            floor(p.z*invdx);
        );
    }

    inline glm::vec3 _GridIndexToPosition(GridIndex g) {
        return glm::vec3(g.i*_dx, g.j*_dx, g.k*_dx);
    }

    inline int _get_unique_identifier() {
        int id = _unique_identifier;
        _unique_identifier++;
        return id;
    }

    void _getGridIndexBounds(glm::vec3 pos, double r, GridIndex *gmin, GridIndex *gmax) {
        GridIndex c = _positionToGridIndex(pos);
        glm::vec3 cpos = _GridIndexToPosition(c);
        glm::vec3 trans = pos - cpos;
        double inv = 1.0 / _dx;

        int imin = c.i - fmax(0, ceil((r-trans.x)*inv));
        int jmin = c.j - fmax(0, ceil((r-trans.y)*inv));
        int kmin = c.k - fmax(0, ceil((r-trans.z)*inv));
        int imax = c.i + fmax(0, ceil((r-size+trans.x)*inv));
        int jmax = c.j + fmax(0, ceil((r-size+trans.y)*inv));
        int kmax = c.k + fmax(0, ceil((r-size+trans.z)*inv));

        *gmin = GridIndex(fmax(imin, 0), fmax(jmin, 0), fmax(kmin, 0));
        *gmax = GridIndex(fmin(imax, _isize-1), fmin(jmax, _jsize-1), fmin(kmax, _ksize-1));
    }

    bool _isSphereCollision(glm::vec3 p1, double r1, glm::vec3 p2, double r2) {
        double maxdistsq = (r1 + r2) * (r1 + r2);
        glm::vec3 v = r2 - r1;
        double distsq = glm::dot(v, v);
        return distsq < maxdistsq;
    }

    int _isize;
    int _jsize;
    int _ksize;
    double _dx;

    int _unique_identifier = 0;

    Array3d<std::vector<SpatialGridObject>> _grid;
};

