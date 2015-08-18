#pragma once

#include <vector>
#include <unordered_map>
#include <assert.h>

#include "glm/glm.hpp"
#include "array3d.h"
#include "grid3d.h"

template <class T>
class SpatialGrid
{
public:
    SpatialGrid() : _isize(0), _jsize(0), _ksize(0),
                    _dx(0.0),
                    _grid(0, 0, 0) {
    }

    SpatialGrid(int i, int j, int k, double cellsize) :  _isize(i), _jsize(j), _ksize(k),
                                                         _dx(cellsize),
                                                         _grid(i, j, k) {
        _grid.setOutOfRangeValue(std::vector<SpatialGridObject>());
    }

    SpatialGrid(SpatialGrid &obj) {
        _isize = obj._isize;
        _jsize = obj._jsize;
        _ksize = obj._ksize;
        _dx = obj._dx;
        _unique_identifier = obj._unique_identifier;
        _grid = obj._grid;
    }

    SpatialGrid operator=(SpatialGrid & rhs) {
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
                    objs->clear();
                    objs->shrink_to_fit();
                }
            }
        }

        _unique_identifier = 0;
    }

    void insert(T object, glm::vec3 pos) {
        std::vector<SpatialGridObject> *objs;
        GridIndex g = Grid3d::positionToGridIndex(pos, _dx);
        int id = _get_unique_identifier();

        objs = _grid.getPointer(g);
        objs.push_back(SpatialGridObject(object, pos, id));
    }

    void insert(T object, glm::vec3 pos, double r) {
        GridIndex gmin, gmax;
        Grid3d::getGridIndexBounds(pos, r, _dx, _isize, _jsize, _ksize, &gmin, &gmax);

        int id = _get_unique_identifier();
        SpatialGridObject obj = SpatialGridObject(object, pos, r, id);

        std::vector<SpatialGridObject> *objs;
        for (int k = gmin.k; k <= gmax.k; k++) {
            for (int j = gmin.j; j <= gmax.j; j++) {
                for (int i = gmin.i; i <= gmax.i; i++) {
                    objs = _grid.getPointer(i, j, k);
                    objs->push_back(obj);
                }
            }
        }
    }
    
    void query(glm::vec3 pos, double r, std::vector<T> &storage) {
        GridIndex gmin, gmax;
        Grid3d::getGridIndexBounds(pos, r, _dx, _isize, _jsize, _ksize &gmin, &gmax);

        std::unordered_map<int, bool> isDuplicate;

        std::vector<SpatialGridObject> *objs;
        SpatialGridObject o;
        for (int k = gmin.k; k <= gmax.k; k++) {
            for (int j = gmin.j; j <= gmax.j; j++) {
                for (int i = gmin.i; i <= gmax.i; i++) {
                    objs = _grid.getPointer(i, j, k);
                    
                    for (int idx = 0; idx < objs->size(); idx++) {
                        o = objs->at(idx);
                        if (isDuplicate.find(o.id) != _isDuplicate.end() && 
                            _isSphereCollision(pos, r, o.position, o.radius)) {
                            storage.push_back(o.object);
                            std::pair<int, bool> pair(o.id, true);
                            isDuplicate(pair);
                        }
                    }

                }
            }
        }
    }

    std::vector<T> query(glm::vec3 pos, double r) {
        std::vector<T> storage;
        query(pos, r, storage);
        return storage;
    }

    void query(glm::vec3 pos, std::vector<T> &storage) {
        GridIndex g = Grid3d::positionToGridIndex(pos, _dx);

        if (!_grid.isIndexInRange(g)) {
            std::cout << pos.x << " " << pos.y << " " << pos.z << std::endl;
            std::cout << g.i << " " << g.j << " " << g.k << std::endl;
            return;
        }

        std::vector<SpatialGridObject> *objs = _grid.getPointer(g);
        SpatialGridObject o;
        objs = _grid.getPointer(g);
                    
        for (unsigned int idx = 0; idx < objs->size(); idx++) {
            o = objs->at(idx);
            glm::vec3 v = o.position - pos;
            double distsq = glm::dot(v, v);
            double maxdistsq = o.radius*o.radius;

            if (distsq < maxdistsq) {
                storage.push_back(o.object);
            }
        }
    }

    std::vector<T> query(glm::vec3 pos) {
        std::vector<T> storage;
        query(pos, storage);
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

    inline int _get_unique_identifier() {
        int id = _unique_identifier;
        _unique_identifier++;
        return id;
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

