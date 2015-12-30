#include <pthread.h>
#include <stdio.h>
#include <iostream>
#include <assert.h>
#include <vector>

#include "threading.h"

#ifndef PRODUCERCONSUMERSTACK_H
#define PRODUCERCONSUMERSTACK_H

template <class T>
class ProducerConsumerStack { 

public:

    ProducerConsumerStack() {
    }

    ProducerConsumerStack(int size) : _maxSize(size) {
        assert(size > 0);
        _vector.reserve(_maxSize);
    }

    ~ProducerConsumerStack() {
    }

    void push(T item) {
        _mutex.lock();
        while (_isFull) {
            _notFull.wait(&_mutex);
        }
        
        assert(_vector.size() < _maxSize);
        _vector.push_back(item);

        _isFull = _vector.size() == _maxSize;
        _isEmpty = _vector.size() == 0;

        _mutex.unlock();
        _notEmpty.signal();
    }

    int push(std::vector<int> &items) {
        _mutex.lock();
        while (_isFull) {
            _notFull.wait(&_mutex);
        }
        
        int remaining = _getRemainingCapacity();
        int numPushed = items.size() <= remaining ? items.size() : remaining;
        for (int i = 0; i < numPushed; i++) {
            _vector.push_back(items[i]);
        }

        _isFull = _vector.size() == _maxSize;
        _isEmpty = _vector.size() == 0;

        _mutex.lock();
        _notEmpty.signal();

        return numPushed;
    }

    int push(std::vector<T> &items, int startindex, int endindex) {
        assert(startindex >= 0 && startindex < items.size());
        assert(endindex >= 0 && endindex < items.size());
        assert(startindex <= endindex);

        int numItems = endindex - startindex + 1;

        _mutex.lock();
        while (_isFull) {
            _notFull.wait(&_mutex);
        }
        
        int remaining = _getRemainingCapacity();
        int numPushed = numItems <= remaining ? numItems : remaining;
        for (int i = 0; i < numPushed; i++) {
            _vector.push_back(items[startindex + i]);
        }

        _isFull = _vector.size() == _maxSize;
        _isEmpty = _vector.size() == 0;

        _mutex.unlock();
        _notEmpty.signal();

        return numPushed;
    }

    void pushAll(std::vector<T> &items) {
        int itemsleft = items.size();
        
        while (itemsleft > 0) {
            int numPushed = push(items, items.size() - itemsleft, items.size() - 1);
            itemsleft -= numPushed;
        }
    }

    T pop() {
        _mutex.lock();
        while (_isEmpty) {
            _notEmpty.wait(&_mutex);
        }
        
        assert(_vector.size() != 0);
        T item = _vector.back();
        _vector.pop_back();

        _isFull = _vector.size() == _maxSize;
        _isEmpty = _vector.size() == 0;

        _mutex.unlock();
        _notFull.signal();

        return item;
    }

    int pop(int numItems, std::vector<T> &items) {

        _mutex.lock();
        while (_isEmpty) {
            _notEmpty.wait(&_mutex);
        }
        
        int numPopped = numItems <= _vector.size() ? numItems : _vector.size();
        items.reserve(items.size() + numPopped);
        for (int i = 0; i < numPopped; i++) {
            items.push_back(_vector.back());
            _vector.pop_back();
        }

        _isFull = _vector.size() == _maxSize;
        _isEmpty = _vector.size() == 0;

        _mutex.unlock();
        _notFull.signal();

        return numPopped;
    }

    void popAll(std::vector<T> &items) {
        pop(_maxSize, items);
    }

    int getCapacity() {
        return _maxSize;
    }

private:

    int _getRemainingCapacity() {
        return _maxSize - _vector.size();
    }

    unsigned int _maxSize = 0;
    std::vector<T> _vector;

    bool _isFull = false;
    bool _isEmpty = true;
    Threading::Mutex _mutex;
    Threading::ConditionVariable _notFull, _notEmpty;
};

#endif