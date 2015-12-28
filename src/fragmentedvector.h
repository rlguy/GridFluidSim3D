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
#ifndef FRAGMENTEDVECTOR_H
#define FRAGMENTEDVECTOR_H

#include <stdio.h>
#include <iostream>
#include <vector>
#include <assert.h>

template <class T>
class FragmentedVector 
{
public:

	FragmentedVector() {
		_initializeElementsPerChunk();
	}

	inline void setFragmentSize(unsigned int numElements) {
		if (_nodes.size() != 0) {
			return;
		}
		_bytesPerFragment = numElements*sizeof(T);
		_initializeElementsPerChunk();
	}

	inline void setFragmentByteSize(unsigned int numBytes) {
		if (_nodes.size() != 0) {
			return;
		}
		_bytesPerFragment = numBytes;
		_initializeElementsPerChunk();
	}

	inline unsigned int size() {
		return _size;
	}

	inline bool empty() {
		return _size == 0;
	}

	inline void reserve(unsigned int n) {
		int numFragments = n / _elementsPerFragment;
		int numNewFragments = numFragments - _nodes.size();
		for (int i = 0; i < numNewFragments; i++) {
			_addNewVectorNode();
		}
	}

	inline void shrink_to_fit() {
		while (!_nodes.empty() && _nodes.back().size() == 0) {
			_nodes.pop_back();
		}
	}

	inline T front() {
		assert(_size > 0);
		return _nodes[0][0];
	}

	inline T back() {
		assert(_size > 0);
		return _nodes[_currentNodeIndex].back();
	}

	inline void push_back(T item) {
		if (!_isCurrentNodeFull()) {
			_nodes[_currentNodeIndex].push_back(item);
			_size++;
		} else {
			if (_isLastNode(_currentNodeIndex)) {
				_addNewVectorNode();
				_currentNodeIndex++;
			} else {
				_currentNodeIndex++;
			}

			_nodes[_currentNodeIndex].push_back(item);
			_size++;
		}
	}

	inline void pop_back() {
		if (_size == 0) {
			return;
		}

		_nodes[_currentNodeIndex].pop_back();
		_size--;

		if (_nodes[_currentNodeIndex].empty()) {
			_currentNodeIndex--;
		}

		std::cout << "pop: " << _currentNodeIndex << " " << _nodes[_currentNodeIndex].size() << std::endl;
	}

	inline void clear() {
		for (unsigned int i = 0; i < _nodes.size(); i++) {
			_nodes[i].clear();
		}
		_currentNodeIndex = 0;
		_size = 0;
	}

	const T operator [](int i) const {
		assert(i >= 0 && i < (int)_size);
		int nodeIdx = i * _invElementsPerFragment;
		int itemIdx = i % _elementsPerFragment;
		return _nodes[nodeIdx][itemIdx];
	}

	T& operator[](int i) {
		assert(i >= 0 && i < (int)_size);
		int nodeIdx = i * _invElementsPerFragment;
		int itemIdx = i % _elementsPerFragment;
		return _nodes[nodeIdx][itemIdx];
	}

private:

	class VectorNode 
	{
		public:

			VectorNode() {
			}

			VectorNode(unsigned int maxsize) : _capacity(maxsize) {
				_vector.reserve(_capacity);
			}

			inline unsigned int size() {
				return _vector.size();
			}

			inline bool empty() {
				return _vector.size() == 0;
			}

			inline bool isFull() {
				return _vector.size() == _capacity;
			}

			inline T front() {
				assert(_vector.size() > 0);
				return _vector[0];
			}

			inline T back() {
				assert(_vector.size() > 0);
				return _vector.back();
			}

			inline void push_back(T item) {
				assert(_vector.size() < _capacity);
				_vector.push_back(item);
			}

			inline void pop_back() {
				if (_vector.size() == 0) {
					return;
				}
				_vector.pop_back();
			}

			inline void clear() {
				_vector.clear();
			}

			const T operator [](int i) const {
				assert(i >= 0 && i < (int)_vector.size());
				return _vector[i];
			}

    		T& operator[](int i) {
    			assert(i >= 0 && i < (int)_vector.size());
				return _vector[i];
    		}

		private:

			unsigned int _capacity = 0;
			std::vector<T> _vector;

	};

	void _initializeElementsPerChunk() {
		_elementsPerFragment = (int)(_bytesPerFragment / sizeof(T));
		if (_elementsPerFragment == 0) {
			_elementsPerFragment = 1;
		}

		_invElementsPerFragment = 1.0 / (double)_elementsPerFragment;
	}

	void _addNewVectorNode() {
		_nodes.push_back(VectorNode(_elementsPerFragment));
	}

	inline bool _isLastNode(int i) {
		return i == _nodes.size() - 1;
	}

	inline bool _isCurrentNodeFull() {
		return _currentNodeIndex == -1 || _nodes[_currentNodeIndex].isFull();
	}

	std::vector<VectorNode> _nodes;
	unsigned int _bytesPerFragment = 5e6;
	unsigned int _elementsPerFragment = 0;
	double _invElementsPerFragment = 0;
	int _currentNodeIndex = -1;
	unsigned int _size = 0;

};

#endif