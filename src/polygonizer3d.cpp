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
#include "polygonizer3d.h"


Polygonizer3d::Polygonizer3d() : _isize(0), _jsize(0), _ksize(0), _dx(1),
                                 _vertexValues(0, 0, 0, 0.0),
                                 _isVertexSet(0, 0, 0, false),
                                 _isCellDone(0, 0, 0, false)
{
}

Polygonizer3d::Polygonizer3d(SurfaceField *field) : _field(field)
{
    _field->getGridDimensions(&_isize, &_jsize, &_ksize);
    _dx = _field->getCellSize();
    _surfaceThreshold = _field->getSurfaceThreshold();

    _vertexValues = Array3d<float>(_isize+1, _jsize+1, _ksize+1, 0.0f);
    _isVertexSet = Array3d<bool>(_isize+1, _jsize+1, _ksize+1, false);
    _isCellDone = Array3d<bool>(_isize, _jsize, _ksize, false);

    _isInitialized = true;
}

Polygonizer3d::Polygonizer3d(ImplicitSurfaceScalarField &scalarField)
{
    _scalarField = scalarField;

    int i, j, k;
    scalarField.getGridDimensions(&i, &j, &k);
    _isize = i - 1;
    _jsize = j - 1;
    _ksize = k - 1;

    _dx = scalarField.getCellSize();
    _surfaceThreshold = scalarField.getSurfaceThreshold();

    _vertexValues = Array3d<float>(_isize+1, _jsize+1, _ksize+1, 0.0f);
    scalarField.getScalarField(_vertexValues);

    _isVertexSet = Array3d<bool>(_isize+1, _jsize+1, _ksize+1, true);
    _isCellDone = Array3d<bool>(_isize, _jsize, _ksize, false);

    _isScalarFieldSet = true;

    _isInitialized = true;
}

const int Polygonizer3d::_edgeTable[256] = {
    0x0, 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
    0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
    0x190, 0x99, 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
    0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
    0x230, 0x339, 0x33, 0x13a, 0x636, 0x73f, 0x435, 0x53c,
    0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
    0x3a0, 0x2a9, 0x1a3, 0xaa, 0x7a6, 0x6af, 0x5a5, 0x4ac,
    0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
    0x460, 0x569, 0x663, 0x76a, 0x66, 0x16f, 0x265, 0x36c,
    0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
    0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff, 0x3f5, 0x2fc,
    0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
    0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55, 0x15c,
    0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
    0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc,
    0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
    0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
    0xcc, 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
    0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
    0x15c, 0x55, 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
    0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
    0x2fc, 0x3f5, 0xff, 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
    0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
    0x36c, 0x265, 0x16f, 0x66, 0x76a, 0x663, 0x569, 0x460,
    0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
    0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa, 0x1a3, 0x2a9, 0x3a0,
    0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
    0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33, 0x339, 0x230,
    0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
    0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99, 0x190,
    0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
    0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0 };

const int Polygonizer3d::_triTable[256][16] = {
    { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1 },
    { 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1 },
    { 3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1 },
    { 3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1 },
    { 9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1 },
    { 9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1 },
    { 2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1 },
    { 8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1 },
    { 9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1 },
    { 4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1 },
    { 3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1 },
    { 4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1 },
    { 4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1 },
    { 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1 },
    { 5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1 },
    { 2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1 },
    { 9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1 },
    { 2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1 },
    { 10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1 },
    { 4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1 },
    { 5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1 },
    { 5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1 },
    { 9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1 },
    { 10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1 },
    { 8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1 },
    { 2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1 },
    { 7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1 },
    { 9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1 },
    { 2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1 },
    { 11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1 },
    { 9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1 },
    { 5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1 },
    { 11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1 },
    { 11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1 },
    { 9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1 },
    { 5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1 },
    { 2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1 },
    { 5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1 },
    { 6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1 },
    { 3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1 },
    { 6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1 },
    { 5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1 },
    { 10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1 },
    { 6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1 },
    { 8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1 },
    { 7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1 },
    { 3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1 },
    { 5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1 },
    { 0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1 },
    { 9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1 },
    { 8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1 },
    { 5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1 },
    { 0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1 },
    { 6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1 },
    { 10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1 },
    { 10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1 },
    { 8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1 },
    { 1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1 },
    { 3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1 },
    { 0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1 },
    { 10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1 },
    { 3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1 },
    { 6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1 },
    { 9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1 },
    { 8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1 },
    { 3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1 },
    { 6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1 },
    { 10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1 },
    { 10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1 },
    { 2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1 },
    { 7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1 },
    { 7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1 },
    { 2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1 },
    { 1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1 },
    { 11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1 },
    { 8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1 },
    { 0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1 },
    { 7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1 },
    { 10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1 },
    { 2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1 },
    { 6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1 },
    { 7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1 },
    { 2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1 },
    { 10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1 },
    { 10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1 },
    { 0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1 },
    { 7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1 },
    { 6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1 },
    { 8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1 },
    { 9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1 },
    { 6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1 },
    { 4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1 },
    { 10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1 },
    { 8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1 },
    { 1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1 },
    { 8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1 },
    { 10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1 },
    { 4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1 },
    { 10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1 },
    { 5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1 },
    { 11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1 },
    { 9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1 },
    { 6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1 },
    { 7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1 },
    { 3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1 },
    { 7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1 },
    { 9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1 },
    { 3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1 },
    { 6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1 },
    { 9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1 },
    { 1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1 },
    { 4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1 },
    { 7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1 },
    { 6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1 },
    { 3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1 },
    { 0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1 },
    { 6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1 },
    { 0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1 },
    { 11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1 },
    { 6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1 },
    { 5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1 },
    { 9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1 },
    { 1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1 },
    { 10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1 },
    { 0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1 },
    { 5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1 },
    { 10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1 },
    { 11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1 },
    { 9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1 },
    { 7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1 },
    { 2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1 },
    { 8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1 },
    { 9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1 },
    { 9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1 },
    { 1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1 },
    { 9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1 },
    { 9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1 },
    { 5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1 },
    { 0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1 },
    { 10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1 },
    { 2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1 },
    { 0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1 },
    { 0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1 },
    { 9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1 },
    { 5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1 },
    { 3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1 },
    { 5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1 },
    { 8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1 },
    { 9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1 },
    { 1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1 },
    { 3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1 },
    { 4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1 },
    { 9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1 },
    { 11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1 },
    { 11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1 },
    { 2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1 },
    { 9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1 },
    { 3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1 },
    { 1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1 },
    { 4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1 },
    { 4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1 },
    { 3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1 },
    { 3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1 },
    { 0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1 },
    { 9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1 },
    { 1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 } };


Polygonizer3d::~Polygonizer3d()
{
}

void Polygonizer3d::setInsideCellIndices(std::vector<GridIndex> indices) {
    _insideIndices.clear();
    _insideIndices.reserve(indices.size());
    for (unsigned int i = 0; i < indices.size(); i++) {
        _insideIndices.push_back(indices[i]);
    }
}

void Polygonizer3d::setScalarField(ImplicitSurfaceScalarField &scalarField) {

    int i, j, k;
    scalarField.getGridDimensions(&i, &j, &k);

    assert(_isize == i - 1 && _jsize == j - 1 && _ksize == k - 1);

    _surfaceThreshold = scalarField.getSurfaceThreshold();

    _vertexValues.fill(0.0);
    scalarField.getScalarField(_vertexValues);

    _isVertexSet.fill(true);

    _isScalarFieldSet = true;
}

void Polygonizer3d::writeSurfaceToOBJ(std::string filename) {
    _surface.writeMeshToOBJ(filename);
}

void Polygonizer3d::_getCellVertexPositions(GridIndex g, vmath::vec3 positions[8]) {
    GridIndex verts[8];
    Grid3d::getGridIndexVertices(g, verts);
    for (int i = 0; i < 8; i++) {
        positions[i] = _getVertexPosition(verts[i]);
    }
}

vmath::vec3 Polygonizer3d::_getVertexPosition(GridIndex g) {
    assert(_vertexValues.isIndexInRange(g));
    return (float)_dx*vmath::vec3((float)g.i, (float)g.j, (float)g.k);
}

double Polygonizer3d::_getVertexFieldValue(GridIndex g) {
    assert(_vertexValues.isIndexInRange(g));

    if (!_isVertexSet(g)) {
        assert(!_isScalarFieldSet);

        vmath::vec3 p = _getVertexPosition(g);
        _vertexValues.set(g, (float)_field->getFieldValue(p));
    }

    return _vertexValues(g);;
}

void Polygonizer3d::_resetVertexValues() {
    _vertexValues.fill(0.0);
    _isVertexSet.fill(false);
}

bool Polygonizer3d::_isCellOutsideSurface(GridIndex g) {
    return _getCellSurfaceStatus(g) == 1;
}

bool Polygonizer3d::_isCellInsideSurface(GridIndex g) {
    return _getCellSurfaceStatus(g) == -1;
}

bool Polygonizer3d::_isCellOnSurface(GridIndex g) {
    return _getCellSurfaceStatus(g) == 0;
}

int Polygonizer3d::_getCellSurfaceStatus(GridIndex g) {
    GridIndex vertices[8];
    Grid3d::getGridIndexVertices(g, vertices);

    bool hasInside = false;
    bool hasOutside = false;
    for (int idx = 0; idx < 8; idx ++) {
        double val = _getVertexFieldValue(vertices[idx]);

        if (val > _surfaceThreshold) {
            hasInside = true;
        }
        else {
            hasOutside = true;
        }
    }

    if (hasInside && hasOutside) {
        return 0;
    } 
    else if (hasInside) {
        return -1;
    }
    else {
        return 1;
    }
}

std::vector<GridIndex> Polygonizer3d::_processSeedCell(GridIndex seed, 
                                                       Array3d<bool> &isCellDone) {
    std::vector<GridIndex> seedSurfaceCells;

    isCellDone.set(seed, true);
    std::vector<GridIndex> queue;
    queue.push_back(seed);

    while (!queue.empty()) {
        GridIndex c = queue[queue.size() - 1];
        queue.pop_back();

        GridIndex neighbours[6];
        Grid3d::getNeighbourGridIndices6(c, neighbours);
        for (int idx = 0; idx < 6; idx++) {
            GridIndex n = neighbours[idx];
            if (Grid3d::isGridIndexInRange(n, _isize, _jsize, _ksize) && 
                    !isCellDone(n) && _isCellOnSurface(n)) {
                isCellDone.set(n, true);
                queue.push_back(n);
            }
        }

        seedSurfaceCells.push_back(c);
    }

    return seedSurfaceCells;
}

std::vector<GridIndex> Polygonizer3d::_findSurfaceCellsUsingInsideIndices() {
    std::vector<GridIndex> surfaceCells;
    _isCellDone.fill(false);

    for (unsigned int i = 0; i < _insideIndices.size(); i++) {
        GridIndex cell = _insideIndices[i];
        
        if (_isCellDone(cell)) {
            continue;
        }

        while (Grid3d::isGridIndexInRange(cell, _isize, _jsize, _ksize)) {

            if (_isCellOnSurface(cell)) {
                std::vector<GridIndex> seedSurfaceCells = _processSeedCell(cell, _isCellDone);
                surfaceCells.insert(surfaceCells.end(), seedSurfaceCells.begin(), seedSurfaceCells.end());
                break;
            }

            // march +z until cell surface is found or index is out of range
            _isCellDone.set(cell, true);
            cell = GridIndex(cell.i, cell.j, cell.k + 1);
        }
    }

    return surfaceCells;
}

std::vector<GridIndex> Polygonizer3d::_findSurfaceCellsUsingScalarField() {
    std::vector<GridIndex> surfaceCells;
    _isCellDone.fill(false);

    for (int k = 0; k < _ksize; k++) {
        for (int j = 0; j < _jsize; j++) {
            for (int i = 0; i < _isize; i++) {
                GridIndex cell = GridIndex(i, j, k);
        
                if (_isCellDone(cell)) {
                    continue;
                }

                while (Grid3d::isGridIndexInRange(cell, _isize, _jsize, _ksize)) {

                    if (_isCellOnSurface(cell)) {
                        std::vector<GridIndex> seedSurfaceCells = _processSeedCell(cell, _isCellDone);
                        surfaceCells.insert(surfaceCells.end(), seedSurfaceCells.begin(), seedSurfaceCells.end());
                        break;
                    }

                    // march +z until cell surface is found or index is out of range
                    _isCellDone.set(cell, true);
                    cell = GridIndex(cell.i, cell.j, cell.k + 1);
                }
            }
        }
    }

    return surfaceCells;
}

std::vector<GridIndex> Polygonizer3d::_findSurfaceCells() {
    if (!_isScalarFieldSet) {
        _resetVertexValues();
    }

    if (_insideIndices.size() == 0 && _isScalarFieldSet) {
        return _findSurfaceCellsUsingScalarField();
    } else {
        return _findSurfaceCellsUsingInsideIndices();
    }
}

int Polygonizer3d::_calculateCubeIndex(GridIndex g, double isolevel) {
    GridIndex vs[8];
    Grid3d::getGridIndexVertices(g, vs);

    int cubeIndex = 0;
    if (_getVertexFieldValue(vs[0]) > isolevel) { cubeIndex |= 1; }
    if (_getVertexFieldValue(vs[1]) > isolevel) { cubeIndex |= 2; }
    if (_getVertexFieldValue(vs[2]) > isolevel) { cubeIndex |= 4; }
    if (_getVertexFieldValue(vs[3]) > isolevel) { cubeIndex |= 8; }
    if (_getVertexFieldValue(vs[4]) > isolevel) { cubeIndex |= 16; }
    if (_getVertexFieldValue(vs[5]) > isolevel) { cubeIndex |= 32; }
    if (_getVertexFieldValue(vs[6]) > isolevel) { cubeIndex |= 64; }
    if (_getVertexFieldValue(vs[7]) > isolevel) { cubeIndex |= 128; }

    return cubeIndex;
}

vmath::vec3 Polygonizer3d::_vertexInterp(double isolevel, vmath::vec3 p1, vmath::vec3 p2, 
                                                        double valp1, double valp2) {

    // Don't return a point that is exactly on p1 or p2.
    // This could result in a triangle with equal vertices and
    // its normal will be calculated as undefined.
    double eps = 10e-6;
    if (fabs(isolevel - valp1) < eps || fabs(valp1 - valp2) < eps) {
        vmath::vec3 v = vmath::normalize(p2 - p1);
        return p1 + (float)(eps*_dx)*v;
    }
    if (fabs(isolevel - valp2) < eps) { 
        vmath::vec3 v = vmath::normalize(p2 - p1);
        return p2 - (float)(eps*_dx)*v; 
    }

    double tol = 1e-6;
    int nmax = 50;
    int n = 0;
    vmath::vec3 a = p1;
    vmath::vec3 b = p2;
    vmath::vec3 c;
    double vala = valp1 - isolevel;
    double valc = 0.0;
    while (n < nmax) {
        c = 0.5f*(a + b);

        if (_isScalarFieldSet) {
            valc = _scalarField.tricubicInterpolation(c) - isolevel;
        } else {
            valc = _field->getFieldValue(c) - isolevel;
        }
        

        if (fabs(valc) < tol || vmath::length(b - a) < tol ) {
            return c;
        }

        n++;

        if ((valc < 0.0 && vala < 0.0) || (valc > 0.0 && vala > 0.0)) {
            a = c;
            vala = valc;
        } else {
            b = c;
        }
        
    }

    // method failed: linearly interpolate value
    double mu = (isolevel - valp1) / (valp2 - valp1);
    return p1 + (float)mu*(p2 - p1);
}

void Polygonizer3d::_calculateVertexList(GridIndex g, double isolevel, int cubeIndex, 
                                         int vertexList[12], EdgeGrid &edges) {
    GridIndex vertices[8];
    double vertexValues[8];
    vmath::vec3 vertexPositions[8];

    Grid3d::getGridIndexVertices(g, vertices);
    for (int i = 0; i < 8; i++) {
        vertexValues[i] = _getVertexFieldValue(vertices[i]);
        vertexPositions[i] = _getVertexPosition(vertices[i]);
    }

    if (_edgeTable[cubeIndex] & 1) {
        if (!edges.isSetU(vertices[0])) {
            vmath::vec3 v = _vertexInterp(isolevel, vertexPositions[0], vertexPositions[1], 
                                                     vertexValues[0],    vertexValues[1]);
            _surface.vertices.push_back(v);
            edges.U.set(vertices[0], _surface.vertices.size() - 1);
            edges.isSetU.set(vertices[0], true);
        }
        vertexList[0] = edges.U(vertices[0]);
    }
    if (_edgeTable[cubeIndex] & 2) {
        if (!edges.isSetW(vertices[1])) {
            vmath::vec3 v = _vertexInterp(isolevel, vertexPositions[1], vertexPositions[2], 
                                                     vertexValues[1],    vertexValues[2]);
            _surface.vertices.push_back(v);
            edges.W.set(vertices[1], _surface.vertices.size() - 1);
            edges.isSetW.set(vertices[1], true);
        }
        vertexList[1] = edges.W(vertices[1]);
    }
    if (_edgeTable[cubeIndex] & 4) {
        if (!edges.isSetU(vertices[3])) {
            vmath::vec3 v = _vertexInterp(isolevel, vertexPositions[2], vertexPositions[3], 
                                                     vertexValues[2],    vertexValues[3]);
            _surface.vertices.push_back(v);
            edges.U.set(vertices[3], _surface.vertices.size() - 1);
            edges.isSetU.set(vertices[3], true);
        }
        vertexList[2] = edges.U(vertices[3]);
    }
    if (_edgeTable[cubeIndex] & 8) {
        if (!edges.isSetW(vertices[0])) {
            vmath::vec3 v = _vertexInterp(isolevel, vertexPositions[3], vertexPositions[0], 
                                                     vertexValues[3],    vertexValues[0]);
            _surface.vertices.push_back(v);
            edges.W.set(vertices[0], _surface.vertices.size() - 1);
            edges.isSetW.set(vertices[0], true);
        }
        vertexList[3] = edges.W(vertices[0]);
    }
    if (_edgeTable[cubeIndex] & 16) {
        if (!edges.isSetU(vertices[4])) {
            vmath::vec3 v = _vertexInterp(isolevel, vertexPositions[4], vertexPositions[5], 
                                                     vertexValues[4],    vertexValues[5]);
            _surface.vertices.push_back(v);
            edges.U.set(vertices[4], _surface.vertices.size() - 1);
            edges.isSetU.set(vertices[4], true);
        }
        vertexList[4] = edges.U(vertices[4]);
    }
    if (_edgeTable[cubeIndex] & 32) {
        if (!edges.isSetW(vertices[5])) {
            vmath::vec3 v = _vertexInterp(isolevel, vertexPositions[5], vertexPositions[6], 
                                                     vertexValues[5],    vertexValues[6]);
            _surface.vertices.push_back(v);
            edges.W.set(vertices[5], _surface.vertices.size() - 1);
            edges.isSetW.set(vertices[5], true);
        }
        vertexList[5] = edges.W(vertices[5]);
    }
    if (_edgeTable[cubeIndex] & 64) {
        if (!edges.isSetU(vertices[7])) {
            vmath::vec3 v = _vertexInterp(isolevel, vertexPositions[6], vertexPositions[7], 
                                                     vertexValues[6],    vertexValues[7]);
            _surface.vertices.push_back(v);
            edges.U.set(vertices[7], _surface.vertices.size() - 1);
            edges.isSetU.set(vertices[7], true);
        }
        vertexList[6] = edges.U(vertices[7]);
    }
    if (_edgeTable[cubeIndex] & 128) {
        if (!edges.isSetW(vertices[4])) {
            vmath::vec3 v = _vertexInterp(isolevel, vertexPositions[7], vertexPositions[4], 
                                                     vertexValues[7],    vertexValues[4]);
            _surface.vertices.push_back(v);
            edges.W.set(vertices[4], _surface.vertices.size() - 1);
            edges.isSetW.set(vertices[4], true);
        }
        vertexList[7] = edges.W(vertices[4]);
    }
    if (_edgeTable[cubeIndex] & 256) {
        if (!edges.isSetV(vertices[0])) {
            vmath::vec3 v = _vertexInterp(isolevel, vertexPositions[0], vertexPositions[4], 
                                                     vertexValues[0],    vertexValues[4]);
            _surface.vertices.push_back(v);
            edges.V.set(vertices[0], _surface.vertices.size() - 1);
            edges.isSetV.set(vertices[0], true);
        }
        vertexList[8] = edges.V(vertices[0]);
    }
    if (_edgeTable[cubeIndex] & 512) {
        if (!edges.isSetV(vertices[1])) {
            vmath::vec3 v = _vertexInterp(isolevel, vertexPositions[1], vertexPositions[5], 
                                                     vertexValues[1],    vertexValues[5]);
            _surface.vertices.push_back(v);
            edges.V.set(vertices[1], _surface.vertices.size() - 1);
            edges.isSetV.set(vertices[1], true);
        }
        vertexList[9] = edges.V(vertices[1]);
    }
    if (_edgeTable[cubeIndex] & 1024) {
        if (!edges.isSetV(vertices[2])) {
            vmath::vec3 v = _vertexInterp(isolevel, vertexPositions[2], vertexPositions[6], 
                                                     vertexValues[2],    vertexValues[6]);
            _surface.vertices.push_back(v);
            edges.V.set(vertices[2], _surface.vertices.size() - 1);
            edges.isSetV.set(vertices[2], true);
        }
        vertexList[10] = edges.V(vertices[2]);
    }
    if (_edgeTable[cubeIndex] & 2048) {
        if (!edges.isSetV(vertices[3])) {
            vmath::vec3 v = _vertexInterp(isolevel, vertexPositions[3], vertexPositions[7], 
                                                     vertexValues[3],    vertexValues[7]);
            _surface.vertices.push_back(v);
            edges.V.set(vertices[3], _surface.vertices.size() - 1);
            edges.isSetV.set(vertices[3], true);
        }
        vertexList[11] = edges.V(vertices[3]);
    }
}

// method of polygonizing a cell is adapted from:
// http://paulbourke.net/geometry/polygonise/
void Polygonizer3d::_polygonizeCell(GridIndex g, double isolevel, EdgeGrid &edges) {
    int cubeIndex = _calculateCubeIndex(g, isolevel);

    /* Cube is entirely in/out of the surface */
    if (_edgeTable[cubeIndex] == 0) {
        return;
    }

    int vertexList[12];
    _calculateVertexList(g, isolevel, cubeIndex, vertexList, edges);

    for (int i = 0; _triTable[cubeIndex][i] != -1; i += 3) {
        Triangle t = Triangle(vertexList[_triTable[cubeIndex][i]],
                              vertexList[_triTable[cubeIndex][i + 1]],
                              vertexList[_triTable[cubeIndex][i + 2]]);

        _surface.triangles.push_back(t);
    }
}

void Polygonizer3d::_calculateSurfaceTriangles() {
    _surface.clear();
    EdgeGrid edges(_isize, _jsize, _ksize);
    for (unsigned int i = 0; i < _surfaceCells.size(); i++) {
        _polygonizeCell(_surfaceCells[i], _surfaceThreshold, edges);
    }
}

void Polygonizer3d::polygonizeSurface() {
    _surfaceCells = _findSurfaceCells();
    _calculateSurfaceTriangles();
    _surface.removeDuplicateTriangles(); // Polygonization method produces
                                         // some identical triangles for some
                                         // currently unknown reason.
    _surface.updateVertexNormals();
}