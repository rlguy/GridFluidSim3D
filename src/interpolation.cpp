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
#include "interpolation.h"

/* 
    Cubic interpolation methods from http://www.paulinternet.nl/?page=bicubic

    - p is indexed in order by p[k][j][i]
    - x, y, z are in [0,1]
    - this function will interpolate the volume between point Index 1 and 2
+*/
double Interpolation::tricubicInterpolate(double p[4][4][4], double x, double y, double z) {
    double arr[4];
    arr[0] = bicubicInterpolate(p[0], y, z);
    arr[1] = bicubicInterpolate(p[1], y, z);
    arr[2] = bicubicInterpolate(p[2], y, z);
    arr[3] = bicubicInterpolate(p[3], y, z);
    return cubicInterpolate(arr, x);
}

double Interpolation::bicubicInterpolate(double p[4][4], double x, double y) {
    double arr[4];
    arr[0] = cubicInterpolate(p[0], y);
    arr[1] = cubicInterpolate(p[1], y);
    arr[2] = cubicInterpolate(p[2], y);
    arr[3] = cubicInterpolate(p[3], y);
    return cubicInterpolate(arr, x);
}

double Interpolation::cubicInterpolate(double p[4], double x) {
    return p[1] + 0.5 * x*(p[2] - p[0] + x*(2.0*p[0] - 5.0*p[1] + 4.0*p[2] - p[3] + x*(3.0*(p[1] - p[2]) + p[3] - p[0])));
}

double Interpolation::fastTricubicInterpolate(double p[64], double x, double y, double z) {
    double b00 = p[0+1] + 0.5 * x*(p[0+2] - p[0+0] + x*(2.0*p[0+0] - 5.0*p[0+1] + 4.0*p[0+2] - p[0+3] + x*(3.0*(p[0+1] - p[0+2]) + p[0+3] - p[0+0])));
    double b01 = p[0+5] + 0.5 * x*(p[0+6] - p[0+4] + x*(2.0*p[0+4] - 5.0*p[0+5] + 4.0*p[0+6] - p[0+7] + x*(3.0*(p[0+5] - p[0+6]) + p[0+7] - p[0+4])));
    double b02 = p[0+9] + 0.5 * x*(p[0+10] - p[0+8] + x*(2.0*p[0+8] - 5.0*p[0+9] + 4.0*p[0+10] - p[0+11] + x*(3.0*(p[0+9] - p[0+10]) + p[0+11] - p[0+8])));
    double b03 = p[0+13] + 0.5 * x*(p[0+14] - p[0+12] + x*(2.0*p[0+12] - 5.0*p[0+13] + 4.0*p[0+14] - p[0+15] + x*(3.0*(p[0+13] - p[0+14]) + p[0+15] - p[0+12])));

    double b10 = p[16+1] + 0.5 * x*(p[16+2] - p[16+0] + x*(2.0*p[16+0] - 5.0*p[16+1] + 4.0*p[16+2] - p[16+3] + x*(3.0*(p[16+1] - p[16+2]) + p[16+3] - p[16+0])));
    double b11 = p[16+5] + 0.5 * x*(p[16+6] - p[16+4] + x*(2.0*p[16+4] - 5.0*p[16+5] + 4.0*p[16+6] - p[16+7] + x*(3.0*(p[16+5] - p[16+6]) + p[16+7] - p[16+4])));
    double b12 = p[16+9] + 0.5 * x*(p[16+10] - p[16+8] + x*(2.0*p[16+8] - 5.0*p[16+9] + 4.0*p[16+10] - p[16+11] + x*(3.0*(p[16+9] - p[16+10]) + p[16+11] - p[16+8])));
    double b13 = p[16+13] + 0.5 * x*(p[16+14] - p[16+12] + x*(2.0*p[16+12] - 5.0*p[16+13] + 4.0*p[16+14] - p[16+15] + x*(3.0*(p[16+13] - p[16+14]) + p[16+15] - p[16+12])));

    double b20 = p[32+1] + 0.5 * x*(p[32+2] - p[32+0] + x*(2.0*p[32+0] - 5.0*p[32+1] + 4.0*p[2+2] - p[2+3] + x*(3.0*(p[2+1] - p[2+2]) + p[2+3] - p[2+0])));
    double b21 = p[32+5] + 0.5 * x*(p[32+6] - p[32+4] + x*(2.0*p[32+4] - 5.0*p[32+5] + 4.0*p[2+6] - p[2+7] + x*(3.0*(p[2+5] - p[2+6]) + p[2+7] - p[2+4])));
    double b22 = p[32+9] + 0.5 * x*(p[32+10] - p[32+8] + x*(2.0*p[32+8] - 5.0*p[32+9] + 4.0*p[2+10] - p[2+11] + x*(3.0*(p[2+9] - p[2+10]) + p[2+11] - p[2+8])));
    double b23 = p[32+13] + 0.5 * x*(p[32+14] - p[32+12] + x*(2.0*p[32+12] - 5.0*p[32+13] + 4.0*p[2+14] - p[2+15] + x*(3.0*(p[2+13] - p[2+14]) + p[2+15] - p[2+12])));

    double b30 = p[48+1] + 0.5 * x*(p[48+2] - p[48+0+0] + x*(2.0*p[48+0] - 5.0*p[48+0+1] + 4.0*p[48+2] - p[48+3] + x*(3.0*(p[48+1] - p[48+2]) + p[48+3] - p[48+0])));
    double b31 = p[48+5] + 0.5 * x*(p[48+6] - p[48+1+0] + x*(2.0*p[48+4] - 5.0*p[48+1+1] + 4.0*p[48+6] - p[48+7] + x*(3.0*(p[48+5] - p[48+6]) + p[48+7] - p[48+4])));
    double b32 = p[48+9] + 0.5 * x*(p[48+10] - p[48+2+0] + x*(2.0*p[48+8] - 5.0*p[48+2+1] + 4.0*p[48+10] - p[48+11] + x*(3.0*(p[48+9] - p[48+10]) + p[48+11] - p[48+8])));
    double b33 = p[48+13] + 0.5 * x*(p[48+14] - p[48+3+0] + x*(2.0*p[48+12] - 5.0*p[48+3+1] + 4.0*p[48+14] - p[48+15] + x*(3.0*(p[48+13] - p[48+14]) + p[48+15] - p[48+12])));

    double c0 = b01 + 0.5 * x*(b02 - b00 + x*(2.0*b00 - 5.0*b01 + 4.0*b02 - b03 + x*(3.0*(b01 - b02) + b03 - b00)));
    double c1 = b11 + 0.5 * x*(b12 - b10 + x*(2.0*b10 - 5.0*b11 + 4.0*b12 - b13 + x*(3.0*(b11 - b12) + b13 - b10)));
    double c2 = b21 + 0.5 * x*(b22 - b20 + x*(2.0*b20 - 5.0*b21 + 4.0*b22 - b23 + x*(3.0*(b21 - b22) + b23 - b20)));
    double c3 = b31 + 0.5 * x*(b32 - b30 + x*(2.0*b30 - 5.0*b31 + 4.0*b32 - b33 + x*(3.0*(b31 - b32) + b33 - b30)));

    return c1 + 0.5 * x*(c2 - c0 + x*(2.0*c0 - 5.0*c1 + 4.0*c2 - c3 + x*(3.0*(c1 - c2) + c3 - c0)));
}

// vertices p are ordered {(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1), 
//                         (1, 0, 1), (0, 1, 1), (1, 1, 0), (1, 1, 1)}
// x, y, z, in range [0,1]
double Interpolation::trilinearInterpolate(double p[8], double x, double y, double z) {
    return p[0] * (1 - x) * (1 - y) * (1 - z) +
           p[1] * x * (1 - y) * (1 - z) + 
           p[2] * (1 - x) * y * (1 - z) + 
           p[3] * (1 - x) * (1 - y) * z +
           p[4] * x * (1 - y) * z + 
           p[5] * (1 - x) * y * z + 
           p[6] * x * y * (1 - z) + 
           p[7] * x * y * z;
}