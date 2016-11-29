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

#include "kernels.h"

namespace Kernels {

std::string tricubicinterpolateCL = R"CLC(
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

#define CHUNK_WIDTH 5
#define CHUNK_HEIGHT 5
#define CHUNK_DEPTH 5

#define U_OFFSET 0

// V_OFFSET + (CHUNK_WIDTH+4)*(CHUNK_HEIGHT+3)*(CHUNK_DEPTH+4)
#define V_OFFSET 648 

// W_OFFSET + (CHUNK_WIDTH+4)*(CHUNK_HEIGHT+4)*(CHUNK_DEPTH+3)
#define W_OFFSET 1296

#define VFIELD_SIZE 1944

// VFIELD_SIZE / 4
#define MAX_VFIELD_LOAD_LOCAL_ID 486

/* 
    Cubic interpolation methods from http://www.paulinternet.nl/?page=bicubic

    - tricubic_interpolate function will interpolate the volume 
      between point p[1][1][1] and p[2][2][2]
    - p is indexed in order by p[k][j][i]
    - x, y, z are in [0,1]
*/
float cubic_interpolate(float p[4], float x) {
    return p[1] + 0.5 * x*(p[2] - p[0] + x*(2.0*p[0] - 5.0*p[1] + 4.0*p[2] - p[3] + x*(3.0*(p[1] - p[2]) + p[3] - p[0])));
}

float bicubic_interpolate(float p[4][4], float x, float y) {
    float arr[4];
    arr[0] = cubic_interpolate(p[0], x);
    arr[1] = cubic_interpolate(p[1], x);
    arr[2] = cubic_interpolate(p[2], x);
    arr[3] = cubic_interpolate(p[3], x);
    return cubic_interpolate(arr, y);
}

float tricubic_interpolate(float p[4][4][4], float x, float y, float z) {
    float arr[4];
    arr[0] = bicubic_interpolate(p[0], x, y);
    arr[1] = bicubic_interpolate(p[1], x, y);
    arr[2] = bicubic_interpolate(p[2], x, y);
    arr[3] = bicubic_interpolate(p[3], x, y);
    return cubic_interpolate(arr, z);
}

int flatten_index(int i, int j, int k, int isize, int jsize) {
    return i + isize * (j + jsize * k);
}

void fill_interpolation_data(__local float *vfield, 
                             int3 voffset, int vwidth, int vheight,
                             float points[4][4][4]) {

    /*
    for (int k = 0; k < 4; k++) {
        for (int j = 0; j < 4; j++) {
            for (int i = 0; i < 4; i++) {
                int flatidx = flatten_index(voffset.x + i,
                                            voffset.y + j,
                                            voffset.z + k,
                                            vwidth, vheight);
                points[k][j][i] = vfield[flatidx];
            }
        }
    }
    */
    
    // The following section optimizes the above loop.
    int flatidx;
    flatidx = flatten_index(voffset.x, voffset.y + 0, voffset.z + 0, vwidth, vheight);
    points[0][0][0] = vfield[flatidx + 0];
    points[0][0][1] = vfield[flatidx + 1];
    points[0][0][2] = vfield[flatidx + 2];
    points[0][0][3] = vfield[flatidx + 3];

    flatidx = flatten_index(voffset.x, voffset.y + 1, voffset.z + 0, vwidth, vheight);
    points[0][1][0] = vfield[flatidx + 0];
    points[0][1][1] = vfield[flatidx + 1];
    points[0][1][2] = vfield[flatidx + 2];
    points[0][1][3] = vfield[flatidx + 3];

    flatidx = flatten_index(voffset.x, voffset.y + 2, voffset.z + 0, vwidth, vheight);
    points[0][2][0] = vfield[flatidx + 0];
    points[0][2][1] = vfield[flatidx + 1];
    points[0][2][2] = vfield[flatidx + 2];
    points[0][2][3] = vfield[flatidx + 3];

    flatidx = flatten_index(voffset.x, voffset.y + 3, voffset.z + 0, vwidth, vheight);
    points[0][3][0] = vfield[flatidx + 0];
    points[0][3][1] = vfield[flatidx + 1];
    points[0][3][2] = vfield[flatidx + 2];
    points[0][3][3] = vfield[flatidx + 3];

    flatidx = flatten_index(voffset.x, voffset.y + 0, voffset.z + 1, vwidth, vheight);
    points[1][0][0] = vfield[flatidx + 0];
    points[1][0][1] = vfield[flatidx + 1];
    points[1][0][2] = vfield[flatidx + 2];
    points[1][0][3] = vfield[flatidx + 3];

    flatidx = flatten_index(voffset.x, voffset.y + 1, voffset.z + 1, vwidth, vheight);
    points[1][1][0] = vfield[flatidx + 0];
    points[1][1][1] = vfield[flatidx + 1];
    points[1][1][2] = vfield[flatidx + 2];
    points[1][1][3] = vfield[flatidx + 3];

    flatidx = flatten_index(voffset.x, voffset.y + 2, voffset.z + 1, vwidth, vheight);
    points[1][2][0] = vfield[flatidx + 0];
    points[1][2][1] = vfield[flatidx + 1];
    points[1][2][2] = vfield[flatidx + 2];
    points[1][2][3] = vfield[flatidx + 3];

    flatidx = flatten_index(voffset.x, voffset.y + 3, voffset.z + 1, vwidth, vheight);
    points[1][3][0] = vfield[flatidx + 0];
    points[1][3][1] = vfield[flatidx + 1];
    points[1][3][2] = vfield[flatidx + 2];
    points[1][3][3] = vfield[flatidx + 3];

    flatidx = flatten_index(voffset.x, voffset.y + 0, voffset.z + 2, vwidth, vheight);
    points[2][0][0] = vfield[flatidx + 0];
    points[2][0][1] = vfield[flatidx + 1];
    points[2][0][2] = vfield[flatidx + 2];
    points[2][0][3] = vfield[flatidx + 3];

    flatidx = flatten_index(voffset.x, voffset.y + 1, voffset.z + 2, vwidth, vheight);
    points[2][1][0] = vfield[flatidx + 0];
    points[2][1][1] = vfield[flatidx + 1];
    points[2][1][2] = vfield[flatidx + 2];
    points[2][1][3] = vfield[flatidx + 3];

    flatidx = flatten_index(voffset.x, voffset.y + 2, voffset.z + 2, vwidth, vheight);
    points[2][2][0] = vfield[flatidx + 0];
    points[2][2][1] = vfield[flatidx + 1];
    points[2][2][2] = vfield[flatidx + 2];
    points[2][2][3] = vfield[flatidx + 3];

    flatidx = flatten_index(voffset.x, voffset.y + 3, voffset.z + 2, vwidth, vheight);
    points[2][3][0] = vfield[flatidx + 0];
    points[2][3][1] = vfield[flatidx + 1];
    points[2][3][2] = vfield[flatidx + 2];
    points[2][3][3] = vfield[flatidx + 3];

    flatidx = flatten_index(voffset.x, voffset.y + 0, voffset.z + 3, vwidth, vheight);
    points[3][0][0] = vfield[flatidx + 0];
    points[3][0][1] = vfield[flatidx + 1];
    points[3][0][2] = vfield[flatidx + 2];
    points[3][0][3] = vfield[flatidx + 3];

    flatidx = flatten_index(voffset.x, voffset.y + 1, voffset.z + 3, vwidth, vheight);
    points[3][1][0] = vfield[flatidx + 0];
    points[3][1][1] = vfield[flatidx + 1];
    points[3][1][2] = vfield[flatidx + 2];
    points[3][1][3] = vfield[flatidx + 3];

    flatidx = flatten_index(voffset.x, voffset.y + 2, voffset.z + 3, vwidth, vheight);
    points[3][2][0] = vfield[flatidx + 0];
    points[3][2][1] = vfield[flatidx + 1];
    points[3][2][2] = vfield[flatidx + 2];
    points[3][2][3] = vfield[flatidx + 3];

    flatidx = flatten_index(voffset.x, voffset.y + 3, voffset.z + 3, vwidth, vheight);
    points[3][3][0] = vfield[flatidx + 0];
    points[3][3][1] = vfield[flatidx + 1];
    points[3][3][2] = vfield[flatidx + 2];
    points[3][3][3] = vfield[flatidx + 3];
}

float interpolate_U(float3 pos, float dx, float invdx, __local float *ufield) {

    pos.y -= 0.5*dx;
    pos.z -= 0.5*dx;

    int3 index = (int3)(floor(pos.x * invdx),
                        floor(pos.y * invdx),
                        floor(pos.z * invdx));

    float3 index_offset = (float3)(index.x * dx,
                                   index.y * dx,
                                   index.z * dx);

    float3 interp_pos = invdx * (pos - index_offset);

    int3 vfield_index_offset = (int3)(index.x - 1 + 1,
                                      index.y - 1 + 2,
                                      index.z - 1 + 2);

    float points[4][4][4];
    int vwidth = CHUNK_WIDTH + 3;
    int vheight = CHUNK_HEIGHT + 4;

    fill_interpolation_data(ufield, vfield_index_offset, vwidth, vheight, points);

    return tricubic_interpolate(points, interp_pos.x,
                                        interp_pos.y,
                                        interp_pos.z);
}

float interpolate_V(float3 pos, float dx, float invdx, __local float *vfield) {

    pos.x -= 0.5*dx;
    pos.z -= 0.5*dx;

    int3 index = (int3)(floor(pos.x * invdx),
                        floor(pos.y * invdx),
                        floor(pos.z * invdx));

    float3 index_offset = (float3)(index.x * dx,
                                   index.y * dx,
                                   index.z * dx);

    float3 interp_pos = invdx * (pos - index_offset);

    int3 vfield_index_offset = (int3)(index.x - 1 + 2,
                                      index.y - 1 + 1,
                                      index.z - 1 + 2);

    float points[4][4][4];
    int vwidth = CHUNK_WIDTH + 4;
    int vheight = CHUNK_HEIGHT + 3;

    fill_interpolation_data(vfield, vfield_index_offset, vwidth, vheight, points);

    return tricubic_interpolate(points, interp_pos.x,
                                        interp_pos.y,
                                        interp_pos.z);
}

float interpolate_W(float3 pos, float dx, float invdx, __local float *wfield) {

    pos.x -= 0.5*dx;
    pos.y -= 0.5*dx;

    int3 index = (int3)(floor(pos.x * invdx),
                        floor(pos.y * invdx),
                        floor(pos.z * invdx));

    float3 index_offset = (float3)(index.x * dx,
                                   index.y * dx,
                                   index.z * dx);

    float3 interp_pos = invdx * (pos - index_offset);

    int3 vfield_index_offset = (int3)(index.x - 1 + 2,
                                      index.y - 1 + 2,
                                      index.z - 1 + 1);

    float points[4][4][4];
    int vwidth = CHUNK_WIDTH + 4;
    int vheight = CHUNK_HEIGHT + 4;

    fill_interpolation_data(wfield, vfield_index_offset, vwidth, vheight, points);

    return tricubic_interpolate(points, interp_pos.x,
                                        interp_pos.y,
                                        interp_pos.z);
}

__kernel void tricubic_interpolate_kernel(__global float *particles,
                                          __global float *vfield_data,
                                          __global int *chunk_offsets,
                                          __local  float *vfield,
                                          float dx) {

    size_t tid = get_global_id(0);
	size_t lid = get_local_id(0);
	size_t gid = tid / get_local_size(0);

    // Load fvield_data into local memory
    if (lid < MAX_VFIELD_LOAD_LOCAL_ID) {
        int local_offset = 4*lid;
        int vfield_data_offset = gid * VFIELD_SIZE + local_offset;

        vfield[local_offset + 0] = vfield_data[vfield_data_offset + 0];
        vfield[local_offset + 1] = vfield_data[vfield_data_offset + 1];
        vfield[local_offset + 2] = vfield_data[vfield_data_offset + 2];
        vfield[local_offset + 3] = vfield_data[vfield_data_offset + 3];
    }

    barrier(CLK_LOCAL_MEM_FENCE);
    
    float3 pos = (float3)(particles[3*tid + 0], 
                          particles[3*tid + 1], 
                          particles[3*tid + 2]);

    int3 chunk_offset = (int3)(chunk_offsets[3*gid + 0],
                               chunk_offsets[3*gid + 1],
                               chunk_offsets[3*gid + 2]);

    int3 index_offset = (int3)(chunk_offset.x * CHUNK_WIDTH,
                               chunk_offset.y * CHUNK_HEIGHT,
                               chunk_offset.z * CHUNK_DEPTH);

    float3 pos_offset = (float3)(index_offset.x * dx,
                                 index_offset.y * dx,
                                 index_offset.z * dx);

    float3 local_pos = pos - pos_offset;

    float invdx = 1.0 / dx;
    float result1 = interpolate_U(local_pos, dx, invdx, &(vfield[U_OFFSET]));
    float result2 = interpolate_V(local_pos, dx, invdx, &(vfield[V_OFFSET]));
    float result3 = interpolate_W(local_pos, dx, invdx, &(vfield[W_OFFSET]));

	particles[3*tid] = result1;
	particles[3*tid + 1] = result2;
	particles[3*tid + 2] = result3;
}
)CLC";

std::string scalarfieldCL = R"CLC(
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

struct KernelCoefficients {
    float coef1, coef2, coef3;
};

void load_local_memory(int chunk_width,
                       int num_values,
                       __global float *global_data, 
                       __local  float *local_data) {

    size_t lid = get_local_id(0);
    size_t gid = get_global_id(0) / get_local_size(0);

    int local_size = get_local_size(0);
    int num_read = ceil((float)num_values / (float)local_size);

    int max_read_local_id = floor((float)num_values / (float)num_read) - 1;
    int num_remainder = num_values - (max_read_local_id + 1) * num_read;

    if (lid <= max_read_local_id) {
        int local_offset = num_read * lid;
        int global_offset = gid * num_values + local_offset;

        for (int i = 0; i < num_read; i++) {
            local_data[local_offset + i] = global_data[global_offset + i];
        }

        if (lid == max_read_local_id) {
            local_offset = local_offset + num_read;
            global_offset = global_offset + num_read;
            for (int j = 0; j < num_remainder; j++) {
                local_data[local_offset + j] = global_data[global_offset + j]; 
            }  
        }
    }

    barrier(CLK_LOCAL_MEM_FENCE);
}

int3 flat_to_3d_index(int flatidx, int isize, int jsize) {
    int i = flatidx % isize;
    int j = (flatidx / isize) % jsize;
    int k = flatidx / (jsize * isize); 
    return (int3)(i, j, k);
}

struct KernelCoefficients calculate_kernel_coefficients(float r) {
    struct KernelCoefficients coefs;
    coefs.coef1 = (4.0f / 9.0f) * (1.0f / (r*r*r*r*r*r));
    coefs.coef2 = (17.0f / 9.0f) * (1.0f / (r*r*r*r));
    coefs.coef3 = (22.0f / 9.0f) * (1.0f / (r*r));

    return coefs;
}

float evaluate_kernel(float rsq, struct KernelCoefficients *coefs) {
    return 1.0 - (*coefs).coef1*rsq*rsq*rsq + (*coefs).coef2*rsq*rsq - (*coefs).coef3*rsq;
}

__kernel void compute_scalar_field_points(__global float *particles,
                                          __global float *field_data,
                                          __global int *chunk_offsets,
                                          __local  float *local_particles,
                                          int num_particles,
                                          int num_groups,
                                          float radius,
                                          float dx) {
    size_t tid = get_global_id(0);
    size_t lid = get_local_id(0);
    size_t gid = tid / get_local_size(0);

    int local_size = get_local_size(0);
    int chunk_width = (int)floor(cbrt((float)local_size));
    int num_cells = chunk_width * chunk_width * chunk_width;

    int local_data_size = 3 * num_particles;
    load_local_memory(chunk_width, local_data_size, particles, local_particles);

    if (lid >= num_cells) {
        return;
    }

    int3 cell_index = flat_to_3d_index(lid, chunk_width, chunk_width);
    float3 cell_center = (float3)(((float)cell_index.x + 0.5f) * dx,
                                  ((float)cell_index.y + 0.5f) * dx,
                                  ((float)cell_index.z + 0.5f) * dx);

    int3 chunk_index = (int3)(chunk_offsets[3*gid + 0],
                              chunk_offsets[3*gid + 1],
                              chunk_offsets[3*gid + 2]);
    float3 position_offset = (float3)(chunk_index.x * chunk_width * dx,
                                      chunk_index.y * chunk_width * dx,
                                      chunk_index.z * chunk_width * dx);

    struct KernelCoefficients coefs = calculate_kernel_coefficients(radius);

    float sum = 0.0f;
    float3 p;
    float rsq;
    float maxrsq = radius * radius;
    float k;
    float3 v;
    for (int i = 0; i < local_data_size; i += 3) {
        p.x = local_particles[i + 0];
        p.y = local_particles[i + 1];
        p.z = local_particles[i + 2];

        p -= position_offset;
        v = p - cell_center;
        rsq = v.x*v.x + v.y*v.y + v.z*v.z;

        if (rsq < maxrsq) {
            sum += evaluate_kernel(rsq, &coefs);
        }
    }

    int fieldidx = gid * num_cells + lid;
    field_data[fieldidx] = sum;
}

__kernel void compute_scalar_field_point_values(__global float *point_values,
                                                __global float *field_data,
                                                __global int *chunk_offsets,
                                                __local  float *local_point_values,
                                                int num_points,
                                                int num_groups,
                                                float radius,
                                                float dx) {
    size_t tid = get_global_id(0);
    size_t lid = get_local_id(0);
    size_t gid = tid / get_local_size(0);

    int local_size = get_local_size(0);
    int chunk_width = (int)floor(cbrt((float)local_size));
    int num_cells = chunk_width * chunk_width * chunk_width;

    int local_data_size = 4 * num_points;
    load_local_memory(chunk_width, local_data_size, point_values, local_point_values);

    if (lid >= num_cells) {
        return;
    }

    int3 cell_index = flat_to_3d_index(lid, chunk_width, chunk_width);
    float3 cell_center = (float3)(((float)cell_index.x + 0.5f) * dx,
                                  ((float)cell_index.y + 0.5f) * dx,
                                  ((float)cell_index.z + 0.5f) * dx);

    int3 chunk_index = (int3)(chunk_offsets[3*gid + 0],
                              chunk_offsets[3*gid + 1],
                              chunk_offsets[3*gid + 2]);
    float3 position_offset = (float3)(chunk_index.x * chunk_width * dx,
                                      chunk_index.y * chunk_width * dx,
                                      chunk_index.z * chunk_width * dx);

    struct KernelCoefficients coefs = calculate_kernel_coefficients(radius);

    float sum = 0.0f;
    float3 p;
    float rsq;
    float maxrsq = radius * radius;
    float value;
    float3 vect;
    for (int i = 0; i < local_data_size; i += 4) {
        p.x   = local_point_values[i + 0];
        p.y   = local_point_values[i + 1];
        p.z   = local_point_values[i + 2];
        value = local_point_values[i + 3];

        p -= position_offset;
        vect = p - cell_center;
        rsq = vect.x*vect.x + vect.y*vect.y + vect.z*vect.z;

        if (rsq < maxrsq) {
            sum += value * evaluate_kernel(rsq, &coefs);
        }
    }

    int fieldidx = gid * num_cells + lid;
    field_data[fieldidx] = sum;
}

__kernel void compute_scalar_weight_field_point_values(__global float *point_values,
                                                       __global float *field_data,
                                                       __global int *chunk_offsets,
                                                       __local  float *local_point_values,
                                                       int num_points,
                                                       int num_groups,
                                                       float radius,
                                                       float dx) {
    size_t tid = get_global_id(0);
    size_t lid = get_local_id(0);
    size_t gid = tid / get_local_size(0);

    int local_size = get_local_size(0);
    int chunk_width = (int)floor(cbrt((float)local_size));
    int num_cells = chunk_width * chunk_width * chunk_width;

    int local_data_size = 4 * num_points;
    load_local_memory(chunk_width, local_data_size, point_values, local_point_values);

    if (lid >= num_cells) {
        return;
    }

    int3 cell_index = flat_to_3d_index(lid, chunk_width, chunk_width);
    float3 cell_center = (float3)(((float)cell_index.x + 0.5f) * dx,
                                  ((float)cell_index.y + 0.5f) * dx,
                                  ((float)cell_index.z + 0.5f) * dx);

    int3 chunk_index = (int3)(chunk_offsets[3*gid + 0],
                              chunk_offsets[3*gid + 1],
                              chunk_offsets[3*gid + 2]);
    float3 position_offset = (float3)(chunk_index.x * chunk_width * dx,
                                      chunk_index.y * chunk_width * dx,
                                      chunk_index.z * chunk_width * dx);

    struct KernelCoefficients coefs = calculate_kernel_coefficients(radius);

    float scalarsum = 0.0f;
    float weightsum = 0.0f;
    float3 p;
    float rsq;
    float maxrsq = radius * radius;
    float value;
    float weight;
    float3 vect;
    for (int i = 0; i < local_data_size; i += 4) {
        p.x   = local_point_values[i + 0];
        p.y   = local_point_values[i + 1];
        p.z   = local_point_values[i + 2];
        value = local_point_values[i + 3];

        p -= position_offset;
        vect = p - cell_center;
        rsq = vect.x*vect.x + vect.y*vect.y + vect.z*vect.z;

        if (rsq < maxrsq) {
            weight = evaluate_kernel(rsq, &coefs);
            scalarsum += value * weight;
            weightsum += weight;
        }
    }

    int scalarfieldidx = gid * num_cells + lid;
    int weightfieldidx = num_groups * num_cells + scalarfieldidx;
    field_data[scalarfieldidx] = scalarsum;
    field_data[weightfieldidx] = weightsum;
}
)CLC";

}
