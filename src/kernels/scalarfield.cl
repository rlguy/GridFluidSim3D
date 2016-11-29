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