#ifndef AABB_T_H
#define AABB_T_H

#include "vector3_c.h"

typedef struct AABB_t {
	Vector3_t position;
	float width;
	float height;
	float depth;
} AABB_t;

#endif