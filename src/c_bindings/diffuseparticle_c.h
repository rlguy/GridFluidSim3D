#ifndef DIFFUSEPARTICLE_T_H
#define DIFFUSEPARTICLE_T_H

#include "vector3_c.h"

typedef struct DiffuseParticle_t {
	Vector3_t position;
	Vector3_t velocity;
	float lifetime;
	char type;
} DiffuseParticle_t;

#endif