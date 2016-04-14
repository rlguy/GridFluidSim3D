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
#ifndef DIFFUSEPARTICLE_H
#define DIFFUSEPARTICLE_H

#include "vmath.h"

enum class DiffuseParticleType : char { 
    bubble = 0x00, 
    foam = 0x01, 
    spray = 0x02,
    notset = 0x03
};

struct DiffuseParticle {
    vmath::vec3 position;
    vmath::vec3 velocity;
    float lifetime;
    DiffuseParticleType type;

    DiffuseParticle() : lifetime(0.0),
                        type(DiffuseParticleType::notset) {}

    DiffuseParticle(vmath::vec3 p, vmath::vec3 v, float time) : 
                        position(p),
                        velocity(v),
                        lifetime(time),
                        type(DiffuseParticleType::notset) {}
};

#endif
