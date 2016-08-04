#include "cbindings.h"

#ifdef _WIN32
    #define EXPORTDLL __declspec(dllexport)
#else
    #define EXPORTDLL
#endif

namespace CBindings {

int SUCCESS = 1;
int FAIL = 0;
char CBINDINGS_ERROR_MESSAGE[4096];

void set_error_message(std::exception &ex) {
    std::string msg = ex.what();
    msg.copy(CBINDINGS_ERROR_MESSAGE, msg.length(), 0);
    CBINDINGS_ERROR_MESSAGE[msg.length()] = '\0';
}

char* get_error_message() {
	return CBINDINGS_ERROR_MESSAGE;
}

Vector3_t to_struct(vmath::vec3 v) {
	return Vector3_t{ v.x, v.y, v.z};
}

vmath::vec3 to_class(Vector3_t v) {
	return vmath::vec3(v.x, v.y, v.z);
}

AABB_t to_struct(AABB b) {
	Vector3_t cpos = to_struct(b.position);
    return AABB_t{ cpos,
                   (float)b.width, 
                   (float)b.height, 
                   (float)b.depth };
}

AABB to_class(AABB_t b) {
	return AABB(b.position.x, b.position.y, b.position.z,
                b.width, b.height, b.depth);
}

MarkerParticle_t to_struct(MarkerParticle p) {
    Vector3_t pos = to_struct(p.position);
    Vector3_t vel = to_struct(p.velocity);
    return MarkerParticle_t{ pos, vel };
}

MarkerParticle to_class(MarkerParticle_t p) {
    vmath::vec3 pos = to_class(p.position);
    vmath::vec3 vel = to_class(p.velocity);
    return MarkerParticle(pos, vel);
}

DiffuseParticle_t to_struct(DiffuseParticle p) {
    Vector3_t pos = to_struct(p.position);
    Vector3_t vel = to_struct(p.velocity);
    return DiffuseParticle_t{ pos, vel, p.lifetime, (char)p.type };
}

DiffuseParticle to_class(DiffuseParticle_t p) {
    vmath::vec3 pos = to_class(p.position);
    vmath::vec3 vel = to_class(p.velocity);
    
    DiffuseParticle dp(pos, vel, p.lifetime);
    dp.type = (DiffuseParticleType)p.type;
    return dp;
}


}

extern "C" {
	EXPORTDLL char* CBindings_get_error_message() {
        return CBindings::get_error_message();
    }
}