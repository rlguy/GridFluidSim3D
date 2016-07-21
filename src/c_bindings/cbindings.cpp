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
	Vector3_t cpos = Vector3_t{ b.position.x, 
                                b.position.y, 
                                b.position.z };
    return AABB_t{ cpos,
                   (float)b.width, 
                   (float)b.height, 
                   (float)b.depth };
}

AABB to_class(AABB_t b) {
	return AABB(b.position.x, b.position.y, b.position.z,
                b.width, b.height, b.depth);
}

}

extern "C" {
	EXPORTDLL char* CBindings_get_error_message() {
        return CBindings::get_error_message();
    }
}