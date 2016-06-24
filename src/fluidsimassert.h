#ifndef NFLUIDSIMDEBUG
    #include <cassert>
    #include <cstdlib>
    #define FLUIDSIM_ASSERT(condition)\
    {\
        if (!(condition))\
        {\
            std::cerr << "Assertion failed: " << #condition <<\
                         ", file " << __FILE__ <<\
                         ", function " << __FUNCTION__ <<\
                         ", line " << __LINE__ << std::endl;\
            abort();\
        }\
    }
#else
    #define FLUIDSIM_ASSERT(condition) (condition)
#endif
