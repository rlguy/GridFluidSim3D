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
#include "stopwatch.h"

StopWatch::StopWatch()
{
}

void StopWatch::start() {
    
    #if defined(__linux__) || defined(__APPLE__) || defined(__MACOSX)
        struct timeval tp;
        gettimeofday(&tp, nullptr);
        _tbegin = (double)tp.tv_sec + (double)tp.tv_usec / 1000000.0;
    #elif defined(_WIN32)
        _tbegin = (double)GetTickCount() / 1000.0;
    #else
    #endif
    
    _isStarted = true;
}


void StopWatch::stop() {
    if (!_isStarted) {
        return;
    }

    #if defined(__linux__) || defined(__APPLE__) || defined(__MACOSX)
        struct timeval tp;
        gettimeofday(&tp, nullptr);
        _tend = (double)tp.tv_sec + (double)tp.tv_usec / 1000000.0;
    #elif defined(_WIN32)
        _tend = (double)GetTickCount() / 1000.0;
    #else
    #endif
    
    double time = _tend - _tbegin;
    _timeRunning += time;
}

void StopWatch::reset() {
    _isStarted = false;
    _timeRunning = 0.0;
}

double StopWatch::getTime() {
    return _timeRunning >= 0.0 ? _timeRunning : 0.0;
}