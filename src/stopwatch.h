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
#ifndef STOPWATCH_H
#define STOPWATCH_H

#if defined(__linux__) || defined(__APPLE__) || defined(__MACOSX)
    #include <sys/time.h>
#elif defined(_WIN32)
    #include <Windows.h>
    #include <Winbase.h>
#else
#endif

class StopWatch
{
public:
    StopWatch();
    void start();
    void stop();
    void reset();
    double getTime();    // in seconds

private:
    bool _isStarted = false;
    double _tbegin, _tend;
    double _timeRunning = 0.0;
};

#endif