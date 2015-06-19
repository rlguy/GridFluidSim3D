#pragma once

#include <Windows.h>
#include <Winbase.h>

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
    float _tbegin, _tend;
    double _timeRunning = 0.0;
};