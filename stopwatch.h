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
    bool isStarted = false;
    float tbegin, tend;
    double timeRunning = 0.0;
};