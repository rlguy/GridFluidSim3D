#include "stopwatch.h"

StopWatch::StopWatch()
{
}

void StopWatch::start() {
    tbegin = (float)GetTickCount() / 1000.0;
    isStarted = true;
}


void StopWatch::stop() {
    if (!isStarted) {
        return;
    }

    // log current running time
    tend = (float)GetTickCount() / 1000.0;
    double time = tend - tbegin;
    timeRunning += time;
}

void StopWatch::reset() {
    isStarted = false;
    timeRunning = 0.0;
}

double StopWatch::getTime() {
    return timeRunning;
}