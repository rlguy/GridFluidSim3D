#include "stopwatch.h"

StopWatch::StopWatch()
{
}

void StopWatch::start() {
    _tbegin = (float)GetTickCount() / 1000.0;
    _isStarted = true;
}


void StopWatch::stop() {
    if (!_isStarted) {
        return;
    }

    // log current running time
    _tend = (float)GetTickCount() / 1000.0;
    double time = _tend - _tbegin;
    _timeRunning += time;
}

void StopWatch::reset() {
    _isStarted = false;
    _timeRunning = 0.0;
}

double StopWatch::getTime() {
    return _timeRunning;
}