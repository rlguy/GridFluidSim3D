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

#ifndef LOGFILE_H
#define LOGFILE_H

#include <stdio.h>
#include <iostream>
#include <math.h>
#include <sstream>
#include <fstream>
#include <time.h>

#include "stopwatch.h"

class LogFile
{
public:
    LogFile();
    LogFile(std::string filename);
    LogFile(std::string filename, std::string extension);
    ~LogFile();

    LogFile(LogFile &obj) {  
        _path = obj._path;
        _filename = obj._filename;
        _startTimeString = obj._startTimeString;
        _separator = obj._separator;
    }

    LogFile operator=(LogFile &rhs)
    {
        _path = rhs._path;
        _filename = rhs._filename;
        _startTimeString = rhs._startTimeString;
        _separator = rhs._separator;
        
        return *this;
    }

    void setPath(std::string path);
    void setSeparator(std::string separator);
    void enableConsole();
    void disableConsole();
    std::string getString();
    void clear();
    void newline();
    void separator();
    void timestamp();
    void log(std::ostream &out);
    void log(std::string str, int indentLevel = 0);
    void log(std::string str, int value, int indentLevel = 0);
    void log(std::string str, double value, int precision = 0, int indentLevel = 0);
    void log(std::string str, std::string value = "", int indentLevel = 0);
    std::string getTime();
    std::string getSrartTimeString() { return _startTimeString; }
    void print(std::string str);

private:
    void _print(std::string str);
    void _write();

    std::string _path;
    std::string _filename;
    std::string _startTimeString;
    std::string _separator;
    std::ostringstream _stream;
    bool _isWritingToConsole = true;
};

#endif