#include <stdio.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>

#include "stopwatch.h"

#pragma once
class LogFile
{
public:
    LogFile();
    LogFile(std::string filename);
    LogFile(std::string filename, std::string extension);
    ~LogFile();

    LogFile(LogFile &obj)
    {  
    }

    LogFile operator=(LogFile & rhs)
    {
        return *this;
    }

    void write();
    void setPath(std::string path);
    void setSeparator(std::string separator);
    void enableConsole();
    void disableConsole();
    std::string getString();
    void clear();
    void newline();
    void separator();
    void timestamp();
    void log(std::string str, int indentLevel = 0);
    void log(std::string str, int value, int indentLevel = 0);
    void log(std::string str, double value, int precision = 0, int indentLevel = 0);
    void log(std::string str, std::string value = "", int indentLevel = 0);
    std::string getTime();
    std::string getSrartTimeString() { return _startTimeString; }

private:
    void _print(std::string str);

    std::string _path;
    std::string _filename;
    std::string _startTimeString;
    std::string _separator;
    std::ostringstream _stream;
    bool _isWritingToConsole = true;
};

