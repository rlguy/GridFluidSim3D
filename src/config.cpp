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
#include "config.h"

namespace Config {
    std::string executableDirectory = CONFIG_EXECUTABLE_DIR;
    std::string outputDirectory     = CONFIG_OUTPUT_DIR;
    std::string bakefilesDirectory  = CONFIG_BAKEFILES_DIR;
    std::string logsDirectory       = CONFIG_LOGS_DIR;
    std::string savestatesDirectory = CONFIG_SAVESTATES_DIR;
    std::string tempDirectory       = CONFIG_TEMP_DIR;

    std::string getExecutableDirectory() {
        return executableDirectory;
    }

    std::string getOutputDirectory() {
        return outputDirectory;
    }

    std::string getBakefilesDirectory() {
        return bakefilesDirectory;
    }

    std::string getLogsDirectory() {
        return logsDirectory;
    }

    std::string getSavestatesDirectory() {
        return savestatesDirectory;
    }

    std::string getTempDirectory() {
        return tempDirectory;
    }

    void setOutputDirectory(std::string dir) {
        outputDirectory = dir;
    }

    void setBakefilesDirectory(std::string dir) {
        bakefilesDirectory = dir;
    }

    void setLogsDirectory(std::string dir) {
        logsDirectory = dir;
    }

    void setSavestatesDirectory(std::string dir) {
        savestatesDirectory = dir;
    }

    void setTempDirectory(std::string dir) {
        tempDirectory = dir;
    }

}