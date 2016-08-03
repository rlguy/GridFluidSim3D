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
    std::string getExecutableDirectory() {
        return std::string(CONFIG_EXECUTABLE_DIR);
    }

    std::string getResourcesDirectory() {
        return std::string(CONFIG_RESOURCES_DIR);
    }

    std::string getOutputDirectory() {
        return std::string(CONFIG_OUTPUT_DIR);
    }

    std::string getBakefilesDirectory() {
        return std::string(CONFIG_BAKEFILES_DIR);
    }

    std::string getLogsDirectory() {
        return std::string(CONFIG_LOGS_DIR);
    }

    std::string getSavestatesDirectory() {
        return std::string(CONFIG_SAVESTATES_DIR);
    }

    std::string getTempDirectory() {
        return std::string(CONFIG_TEMP_DIR);
    }
}