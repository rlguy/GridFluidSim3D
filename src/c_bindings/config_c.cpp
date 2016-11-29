#include "../config.h"

#include <cstring>

#ifdef _WIN32
    #define EXPORTDLL __declspec(dllexport)
#else
    #define EXPORTDLL
#endif

extern "C" {
    
    EXPORTDLL void Config_get_executable_directory(char *dir) {
        std::string dirstr = Config::getExecutableDirectory();
        memcpy(dir, &dirstr[0], dirstr.size());
        dir[dirstr.size()] = '\0';
    }

    EXPORTDLL void Config_get_output_directory(char *dir) {
        std::string dirstr = Config::getOutputDirectory();
        memcpy(dir, &dirstr[0], dirstr.size());
        dir[dirstr.size()] = '\0';
    }

    EXPORTDLL void Config_get_bakefiles_directory(char *dir) {
        std::string dirstr = Config::getBakefilesDirectory();
        memcpy(dir, &dirstr[0], dirstr.size());
        dir[dirstr.size()] = '\0';
    }

    EXPORTDLL void Config_get_logs_directory(char *dir) {
        std::string dirstr = Config::getLogsDirectory();
        memcpy(dir, &dirstr[0], dirstr.size());
        dir[dirstr.size()] = '\0';
    }

    EXPORTDLL void Config_get_savestates_directory(char *dir) {
        std::string dirstr = Config::getSavestatesDirectory();
        memcpy(dir, &dirstr[0], dirstr.size());
        dir[dirstr.size()] = '\0';
    }

    EXPORTDLL void Config_get_temp_directory(char *dir) {
        std::string dirstr = Config::getTempDirectory();
        memcpy(dir, &dirstr[0], dirstr.size());
        dir[dirstr.size()] = '\0';
    }

    EXPORTDLL void Config_set_output_directory(char *dir) {
        Config::setOutputDirectory(std::string(dir));
    }

    EXPORTDLL void Config_set_bakefiles_directory(char *dir) {
        Config::setBakefilesDirectory(std::string(dir));
    }

    EXPORTDLL void Config_set_logs_directory(char *dir) {
        Config::setLogsDirectory(std::string(dir));
    }

    EXPORTDLL void Config_set_savestates_directory(char *dir) {
        Config::setSavestatesDirectory(std::string(dir));
    }

    EXPORTDLL void Config_set_temp_directory(char *dir) {
        Config::setTempDirectory(std::string(dir));
    }
    
}
