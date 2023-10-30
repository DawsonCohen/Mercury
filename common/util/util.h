#ifndef __UTIL_H__
#define __UTIL_H__

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include "config.h"

namespace util {
    template<typename... Ts>
    std::string DataToCSV(const std::string& header, const std::vector<std::tuple<Ts...>>& data);

    int MakeDirectory(const std::string& directory);

    int WriteCSV(const std::string& filename, const std::string& directory, const std::string& datastring, bool append = false);

    void RemoveOldFiles(const std::string& dir);

    RobotType ReadRobotType(const std::string& filename);

    namespace common {
        Config ReadConfigFile(const std::string& filename);
    }
}

#endif