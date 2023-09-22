#ifndef __UTIL_H__
#define __UTIL_H__

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include "config.h"

namespace util {
    std::string FitnessHistoryToCSV(std::vector<std::tuple<ulong,float>>& h);

    std::string PopulationFitnessHistoryToCSV(std::vector<std::tuple<ulong, std::vector<float>, std::vector<float>>> h);

    std::string PopulationDiversityHistoryToCSV(std::vector<std::tuple<ulong, std::vector<float>, std::vector<float>>> h);

    template<typename... Ts>
    std::string DataToCSV(const std::string& header, const std::vector<std::tuple<Ts...>>& data);

    int MakeDirectory(const std::string& directory);

    int WriteCSV(const std::string& filename, const std::string& directory, const std::string& datastring);

    void RemoveOldFiles(const std::string& dir);

    Config ReadConfigFile(const std::string& filename);

    RobotType ReadRobotType(const std::string& filename);
}

#endif