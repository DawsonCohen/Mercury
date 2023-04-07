#ifndef __UTIL_H__
#define __UTIL_H__

#include <string>
#include "VoxelRobot.h"

namespace util {
    VoxelRobot ReadVoxelRobot(const char* filename);

    template<typename T>
    std::string SolutionToCSV(const T& h) {
        std::string s = h.DirectEncode();
        return s;
    }

    std::string FitnessHistoryToCSV(std::vector<std::tuple<ulong,float>>& h);

    std::string PopulationFitnessHistoryToCSV(std::vector<std::tuple<ulong, std::vector<float>, std::vector<float>>> h);

    std::string PopulationDiversityHistoryToCSV(std::vector<std::tuple<ulong, std::vector<float>, std::vector<float>>> h);

    template<typename... Ts>
    std::string DataToCSV(std::string header, std::vector<std::tuple<Ts...>> const& data)
    {
        std::ostringstream os;
        
        // Write header
        os << header << std::endl;
        
        // Write data
        for (auto const& row : data)
        {
            // Write each field of the row separated by commas
            bool first = true;
            ((os << (first ? first = false, "" : ","), os << std::get<Ts>(row)), ...);
            
            os << std::endl;
        }
        
        return os.str();
    }

    int WriteCSV(const char* filename, std::string datastring);

    void RemoveOldFiles(const char* dir);
}

#endif