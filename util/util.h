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
    std::string DataToCSV(std::string header, std::vector<std::tuple<Ts...>> const& h)
    {
        std::string s = header + "\n";

        for(auto const& tup : h) {
            std::apply
            (
                [&s](Ts const&... tupleArgs)
                {
                    std::size_t n{0};
                    ((s += std::to_string(tupleArgs) + (++n != sizeof...(Ts) ? ", " : "")), ...);
                }, tup
            );
            s += "\n";
        }
        return s;
    }

    int WriteCSV(const char* filename, std::string datastring);

    void RemoveOldFiles(const char* dir);
}

#endif