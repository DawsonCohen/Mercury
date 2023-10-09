#ifndef __OPTIMIZER_UTIL_H__
#define __OPTIMIZER_UTIL_H__

#include "util.h"
#include "optimizer_config.h"
#include "util.h"

namespace util {
    std::string FitnessHistoryToCSV(std::vector<std::tuple<ulong,float>>& h);

    std::string PopulationFitnessHistoryToCSV(std::vector<std::tuple<ulong, std::vector<float>, std::vector<float>>> h);

    std::string PopulationDiversityHistoryToCSV(std::vector<std::tuple<ulong, std::vector<float>, std::vector<float>>> h);

    namespace optimizer {
        OptimizerConfig ReadConfigFile(const std::string& filename);
    }
    
}

#endif