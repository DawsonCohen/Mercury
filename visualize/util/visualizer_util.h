#ifndef __VISUALIZER_UTIL_H__
#define __VISUALIZER_UTIL_H__

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include "visualizer_config.h"
#include "optimizer_config.h"
#include "optimizer_util.h"
#include "util.h"

namespace util {
    namespace visualizer {
        VisualizerConfig ReadConfigFile(const std::string& filename);
    }

    namespace optimizer {
        OptimizerConfig ReadConfigFile(const std::string& filename);
    }
}

#endif