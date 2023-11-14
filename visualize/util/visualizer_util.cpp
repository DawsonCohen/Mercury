#include <fstream>
#include <iostream>
#include <dirent.h>
#include <unistd.h>
#include <chrono>
#include <ctime>
#include <sys/stat.h>
#include <cstring>
#include <memory>
#include <unordered_map>
#include "visualizer_util.h"

namespace util {

VisualizerConfig visualizer::ReadConfigFile(const std::string& filename) {
    std::unordered_map<std::string, std::string> config_map;

    VisualizerConfig config = util::common::ReadConfigFile(filename);

    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Error opening config file: " << filename << std::endl;
        return config;
    }

    std::string line;
    while (std::getline(infile, line)) {
        // Ignore comments and empty lines
        if (line.empty() || line[0] == '#') {
            continue;
        }

        // Split line into key-value pair
        std::size_t pos = line.find('=');
        if (pos == std::string::npos) {
            std::cerr << "Error parsing config file: " << filename << std::endl;
            return config;
        }

        std::string key = line.substr(0, pos);
        std::string value = line.substr(pos+1);

        std::cout << key << ": " << value << std::endl;

        config_map[key] = value;
    }
    
    if(config_map.find("INTERACTIVE") != config_map.end()) {
        if(isdigit(config_map["INTERACTIVE"][0]) && stoi(config_map["INTERACTIVE"]))
            config.visualizer.interactive = true;
    }

    if(config_map.find("WRITE_VIDEO") != config_map.end()) {
        if(isdigit(config_map["WRITE_VIDEO"][0]) && stoi(config_map["WRITE_VIDEO"]))
            config.objectives.video = true;
    }

    if(config_map.find("VERIFY") != config_map.end()) {
        if(isdigit(config_map["VERIFY"][0]) && stoi(config_map["VERIFY"]))
            config.objectives.verify = true;
    }
    
    if(config_map.find("ZOO") != config_map.end()) {
        if(isdigit(config_map["ZOO"][0]) && stoi(config_map["ZOO"]))
            config.objectives.zoo = true;
    }

    
    if(config_map.find("STATIONARY") != config_map.end()) {
        if(isdigit(config_map["STATIONARY"][0]) && stoi(config_map["STATIONARY"]))
            config.objectives.stationary = true;
    }
    
    if(config_map.find("RAND_COUNT") != config_map.end()) {
        if(isdigit(config_map["RAND_COUNT"][0]) && stoi(config_map["RAND_COUNT"]))
            config.objectives.zoo = true;
    }

    if(config_map.find("SHOWCASE_TIME") != config_map.end()) {
        config.visualizer.showcase_time = stof(config_map["SHOWCASE_TIME"]);
    }

    if(config_map.find("FPS") != config_map.end()) {
        config.renderer.fps = stof(config_map["FPS"]);
    }

    if(config_map.find("DIM") != config_map.end()) {
        std::istringstream ss(config_map["CUDA_VISIBLE_DEVICES"]);
        std::string cell;
        
        std::getline(ss, cell, ',');
        config.renderer.width = std::stoi(cell);
        std::getline(ss, cell, ',');
        config.renderer.height = std::stoi(cell);
    }

    if(config_map.find("HIDDEN_LAYER_SIZES") != config_map.end()) {
        config.nnrobot.hidden_layer_sizes.clear();
        
        std::istringstream ss(config_map["HIDDEN_LAYER_SIZES"]);
        std::string cell;

        while (std::getline(ss, cell, ',')) {
            config.nnrobot.hidden_layer_sizes.push_back(std::stoi(cell));
        }     
    }
    
    return config;
}

}