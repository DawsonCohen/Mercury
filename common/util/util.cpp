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
#include "util.h"

namespace util {

void RemoveOldFiles(const std::string& dir) {
    DIR* directory = opendir(dir.data());
    if (directory == nullptr) {
        return;
    }

    dirent* entry;
    while ((entry = readdir(directory)) != nullptr) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        const char* path = entry->d_name;
        struct stat file_info;
        if (stat(path, &file_info) == -1) {
            continue;
        }

        if (S_ISDIR(file_info.st_mode)) {
            RemoveOldFiles(path);
            rmdir(path);
        } else {
            unlink(path);
        }
    }

    closedir(directory);
}

template<typename... Ts>
std::string DataToCSV(const std::string& header, const std::vector<std::tuple<Ts...>>& data){
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

int MakeDirectory(const std::string& directory) {
    if (mkdir(directory.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1) {
        if (errno != EEXIST && errno != EISDIR) {
            std::cerr << "Error: Could not create output directory " << directory << std::endl;
            return 1;
        }
    }
    return 0;
}

int WriteCSV(const std::string& filename, const std::string& directory, const std::string& datastring) {
    if(MakeDirectory(directory) != 0) {
        return 1;
    };

    std::ofstream outfile(directory + std::string("/") + filename);

    if(outfile.is_open())
        outfile << datastring;
    else {
        std::cerr << "Error parsing config file: " << std::string(filename) << std::endl;
        return 1;
    }

    return 0;
}

Config common::ReadConfigFile(const std::string& filename) {
    std::unordered_map<std::string, std::string> config_map;

    Config config;

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

    if(config_map.find("ROBOT_TYPE") != config_map.end()) {
        if(config_map["ROBOT_TYPE"] == "NNRobot") {
            config.robot_type = ROBOT_NN;
        } else if(config_map["ROBOT_TYPE"] == "VoxelRobot") {
            config.robot_type = ROBOT_VOXEL;
        } else {
            std::cerr << "Robot type " << config_map["ROBOT_TYPE"] << " not supported" << std::endl;
        }
    }

    if(config_map.find("OUT_DIR") != config_map.end()) {
        config.io.out_dir = config_map["OUT_DIR"];
    }

    if(config_map.find("IN_DIR") != config_map.end()) {
        config.io.in_dir = config_map["IN_DIR"];
    }

    if(config_map.find("TIME_STEP") != config_map.end()) {
        config.simulator.time_step = stof(config_map["TIME_STEP"]);
    }

    if(config_map.find("REPLACED_AMOUNT") != config_map.end()) {
        config.simulator.replaced_springs_per_element = stoi(config_map["REPLACED_AMOUNT"]);
    }

    if(config_map.find("DEVO_TIME") != config_map.end()) {
        config.devo.devo_time = stof(config_map["DEVO_TIME"]);
    }

    if(config_map.find("DEVO_CYCLES") != config_map.end()) {
        config.devo.devo_cycles = stoi(config_map["DEVO_CYCLES"]);
    }

    if(config_map.find("CROSSOVER_NEURONS") != config_map.end()) {
        config.nnrobot.crossover_neuron_count = stoi(config_map["CROSSOVER_NEURONS"]);
    }

    if(config_map.find("MUTATION_WEIGHTS") != config_map.end()) {
        config.nnrobot.mutation_weight_count = stoi(config_map["MUTATION_WEIGHTS"]);
    }

    if(config_map.find("SPRINGS_PER_MASS") != config_map.end()) {
        config.nnrobot.springs_per_mass = stoi(config_map["SPRINGS_PER_MASS"]);
    }

    if(config_map.find("HIDDEN_LAYER_SIZES") != config_map.end()) {
        config.nnrobot.hidden_layer_sizes.clear();
        
        std::istringstream ss(config_map["HIDDEN_LAYER_SIZES"]);
        std::string cell;

        while (std::getline(ss, cell, ',')) {
            config.nnrobot.hidden_layer_sizes.push_back(std::stoi(cell));
        }
    }

    if(config_map.find("CUDA_VISIBLE_DEVICES") != config_map.end()) {
        config.hardware.cuda_device_ids.clear();
        
        std::istringstream ss(config_map["CUDA_VISIBLE_DEVICES"]);
        std::string cell;

        while (std::getline(ss, cell, ',')) {
            config.hardware.cuda_device_ids.push_back(std::stoi(cell));
        }
    }

    return config;
}

RobotType ReadRobotType(const std::string& filename) {
    std::ifstream file(filename);
    if (file) {
        std::string line;
        while (std::getline(file, line)) {
            // Ignore comments and empty lines
            if (line.empty() || line[0] == '#') {
                continue;
            }

            // Split line into key-value pair
            std::size_t pos = line.find('=');
            if (pos == std::string::npos) {
                continue;
            }

            std::string key = line.substr(0, pos);
            std::string value = line.substr(pos+1);
            if(key == "type") {
                if(value == "NNRobot")
                    return ROBOT_NN;
                else if(value == "VoxelRobot")
                    return ROBOT_VOXEL;
                else
                    break;
            }
        }
        std::cerr << "ERROR: ReadRobotType could not parse config file " << filename << std::endl;
    } else {
        std::cerr << "ERROR: config file " << filename << " does not exist" << std::endl;
    }
    return ROBOT_VOXEL;
}

}