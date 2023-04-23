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
    
std::string FitnessHistoryToCSV(std::vector<std::tuple<ulong,float>>& h) {
    std::string s = "evaluation, solution_fitness\n";
    for(size_t i = 0; i < h.size(); i++) {
        s += std::to_string(std::get<0>(h[i])) + ", " + std::to_string(std::get<1>(h[i]))+"\n";
    }

    return s;
}

std::string PopulationFitnessHistoryToCSV(std::vector<std::tuple<ulong, std::vector<float>, std::vector<float>>> h) {
    std::string s = "evaluation";
    for(size_t i = 0; i < std::get<1>(h[0]).size(); i++) {
        s += ", organism_"+std::to_string(i);
    }
    s+="\n";

    for(size_t i = 0; i < h.size(); i++) {
        for(size_t j = 0; j < std::get<1>(h[0]).size(); j++) {
            s += std::to_string(std::get<0>(h[i])) + ", "+std::to_string(std::get<1>(h[i])[j]);
        }
        s+="\n";
    }
    return s;
}

std::string PopulationDiversityHistoryToCSV(std::vector<std::tuple<ulong, std::vector<float>, std::vector<float>>> h) {
    std::string s = "evaluation";
    for(size_t i = 0; i < std::get<1>(h[0]).size(); i++) {
        s += ", organism_"+std::to_string(i);
    }
    s+="\n";

    for(size_t i = 0; i < h.size(); i++) {
        for(size_t j = 0; j < std::get<1>(h[0]).size(); j++) {
            s += std::to_string(std::get<0>(h[i])) + ", "+std::to_string(std::get<2>(h[i])[j]);
        }
        s+="\n";
    }
    return s;
}

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

Config ReadConfigFile(const std::string& filename) {
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
    
    if(config_map.find("POP_SIZE") != config_map.end()) {
        config.optimizer.pop_size = stoi(config_map["POP_SIZE"]);
        config.evaluator.pop_size = stoi(config_map["POP_SIZE"]);
    }

    // OPTIMIZER CONFIGS
    if(config_map.find("REPEATS") != config_map.end()) {
        config.optimizer.repeats = stoi(config_map["REPEATS"]);
    }

    if(config_map.find("MAX_EVALS") != config_map.end()) {
        config.optimizer.max_evals = stoi(config_map["MAX_EVALS"]);
    }

    if(config_map.find("NICHE_COUNT") != config_map.end()) {
        config.optimizer.niche_count = stoi(config_map["NICHE_COUNT"]);
    }

    if(config_map.find("STEPS_TO_COMBINE") != config_map.end()) {
        config.optimizer.steps_to_combine = stoi(config_map["STEPS_TO_COMBINE"]);
    }

    if(config_map.find("STEPS_TO_EXCHANGE") != config_map.end()) {
        config.optimizer.steps_to_exchange = stoi(config_map["STEPS_TO_EXCHANGE"]);
    }

    if(config_map.find("MUTATION") != config_map.end()) {
        if(config_map["MUTATION"] == "mutate") {
            config.optimizer.mutation = MUTATE;
        } else if(config_map["MUTATION"] == "random") {
            config.optimizer.mutation = MUTATE_RANDOM;
        } else {
            std::cerr << "Mutation type " << config_map["MUTATION"] << " not supported" << std::endl;
        }
    }

    if(config_map.find("CROSSOVER") != config_map.end()) {
        if(config_map["CROSSOVER"] == "swap") {
            config.optimizer.crossover = CROSS_SWAP;
        } else if(config_map["CROSSOVER"] == "dc") {
            config.optimizer.crossover = CROSS_DC;
        } else if(config_map["CROSSOVER"] == "none") {
            config.optimizer.crossover = CROSS_NONE;
        } else if(config_map["CROSSOVER"] == "beam") {
            config.optimizer.crossover = CROSS_BEAM;
        } else {
            std::cerr << "Crossover type " << config_map["CROSSOVER"] << " not supported" << std::endl;
        }
    }

    if(config_map.find("NICHE") != config_map.end()) {
        if(config_map["NICHE"] == "alps") {
            config.optimizer.niche = NICHE_ALPS;
        } else if(config_map["NICHE"] == "hfc") {
            config.optimizer.niche = NICHE_HFC;
        } else if(config_map["NICHE"] == "none") {
            config.optimizer.niche = NICHE_NONE;
        } else {
            std::cerr << "niche type " << config_map["NICHE"] << " not supported" << std::endl;
        }
    }

    if(config_map.find("MUTATION_RATE") != config_map.end()) {
        config.optimizer.mutation_rate = stof(config_map["MUTATION_RATE"]);
    }

    if(config_map.find("CROSSOVER_RATE") != config_map.end()) {
        config.optimizer.crossover_rate = stof(config_map["CROSSOVER_RATE"]);
    }

    if(config_map.find("ELITISM") != config_map.end()) {
        config.optimizer.elitism = stof(config_map["ELITISM"]);
    }

    if(config_map.find("BASE_TIME") != config_map.end()) {
        config.evaluator.base_time = stof(config_map["BASE_TIME"]);
    }

    if(config_map.find("EVAL_TIME") != config_map.end()) {
        config.evaluator.eval_time = stof(config_map["EVAL_TIME"]);
    }

    if(config_map.find("OUT_DIR") != config_map.end()) {
        config.io.out_dir = config_map["OUT_DIR"];
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
        
        size_t pos = 0;
        std::string sizes_str = config_map["HIDDEN_LAYER_SIZES"];
        std::string delimiter = ",";
        while ((pos = sizes_str.find(",")) != std::string::npos) {
            std::string token = sizes_str.substr(0, pos);
            int size_i = std::stoi(token);
            config.nnrobot.hidden_layer_sizes.push_back(size_i);
            sizes_str.erase(0, pos + delimiter.length());
        }
        config.nnrobot.hidden_layer_sizes.push_back(std::stoi(sizes_str));
    }

    /* TODO: 
    
    width
    height
    fps
    max_time

    in_file
    in_dir

    cuda_device_ids
    */ 

    return config;
}

}