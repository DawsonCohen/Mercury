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

#include "optimizer_util.h"

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

OptimizerConfig optimizer::ReadConfigFile(const std::string& filename) {
    std::unordered_map<std::string, std::string> config_map;

    OptimizerConfig config = util::common::ReadConfigFile(filename);

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
        
        config_map[key] = value;
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

    if(config_map.find("SAVE_SKIP") != config_map.end()) {
        config.optimizer.save_skip = stoi(config_map["SAVE_SKIP"]);
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

    if(config_map.find("REPLACEMENT") != config_map.end()) {
        if(config_map["REPLACEMENT"] == "pareto") {
            config.optimizer.replacement = PARETO;
        } else if(config_map["REPLACEMENT"] == "standard") {
            config.optimizer.replacement = REPLACE_STANDARD;
        } else {
            std::cerr << "Replacement type " << config_map["REPLACEMENT"] << " not supported" << std::endl;
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

    return config;
}

}