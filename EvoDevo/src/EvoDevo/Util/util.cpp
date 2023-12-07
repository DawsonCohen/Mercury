#include "evpch.h"

#include <fstream>
#include <dirent.h>
#include <unistd.h>
#include <chrono>
#include <sys/stat.h>
#include <memory>

#include "EvoDevo/Util/util.h"

namespace EvoDevo {

    void FileWriter::Open(const std::string& filepath, bool append) {
        std::string directory = filepath.substr(0, filepath.find_last_of('/'));
        if (Util::MakeDirectory(directory)) {
            EV_CORE_ASSERT(false, "Could not create output directory to {0}", directory);
            return;
        };

        if(m_OutputStream.is_open()) m_OutputStream.close();

        m_OutputStream = std::ofstream(filepath, std::ios::out | (append ? std::ios::app : std::ios::trunc));

        EV_CORE_ASSERT(m_OutputStream.is_open(), "Could not open file {0}", filepath);

        this->Write("TEST\n");
    }

    
namespace Util {

    int MakeDirectory(const std::string& directory) {
        std::string path = directory;
        std::string token;
        size_t pos = 0;
        while ((pos = path.find('/')) != std::string::npos) {
            token += path.substr(0, pos) + "/";
            if (mkdir(token.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1) {
                if (errno != EEXIST && errno != EISDIR) {
                    std::cerr << "Error: Could not create output directory " << token << std::endl;
                    return 1;
                }
            }
            path.erase(0, pos + 1);
        }
        token += path; // Include the last directory in the path
        if (mkdir(token.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1) {
            if (errno != EEXIST && errno != EISDIR) {
                std::cerr << "Error: Could not create output directory " << token << std::endl;
                return 1;
            }
        }
        return 0;
    }


    int WriteFile(const std::string& filepath, const std::string& datastring, bool append) {
        std::string directory = filepath.substr(0, filepath.find_last_of('/'));
        if (MakeDirectory(directory)) {
            EV_CORE_ASSERT(false, "Could not create output directory to {0}", directory);
            return 1;
        };
        std::ofstream outfile(filepath, std::ios::out | (append ? std::ios::app : std::ios::trunc));

        if(outfile.is_open()) {
            outfile << datastring;
        } else {
            std::cerr << "Error writing to file: " << filepath << std::endl;
            return 1;
        }

        outfile.flush();

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

        if(config_map.find("OUT_DIR") != config_map.end()) {
            config.io.out_dir = config_map["OUT_DIR"];
        }

        if(config_map.find("IN_DIR") != config_map.end()) {
            config.io.in_dir = config_map["IN_DIR"];
        }

        if(config_map.find("BASE_DIR") != config_map.end()) {
            config.io.base_dir = config_map["BASE_DIR"];
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
            config.nnrobot.crossover_neuron_count = stof(config_map["CROSSOVER_NEURONS"]);
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

        if(config_map.find("CROSSOVER_DISTRIBUTION") != config_map.end()) {
            if(config_map["CROSSOVER_DISTRIBUTION"] == "none") {
                config.nnrobot.crossover_distribution = CROSS_DIST_NONE;
            } else if(config_map["CROSSOVER_DISTRIBUTION"] == "binomial") {
                config.nnrobot.crossover_distribution = CROSS_DIST_BINOMIAL;
            } else {
                std::cerr << "Crossover distribution " << config_map["CROSSOVER_DISTRIBUTION"] << " not supported" << std::endl;
            }
        }

        if(config_map.find("CROSSOVER_TYPE") != config_map.end()) {
            if(config_map["CROSSOVER_TYPE"] == "individual") {
                config.nnrobot.crossover_type = CROSS_INDIVIDUAL;
            } else if(config_map["CROSSOVER_TYPE"] == "contiguous") {
                config.nnrobot.crossover_type = CROSS_CONTIGUOUS;
            } else {
                std::cerr << "Crossover type " << config_map["CROSSOVER_TYPE"] << " not supported" << std::endl;
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

    RobotType ReadRobotType(const std::string& filename) {
        std::ifstream file(filename);
        if(!file.is_open()) {
            std::cerr << "ERROR: config file " << filename << " does not exist" << std::endl;
            exit(0);
        }

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
        return ROBOT_VOXEL;
    }

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

}

}