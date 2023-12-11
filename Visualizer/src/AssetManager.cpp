#include "AssetManager.h"
#include "Models/robot_model.h"

#include <filesystem>
#include <regex>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <sys/stat.h>

using namespace EvoDevo;

AssetManager::~AssetManager() {
    for(auto asset : m_Assets) {
        delete asset;
    }
}

void AssetManager::loadAssets(std::string directory) {
    printf("IN_DIR: %s\n", directory.c_str());
    std::string filename_template = "^solution_\\w+\\.\\w+$";
    std::regex pattern(filename_template);

    std::string config_template = "^config.txt$";
    std::regex config_pattern(config_template);

    for (const auto& file : std::filesystem::directory_iterator(directory))
    {
        if (std::filesystem::is_regular_file(file.path()) &&
            std::regex_match(file.path().filename().string(), pattern))
        {
            std::cout << file.path().filename() << std::endl;
            SoftBody r;
            switch(Util::ReadRobotType(file.path())) {
                case ROBOT_VOXEL: {
                    VoxelRobot solution;
                    solution.Decode(file.path());
                    r = (SoftBody) solution;
                    break;
                }
                case ROBOT_NN:
                default: {
                    NNRobot solution;
                    solution.Decode(file.path());
                    r = (SoftBody) solution;
                }
            }
            m_Assets.push_back(new RobotModel(r));
        }
    }
}

void AssetManager::loadRandomAssets(size_t count, RobotType type) {
    printf("INITIALIZING %lu RANDOM ASSETS\n", count);
    uint seed = std::chrono::system_clock::now().time_since_epoch().count();
    srand(seed);
    std::vector<SoftBody*> solutions;
    for(uint i = 0; i < count; i++) {
        SoftBody r;
        switch(type) {
            case ROBOT_VOXEL: {
                VoxelRobot solution = VoxelRobot();
                solution.Randomize();
                solution.Build();
                r = (SoftBody) solution;
                break;
            }
            case ROBOT_NN:
            default: {
                NNRobot solution = NNRobot();
                solution.Randomize();
                solution.Build();
                r = (SoftBody) solution;
            }
        }
        m_Assets.push_back(new RobotModel(r));
    }
}

