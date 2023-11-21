#include "Simulator.h"
#include "visualizer_util.h"
#include "NNRobot.h"
#include "VoxelRobot.h"
#include <Eigen/Core>

#include "robot_model.h"
#include "application.h"

#include "Evaluator.h"

#include <opencv2/opencv.hpp>

#include <filesystem>
#include <regex>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <sys/stat.h>

std::string config_file = "configs/config.random";

void handle_commandline_args(int argc, char** argv);
int handle_file_io();

void Visualize(std::vector<SoftBody>& R);

std::string out_sol_video_file;
std::string in_sol_file;

Simulator sim;
VisualizerConfig config;
OptimizerConfig opt_config = OptimizerConfig();

int main(int argc, char** argv)
{
	handle_commandline_args(argc, argv);

	printf("----CONFIG----\n");
	config = util::visualizer::ReadConfigFile(config_file);
	handle_file_io();
	printf("--------------\n");

	std::vector<SoftBody> solutions;

	if(config.objectives.verify) {
		printf("IN_DIR: %s\n", config.io.in_dir.c_str());
		if(config.io.in_dir == "") {
			std::string filename = config.io.base_dir + "/latest.txt";
			std::ifstream file(filename);
			if(!file.is_open()) {
				std::cerr << "ERROR: directory file " << filename << " does not exist" << std::endl;
				exit(0);
			}
			std::string line;
			std::getline(file, line);
			config.io.in_dir = line;
		}
		if(config.io.out_dir == "") {
			config.io.out_dir = config.io.in_dir;
		}

		std::string filename_template = "^solution_\\w+\\.\\w+$";
		std::regex pattern(filename_template);

		std::string config_template = "^config.txt$";
		std::regex config_pattern(config_template);

		for (const auto& file : std::filesystem::directory_iterator(config.io.in_dir))
		{
			if(std::filesystem::is_regular_file(file.path()) &&
				std::regex_match(file.path().filename().string(), config_pattern))
			{
				opt_config = util::optimizer::ReadConfigFile(file.path());
			}
		}

		NNRobot::Configure(opt_config.nnrobot);
		for (const auto& file : std::filesystem::directory_iterator(config.io.in_dir))
		{
			if (std::filesystem::is_regular_file(file.path()) &&
				std::regex_match(file.path().filename().string(), pattern))
			{
				std::cout << file.path().filename() << std::endl;
				switch(util::ReadRobotType(file.path())) {
					case ROBOT_VOXEL: {
						VoxelRobot solution;
						solution.Decode(file.path());
						solutions.push_back(solution);
						break;
					}
					case ROBOT_NN:
					default: {
						NNRobot solution;
						solution.Decode(file.path());
						solutions.push_back(solution);
					}
				} 
			}
		}
	} else {
		NNRobot::Configure(config.nnrobot);
		uint seed = std::chrono::system_clock::now().time_since_epoch().count();
		srand(seed);
		for(uint i = 0; i < config.visualizer.rand_count; i++) {
			switch(config.robot_type) {
				case ROBOT_VOXEL: {
					VoxelRobot solution = VoxelRobot();
					solution.Randomize();
					solutions.push_back(solution);
					break;
				}
				case ROBOT_NN:
				default: {
					NNRobot solution = NNRobot();
					solution.Randomize();
					solution.Build();
					solutions.push_back(solution);
				}
			}
		}
	}
	std::cout << "Ouput directory: " << config.io.out_dir << std::endl;
	std::cout << "Input directory: " << config.io.in_dir << std::endl;

	opt_config.evaluator.pop_size = solutions.size();
	opt_config.evaluator.base_time = 0.0f;
	opt_config.devo.devo_cycles = 0;
	Evaluator<SoftBody>::Initialize(opt_config);
	// Evaluator<SoftBody>::BatchEvaluate(solutions,true);

	Application application(solutions, config);
	application.run();
	
	return 0;
}

int handle_file_io() {
	std::ifstream src(config_file, std::ios::binary);
	if(!src) {
		std::cerr << "Error opening config file: " << config_file << std::endl;
		return 1;
	}

	return 0;
}

void handle_commandline_args(int argc, char** argv) {
	if(argc > 1)
		config_file = std::string(argv[1]);
}