#include "Simulator.h"
#include "optimizer.h"
#include "optimizer_util.h"
#include "NNRobot.h"
#include "VoxelRobot.h"
#include <Eigen/Core>

#include <filesystem>
#include <regex>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <sys/stat.h>

uint runID = 0;
uint solID = 0;

std::string config_file = "configs/config.default";

void handle_commandline_args(int argc, char** argv);
int handle_file_io();

template<typename T>
std::vector<T> Solve();

Simulator sim;
OptimizerConfig config;

int main(int argc, char** argv)
{
	handle_commandline_args(argc, argv);

	printf("----CONFIG----\n");
	config = util::optimizer::ReadConfigFile(config_file);
	handle_file_io();
	printf("--------------\n");
	std::cout << "Ouput directory: " << config.io.out_dir << std::endl;

	std::vector<SoftBody> solutions;
	
	switch(config.robot_type){
		case ROBOT_VOXEL:
			Evaluator<VoxelRobot>::Initialize(config);
			break;
		case ROBOT_NN:
		default:
			Evaluator<NNRobot>::Initialize(config);
			NNRobot::Configure(config.nnrobot);
	}

	switch(config.robot_type) {
		case ROBOT_VOXEL:
		{
			std::vector<VoxelRobot> v_solutions = Solve<VoxelRobot>();
			for(VoxelRobot R : v_solutions) {
				solutions.push_back(R);
			}
			break;
		}
		case ROBOT_NN:
		default:
		{
			std::vector<NNRobot> n_solutions = Solve<NNRobot>();
			for(NNRobot R : n_solutions) {
				solutions.push_back(R);
			}
		}
	}

	return 0;
}

template<typename T>
std::vector<T> Solve() {
	Optimizer<T> O;
    O.Solve(config);
	return O.getSolutions();
}

int handle_file_io() {
	// Get current date and time
	auto now = std::chrono::system_clock::now();
	time_t current_time = std::chrono::system_clock::to_time_t(now);

	// Convert current time to a string with format YYYY-MM-DD-HHMMSS
	char time_str[20];
	strftime(time_str, sizeof(time_str), "%Y-%m-%d-%H%M%S", localtime(&current_time));

	// Create config out_dir folder
	util::MakeDirectory(config.io.out_dir);

	// Create folder with current time as name
	config.io.out_dir = config.io.out_dir + "/" + std::string(time_str);
	util::MakeDirectory(config.io.out_dir);
	
	std::ifstream src(config_file, std::ios::binary);
	if(!src) {
		std::cerr << "Error opening config file: " << config_file << std::endl;
		return 1;
	}
	std::ofstream dst(config.io.out_dir + "/config.txt", std::ios::binary);
	if (!dst) {
        std::cerr << "Error creating result file: " << config.io.out_dir << "/config.txt" << std::endl;
        return 1;
    }
	dst << src.rdbuf();
	if (dst.fail()) {
        std::cerr << "Error writing to result file: " << config.io.out_dir << "/config.txt" << std::endl;
		return 1;
	}

	return 0;
}

void handle_commandline_args(int argc, char** argv) {
	if(argc > 1)
		config_file = std::string(argv[1]);
}