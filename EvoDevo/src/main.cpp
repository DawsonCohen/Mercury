#include "EvoDevo.h"

#include "EvoDevo/Evolvables/NNRobot.h"
#include "EvoDevo/Optimizer/optimizer.h"
#include <Eigen/Core>

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

EvoDevo::Simulator sim;
EvoDevo::Config config;

int main(int argc, char** argv)
{
	handle_commandline_args(argc, argv);

	printf("----CONFIG----\n");
	config = EvoDevo::Util::ReadConfigFile(config_file);
	handle_file_io();
	printf("--------------\n");
	std::cout << "Ouput directory: " << config.io.out_dir << std::endl;

	std::vector<EvoDevo::SoftBody> solutions;
	
	switch(config.robot_type) 
	{
		/*case ROBOT_VOXEL:
			Evaluator<VoxelRobot>::Initialize(config);
			break;*/
		case EvoDevo::ROBOT_NN:
		default:
			EvoDevo::Evaluator<EvoDevo::NNRobot>::Initialize(config);
			EvoDevo::NNRobot::Configure(config.nnrobot);
	}

	switch(config.robot_type)
	{
		/*case ROBOT_VOXEL:
		{
			std::vector<VoxelRobot> v_solutions = Solve<VoxelRobot>();
			for(VoxelRobot R : v_solutions) {
				solutions.push_back(R);
			}
		}
		break;*/
		case EvoDevo::ROBOT_NN:
		default:
		{
			std::vector<EvoDevo::NNRobot> n_solutions = Solve<EvoDevo::NNRobot>();
			for(EvoDevo::NNRobot R : n_solutions) {
				solutions.push_back(R);
			}
		}
	}

	return 0;
}

template<typename T>
std::vector<T> Solve() {
	EvoDevo::Optimizer<T> O;
    return O.Solve(config);
}

int handle_file_io() {
	// Get current date and time
	auto now = std::chrono::system_clock::now();
	time_t current_time = std::chrono::system_clock::to_time_t(now);

	// Convert current time to a string with format YYYY-MM-DD-HHMMSS
	char time_str[20];
	strftime(time_str, sizeof(time_str), "%Y-%m-%d-%H%M%S", localtime(&current_time));

	// Create config out_dir folder
	EvoDevo::Util::MakeDirectory(config.io.out_dir);
	config.io.base_dir = config.io.out_dir;

	// Create folder with current time as name
	config.io.out_dir = config.io.out_dir + "/" + std::string(time_str);
	EvoDevo::Util::MakeDirectory(config.io.out_dir);

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