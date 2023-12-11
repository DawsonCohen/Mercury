#include "EvoDevo.h"
#include <Elastic.h>
#include "Elastic/Core/EntryPoint.h"

#include "Visualizer.h"

// #include <opencv2/opencv.hpp>

#include <filesystem>
#include <regex>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <sys/stat.h>

using namespace EvoDevo;

std::string config_file = "configs/config.random";

void handle_commandline_args(Elastic::ApplicationCommandLineArgs args);

std::string out_sol_video_file;
std::string in_sol_file;

class Sandbox : public Elastic::Application
{
public:
	Sandbox(const Elastic::ApplicationSpecification& specification)
		: Elastic::Application(specification)
	{
		PushLayer(new Visualizer(config_file));
	}

	~Sandbox()
	{
	}
};

Elastic::Application* Elastic::CreateApplication(Elastic::ApplicationCommandLineArgs args)
{
	ApplicationSpecification spec;
	spec.Name = "Visualizer";
	spec.WorkingDirectory = ".";
	spec.CommandLineArgs = args;

	handle_commandline_args(args);


	uint seed = std::chrono::system_clock::now().time_since_epoch().count();
	srand(seed);

	return new Sandbox(spec);
}

void handle_commandline_args(Elastic::ApplicationCommandLineArgs args) {
	if(args.Count > 1)
		config_file = std::string(args[1]);
}