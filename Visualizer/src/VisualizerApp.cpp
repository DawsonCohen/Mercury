#include "EvoDevo.h"
#include "Elastic.h"

#include "Sandbox2D.h"

#include "Evaluator.h"

#include <opencv2/opencv.hpp>

#include <filesystem>
#include <regex>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <sys/stat.h>

using namespace EvoDevo;

std::string config_file = "configs/config.random";

void handle_commandline_args(int argc, char** argv);
int handle_file_io();

void Visualize(std::vector<SoftBody>& R);

std::string out_sol_video_file;
std::string in_sol_file;

VisualizerConfig config;

class Sandbox : public Elastic::Application
{
public:
	Sandbox(const Elastic::ApplicationSpecification& specification)
		: Elastic::Application(specification)
	{
		PushLayer(new Sandbox2D());
	}

	~Sandbox()
	{
	}
};

Elastic::Application* Elastic::CreateApplication(Elastic::ApplicationCommandLineArgs args)
{
	ApplicationSpecification spec;
	spec.Name = "Sandbox";
	spec.WorkingDirectory = "Sandbox";
	spec.CommandLineArgs = args;

	handle_commandline_args(args);

	return new Sandbox(spec);
}

void handle_commandline_args(Elastic::ApplicationCommandLineArgs args) {
	if(args.Count > 1)
		config_file = std::string(args[1]);
}