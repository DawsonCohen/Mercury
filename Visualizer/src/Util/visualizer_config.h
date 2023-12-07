#ifndef __VISUALIZER_CONFIG_H__
#define __VISUALIZER_CONFIG_H__

#include "EvoDevo.h"
#include <string>
#include <vector>

struct VisualizerConfig : public EvoDevo::Config {
	struct Objectives {
		bool verify = false;
		bool zoo = false;
		bool stationary = false;
	} objectives;
	struct Visualizer {
		unsigned int rand_count = 10;
		float showcase_time = 15.0f;
		bool dragVis = false;
		bool writeVideo = false;
		bool headless = false;
	} visualizer;

	struct Renderer {
		float fps = 30;
		int	  width = 1600;
		int   height = 900;
	} renderer;

	VisualizerConfig() {
		simulator.visual = true;
	}

	VisualizerConfig(const Config& config) : Config(config) {
		simulator.visual = true;
	}
};

#endif