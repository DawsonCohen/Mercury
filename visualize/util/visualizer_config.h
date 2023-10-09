#ifndef __VISUALIZER_CONFIG_H__
#define __VISUALIZER_CONFIG_H__

#include <string>
#include <vector>
#include "config.h"

struct VisualizerConfig : public Config {
	struct Objectives {
		bool interactive = false;
		bool verify = false;
		bool zoo = false;
		bool stationary = false;
		bool video = false;
	} objectives;
	struct Visualizer {
		uint rand_count = 100;
		float showcase_time = 15.0f;
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