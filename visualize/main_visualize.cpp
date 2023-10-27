#include "Simulator.h"
#include "visualizer_util.h"
#include "NNRobot.h"
#include "VoxelRobot.h"
#include <Eigen/Core>

#include "plane_model.h"
#include "robot_model.h"
#include "Renderer.h"

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

GLFWwindow* GLFWsetup(bool visualize);
void GLFWinitialize();
void GLFWterminate(GLFWwindow*);

void Render(std::vector<SoftBody>& robots);
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
	std::cout << "Ouput directory: " << config.io.out_dir << std::endl;
	std::cout << "Input directory: " << config.io.in_dir << std::endl;

	std::vector<SoftBody> solutions;

	// TODO: Update for new directories
	if(config.objectives.verify) {
		std::string filename_template = "^solution_\\w+\\.\\w+$";
		std::regex pattern(filename_template);

		std::string config_template = "^config.txt$";
		std::regex config_pattern(config_template);

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
			} else if(std::filesystem::is_regular_file(file.path()) &&
				std::regex_match(file.path().filename().string(), config_pattern))
				{
					opt_config = util::optimizer::ReadConfigFile(file.path());
			}
		}
	}
	
	if(!config.objectives.verify) {
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
					solutions.push_back(solution);
				}
			}
		}
	}

	Evaluator<SoftBody>::Initialize(opt_config);
	Evaluator<SoftBody>::BatchEvaluate(solutions);

	
	if(config.objectives.video) {
		Render(solutions);
	}

	if(config.objectives.interactive) {
		Visualize(solutions);
	}

	return 0;
}

void Render(std::vector<SoftBody>& robots) {
	printf("RENDERING\n");

	GLFWwindow* window = GLFWsetup(false);
	
	{ // non-glfw scope
		Shader shader("./shaders/vert.glsl", "./shaders/frag.glsl");
		Camera camera(window, glm::vec3(0.0f, 0.75f, 5.0f), 0.75f, robots.size());

		SoftBody R = robots[camera.tabIdx];
		RobotModel<SoftBody> model(R);

		// opencv video writer
		cv::VideoWriter video;

		if(config.objectives.stationary)
			out_sol_video_file =  config.io.out_dir + "/stationary_0.avi";
		else if(config.objectives.zoo)
			out_sol_video_file =  config.io.out_dir + "/zoo_0.avi";
		else if(config.objectives.verify)
			out_sol_video_file =  config.io.out_dir + "/solutions_0.avi";
		else
			out_sol_video_file =  config.io.out_dir + "/random_0.avi";
		
		video = cv::VideoWriter(cv::String(out_sol_video_file),
			cv::VideoWriter::fourcc('M','J','P','G'), config.renderer.fps, cv::Size(config.renderer.width,config.renderer.height));

		video.open(cv::String(out_sol_video_file),cv::VideoWriter::fourcc('M','J','P','G'), config.renderer.fps, cv::Size(config.renderer.width,config.renderer.height));
		assert(video.isOpened());

		float max_render_time = config.visualizer.showcase_time;
		
		sim.Initialize(robots[0], 1, config.simulator);

		float time_step = 1 / config.renderer.fps;
		uint devo_cycles = config.devo.devo_cycles;
		float devo_time = config.devo.devo_time;
		float timeToDevo = devo_time;

		// Main while loop
		uint tabId = camera.tabIdx;
		std::vector<Element> robot_elements(1);
		for(auto solution : robots) {
			tabId = camera.tabIdx;
			printf("rendering %u/%lu\n",tabId+1,robots.size());

			SoftBody R = robots[tabId];
			sim.Reset();
			robot_elements[0] = {R.getMasses(),R.getSprings()};
			std::vector<ElementTracker> trackers = sim.SetElements(robot_elements);
						
			while (!glfwWindowShouldClose(window) && sim.getTotalTime() < max_render_time)
			{
				if(devo_cycles > 0 && timeToDevo <= 0.0f) {
					devo_cycles--;
					timeToDevo = devo_time;
					sim.Devo();
				} else if(devo_cycles > 0) {
					timeToDevo -= time_step;
				}

				sim.Simulate(time_step);
				
				std::vector<Element> results = sim.Collect(trackers);

				R.Update(results[0]);
				model.Update(R);

				// Specify the color of the background
				GLCall(glClearColor(1.0f, 1.0f, 1.0f, 1.0f));
				// GLCall(glClearColor(0.1f, 0.1f, 0.1f, 1.0f));
				// Clean the back buffer and depth buffer
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

				// Handles camera inputs
				camera.Update();
				// Updates and exports the camera matrix to the Vertex Shader
				camera.updateMatrix(45.0f, 0.1f, 100.0f);

				model.Draw(shader, camera);

				cv::Mat frame(config.renderer.height,config.renderer.width,CV_8UC3);
				//use fast 4-byte alignment (default anyway) if possible
				glPixelStorei(GL_PACK_ALIGNMENT, (frame.step & 3) ? 1 : 4);

				//set length of one complete row in destination data (doesn't need to equal frame.cols)
				glPixelStorei(GL_PACK_ROW_LENGTH, frame.step/frame.elemSize());
				glReadPixels(0, 0, frame.cols, frame.rows, GL_BGR, GL_UNSIGNED_BYTE, frame.data);

				cv::flip(frame, frame, 0);

				video.write(frame);

				// Swap the back buffer with the front buffer
				glfwSwapBuffers(window);
				// Take care of all GLFW events
				glfwPollEvents();
			}
			camera.Tab();
		}
		video.release();
	}

	GLFWterminate(window);
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	// unused variables
	(void) scancode;
	(void) mods;

	Camera* camera = static_cast<Camera*>(glfwGetWindowUserPointer(window));

	if(camera){
		if (action == GLFW_PRESS) {
			// Handle key press.
			if (key == GLFW_KEY_TAB) {
				camera->Tab();
			}
		}
	}
}

void Visualize(std::vector<SoftBody>& robots) {
	printf("VISUALIZING\n");

	// R.printObjectPositions();

	GLFWwindow* window = GLFWsetup(true);


	{ // non-glfw scope
		Shader shader("./shaders/vert.glsl", "./shaders/frag.glsl");
		Camera camera(window, glm::vec3(0.0f, 0.75f, 5.0f), 0.75f, robots.size());

		SoftBody R = robots[camera.tabIdx];
		RobotModel<SoftBody> model(R);

		// opencv video writer
		cv::VideoWriter video;
		if(config.objectives.video) {
			out_sol_video_file =  config.io.out_dir + "/solution_live_video.avi";
			video = cv::VideoWriter(out_sol_video_file,cv::VideoWriter::fourcc('M','J','P','G'), config.renderer.fps, cv::Size(config.renderer.width,config.renderer.height));
			video.open(out_sol_video_file,cv::VideoWriter::fourcc('M','J','P','G'), config.renderer.fps, cv::Size(config.renderer.width,config.renderer.height));
			assert(video.isOpened());
		}

		float prevTime = glfwGetTime();

		sim.Initialize(robots[0], 1, config.simulator);

		float time_step = 1 / config.renderer.fps;
		uint devo_cycles = config.devo.devo_cycles;
		float devo_time = config.devo.devo_time;
		float timeToDevo = devo_time;

		// Main while loop
		uint i = 0;
		std::vector<Element> robot_elements(1);
		
		glfwSetWindowUserPointer(window, &camera);
		glfwSetKeyCallback(window, keyCallback);

		int tabId = -1;
		std::vector<ElementTracker> trackers;
		std::vector<Element> results;
		while (!glfwWindowShouldClose(window))
		{
			if(tabId != (int) camera.tabIdx) {
				tabId = camera.tabIdx;
				sim.Reset();
				timeToDevo = devo_time;
				devo_cycles = config.devo.devo_cycles;

				R = robots[tabId];
				R.updateBaseline();
				robot_elements[0] = {R.getMasses(),R.getSprings()};
				trackers = sim.SetElements(robot_elements);

				printf("Robot %i, fitness: %f\n", tabId, R.fitness());
			}

			if(devo_cycles > 0 && timeToDevo <= 0.0f) {
					devo_cycles--;
					timeToDevo = devo_time;
					sim.Devo();
					results = sim.Collect(trackers);
					R.Update(results[0]);
					R.updateBaseline();
			} else if(devo_cycles > 0) {
				timeToDevo -= time_step;
			}

			// printf("Iteration: %u\n", i);
			float crntTime = glfwGetTime();
			
			sim.Simulate(time_step);

			results = sim.Collect(trackers);

			R.Update(results[0]);

			// printf("Robot %i, fitness: %f\n", tabId, R.fitness());

			model.Update(R);

			while ((crntTime - prevTime) < 1 / config.renderer.fps) {
				crntTime = glfwGetTime();
			}

			// Simple timer
			prevTime = crntTime;
			
			// Specify the color of the background
			// GLCall(glClearColor(0.73f, 0.85f, 0.92f, 1.0f));
			GLCall(glClearColor(0.1f, 0.1f, 0.1f, 1.0f));
			// GLCall(glClearColor(0.0f, 0.0f, 0.0f, 1.0f));
			// GLCall(glClearColor(1.0f, 1.0f, 1.0f, 1.0f));
			// Clean the back buffer and depth buffer
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			// Handles camera inputs
			camera.Update();
			// Updates and exports the camera matrix to the Vertex Shader
			camera.updateMatrix(45.0f, 0.1f, 100.0f);

			model.Draw(shader, camera);

			if(config.objectives.video) {
				cv::Mat frame(config.renderer.height,config.renderer.width,CV_8UC3);
				//use fast 4-byte alignment (default anyway) if possible
				glPixelStorei(GL_PACK_ALIGNMENT, (frame.step & 3) ? 1 : 4);

				//set length of one complete row in destination data (doesn't need to equal frame.cols)
				glPixelStorei(GL_PACK_ROW_LENGTH, frame.step/frame.elemSize());
				glReadPixels(0, 0, frame.cols, frame.rows, GL_BGR, GL_UNSIGNED_BYTE, frame.data);

				cv::flip(frame, frame, 0);

				video.write(frame);
			}

			if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
				glfwSetWindowShouldClose(window, true);
			}

			// Swap the back buffer with the front buffer
			glfwSwapBuffers(window);
			// Take care of all GLFW events
			glfwPollEvents();
			i++;
		}
		video.release();
	}
	GLFWterminate(window);
}


GLFWwindow* GLFWsetup(bool visualize) {
	// Initialize GLFW
	glfwInit();

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_VISIBLE, visualize ? GLFW_TRUE : GLFW_FALSE);

	// Create a GLFWwindow object of config.renderer.width by config.renderer.height pixels, naming it "Cubes!"
	GLFWwindow* window = glfwCreateWindow(config.renderer.width, config.renderer.height, "Robots!", NULL, NULL);
	// Error check if the window fails to create
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		exit(-1);
	}

	// Introduce the window into the current context
	glfwMakeContextCurrent(window);

	// Load GLAD so it configures OpenGL
	gladLoadGL();

	GLFWinitialize();
	return window;
}

void GLFWinitialize()	        // We call this right after our OpenGL window is created.
{
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);		// This Will Clear The Background Color To Black
	glClearDepth(1.0);				// Enables Clearing Of The Depth Buffer

	glEnable(GL_DEPTH_TEST);		        // Enables Depth Testing
	glEnable(GL_LINE_SMOOTH);

	// Rasterized line config.renderer.width
	glLineWidth(3.0f);

	// Specify the viewport of OpenGL in the Window
	// In this case the viewport goes from x = 0, y = 0, to x = config.renderer.width, y = config.renderer.height
	glViewport(0, 0, config.renderer.width, config.renderer.height);
}

void GLFWterminate(GLFWwindow* window) {
	// Delete all the objects we've created
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LINE_SMOOTH);

	// Delete window before ending the program
	glfwDestroyWindow(window);
	// Terminate GLFW before ending the program
	glfwTerminate();
}

int handle_file_io() {
	// // Get current date and time
	// auto now = std::chrono::system_clock::now();
	// time_t current_time = std::chrono::system_clock::to_time_t(now);

	// // Convert current time to a string with format YYYY-MM-DD-HHMMSS
	// char time_str[20];
	// strftime(time_str, sizeof(time_str), "%Y-%m-%d-%H%M%S", localtime(&current_time));

	// // Create config out_dir folder
	// util::MakeDirectory(config.io.out_dir);

	// // Create folder with current time as name
	// config.io.out_dir = config.io.out_dir + "/" + std::string(time_str);
	// util::MakeDirectory(config.io.out_dir);
	
	std::ifstream src(config_file, std::ios::binary);
	if(!src) {
		std::cerr << "Error opening config file: " << config_file << std::endl;
		return 1;
	}
	// std::ofstream dst(config.io.out_dir + "/config.txt", std::ios::binary);
	// if (!dst) {
    //     std::cerr << "Error creating result file: " << config.io.out_dir << "/config.txt" << std::endl;
    //     return 1;
    // }
	// dst << src.rdbuf();
	// if (dst.fail()) {
    //     std::cerr << "Error writing to result file: " << config.io.out_dir << "/config.txt" << std::endl;
	// 	return 1;
	// }

	return 0;
}

void handle_commandline_args(int argc, char** argv) {
	if(argc > 1)
		config_file = std::string(argv[1]);
}