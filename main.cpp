#include "Simulator.h"
#include "optimizer.h"
#include "util.h"
#include "NNRobot.h"
#include "VoxelRobot.h"

#ifdef VIDEO
#include "plane_model.h"
#include "robot_model.h"
#include "util.h"

#include "Renderer.h"

#ifdef WRITE_VIDEO
#include <opencv2/opencv.hpp>
#endif
#endif

#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <sys/stat.h>

uint runID = 0;
uint solID = 0;

std::string config_file = "./config.txt";

void handle_commandline_args(const int& argc, char** argv);
int handle_file_io();

#ifdef VIDEO
GLFWwindow* GLFWsetup(bool visualize);
void GLFWinitialize();

void Render(std::vector<SoftBody>& robots);
void Visualize(std::vector<SoftBody>& R);
#endif

template<typename T>
std::vector<T> Solve();

Simulator sim;
// struct OptimizerStrategies {
// 	Optimizer<Candidate>::MutationStrat mutator = Optimizer<Candidate>::MUTATE;
// 	Optimizer<Candidate>::CrossoverStrat crossover = Optimizer<Candidate>::CROSS_SWAP;
// 	Optimizer<Candidate>::NichingStrat niche = Optimizer<Candidate>::NICHE_NONE;
// } strats;
Config config;

int main(int argc, char** argv)
{
	#ifndef OPTIMIZE
	printf("NOT OPTIMIZING\n");
	#endif

	handle_commandline_args(argc, argv);

	printf("----CONFIG----\n");
	config = util::ReadConfigFile("config.txt");
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
	}
	
	#ifdef OPTIMIZE
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
	#endif

	#ifdef VERIFY
	uint sol = 0;
	while(true) {
		char filepath[MAX_FILE_PATH];
		if(snprintf(filepath, MAX_FILE_PATH, "%s/solution_%u_%u.csv", io.out_dir, runID, sol) >= MAX_FILE_PATH)
			exit(-1);

		if (!std::filesystem::exists(filepath)) break;
		
		printf("Verifying file %s\n", filepath);
		ROBOT_TYPE solution;
		solution.Decode(filepath);
		solutions.push_back(solution);

		sol++;
	}
	#endif

	#if !defined(OPTIMIZE) && !defined(VERIFY) && !defined(ZOO)
	uint seed = std::chrono::system_clock::now().time_since_epoch().count();
    srand(seed);
	for(uint i = 0; i < config.pop_size; i++) {
		switch(config.robot_type) {
			case ROBOT_VOXEL: {
				VoxelRobot solution = VoxelRobot();
				solutions.push_back(solution);
			}
			case ROBOT_NN:
			default: {
				NNRobot solution = NNRobot();
				solutions.push_back(solution);
			}
		}
	}
	#endif

	#ifdef BOUNCE
	for(SoftBody& R : solutions) {
		R.translate(glm::vec3(0.0f,7.0f,0.0f));
	}
	#endif

	#ifdef WRITE_VIDEO
	// for(ROBOT_TYPE& R : solutions) {
	Render(solutions);
	// }
	#endif

	#ifdef VISUALIZE
	Visualize(solutions);
	#endif

	return 0;
}

#ifdef OPTIMIZE
template<typename T>
std::vector<T> Solve() {
	Optimizer<T> O;
    O.Solve(config);
	return O.getSolutions();
}
#endif

#ifdef VIDEO

#ifdef WRITE_VIDEO
void Render(std::vector<SoftBody>& robots) {
	printf("RENDERING\n");

	Camera camera(WIDTH, HEIGHT, glm::vec3(0.0f, 0.75f, 5.0f), 0.75f, robots.size());
	// Camera camera(WIDTH, HEIGHT, glm::vec3(-1.5f, 10.0f, 10.0f), 5.0f);
	// Camera camera(WIDTH, HEIGHT, glm::vec3(4.0f, 10.0f, 15.0f), 20.0f);
	// camera = Camera(WIDTH, HEIGHT, glm::vec3(5.0f, 5.0f, 30.0f), 1.0f);

	SoftBody R = robots[camera.tabIdx];
	RobotModel<SoftBody> model(R);

	GLFWwindow* window = GLFWsetup(false);
	Shader shader("../shaders/vert.glsl", "../shaders/frag.glsl");
	shader.Bind();

	model.Bind();

	shader.Unbind();
	
	// opencv video writer
	cv::VideoWriter video;

	#if defined(BOUNCE) && !defined(ENV_GRAVITY)
	if(snprintf(io.out_sol_video_file,sizeof(io.out_sol_video_file),"%s/stationary_%u_%u.avi",io.out_dir, runID, solID))
	#elif defined(BOUNCE)
	if(snprintf(io.out_sol_video_file,sizeof(io.out_sol_video_file),"%s/bounce_%u_%u.avi",io.out_dir, runID, solID))
	#elif defined(ZOO)
	if(snprintf(io.out_sol_video_file,sizeof(io.out_sol_video_file),"%s/zoo_0.avi",io.out_dir))
	#elif !defined(OPTIMIZE)
	if(snprintf(io.out_sol_video_file,sizeof(io.out_sol_video_file),"%s/random_0.avi",io.out_dir))
	#else
	if(snprintf(io.out_sol_video_file,sizeof(io.out_sol_video_file),"%s/solution_video_%u_%u.avi",io.out_dir, runID, solID))
	#endif
	{
		printf("%s\n",io.out_sol_video_file);
		video = cv::VideoWriter(cv::String(io.out_sol_video_file),
			cv::VideoWriter::fourcc('M','J','P','G'), FPS, cv::Size(WIDTH,HEIGHT));

		video.open(cv::String(io.out_sol_video_file),cv::VideoWriter::fourcc('M','J','P','G'), FPS, cv::Size(WIDTH,HEIGHT));
		assert(video.isOpened());
	}
	else printf("Something Failed\n");

	PlaneModel p;

	float max_render_time = 5;
	sim.Initialize(robots[0], 1);
	sim.setMaxTime(1 / FPS);

	// Main while loop
	uint tabId = camera.tabIdx;
	std::vector<Element> robot_elements(1);
	for(auto solution : robots) {
		tabId = camera.tabIdx;
		printf("rendering %u/%lu\n",tabId+1,robots.size());

		model.Unbind();
		SoftBody R = robots[tabId];
		model.Update(R);
		model.Bind();
		sim.Reset();
		while (!glfwWindowShouldClose(window) && sim.getTotalTime() < max_render_time)
		{
			robot_elements[0] = {R.getMasses(),R.getSprings()};
			std::vector<ElementTracker> trackers = sim.Simulate(robot_elements);
			std::vector<Element> results = sim.Collect(trackers);

			R.Update(results[0]);
			model.Update(R);

			// Specify the color of the background
			GLCall(glClearColor(0.73f, 0.85f, 0.92f, 1.0f));
			// Clean the back buffer and depth buffer
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			// Handles camera inputs
			camera.Inputs(window);
			// Updates and exports the camera matrix to the Vertex Shader
			camera.updateMatrix(45.0f, 0.1f, 100.0f);

			model.Draw(shader, camera);

			p.Draw(shader, camera);

			cv::Mat frame(HEIGHT,WIDTH,CV_8UC3);
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
	// Delete all the objects we've created
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LINE_SMOOTH);

	// Delete window before ending the program
	glfwDestroyWindow(window);
	// Terminate GLFW before ending the program
	glfwTerminate();
}
#endif

void Visualize(std::vector<SoftBody>& robots) {
	printf("VISUALIZING\n");

	// R.printObjectPositions();

	GLFWwindow* window = GLFWsetup(true);
	Shader shader("../shaders/vert.glsl", "../shaders/frag.glsl");
	shader.Bind();
	Camera camera(WIDTH, HEIGHT, glm::vec3(0.0f, 0.75f, 5.0f), 0.75f, robots.size());
	// Camera camera(WIDTH, HEIGHT, glm::vec3(-1.5f, 5.0f, 15.0f), 5.0f);

	SoftBody R = robots[camera.tabIdx];
	RobotModel<SoftBody> model(R);

	model.Bind();

	shader.Unbind();
	
	#ifdef WRITE_VIDEO
	// opencv video writer
	cv::VideoWriter video;
	if(snprintf(io.out_sol_video_file,sizeof(io.out_sol_video_file),"%s/solution_live_video.avi",io.out_dir)) {
		video = cv::VideoWriter(io.out_sol_video_file,cv::VideoWriter::fourcc('M','J','P','G'), FPS, cv::Size(WIDTH,HEIGHT));

		video.open(io.out_sol_file,cv::VideoWriter::fourcc('M','J','P','G'), FPS, cv::Size(WIDTH,HEIGHT));
		assert(video.isOpened());
	}
	else printf("Something Failed\n");
	#endif

	PlaneModel p;

	float prevTime = glfwGetTime();

	sim.Initialize(robots[0], 1);
	sim.setMaxTime( 1 / FPS);

	// Main while loop
	uint i = 0;
	std::vector<Element> robot_elements(1);

	// printf("-----------------------\n");
	
	uint tabId = camera.tabIdx;
	while (!glfwWindowShouldClose(window))
	{
		if(tabId != camera.tabIdx) {
			tabId = camera.tabIdx;
			model.Unbind();
			R = robots[tabId];
			model.Update(R);
			model.Bind();
			sim.Reset();
		}

		// printf("Iteration: %u\n", i);
		float crntTime = glfwGetTime();
		
		robot_elements[0] = {R.getMasses(),R.getSprings()};
		std::vector<ElementTracker> trackers = sim.Simulate(robot_elements);
		std::vector<Element> results = sim.Collect(trackers);

		R.Update(results[0]);

		model.Update(R);

		while ((crntTime - prevTime) < 1 / FPS) {
			crntTime = glfwGetTime();
		}

		// Simple timer
		prevTime = crntTime;
		
		// Specify the color of the background
		// GLCall(glClearColor(0.73f, 0.85f, 0.92f, 1.0f));
		GLCall(glClearColor(0.1f, 0.1f, 0.1f, 1.0f));
		// Clean the back buffer and depth buffer
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Handles camera inputs
		camera.Inputs(window);
		// Updates and exports the camera matrix to the Vertex Shader
		camera.updateMatrix(45.0f, 0.1f, 100.0f);

		model.Draw(shader, camera);

		p.Draw(shader, camera);

		#ifdef WRITE_VIDEO
		cv::Mat frame(HEIGHT,WIDTH,CV_8UC3);
		//use fast 4-byte alignment (default anyway) if possible
		glPixelStorei(GL_PACK_ALIGNMENT, (frame.step & 3) ? 1 : 4);

		//set length of one complete row in destination data (doesn't need to equal frame.cols)
		glPixelStorei(GL_PACK_ROW_LENGTH, frame.step/frame.elemSize());
		glReadPixels(0, 0, frame.cols, frame.rows, GL_BGR, GL_UNSIGNED_BYTE, frame.data);

		cv::flip(frame, frame, 0);

		video.write(frame);
		#endif

		// Swap the back buffer with the front buffer
		glfwSwapBuffers(window);
		// Take care of all GLFW events
		glfwPollEvents();
		i++;
	}

	#ifdef WRITE_VIDEO
	video.release();
	#endif
	// Delete all the objects we've created
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LINE_SMOOTH);

	// Delete window before ending the program
	glfwDestroyWindow(window);
	// Terminate GLFW before ending the program
	glfwTerminate();
}


GLFWwindow* GLFWsetup(bool visualize) {
	// Initialize GLFW
	glfwInit();

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_VISIBLE, visualize ? GLFW_TRUE : GLFW_FALSE);

	// Create a GLFWwindow object of WIDTH by HEIGHT pixels, naming it "Cubes!"
	GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "Robots!", NULL, NULL);
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

	// Rasterized line width
	glLineWidth(3.0f);

	// Specify the viewport of OpenGL in the Window
	// In this case the viewport goes from x = 0, y = 0, to x = WIDTH, y = HEIGHT
	glViewport(0, 0, WIDTH, HEIGHT);
}
#endif

void handle_commandline_args(const int& argc, char** argv) {
    // for(int i = 0; i < argc; i++) {
    //     if(strcmp(argv[i], "-mutate") == 0) {
	// 		config.optimizer.crossover = argv[i+1];
    //     }

    //     if(strcmp(argv[i], "-cross") == 0) {
	// 		config.optimizer.crossover = argv[i+1];
    //     }

    //     if(strcmp(argv[i], "-niche") == 0) {
	// 		config.optimizer.niche = argv[i+1];
    //     }
		
	// 	if(strcmp(argv[i], "-run") == 0) {
	// 		runID = atoi(argv[i+1]);
    //     }

	// 	if(strcmp(argv[i], "-sol") == 0) {
	// 		solID = atoi(argv[i+1]);
    //     }
    // }
}

int handle_file_io() {
	// Get current date and time
	auto now = std::chrono::system_clock::now();
	time_t current_time = std::chrono::system_clock::to_time_t(now);

	// Convert current time to a string with format YYYY-MM-DD-HHMMSS
	char time_str[20];
	strftime(time_str, sizeof(time_str), "%Y-%m-%d-%H%M%S", localtime(&current_time));

	// Create folder with current time as name
	config.io.out_dir = config.io.out_dir + "/" + std::string(time_str);
	if (mkdir(config.io.out_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1) {
		std::cerr << "Error: Could not create output folder." << std::endl;
		return 1;
	}

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