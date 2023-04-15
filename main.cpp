#include "Simulator.h"
#include "optimizer.h"
#include "util.h"
#include "NNRobot.h"

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
#include <sys/stat.h>

#define ROBOT_TYPE NNRobot

#define WIDTH	1600
#define HEIGHT	900
#define FPS		30.0f

#define MAX_FILE_PATH (int) 500
#define REPEATS 1

#define BASE_TIME 5.0f
#define MAX_TIME 10.0f

#define MAX_EVALS (ulong) 1e5

#define POP_SIZE (uint) 32
#define TRHEAD_COUNT (uint) std::thread::hardware_concurrency()
#define NICHE_COUNT (uint) std::thread::hardware_concurrency()
#define STEPS_TO_COMBINE (uint) 1e2
#define STEPS_TO_EXCHANGE (uint) 5e3

#define MAX_HEAP (uint) pow(2,MAX_DEPTH+1)-1

#define HIST_SKIP_FACTOR (uint) 1
#define PRINT_SKIP (uint) 1

uint runID = 0;
uint solID = 0;

struct IOLocations {
    char in_file[MAX_FILE_PATH];
    char out_dir[MAX_FILE_PATH];
    char out_pop_fit_file[MAX_FILE_PATH];
    char out_pop_div_file[MAX_FILE_PATH];
    char out_sol_file[MAX_FILE_PATH];
    char out_sol_fit_file[MAX_FILE_PATH];
    char out_sol_text_file[MAX_FILE_PATH];
	#ifdef WRITE_VIDEO
	char out_sol_video_file[MAX_FILE_PATH];
	#endif
};

struct OptimizationStrats {
    Optimizer<ROBOT_TYPE>::MutationStrat mutator = Optimizer<ROBOT_TYPE>::MUTATE;
    Optimizer<ROBOT_TYPE>::CrossoverStrat crossover = Optimizer<ROBOT_TYPE>::CROSS_SWAP;
    Optimizer<ROBOT_TYPE>::NichingStrat niche = Optimizer<ROBOT_TYPE>::NICHE_NONE;
    // ROBOT_TYPE::Encoding encoding = ROBOT_TYPE::ENCODE_RADIUS;
};

void handle_commandline_args(const int& argc, char** argv);
void handle_file_io();

#ifdef VIDEO
GLFWwindow* GLFWsetup(bool visualize);
void GLFWinitialize();

void Render(std::vector<ROBOT_TYPE>& robots);
void Visualize(std::vector<ROBOT_TYPE>& R);
#endif

std::vector<ROBOT_TYPE> Solve();

OptimizationStrats strats;
IOLocations io;
Simulator sim;

int main(int argc, char** argv)
{
	handle_commandline_args(argc, argv);
	handle_file_io();
	printf("Output directory: %s\n",io.out_dir);

	std::vector<ROBOT_TYPE> solutions;
	Evaluator<ROBOT_TYPE>::Initialize(POP_SIZE, BASE_TIME, MAX_TIME);
	
	#ifdef OPTIMIZE
	solutions = Solve();
	#endif

	#ifdef VERIFY
	uint sol = 0;
	while(true) {
		char filepath[MAX_FILE_PATH];
		if(snprintf(filepath, MAX_FILE_PATH, "%s/solution_%u_%u.csv", io.out_dir, runID, sol) >= MAX_FILE_PATH)
			exit(-1);

		if (!std::filesystem::exists(filepath)) break;
		
		printf("Verifying file %s\n", filepath);
		solutions.push_back(ReadVoxelRobot(filepath));

		sol++;
	}
	#endif

	#if !defined(OPTIMIZE) && !defined(VERIFY) && !defined(ZOO)
	uint seed = std::chrono::system_clock::now().time_since_epoch().count();
    srand(seed);
	for(uint i = 0; i < POP_SIZE; i++) {
		ROBOT_TYPE solution = ROBOT_TYPE();
		// solution.Randomize();
		solutions.push_back(solution);
	}
	#endif

	#ifdef BOUNCE
	for(ROBOT_TYPE& R : solutions) {
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
std::vector<ROBOT_TYPE> Solve() {
	Optimizer<ROBOT_TYPE> O;
	sim.setMaxTime(MAX_TIME);
	
	O.niche_count = NICHE_COUNT;
	O.thread_count = TRHEAD_COUNT;
	O.steps_to_combine = STEPS_TO_COMBINE;
	O.exchange_steps = STEPS_TO_EXCHANGE;
	O.max_evals = MAX_EVALS;
	O.pop_size = POP_SIZE;
	O.hist_skip_factor = HIST_SKIP_FACTOR;
	O.print_skip = PRINT_SKIP;

	O.mutator = strats.mutator;
    O.crossover = strats.crossover;
    O.niche = strats.niche;
    
    util::RemoveOldFiles(io.out_dir);
	
    for(unsigned N = 0; N < REPEATS; N++) {
        printf("Started Run %i\n",N);
		std::vector<ROBOT_TYPE> solutions = O.Solve();

		printf("SOLUTIONS: %lu\n", solutions.size());

		if(snprintf(io.out_sol_fit_file,sizeof(io.out_sol_fit_file),"%s/solution_history_%i.csv",io.out_dir,N) < (int) sizeof(io.out_sol_fit_file)
            && snprintf(io.out_pop_fit_file,sizeof(io.out_pop_fit_file),"%s/fitness_history_%i.csv",io.out_dir,N) < (int) sizeof(io.out_pop_fit_file)
			&& snprintf(io.out_pop_div_file,sizeof(io.out_pop_div_file),"%s/diversity_history_%i.csv",io.out_dir,N) < (int) sizeof(io.out_pop_div_file)){
            
            printf("Writing History\n");
			
            std::string fitnessHistoryCSV = util::FitnessHistoryToCSV(O.getFitnessHistory());
            std::string popFitHistoryCSV = util::PopulationFitnessHistoryToCSV(O.getPopulationHistory());
            std::string popDivHistoryCSV = util::PopulationDiversityHistoryToCSV(O.getPopulationHistory());
            util::WriteCSV(io.out_pop_fit_file,popFitHistoryCSV);
            util::WriteCSV(io.out_pop_div_file,popDivHistoryCSV);
            util::WriteCSV(io.out_sol_fit_file,fitnessHistoryCSV);
        }   else {
            printf("Something Failed\n");
        }


		for(uint i = 0; i < solutions.size(); i++) {
			if(snprintf(io.out_sol_file,sizeof(io.out_sol_file),"%s/solution_%i_%i.csv",io.out_dir,N,i) < (int) sizeof(io.out_sol_file)){
				std::string solutionCSV = util::SolutionToCSV<ROBOT_TYPE>(solutions[i]);
				util::WriteCSV(io.out_sol_file,solutionCSV);
			} else {
				printf("Something went terribly wrong");
			}
		}

		printf("Run %i Success\n", N);
		O.reset();
	}
	return O.getSolutions();
}
#endif

#ifdef VIDEO

#ifdef WRITE_VIDEO
void Render(std::vector<ROBOT_TYPE>& robots) {
	printf("RENDERING\n");

	Camera camera(WIDTH, HEIGHT, glm::vec3(0.0f, 0.75f, 5.0f), 0.75f, robots.size());
	// Camera camera(WIDTH, HEIGHT, glm::vec3(-1.5f, 10.0f, 10.0f), 5.0f);
	// Camera camera(WIDTH, HEIGHT, glm::vec3(4.0f, 10.0f, 15.0f), 20.0f);
	// camera = Camera(WIDTH, HEIGHT, glm::vec3(5.0f, 5.0f, 30.0f), 1.0f);

	ROBOT_TYPE R = robots[camera.tabIdx];
	RobotModel<ROBOT_TYPE> model(R);

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
		ROBOT_TYPE R = robots[tabId];
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

void Visualize(std::vector<ROBOT_TYPE>& robots) {
	printf("VISUALIZING\n");

	// R.printObjectPositions();

	GLFWwindow* window = GLFWsetup(true);
	Shader shader("../shaders/vert.glsl", "../shaders/frag.glsl");
	shader.Bind();
	Camera camera(WIDTH, HEIGHT, glm::vec3(0.0f, 0.75f, 5.0f), 0.75f, robots.size());
	// Camera camera(WIDTH, HEIGHT, glm::vec3(-1.5f, 5.0f, 15.0f), 5.0f);

	ROBOT_TYPE R = robots[camera.tabIdx];
	RobotModel<ROBOT_TYPE> model(R);

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
    for(int i = 0; i < argc; i++) {
        if(strcmp(argv[i], "-mutate") == 0) {
			strats.mutator = Optimizer<ROBOT_TYPE>::MUTATE;
			if(i < argc) {
				if(strcmp(argv[i+1], "full") == 0) {
					strats.mutator = Optimizer<ROBOT_TYPE>::RANDOMIZE;
				} else if(strcmp(argv[i+1], "vox") == 0) {
					strats.mutator = Optimizer<ROBOT_TYPE>::MUTATE;
				}
			}
        }

        if(strcmp(argv[i], "-cross") == 0) {
			strats.crossover = Optimizer<ROBOT_TYPE>::CROSS_SWAP;

            if(strcmp(argv[i+1], "none") == 0) {
				strats.crossover = Optimizer<ROBOT_TYPE>::CROSS_NONE;
            } else if(strcmp(argv[i+1], "beam") == 0) {
                strats.crossover = Optimizer<ROBOT_TYPE>::CROSS_BEAM;
            } else if(strcmp(argv[i+1], "swap") == 0) {
                strats.crossover = Optimizer<ROBOT_TYPE>::CROSS_SWAP;
            } else if(strcmp(argv[i+1], "dc") == 0) {
                strats.crossover = Optimizer<ROBOT_TYPE>::CROSS_DC;
            }
        }

        if(strcmp(argv[i], "-niche") == 0) {
            if(strcmp(argv[i+1], "none") == 0) {
                strats.niche = Optimizer<ROBOT_TYPE>::NICHE_NONE;
            } else if(strcmp(argv[i+1], "hfc") == 0) {
                strats.niche = Optimizer<ROBOT_TYPE>::NICHE_HFC;
            } else if(strcmp(argv[i+1], "malps") == 0) {
                strats.niche = Optimizer<ROBOT_TYPE>::NICHE_MALPS;
            }
        }
		
		if(strcmp(argv[i], "-run") == 0) {
			runID = atoi(argv[i+1]);
        }

		if(strcmp(argv[i], "-sol") == 0) {
			solID = atoi(argv[i+1]);
        }

		// if(strcmp(argv[i], "-encode") == 0) {
        //     if(strcmp(argv[i+1], "rad") == 0) {
		// 		strats.encoding = ROBOT_TYPE::ENCODE_RADIUS;
		// 		ROBOT_TYPE::repr = ROBOT_TYPE::ENCODE_RADIUS;
		// 	} else if(strcmp(argv[i+1], "direct") == 0) {
		// 		strats.encoding = ROBOT_TYPE::ENCODE_DIRECT;
		// 		ROBOT_TYPE::repr = ROBOT_TYPE::ENCODE_DIRECT;
		// 	}
        // }
    }
}

void handle_file_io() {
    char* out_dir = io.out_dir;

	strcpy(out_dir,"../z_results");

	#ifdef OPTIMIZE
    switch(strats.niche)
    {
    case Optimizer<ROBOT_TYPE>::NICHE_NONE:
        strcat(out_dir,"/NoNiche");
        break;
    case Optimizer<ROBOT_TYPE>::NICHE_HFC:
        strcat(out_dir,"/HFC");
        break;
    case Optimizer<ROBOT_TYPE>::NICHE_MALPS:
        strcat(out_dir,"/MALPS");
        break;
    default:
        strcat(out_dir,"/NoNiche");
    }

    mkdir(out_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    switch(strats.mutator)
    {
    case Optimizer<ROBOT_TYPE>::RANDOMIZE:
        strcat(out_dir,"/Random");
        break;
    case Optimizer<ROBOT_TYPE>::MUTATE:
        strcat(out_dir,"/Mutate");
        break;
    default:
        strcat(out_dir,"/RandomSearch");
    }

    mkdir(out_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    switch(strats.crossover)
    {
    case Optimizer<ROBOT_TYPE>::CROSS_NONE:
        strcat(out_dir,"/Nonparallel");
        break;
    case Optimizer<ROBOT_TYPE>::CROSS_BEAM:
        strcat(out_dir,"/Parallel");
        break;
    case Optimizer<ROBOT_TYPE>::CROSS_SWAP:
        strcat(out_dir,"/Swap");
        break;
    case Optimizer<ROBOT_TYPE>::CROSS_DC:
        strcat(out_dir,"/DC");
        break;
    }
	#else
	strcat(out_dir,"/z_tests");
	#endif


    mkdir(out_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    // switch(strats.encoding)
    // {
    // case ROBOT_TYPE::ENCODE_DIRECT:
    //     strcat(out_dir,"/Simple");
    //     break;
    // case ROBOT_TYPE::ENCODE_RADIUS:
	// default:
    //     strcat(out_dir,"/Radius");
    //     break;
    // }

    mkdir(out_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
}