#include "Simulator.h"
#include "optimizer.h"
#include "plane.h"
#include "util.h"

#include "Renderer.h"

#ifdef WRITE_VIDEO
#include <opencv2/opencv.hpp>
#endif

#include <thread>
#include <chrono>
#include <iostream>
#include <sys/stat.h>

#define WIDTH	1600
#define HEIGHT	900
#define FPS		30.0f

#define MAX_FILE_PATH (int) 500
#define REPEATS 1

#define BASE_TIME 5.0f
#define MAX_TIME 10.0f

#define MAX_EVALS (ulong) 1e6

#define POP_SIZE (uint) 256
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
    Optimizer::MutationStrat mutator = Optimizer::MUTATE;
    Optimizer::CrossoverStrat crossover = Optimizer::CROSS_SWAP;
    Optimizer::NichingStrat niche = Optimizer::NICHE_NONE;
    Robot::Encoding encoding = Robot::ENCODE_RADIUS;
};

void handle_commandline_args(const int& argc, char** argv);
void handle_file_io();
GLFWwindow* GLFWsetup(bool visualize);
void GLFWinitialize();

std::vector<Robot> Solve();
void Render(Robot& R);
void Visualize(Robot& R);

OptimizationStrats strats;
IOLocations io;
Simulator sim;

int main(int argc, char** argv)
{
	handle_commandline_args(argc, argv);
	handle_file_io();
	printf("Output directory: %s\n",io.out_dir);

	std::vector<Robot> solutions;
	Evaluator::Initialize(POP_SIZE, BASE_TIME, MAX_TIME);
	
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
		solutions.push_back(ReadRobot(filepath));

		sol++;
	}
	#endif

	// TODO
	#ifdef ZOO
	strcpy(io.out_dir, "../z_results/Zoo");

	SoftBody zoo;

	char best_files[9][MAX_FILE_PATH];
	strcpy(best_files[0], "../z_results/MALPS/Mutate/DC/Simple/solution_0.csv");
	strcpy(best_files[1], "../z_results/MALPS/Mutate/DC/Simple/solution_1.csv");
	strcpy(best_files[2], "../z_results/MALPS/Mutate/DC/Simple/solution_2.csv");

	strcpy(best_files[3], "../z_results/NoNiche/Random/Nonparallel/Simple/solution_0.csv");
	strcpy(best_files[4], "../z_results/NoNiche/Random/Nonparallel/Simple/solution_1.csv");
	strcpy(best_files[5], "../z_results/NoNiche/Random/Nonparallel/Simple/solution_2.csv");

	strcpy(best_files[6], "../z_results/NoNiche/Mutate/Swap/Simple/solution_0.csv");
	strcpy(best_files[7], "../z_results/NoNiche/Mutate/Swap/Simple/solution_1.csv");
	strcpy(best_files[8], "../z_results/NoNiche/Mutate/Swap/Simple/solution_2.csv");
	
	for(uint i = 0; i < 9; i++) {
		Robot sol = ReadRobot(best_files[i]);
		float x = (i % 3) * 5.0f;
		float z = (i/3) * 5.0f;
		sol.translate(glm::vec3(x, 0.0f, -z));
		zoo.append(sol);
	}

	zoo = Robot();

	printf("Total Masses: %lu\n",zoo.getMasses().size());
	printf("Total Springs: %lu\n",zoo.getSprings().size());

	solution = zoo;
	#endif

	#if !defined(OPTIMIZE) && !defined(VERIFY) && !defined(ZOO)
	Robot solution = Robot();
	uint seed = std::chrono::system_clock::now().time_since_epoch().count();
    srand(seed);
	solution.Randomize();
	solutions.push_back(solution);
	#endif

	#ifdef BOUNCE
	for(Robot& R : solutions) {
		R.translate(glm::vec3(0.0f,7.0f,0.0f));
	}
	#endif

	#ifdef WRITE_VIDEO
	Render(solutions[solID]);
	#endif

	#ifdef VISUALIZE
	Visualize(solutions[solID]);
	#endif

	return 0;
}

std::vector<Robot> Solve() {
	Optimizer O;
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
    
    RemoveOldFiles(io.out_dir);
	
    for(unsigned N = 0; N < REPEATS; N++) {
        printf("Started Run %i\n",N);
		std::vector<Robot> solutions = O.Solve();

		printf("SOLUTIONS: %lu\n", solutions.size());

		if(snprintf(io.out_sol_fit_file,sizeof(io.out_sol_fit_file),"%s/solution_history_%i.csv",io.out_dir,N) < (int) sizeof(io.out_sol_fit_file)
            && snprintf(io.out_pop_fit_file,sizeof(io.out_pop_fit_file),"%s/fitness_history_%i.csv",io.out_dir,N) < (int) sizeof(io.out_pop_fit_file)
			&& snprintf(io.out_pop_div_file,sizeof(io.out_pop_div_file),"%s/diversity_history_%i.csv",io.out_dir,N) < (int) sizeof(io.out_pop_div_file)){
            
            printf("Writing History\n");
			
            std::string fitnessHistoryCSV = FitnessHistoryToCSV(O.getFitnessHistory());
            std::string popFitHistoryCSV = PopulationFitnessHistoryToCSV(O.getPopulationHistory());
            std::string popDivHistoryCSV = PopulationDiversityHistoryToCSV(O.getPopulationHistory());
            WriteCSV(io.out_pop_fit_file,popFitHistoryCSV);
            WriteCSV(io.out_pop_div_file,popDivHistoryCSV);
            WriteCSV(io.out_sol_fit_file,fitnessHistoryCSV);
        }   else {
            printf("Something Failed\n");
        }


		for(uint i = 0; i < solutions.size(); i++) {
			if(snprintf(io.out_sol_file,sizeof(io.out_sol_file),"%s/solution_%i_%i.csv",io.out_dir,N,i) < (int) sizeof(io.out_sol_file)){
				std::string solutionCSV = SolutionToCSV(solutions[i]);
				WriteCSV(io.out_sol_file,solutionCSV);
			} else {
				printf("Something went terribly wrong");
			}
		}

		printf("Run %i Success\n", N);
		O.reset();
	}
	return O.getSolutions();
}

void Render(Robot& R) {
	printf("RENDERING\n");

	GLFWwindow* window = GLFWsetup(false);
	Shader shader("../shaders/vert.glsl", "../shaders/frag.glsl");
	shader.Bind();
	Camera camera(WIDTH, HEIGHT, glm::vec3(-1.5f, 10.0f, 10.0f), 5.0f);
	// Camera camera(WIDTH, HEIGHT, glm::vec3(4.0f, 10.0f, 15.0f), 20.0f);

	R.Bind();

	shader.Unbind();
	
	#ifdef WRITE_VIDEO
	// opencv video writer
	cv::VideoWriter video;

	camera = Camera(WIDTH, HEIGHT, glm::vec3(5.0f, 5.0f, 30.0f), 1.0f);

	#if defined(BOUNCE) && !defined(ENV_GRAVITY)
	if(snprintf(io.out_sol_video_file,sizeof(io.out_sol_video_file),"%s/stationary_%u_%u.avi",io.out_dir, runID, solID))
	#elif defined(BOUNCE)
	if(snprintf(io.out_sol_video_file,sizeof(io.out_sol_video_file),"%s/bounce_%u_%u.avi",io.out_dir, runID, solID))
	#elif defined(ZOO)
	if(snprintf(io.out_sol_video_file,sizeof(io.out_sol_video_file),"%s/zoo_0.avi",io.out_dir))
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
	#endif

	Plane p;

	float max_render_time = 15;
	sim.setMaxTime(1/ FPS);

	// Main while loop
	while (!glfwWindowShouldClose(window) && sim.getTotalTime() < max_render_time)
	{
		std::vector<Element> robot_elements(1);
		robot_elements[0] = (Element) R;

		std::vector<ElementTracker> trackers = sim.Simulate(robot_elements);
		std::vector<Element> results = sim.Collect(trackers);
		R.Update(results[0]);

		printf("%f\n", sim.getTotalTime());

		// Specify the color of the background
		GLCall(glClearColor(0.73f, 0.85f, 0.92f, 1.0f));
		// Clean the back buffer and depth buffer
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Handles camera inputs
		camera.Inputs(window);
		// Updates and exports the camera matrix to the Vertex Shader
		camera.updateMatrix(45.0f, 0.1f, 100.0f);

		R.Draw(shader, camera);

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

void Visualize(Robot& R) {
	printf("VISUALIZING\n");

	// R.printObjectPositions();

	GLFWwindow* window = GLFWsetup(true);
	Shader shader("../shaders/vert.glsl", "../shaders/frag.glsl");
	shader.Bind();
	Camera camera(WIDTH, HEIGHT, glm::vec3(5.0f, 5.0f, 30.0f), 1.0f);
	// Camera camera(WIDTH, HEIGHT, glm::vec3(-1.5f, 5.0f, 15.0f), 5.0f);

	R.Bind();

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

	Plane p;

	float prevTime = glfwGetTime();

	sim.Initialize(R, 1);
	sim.setMaxTime( 1 / FPS);

	// Main while loop
	uint i = 0;
	std::vector<Element> robot_elements(1);

	// printf("-----------------------\n");
	
	while (!glfwWindowShouldClose(window))
	{
		// printf("Iteration: %u\n", i);
		float crntTime = glfwGetTime();
		
		robot_elements[0] = {R.getMasses(),R.getSprings()};
		std::vector<ElementTracker> trackers = sim.Simulate(robot_elements);
		std::vector<Element> results = sim.Collect(trackers);
		
		// for(uint i = 0; i < results[0].masses.size(); i++) {
		// 	Mass m = results[0].masses[i];
		// 	printf("Mass %u: (%f,%f,%f)\n", i, m.pos.x, m.pos.y, m.pos.z);
		// }

		R.Update(results[0]);

		// R.printMesh();
		
		// R.printObjectPositions();

		while ((crntTime - prevTime) < 1 / FPS) {
			crntTime = glfwGetTime();
		}

		// Simple timer
		prevTime = crntTime;
		
		// Specify the color of the background
		GLCall(glClearColor(0.73f, 0.85f, 0.92f, 1.0f));
		// Clean the back buffer and depth buffer
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Handles camera inputs
		camera.Inputs(window);
		// Updates and exports the camera matrix to the Vertex Shader
		camera.updateMatrix(45.0f, 0.1f, 100.0f);

		R.Draw(shader, camera);

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

void handle_commandline_args(const int& argc, char** argv) {
    for(int i = 0; i < argc; i++) {
        if(strcmp(argv[i], "-mutate") == 0) {
			strats.mutator = Optimizer::MUTATE;
			if(i < argc) {
				if(strcmp(argv[i+1], "full") == 0) {
					strats.mutator = Optimizer::RANDOMIZE;
				} else if(strcmp(argv[i+1], "vox") == 0) {
					strats.mutator = Optimizer::MUTATE;
				}
			}
        }

        if(strcmp(argv[i], "-cross") == 0) {
			strats.crossover = Optimizer::CROSS_SWAP;

            if(strcmp(argv[i+1], "none") == 0) {
				strats.crossover = Optimizer::CROSS_NONE;
            } else if(strcmp(argv[i+1], "beam") == 0) {
                strats.crossover = Optimizer::CROSS_BEAM;
            } else if(strcmp(argv[i+1], "swap") == 0) {
                strats.crossover = Optimizer::CROSS_SWAP;
            } else if(strcmp(argv[i+1], "dc") == 0) {
                strats.crossover = Optimizer::CROSS_DC;
            }
        }

        if(strcmp(argv[i], "-niche") == 0) {
            if(strcmp(argv[i+1], "none") == 0) {
                strats.niche = Optimizer::NICHE_NONE;
            } else if(strcmp(argv[i+1], "hfc") == 0) {
                strats.niche = Optimizer::NICHE_HFC;
            } else if(strcmp(argv[i+1], "malps") == 0) {
                strats.niche = Optimizer::NICHE_MALPS;
            }
        }
		
		if(strcmp(argv[i], "-run") == 0) {
			runID = atoi(argv[i+1]);
        }

		if(strcmp(argv[i], "-sol") == 0) {
			solID = atoi(argv[i+1]);
        }

		if(strcmp(argv[i], "-encode") == 0) {
            if(strcmp(argv[i+1], "rad") == 0) {
				strats.encoding = Robot::ENCODE_RADIUS;
				Robot::repr = Robot::ENCODE_RADIUS;
			} else if(strcmp(argv[i+1], "direct") == 0) {
				strats.encoding = Robot::ENCODE_DIRECT;
				Robot::repr = Robot::ENCODE_DIRECT;
			}
        }
    }
}

void handle_file_io() {
    char* out_dir = io.out_dir;

	strcpy(out_dir,"../z_results");

    switch(strats.niche)
    {
    case Optimizer::NICHE_NONE:
        strcat(out_dir,"/NoNiche");
        break;
    case Optimizer::NICHE_HFC:
        strcat(out_dir,"/HFC");
        break;
    case Optimizer::NICHE_MALPS:
        strcat(out_dir,"/MALPS");
        break;
    default:
        strcat(out_dir,"/NoNiche");
    }

    mkdir(out_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    switch(strats.mutator)
    {
    case Optimizer::RANDOMIZE:
        strcat(out_dir,"/Random");
        break;
    case Optimizer::MUTATE:
        strcat(out_dir,"/Mutate");
        break;
    default:
        strcat(out_dir,"/RandomSearch");
    }

    mkdir(out_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    switch(strats.crossover)
    {
    case Optimizer::CROSS_NONE:
        strcat(out_dir,"/Nonparallel");
        break;
    case Optimizer::CROSS_BEAM:
        strcat(out_dir,"/Parallel");
        break;
    case Optimizer::CROSS_SWAP:
        strcat(out_dir,"/Swap");
        break;
    case Optimizer::CROSS_DC:
        strcat(out_dir,"/DC");
        break;
    }

    mkdir(out_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    switch(strats.encoding)
    {
    case Robot::ENCODE_DIRECT:
        strcat(out_dir,"/Simple");
        break;
    case Robot::ENCODE_RADIUS:
	default:
        strcat(out_dir,"/Radius");
        break;
    }

    mkdir(out_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
}


GLFWwindow* GLFWsetup(bool visualize) {
	// Initialize GLFW
	glfwInit();

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_VISIBLE, visualize ? GLFW_TRUE : GLFW_FALSE);
	// glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT,GLFW_TRUE);

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
	// Lighting properties
	// GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	// GLfloat mat_shininess[] = { 50.0 };
	// GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };

	// // set the lighting properties
	// glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	// glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
	// glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	// glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE ) ;

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);		// This Will Clear The Background Color To Black
	glClearDepth(1.0);				// Enables Clearing Of The Depth Buffer
	// glDepthFunc(GL_LESS);			        // The Type Of Depth Test To Do
	// glShadeModel(GL_SMOOTH);			// Enables Smooth Color Shading


	glEnable(GL_DEPTH_TEST);		        // Enables Depth Testing
	glEnable(GL_LINE_SMOOTH);
	// glEnable (GL_LIGHTING);
	// glEnable (GL_COLOR_MATERIAL);
	// glEnable (GL_LIGHT0);

	// Wireframe mode
	// glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );

	// Rasterized line width
	glLineWidth(3.0f);


	// glMatrixMode(GL_PROJECTION);
	// glLoadIdentity();				// Reset The Projection Matrix

	// Specify the viewport of OpenGL in the Window
	// In this case the viewport goes from x = 0, y = 0, to x = WIDTH, y = HEIGHT
	glViewport(0, 0, WIDTH, HEIGHT);

	// glMatrixMode(GL_MODELVIEW);
}
