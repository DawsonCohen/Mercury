#include "application.h"

#define min(a,b) a < b ? a : b
#define max(a,b) a > b ? a : b

Application::Application(std::vector<SoftBody> robots, VisualizerConfig config) :
    currentState(State::Playing),
    config(config)
{
    Configure(config);
    createWindow(config.renderer.width, config.renderer.height);
    
    glfwSetWindowUserPointer(window, &eventSystem);

    camera = Camera(config.renderer.width, config.renderer.height);
    renderer = Renderer("./shaders/vert.glsl", "./shaders/frag.glsl");
    videoWriter = VideoWriter(config.io.out_dir + "/solutions_0.avi",config.renderer.width, config.renderer.height, 30);

    std::vector<RobotModel> assets;
    for(Element& R : robots) {
        assets.push_back(RobotModel(R));
    }
    assetManager.loadAssets(assets);

    // Handle triggered events
    eventSystem.subscribe(EVENT_TAB_PRESSED, [this] {
        assetManager.switchToNextAsset();
    });

    eventSystem.subscribe(EVENT_I_PRESSED, [this] {
        dragVis = !dragVis;
    });

    eventSystem.subscribe(EVENT_UP_PRESSED, [this] {
        sim_speed = min(2.0f, sim_speed * 2.0f);
        printf("%f\n",sim_speed);
    });

    eventSystem.subscribe(EVENT_DOWN_PRESSED, [this] {
        sim_speed = max(pow(2,-15), sim_speed / 2.0f);
        printf("%f\n",sim_speed);
    });
}


void Application::run()
{
	printf("VISUALIZING\n");

    inputManager.setWindow(window);
    inputManager.startInputThread();

    Simulator sim;
    sim.Initialize(config.simulator);
    sim.Reset();

    float time_step = 1 / config.renderer.fps;
    uint devo_cycles = config.devo.devo_cycles;
    float devo_time = config.devo.devo_time;
    float timeToDevo = devo_time;
    
    std::vector<Element> robot_elements(1);
    std::vector<ElementTracker> trackers;
    std::vector<Element> results;
    RobotModel R = assetManager.getCurrentAsset();
    robot_elements[0] = R;
    trackers = sim.SetElements(robot_elements);
    float prevTime = glfwGetTime();
    
    cv::Mat frame(windowHeight, windowWidth, CV_8UC3);
    int frame_period = 1.0 / config.renderer.fps;
    std::vector<RobotModel::RobotMeshGroup> drawgroups = {RobotModel::MESH_GROUP_BODY};
    while (!glfwWindowShouldClose(window)) {
        InputState state = inputManager.getState();
        camera.UpdateCameraPosition(state);

        handleAssetChange(assetManager, R, sim, robot_elements, trackers);

        if(dragVis) {
            drawgroups = { RobotModel::MESH_GROUP_BODY, RobotModel::MESH_GROUP_DRAG };
        } else {
            drawgroups = { RobotModel::MESH_GROUP_BODY };
        }
        
        if(devo_cycles > 0 && timeToDevo <= 0.0f) {
            devo_cycles--;
            timeToDevo = devo_time;
            sim.Devo();
            results = sim.Collect(trackers);
            R.Update(results[0], drawgroups);
        } else if(devo_cycles > 0) {
            timeToDevo -= time_step;
        }

        float crntTime = glfwGetTime();
        sim.Simulate(sim_speed * time_step);
        results = sim.Collect(trackers);
        R.Update(results[0], drawgroups);

        while ((crntTime - prevTime) < frame_period) {
            crntTime = glfwGetTime();
        }
        prevTime = crntTime;

        // Set the clear color
        GLCall(glClearColor(0.1f, 0.1f, 0.1f, 1.0f));
        // Clear the color and depth buffers
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        renderer.Render(camera, &R);

        if(config.visualizer.writeVideo) {
            captureFrame(frame);
            videoWriter.writeFrame(frame);
        }

        glfwSwapBuffers(window);
    }
}

void Application::createWindow(int width, int height) {
    windowWidth = width;
    windowHeight = height;

    // Create the window and set its context current
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        exit(EXIT_FAILURE);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_VISIBLE, interactive ? GLFW_TRUE : GLFW_FALSE);

    window = glfwCreateWindow(width, height, "Evo Devo", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(window);
    glfwSetWindowUserPointer(window, &eventSystem);

    // Load GLAD so it configures OpenGL
    gladLoadGL();
    GLFWinitialize(width, height);

    if (!gladLoadGL()) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        exit(EXIT_FAILURE);
    }
}

void Application::GLFWinitialize(int width, int height)
{
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);		// This Will Clear The Background Color To Black
    glClearDepth(1.0);				// Enables Clearing Of The Depth Buffer

    glEnable(GL_DEPTH_TEST);		        // Enables Depth Testing
    glEnable(GL_LINE_SMOOTH);

    // Specify the viewport of OpenGL in the Window
    // In this case the viewport goes from x = 0, y = 0, to x = config.renderer.width, y = config.renderer.height
    glViewport(0, 0, width, height);
}

void Application::GLFWterminate(GLFWwindow* window) {
    // Delete all the objects we've created
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LINE_SMOOTH);

    // Delete window before ending the program
    glfwDestroyWindow(window);
    // Terminate GLFW before ending the program
    glfwTerminate();
}
