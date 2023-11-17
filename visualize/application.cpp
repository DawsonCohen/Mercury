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

    if(config.visualizer.writeVideo) {
        videoWriter = VideoWriter(config.io.out_dir + "/solutions_0.avi", config.renderer.fps, config.renderer.width, config.renderer.height);
    }

    std::vector<RobotModel> assets;
    for(SoftBody& R : robots) {
        assets.push_back(R);
    }
    assetManager.loadAssets(assets);

    if(headless) return;

    // Handle triggered events
    eventSystem.subscribe(EVENT_TAB_PRESSED, [this] {
        assetManager.switchToNextAsset();
    });

    eventSystem.subscribe(EVENT_I_PRESSED, [this] {
        dragVis = (DragVisState) (((int) dragVis + 1) % 3);
    });

    eventSystem.subscribe(EVENT_UP_PRESSED, [this] {
        sim_speed = min(2.0f, sim_speed * 2.0f);
        printf("%f\n",sim_speed);
    });

    eventSystem.subscribe(EVENT_DOWN_PRESSED, [this] {
        sim_speed = max(pow(2,-32), sim_speed / 2.0f);
        if(sim.getDeltaT() > sim_speed / this->config.renderer.fps) {
            sim.setTimeStep(sim_speed / this->config.renderer.fps);
        }
        printf("%f\n",sim_speed);
    });
}


void Application::run()
{
	printf("VISUALIZING\n");

    inputManager.setWindow(window);
    inputManager.startInputThread();

    sim.Initialize(config.simulator);
    sim.Reset();

    float time_step = 1 / config.renderer.fps;
    devo_cycles = config.devo.devo_cycles;
    devo_time = config.devo.devo_time;
    timeToDevo = devo_time;

    ElementTracker tracker;
    std::vector<Element> results;
    RobotModel R = assetManager.getCurrentAsset();
    Element Relement;
    tracker = sim.SetElement(R);
    float prevTime = glfwGetTime();
    
    std::vector<RobotModel::RobotMeshGroup> drawgroups = {RobotModel::MESH_GROUP_BODY};
    while (!glfwWindowShouldClose(window)) {
        InputState state = inputManager.getState();
        camera.UpdateCameraPosition(state);

        if(headless) {
            if(sim.getTotalTime() > config.visualizer.showcase_time) {
                assetManager.switchToNextAsset();
                if(assetManager.hasWrapped()) {
                    glfwWindowShouldClose(window);
                    break;
                }
            }
        }

        handleAssetChange(assetManager, R, sim, tracker);

        switch(dragVis) {
            case DRAG_VIS_ALL:
                drawgroups = { RobotModel::MESH_GROUP_BODY, RobotModel::MESH_GROUP_DRAG };
                break;
            case DRAG_VIS_ONE:
                drawgroups = { RobotModel::MESH_GROUP_BODY, RobotModel::MESH_GROUP_DRAG_ONE };
                break;
            case DRAG_VIS_NONE:
                drawgroups = { RobotModel::MESH_GROUP_BODY };
        }
        
        if(devo_cycles > 0 && timeToDevo <= 0.0f) {
            devo_cycles--;
            timeToDevo = devo_time;
            sim.Devo();
            Relement = sim.Collect(tracker);
            R.Update(Relement, drawgroups);
        } else if(devo_cycles > 0) {
            timeToDevo -= time_step;
        }

        float crntTime = glfwGetTime();
        sim.Simulate(sim_speed * time_step);
        Relement = sim.Collect(tracker);
        R.Update(Relement, drawgroups);

        if(!headless) {
            while ((crntTime - prevTime) < time_step) {
                crntTime = glfwGetTime();
            }
        }
        prevTime = crntTime;

        // Set the clear color
        GLCall(glClearColor(0.1f, 0.1f, 0.1f, 1.0f));
        // Clear the color and depth buffers
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        renderer.Render(camera, &R);

        if(config.visualizer.writeVideo) {
            cv::Mat frame(windowHeight, windowWidth, CV_8UC3);
            // cv::Mat frame(config.renderer.height,config.renderer.width,CV_8UC3);
            glPixelStorei(GL_PACK_ALIGNMENT, (frame.step & 3) ? 1 : 4);
            glPixelStorei(GL_PACK_ROW_LENGTH, frame.step / frame.elemSize());
            glReadPixels(0, 0, frame.cols, frame.rows, GL_BGR, GL_UNSIGNED_BYTE, frame.data);
            cv::flip(frame, frame, 0);
            videoWriter.writeFrame(frame);
        }

        glfwSwapBuffers(window);
    }

    GLFWterminate(window);
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
    glfwWindowHint(GLFW_VISIBLE, headless ? GLFW_FALSE : GLFW_TRUE);

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
