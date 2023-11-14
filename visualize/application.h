#ifndef __APPLICATION_H
#define __APPLICATION_H

#include "Renderer.h"
#include "InputManager.h"
#include "AssetManager.h"
#include "VideoWriter.h"
#include "visualizer_config.h"
#include "SoftBody.h"
#include "robot_model.h"
#include "Simulator.h"

class Application {
    enum DragVisState {
        DRAG_VIS_NONE = 0,
        DRAG_VIS_ONE = 1,
        DRAG_VIS_ALL = 2,
    };
    
public:
    enum class State {
        Playing,
        Paused
    };

    Application(std::vector<SoftBody> robots, VisualizerConfig config = VisualizerConfig());

    void Configure(VisualizerConfig config) {
        if(config.visualizer.dragVis) dragVis = DRAG_VIS_ALL;
        writeVideo = config.visualizer.writeVideo;
        showcaseTime = config.visualizer.showcase_time;
        headless = config.visualizer.headless;
    }


    void run();

private:
    State currentState;
    bool headless;
    VisualizerConfig config;

    Simulator sim;
    EventSystem eventSystem;
    Camera camera;
    Renderer renderer;
    InputManager inputManager;
    AssetManager assetManager;
    VideoWriter videoWriter;
    GLFWwindow* window;

    int windowWidth, windowHeight;
    bool writeVideo = false;
    DragVisState dragVis = DRAG_VIS_NONE;
    float showcaseTime = 5.0f;
    float sim_speed = 1.0f;

    void createWindow(int width, int height);
    void GLFWinitialize(int width, int height);
    void GLFWterminate(GLFWwindow* window);
    void captureFrame(cv::Mat& frame) {
        glPixelStorei(GL_PACK_ALIGNMENT, (frame.step & 3) ? 1 : 4);
        glPixelStorei(GL_PACK_ROW_LENGTH, frame.step / frame.elemSize());
        glReadPixels(0, 0, frame.cols, frame.rows, GL_BGR, GL_UNSIGNED_BYTE, frame.data);
        cv::flip(frame, frame, 0);
    }
    void handleAssetChange(AssetManager& assetManager, RobotModel& R, Simulator& sim, std::vector<Element>& robot_elements, std::vector<ElementTracker>& trackers) {
        if (assetManager.hasAssetChanged()) {
            assetManager.clearAssetChangedFlag();
            R = assetManager.getCurrentAsset();
            robot_elements[0] = {R.masses, R.springs};
            sim.Reset();
            trackers = sim.SetElements(robot_elements);
        }
    }
};

#endif