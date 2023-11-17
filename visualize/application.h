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
    float sim_speed = 2.0f;

    uint devo_cycles;
    float devo_time;
    float timeToDevo;

    void createWindow(int width, int height);
    void GLFWinitialize(int width, int height);
    void GLFWterminate(GLFWwindow* window);
    void captureFrame(cv::Mat& frame) {
        glPixelStorei(GL_PACK_ALIGNMENT, (frame.step & 3) ? 1 : 4);
        glPixelStorei(GL_PACK_ROW_LENGTH, frame.step / frame.elemSize());
        glReadPixels(0, 0, frame.cols, frame.rows, GL_BGR, GL_UNSIGNED_BYTE, frame.data);
        cv::flip(frame, frame, 0);
    }
    void handleAssetChange(AssetManager& assetManager, RobotModel& R, Simulator& sim, ElementTracker& tracker) {
        if (assetManager.hasAssetChanged()) {
            assetManager.clearAssetChangedFlag();
            R = assetManager.getCurrentAsset();
            sim.Reset();
            tracker = sim.SetElement(R);

            devo_cycles = config.devo.devo_cycles;
            devo_time = config.devo.devo_time;
            timeToDevo = devo_time;
        }
    }
};

#endif