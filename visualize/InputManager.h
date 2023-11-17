#ifndef __INPUT_MANAGER_H__
#define __INPUT_MANAGER_H__

#include <thread>
#include <GLFW/glfw3.h>

#include "EventSystem.h"

class InputManager
{
public:
    InputManager(GLFWwindow *window);

    void startInputThread()
    {
        // Start the input handling thread
        inputThread = std::thread([this] { inputLoop(); });
    }


    InputManager() {}

    ~InputManager()
    {
        inputThread.join();
    }
    
    InputState getState() const {
        return state;
    }

    void setWindow(GLFWwindow *window);

    friend void swap(InputManager& m1, InputManager& m2) {
        using std::swap;
        swap(m1.window, m2.window);
        swap(m1.state, m2.state);
        swap(m1.firstClick, m2.firstClick);
    }

    InputManager& operator=(InputManager src) {
        swap(*this, src);
        return *this;
    }

private:
    GLFWwindow *window;
    std::thread inputThread;

    bool firstClick = false;
    double clickedMousePosX, clickedMousePosY;

    InputState state;

    void inputLoop();
};

#endif