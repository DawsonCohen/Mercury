#include "InputManager.h"
#include "stdio.h"
#include <assert.h>

void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    // unused variables
    (void)scancode;
    (void)mods;

    if(action == GLFW_PRESS) {
        EventSystem *eventSys = static_cast<EventSystem *>(glfwGetWindowUserPointer(window));
        switch(key) {
            case GLFW_KEY_TAB:
                eventSys->trigger(EVENT_TAB_PRESSED);
                break;
            case GLFW_KEY_I:
                eventSys->trigger(EVENT_I_PRESSED);
                break;
            case GLFW_KEY_P:
                eventSys->trigger(EVENT_P_PRESSED);
                break;
            case GLFW_KEY_UP:
                eventSys->trigger(EVENT_UP_PRESSED);
                break;
            case GLFW_KEY_DOWN:
                eventSys->trigger(EVENT_DOWN_PRESSED);
                break;
            case GLFW_KEY_ESCAPE:
                glfwSetWindowShouldClose(window, true);
                break;
            default:
                break;
        }
    }
}

InputManager::InputManager(GLFWwindow *window) : window(window)
{
    // Set up GLFW callbacks for keyboard and mouse input
    glfwSetKeyCallback(window, keyCallback);
}

void InputManager::setWindow(GLFWwindow *window) {
    this->window = window;
    glfwSetKeyCallback(window, keyCallback);
}

void InputManager::inputLoop() {
    while (!glfwWindowShouldClose(window))
        {
            // Handle input in this loop
            glfwPollEvents();
            // You can add more input handling here

            int windowWidth, windowHeight;
            glfwGetWindowSize(window, &windowWidth, &windowHeight);

            // Handles movement key inputs
            state.windowWidth = windowWidth;
            state.windowHeight = windowHeight;
            state.isWPressed = (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS);
            state.isAPressed = (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS);
            state.isSPressed = (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS);
            state.isDPressed = (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS);
            state.isSpacePressed = (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS);
            state.isLeftCtrlPressed = (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS);
            state.isLeftAltPressed = (glfwGetKey(window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS);
            state.isLeftShiftPressed = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS);
            state.isRightShiftPressed = (glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

            state.isLeftMousePressed = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
            if(state.isLeftMousePressed) {
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);

                // Prevents camera from jumping on the first click
                if (firstClick)
                {
                    firstClick = false;
                    glfwGetCursorPos(window, &clickedMousePosX, &clickedMousePosY);
                    glfwSetCursorPos(window, (windowWidth / 2), (windowHeight / 2));
                }

                // Stores the coordinates of the cursor
                double mouseX;
                double mouseY;
                // Fetches the coordinates of the cursor
                glfwGetCursorPos(window, &mouseX, &mouseY);

                state.mouseX = mouseX;
                state.mouseY = mouseY;

                // Sets mouse cursor to the middle of the screen so that it doesn't end up roaming around
                // glfwSetCursorPos(window, (windowWidth / 2), (windowHeight / 2));
            }
            if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE)
            {
                firstClick = true;
                // Unhides cursor since camera is not looking around anymore
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            }
        }
}