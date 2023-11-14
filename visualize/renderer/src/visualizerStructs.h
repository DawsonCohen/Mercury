#ifndef __EVENTS_H__
#define __EVENTS_H__

enum EventType {
    EVENT_TAB_PRESSED,
    EVENT_SPACE_PRESSED,
    EVENT_P_PRESSED,
    EVENT_I_PRESSED,
    EVENT_UP_PRESSED,
    EVENT_DOWN_PRESSED
};

struct InputState {
    bool isWPressed = false;
    bool isAPressed = false;
    bool isSPressed = false;
    bool isDPressed = false;
    bool isSpacePressed = false;
    bool isLeftAltPressed = false;
    bool isLeftCtrlPressed = false;
    bool isLeftShiftPressed = false;
    bool isRightShiftPressed = false;
    bool isLeftMousePressed = false;

    int windowWidth = 1600;
    int windowHeight = 900;

    double mouseX = 0.0f;
    double mouseY = 0.0f;
};

#endif