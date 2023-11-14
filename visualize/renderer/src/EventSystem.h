#ifndef __EVENT_SYSTEM_H__
#define __EVENT_SYSTEM_H__

#include "visualizerStructs.h"

#include <functional>

class EventSystem {
public:
    using EventHandler = std::function<void()>;

    void subscribe(const EventType event, EventHandler handler) {
        eventHandlers[event].push_back(handler);
    }

    void trigger(const EventType event) {
        for (const auto& handler : eventHandlers[event]) {
            handler();
        }
    }

private:
    std::unordered_map<EventType, std::vector<EventHandler>> eventHandlers;
};

#endif