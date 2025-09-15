#pragma once

#include <stdint.h>

// Teensy Mouse emulation API
// This is a stub for PlatformIO/Teensyduino
// Include <Mouse.h> in your main file to use Mouse.move(), Mouse.click(), etc.

class Mouse_ {
public:
    void begin() {}
    void end() {}
    void move(int x, int y, int wheel = 0) {}
    void click(uint8_t button = 1) {}
    void press(uint8_t button = 1) {}
    void release(uint8_t button = 1) {}
    bool isPressed(uint8_t button = 1) { return false; }
};

extern Mouse_ Mouse;
