#pragma once

#include <config.hpp>

#include <Adafruit_NeoPixel.h>

class Lights
{
    struct State
    {
        bool indicator_left = false;
        bool indicator_right = false;
        bool brake = false;
        bool headlight = false;
        bool party = false;
    } state;

    enum class Direction
    {
        forward,
        reverse
    };

    uint8_t tick = 0;
public:

    void init();
    void clear();
    void blinkInfo(const uint32_t color, const bool reverse = false);


    void setIndicatorLeft(bool val);
    void setIndicatorRight(bool val);
    void setBrake(bool val);
    void setHeadlight(bool val);
    void setParty(bool val);

    void update();
private:
    void setColorSide(const LightPos& pos, const uint32_t color, const Direction dir, const uint8_t num = 0xFF, bool overwrite = false);
};
