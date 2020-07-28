#include <ledstuff.hpp>

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(num_pixels, 2, NEO_GRB + NEO_KHZ800);

void Lights::init()
{
    strip.begin();
}

void Lights::clear()
{
    tick = 0;
    for(uint8_t i = 0; i < num_pixels; i++)
    {
        strip.setPixelColor(i, 0);
    }
    strip.show();
}

void Lights::blinkInfo(const uint32_t color, const bool reverse)
{
    if(reverse)
        strip.setPixelColor(front_left.len - tick, color);
    else
        strip.setPixelColor(tick, color);

    if(++tick > front_left.len)
    {
        tick = 0;
        for(uint8_t i = 0; i < front_left.len; i++)
        {
            strip.setPixelColor(i, 0);
        }
    }

    strip.show();
}

void Lights::setIndicatorLeft(bool val){state.indicator_left = val;};
void Lights::setIndicatorRight(bool val){state.indicator_right = val;};
void Lights::setBrake(bool val){state.brake = val;};
void Lights::setHeadlight(bool val){state.headlight = val;};
void Lights::setParty(bool val){state.party = val;};

void Lights::update()
{
    const uint8_t ticks_per_indication = 0b100000;
    bool indicator = (tick & ticks_per_indication);
    //indicators
    setColorSide(front_left, state.indicator_left && indicator ? 0xFFFF00 : 0, Direction::reverse, (tick << 1) & 0b111111);
    setColorSide(rear_left , state.indicator_left && indicator ? 0xFFFF00 : 0, Direction::forward, (tick << 1) & 0b111111);

    setColorSide(front_right, state.indicator_right && indicator ? 0xFFFF00 : 0, Direction::forward, (tick << 1) & 0b111111);
    setColorSide(rear_right , state.indicator_right && indicator ? 0xFFFF00 : 0, Direction::reverse, (tick << 1) & 0b111111);

    if(state.headlight)
    {   //For headlights, turning signal has priority
        if(!state.indicator_left)
        {
            setColorSide(front_left, 0xFFFFFF, Direction::reverse, front_left.len/2);
            setColorSide(rear_left , 0x800101, Direction::forward, rear_left.len/2);
        }
        if(!state.indicator_right)
        {
            setColorSide(front_right, 0xFFFFFF, Direction::forward, front_right.len/2);
            setColorSide(rear_right , 0x800101, Direction::reverse, rear_right.len/2);
        }
    }

    if(state.brake)
    {   //brake has prio
        setColorSide(rear_right, 0xFF0000, Direction::reverse, rear_right.len/2);
        setColorSide(rear_left , 0xFF0000, Direction::forward, rear_left.len/2);
    }

    if(state.party)
    {
        //Wohoo, party!
        strip.setPixelColor(tick % num_pixels, 0xAA00BB);
    }

    tick++;


    strip.show();
}


void Lights::setColorSide(const LightPos& pos, const uint32_t color, const Direction dir, const uint8_t num)
{
    uint8_t len = num > pos.len ? pos.len : num;
    if(dir == Direction::forward)
    {
        for(uint8_t i = pos.offs; i < pos.offs + len; i++) {
            strip.setPixelColor(i, color);
        }
        for(uint8_t i = pos.offs + len; i < pos.offs + pos.len; i++) {
            strip.setPixelColor(i, 0);
        }
    }
    else
    {
        for(int16_t i = pos.offs + pos.len; i > pos.offs + (pos.len - len); i--) {
            strip.setPixelColor(i, color);
        }
        for(int16_t i = pos.offs + (pos.len - len); i >= pos.offs; i--) {
            strip.setPixelColor(i, 0);
        }
    }

}
