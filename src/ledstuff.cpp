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
        tick = 0;
}

void Lights::setIndicatorLeft(bool val){state.indicator_left = val;};
void Lights::setIndicatorRight(bool val){state.indicator_right = val;};
void Lights::setBrake(bool val){state.brake = val;};
void Lights::setHeadlight(bool val){state.headlight = val;};
void Lights::setParty(bool val){state.party = val;};

void Lights::update()
{
    bool indicator = (tick & 0b100);
    setColorSide(front_left, state.indicator_left && indicator ? 0xFFFF00 : 0);
    setColorSide(rear_left , state.indicator_left && indicator ? 0xFFFF00 : 0);

    setColorSide(front_right, state.indicator_right && indicator ? 0xFFFF00 : 0);
    setColorSide(rear_right , state.indicator_right && indicator ? 0xFFFF00 : 0);

    if(state.headlight)
    {   //For headlights, turning signal has priority
        if(!state.indicator_left)
        {
            setColorSide(front_left , 0xFFFFFF, Division::second_half);
        }
        if(!state.indicator_right)
        {
            setColorSide(front_right, 0xFFFFFF, Division::first_half);
        }
    }

    if(state.brake)
    {   //brake has prio
        setColorSide(rear_right, 0xFF0000, Division::second_half);
        setColorSide(rear_left , 0xFF0000, Division::first_half);
    }

    if(state.party)
    {
        //Wohoo, party!
        strip.setPixelColor(tick % num_pixels, 0xAA00BB);
    }

    tick++;


    strip.show();
}


void Lights::setColorSide(const LightPos& pos, const uint32_t color, const Division div)
{
    uint8_t start = div != Division::second_half ? pos.offs : pos.offs + pos.len/2;
    uint8_t len   = div == Division::all ? pos.len : pos.len/2;
    for(uint8_t i = start; i < start + len; i++) {
        strip.setPixelColor(i, color);
    }
}
