#include "autonSelector.hpp"
#include "underglow.hpp"

// ============================================================================
//  UNDERGLOW ANIMATIONS  -  the list on the brain's LEDS tab
//
//  Colours are normal screen colours (0xRRGGBB). The ready-made kinds are:
//    underglow::solid(name, colour)
//    underglow::wave(name, colourA, colourB, hold, fade, stepMs, throughDark)
//        hold:   pixels of each colour
//        fade:   pixels it takes to blend from one to the other
//        stepMs: how often it moves along one pixel (bigger = slower)
//        throughDark (optional): fade A down and B up instead of mixing them
//    underglow::allianceWave(name, hold, fade, stepMs)
//    underglow::rainbow(name, stepMs)
//    underglow::off()
//
//  For anything else HitLib can do (pulse, flash, twinkle...), write your own:
//    {"SPARKLE",
//     [](hitlib::LedGroup& g) { g.twinkle({0xFFFFFF, 0xEEA60F}, 30); },  // what the LEDs do
//     [](int, uint32_t) { return 0xEEA60Fu; }},                         // what the brain previews
// ============================================================================

namespace underglow {

const std::vector<Animation> animations = {
    solid("LOGO YELLOW", theme::YELLOW),
    wave("CHRISTMAS", 0xFF0000, 0x00FF00, 6, 4, 80, true), // red and green, never mixed into yellow
    wave("YELLOW WAVE", theme::YELLOW, 0x302808, 5, 5, 60),
    allianceWave("ALLIANCE", 5, 5, 60),
    rainbow("RAINBOW", 40),
    off(),
};

} // namespace underglow
