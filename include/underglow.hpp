#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include "hitlib/hitapi.hpp"

// ============================================================================
//  UNDERGLOW  -  two WS2812B LED strips driven with HitLib
//  (https://github.com/advisorylabs/HitLib, by 96671H Hitmen)
//
//  The animation is picked on the brain's LEDS tab: one for during the match
//  and one for before / after it. The list of animations is in
//  src/underglowAnimations.cpp.
// ============================================================================
namespace underglow {

// ---------------------------------------------------------------- wiring ----
inline constexpr uint8_t LEFT_PORT = 1;  // 3-wire port A
inline constexpr uint8_t RIGHT_PORT = 2; // 3-wire port B

// !! Count the LEDs on one strip and put the number here (max 64 per strip).
// Too low leaves the end of the strip dark, too high wastes a little time.
inline constexpr uint8_t LEDS_PER_SIDE = 30;

// If one strip's patterns run the wrong way compared to the other (its wire
// comes in at the other end), set that side to true.
inline constexpr bool LEFT_REVERSED = false;
inline constexpr bool RIGHT_REVERSED = false;

// 0-100. The strips are powered from the brain's 3-wire ports, so keep this
// down unless they're on their own power.
inline constexpr uint8_t BRIGHTNESS = 50;

// ------------------------------------------------------------ animations ----
struct Animation {
    const char* name; // shown on the LEDS tab
    // sets the strips going (HitLib keeps it running in the background)
    std::function<void(hitlib::LedGroup& strips)> start;
    // the LED colour of pixel `pixel`, `ms` after start. The brain's preview draws this
    std::function<uint32_t(int pixel, uint32_t ms)> look;
    // restart it when the alliance colour changes
    bool followsAlliance = false;
};

/** the list, in src/underglowAnimations.cpp */
extern const std::vector<Animation> animations;

// Ready-made animations for the list. Colours are normal screen colours
// (0xRRGGBB, like theme::YELLOW), they're converted so the LEDs look the same.
Animation solid(const char* name, uint32_t colour);
/**
 * smooth repeating wave: `hold` pixels of a, fades over `fade` pixels to b, holds b, fades back.
 * Moves one pixel every `stepMs`. With throughDark the fade dims a down and brings b up, instead of
 * mixing them (red + green mixed makes yellow, so CHRISTMAS uses it)
 */
Animation wave(const char* name, uint32_t a, uint32_t b, uint8_t hold, uint8_t fade, uint16_t stepMs,
               bool throughDark = false);
/** wave() in the alliance colour picked on the ALLIANCE tab */
Animation allianceWave(const char* name, uint8_t hold, uint8_t fade, uint16_t stepMs);
Animation rainbow(const char* name, uint16_t stepMs);
Animation off(const char* name = "OFF");

/** screen colour -> the value to send to the LEDs so they look like it (LEDs are linear, screens aren't) */
uint32_t toLed(uint32_t screenColour);
/** the other way round, for drawing an LED colour on the brain */
uint32_t toScreen(uint32_t ledColour);

// ------------------------------------------------------------------- use ----
/**
 * Starts the strips. Call once in initialize(), after selector::init().
 * @param idle  animation for before / after the match (index into animations)
 * @param match animation for during the match
 */
void init(int idle, int match);

void setIdle(int index);
void setMatch(int index);
int idleIndex();
int matchIndex();

/** play an animation for a few seconds then go back (the LEDS tab uses this so you can see your pick) */
void tryOut(int index, uint32_t ms = 5000);
void stopTryOut();

enum class Showing { IDLE, MATCH, TRYING };
/** what the strips are showing right now */
Showing showing();

/** checks the match state and switches animation. underglow's own task calls this, you don't need to */
void update();

} // namespace underglow
