#include "underglow.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>

#include "autonSelector.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

namespace underglow {
namespace {

hitlib::LedStrand leftStrip({{LEFT_PORT, LEDS_PER_SIDE, LEFT_REVERSED}});
hitlib::LedStrand rightStrip({{RIGHT_PORT, LEDS_PER_SIDE, RIGHT_REVERSED}});
hitlib::LedGroup strips;

// HitLib holds the 3-wire bus for 12 ms per strip it updates, so two moving strips need 24 ms a frame.
// 30 keeps every frame on time (at 20 the animations would run slow, and drift from the brain's preview)
constexpr uint32_t REFRESH_MS = 30;

std::atomic<int> idleChoice{0};
std::atomic<int> matchChoice{0};
std::atomic<int> tryChoice{-1};
std::atomic<uint32_t> tryUntil{0};
std::atomic<Showing> nowShowing{Showing::IDLE};
std::atomic<bool> started{false};

// what's on the strips now, so update() only restarts an animation when something changed
int playing = -1;
selector::Alliance playingAlliance = selector::Alliance::RED;

int clampIndex(int index) {
    if (animations.empty()) return -1;
    return std::clamp(index, 0, (int)animations.size() - 1);
}

uint8_t gammaUp(uint8_t v) { return (uint8_t)std::lround(255.0 * std::pow(v / 255.0, 2.2)); }
uint8_t gammaDown(uint8_t v) { return (uint8_t)std::lround(255.0 * std::pow(v / 255.0, 1 / 2.2)); }

uint32_t mapChannels(uint32_t c, uint8_t (*f)(uint8_t)) {
    return (uint32_t)f((c >> 16) & 0xFF) << 16 | (uint32_t)f((c >> 8) & 0xFF) << 8 | f(c & 0xFF);
}

// blend two screen colours, t = 0..1
uint32_t mix(uint32_t a, uint32_t b, double t) {
    uint32_t out = 0;
    for (int shift = 16; shift >= 0; shift -= 8) {
        double ca = (a >> shift) & 0xFF, cb = (b >> shift) & 0xFF;
        out |= (uint32_t)std::lround(ca + (cb - ca) * t) << shift;
    }
    return out;
}

// a fade from a to b, t = 0..1. throughDark dims a to nothing then brings b up
uint32_t fadeAt(uint32_t a, uint32_t b, double t, bool throughDark) {
    if (!throughDark) return mix(a, b, t);
    return t < 0.5 ? mix(a, 0, t * 2) : mix(0, b, t * 2 - 1);
}

// one repeat of a wave, as LED colours: a held, fade to b, b held, fade back to a.
// The fade is worked out on screen colours, so to the eye it's an even crossfade
std::vector<uint32_t> waveUnit(uint32_t a, uint32_t b, uint8_t hold, uint8_t fade, bool throughDark = false) {
    std::vector<uint32_t> unit;
    for (int i = 0; i < hold; i++) unit.push_back(toLed(a));
    for (int i = 1; i <= fade; i++) unit.push_back(toLed(fadeAt(a, b, i / (fade + 1.0), throughDark)));
    for (int i = 0; i < hold; i++) unit.push_back(toLed(b));
    for (int i = 1; i <= fade; i++) unit.push_back(toLed(fadeAt(b, a, i / (fade + 1.0), throughDark)));
    if (unit.empty()) unit.push_back(toLed(a));
    return unit;
}

// plays a wave with HitLib's bitscroll: no gaps between the pieces and repeating
// makes it tile along the strip with no seam
void startWave(hitlib::LedGroup& group, const std::vector<uint32_t>& unit, uint16_t stepMs) {
    std::vector<hitlib::LedStrand::BitScrollSegment> segments;
    for (uint32_t colour : unit) segments.push_back({colour, 1});
    group.bitscroll(segments, hitlib::LedStrand::Speed::everyMs(stepMs), /*invert*/ false, /*bgColor*/ 0,
                    /*bounce*/ false, /*spacing*/ 0, /*repeating*/ true);
}

// how many pixels a pattern stepping every stepMs has moved after ms
uint32_t stepsAfter(uint32_t ms, uint16_t stepMs) {
    uint32_t ticksPerStep = std::max<uint32_t>(1, (stepMs + REFRESH_MS / 2) / REFRESH_MS);
    return ms / (REFRESH_MS * ticksPerStep);
}

uint32_t allianceColour() { return selector::isRed() ? theme::RED : theme::BLUE; }

void apply(int index) {
    playing = index;
    playingAlliance = selector::alliance();
    if (index < 0) {
        strips.off();
        return;
    }
    animations[index].start(strips);
}

void taskLoop() {
    while (true) {
        update();
        pros::delay(50);
    }
}

} // namespace

// ------------------------------------------------------------ animations ----
uint32_t toLed(uint32_t screenColour) { return mapChannels(screenColour, gammaUp); }
uint32_t toScreen(uint32_t ledColour) { return mapChannels(ledColour, gammaDown); }

Animation solid(const char* name, uint32_t colour) {
    uint32_t led = toLed(colour);
    return {name, [led](hitlib::LedGroup& g) { g.setColor(led); }, [led](int, uint32_t) { return led; }};
}

Animation wave(const char* name, uint32_t a, uint32_t b, uint8_t hold, uint8_t fade, uint16_t stepMs,
               bool throughDark) {
    std::vector<uint32_t> unit = waveUnit(a, b, hold, fade, throughDark);
    return {name, [unit, stepMs](hitlib::LedGroup& g) { startWave(g, unit, stepMs); },
            [unit, stepMs](int pixel, uint32_t ms) {
                return unit[(pixel + stepsAfter(ms, stepMs)) % unit.size()];
            }};
}

Animation allianceWave(const char* name, uint8_t hold, uint8_t fade, uint16_t stepMs) {
    // dim end of the wave: the alliance colour at a fifth of the brightness
    auto unitNow = [hold, fade] {
        uint32_t colour = allianceColour();
        return waveUnit(colour, mix(0, colour, 0.2), hold, fade);
    };
    return {name, [unitNow, stepMs](hitlib::LedGroup& g) { startWave(g, unitNow(), stepMs); },
            [unitNow, stepMs](int pixel, uint32_t ms) {
                std::vector<uint32_t> unit = unitNow();
                return unit[(pixel + stepsAfter(ms, stepMs)) % unit.size()];
            },
            true};
}

Animation rainbow(const char* name, uint16_t stepMs) {
    return {name, [stepMs](hitlib::LedGroup& g) { g.rainbow(hitlib::LedStrand::Speed::everyMs(stepMs)); },
            [stepMs](int pixel, uint32_t ms) {
                // HitLib's rainbow: one trip round the colour wheel along the strip
                int i = (int)((pixel + stepsAfter(ms, stepMs)) % LEDS_PER_SIDE);
                uint8_t pos = (uint8_t)(255 - i * 256 / LEDS_PER_SIDE);
                if (pos < 85) return (uint32_t)(255 - pos * 3) << 16 | (uint32_t)(pos * 3);
                if (pos < 170) {
                    pos -= 85;
                    return (uint32_t)(pos * 3) << 8 | (uint32_t)(255 - pos * 3);
                }
                pos -= 170;
                return (uint32_t)(pos * 3) << 16 | (uint32_t)(255 - pos * 3) << 8;
            }};
}

Animation off(const char* name) {
    return {name, [](hitlib::LedGroup& g) { g.off(); }, [](int, uint32_t) { return 0u; }};
}

// ------------------------------------------------------------------- use ----
void init(int idle, int match) {
    if (started.exchange(true)) return;
    idleChoice = clampIndex(idle);
    matchChoice = clampIndex(match);

    strips.add(&leftStrip);
    strips.add(&rightStrip);
    strips.init(REFRESH_MS);
    strips.setBrightness(BRIGHTNESS);
    strips.start();

    update();
    pros::Task([] { taskLoop(); }, "underglow");
}

void setIdle(int index) { idleChoice = clampIndex(index); }
void setMatch(int index) { matchChoice = clampIndex(index); }
int idleIndex() { return idleChoice; }
int matchIndex() { return matchChoice; }

void tryOut(int index, uint32_t ms) {
    tryUntil = pros::millis() + ms;
    tryChoice = clampIndex(index);
}

void stopTryOut() { tryChoice = -1; }

Showing showing() { return nowShowing; }

void update() {
    if (!started) return;
    bool inMatch = pros::competition::is_connected() && !pros::competition::is_disabled();

    // a try-out only runs outside a match, and ends when its time is up
    int trying = tryChoice;
    if (trying >= 0 && (inMatch || (int32_t)(pros::millis() - tryUntil) >= 0)) {
        tryChoice.compare_exchange_strong(trying, -1);
        trying = -1;
    }

    int want;
    if (trying >= 0) {
        want = trying;
        nowShowing = Showing::TRYING;
    } else if (inMatch) {
        want = matchChoice;
        nowShowing = Showing::MATCH;
    } else {
        want = idleChoice;
        nowShowing = Showing::IDLE;
    }

    // alliance animations restart when the alliance colour is changed
    bool allianceChanged = want >= 0 && animations[want].followsAlliance && selector::alliance() != playingAlliance;
    if (want != playing || allianceChanged) apply(want);
}

} // namespace underglow
