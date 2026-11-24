#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "liblvgl/lvgl.h"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

// ============================================================================
//  THEME  -  Override 1940D
//  Colours are taken straight from the team logo.
//  Logo + font live in src/assets/ (generated from the logo image and the
//  "Unbounded Black" font, the closest free match to the logo lettering).
// ============================================================================
namespace theme {
inline constexpr uint32_t BLACK = 0x000000;        // screen background (logo background)
inline constexpr uint32_t SURFACE = 0x141414;      // unselected buttons / panels
inline constexpr uint32_t BORDER = 0x2A2A2A;       // outlines on dark parts
inline constexpr uint32_t YELLOW = 0xF7D246;       // lightning bolt yellow
inline constexpr uint32_t YELLOW_PRESSED = 0xC9A834; // yellow while a button is held
inline constexpr uint32_t WHITE = 0xFFFFFF;        // gear / wordmark white
inline constexpr uint32_t GREY = 0x9A9A9A;         // descriptions and captions
inline constexpr uint32_t RED = 0xE53935;          // red alliance, and "too hot" on the stats tab
inline constexpr uint32_t BLUE = 0x1E88E5;         // blue alliance
inline constexpr int RADIUS = 6;
} // namespace theme

namespace selector {

// ------------------------------------------------------------- AUTON tab ----
struct Auton {
    std::string name;        // shown on the button, keep it short (~10 chars)
    std::string description; // shown under the buttons when selected
    std::function<void()> run;
};

// ------------------------------------------------------------- STATS tab ----
struct Stat {
    std::string name;             // tile title, keep it short (~8 chars)
    std::function<double()> read; // returns the number to show
    std::string format = "%.0f";  // how to print it, e.g. "%.1f" or "%.0f°C"
    // The number turns yellow at `warn` and red at `danger`.
    // If danger is LOWER than warn it works the other way round (low is bad, like a battery).
    double warn = INFINITY;
    double danger = INFINITY;
};

/** hottest motor temperature of a motor / motor group. Yellow at 45°C, red at 55°C (motor starts cutting power) */
Stat motorTemp(const std::string& name, pros::Motor& motor);
Stat motorTemp(const std::string& name, pros::MotorGroup& motors);

/** brain battery percent. Yellow under 50%, red under 25% */
Stat battery();

// ---------------------------------------------------------- ALLIANCE tab ----
enum class Alliance { RED, BLUE };

// ----------------------------------------------------------- DISPLAY tab ----
// One picture / animation that can be shown on the brain during a match.
// The list is in src/matchScreens.cpp, new ones are made with tools/make_match_image.py
struct MatchScreen {
    const char* name;                   // shown on the DISPLAY tab
    const lv_image_dsc_t* image;        // the picture from src/assets/
    bool animated = false;              // true if it's a GIF
    uint32_t background = theme::BLACK; // colour around the picture if it doesn't fill the screen
};
extern const std::vector<MatchScreen> matchScreens;

// -------------------------------------------------------------------------
/**
 * Builds the brain screen (tabs: AUTON, STATS, ALLIANCE, DISPLAY). Call once in initialize().
 * Don't also call pros::lcd::initialize(), they both draw on the same screen.
 *
 * @param autons       autons to list. 12 fit on a page, any more go on extra pages
 * @param stats        tiles for the STATS tab (up to 12)
 * @param defaultAuton index of the auton that starts selected
 */
void init(const std::vector<Auton>& autons, const std::vector<Stat>& stats = {}, int defaultAuton = 0);

/** index of the selected auton */
int selectedIndex();

/** name of the selected auton */
std::string selectedName();

/** runs the selected auton, call this in autonomous() */
void runSelected();

/** alliance colour picked on the ALLIANCE tab */
Alliance alliance();
inline bool isRed() { return alliance() == Alliance::RED; }
inline bool isBlue() { return alliance() == Alliance::BLUE; }

// Everything below can be called from any task.

/** select an auton from code */
void select(int index);

/** set the alliance colour from code */
void setAlliance(Alliance colour);

/**
 * Show the picture chosen on the DISPLAY tab full screen / go back to the menu.
 * You don't normally need these: the picture comes up by itself when the field
 * (or a competition switch) enables the robot, and the menu comes back when it's disabled.
 */
void showMatchScreen();
void showMenu();

/** select the auton with this name. false if there isn't one */
bool selectByName(const std::string& name);

/** picture picked on the DISPLAY tab: index into matchScreens, matchScreens.size() means OFF */
int matchScreenIndex();
void setMatchScreen(int index);

/** shows a message along the bottom of the brain screen for a few seconds */
void notify(const std::string& text, uint32_t ms = 4000);

} // namespace selector
