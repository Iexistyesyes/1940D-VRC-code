#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace theme {
inline constexpr uint32_t BLACK = 0x000000;        // screen background (logo background)
inline constexpr uint32_t SURFACE = 0x141414;      // unselected buttons / panels
inline constexpr uint32_t BORDER = 0x2A2A2A;       // outlines on dark parts
inline constexpr uint32_t YELLOW = 0xF7D246;       // lightning bolt yellow
inline constexpr uint32_t YELLOW_PRESSED = 0xC9A834; // yellow while a button is held
inline constexpr uint32_t WHITE = 0xFFFFFF;        // gear / wordmark white
inline constexpr uint32_t GREY = 0x9A9A9A;         // descriptions and captions
inline constexpr int RADIUS = 6;
} // namespace theme

namespace selector {

struct Auton {
    std::string name;        // shown on the button, keep it short (~10 chars)
    std::string description; // shown in the side panel
    std::function<void()> run;
};

/**
 * Builds the selector on the brain screen. Call once in initialize().
 * Don't also call pros::lcd::initialize(), they both draw on the same screen.
 * Up to 6 autons fit on screen.
 */
void init(const std::vector<Auton>& autons, int defaultAuton = 0);

/** index of the selected auton */
int selectedIndex();

/** name of the selected auton */
std::string selectedName();

/** runs the selected auton, call this in autonomous() */
void runSelected();

/** select an auton from code (safe to call from any task) */
void select(int index);

/** small status text in the top right corner, e.g. odom pose (safe from any task) */
void setStatus(const std::string& text);

} // namespace selector
