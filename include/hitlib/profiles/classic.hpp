#pragma once

/**
 * @file profiles/classic.hpp
 * @brief Ready-made Classic, Modern and Showy LED profiles.
 *
 * @ingroup profiles
 *
 * Include this header to access all three built-in profiles.
 *
 * ### Mode index table (all profiles)
 *
 * Classic and Modern have seven modes each and share their numbering.  Showy
 * has five, so its indices are its own: no Showoff or Matchload, and Endgame
 * is 4 rather than 6.  activateMode() takes a raw index into the profile's
 * mode array, so activateMode(6) against Showy is out of range.
 *
 * | Mode      | Classic                         | Modern                            | Showy                       |
 * |-----------|---------------------------------|-----------------------------------|-----------------------------|
 * | Showoff   | `0` rainbow scroll              | `0` rainbow scroll                | —                           |
 * | Idle      | `1` magenta flow                | `1` pink pulse                    | `0` purple flow             |
 * | Red       | `2` red pulse                   | `2` red pulse + orange bg         | `1` white pulse / red bg    |
 * | Blue      | `3` blue pulse                  | `3` blue pulse + cyan bg          | `2` white pulse / blue bg   |
 * | Scoring   | `4` green pulse                 | `4` green flash                   | `3` teal pulse              |
 * | Matchload | `5` yellow pulse                | `5` yellow pulse                  | —                           |
 * | Endgame   | `6` warn → white → cycle (12 s) | `6` solid green → pulse (19 s)    | `4` yellow → rainbow (10 s) |
 *
 * ### Usage
 * @code{.cpp}
 * #include "hitlib/hitapi.hpp"
 * #include "hitlib/profiles/classic.hpp"
 *
 * hitlib::LedGroup group;
 *
 * void initialize() {
 *     group.init();
 *     group.start();
 *     group.attachProfile(&hitlib::profiles::classic);
 *     group.activateMode(1);   // Idle
 * }
 *
 * void autonomous() {
 *     group.activateModeTimed(4, 1500);   // Scoring for 1.5 s
 * }
 *
 * void opcontrol() {
 *     group.activateMode(6);   // Endgame
 * }
 * @endcode
 */

#include "hitlib/led_strand.hpp"
#include "hitlib/led_profile.hpp"
#include "hitlib/led_sequencer.hpp"

namespace hitlib::profiles {

// ============================================================================
/// @addtogroup profiles
/// @{

// ============================================================================
// Classic
// ============================================================================

/**
 * @name Classic - setup functions
 * Internal callbacks used by the Classic profile modes.
 * @{
 */
inline void classicIdle        (LedStrand& s) { s.flow(0xFF00DD, 0x000000, 1); }
inline void classicRed         (LedStrand& s) { s.pulse(0xFF0000, 5, 1); }
inline void classicBlue        (LedStrand& s) { s.pulse(0x0000FF, 5, 1); }
inline void classicScore       (LedStrand& s) { s.pulse(0x00FF00, 5, 1); }
inline void classicMatchloader (LedStrand& s) { s.pulse(0xEED202, 5, 1); }
inline void showoff            (LedStrand& s) { s.rainbow(1); }

inline void classicEgWarn  (LedStrand& s) { s.flash(0xFFFF00, 120, 120); }
inline void classicEgWhite (LedStrand& s) { s.setColor(0xFFFFFF); }
inline void classicEgCycle (LedStrand& s) { s.pulse(0xFF0000, 5, 1); }
/// @}

/// @cond INTERNAL
inline const Sequencer::Phase classicEgPhases[] = {
    {1500, classicEgWarn},
    {8500, classicEgWhite},
    {2000, classicEgCycle},
};
inline Sequencer classicEndgameSeq(classicEgPhases, 3);
inline void classicEgActivate(LedStrand& s) { classicEndgameSeq.start(s); }
inline void classicEgTick    (LedStrand& s) { classicEndgameSeq.update(s); }
/// @endcond

inline const ProfileMode classicModes[] = {
    {"Showoff",      15,  showoff,            nullptr},
    {"Idle",         10,  classicIdle,        nullptr},
    {"Red",          20,  classicRed,         nullptr},
    {"Blue",         20,  classicBlue,        nullptr},
    {"Scoring",      70,  classicScore,       nullptr},
    {"Matchloading", 90,  classicMatchloader, nullptr},
    {"Endgame",      100, classicEgActivate,  classicEgTick},
};

/**
 * @brief Classic profile instance.
 *
 * Idle: flowing magenta. Match: color pulse. Endgame: warn -> white -> cycle.
 *
 * @see classicModes
 */
inline const Profile classic = {"Classic", classicModes, 7};

// ============================================================================
// Modern
// ============================================================================

/// @cond INTERNAL
inline void modernIdle        (LedStrand& s) { s.pulse(0xFF66CC, 8, 1); }
inline void modernRed         (LedStrand& s) { s.pulse(0xFF0000, 14, 1, 0xFF6200); }
inline void modernBlue        (LedStrand& s) { s.pulse(0x0000FF, 14, 1, 0x009DFF); }
inline void modernScore       (LedStrand& s) { s.flash(0x11FF00, 100, 100); }
inline void modernMatchloader (LedStrand& s) { s.pulse(0xEED202, 5, 1); }
inline void modernEgGreen     (LedStrand& s) { s.setColor(0x11FF00); }
inline void modernEgPulse     (LedStrand& s) { s.pulse(0x11FF00, 8, 2); }

inline const Sequencer::Phase modernEgPhases[] = {
    {15000, modernEgGreen},
    {4000,  modernEgPulse},
};
inline Sequencer modernEndgameSeq(modernEgPhases, 2);
inline void modernEgActivate(LedStrand& s) { modernEndgameSeq.start(s); }
inline void modernEgTick    (LedStrand& s) { modernEndgameSeq.update(s); }
/// @endcond

inline const ProfileMode modernModes[] = {
    {"Showoff",      15,  showoff,           nullptr},
    {"Idle",         10,  modernIdle,        nullptr},
    {"Red",          20,  modernRed,         nullptr},
    {"Blue",         20,  modernBlue,        nullptr},
    {"Scoring",      70,  modernScore,       nullptr},
    {"Matchloading", 90,  modernMatchloader, nullptr},
    {"Endgame",      100, modernEgActivate,  modernEgTick},
};

/**
 * @brief Modern profile instance.
 *
 * Idle: pink pulse. Match: alliance-colored pulse with accent background.
 * Endgame: solid green -> pulsing green.
 */
inline const Profile modern = {"Modern", modernModes, 7};

// ============================================================================
// Showy
// ============================================================================

/// @cond INTERNAL
inline void showyIdle  (LedStrand& s) { s.flow(0xAC00FF, 0x000000, 1); }
inline void showyRed   (LedStrand& s) { s.pulse(0xFFFFFF, 4, 1, 0xFF0000); }
inline void showyBlue  (LedStrand& s) { s.pulse(0xFFFFFF, 4, 1, 0x0000FF); }
inline void showyScore (LedStrand& s) { s.pulse(0x00FF88, 6, 2); }
inline void showyEgPulse   (LedStrand& s) { s.pulse(0xFFFF00, 6, 1); }
inline void showyEgRainbow (LedStrand& s) { s.rainbow(2); }

inline const Sequencer::Phase showyEgPhases[] = {
    {2000, showyEgPulse},
    {8000, showyEgRainbow},
};
inline Sequencer showyEndgameSeq(showyEgPhases, 2);
inline void showyEgActivate(LedStrand& s) { showyEndgameSeq.start(s); }
inline void showyEgTick    (LedStrand& s) { showyEndgameSeq.update(s); }
/// @endcond

inline const ProfileMode showyModes[] = {
    {"Idle",     10,  showyIdle,       nullptr},
    {"Red",      20,  showyRed,        nullptr},
    {"Blue",     20,  showyBlue,       nullptr},
    {"Scoring",  90,  showyScore,      nullptr},
    {"Endgame",  100, showyEgActivate, showyEgTick},
};

/**
 * @brief Showy profile instance.
 *
 * Idle: purple flow. Match: white pulse on alliance background.
 * Endgame: yellow pulse -> rainbow.
 */
inline const Profile showy = {"Showy", showyModes, 5};

/// @}

} // namespace hitlib::profiles