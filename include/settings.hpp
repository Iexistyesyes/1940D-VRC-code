#pragma once

// ============================================================================
//  SETTINGS  -  remembers the brain screen picks between runs
//
//  Saved on the brain's microSD card (the V5 can't save anywhere else), as a
//  plain text file you can open on a computer:
//
//      auton=RIGHT QUAD
//      alliance=RED
//      display=OVERRIDE LOGO
//      leds_idle=CHRISTMAS
//      leds_match=LOGO YELLOW
//
//  Picks are saved by name, so adding or reordering autons / pictures /
//  animations doesn't mix them up. A name that's gone is just skipped.
//  Delete the file to go back to the defaults in main.cpp.
// ============================================================================

#ifndef SETTINGS_FILE
#define SETTINGS_FILE "/usd/1940D_settings.txt"
#endif

namespace settings {

/**
 * Loads the saved picks and keeps saving them whenever they change.
 * Call in initialize() after selector::init() and underglow::init().
 * @return false if there's no SD card (the brain shows a message, and picks
 *         are not remembered)
 */
bool init();

} // namespace settings
