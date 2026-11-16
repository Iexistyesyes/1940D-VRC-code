#pragma once
#include <cstdint>

/**
 * @file led_profile.hpp
 * @brief Profile and mode descriptors for the hitlib event-driven animation system.
 */

namespace hitlib {

class LedStrand; ///< Forward declaration.

/**
 * @brief Callback signature used for mode activation and per-tick updates.
 *
 * @param strand The strand this mode is running on.
 */
using ModeFn = void (*)(LedStrand& strand);

// ============================================================================

/**
 * @brief Describes a single named animation mode within a Profile.
 *
 * When multiple modes are active simultaneously on a strand the one with the
 * highest @p priority wins.  Ties are broken by insertion order (later wins).
 *
 * ### Lifecycle
 * 1. Mode is pushed onto the strand's mode stack via
 *    LedStrand::activateMode() or LedStrand::activateModeTimed().
 * 2. On the next tick, LedStrand::computeEffectiveMode() selects the winner.
 * 3. @p onActivate is called **once** when the winning mode changes.
 * 4. @p onTick is called **every refresh tick** while the mode remains active.
 *
 * @see Profile
 * @see LedStrand::attachProfile
 */
struct ProfileMode {
    const char* name;           ///< Label shown in debug output.
    uint8_t     priority;       ///< Higher value beats lower. Range: 0–255.
    ModeFn      onActivate;     ///< Called once when this mode becomes the winner.
    ModeFn      onTick;         ///< Called every tick while active. May be @c nullptr.
};

// ============================================================================

/**
 * @brief An immutable, statically-allocated collection of ProfileMode entries.
 *
 * A Profile holds no per-strand state, state lives inside LedStrand.
 * Multiple strands (or groups) can share the same Profile instance.
 *
 * ### Example: declaring a custom profile
 * @code{.cpp}
 * static void myIdle (hitlib::LedStrand& s) { s.rainbow(1); }
 * static void myAlert(hitlib::LedStrand& s) { s.flash(0xFF0000, 100, 100); }
 *
 * static const hitlib::ProfileMode myModes[] = {
 *     {"Idle",  10, myIdle,  nullptr},
 *     {"Alert", 80, myAlert, nullptr},
 * };
 * static const hitlib::Profile myProfile = {"MyRobot", myModes, 2};
 * @endcode
 *
 * @see ProfileMode
 * @see LedStrand::attachProfile
 */
struct Profile {
    const char*        name;      ///< Profile name (for display/debug).
    const ProfileMode* modes;     ///< Pointer to the mode array.
    uint8_t            modeCount; ///< Number of entries in @p modes.
};

} // namespace hitlib