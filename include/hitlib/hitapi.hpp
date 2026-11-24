#pragma once

/**
 * @file hitapi.hpp
 * @brief Top-level convenience header, includes the full hitlib public API.
 *
 * @code{.cpp}
 * #include "hitlib/hitapi.hpp"
 * @endcode
 *
 * @defgroup core      Core Classes
 * @brief LedStrand, LedGroup and the animation engine.
 *
 * @defgroup profiles  Built-in Profiles
 * @brief Ready-made Classic, Modern and Showy profiles.
 *
 * @defgroup types     Supporting Types
 * @brief Profile, ProfileMode, Sequencer and related data types.
 */

#include "hitlib/led_strand.hpp"
#include "hitlib/led_group.hpp"
#include "hitlib/led_profile.hpp"
#include "hitlib/led_sequencer.hpp"