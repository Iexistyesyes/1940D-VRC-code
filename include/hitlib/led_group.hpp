#pragma once
#include "led_strand.hpp"
#include "led_profile.hpp"
#include <cstddef>
#include <vector>

/**
 * @file led_group.hpp
 * @brief Fan-out controller for multiple LedStrand instances.
 */

namespace hitlib {

// ============================================================================

/**
 * @brief Owns a set of LedStrand pointers and fans all animation commands out
 *        to each strand simultaneously.
 *
 * LedGroup starts a single background task that calls LedStrand::tick() on every
 * strand it owns at the configured refresh interval.  Multiple independent
 * groups are fully supported, each runs its own task.  What they do share is
 * the V5's ADI link: every frame a strand sends holds it for 12 ms, one strand
 * at a time, across all groups.  A strand whose frame has not changed sends
 * nothing, so only the strands that are animating pay for it.
 *
 * ### Setup
 * @code{.cpp}
 * hitlib::LedStrand strand1(6, 63);
 * hitlib::LedStrand strand2(7, 63);
 * hitlib::LedGroup  group;
 *
 * void initialize() {
 *     group.add(&strand1);
 *     group.add(&strand2);
 *     group.init();    // initializes hardware on all strands
 *     group.start();   // starts the refresh task
 * }
 * @endcode
 *
 * @see LedStrand
 */
class LedGroup {
public:

    // ========================================================================
    /// @name Setup
    /// @{

    /**
     * @brief Register a strand with this group.
     *
     * Normally called before init().  A strand added after init() is
     * initialized on the way in and set to the group's interval, and one
     * added after start() is ticked from the next cycle on.  Adding a strand
     * that is already in the group does nothing.
     *
     * @param strand  Pointer to a LedStrand.  The strand is not owned by the
     *                group, it must outlive the group.
     */
    void add(LedStrand* strand);

    /**
     * @brief Initialize hardware on all registered strands.
     *
     * Calls LedStrand::init() on each strand.  Safe to call from PROS
     * `initialize()` or the start of VEXcode's `main()`.
     *
     * The group ticks every strand at one interval, so it also becomes the
     * interval each strand converts its durations with (flash times,
     * Speed::everyMs() and the like), whatever the strand was constructed
     * with.
     *
     * @param refreshMs  Refresh interval passed to the group task in milliseconds
     *                   (default 20 ms = 50 Hz).  Pass 0 to use the first
     *                   strand's own configured interval for the whole group.
     */
    void init(uint32_t refreshMs = 20);

    /**
     * @brief Start the group refresh task.
     *
     * Call after init().  The task calls tick() on every strand each cycle.
     * Calling it again does nothing: there is only ever one task per group.
     *
     * A cycle can't be shorter than the ADI writes in it take, 12 ms for
     * each strand whose frame changed.  A cycle that overruns by a whole
     * interval or more restarts the schedule from that moment rather than
     * trying to catch up, so lost time is never paid back as a burst of fast
     * ticks.
     */
    void start();

    /// @}

    // ========================================================================
    /// @name Base Animations
    ///
    /// Fan-out wrappers, each call is forwarded to every strand in the group.
    /// @{

    /** @copydoc LedStrand::off */
    void off();

    /** @copydoc LedStrand::setColor */
    void setColor(uint32_t color);

    /** @copydoc LedStrand::pulse */
    void pulse(uint32_t color, uint8_t runLength, LedStrand::Speed speed,
               uint32_t bgColor = 0x000000, bool invert = false, bool bounce = false);

    /** @copydoc LedStrand::flash */
    void flash(uint32_t color, uint32_t onMs, uint32_t offMs, uint32_t bgColor = 0x000000);

    /** @copydoc LedStrand::flow */
    void flow(uint32_t color1, uint32_t color2, LedStrand::Speed speed, bool invert = false,
              bool seamless = true);

    /** @copydoc LedStrand::rainbow */
    void rainbow(LedStrand::Speed speed, bool invert = false);

    /** @copydoc LedStrand::twinkle */
    void twinkle(const std::vector<uint32_t>& colors, uint8_t densityPct = 30,
                 uint8_t fadeStep = 16, uint32_t bgColor = 0x000000, uint8_t bitWidth = 1);

    /** @copydoc LedStrand::bitscroll */
    void bitscroll(const std::vector<LedStrand::BitScrollSegment>& segments, LedStrand::Speed speed,
                   bool invert = false, uint32_t bgColor = 0x000000, bool bounce = false,
                   uint8_t spacing = 5, bool repeating = true);

    /** @copydoc LedStrand::levelFill */
    void levelFill(uint32_t color, uint32_t color2 = 0x000000, bool gradient = false,
                   uint32_t bgColor = 0x000000, bool invert = false);

    /** @copydoc LedStrand::setLevel */
    void setLevel(uint8_t level);

    /** @copydoc LedStrand::levelSource */
    void levelSource(LedStrand::LevelFn read, double emptyAt, double fullAt, bool wrap = false,
                     uint8_t smoothing = 0);

    /** @copydoc LedStrand::clearLevelSource */
    void clearLevelSource();

    /** @copydoc LedStrand::musicSync */
    void musicSync(const LedStrand::MusicTrack& track, uint32_t color, uint32_t color2 = 0x000000,
                   bool gradient = false, uint32_t bgColor = 0x000000, bool invert = false,
                   uint8_t sensitivity = 100, bool loop = false);

    /** @copydoc LedStrand::musicSeek */
    void musicSeek(uint32_t positionMs);

    /** @copydoc LedStrand::musicPause */
    void musicPause(bool paused = true);

    /** @copydoc LedStrand::setSensitivity */
    void setSensitivity(uint8_t pct);

    /** @copydoc LedStrand::spliceMask */
    void spliceMask(uint8_t sections, bool invert = false, bool alternating = false,
                    uint32_t altPeriodMs = 100, uint32_t bgColor = 0x000000,
                    bool useOverlay = false);

    /** @copydoc LedStrand::spliceMaskCustom */
    void spliceMaskCustom(const std::vector<LedStrand::SpliceRegion>& regions);

    /** @copydoc LedStrand::spreadMask */
    void spreadMask(LedStrand::Speed speed,
                    LedStrand::SpreadOrigin origin = LedStrand::SpreadOrigin::EDGE,
                    bool invert = false, bool bounce = false, uint32_t bgColor = 0x000000,
                    bool useOverlay = false);

    /** @copydoc LedStrand::twinkleMask */
    void twinkleMask(uint8_t densityPct = 30, uint8_t fadeStep = 16, uint8_t bitWidth = 1,
                     bool invert = false, uint32_t bgColor = 0x000000, bool useOverlay = false);

    /** @copydoc LedStrand::clearSpliceMask */
    void clearSpliceMask();

    /** @copydoc LedStrand::transition */
    void transition(const LedStrand::Transition& t);

    /** @copydoc LedStrand::setBrightness */
    void setBrightness(uint8_t pct);

    /// @}

    // ========================================================================
    /// @name Overlay Animations
    /// @{

    /** @copydoc LedStrand::overlaySetColor */
    void overlaySetColor(uint32_t color);

    /** @copydoc LedStrand::overlayPulse */
    void overlayPulse(uint32_t color, uint8_t runLength, LedStrand::Speed speed,
                      uint32_t bgColor = 0x000000, bool invert = false);

    /** @copydoc LedStrand::overlayFlash */
    void overlayFlash(uint32_t color, uint32_t onMs, uint32_t offMs,
                      uint32_t bgColor = 0x000000);

    /** @copydoc LedStrand::overlayFlow */
    void overlayFlow(uint32_t color1, uint32_t color2, LedStrand::Speed speed, bool seamless = true,
                     bool invert = false);

    /** @copydoc LedStrand::overlayRainbow */
    void overlayRainbow(LedStrand::Speed speed, bool invert = false);

    /// @}

    // ========================================================================
    /// @name Profile System
    /// @{

    /** @copydoc LedStrand::attachProfile */
    void attachProfile(const Profile* profile);

    /** @copydoc LedStrand::detachProfile */
    void detachProfile();

    /** @copydoc LedStrand::activateMode */
    void activateMode(uint8_t modeIdx);

    /** @copydoc LedStrand::activateModeTimed */
    void activateModeTimed(uint8_t modeIdx, uint32_t durationMs);

    /** @copydoc LedStrand::deactivateMode */
    void deactivateMode(uint8_t modeIdx);

    /// @}

    // ========================================================================
    /// @name Direct Access
    /// @{

    /**
     * @brief Access an individual strand by index.
     *
     * Useful when you need to apply different animations to specific strands
     * while still using the group for shared commands.
     *
     * @param i  Zero-based index in the order strands were added.
     */
    LedStrand* operator[](size_t i);

    /**
     * @brief Return the number of strands in this group.
     */
    size_t size() const;

    /// @}

private:
    std::vector<LedStrand*> strands;
    uint32_t refreshMs   = 20;
    bool     initialized = false;
    bool     started     = false;
    // Guards the fields above, so add() can run while the task is going round
    // the strands. The task holds it only to copy the list, never across a
    // tick. See the lock order in led_group.cpp.
    mutable platform::Mutex mutex;

    void groupTask();
    static void taskEntry(void* group);

    // Calls fn(strand) for every strand, holding the lock across the loop.
    template <typename Fn>
    void forEach(Fn fn) {
        mutex.take();
        for (LedStrand* s : strands) fn(*s);
        mutex.give();
    }
};

} // namespace hitlib