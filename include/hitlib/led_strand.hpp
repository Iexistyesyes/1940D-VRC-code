#pragma once
#include "led_profile.hpp"
#include "platform.hpp"
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <utility>
#include <vector>

/**
 * @file led_strand.hpp
 * @brief Single addressable LED strip driver with layered animation engine.
 */

namespace hitlib {

// ============================================================================

/**
 * @brief Driver for a single WS2812B-compatible LED strip on a VEX ADI port.
 *
 * LedStrand owns one addressable-LED ADI port and drives it from a
 * task-managed refresh loop via LedGroup.  All public animation methods are
 * **thread-safe**, they take effect on the next refresh tick.
 *
 * ### Layer model
 * ```
 * base buffer      <- flow / rainbow / pulse / bitscroll / twinkle, or a
 *                     level meter driven by hand, by a live value, or by a
 *                     music envelope
 * overlay buffer   <- second independent animation
 * mask             <- final per-pixel override (bgColor or overlay), by
 *                     equal alternating bins, by arbitrary regions, by a
 *                     spreading wipe or by twinkling bits
 * transition       <- the frame that was showing, wiped away as whatever
 *                     replaced it comes in
 * ```
 *
 * ### Typical setup
 * @code{.cpp}
 * hitlib::LedStrand strand(6, 63);   // ADI port 6, 63 LEDs
 * hitlib::LedGroup  group;
 *
 * void initialize() {
 *     group.add(&strand);
 *     group.init();
 *     group.start();
 * }
 *
 * void opcontrol() {
 *     strand.rainbow(1);
 * }
 * @endcode
 *
 * @see LedGroup
 */
class LedStrand {
public:

    // ========================================================================
    /// @name Types
    /// @{

    /**
     * @brief A port on an ADI expander: the Smart Port the expander is plugged
     *        into, and the ADI port on it.
     *
     * Written as a braced pair, the same way PROS writes its own expander
     * ports:
     * @code{.cpp}
     * hitlib::LedStrand strand({2, 1}, 63);   // expander on Smart Port 2, ADI port A
     * @endcode
     */
    struct ExpanderPort {
        uint8_t smartPort; ///< Smart Port the expander is connected to (1–21).
        uint8_t adiPort;   ///< ADI port on the expander (1–8).
    };

    /**
     * @brief One physical strip in a chain: where it is plugged in and how
     *        many LEDs it has.
     *
     * A chain is one LedStrand spread across several strips, so an animation
     * runs off the end of one strip and onto the next as if they were a
     * single long one.  See the chain constructor.
     *
     * A strip mounted the other way round - its data-in end, LED 0, at the
     * far end of the chain's run - takes @c true as a third value.  Its part
     * of every frame is then sent back to front, so the chain still reads in
     * one direction from end to end instead of doubling back on that strip.
     *
     * @code{.cpp}
     * hitlib::LedStrand::Segment{1, 20};          // ADI port A, 20 LEDs
     * hitlib::LedStrand::Segment{{2, 1}, 20};     // expander on Smart Port 2, port A
     * hitlib::LedStrand::Segment{3, 20, true};    // ADI port C, wired backwards
     * @endcode
     */
    struct Segment {
        uint8_t smartPort; ///< Smart Port of the expander, or 0 for the brain's own ADI ports.
        uint8_t adiPort;   ///< ADI port (1–8).
        uint8_t length;    ///< LEDs on this strip.  Clamped to MAX_LEDS (64).
        bool    reversed;  ///< Wired backwards: LED 0 is the last pixel of this strip's part of the chain.

        /// A strip on one of the brain's own ADI ports.
        constexpr Segment(uint8_t adi, uint8_t len, bool rev = false)
            : smartPort(0), adiPort(adi), length(len), reversed(rev) {}
        /// A strip on an ADI expander's port.
        constexpr Segment(ExpanderPort port, uint8_t len, bool rev = false)
            : smartPort(port.smartPort), adiPort(port.adiPort), length(len), reversed(rev) {}
    };

    /**
     * @brief One colored segment in a bitscroll pattern.
     */
    struct BitScrollSegment {
        uint32_t color; ///< Segment color (0xRRGGBB).
        uint8_t  width; ///< Width in pixels.
    };

    /**
     * @brief How fast a moving animation travels: so many pixels, so often.
     *
     * A bare number is pixels per refresh tick, which is what @c speed has
     * always meant, so `strand.rainbow(2)` is unchanged.  A pixel a tick is
     * the fastest-but-one a bare number can say and the slowest it can say,
     * so for anything slower give the time between steps instead:
     *
     * @code{.cpp}
     * using Speed = hitlib::LedStrand::Speed;
     * strand.pulse(0xFF0000, 5, Speed::everyMs(80));               // a pixel every 80 ms
     * strand.flow(0xFF0000, 0x0000FF, Speed::everyMs(40, 2));      // two pixels every 40 ms
     * @endcode
     *
     * Like flash()'s durations, @c stepMs is rounded to whole refresh ticks,
     * so the pace is even: a 100 ms step on a 20 ms strand moves on every
     * fifth tick, never on a mix of fourth and sixth ones.
     */
    struct Speed {
        uint8_t  pixels; ///< Pixels moved per step.
        uint16_t stepMs; ///< Milliseconds between steps. 0 steps on every refresh tick.

        /// @p pixelsPerStep pixels every @p ms milliseconds; with no @p ms,
        /// that many pixels on every refresh tick.
        constexpr Speed(int pixelsPerStep = 1, uint16_t ms = 0)
            : pixels((uint8_t)pixelsPerStep), stepMs(ms) {}

        /// @p pixelsPerStep pixels (default one) every @p ms milliseconds.
        static constexpr Speed everyMs(uint16_t ms, uint8_t pixelsPerStep = 1) {
            return Speed(pixelsPerStep, ms);
        }
    };

    /**
     * @brief Signature for the reader a fill meter follows, see levelSource().
     *
     * Called once per refresh tick, from the LedGroup task rather than from
     * your loop, and expected to just read something and return it, in
     * whatever units that something already speaks.
     *
     * A capture-less lambda converts to one of these, which is how it usually
     * gets written:
     *
     * @code{.cpp}
     * pros::Motor intake(11);
     * strand.levelSource([] { return intake.get_temperature(); }, 20.0, 70.0);
     * @endcode
     *
     * A lambda has to actually return a @c double, so a reading that comes
     * back as an integer needs saying so: `[] { return (double)rot.get_angle(); }`.
     *
     * @note Because the reader runs on the LED task, it must not block, and
     *       must not take a lock your own loop already holds while waiting on
     *       the LEDs.  Reading a device or a plain variable is fine.
     */
    using LevelFn = double (*)();

    /**
     * @brief What a custom splice mask region shows.
     *
     * OFF through BITSCROLL mirror the overlay*() animation vocabulary (see
     * @ref overlaySetColor "Overlay Animations"), since each region gets a
     * buffer built and animated the same way.
     *
     * @c GAUGE animates from a *reading* rather than from a clock, making the
     * region an independent meter.  See @ref GaugeStop and SpliceRegion's
     * gauge fields.
     */
    enum class SpliceRegionAnimKind : uint8_t {
        OFF, SOLID, PULSE, FLASH, FLOW, RAINBOW, TWINKLE, BITSCROLL, GAUGE
    };

    /**
     * @brief One color on a GAUGE region's scale.
     *
     * Stops are given in the reading's own units, not in pixels or 0-255:
     * `{55.0, 0xFF7000}` is "orange at 55 °C".  They are sorted and mapped
     * onto the region's @c emptyAt - @c fullAt range once, when the mask is
     * applied.  Two stops at the same reading make a hard edge, and keep the
     * order they were given in: the first is the color below it.
     *
     * Below the first stop and above the last, the gauge shows that stop's
     * color, so a scale never has to cover a range it doesn't care about.
     *
     * @see GaugeBlend for what happens between two stops.
     */
    struct GaugeStop {
        double   at;    ///< Reading this color belongs to, in the source's units.
        uint32_t color; ///< Color shown at that reading (0xRRGGBB).
    };

    /**
     * @brief How a GAUGE region turns its level into pixels.
     */
    enum class GaugeStyle : uint8_t {
        HEAT, ///< The whole region shows one color, picked off the scale.
        BAR,  ///< The region fills proportionally, like a miniature levelFill().
    };

    /**
     * @brief How a GAUGE region moves between two stops.
     */
    enum class GaugeBlend : uint8_t {
        LERP, ///< Blend smoothly, so a rising reading slides between colors.
        STEP, ///< Hold each stop's color until the next one is reached (a
              ///< reading exactly on a stop has reached it).
    };

    /**
     * @brief One independently placed override region for a custom splice mask.
     *
     * Each region gets its own animation buffer, generated over just that
     * region's width, and animates independently of every other region and
     * of the base/overlay buffers. Unlike spliceMask()'s single shared
     * overlay, every region here can show something different at once.
     */
    struct SpliceRegion {
        uint8_t  start;                                        ///< First pixel index covered by this region.
        uint8_t  width;                                         ///< Number of pixels covered.
        SpliceRegionAnimKind kind = SpliceRegionAnimKind::OFF;  ///< What this region shows.
        uint32_t color     = 0xFFFFFF; ///< Foreground color (SOLID/PULSE/FLASH/FLOW/BITSCROLL).
        uint32_t color2    = 0x0000FF; ///< FLOW end color.
        uint32_t bgColor   = 0x000000; ///< PULSE/FLASH/TWINKLE/BITSCROLL background, and a GAUGE BAR's unlit part.
        uint8_t  runLength = 5;        ///< PULSE run length.
        Speed    speed     = 1;        ///< PULSE/FLOW/RAINBOW/BITSCROLL motion, see Speed.
        uint32_t onMs      = 250;      ///< FLASH lit duration (ms).
        uint32_t offMs     = 250;      ///< FLASH blank duration (ms).
        bool     seamless  = true;     ///< FLOW only. See @c flow().
        /// Reverse direction: a PULSE, FLOW, RAINBOW or BITSCROLL moves the
        /// other way, a GAUGE BAR fills from the far end of the region.
        bool     invert    = false;

        // ---- TWINKLE only ----

        /// Colors sparks are drawn from.  An empty palette leaves the region dark.
        std::vector<uint32_t> palette;
        uint8_t densityPct = 30;  ///< Share of the region lit at once (0-100).
        uint8_t fadeStep   = 16;  ///< Brightness step per tick, so bigger is faster.

        // ---- BITSCROLL only ----

        uint8_t segmentWidth = 3;    ///< Pixels per lit run.
        uint8_t spacing      = 5;    ///< Background pixels between runs.
        bool    repeating    = true; ///< Tile the pattern across the region, or scroll one copy.

        // ---- GAUGE only, ignored by every other kind ----

        /// Value this gauge follows, polled once per tick.  @c nullptr leaves
        /// the region hand-driven through setRegionLevel().
        LevelFn read = nullptr;
        double  emptyAt = 0.0;    ///< Reading at the bottom of the scale.
        double  fullAt  = 100.0;  ///< Reading at the top of it.
        bool    wrap    = false;  ///< Cycle past @c fullAt instead of pinning at it.
        uint8_t smoothing = 0;    ///< 0-99, as in levelSource().
        GaugeStyle style = GaugeStyle::HEAT; ///< Whole-region color, or a bar.
        GaugeBlend blend = GaugeBlend::LERP; ///< Blend between stops, or hold each.
        /// The scale, in the reading's own units.  Empty falls back to
        /// @c color at @c emptyAt blending to @c color2 at @c fullAt, so a
        /// two-color gauge needs no stops at all.
        std::vector<GaugeStop> stops;

        // ---- Added in 1.6, last so a region braced in field order before
        // ---- it still means what it did ----

        uint8_t bitWidth = 1;  ///< TWINKLE: pixels per spark.
    };

    /**
     * @brief A song's intensity envelope, baked ahead of time into one
     * 8-bit sample per frame.
     *
     * The V5 has no audio input and no MIDI decoder, so a strand can't listen
     * to the music it's syncing to.  Instead Pattern Studio reads a MIDI file
     * on the desktop, renders it down to a level-per-frame envelope, and
     * exports that envelope as a `constexpr` table.  On the robot musicSync()
     * just plays the table back against the wall clock, which costs one array
     * lookup and a lerp per tick regardless of how long or dense the song is.
     *
     * A three-minute song at the default 25 ms frame is about 7 KB of flash.
     *
     * @code{.cpp}
     * // Generated by Pattern Studio, in the exported header:
     * inline const uint8_t songSamples[] = {0, 3, 17, 42, ...};
     * inline const hitlib::LedStrand::MusicTrack song = {songSamples, 8840, 25};
     * @endcode
     *
     * @note Like Profile, a MusicTrack must outlive the strand it is handed
     *       to, the strand stores a pointer, not a copy.
     */
    struct MusicTrack {
        const uint8_t* samples;    ///< Intensity 0-255, one entry per frame.
        uint16_t       frameCount; ///< Number of samples in @p samples.
        uint16_t       frameMs;    ///< Milliseconds between consecutive samples.
    };

    /**
     * @brief Where a spread starts: see spreadMask() and Transition.
     *
     * Each has a reverse, picked with the @c invert that goes with it: an
     * EDGE spread inverted runs from the far end back to pixel 0, and a
     * CENTER spread inverted starts at both ends and closes in on the middle.
     */
    enum class SpreadOrigin : uint8_t {
        EDGE,   ///< From pixel 0 to the far end.
        CENTER, ///< From the middle out to both ends at once.
    };

    /**
     * @brief How a transition() wipes from the old frame to the new animation.
     */
    enum class TransitionKind : uint8_t {
        CUT,     ///< No wipe: the new animation shows on the very next frame.
        SPREAD,  ///< The new animation spreads in from an end or from the middle.
        TWINKLE, ///< The new animation appears a bit at a time, at random, each
                 ///< bit staying lit once it has come in.
    };

    /**
     * @brief A wipe from whatever is showing into whatever is set up next.
     *
     * Handed to transition(), or carried by a Sequencer::Phase to say how that
     * phase takes over from the one before it.  The outgoing frame holds still
     * while the incoming animation, already running, is wiped in over it.
     *
     * @code{.cpp}
     * using LS = hitlib::LedStrand;
     * strand.transition({LS::TransitionKind::SPREAD, 600, LS::SpreadOrigin::CENTER});
     * strand.rainbow(2);   // spreads out from the middle over 600 ms
     * @endcode
     *
     * @note No default member initializers, deliberately, as for Layer: that
     *       keeps this a C++11 aggregate, so `{}` - or leaving a Phase's
     *       transition out altogether - is a plain cut on every platform.
     */
    struct Transition {
        TransitionKind kind;       ///< CUT, the zero value, does nothing.
        uint16_t       durationMs; ///< How long the wipe takes.  0 is a cut.
        SpreadOrigin   origin;     ///< SPREAD: from pixel 0, or from the middle.
        bool           invert;     ///< SPREAD: from the far end, or from both ends inward.
        uint8_t        bitWidth;   ///< TWINKLE: pixels per bit.  0 is one.
        /// TWINKLE: brightness each bit gains per tick as it comes in, so
        /// bigger is quicker.  0 lights a bit all at once.
        uint8_t        fadeStep;
    };

    // ========================================================================
    // Custom animations (Compositions)
    // ========================================================================

    /// @name Custom Animation Types
    ///
    /// Ceilings first: Layer::bindings needs MAX_BINDINGS as an array bound,
    /// and a class is not complete until its closing brace, so these cannot
    /// live beside MAX_LEDS at the bottom.
    /// @{

    static constexpr uint8_t MAX_LAYERS       = 8;  ///< Layers in one Composition.
    static constexpr uint8_t MAX_DRIVERS      = 8;  ///< Drivers in one Composition.
    static constexpr uint8_t MAX_BINDINGS     = 4;  ///< Bindings on one Layer.
    static constexpr uint8_t CURVE_POINTS     = 17; ///< Knots in a Binding curve.

    /**
     * @brief What one layer of a Composition draws.
     *
     * OFF through GAUGE are @ref SpliceRegionAnimKind entry for entry: a layer
     * and a splice region say the same words and take the same parameters, so
     * anything learned about one applies to the other.  METER is levelFill()'s
     * meter scoped to a layer's span.  FRAMES is a hand-drawn pixel strip.
     */
    enum class LayerKind : uint8_t {
        OFF, SOLID, PULSE, FLASH, FLOW, RAINBOW, TWINKLE, BITSCROLL, GAUGE, METER, FRAMES
    };

    /**
     * @brief How a layer lands on what the layers below it already drew.
     *
     * KEY is the one that makes stacking work: a PULSE layer over a wash
     * paints only its lit run and lets the wash show through the gaps instead
     * of blanking them.  Every generator reports a coverage per pixel - a
     * bitscroll gap, an unlit meter pixel and a half-faded spark are all
     * partly or wholly transparent - and KEY is what reads it.
     */
    enum class LayerBlend : uint8_t {
        OVER,     ///< Replace, coverage ignored. What a splice region does today.
        KEY,      ///< Replace where this layer covers; let the layers below through where it does not.
        ADD,      ///< Saturating add - accents, glows, sparks.
        SCREEN,   ///< 255 - (255-a)(255-b)/255. Brightens without clipping.
        MULTIPLY, ///< Per-channel a*b/255 - a tint or a vignette.
        MASK,     ///< This layer's brightness is how much of the layers below survives.
    };

    /// Where a driver's 0-255 comes from.
    enum class DriverKind : uint8_t {
        CONSTANT, ///< A byte the robot's own code moves, see setDriverLevel().
        SENSOR,   ///< A LevelFn, mapped and smoothed exactly as levelSource() does.
        MUSIC,    ///< A baked MusicTrack, sampled exactly as musicSync() does.
        LFO,      ///< A free-running shape on a period, off the wall clock.
    };

    enum class LfoShape : uint8_t { TRIANGLE, SAW, RAMP_DOWN, SQUARE, SINE };

    /// A layer parameter a driver is allowed to move, once per tick.
    enum class LayerParam : uint8_t {
        NONE, OPACITY, SPEED, RUN_LENGTH, LEVEL, START, WIDTH, DENSITY, MIX
    };

    /// Which pixels a layer owns.
    struct LayerSpan {
        uint8_t start;   ///< First pixel.
        uint8_t width;   ///< Pixels. 0 means "to the end of the strand".
        /// 0 draws one span; otherwise repeat it every N pixels.  A repeat is
        /// never wider than N, so a span wider than the stride (width 0
        /// included) shows its first N pixels in each and no pixel is drawn
        /// twice.
        uint8_t stride;
        /// With @c stride, sample the animation at the strand-wide pixel index
        /// rather than restarting it in each repeat. This is the difference
        /// between one scroll cut into windows (what spliceMask()'s shared
        /// overlay does) and N synchronized copies of it.  The motion runs
        /// across the whole strand as well, so a scroll, a bounce or a
        /// sparkle passes through every window rather than the first alone.
        bool    continuous;
    };

    /// How a layer lands. @c opacity is 0-255 and is bindable.
    struct LayerMix { LayerBlend blend; uint8_t opacity; };

    /**
     * @brief Ties one driver to one parameter of one layer.
     *
     * The parameter is @c atZero when the driver reads 0 and @c atFull when it
     * reads 255.  @c atFull below @c atZero reverses the binding, the same way
     * levelSource()'s @c fullAt below @c emptyAt reverses a meter.
     *
     * @c curve, when set, points at @ref CURVE_POINTS bytes that reshape the
     * driver's 0-255 before the line is applied, linearly interpolated between
     * the knots.  A curve that steps makes a binding a threshold; one that
     * eases makes it an envelope.  @c nullptr is the straight line.
     */
    struct Binding {
        LayerParam     param;   ///< NONE is an unused slot.
        uint8_t        driver;  ///< Index into Composition::drivers.
        int16_t        atZero;
        int16_t        atFull;
        const uint8_t* curve;   ///< CURVE_POINTS bytes, or nullptr.
    };

    struct LayerPaint  { uint32_t color; uint32_t color2; uint32_t bgColor; };
    /// @c speed.pixels is what a LayerParam::SPEED binding moves; the time
    /// between steps stays the layer's own.
    struct LayerMotion { uint8_t runLength; Speed speed; bool invert; bool bounce; bool seamless; };
    struct LayerFlash  { uint32_t onMs; uint32_t offMs; };

    /**
     * @brief When a layer is on, inside the composition's loop.
     *
     * Both @c startMs and @c endMs zero means "always on".  An @c endMs at or
     * before @c startMs runs across the loop point.  Two layers with
     * overlapping windows and matching fades *are* a crossfade; there is no
     * separate crossfade mechanism to learn.
     *
     * A window that starts at or past the loop point never opens, and one
     * that ends past it ends at it - which is where a loop shortened after
     * its layers were placed leaves them.
     */
    struct LayerTime {
        uint32_t loopMs;     ///< This layer's own clock; 0 uses the composition's.
        uint32_t startMs;
        uint32_t endMs;      ///< 0 with a non-zero startMs means "to the loop point".
        uint16_t fadeInMs;
        uint16_t fadeOutMs;
        bool     retrigger;  ///< Restart this layer's motion each time its window opens.
    };

    struct LayerSparkle {
        const uint32_t* palette;      ///< Colors sparks are drawn from. nullptr leaves the span dark.
        uint8_t         paletteCount;
        uint8_t         densityPct;   ///< Share of the span lit at once (0-100). Bindable.
        uint8_t         fadeStep;     ///< Brightness step per tick, so bigger is faster.
        uint8_t         bitWidth;     ///< Pixels per spark. 0, what a table from before 1.6 leaves it, is one.
    };

    struct LayerPattern {
        const BitScrollSegment* segments;      ///< The lit runs, in order.
        uint8_t                 segmentCount;
        uint8_t                 spacing;       ///< Background pixels between runs.
        bool                    repeating;     ///< Tile across the span, or scroll one copy.
    };

    struct LayerGauge {
        uint8_t    level;      ///< Starting level; bind LayerParam::LEVEL to move it.
        bool       gradient;   ///< METER: lay color -> color2 across the span.
        GaugeStyle style;      ///< GAUGE: whole-span color, or a bar.
        GaugeBlend blend;
        const GaugeStop* stops;    ///< The scale, in the driver's own units. Empty
                                   ///< falls back to color at emptyAt -> color2 at fullAt.
        uint8_t          stopCount;
        double           emptyAt;  ///< Units @c stops are given in.
        double           fullAt;
    };

    /**
     * @brief A hand-drawn pixel strip: x = pixel, y = frame.
     *
     * Each frame is run-length coded, one byte per run: the top three bits are
     * the run length minus one (1-8 pixels) and the bottom five a palette
     * index (0-31).  Palette index 0 is transparent, so a drawn shape lets the
     * layers below through around it.  Identical frames share an offset in
     * @c index, so a two-frame blink costs two runs and four index entries.
     *
     * @c width is the frame's own width, not the layer's span, so a frame
     * wider than the span can be scrolled through it.  A decoder clamps a run
     * to whatever width is left, so a hand-edited table cannot overrun.
     *
     * Each frame either holds still, showing exactly what was drawn, or
     * scrolls at the layer's LayerMotion speed, as @c scroll says.  A run of
     * scrolling frames is one continuous scroll; a frame that holds puts the
     * drawing back where it was drawn, and a scrolling frame after it starts
     * from there.
     *
     * @note Scrolls, but does not bounce - a bounce needs a padded master
     *       pattern, which a frame strip does not build.
     */
    struct LayerFrames {
        const uint8_t*  data;
        const uint16_t* index;        ///< Byte offset of each frame in @c data.
        const uint32_t* palette;      ///< Index 0 is transparent whatever color it names.
        uint8_t         paletteCount;
        uint8_t         width;        ///< Pixels per frame.
        uint8_t         count;        ///< Frames.
        uint16_t        frameMs;      ///< How long each frame holds.
        /// One flag per frame: non-zero scrolls that frame.  @c nullptr holds
        /// every frame still.
        const uint8_t*  scroll;
    };

    /**
     * @brief One layer of a Composition: what it draws, where, when, what
     *        moves it, and how it lands on the layers below it.
     *
     * A layer with no timing and no bindings, drawn OVER the whole strand, is
     * exactly one of the preset animations.  Everything else here is what a
     * splice region, the overlay buffer and the sequencer could each do
     * separately, made available to all of them at once.
     *
     * @note No default member initializers, deliberately: that keeps this a
     *       C++11 aggregate, so the PROS and VEXcode exports brace-initialize
     *       it with the same characters and both keep it @c const, in flash.
     *       Defaults live in Pattern Studio, which is where a user-facing
     *       default belongs.  Groups a kind ignores are written @c {}.
     */
    struct Layer {
        const char*  name;      ///< Shown nowhere on the robot; it is what makes the export readable.
        LayerKind    kind;
        LayerSpan    span;
        LayerMix     mix;
        Binding      bindings[MAX_BINDINGS];
        LayerPaint   paint;
        LayerMotion  motion;
        LayerFlash   flash;
        LayerTime    time;
        LayerSparkle sparkle;
        LayerPattern pattern;
        LayerGauge   gauge;
        LayerFrames  frames;
    };

    /**
     * @brief One thing a composition watches, reduced to a byte per tick.
     *
     * The three drivers that already exist in this class, plus an oscillator.
     * SENSOR is levelSource()'s reader, range, wrap and smoothing; MUSIC is
     * musicSync()'s track and sensitivity; CONSTANT is setLevel().  A
     * composition evaluates each driver exactly once per tick, before any
     * layer runs, so two layers bound to one reading cannot disagree about it
     * within a frame.
     */
    struct Driver {
        const char*       name;
        DriverKind        kind;
        uint8_t           value;        ///< CONSTANT's byte, moved by setDriverLevel().
        LevelFn           read;         ///< SENSOR. nullptr leaves the driver at 0.
        double            emptyAt;
        double            fullAt;
        bool              wrap;
        uint8_t           smoothing;    ///< 0-99, as in levelSource().
        const MusicTrack* track;        ///< MUSIC. Must outlive the strand.
        uint8_t           sensitivity;
        bool              loop;
        LfoShape          shape;        ///< LFO.
        uint32_t          periodMs;
        uint8_t           phasePct;     ///< Offset into the period, so two LFOs can differ.
    };

    /**
     * @brief A named, reusable custom animation: layers over a driver table
     *        and a clock.
     *
     * Like Profile and MusicTrack, a Composition is pointed at rather than
     * copied, so it and everything it points at must outlive the strand.
     * Pattern Studio emits all of it as namespace-scope constants.
     */
    struct Composition {
        const char*   name;
        const Layer*  layers;       ///< Drawn in order; layer 0 is the bottom.
        uint8_t       layerCount;
        const Driver* drivers;
        uint8_t       driverCount;
        uint32_t      loopMs;       ///< 0 means the timeline never wraps.
        uint32_t      bgColor;      ///< What the frame starts as, under layer 0.
    };

    /// @}

    /// @}

    // ========================================================================
    /// @name Construction
    /// @{

    /**
     * @brief Construct a strand on a standard ADI port.
     *
     * @param adiPort    ADI port number (1–8).
     * @param length     Number of LEDs.  Clamped to MAX_LEDS (64).
     * @param refreshMs  Refresh interval in milliseconds (default 20 ms = 50 Hz).
     *                   A LedGroup replaces it with its own, see
     *                   LedGroup::init().
     */
    LedStrand(uint8_t adiPort, uint8_t length, uint32_t refreshMs = 20);

    /**
     * @brief Construct a strand on an ADI expander port.
     *
     * @code{.cpp}
     * hitlib::LedStrand strand({2, 1}, 63);   // Smart Port 2, ADI port A, 63 LEDs
     * @endcode
     *
     * @param port       Smart Port of the expander, and ADI port on it.
     * @param length     Number of LEDs.  Clamped to MAX_LEDS (64).
     * @param refreshMs  Refresh interval in milliseconds (default 20 ms).
     *                   A LedGroup replaces it with its own.
     */
    LedStrand(ExpanderPort port, uint8_t length, uint32_t refreshMs = 20);

    /**
     * @brief Construct a strand on an ADI expander port, from four numbers.
     *
     * Same as `LedStrand({smartPort, adiPort}, length, refreshMs)`.  The
     * interval has no default here: three bare numbers always mean
     * `(adiPort, length, refreshMs)`, since nothing else could tell
     * `(2, 1, 63)` the expander apart from `(2, 1, 63)` the brain port.
     *
     * @param smartPort  Smart port the expander is connected to (1–21).
     * @param adiPort    ADI port on the expander (1–8).
     * @param length     Number of LEDs.  Clamped to MAX_LEDS (64).
     * @param refreshMs  Refresh interval in milliseconds.  A LedGroup
     *                   replaces it with its own.
     */
    LedStrand(uint8_t smartPort, uint8_t adiPort, uint8_t length, uint32_t refreshMs);

    /**
     * @brief Construct one strand spread across several strips, chained end
     *        to end.
     *
     * Every animation, mask and profile then runs across all of them as one
     * long strand: a pulse leaves the end of the first strip and carries on
     * at the start of the second.  Pixel 0 is the first strip's first LED,
     * and the strips follow in the order given.
     *
     * @code{.cpp}
     * // Four 20-LED strips around the chassis, as one 80-LED ring.
     * hitlib::LedStrand ring({{1, 20}, {2, 20}, {3, 20}, {4, 20}});
     *
     * // Mixing the brain's ports and an expander's works the same way.
     * hitlib::LedStrand sides({{1, 30}, {{5, 1}, 30}});
     *
     * // The back strip was mounted with its data-in at the far end, so
     * // without the true a pulse would run toward the front along it.
     * hitlib::LedStrand loop({{1, 30}, {2, 30, true}});
     * @endcode
     *
     * A strip wired backwards takes @c true as its third value (see Segment).
     * Pixel numbers stay in chain order either way, so a mask, region or
     * fill placed along the chain lands where it would on a strip wired the
     * usual way; only what that strip is sent is turned round.
     *
     * Each strip still costs its own 12 ms ADI write when its part of the
     * frame changes, the same as a strand of its own would.
     *
     * @param chain      The strips, in pixel order.  Up to MAX_SEGMENTS (8);
     *                   each is clamped to MAX_LEDS (64), and the whole chain
     *                   to MAX_CHAIN_LEDS (128), dropping whatever is past it.
     * @param refreshMs  Refresh interval in milliseconds (default 20 ms).
     *                   A LedGroup replaces it with its own.
     */
    LedStrand(std::initializer_list<Segment> chain, uint32_t refreshMs = 20);

    /// @}

    // ========================================================================
    /// @name Initialization
    /// @{

    /**
     * @brief Initialize the hardware LED object.
     *
     * Must be called from PROS `initialize()` or VEXcode's `main()` (or before
     * the first tick).
     * Safe to call multiple times, subsequent calls are no-ops.
     */
    void init();

    /**
     * @brief Advance one animation frame.
     *
     * Called automatically by LedGroup's internal task.
     * **Do not call this directly.**
     */
    void tick();

    /// @}

    // ========================================================================
    /// @name Base Animations
    ///
    /// All methods are thread-safe.
    /// Changes take effect on the next refresh tick.
    /// @{

    /**
     * @brief Turn all LEDs off (set to black).
     */
    void off();

    /**
     * @brief Set all LEDs to a solid color.
     *
     * @param color  24-bit RGB color (0xRRGGBB).
     */
    void setColor(uint32_t color);

    /**
     * @brief Animate a run of color moving across a background.
     *
     * @param color      Foreground color (0xRRGGBB).
     * @param runLength  Number of lit pixels in the run.
     * @param speed      Pixels advanced per tick, or Speed::everyMs() for
     *                   slower than one a tick.
     * @param bgColor    Background color (default black).
     * @param invert     Reverse the travel direction (default @c false).
     * @param bounce     Reverse direction at each end instead of wrapping
     *                   (default @c false).
     */
    void pulse(uint32_t color, uint8_t runLength, Speed speed,
               uint32_t bgColor = 0x000000, bool invert = false, bool bounce = false);

    /**
     * @brief Blink the whole strip on and off.
     *
     * Every pixel lights at once for @p onMs, then the whole strip shows
     * @p bgColor for @p offMs, and the cycle repeats.  On and off times are
     * independent, so rate and duty cycle are set separately.
     *
     * Durations are rounded to whole refresh ticks and clamped to a minimum
     * of one tick, so the strand can't be asked to blink faster than its
     * refresh interval (see the @c refreshMs constructor parameter).
     *
     * @param color    Lit color (0xRRGGBB).
     * @param onMs     How long the strip stays lit, in milliseconds.
     * @param offMs    How long the strip stays blank, in milliseconds.
     * @param bgColor  Background color shown while blank (default black).
     */
    void flash(uint32_t color, uint32_t onMs, uint32_t offMs, uint32_t bgColor = 0x000000);

    /**
     * @brief Scroll a two-color gradient across the strip.
     *
     * @param color1    Start color (0xRRGGBB).
     * @param color2    End color (0xRRGGBB).
     * @param speed     Pixels shifted per tick, or Speed::everyMs().
     * @param invert    Scroll in the reverse direction (default @c false).
     * @param seamless  Loop the gradient back to @p color1 instead of cutting
     *                  straight from @p color2 to @p color1 at the wrap
     *                  (default @c true).
     */
    void flow(uint32_t color1, uint32_t color2, Speed speed, bool invert = false,
              bool seamless = true);

    /**
     * @brief Scroll a full HSV rainbow across the strip.
     *
     * @param speed   Pixels shifted per tick, or Speed::everyMs().
     * @param invert  Scroll in the reverse direction (default @c false).
     */
    void rainbow(Speed speed, bool invert = false);

    /**
     * @brief Spawn randomly fading sparkles from a color palette.
     *
     * Each sparkle fades in to full brightness, holds briefly, then fades out.
     * New sparkles are spawned at most one per tick to stagger phase offsets.
     *
     * @param colors       Color palette, one color is chosen at random per sparkle.
     * @param densityPct   Target percentage of LEDs simultaneously lit (0–100).
     * @param fadeStep     Brightness step applied each tick (higher = faster fade).
     * @param bgColor      Background color shown on unlit pixels (default black).
     * @param bitWidth     Pixels per sparkle (default 1).  Wider bits light
     *                     several neighbouring pixels as one, so a sparkle reads
     *                     from further away; the strip is cut into bits from
     *                     pixel 0, and a last bit too short to be whole is as
     *                     wide as the pixels left.
     */
    void twinkle(const std::vector<uint32_t>& colors, uint8_t densityPct = 30,
                 uint8_t fadeStep = 16, uint32_t bgColor = 0x000000, uint8_t bitWidth = 1);

    /**
     * @brief Scroll a pattern of colored segments across the strip.
     *
     * @param segments   List of BitScrollSegment descriptors defining the pattern.
     * @param speed      Pixels advanced per tick, or Speed::everyMs().
     * @param invert     Scroll in the reverse direction (default @c false).
     * @param bgColor    Color shown between segments and in blank areas.
     * @param bounce     Rock the pattern back and forth instead of wrapping.
     * @param spacing    Gap pixels inserted between segments (default 5).
     * @param repeating  Tile the pattern across the whole strip (@c true) or
     *                   show a single copy of it (@c false). works for
     *                   both wrapping and @p bounce travel.
     */
    void bitscroll(const std::vector<BitScrollSegment>& segments, Speed speed,
                   bool invert = false, uint32_t bgColor = 0x000000, bool bounce = false,
                   uint8_t spacing = 5, bool repeating = true);

    /// @}

    // ========================================================================
    /// @name Level Meter & Music Sync
    ///
    /// A meter fills the strip from one end in proportion to a 0-255 level.
    /// levelFill() decides what it looks like, and one of three things decides
    /// how full it is:
    ///
    /// - setLevel(), called from your own code;
    /// - levelSource(), which polls a value for you every tick, so a battery
    ///   gauge or a motor-heat bar needs no code in your control loop at all;
    /// - musicSync(), which plays a baked MusicTrack so the strip fills and
    ///   empties in time with a song.
    ///
    /// The three are mutually exclusive: whichever was set up last is what
    /// drives the meter, so handing it to a song doesn't leave a stale sensor
    /// fighting it for the same pixels.
    /// @{

    /**
     * @brief Configure the strip as a level meter, and take manual control of it.
     *
     * Sets up the colors the meter fills with, then leaves the level itself
     * alone, drive it with setLevel().  A meter starts empty, so nothing
     * lights until the first setLevel() call.
     *
     * With @p gradient the two colors are laid out across the **whole strip**,
     * not across the lit part, so a given pixel is always the same color no
     * matter how full the meter is (a VU meter that runs green at the bottom
     * and red at the top, rather than a gradient that stretches).  With
     * @p gradient @c false only @p color is used.
     *
     * The pixel at the edge of the fill is dimmed proportionally rather than
     * snapping on, so a 30-LED strand shows a smooth ramp instead of 30
     * visible steps.
     *
     * Calling this takes the meter back to manual control: any MusicTrack or
     * levelSource() reader is detached, since either would overwrite whatever
     * setLevel() puts there on the very next tick.
     *
     * @param color    Fill color, and the gradient's start color (0xRRGGBB).
     * @param color2   Gradient end color.  Ignored unless @p gradient.
     * @param gradient Blend @p color to @p color2 across the strip.
     * @param bgColor  Color shown on the unfilled part (default black).
     * @param invert   Fill from the far end of the strip instead of pixel 0.
     */
    void levelFill(uint32_t color, uint32_t color2 = 0x000000, bool gradient = false,
                   uint32_t bgColor = 0x000000, bool invert = false);

    /**
     * @brief Set how full the meter is.
     *
     * Cheap enough to call every control-loop iteration, it only stores a
     * byte, the fill is computed at flush time.  Detaches any MusicTrack or
     * levelSource() reader, so manual updates aren't fighting one of those for
     * the same meter.
     *
     * @param level  0 = empty, 255 = the whole strip lit.
     */
    void setLevel(uint8_t level);

    /** @brief Return the meter's current level (0-255). */
    uint8_t getLevel() const { return levelValue; }

    /**
     * @brief Point the meter at a live value and let it follow it on its own.
     *
     * The strand polls @p read once per refresh tick and maps what comes back
     * onto the strip, so a gauge needs no code in your control loop: set it up
     * once (in @c initialize(), or in a ProfileMode's @c onActivate) and the
     * bar tracks the value from then on.
     *
     * Units are yours to choose, @p emptyAt and @p fullAt are given in
     * whatever the reader returns:
     *
     * @code{.cpp}
     * pros::Motor intake(11);
     *
     * strand.levelFill(0x00FF00, 0xFF0000, true);                 // green -> red scale
     * strand.levelSource([] { return intake.get_temperature(); },
     *                    20.0, 70.0);                             // cool -> shutdown hot
     *
     * // A battery gauge that empties as the battery does:
     * strand.levelSource([] { return pros::battery::get_capacity(); }, 0.0, 100.0);
     *
     * // A bar that fills once per revolution and starts over:
     * strand.levelSource([] { return arm.get_position(); }, 0.0, 360.0, true);
     * @endcode
     *
     * Values outside the range clamp to empty or full.  With @p wrap they
     * cycle instead, for a continuously turning motor or a heading: 450° of a
     * 0-360 range shows the bar a quarter full, not full.
     *
     * Putting @p fullAt below @p emptyAt is allowed and reverses the meter, so
     * a "distance remaining" bar that drains as a number climbs needs no
     * arithmetic of its own.
     *
     * This keeps the colors from the preceding levelFill(), it only changes
     * what drives the fill, so call levelFill() first.  Any MusicTrack is
     * detached.
     *
     * @param read       Value to follow.  @c nullptr clears the source, so
     *                   exported code can name a hook before it is assigned.
     * @param emptyAt    Reading that shows an empty strip.
     * @param fullAt     Reading that shows a full one.
     * @param wrap       Cycle back to empty past @p fullAt instead of clamping.
     * @param smoothing  0-99.  How much of the previous frame's fill to keep
     *                   each tick: 0 follows the value exactly, higher glides
     *                   toward it, for noisy readings (motor velocity, current
     *                   draw).  The bar still reaches its target, over a few
     *                   ticks.
     */
    void levelSource(LevelFn read, double emptyAt, double fullAt, bool wrap = false,
                     uint8_t smoothing = 0);

    /**
     * @brief Stop following a levelSource() reader, leaving the fill where it is.
     *
     * The meter stays set up and keeps its colors, it just stops moving on its
     * own, so setLevel() takes over cleanly from here.
     */
    void clearLevelSource();

    /** @brief Whether a levelSource() reader is currently driving the meter. */
    bool levelSourceActive() const { return levelRead != nullptr; }

    /**
     * @brief Fill the meter from a baked music envelope, and start playing it.
     *
     * Playback is anchored to the wall clock at the moment of this call, so
     * call it when the music starts, typically from a ProfileMode's
     * @c onActivate.  The strand then tracks the song on its own with no
     * further calls, samples are interpolated between frames so the fill stays
     * smooth even when the envelope's frame rate is coarser than @c refreshMs.
     *
     * Colors, @p gradient and @p invert behave exactly as in levelFill().  Any
     * levelSource() reader is detached, the song drives the meter now.
     *
     * @param track        Envelope to play.  Must outlive the strand.
     * @param color        Fill color, and the gradient's start color.
     * @param color2       Gradient end color.  Ignored unless @p gradient.
     * @param gradient     Blend @p color to @p color2 across the strip.
     * @param bgColor      Color shown on the unfilled part.
     * @param invert       Fill from the far end of the strip.
     * @param sensitivity  Gain applied to every sample, as a percentage.
     *                     100 = the envelope as baked, higher makes quiet
     *                     passages reach further up the strip (and clips loud
     *                     ones at full), lower makes it more selective.
     * @param loop         Restart from the top when the song ends instead of
     *                     going dark.
     */
    void musicSync(const MusicTrack& track, uint32_t color, uint32_t color2 = 0x000000,
                   bool gradient = false, uint32_t bgColor = 0x000000, bool invert = false,
                   uint8_t sensitivity = 100, bool loop = false);

    /**
     * @brief Jump playback to a position in the song.
     *
     * @param positionMs  Offset from the start of the track, in milliseconds.
     */
    void musicSeek(uint32_t positionMs);

    /**
     * @brief Pause or resume playback, holding the meter where it is.
     *
     * @param paused  @c true to pause, @c false to resume.
     */
    void musicPause(bool paused = true);

    /** @brief Whether a MusicTrack is attached and not paused. */
    bool musicPlaying() const { return musicTrack != nullptr && !musicPaused; }

    /** @brief Current playback position in milliseconds. */
    uint32_t musicPositionMs() const;

    /**
     * @brief Change the gain applied to envelope samples without restarting.
     *
     * @param pct  Percentage gain, see musicSync()'s @p sensitivity.
     */
    void setSensitivity(uint8_t pct);

    /// @}

    // ========================================================================
    /// @name Masks
    ///
    /// Overrides part of the strip: as equal alternating bins sharing one
    /// overlay buffer (spliceMask), as arbitrarily placed regions that each
    /// animate independently (spliceMaskCustom), as a wipe that spreads across
    /// the strip and back (spreadMask), or as twinkling bits (twinkleMask).
    /// A strand has one mask at a time: whichever was called most recently is
    /// what's active, and clearSpliceMask() removes it whichever it is.
    /// @{

    /**
     * @brief Apply a splice mask that overrides alternating equal-width bins.
     *
     * The strip is divided into `sections + 1` equal bins.  Even-indexed bins
     * (or their complement when @p invert is @c true) show @p bgColor, or the
     * overlay buffer when @p useOverlay is @c true, instead of the active base
     * animation.
     *
     * @param sections      Number of divider boundaries (e.g. 1 = two halves).
     *                      Pass @c 0 to disable.
     * @param invert        Swap which bins are overridden (default @c false).
     * @param alternating   Toggle @p invert automatically every @p altPeriodMs.
     * @param altPeriodMs   Toggle period when @p alternating is @c true (ms).
     * @param bgColor       Color shown in masked bins when @p useOverlay is @c false.
     * @param useOverlay    Show the overlay buffer in masked bins instead of @p bgColor.
     */
    void spliceMask(uint8_t sections, bool invert = false, bool alternating = false,
                    uint32_t altPeriodMs = 100, uint32_t bgColor = 0x000000,
                    bool useOverlay = false);

    /**
     * @brief Apply a splice mask made of independently placed/sized regions,
     * each running its own animation.
     *
     * Unlike spliceMask(), regions can start and end anywhere on the strip,
     * don't alternate, and each gets a dedicated buffer generated over just
     * its own width. So e.g. one region can rainbow-scroll while another
     * pulses, simultaneously. Regions stay fixed until spliceMaskCustom() or
     * clearSpliceMask() is called again. Later entries win where regions
     * overlap.
     *
     * A GAUGE region is the same idea pointed at a sensor instead of a clock:
     * it polls its own reader every tick and colors itself off its own scale,
     * so one strip can carry several independent meters.  The strand-wide
     * levelFill() meter is limited to one per strand.
     *
     * @code{.cpp}
     * // Six motors, six segments of a 60-LED strip under the drivebase, each
     * // colored by how hot its own motor is.
     * pros::Motor drive[6] = {...};
     * std::vector<LedStrand::SpliceRegion> regions;
     * for (uint8_t i = 0; i < 6; ++i) {
     *     regions.push_back(LedStrand::motorHeatGauge(i * 10, 9, readers[i]));
     * }
     * strand.spliceMaskCustom(regions);
     * @endcode
     *
     * @param regions  Override regions to apply.
     */
    void spliceMaskCustom(const std::vector<SpliceRegion>& regions);

    /**
     * @brief Set how full a hand-driven GAUGE region is.
     *
     * The counterpart of setLevel() for one region rather than the whole
     * strand, for gauges whose SpliceRegion left @c read as @c nullptr.  Cheap
     * enough to call every control-loop iteration: it stores a byte, and the
     * region repaints at flush time.
     *
     * Ignored for a region that has a reader of its own, which would overwrite
     * it on the very next tick anyway.
     *
     * @param regionIdx  Index into the vector last handed to spliceMaskCustom().
     * @param level      0 = the bottom of the region's scale, 255 = the top.
     */
    void setRegionLevel(uint8_t regionIdx, uint8_t level);

    /**
     * @brief A GAUGE region pre-loaded with the V5 motor's own heat schedule.
     *
     * The stops are the temperatures the motor itself changes behaviour at, so
     * the colors mean something specific rather than being a pretty ramp:
     *
     * | Reading  | Color   | What the motor is doing        |
     * |----------|---------|--------------------------------|
     * | 20 °C    | green   | cold, full power               |
     * | 45 °C    | yellow  | warm, nearing the first cut    |
     * | 55 °C    | orange  | current limited to 50%         |
     * | 60 °C    | red     | current limited to 25%         |
     * | 65 °C    | deep red| current limited to 12.5%       |
     * | 70 °C    | magenta | shut down                      |
     *
     * Magenta at the top is deliberate - it is the one color on the scale that
     * cannot be mistaken for "a bit hotter than the last one".
     *
     * The returned region is an ordinary SpliceRegion, so anything about it can
     * be overridden before it is handed to spliceMaskCustom().
     *
     * @param start  First pixel of the segment.
     * @param width  Number of pixels in it.
     * @param read   Reader returning a motor temperature in °C.  @c nullptr
     *               leaves the segment driven by setRegionLevel().
     */
    static SpliceRegion motorHeatGauge(uint8_t start, uint8_t width, LevelFn read = nullptr);

    /**
     * @brief Show the animation only across a stretch that spreads over the
     *        strip and back.
     *
     * The animation keeps running underneath; the spread decides how much of
     * it shows.  Outside the spread the strip shows @p bgColor, or the overlay
     * buffer with @p useOverlay.  The spread starts empty and grows by
     * @p speed until it covers the whole strip.  Then, with @p bounce, it
     * shrinks back the way it came; without, it starts over from empty.
     *
     * @code{.cpp}
     * using LS = hitlib::LedStrand;
     * strand.rainbow(1);
     * strand.spreadMask(LS::Speed::everyMs(40), LS::SpreadOrigin::CENTER, false, true);
     * @endcode
     *
     * @param speed       How far the edge moves per step, or Speed::everyMs()
     *                    for slower.  A CENTER spread moves both of its edges
     *                    this far.
     * @param origin      Spread from pixel 0, or from the middle.
     * @param invert      Spread from the far end instead, or from both ends in
     *                    toward the middle.
     * @param bounce      Shrink back once full, instead of starting over.
     * @param bgColor     Color shown outside the spread when @p useOverlay is
     *                    @c false.
     * @param useOverlay  Show the overlay buffer outside the spread instead.
     */
    void spreadMask(Speed speed, SpreadOrigin origin = SpreadOrigin::EDGE, bool invert = false,
                    bool bounce = false, uint32_t bgColor = 0x000000, bool useOverlay = false);

    /**
     * @brief Show the animation only through twinkling bits.
     *
     * twinkle()'s sparkles, used as a mask: each bit fades in to show the
     * animation underneath, holds, and fades back out to @p bgColor (or the
     * overlay buffer, with @p useOverlay).  With @p invert the bits work the
     * other way round, punching @p bgColor through a strip that otherwise
     * shows the animation.
     *
     * @param densityPct  Share of the strip showing through at once (0-100).
     * @param fadeStep    Brightness step per tick, so bigger is faster.
     * @param bitWidth    Pixels per bit.
     * @param invert      Bits hide the animation instead of revealing it.
     * @param bgColor     Color shown where the animation is masked, when
     *                    @p useOverlay is @c false.
     * @param useOverlay  Show the overlay buffer there instead.
     */
    void twinkleMask(uint8_t densityPct = 30, uint8_t fadeStep = 16, uint8_t bitWidth = 1,
                     bool invert = false, uint32_t bgColor = 0x000000, bool useOverlay = false);

    /**
     * @brief Remove the active mask, whichever kind is active.
     */
    void clearSpliceMask();

    /// @}

    // ========================================================================
    /// @name Transitions
    /// @{

    /**
     * @brief Wipe from what is showing now into whatever is set up next.
     *
     * Takes a snapshot of the frame on the strip, then for the next
     * @c durationMs wipes it away to reveal what the strand is showing by
     * then - so call this, then start the animation the wipe should bring in.
     * The new animation runs from the moment it is set, so it is already
     * moving as it is revealed; the old frame holds still while it goes.
     *
     * A Sequencer::Phase with a transition calls this for you as it starts.
     * Calling it again mid-wipe starts a new wipe from the frame showing
     * then, half-wiped as it is.
     *
     * @code{.cpp}
     * using LS = hitlib::LedStrand;
     * strand.transition({LS::TransitionKind::TWINKLE, 800, LS::SpreadOrigin::EDGE, false, 2, 64});
     * strand.setColor(0x00FF00);   // green twinkles in, two pixels at a time
     * @endcode
     *
     * @param t  The wipe.  A CUT, or a zero @c durationMs, stops any wipe in
     *           progress and does nothing else.
     */
    void transition(const Transition& t);

    /** @brief Whether a transition() is still wiping. */
    bool transitioning() const { return transKind != TransitionKind::CUT; }

    /// @}

    // ========================================================================
    /// @name Overlay Animations
    ///
    /// Write into a second buffer, shown in spliceMask()'s masked bins instead
    /// of @c bgColor when @p useOverlay is set.
    /// @{

    /** @brief Set the overlay buffer to a solid color. */
    void overlaySetColor(uint32_t color);

    /**
     * @brief Animate a moving run in the overlay buffer (see pulse()).
     *
     * No @c bounce, for the reason overlayBitscroll() gives.
     */
    void overlayPulse(uint32_t color, uint8_t runLength, Speed speed,
                      uint32_t bgColor = 0x000000, bool invert = false);

    /** @brief Blink the overlay buffer on and off (see flash()). */
    void overlayFlash(uint32_t color, uint32_t onMs, uint32_t offMs,
                      uint32_t bgColor = 0x000000);

    /**
     * @brief Scroll a gradient in the overlay buffer.
     * @param color1    Color the gradient starts from (0xRRGGBB).
     * @param color2    Color the gradient runs to (0xRRGGBB).
     * @param speed     Pixels shifted per refresh tick, or Speed::everyMs().
     * @param seamless  Loop the gradient back to @p color1 instead of
     *                  cutting straight from @p color2 to @p color1 at the
     *                  wrap (default @c true). See @c flow().
     * @param invert    Scroll in the reverse direction (default @c false).
     *                  Last, unlike flow()'s, so a call written before it
     *                  existed still means what it did.
     */
    void overlayFlow(uint32_t color1, uint32_t color2, Speed speed, bool seamless = true,
                     bool invert = false);

    /** @brief Scroll a rainbow in the overlay buffer (see rainbow()). */
    void overlayRainbow(Speed speed, bool invert = false);

    /** @brief Sparkle random pixels of the overlay buffer (see twinkle()). */
    void overlayTwinkle(const std::vector<uint32_t>& colors, uint8_t densityPct = 30,
                        uint8_t fadeStep = 16, uint32_t bgColor = 0x000000,
                        uint8_t bitWidth = 1);

    /**
     * @brief Scroll a repeating bit pattern in the overlay buffer (see bitscroll()).
     *
     * No @c bounce: the overlay is a single scrolling buffer, and bouncing
     * needs a wider master pattern to slide a window over.  Use the base
     * animation for that.
     */
    void overlayBitscroll(const std::vector<BitScrollSegment>& segments, Speed speed,
                          bool invert = false, uint32_t bgColor = 0x000000,
                          uint8_t spacing = 5, bool repeating = true);

    /// @}

    // ========================================================================
    /// @name Brightness
    /// @{

    /**
     * @brief Set global brightness for this strand.
     *
     * Applied non-destructively at flush time, the animation buffers are not
     * modified.  Does not require the strand to be re-initialized.
     *
     * @param pct  Brightness percentage (0 = off, 100 = full).  Clamped to
     *             [0, 100].
     */
    void setBrightness(uint8_t pct);

    /**
     * @brief Return the current brightness percentage.
     */
    uint8_t getBrightness() const { return brightnessPct; }

    /// @}

    // ========================================================================
    /// @name Profile System
    /// @{

    /**
     * @brief Attach a Profile and reset the mode stack.
     *
     * @param profile  Pointer to a statically-allocated Profile.
     *                 Must remain valid for the lifetime of the attachment.
     */
    void attachProfile(const Profile* profile);

    /**
     * @brief Detach the active profile and turn the strand off.
     */
    void detachProfile();

    /**
     * @brief Push a persistent mode onto the mode stack.
     *
     * The mode remains active until explicitly removed with deactivateMode().
     * Calling this for an already-active persistent mode is a no-op.
     *
     * @param modeIdx  Index into the attached profile's modes array.
     */
    void activateMode(uint8_t modeIdx);

    /**
     * @brief Push a timed mode onto the mode stack.
     *
     * The mode auto-expires after @p durationMs milliseconds.  If the mode
     * is already active (as a timed entry), its deadline is extended rather
     * than creating a duplicate.
     *
     * @param modeIdx     Index into the attached profile's modes array.
     * @param durationMs  How long the mode stays active (milliseconds).
     */
    void activateModeTimed(uint8_t modeIdx, uint32_t durationMs);

    /**
     * @brief Remove a mode from the stack immediately.
     *
     * @param modeIdx  Index of the mode to remove.
     */
    void deactivateMode(uint8_t modeIdx);

    /// @}

    // ========================================================================
    // ========================================================================
    /// @name Custom Animations
    ///
    /// A Composition is a custom animation designed in Pattern Studio's
    /// Animator: an ordered list of Layers over a shared Driver table and a
    /// shared clock.  It replaces the base animation and composites into the
    /// same buffer, so brightness, the overlay, the splice mask and the
    /// profile system all keep working on top of it.
    /// @{

    /**
     * @brief Play a Composition as this strand's base animation.
     *
     * Detaches any levelSource() reader and MusicTrack - the composition's own
     * drivers take those over - and takes down any preset animation, the same
     * way calling rainbow() takes down a bouncing pulse.
     *
     * All one-time work happens here: twinkle state, bitscroll patterns,
     * decoded frames, resolving gauge scales onto the 0-255 axis.  A tick
     * allocates nothing.  Re-installing the composition that is already
     * playing is a no-op, so a Sequencer phase that re-issues it every 200 ms
     * costs a pointer comparison rather than heap churn.
     *
     * @param comp  The composition.  It, and everything it points at, must
     *              outlive the strand: Pattern Studio emits all of it as
     *              namespace-scope constants.
     */
    void compose(const Composition& comp);

    /// @brief Stop playing a composition and leave the strand dark.
    void clearComposition();

    /// @brief Whether a composition is currently driving the base buffer.
    bool composing() const { return composition != nullptr; }

    /**
     * @brief Set a CONSTANT driver's value - the composition's setLevel().
     *
     * Cheap enough to call every control-loop iteration: it stores a byte.
     * Ignored for a driver that is not CONSTANT, which would overwrite it on
     * the next tick anyway.
     *
     * @param driverIdx  Index into the composition's drivers array.  The
     *                   generated header names each one as a @c driver::
     *                   constant.
     * @param value      0-255, mapped onto each binding's atZero..atFull.
     */
    void setDriverLevel(uint8_t driverIdx, uint8_t value);

    /// @brief What a driver last read.  Useful for checking a binding.
    uint8_t getDriverLevel(uint8_t driverIdx) const;

    /// @brief Restart the composition's timeline, and every layer, from zero.
    void restartComposition();

    /**
     * @brief Seek every MUSIC driver in this composition.  See musicSeek().
     *
     * A composition may hold several MUSIC drivers on different bands of the
     * same song; they share one clock, so they stay in step by construction
     * and this moves all of them together.  The layers' timing windows run on
     * that clock too, so they move with it.
     */
    void composeMusicSeek(uint32_t positionMs);

    /**
     * @brief Pause or resume every MUSIC driver in this composition.
     *
     * The layers' timing windows share the drivers' clock, so pausing holds
     * them where they are as well, and resuming carries on from there.  Each
     * layer's own motion - a scroll, a flash - keeps running meanwhile.
     */
    void composeMusicPause(bool paused = true);

    /// @brief Position of this composition's music clock, in milliseconds.
    uint32_t composeMusicPositionMs() const;

    /// @}

    // ========================================================================
    /// @name Accessors
    /// @{

    /** @brief Return the number of LEDs in this strand, across every strip of a chain. */
    uint8_t getLength() const { return length; }

    /** @brief Return how many strips this strand drives: 1, or a chain's count. */
    uint8_t getSegmentCount() const { return segmentCount; }

    /// @}

    // ========================================================================
    /// Maximum supported LEDs on one strip, i.e. one ADI port.
    static constexpr uint8_t MAX_LEDS = 64;
    /// Most strips one chained strand can drive.
    static constexpr uint8_t MAX_SEGMENTS = 8;
    /// Most LEDs one strand drives across a whole chain.
    static constexpr uint8_t MAX_CHAIN_LEDS = 128;

private:
    // LedGroup sets refreshMs to the interval it actually ticks the strand at.
    friend class LedGroup;

    // ---- Hardware ----
    // One entry per strip; a strand that is not a chain has one. `length` is
    // the whole strand's, the sum of theirs.
    struct Strip {
        uint8_t           smartPort;
        uint8_t           adiPort;
        uint8_t           length;
        bool              reversed; ///< Sent its part of the frame back to front.
        uint32_t          sentMs;   ///< When its part of the frame last went out.
        platform::AdiLed* led;
    };
    Strip    strips[MAX_SEGMENTS] = {};
    uint8_t  segmentCount = 1;
    uint8_t  length;
    uint32_t refreshMs;
    platform::Mutex mutex;

    // The last frame actually written to the strips. flushBuffer() skips a
    // strip whose part of the frame is identical to it, since the pixels hold
    // what they were sent, and every write costs a 12 ms slot on the shared
    // ADI. It is also what a transition() wipes away from.
    uint32_t lastSent[MAX_CHAIN_LEDS] = {0};
    bool     lastSentValid      = false;

    // Called by LedGroup, which ticks the strand at its own interval: every
    // duration converted to ticks after this counts at that rate.
    void setRefreshMs(uint32_t ms);

    enum class AnimMode : uint8_t { STATIC, SHIFT, TWINKLE, FLASH, LEVEL };

    // Paces a moving animation: due() is true once every `hold` refresh
    // ticks. A hold of 1 - what a bare-number Speed gives - moves on every
    // tick, which is all `speed` could do before it could be slower.
    struct StepPacer {
        uint16_t hold    = 1;
        uint16_t counter = 0;
        void reset(uint16_t ticks) { hold = ticks ? ticks : 1; counter = 0; }
        void restart() { counter = 0; }
        bool due() {
            if (++counter < hold) return false;
            counter = 0;
            return true;
        }
    };

    // Base buffer
    AnimMode              animMode     = AnimMode::STATIC;
    int                   shiftStep    = 0;
    uint8_t               shiftVariant = 0;   // pixels per step, pre-reversed for invert
    StepPacer             shiftPacer;         // SHIFT, and both bounces
    std::vector<uint32_t> buffer;

    // Pulse bounce
    uint8_t  pulseRunLen = 0;
    uint32_t pulseColor  = 0;
    uint32_t pulseBg     = 0;
    uint8_t  pulseSpeed  = 1;
    int16_t  pulseOffset = 0;
    int8_t   pulseDir    = 1;

    // Flash - whole-strip blink driven by tick counts rather than a shifting
    // buffer, so the lit and blank halves can have independent durations.
    uint32_t flashColor    = 0;
    uint32_t flashBgColor  = 0;
    uint16_t flashOnTicks  = 1;
    uint16_t flashOffTicks = 1;
    uint16_t flashCounter  = 0;
    bool     flashLit      = true;

    // Bitscroll bounce
    std::vector<uint32_t> bitscrollMaster;
    int16_t               bounceScrollPos = 0;
    int8_t                bounceScrollDir = 1;
    uint8_t               bounceSpeed     = 1;

    // Twinkle - per-pixel fade state for one buffer. A struct rather than loose
    // members because three different buffers animate this way: the base
    // strand, the overlay, and any number of custom regions.
    // The per-bit arrays hold one entry per bit, not per pixel: a bit is
    // `bitWidth` pixels wide, and pixel i belongs to bit i / bitWidth.
    struct TwinkleState {
        std::vector<uint8_t>  level;
        std::vector<uint8_t>  target;
        std::vector<uint8_t>  colorIdx;
        std::vector<uint8_t>  holdTicks;
        std::vector<uint32_t> palette;
        uint8_t  densityPct = 30;
        uint8_t  fadeStep   = 16;
        uint8_t  bitWidth   = 1;
        uint32_t bgColor    = 0x000000;

        void reset(uint8_t width, const std::vector<uint32_t>& colors, uint8_t density,
                   uint8_t fade, uint32_t bg, uint8_t bits = 1);
        // Bits it takes to cover @p width pixels, the last one possibly short.
        uint8_t bitsFor(uint8_t width) const {
            return (uint8_t)(((uint16_t)width + bitWidth - 1) / bitWidth);
        }
    };
    static void advanceTwinkleInto(TwinkleState& t, std::vector<uint32_t>& buf);

    // advanceTwinkleInto() split in two, so a custom-animation layer can take
    // the per-pixel brightness in @c t.level as coverage and blend it over
    // whatever is beneath instead of over a single background color.
    // advanceTwinkleInto() is exactly these two in sequence. Both take a
    // width in pixels.
    static void stepTwinkle(TwinkleState& t, uint8_t width);
    static void paintTwinkleInto(const TwinkleState& t, std::vector<uint32_t>& buf, uint8_t width);

    TwinkleState twinkleState;

    // Level meter - buffer holds the meter's colors across the whole strip and
    // levelValue picks how much of it is revealed, so changing the level costs
    // nothing until flush time.
    uint8_t  levelValue  = 0;
    uint32_t levelBg     = 0x000000;
    bool     levelInvert = false;

    // Live fill source - a reader polled once per tick plus the range that
    // maps what it returns onto 0-255. levelPrimed exists so the first sample
    // after levelSource() lands instantly instead of the bar crawling up to it
    // from empty through the smoothing filter.
    LevelFn  levelRead   = nullptr;
    double   levelEmptyAt = 0.0;
    double   levelFullAt  = 100.0;
    bool     levelWrap    = false;
    uint8_t  levelSmooth  = 0;
    bool     levelPrimed  = false;

    // Music sync - playback is a wall-clock anchor rather than a tick counter,
    // so a strand that misses a tick (or refreshes slowly) stays in step with
    // the music instead of drifting behind it.
    const MusicTrack* musicTrack   = nullptr;
    uint32_t musicAnchorMs         = 0;  // millis() that corresponds to position 0
    uint32_t musicPausedAt         = 0;  // position held while paused
    bool     musicPaused           = false;
    bool     musicLoop             = false;
    uint8_t  musicSensitivity      = 100;

    // Splice mask, and the spread and twinkle masks, which take its place
    enum class SpliceMode : uint8_t { SPLIT, CUSTOM, SPREAD, TWINKLE };

    // Runtime animation state for one CUSTOM-mode region, a scaled-down
    // version of the overlay buffer (buffer + shift step/speed), but one per
    // region instead of shared.
    struct SpliceRegionState {
        uint8_t                start = 0;
        std::vector<uint32_t>  buffer;
        int                    shiftStep  = 0;
        uint8_t                shiftSpeed = 0;
        StepPacer              shiftPacer;
        // Index in the vector the caller passed to spliceMaskCustom(), which
        // is what setRegionLevel() addresses. Not the index in spliceRegions:
        // SOLID and OFF regions never get a state, so the two disagree.
        uint8_t  userIdx = 0;
        // GAUGE - a per-region copy of the strand-wide meter's state, so each
        // segment follows its own reading at its own range and smoothing.
        bool       gauge       = false;
        LevelFn    levelRead   = nullptr;
        double     levelEmptyAt = 0.0;
        double     levelFullAt  = 100.0;
        bool       levelWrap    = false;
        uint8_t    levelSmooth  = 0;
        uint8_t    levelValue   = 0;
        bool       levelPrimed  = false;
        bool       gaugeInvert  = false;
        GaugeStyle gaugeStyle   = GaugeStyle::HEAT;
        GaugeBlend gaugeBlend   = GaugeBlend::LERP;
        uint32_t   gaugeBg      = 0x000000;
        // The scale, resolved onto the 0-255 axis levelValue lives on, so a
        // tick only has to bracket a byte rather than redo the mapping.
        std::vector<std::pair<uint8_t, uint32_t>> gaugeStops;
        // FLASH regions blink their whole buffer on a tick timer instead of
        // shifting it, mirroring the strand-wide flash state.
        bool     flashing      = false;
        uint32_t flashColor    = 0;
        uint32_t flashBgColor  = 0;
        uint16_t flashOnTicks  = 1;
        uint16_t flashOffTicks = 1;
        uint16_t flashCounter  = 0;
        bool     flashLit      = true;
        // TWINKLE regions repaint their buffer every tick rather than shifting
        // it, the same way the strand-wide twinkle does.
        bool         twinkling = false;
        TwinkleState twinkle;
    };

    bool              spliceActive      = false;
    SpliceMode        spliceMode        = SpliceMode::SPLIT;
    uint8_t           spliceSections    = 0;
    bool              spliceInvert      = false;
    bool              spliceAlternating = false;
    uint32_t          spliceAltMs       = 100;
    uint32_t          spliceBgColor     = 0x000000;
    bool              spliceUseOverlay  = false;
    bool              spliceAltPhase    = false;
    uint32_t          spliceLastToggleMs = 0;
    std::vector<bool>     spliceShowAnim;
    std::vector<uint32_t> splicePixelBg;
    std::vector<bool>     splicePixelUseOverlay;
    std::vector<int16_t>  splicePixelRegionIdx;  // CUSTOM mode: index into spliceRegions, or -1
    std::vector<SpliceRegionState> spliceRegions; // CUSTOM mode only
    // Bumped whenever the mask is replaced. advanceSpliceRegions() releases
    // the mutex to run a gauge's reader (user code, which may call back in),
    // and compares this afterwards to notice that the regions it was walking
    // are gone.
    uint32_t spliceGeneration = 0;

    // Spread mask - how many ranks (see spreadRank()) the spread covers, and
    // which way it is going. spliceBgColor/spliceUseOverlay above say what
    // shows outside it.
    SpreadOrigin spreadOrigin = SpreadOrigin::EDGE;
    bool         spreadInvert = false;
    bool         spreadBounce = false;
    uint8_t      spreadPos    = 0;
    int8_t       spreadDir    = 1;
    uint8_t      spreadSpeed  = 1;
    StepPacer    spreadPacer;

    // Twinkle mask - a twinkle whose levels are how much of the animation
    // shows, rather than how bright a spark is. spliceInvert swaps that round.
    TwinkleState maskTwinkle;

    // Transition - the frame that was showing when transition() was called,
    // and how far the wipe away from it has got. transKind is CUT whenever no
    // wipe is running.
    TransitionKind        transKind     = TransitionKind::CUT;
    SpreadOrigin          transOrigin   = SpreadOrigin::EDGE;
    bool                  transInvert   = false;
    uint8_t               transBitWidth = 1;
    uint8_t               transFadeStep = 255;
    uint16_t              transTicks    = 1;
    uint16_t              transElapsed  = 0;
    std::vector<uint32_t> transFrom;
    // TWINKLE: the tick each bit starts coming in on, drawn at random so the
    // last bit is fully in by transTicks.
    std::vector<uint16_t> transBitStart;

    // Overlay buffer
    AnimMode              overlayAnimMode   = AnimMode::STATIC;
    int                   overlayShiftStep  = 0;
    uint8_t               overlayShiftSpeed = 0;
    StepPacer             overlayShiftPacer;
    std::vector<uint32_t> overlayBuffer;

    // Overlay flash - mirrors the base flash state above.
    uint32_t overlayFlashColor    = 0;
    uint32_t overlayFlashBgColor  = 0;
    uint16_t overlayFlashOnTicks  = 1;
    uint16_t overlayFlashOffTicks = 1;
    uint16_t overlayFlashCounter  = 0;
    bool     overlayFlashLit      = true;

    // Overlay twinkle - mirrors the base twinkle state above.
    TwinkleState overlayTwinkleState;

    // Brightness
    uint8_t brightnessPct = 100;

    // Profile state
    struct ModeEntry {
        uint8_t  modeIdx;
        bool     persistent;
        uint32_t endMs;
    };
    const Profile*         activeProfile = nullptr;
    std::vector<ModeEntry> modeStack;
    int16_t                lastModeIdx = -1;

    // Internal implementations (no lock)
    void setColorNL(uint32_t color);
    void pulseNL(uint32_t color, uint8_t runLen, Speed speed, uint32_t bg, bool invert, bool bounce);
    void flashNL(uint32_t color, uint32_t onMs, uint32_t offMs, uint32_t bg);
    void flowNL(uint32_t c1, uint32_t c2, Speed speed, bool invert, bool seamless);
    void rainbowNL(Speed speed, bool invert);
    void twinkleNL(const std::vector<uint32_t>& colors, uint8_t densityPct,
                   uint8_t fadeStep, uint32_t bgColor, uint8_t bitWidth);
    void bitscrollNL(const std::vector<BitScrollSegment>& segments, Speed speed, bool invert,
                     uint32_t bgColor, bool bounce, uint8_t spacing, bool repeating);

    void overlaySetColorNL(uint32_t color);
    void overlayPulseNL(uint32_t color, uint8_t runLen, Speed speed, uint32_t bg, bool invert);
    void overlayFlashNL(uint32_t color, uint32_t onMs, uint32_t offMs, uint32_t bg);
    void overlayFlowNL(uint32_t c1, uint32_t c2, Speed speed, bool seamless, bool invert);
    void overlayRainbowNL(Speed speed, bool invert);
    void overlayTwinkleNL(const std::vector<uint32_t>& colors, uint8_t densityPct,
                          uint8_t fadeStep, uint32_t bgColor, uint8_t bitWidth);
    void overlayBitscrollNL(const std::vector<BitScrollSegment>& segments, Speed speed,
                            bool invert, uint32_t bgColor, uint8_t spacing, bool repeating);

    void levelFillNL(uint32_t color, uint32_t color2, bool gradient, uint32_t bg, bool invert);

    // One tile of a bitscroll pattern: the lit runs and the gaps between them.
    // `contentLen` comes back as the size up to the last lit pixel, i.e.
    // without the trailing gap, which only separates one tile from the next.
    static std::vector<uint32_t> buildBitscrollUnit(const std::vector<BitScrollSegment>& segments,
                                                    uint32_t bgColor, uint8_t spacing,
                                                    size_t& contentLen);
    // The same tile, from a plain array rather than a vector, because a custom
    // animation's segments live in a flash-resident table with no vector to
    // hand over. @p coverage, when given, comes back the same length as the
    // returned unit: 255 for a lit pixel and 0 for a gap, filled in the same
    // loop so it cannot drift from the colors. The vector overload forwards
    // here.
    static std::vector<uint32_t> buildBitscrollUnit(const BitScrollSegment* segments,
                                                    uint8_t count, uint32_t bgColor,
                                                    uint8_t spacing, size_t& contentLen,
                                                    std::vector<uint8_t>* coverage = nullptr);
    // Lay one tile into `buf` across `width` pixels - tiled when `repeating`,
    // a single copy on `bgColor` when not - and return the per-step shift that
    // scrolls it. Shared by the base, overlay, and per-region bitscrolls.
    static uint8_t fillBitscrollBuffer(std::vector<uint32_t>& buf,
                                       const std::vector<uint32_t>& unit, size_t contentLen,
                                       uint8_t width, uint32_t bgColor, uint8_t speed,
                                       bool invert, bool repeating);

    void advanceLevel();
    void advanceFlash();
    void advanceOverlayFlash();
    void advancePulseBounce();
    void advanceBitscrollBounce();
    void fillBitscrollFromMaster();
    void advanceSpliceAlternating(uint32_t nowMs);
    void advanceSpliceRegions();
    void advanceSpreadMask();
    void paintGaugeRegion(SpliceRegionState& state);
    void rebuildSpliceMask();
    // How much of the animation a SPREAD or TWINKLE mask lets through at
    // pixel @p i, 0-255.
    uint8_t maskCoverage(uint8_t i) const;
    // How much of the new frame a transition shows at pixel @p i, 0-255,
    // given the cutoff the SPREAD kind works out once per frame.
    uint8_t transitionCoverage(uint8_t i, uint32_t full, uint8_t frac) const;

    // A spread covers a strip of @p len pixels in spreadRanks() steps, and
    // pixel @p i is covered once the spread has reached spreadRank(). EDGE
    // has a rank per pixel; CENTER one per pair of pixels equally far from
    // the middle (and a lone middle pixel, on an odd strip).
    static uint8_t spreadRanks(uint8_t len, SpreadOrigin origin);
    static uint8_t spreadRank(uint8_t i, uint8_t len, SpreadOrigin origin, bool invert);

    void     shiftBuffer();
    void     shiftOverlayBuffer();
    void     flushBuffer(uint32_t now);   // called with the mutex held; gives it back
    uint32_t levelPixel(uint8_t i, uint32_t full, uint8_t frac) const;
    uint8_t  sampleMusicNL(uint32_t positionMs) const;
    uint8_t  mapLevelNL(double raw) const;
    uint8_t  smoothLevelNL(uint8_t target) const;

    // The mapping and smoothing above, with the range passed in rather than
    // read off the strand, so a gauge region can reuse them for its own.
    static uint8_t mapLevelTo(double raw, double emptyAt, double fullAt, bool wrap);
    static uint8_t smoothLevelTo(uint8_t target, uint8_t current, uint8_t smoothing, bool wrap);
    static uint32_t gaugeColorAt(const SpliceRegionState& state, uint8_t level);

    // gaugeColorAt()'s scale lookup, with the stops passed in rather than read
    // off a region, so a custom-animation layer can carry its own scale.
    // gaugeColorAt() is a thin wrapper over this.
    static uint32_t scaleColorAt(const std::pair<uint8_t, uint32_t>* stops, uint8_t count,
                                 GaugeBlend blend, uint32_t bgColor, uint8_t level);

    // sampleMusicNL()'s body, with the track and playback options passed in
    // rather than read off the strand, so a custom-animation driver can play a
    // track of its own. sampleMusicNL() is a thin wrapper over this.
    static uint8_t sampleTrackNL(const MusicTrack* track, uint32_t positionMs,
                                 uint8_t sensitivity, bool loop);

    // One color from a two-color ramp: genGradient()'s loop body, extracted so
    // a custom-animation layer can sample a gradient without building the
    // whole buffer. genGradient() is written in terms of this, so the two can
    // never disagree.
    static uint32_t gradientPixel(uint32_t c1, uint32_t c2, uint8_t i, uint8_t len,
                                  bool seamless);

    // Blend @p fg over @p bg by @p level (0-255), and the NeoPixel-style hue
    // wheel. Private statics rather than file-local helpers so the custom
    // animation evaluator in led_custom.cpp can reach the same bodies the
    // presets use.
    static uint32_t lerpColor(uint32_t bg, uint32_t fg, uint8_t level);
    static uint32_t wheel(uint8_t pos);

    int16_t computeEffectiveMode() const;
    void    pruneExpired(uint32_t now);
    uint32_t applyBrightness(uint32_t color) const;
    uint16_t msToTicks(uint32_t ms) const;
    /// Refresh ticks between a Speed's steps: 1 for a bare number.
    uint16_t stepTicks(Speed s) const { return s.stepMs ? msToTicks(s.stepMs) : 1; }

    /// A two-color ramp across @p len pixels. Built from gradientPixel().
    static std::vector<uint32_t> genGradient(uint32_t c1, uint32_t c2, uint8_t len,
                                              bool seamless = false);
    static std::vector<uint32_t> genRainbow(uint8_t len);

    // ========================================================================
    // Custom animation state and evaluator (src/led_custom.cpp)
    // ========================================================================

    /// Every scratch word and every value layerPixel() returns is
    /// `(coverage << 24) | 0xRRGGBB`. The top byte of a color is free, so a
    /// bitscroll gap, an unlit meter pixel, a meter's *partial* edge pixel, a
    /// half-faded spark and a drawn frame's transparent index are all one
    /// concept and one array - which is what makes KEY a single mechanism
    /// rather than a special case per generator, and correct at a soft edge.
    static constexpr uint32_t COVER_OPAQUE = 0xFF000000u;

    /// A layer's bindable parameters after its bindings have been applied.
    /// Rebuilt every tick rather than stored, because a driver can move any of
    /// them between one tick and the next.
    struct Resolved {
        uint8_t start;
        uint8_t width;
        uint8_t runLength;
        uint8_t speed;       ///< Pixels per step; the step's timing is not bindable.
        uint8_t densityPct;
        uint8_t opacity;
        uint8_t level;
        uint8_t mix;
    };

    /// What one layer remembers between ticks. No buffer of its own: the
    /// scratch it needs lives in the shared arena below, at @c scratchAt.
    struct LayerState {
        uint16_t shiftAcc     = 0;   ///< Scroll position, mod the sampling ring (motionRing()).
        int16_t  bouncePos    = 0;
        int8_t   bounceDir    = 1;
        uint16_t flashCounter = 0;
        bool     flashLit     = true;
        bool     wasOpen      = false;  ///< Whether its window was open last tick.
        uint16_t frameIdx     = 0;      ///< FRAMES: which frame is showing.
        uint16_t frameCounter = 0;      ///< FRAMES: ticks it has been showing.
        StepPacer step;                 ///< When the motion next moves, see LayerMotion::speed.
        uint16_t scratchAt    = 0;      ///< Index into composeArena.
        uint16_t scratchLen   = 0;      ///< Words reserved there, 0 for none.
        uint16_t contentLen   = 0;      ///< BITSCROLL: unit size up to the last lit pixel.
        uint8_t  maxSpan      = 0;      ///< Widest this layer can ever be, see compose().
        TwinkleState twinkle;           ///< TWINKLE only; the existing struct, unchanged.
        /// GAUGE/METER: the scale resolved onto the 0-255 axis, sorted - the
        /// same representation SpliceRegionState::gaugeStops uses.
        std::vector<std::pair<uint8_t, uint32_t>> stops;
    };

    const Composition* composition     = nullptr;
    uint32_t           composeAnchorMs = 0;
    /// Bumped whenever the composition is replaced, cleared, or dropped by a
    /// preset (dropCompositionNL()). evalDriversNL() releases the mutex to run
    /// a SENSOR reader (user code, which may call back in) and compares this
    /// afterwards, the same way advanceSpliceRegions() does.
    uint32_t           composeGeneration    = 0;
    uint32_t           composeMusicPausedAt = 0;
    bool               composeMusicPaused   = false;

    std::vector<LayerState> layerStates;
    /// One grow-only scratch block for the whole composition: every layer's
    /// pattern unit, bounce master or decoded frame, packed end to end. Sized
    /// in compose() and never resized inside tick(), so re-composing reuses
    /// it when it is already big enough and a Sequencer phase that re-issues
    /// its animation is a memcpy rather than a malloc.
    std::vector<uint32_t> composeArena;
    uint8_t driverValue[MAX_DRIVERS]  = {0};
    bool    driverPrimed[MAX_DRIVERS] = {false};

    void    advanceComposition(uint32_t nowMs);
    void    dropCompositionNL();
    uint32_t composeClockNL(uint32_t nowMs) const;
    bool    evalDriversNL(const Composition& c, uint32_t nowMs);
    void    resolveLayerNL(const Composition& c, const Layer& l, Resolved& p) const;
    uint8_t resolveSpan(const Resolved& p, const LayerState& st) const;
    uint8_t motionRing(const Layer& l, uint8_t span) const;
    void    prepareLayerNL(const Layer& l, LayerState& st, uint8_t span, uint16_t& arenaAt);
    void    buildLayerNL(const Layer& l, LayerState& st);
    void    resetLayerStateNL(const Layer& l, LayerState& st, uint8_t ring);
    void    advanceLayerNL(const Layer& l, LayerState& st, const Resolved& p, uint8_t ring);
    uint32_t layerPixel(const Layer& l, const LayerState& st, const Resolved& p,
                        uint8_t k, uint8_t span) const;
    void    compositeLayerNL(const Layer& l, const LayerState& st, const Resolved& p,
                             uint8_t span, uint8_t alpha);
    void    decodeFrameNL(const Layer& l, LayerState& st);

    static uint8_t  nominalSpan(const Layer& l, uint8_t len);
    static uint8_t  applyCurve(const uint8_t* curve, uint8_t v);
    static uint8_t  layerWindowAlpha(const Layer& l, uint32_t t, uint32_t period);
    static uint8_t  shiftFor(uint8_t speed, uint8_t ring, bool invert);
    static uint8_t  lfoValue(const Driver& d, uint32_t nowMs);
    static uint32_t blendPixel(LayerBlend m, uint32_t dst, uint32_t src, uint8_t a);
    static uint32_t addSat(uint32_t a, uint32_t b);
    static uint32_t screenColor(uint32_t a, uint32_t b);
    static uint32_t mulColor(uint32_t a, uint32_t b);
    static uint8_t  luma(uint32_t c);
};

} // namespace hitlib