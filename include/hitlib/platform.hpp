#pragma once
#include <cstdint>

/**
 * @file platform.hpp
 * @brief The handful of OS and hardware calls HitLib needs, kept apart from
 *        any one SDK so the library builds under PROS and VEXcode alike.
 *
 * Nothing here includes a PROS or VEX header: each backend lives in its own
 * source file (`platform_pros.cpp`, `platform_vexcode.cpp`) and only the one
 * matching the project compiles to anything.  The platform is detected from
 * the headers on the include path; to force it, define one of these before
 * building the library:
 *
 * | Macro                        | Platform                               |
 * |------------------------------|----------------------------------------|
 * | `HITLIB_PLATFORM_PROS=1`     | PROS kernel 4.1 or later               |
 * | `HITLIB_PLATFORM_VEXCODE=1`  | VEXcode V5, VEXcode Pro V5, VS Code VEX extension |
 * | `HITLIB_PLATFORM_HOST=1`     | A desktop build, for tests only. Never auto-detected. |
 *
 * User code never needs to touch this header.
 */

#if !defined(HITLIB_PLATFORM_PROS) && !defined(HITLIB_PLATFORM_VEXCODE) && \
    !defined(HITLIB_PLATFORM_HOST)
#  if defined(__has_include)
#    if __has_include("pros/rtos.hpp")
#      define HITLIB_PLATFORM_PROS 1
#    elif __has_include("v5_vcs.h")
#      define HITLIB_PLATFORM_VEXCODE 1
#    endif
#  endif
#endif

#ifndef HITLIB_PLATFORM_PROS
#  define HITLIB_PLATFORM_PROS 0
#endif
#ifndef HITLIB_PLATFORM_VEXCODE
#  define HITLIB_PLATFORM_VEXCODE 0
#endif
// Deliberately never auto-detected: a desktop compiler has no header that
// identifies it as "the test platform", and silently picking it would turn a
// misconfigured robot build into one that compiles and drives nothing.
#ifndef HITLIB_PLATFORM_HOST
#  define HITLIB_PLATFORM_HOST 0
#endif

#if (HITLIB_PLATFORM_PROS + HITLIB_PLATFORM_VEXCODE + HITLIB_PLATFORM_HOST) > 1
#  error "HitLib: more than one HITLIB_PLATFORM_* macro is set; pick one."
#elif (HITLIB_PLATFORM_PROS + HITLIB_PLATFORM_VEXCODE + HITLIB_PLATFORM_HOST) == 0
#  error "HitLib: couldn't find PROS (pros/rtos.hpp) or the VEX SDK (v5_vcs.h) on the include path. Define HITLIB_PLATFORM_PROS=1 or HITLIB_PLATFORM_VEXCODE=1."
#endif

namespace hitlib {
namespace platform {

/** @brief Milliseconds since the program started. */
uint32_t millis();

/** @brief Sleep the calling task for @p ms milliseconds. */
void delay(uint32_t ms);

/**
 * @brief Sleep until @p prevMs + @p ms, then advance @p prevMs by @p ms.
 *
 * Keeps a loop on a fixed period no matter how long its body took.  A body
 * that overran the period returns straight away (after yielding) rather than
 * sleeping.
 */
void delayUntil(uint32_t& prevMs, uint32_t ms);

/**
 * @brief Start a background task that runs @p entry(@p arg).
 *
 * The task is expected never to return.
 */
void startTask(void (*entry)(void*), void* arg, const char* name);

/** @brief Non-recursive mutex, blocking take. */
class Mutex {
public:
    Mutex();
    ~Mutex();

    Mutex(const Mutex&)            = delete;
    Mutex& operator=(const Mutex&) = delete;

    void take();
    void give();

private:
    void* handle;
};

/**
 * @brief A WS2812B strip on one ADI port, either the brain's own or an
 *        ADI expander's.
 */
class AdiLed {
public:
    /**
     * @param smartPort  Smart Port of the ADI expander (1-21), or 0 for the
     *                   brain's built-in ADI ports.
     * @param adiPort    ADI port (1-8).
     * @param length     Number of LEDs.
     */
    AdiLed(uint8_t smartPort, uint8_t adiPort, uint8_t length);
    ~AdiLed();

    AdiLed(const AdiLed&)            = delete;
    AdiLed& operator=(const AdiLed&) = delete;

    /** @brief Send @p count 0xRRGGBB colors to the strip, starting at LED 0. */
    void write(const uint32_t* pixels, uint8_t count);

private:
    void* impl;
};

#if HITLIB_PLATFORM_HOST
/**
 * @brief Controls for the desktop test backend. Present only when
 *        @c HITLIB_PLATFORM_HOST is set, so nothing on a robot can reach it.
 *
 * The point of this backend is to run the *real* animation code on a desktop
 * and compare its frames against the Python mirror in `hitlib_sim`, which is
 * what keeps Pattern Studio's preview honest.  Two things make that possible:
 * a clock the test drives rather than one that runs on its own, and a strip
 * that records what was written to it instead of lighting anything.
 *
 * Time does not pass by itself here.  delay() does nothing by default, because
 * the 12 ms pacing delay inside LedStrand::flushBuffer() exists to give the ADI
 * driver breathing room and has no meaning off-hardware; letting it advance
 * the clock would put every frame 12 ms ahead of the simulator's.  A test
 * advances the clock itself, by exactly one refresh interval per tick, which
 * is the same thing `hitlib_sim.Strand.tick()` does to its `now_ms`.  A test
 * about the robot's timing can make the pacing cost time with onDelay().
 */
namespace host {

/** @brief Set the virtual clock millis() reads. */
void setMillis(uint32_t ms);

/** @brief Move the virtual clock forward by @p ms. */
void advanceMillis(uint32_t ms);

/** @brief Reset the clock to 0, forget any captured frame and started task,
 *         zero the counters and remove the hooks. */
void reset();

/**
 * @brief The most recent frame handed to AdiLed::write(), or @c nullptr
 *        before the first one.
 *
 * One frame for the whole process, not one per strip: the parity harness
 * drives a single strand, and a per-port table would be state the tests do
 * not use.  Valid until the next write().  A strand skips a frame identical
 * to the last one it sent, so with several strands this may be another
 * strand's; tell them apart by lastFrameCount() from an onWrite() hook.
 */
const uint32_t* lastFrame();

/** @brief How many pixels lastFrame() holds. */
uint8_t lastFrameCount();

/**
 * @brief The most recent frame written to one port, or @c nullptr before the
 *        first.
 *
 * What a chained strand's harness reads: a chain writes each of its strips
 * separately, so lastFrame() only ever holds the last strip's part.  Valid
 * until the next write to that port.
 *
 * @param smartPort  The expander's Smart Port, or 0 for the brain's own ports.
 * @param adiPort    The ADI port (1-8).
 */
const uint32_t* lastFrameOn(uint8_t smartPort, uint8_t adiPort);

/** @brief How many pixels lastFrameOn() holds for that port. */
uint8_t lastFrameCountOn(uint8_t smartPort, uint8_t adiPort);

/** @brief How many times write() has been called, so a test can tell a
 *         repeated frame from no frame at all. */
uint32_t writeCount();

/**
 * @brief Call @p hook from inside every AdiLed::write(), at the point where
 *        the real backends hand the frame to the ADI.  @c nullptr removes it.
 *
 * Lets a test see what else is true while a frame is on the wire: which locks
 * are held (heldMutexes()), and whether a call from user code landing right
 * then would have had to wait for one (contendedTakes()).
 */
void onWrite(void (*hook)());

/**
 * @brief Call @p hook from inside every delay(), with the milliseconds asked
 *        for.  @c nullptr removes it.
 *
 * delay() still does not move the clock by itself.  A test that wants the
 * ADI pacing to cost time, as it does on the robot, advances the clock from
 * here.
 */
void onDelay(void (*hook)(uint32_t ms));

/** @brief How many Mutexes are taken right now. */
uint32_t heldMutexes();

/**
 * @brief How many times take() was called on a Mutex that was already held.
 *
 * On the robot each of those is a task blocked until the holder lets go, or
 * a deadlock if the holder is the same task.  Nothing blocks here, so the
 * count is all that is left of it.
 */
uint32_t contendedTakes();

/** @brief How many tasks startTask() has been asked for.  None of them runs
 *         by itself. */
uint32_t taskCount();

/**
 * @brief Run the @p index th task startTask() was given until it has gone
 *        round its loop @p cycles times, i.e. called delayUntil() that often,
 *        then stop it there.
 *
 * The task starts again from its entry point on every call, so its locals,
 * a loop's @c prevTime among them, do not carry over from one call to the
 * next.
 */
void runTask(uint32_t index, uint32_t cycles);

} // namespace host
#endif // HITLIB_PLATFORM_HOST

} // namespace platform
} // namespace hitlib
