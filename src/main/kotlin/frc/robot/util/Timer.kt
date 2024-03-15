package frc.robot.util

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotController

/**
 * A timer class.
 *
 * <p>Note that if the user calls SimHooks.restartTiming(), they should also reset the timer so
 * get() won't return a negative duration.
 */
class Timer {
    /**
     * Return the system clock time in seconds. Return the time from the FPGA hardware clock in
     * seconds since the FPGA started.
     *
     * @return Robot running time in seconds.
     */
    fun getFPGATimestamp(): Double {
        return RobotController.getFPGATime() / 1000000.0;
    }

    /**
     * Return the approximate match time. The FMS does not send an official match time to the robots,
     * but does send an approximate match time. The value will count down the time remaining in the
     * current period (auto or teleop). Warning: This is not an official time (so it cannot be used to
     * dispute ref calls or guarantee that a function will trigger before the match ends) The Practice
     * Match function of the DS approximates the behavior seen on the field.
     *
     * @return Time remaining in current match period (auto or teleop) in seconds
     */
    fun getMatchTime(): Double {
        return DriverStation.getMatchTime();
    }

    /**
     * Pause the thread for a specified time. Pause the execution of the thread for a specified period
     * of time given in seconds. Motors will continue to run at their last assigned values, and
     * sensors will continue to update. Only the task containing the wait will pause until the wait
     * time is expired.
     *
     * @param seconds Length of time to pause
     */
    fun delay(seconds: Double) {
        try {
            Thread.sleep((seconds * 1e3) as Long);
        } catch (ex: InterruptedException) {
            Thread.currentThread().interrupt();
        }
    }

    var m_startTime: Double = getMsClock();
    var m_accumulatedTime: Double = 0.0;
    var isRunning: Boolean = false

    /** Timer constructor. */
    init {
        reset();
    }

    private fun getMsClock(): Double {
        return RobotController.getFPGATime() / 1000.0;
    }

    /**
     * Get the current time from the timer. If the clock is running it is derived from the current
     * system clock the start time stored in the timer class. If the clock is not running, then return
     * the time when it was last stopped.
     *
     * @return Current time value for this timer in seconds
     */
    fun get(): Double {
        if (isRunning) {
            return m_accumulatedTime + (getMsClock() - m_startTime) / 1000.0;
        } else {
            return m_accumulatedTime;
        }
    }

    /**
     * Reset the timer by setting the time to 0.
     *
     * <p>Make the timer startTime the current time so new requests will be relative now.
     */
    fun reset() {
        m_accumulatedTime = 0.0;
        m_startTime = getMsClock();
    }

    /**
     * Start the timer running. Just set the running flag to true indicating that all time requests
     * should be relative to the system clock. Note that this method is a no-op if the timer is
     * already running.
     */
    fun start() {
        if (!isRunning) {
            m_startTime = getMsClock();
            isRunning = true;
        }
    }

    /**
     * Restart the timer by stopping the timer, if it is not already stopped, resetting the
     * accumulated time, then starting the timer again. If you want an event to periodically reoccur
     * at some time interval from the start time, consider using advanceIfElapsed() instead.
     */
    fun restart() {
        if (isRunning) {
            stop();
        }
        reset();
        start();
    }

    /**
     * Stop the timer. This computes the time as of now and clears the running flag, causing all
     * subsequent time requests to be read from the accumulated time rather than looking at the system
     * clock.
     */
    fun stop() {
        m_accumulatedTime = get();
        isRunning = false;
    }

    /**
     * Check if the period specified has passed.
     *
     * @param seconds The period to check.
     * @return Whether the period has passed.
     */
    fun hasElapsed(seconds: Double): Boolean {
        return get() >= seconds;
    }

    /**
     * Check if the period specified has passed and if it has, advance the start time by that period.
     * This is useful to decide if it's time to do periodic work without drifting later by the time it
     * took to get around to checking.
     *
     * @param seconds The period to check.
     * @return Whether the period has passed.
     */
    fun advanceIfElapsed(seconds: Double): Boolean {
        if (get() >= seconds) {
            // Advance the start time by the period.
            // Don't set it to the current time... we want to avoid drift.
            m_startTime += seconds * 1000;
            return true;
        } else {
            return false;
        }
    }
}
