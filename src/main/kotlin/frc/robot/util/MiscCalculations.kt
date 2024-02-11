package frc.robot.util

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.util.WPIUtilJNI
import kotlin.math.abs

/** Miscellaneous calculations. */
object MiscCalculations {
    /**
     * Converts Gs to meters per second.
     *
     * @param g The number of Gs you are traveling.
     *
     * @return The speed in meters per second.
     */
    fun gToMetersPerSecond(g: Double): Double = g * 9.8066

    /**
     * Deadzones an input so it can not be below a certain value.
     *
     * @param input The input you want to deadzone.
     * @param deadzoneValue The minimum value you will allow.
     *
     * @return The deadzoned value.
     */
    fun calculateDeadzone(input: Double, deadzoneValue: Double): Double = if (abs(input) > deadzoneValue) {
        (input + if (input > 0.0) {
            -deadzoneValue
        } else {
            deadzoneValue
        }) / (1.0 - deadzoneValue)
    } else 0.0

    /**
     * A function to get the current time in seconds
     *
     * @return The current time in seconds
     */
    fun getCurrentTimeSeconds(): Double = WPIUtilJNI.now() * 1.0e-6

    fun closestPoint(reference: Translation2d, points: Array<Translation2d>): Translation2d {
        var closest = points[0]
        var closestDistance = reference.getDistance(points[0])
        for (point in points) {
            val distance = reference.getDistance(point)
            if (distance < closestDistance) {
                closest = point
                closestDistance = distance
            }
        }
        return closest
    }
}
