//package cshcyberhawks.swolib.math

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
     * Deadzones an input so it can not be below a certain value. This is a weighted deadzone.
     *
     * @param input The input you want to deadzone.
     * @param deadzoneValue The minimum value you will allow.
     *
     * @return The deadzoned value.
     */
    fun calculateDeadzone(input: Double, deadzoneValue: Double): Double {
        if (abs(input) > deadzoneValue) {
            if (input > 0) {
                return (input - deadzoneValue) * (1 / (1 - deadzoneValue))
            } else {
                return (input + deadzoneValue) * (1 / (1 - deadzoneValue))
            }
        } else {
            return 0.0
        }
    }

    /**
     * Determines whether n_1 & n_2 are approximately equal (within a certain range). This range is inclusive
     */
    fun appxEqual(n1: Double, n2: Double, range: Double): Boolean {
        if (abs(n1 - n2) <= range) {
            return true;
        }
        return false;
    }

    /**
     * A function to get the current time in milliseconds
     *
     * @return The current time in milliseconds
     */
    fun getCurrentTime(): Double = WPIUtilJNI.now() * 1.0e-6


    fun translation2dWithinRange(current: Translation2d, range: Pair<Translation2d, Translation2d>): Boolean {
        val range_start = range.first
        val range_end = range.second
        if (current.x > range_start.x && current.y > range_start.y && current.x < range_end.x && current.y < range_end.y) {
            return true
        }
        return false
    }

    fun findMatchingTranslation2dRange(
        current: Translation2d,
        ranges: Array<Pair<Translation2d, Translation2d>>,
        default: Pair<Translation2d, Translation2d> = Pair(
            Translation2d(-1.0, -1.0),
            Translation2d(-1.0, -1.0)
        )
    ): Pair<Translation2d, Translation2d> {
        for (range: Pair<Translation2d, Translation2d> in ranges) {
            if (translation2dWithinRange(current, range)) {
                return range
            }
        }
        return default
    }

    /**
     * A function to find the closes vector2 (relative to a reference vector2) in an array of vector2s
     * @param reference The reference vector2
     * @param points The array of vector2s
     * @return The closest vector2
     */
//    fun closestPoint(reference: Vector2, points: Array<Vector2>): Vector2 {
//        var closest = points[0]
//        var closestDistance = reference.distance(points[0])
//        for (point in points) {
//            val distance = reference.distance(point)
//            if (distance < closestDistance) {
//                closest = point
//                closestDistance = distance
//            }
//        }
//        return closest
//    }

}