package frc.robot.constants

import edu.wpi.first.math.util.Units
import kotlin.math.PI

object TargetingConstants {
    // shooter velocity transfer proportion
    var velocityMultiplier = 1.0

    // coords of point we're aiming at relative to center of base of the speaker board (board with the fiducials)
    var endpointX = .2
    var endpointY = 0.0
    var endpointZ = 2.03

    // coords of center of speaker backboard
    var speakerX = 0.0
    var speakerY = Units.inchesToMeters(243.654)

    // height that we shoot from; technically varies a bit but lets just say it doesnt
    var shooterZ = Units.inchesToMeters(24.0)

    var stupidConstant = 0.0

    // 5 maybe
    var constantStupidConstant = 0.0
    // 11.0 maybe

//    var shooterZ = 0.0

    val ROBOT_ANGLE_DEADZONE = 5.0

    // rpm to units per sec conversion factor
    // (rpm) * 2pi (rad/r) * r (r units/rad) * 1/60 (min/s)
    fun rpm2ups(r: Double): Double = PI * r / 30.0
    fun ups2rpm(r: Double): Double = 30.0 / (PI * r)
}