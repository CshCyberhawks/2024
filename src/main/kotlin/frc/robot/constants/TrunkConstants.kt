package frc.robot.constants

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

object TrunkConstants {
    // someone rename these
    const val POSITION_GEAR_RATIO = 15
    const val ROTATION_GEAR_RATIO = 100
    const val MAX_PIVOT_HEIGHT_METERS = 0.64446
    const val MIN_PIVOT_HEIGHT_METERS = 0.348701
    const val ELEVATOR_LENGTH_METERS = 0.6133

    const val MOVER_GEAR_RADIUS_METERS = 0.0127
    const val MOVER_GEAR_CIRCUMFERENCE_METERS = 2 * PI * MOVER_GEAR_RADIUS_METERS
    const val ELEVATOR_LENGTH_REVOLUTIONS = TrunkConstants.ELEVATOR_LENGTH_METERS / TrunkConstants.MOVER_GEAR_CIRCUMFERENCE_METERS
    const val ELEVATOR_ANGLE_DEGREES = 28.8309683
    val d2y = sin(Math.toRadians(ELEVATOR_ANGLE_DEGREES))
    val d2x = cos(Math.toRadians(ELEVATOR_ANGLE_DEGREES))

//    var MIN_SAFE_ANGLE: Double = TODO()
//    var TARGET_SAFE_ANGLE: Double = TODO()
//    var MIN_SAFE_DISTANCE: Double = TODO()
//    var ABS_MIN_ANGLE: Double = TODO()
//    var ABS_MAX_ANGLE: Double = TODO()
//    var TOP_M: Double = TODO()
//    var BOTTOM_M: Double = TODO()
//    var SPEAKER_POSITION: Double = TODO()
//    var AMP_POSITION: Double = TODO()
//    var AMP_ANGLE: Double = TODO()
//    var INTAKE_POSITION: Double = TODO()
//    var INTAKE_ANGLE: Double = TODO()
//    var CROSSBAR_BOTTOM: Double = TODO()
//    var CROSSBAR_TOP: Double = TODO()
//    var STOW_POSITION: Double = TODO()
//    var STOW_ANGLE: Double = TODO()

    // TODO: All of these are bullshit values so I could test simulation
    var MIN_SAFE_ANGLE: Double = 0.0
    var TARGET_SAFE_ANGLE: Double = 0.0
    var MIN_SAFE_DISTANCE: Double = 0.0
    var ABS_MIN_ANGLE: Double = 0.0
    var ABS_MAX_ANGLE: Double = 0.0
    var TOP_M: Double = 0.0
    var BOTTOM_M: Double = 0.0
    var SPEAKER_POSITION: Double = 0.8
    var AMP_POSITION: Double = 0.7
    var AMP_ANGLE: Double = 215.0
    var INTAKE_POSITION: Double = 0.3
    var INTAKE_ANGLE: Double = 30.0
    var CROSSBAR_BOTTOM: Double = 0.0
    var CROSSBAR_TOP: Double = 0.0
    var STOW_POSITION: Double = 0.0
    var STOW_ANGLE: Double = 0.0

    var rotationOffset: Double = 0.0

    const val SMART_MOTION_SLOT = 0

    var positionKP = 5e-5
    var positionKI = 1e-6
    var positionKD = 0.0
    var positionIz = 0.0
    var positionFF = 0.000156
    var positionMax = 1.0
    var positionMin = -1.0
    var positionMinRPM = 10.0
    var positionMaxRPM = 5700.0
    var positionMaxAcceleration = 1500.0
    var positionMaxError = 5.0

    var rotationKP = 5e-5
    var rotationKI = 1e-6
    var rotationKD = 0.0
    var rotationIz = 0.0
    var rotationFF = 0.000156
    var rotationMin = -1.0
    var rotationMax = 1.0
    var rotationMinRPM = 10.0
    var rotationMaxRPM = 5700.0
    var rotationMaxAcceleration = 1500.0
    var rotationMaxError = 5.0
}