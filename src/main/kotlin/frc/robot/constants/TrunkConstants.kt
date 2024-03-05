package frc.robot.constants

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

object TrunkConstants {
    // someone rename these
    const val MAX_PIVOT_HEIGHT_M = 0.8
    const val MIN_PIVOT_HEIGHT_M = 0.0
    const val THING_LENGTH_M = 0.6133
    const val POSITION_CONVERSION_FACTOR = -0.03980722784
    const val ELEVATOR2M = -0.03980722784
    const val M2ELEVATOR = 1.0 / ELEVATOR2M
//    0.769531 - 10.342285 = -9.572754
//    0.405762 - 9.967285 = -9.561523
//    0.386719 - 9.974365 = -9.587646
//    0.397217 - 9.974365 = -9.577148
//    0.406494 - 9.969727 = -9.563233
//    0.398438 - 9.962891 = -9.564453
//    -9.572754 + -9.561523 + -9.587646 + -9.577148 + -9.563233 + -9.564453 = -57.426757
//    -57.426757 / 6 = -9.571126167
//    .3810000 / -9.571126167  = -0.03980722784


    //        0.0609217993
//    const val MOVER_GEAR_RADIUS_M = 0.0127
//    const val MOVER_GEAR_CIRCUMFERENCE_M = MOVER_GEAR_RADIUS_M * 2.0 * PI
    const val ELEVATOR_ANGLE = 28.8309683
    val d2y = sin(ELEVATOR_ANGLE * PI / 180.0)
    val d2x = cos(ELEVATOR_ANGLE * PI / 180.0)

    var SAFE_TRAVEL_ANGLE: Double = 60.0

    var MAX_ANGLE: Double = 170.0

    var SPEAKER_POSITION: Double = 0.381

    var AMP_POSITION: Double = 0.4
    var AMP_ANGLE: Double = 140.0

    var INTAKE_POSITION: Double = 0.01
    var INTAKE_ANGLE: Double = 70.0

    var CROSSBAR_BOTTOM: Double = 0.05
    var CROSSBAR_TOP: Double = .5

    var STOW_POSITION: Double = 0.381
    var STOW_ANGLE: Double = 65.0

    var TRAP_POSITION: Double = 0.7
    var TRAP_ANGLE: Double = 160.0

    var TOP_BREAK_BEAM_POSITION: Double = .3810000
    var BOTTOM_BREAK_BEAM_POSITION: Double = 0.0

    val MIN_ANGLE_BELOW_CROSSBAR = -10.0
    val MIN_ANGLE_ABOVE_CROSSBAR = -20.0

    var rotationOffset: Double = 282.0

    var positionKP = 22.0
    var positionKI = 0.0
    var positionKD = 0.1
    var positionIz = 0.0

    var positionFF = 0.02

    val rotationFFkS = 0.03
    val rotationFFkG = 0.48
    val rotationFFkV = 2.44
    val rotationFFkA = 0.02

    val rotationEncoderID = 9

    var rotationKP = 7.0
    var rotationKI = 0.0001
    var rotationKD = 1.25

    //Degrees/sec^2
    var rotationMaxAcceleration = 40.0

    //degrees/sec
    var rotationMaxVelo = 30.0
    var rotationMaxError = 5.0

    val ANGLE_DEADZONE = 4.0

    val SAFE_TO_DROP_INTAKE_POSITION = 0.23


    val UNDER_STAGE_SHOOTING_OFFSET = 0.0
    val SHOOTING_OFFSET = 0.0

    val UNDER_STAGE_SHOOTING_HEIGHT = 0.0
    val SHOOTING_HEIGHT = 0.0

    val SPEAKER_FROM_STAGE_ANGLE = STOW_ANGLE
    val SPEAKER_FROM_STAGE_POSITION = STOW_POSITION
}