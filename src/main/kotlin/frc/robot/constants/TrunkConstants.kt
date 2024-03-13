package frc.robot.constants

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

object TrunkConstants {
    // someone rename these
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

    const val ELEVATOR_ANGLE = 28.8309683
    val d2y = sin(ELEVATOR_ANGLE * PI / 180.0)
    val d2x = cos(ELEVATOR_ANGLE * PI / 180.0)

    var TRAVEL_ANGLE: Double = 63.0

    var MAX_ANGLE: Double = 190.0
    var MAX_SHOOT_ANGLE = 90.0
    var MIN_SHOOT_ANGLE = 50.0

    var SPEAKER_POSITION: Double = 0.381

    var AMP_POSITION: Double = 0.3281
    var AMP_ANGLE: Double = 185.0
//    var AMP_ANGLE: Double = 180.0


    var INTAKE_POSITION: Double = 0.01

    var CROSSBAR_BOTTOM: Double = 0.05
    var CROSSBAR_TOP: Double = .5

    var STOW_POSITION: Double = 0.381
    var STOW_ANGLE: Double = 68.0

    var TRAP_POSITION: Double = 0.7
    var TRAP_ANGLE: Double = 160.0

    var TOP_BREAK_BEAM_POSITION: Double = .3810000
    var BOTTOM_BREAK_BEAM_POSITION: Double = 0.0

    const val ROTATION_OFFSET: Double = -35.00

    var positionKP = 22.0
    var positionKI = 0.0
    var positionKD = 0.1

    var positionFF = 0.02

    val rotationFFkS = 0.03
    val rotationFFkG = 0.48
    val rotationFFkV = 2.44
    val rotationFFkA = 0.02

    val rotationEncoderID = 9

//    var rotationKP = 7.0
        var rotationKP = 5.0
//        var rotationKI = 0.001
    var rotationKI = 0.0
    var rotationKD = .125
//    var rotationTrapConstraints = TrapezoidProfile.Constraints()

    //Degrees/sec^2
    var rotationMaxAcceleration = 40.0

    //degrees/sec
    var rotationMaxVelo = 30.0
    var rotationMaxError = 5.0

    const val ANGLE_DEADZONE = 2.5
    const val POSITION_DEADZONE = .01

    val SAFE_TO_DROP_INTAKE_POSITION = 0.23

    val UNDER_STAGE_SHOOTING_OFFSET = 0.0
    val SHOOTING_OFFSET = 0.0
// TODO: actually tune
    val UNDER_STAGE_SHOOTING_HEIGHT = 0.3
    val SHOOTING_HEIGHT = 0.3

    val SPEAKER_FROM_STAGE_ANGLE = STOW_ANGLE
    val SPEAKER_FROM_STAGE_POSITION = STOW_POSITION
}