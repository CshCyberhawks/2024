package frc.robot.constants

object TrunkConstants {

    const val ELEVATOR_MOTOR_ID = 20
    const val FOLLOWER_PIVOT_MOTOR_ID = 21
    const val MASTER_PIVOT_MOTOR_ID = 22
    const val CLIMB_LATCH_ID: Int = 0 // please find real value

    const val ELEVATOR2M = -0.03980722784
    const val M2ELEVATOR = 1.0 / ELEVATOR2M

    const val ELEVATOR_ANGLE = 28.8309683
    var SAFE_TRAVEL_ANGLE: Double = 65.0
    var LEGAL_PIVOT_POSITION = 0.1
    var LEGAL_PIVOT_POSITION_TARGET = 0.18

    var MAX_SHOOT_ANGLE = 90.0
    var MIN_SHOOT_ANGLE = 50.0

    var AMP_POSITION: Double = 0.08
    var AMP_ANGLE: Double = 190.0

    var INTAKE_POSITION: Double = 0.02
    var INTAKE_ANGLE: Double = 70.0

    var STOW_POSITION: Double = 0.385
    var STOW_ANGLE: Double = 53.0
    var HIGH_STOW_ANGLE: Double = 67.0

    var TRAP_POSITION: Double = 0.7
    var TRAP_ANGLE: Double = 160.0

    var TOP_BREAK_BEAM_POSITION: Double = .3810000

    var throughboreRotationOffset: Double = 239.4
    var falconRotationOffset: Double = -64.6

    var positionKP = 10.0
    var positionKI = 0.0
    var positionKD = 0.1

    var positionFF = 0.02

    val lowRotationFFkS = 0.03
    val lowRotationFFkG = 0.33
    val lowRotationFFkV = 2.25
    val lowRotationFFkA = 0.02

    val rotationEncoderID = 9

    var lowRotationKP = 0.32
    var lowRotationKI = 0.0
    var lowRotationKD = .0006
//    var rotationTrapConstraints = TrapezoidProfile.Constraints()

    //Degrees/sec^2
    var lowRotationMaxAcceleration = 200.0

    //degrees/sec
    var lowRotationMaxVelo = 150.0

    val highRotationFFkS = 0.03
    val highRotationFFkG = 0.33
    val highRotationFFkV = 2.25
    val highRotationFFkA = 0.02

    var highRotationKP = 0.1
    var highRotationKI = 0.0
    var highRotationKD = 0.0
    //    var rotationTrapConstraints = TrapezoidProfile.Constraints()

    //Degrees/sec^2
    var highRotationMaxAcceleration = 75.0

    //degrees/sec
    var highRotationMaxVelo = 180.0

    var climbRotationKP = 0.1
    var climbRotationKI = 0.0
    var climbRotationKD = 0.01
    //    var rotationTrapConstraints = TrapezoidProfile.Constraints()

    //Degrees/sec^2
    var climbRotationMaxAcceleration = 75.0

    //degrees/sec
    var climbRotationMaxVelo = 180.0

    val ANGLE_DEADZONE = 1.0

    val SAFE_TO_DROP_INTAKE_POSITION = 0.15
    val SAFE_PIVOT_POSITION = 0.378

    val UNDER_STAGE_SHOOTING_OFFSET = 0.0
    val SHOOTING_OFFSET = 0.0

    // TODO: actually tune
    val UNDER_STAGE_SHOOTING_HEIGHT = 0.3
    val SHOOTING_HEIGHT = 0.3

    val SPEAKER_FROM_STAGE_ANGLE = STOW_ANGLE
    val SPEAKER_FROM_STAGE_POSITION = STOW_POSITION

    val ELEVATOR_DEADZONE = .02

    var MIN_ROT_VOLTS = -2.0

    val MAX_ROT_VOLTS = 5.0
}
