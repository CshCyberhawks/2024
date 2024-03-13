package frc.robot.constants

object DriveConstants {
    //Code (virtual) constants
    const val TELEOP_DEADZONE_X: Double = .15
    const val TELEOP_DEADZONE_Y: Double = .15
    const val TELEOP_DEADZONE_TWIST_ONE_JOY: Double = .15
    const val TELEOP_DEADZONE_TWIST_TWO_JOY: Double = .3

    const val TWO_JOYSTICKS: Boolean = true

    const val MAX_SPEED: Double = 4.7

    //degrees
    const val TELEOP_TRANSLATION_AUTOTWIST_DEADZONE = 2.0

    const val MAX_ANGLE_SPEED: Double = 4.0
    const val MAX_ANGLE_ACCEL: Double = 3.0

    //Physical (hardware & physics0 constants

    // TODO: make these ids not bad (1-4 clockwise)
    const val FRONT_LEFT_DRIVE_ID: Int = 6
    const val FRONT_RIGHT_DRIVE_ID: Int = 8
    const val BACK_RIGHT_DRIVE_ID: Int = 7
    const val BACK_LEFT_DRIVE_ID: Int = 5

    const val FRONT_LEFT_TWIST_ID: Int = 10
    const val FRONT_RIGHT_TWIST_ID: Int = 12
    const val BACK_RIGHT_TWIST_ID: Int = 11
    const val BACK_LEFT_TWIST_ID: Int = 9

    const val FRONT_LEFT_ENCODER_ID: Int = 2
    const val FRONT_RIGHT_ENCODER_ID: Int = 4
    const val BACK_RIGHT_ENCODER_ID: Int = 3
    const val BACK_LEFT_ENCODER_ID: Int = 1

    const val NORMAL_PIGEON_ID: Int = 30
    const val REVERSE_PIGEON_ID: Int = 31

    // these are random
    // module positions (center of robot to center of wheel)
    const val MODULE_X_OFFSET: Double = 12.375
    const val MODULE_Y_OFFSET: Double = 12.375
    const val ROBOT_RADIUS: Double = 17.500

    const val CONVERSION_FACTOR_ANGLE: Double = 0.045777493
    const val CONVERSION_FACTOR_DRIVE: Double = 16.8

//    val DRIVE_PID: PIDFConfig = PIDFConfig(0.00023, 0.0000002, 1.0, 0.0)
//    val TWIST_PID: PIDFConfig = PIDFConfig(0.004, 0.0, 1.5, 0.0)

    // idk if these are right
    const val WHEEL_GRIP_COEFFICIENT_OF_FRICTION: Double = 1.19
    const val OPTIMAL_VOLTAGE: Double = 12.0
    const val DRIVE_MOTOR_CURRENT_LIMIT: Int = 40
    const val TWIST_MOTOR_CURRENT_LIMIT: Int = 20
    const val DRIVE_MOTOR_RAMP_RATE: Double = 0.25
    const val TWIST_MOTOR_RAMP_RATE: Double = 0.25

    const val ANGLE_JOYSTICK_RADIUS_DEADBAND: Double = 0.5

    const val FRONT_LEFT_ANGLE_OFFSET: Double = 63.193359375
    const val FRONT_RIGHT_ANGLE_OFFSET: Double = 86.63661193847656
    const val BACK_RIGHT_ANGLE_OFFSET: Double = 234.6
    const val BACK_LEFT_ANGLE_OFFSET: Double = 45.0
}
