//package frc.robot.constants.yagsl_configs
//
//import edu.wpi.first.math.geometry.Rotation3d
//import frc.robot.util.DualPigeon2Swerve
//import swervelib.encoders.CANCoderSwerve
//import swervelib.motors.SparkMaxSwerve
//import swervelib.parser.PIDFConfig
//import swervelib.parser.json.MotorConfigDouble
//
//object SlippyConfig {
//    const val FRONT_LEFT_DRIVE_ID: Int = 6
//    const val FRONT_RIGHT_DRIVE_ID: Int = 8
//    const val BACK_RIGHT_DRIVE_ID: Int = 7
//    const val BACK_LEFT_DRIVE_ID: Int = 5
//
//    const val FRONT_LEFT_TWIST_ID: Int = 10
//    const val FRONT_RIGHT_TWIST_ID: Int = 12
//    const val BACK_RIGHT_TWIST_ID: Int = 11
//    const val BACK_LEFT_TWIST_ID: Int = 9
//
//    const val FRONT_LEFT_ENCODER_ID: Int = 2
//    const val FRONT_RIGHT_ENCODER_ID: Int = 4
//    const val BACK_RIGHT_ENCODER_ID: Int = 3
//    const val BACK_LEFT_ENCODER_ID: Int = 1
//
//    const val NORMAL_PIGEON_ID = 30
//    const val REVERSE_PIGEON_ID = 31
//
//    val imu = DualPigeon2Swerve(
//        NORMAL_PIGEON_ID,
//        REVERSE_PIGEON_ID,
//        "",
//        Rotation3d(),
//    )
//
//    const val DRIVE_MOTOR_RAMP_RATE = 0.25
//    const val TWIST_MOTOR_RAMP_RATE = 0.25
//
//    const val ANGLE_JOYSTICK_RADIUS_DEADBAND = 0.5
//
//    val CONVERSION_FACTORS = MotorConfigDouble(0.045777493, 16.8)
//
//    val DRIVE_PID = PIDFConfig(0.00023, 0.0000002, 1.0)
//    val TWIST_PID = PIDFConfig(0.004, 1.5)
//    val HEADING_PID = PIDFConfig(.4, .01)
//
//    val FRONT_LEFT_DRIVE_MOTOR = SparkMaxSwerve(FRONT_LEFT_DRIVE_ID, true)
//    val FRONT_LEFT_TWIST_MOTOR = SparkMaxSwerve(FRONT_LEFT_TWIST_ID, false)
//    val FRONT_LEFT_ENCODER = CANCoderSwerve(FRONT_LEFT_ENCODER_ID)
//
//    val FRONT_RIGHT_DRIVE_MOTOR = SparkMaxSwerve(FRONT_RIGHT_DRIVE_ID, true)
//    val FRONT_RIGHT_TWIST_MOTOR = SparkMaxSwerve(FRONT_RIGHT_TWIST_ID, false)
//    val FRONT_RIGHT_ENCODER = CANCoderSwerve(FRONT_RIGHT_ENCODER_ID)
//
//    val BACK_RIGHT_DRIVE_MOTOR = SparkMaxSwerve(BACK_RIGHT_DRIVE_ID, true)
//    val BACK_RIGHT_TWIST_MOTOR = SparkMaxSwerve(BACK_RIGHT_TWIST_ID, false)
//    val BACK_RIGHT_ENCODER = CANCoderSwerve(BACK_RIGHT_ENCODER_ID)
//
//    val BACK_LEFT_DRIVE_MOTOR = SparkMaxSwerve(BACK_LEFT_DRIVE_ID, true)
//    val BACK_LEFT_TWIST_MOTOR = SparkMaxSwerve(BACK_LEFT_TWIST_ID, false)
//    val BACK_LEFT_ENCODER = CANCoderSwerve(BACK_LEFT_ENCODER_ID)
//
//    val slippy = YAGSLConfig(
//        imu,
//        false,
//        .5,
//        4.7,
//        12.375,
//        12.375,
//        CONVERSION_FACTORS,
//        1.19,
//        12.0,
//        40,
//        20,
//        .25,
//        .25,
//        FRONT_LEFT_DRIVE_MOTOR,
//        false,
//        FRONT_LEFT_TWIST_MOTOR,
//        true,
//        FRONT_LEFT_ENCODER,
//        -114.609,
//        FRONT_RIGHT_DRIVE_MOTOR,
//        false,
//        FRONT_RIGHT_TWIST_MOTOR,
//        true,
//        FRONT_RIGHT_ENCODER,
//        -50.977,
//        BACK_RIGHT_DRIVE_MOTOR,
//        true,
//        BACK_RIGHT_TWIST_MOTOR,
//        true,
//        BACK_RIGHT_ENCODER,
//        -18.281,
//        BACK_LEFT_DRIVE_MOTOR,
//        true,
//        BACK_LEFT_TWIST_MOTOR,
//        true,
//        BACK_LEFT_ENCODER,
//        6.504,
//        DRIVE_PID,
//        TWIST_PID,
//        HEADING_PID,
//    )
//}