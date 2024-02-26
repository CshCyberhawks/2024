//package frc.robot.constants.yagsl_configs
//
//import swervelib.encoders.CANCoderSwerve
//import swervelib.imu.Pigeon2Swerve
//import swervelib.motors.SparkMaxSwerve
//import swervelib.motors.TalonFXSwerve
//import swervelib.parser.PIDFConfig
//import swervelib.parser.json.MotorConfigDouble
//
//object GoodNewsGooseConfig {
//    val imu = Pigeon2Swerve(
//        30,
//        "",
//    )
//    val HEADING_PID = PIDFConfig(0.0, 0.0)
//
//    val CONVERSION_FACTORS = MotorConfigDouble(16.8, 0.045777493)
//
//    val DRIVE_PID = PIDFConfig(0.00023, 0.0000002, 1.0)
//    val TWIST_PID = PIDFConfig(0.004, 1.5)
//
//    val FRONT_LEFT_DRIVE_MOTOR = TalonFXSwerve(1, true)
//    val FRONT_LEFT_TWIST_MOTOR = SparkMaxSwerve(2, false)
//    val FRONT_LEFT_ENCODER = CANCoderSwerve(3)
//
//    val FRONT_RIGHT_DRIVE_MOTOR = TalonFXSwerve(4, true)
//    val FRONT_RIGHT_TWIST_MOTOR = SparkMaxSwerve(5, false)
//    val FRONT_RIGHT_ENCODER = CANCoderSwerve(6)
//
//    val BACK_RIGHT_DRIVE_MOTOR = SparkMaxSwerve(7, true)
//    val BACK_RIGHT_TWIST_MOTOR = SparkMaxSwerve(8, false)
//    val BACK_RIGHT_ENCODER = CANCoderSwerve(9)
//
//    val BACK_LEFT_DRIVE_MOTOR = SparkMaxSwerve(10, true)
//    val BACK_LEFT_TWIST_MOTOR = SparkMaxSwerve(11, false)
//    val BACK_LEFT_ENCODER = CANCoderSwerve(12)
//
//    val goodNewsGoose = YAGSLConfig(
//        imu,
//        false,
//        .5,
//        4.20,
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
//        true,
//        FRONT_LEFT_TWIST_MOTOR,
//        true,
//        FRONT_LEFT_ENCODER,
//        296.630859375,
//        FRONT_RIGHT_DRIVE_MOTOR,
//        false,
//        FRONT_RIGHT_TWIST_MOTOR,
//        true,
//        FRONT_RIGHT_ENCODER,
//        146.25,
//        BACK_RIGHT_DRIVE_MOTOR,
//        true,
//        BACK_RIGHT_TWIST_MOTOR,
//        true,
//        BACK_RIGHT_ENCODER,
//        199.86328125,
//        BACK_LEFT_DRIVE_MOTOR,
//        true,
//        BACK_LEFT_TWIST_MOTOR,
//        true,
//        BACK_LEFT_ENCODER,
//        142.91015625,
//        DRIVE_PID,
//        TWIST_PID,
//        HEADING_PID,
//    )
//}
//
////package frc.robot.constants.yagsl_configs
////
////import swervelib.encoders.CANCoderSwerve
////import swervelib.imu.Pigeon2Swerve
////import swervelib.motors.SparkMaxSwerve
////import swervelib.motors.TalonFXSwerve
////import swervelib.parser.PIDFConfig
////import swervelib.parser.json.MotorConfigDouble
////
////object GoodNewsGooseConfig {
////    val imu = Pigeon2Swerve(
////        30,
////        "",
////    )
////    val HEADING_PID = PIDFConfig(0.0, 0.0)
////
////    val CONVERSION_FACTORS = MotorConfigDouble(16.8, 0.05215454470665408)
////
////    val DRIVE_PID = PIDFConfig(0.00023, 0.0000002, 1.0)
////    val TWIST_PID = PIDFConfig(0.004, 1.5)
////
////    val FRONT_LEFT_DRIVE_MOTOR = TalonFXSwerve(5, true)
////    val FRONT_LEFT_TWIST_MOTOR = SparkMaxSwerve(9, false)
////    val FRONT_LEFT_ENCODER = CANCoderSwerve(13, "")
////
////    val FRONT_RIGHT_DRIVE_MOTOR = TalonFXSwerve(2, true)
////    val FRONT_RIGHT_TWIST_MOTOR = SparkMaxSwerve(6, false)
////    val FRONT_RIGHT_ENCODER = CANCoderSwerve(10, "")
////
////    val BACK_RIGHT_DRIVE_MOTOR = TalonFXSwerve(3, true)
////    val BACK_RIGHT_TWIST_MOTOR = SparkMaxSwerve(7, false)
////    val BACK_RIGHT_ENCODER = CANCoderSwerve(11, "")
////
////    val BACK_LEFT_DRIVE_MOTOR = TalonFXSwerve(4, true)
////    val BACK_LEFT_TWIST_MOTOR = SparkMaxSwerve(8, false)
////    val BACK_LEFT_ENCODER = CANCoderSwerve(12, "")
////
////    val goodNewsGoose = YAGSLConfig(
////        imu = imu,
////        invertedIMU = false,
////        angleJoystickDeadband = .5,
////        maxSpeedMPS = 4.50,
////        moduleXOffset = 12.375,
////        moduleYOffset = 12.375,
////        conversionFactors = CONVERSION_FACTORS,
////        wheelFriction = 1.19,
////        optimalVoltage = 12.0,
////        driveCurrentLimit = 40,
////        twistCurrentLimit = 20,
////        driveRampRate = .25,
////        twistRampRate = .25,
////        flDrive = FRONT_LEFT_DRIVE_MOTOR,
////        flDriveInverted = true,
////        flTwist = FRONT_LEFT_TWIST_MOTOR,
////        flTwistInverted = true,
////        flEncoder = FRONT_LEFT_ENCODER,
////        flEncoderInverted = false,
////        flOffset = 296.630859375,
////        frDrive = FRONT_RIGHT_DRIVE_MOTOR,
////        frDriveInverted = false,
////        frTwist = FRONT_RIGHT_TWIST_MOTOR,
////        frTwistInverted = true,
////        frEncoder = FRONT_RIGHT_ENCODER,
////        frEncoderInverted = false,
////        frOffset = 146.25,
////        brDrive = BACK_RIGHT_DRIVE_MOTOR,
////        brDriveInverted = true,
////        brTwist = BACK_RIGHT_TWIST_MOTOR,
////        brTwistInverted = true,
////        brEncoder = BACK_RIGHT_ENCODER,
////        brEncoderInverted = false,
////        brOffset = 199.86328125,
////        blDrive = BACK_LEFT_DRIVE_MOTOR,
////        blDriveInverted = true,
////        blTwist = BACK_LEFT_TWIST_MOTOR,
////        blTwistInverted = true,
////        blEncoder = BACK_LEFT_ENCODER,
////        blEncoderInverted = false,
////        blOffset = 142.91015625,
////        drivePID = DRIVE_PID,
////        twistPID = TWIST_PID,
////        headingPID = HEADING_PID,
////    )
////}
//>>>>>>> 9fdb31cd4376071a054bbc9671aa7105e14da6d6
