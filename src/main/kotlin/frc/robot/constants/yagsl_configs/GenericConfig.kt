package frc.robot.constants.yagsl_configs

import swervelib.encoders.SwerveAbsoluteEncoder
import swervelib.imu.SwerveIMU
import swervelib.motors.SwerveMotor
import swervelib.parser.*
import swervelib.parser.json.MotorConfigDouble

class YAGSLConfig(
    val imu: SwerveIMU,
    val invertedIMU: Boolean,
    val angleJoystickDeadband: Double,
    val maxSpeedMPS: Double,
    val moduleXOffset: Double,
    val moduleYOffset: Double,

    val conversionFactors: MotorConfigDouble,
    val wheelFriction: Double,
    val optimalVoltage: Double,
    val driveCurrentLimit: Int,
    val twistCurrentLimit: Int,
    val driveRampRate: Double,
    val twistRampRate: Double,

    val flDrive: SwerveMotor,
    val flDriveInverted: Boolean,
    val flTwist: SwerveMotor,
    val flTwistInverted: Boolean,
    val flEncoder: SwerveAbsoluteEncoder,
    val flEncoderInverted: Boolean,
    val flOffset: Double,

    val frDrive: SwerveMotor,
    val frDriveInverted: Boolean,
    val frTwist: SwerveMotor,
    val frTwistInverted: Boolean,
    val frEncoder: SwerveAbsoluteEncoder,
    val frEncoderInverted: Boolean,
    val frOffset: Double,

    val brDrive: SwerveMotor,
    val brDriveInverted: Boolean,
    val brTwist: SwerveMotor,
    val brTwistInverted: Boolean,
    val brEncoder: SwerveAbsoluteEncoder,
    val brEncoderInverted: Boolean,
    val brOffset: Double,

    val blDrive: SwerveMotor,
    val blDriveInverted: Boolean,
    val blTwist: SwerveMotor,
    val blTwistInverted: Boolean,
    val blEncoder: SwerveAbsoluteEncoder,
    val blEncoderInverted: Boolean,
    val blOffset: Double,

    val drivePID: PIDFConfig,
    val twistPID: PIDFConfig,
    val headingPID: PIDFConfig,

    ) {

    val moduleCharacteristics = SwerveModulePhysicalCharacteristics(
        conversionFactors,
        wheelFriction,
        optimalVoltage,
        driveCurrentLimit,
        twistCurrentLimit,
        driveRampRate,
        twistRampRate,
    )

    val flConfig = SwerveModuleConfiguration(
        flDrive,
        flTwist,
        conversionFactors,
        flEncoder,
        flOffset,
        -moduleXOffset,
        moduleYOffset,
        twistPID,
        drivePID,
        moduleCharacteristics,
        flEncoderInverted,
        flDriveInverted,
        flTwistInverted,
        "Front Left Swerve Module"
    )
    val frConfig = SwerveModuleConfiguration(
        frDrive,
        frTwist,
        conversionFactors,
        frEncoder,
        frOffset,
        moduleXOffset,
        moduleYOffset,
        twistPID,
        drivePID,
        moduleCharacteristics,
        frEncoderInverted,
        frDriveInverted,
        frTwistInverted,
        "Front Right Swerve Module"
    )
    val blConfig = SwerveModuleConfiguration(
        blDrive,
        blTwist,
        conversionFactors,
        blEncoder,
        blOffset,
        -moduleXOffset,
        -moduleYOffset,
        twistPID,
        drivePID,
        moduleCharacteristics,
        blEncoderInverted,
        blDriveInverted,
        blTwistInverted,
        "Back Left Swerve Module"
    )
    val brConfig = SwerveModuleConfiguration(
        brDrive,
        brTwist,
        conversionFactors,
        brEncoder,
        brOffset,
        moduleXOffset,
        -moduleYOffset,
        twistPID,
        drivePID,
        moduleCharacteristics,
        brEncoderInverted,
        brDriveInverted,
        brTwistInverted,
        "Back Right Swerve Module"
    )

    val driveConfig = SwerveDriveConfiguration(
        arrayOf(flConfig, frConfig, blConfig, brConfig),
        imu,
        invertedIMU,
        swervelib.math.SwerveMath.createDriveFeedforward(optimalVoltage, maxSpeedMPS, wheelFriction),
        moduleCharacteristics,
    )

    val controllerConfig = SwerveControllerConfiguration(
        driveConfig,
        headingPID,
        angleJoystickDeadband,
        maxSpeedMPS,
    )
}