package frc.robot.subsystems

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.DriveConstants
import frc.robot.constants.PathPlannerLibConstants
import frc.robot.constants.yagsl_configs.YAGSLConfig
import org.littletonrobotics.junction.Logger
import swervelib.SwerveDrive
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import java.io.File
import kotlin.math.abs

class SwerveSystem(private val io: SwerveSystemIO, val swerveDrive: SwerveDrive) : SubsystemBase() {
    private val inputs: SwerveSystemIO.SwerveSystemIOInputs = SwerveSystemIO.SwerveSystemIOInputs

    var inputRotation: Double = 0.0
    private val autoConstraints: PathConstraints

    private val xPID: PIDController = PIDController(.1, 0.0, 0.01)
    private val yPID: PIDController = PIDController(.1, 0.0, 0.01)

    private val PIDDeadzone = .005;

    constructor(io: SwerveSystemIO, config: YAGSLConfig) : this(
        io,
        swerveDrive = SwerveDrive(
            config.driveConfig,
            config.controllerConfig,
            config.maxSpeedMPS,
        )
    )

    constructor(io: SwerveSystemIO, config: File) : this(
        io,
        swerveDrive = try {
            SwerveParser(config).createSwerveDrive(DriveConstants.MAX_SPEED)
        } catch (e: Exception) {
            throw RuntimeException(e)
        }
    )

    init {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.NONE

//        swerveDrive.setHeadingCorrection(false)
        swerveDrive.chassisVelocityCorrection = true
        swerveDrive.setHeadingCorrection(true)
        swerveDrive.setMotorIdleMode(false)
        swerveDrive.pushOffsetsToControllers()
        setupPathPlanner()
        autoConstraints = PathConstraints(
            swerveDrive.maximumVelocity, 4.0,
            swerveDrive.maximumAngularVelocity, Units.degreesToRadians(720.0)
        )

    }

    private fun setupPathPlanner() {
        AutoBuilder.configureHolonomic(
            swerveDrive::getPose,
            swerveDrive::resetOdometry,
            swerveDrive::getRobotVelocity,
            this::autoDrive,
            HolonomicPathFollowerConfig(
                PathPlannerLibConstants.translationPID,
                PIDConstants(
                    swerveDrive.swerveController.thetaController.p,
                    swerveDrive.swerveController.thetaController.i,
                    swerveDrive.swerveController.thetaController.d,
                ),
                swerveDrive.maximumVelocity,
                swerveDrive.swerveDriveConfiguration.driveBaseRadiusMeters,
                PathPlannerLibConstants.replanningConfig,
            ),
            this::isRed,
            this,
        )
    }

    fun drive(translation: Translation2d, rotation: Double, fieldRelative: Boolean) {
        inputRotation = rotation

        var translation = translation;

        xPID.setpoint = translation.x
        yPID.setpoint = translation.y

        val xPIDOut = xPID.calculate(swerveDrive.robotVelocity.vxMetersPerSecond);
        val yPIDOut = yPID.calculate(swerveDrive.robotVelocity.vyMetersPerSecond);

        SmartDashboard.putNumber("xPID", xPIDOut)
        SmartDashboard.putNumber("yPID", yPIDOut)

        if (abs(xPIDOut) > PIDDeadzone || abs(yPIDOut) > PIDDeadzone) {
            if (abs(translation.x) > .05 && abs(translation.y) > .05 && abs(rotation) > .1) {
                translation = Translation2d(translation.x + xPIDOut, translation.y + yPIDOut);
            }
        }

//        swerveDrive.drive(translation, rotation, fieldRelative, false)
        swerveDrive.drive(translation, rotation, true, false)

    }

    fun autoDrive(velocity: ChassisSpeeds) {
        swerveDrive.drive(velocity)
    }

    fun isRed(): Boolean =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("SwerveSystem", inputs)
        Logger.recordOutput("RobotAccel", swerveDrive.accel.orElse(Translation3d(0.0, 0.0, 0.0)))
        Logger.recordOutput("RobotVelocity", swerveDrive.fieldVelocity)
        Logger.recordOutput("RobotRotation", swerveDrive.gyroRotation3d.angle)
        Logger.recordOutput("RobotPose", swerveDrive.pose)
    }

    fun driveToPose(pose: Pose2d): Command {
        return AutoBuilder.pathfindToPose(
            pose,
            autoConstraints,
            0.0,  // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        )
    }
}
