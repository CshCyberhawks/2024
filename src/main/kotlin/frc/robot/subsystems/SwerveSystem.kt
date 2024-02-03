package frc.robot.subsystems

import KalmanVisionEstimator
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotContainer
import frc.robot.constants.PathPlannerLibConstants
import frc.robot.constants.yagsl_configs.YAGSLConfig
import swervelib.SwerveDrive
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import java.io.File

class SwerveSystem(val swerveDrive: SwerveDrive) : SubsystemBase() {
    private val autoConstraints: PathConstraints

    constructor(config: YAGSLConfig) : this(
        swerveDrive = SwerveDrive(
            config.driveConfig,
            config.controllerConfig,
            config.maxSpeedMPS,
        )
    )

    constructor(config: File, maxSpeedMPS: Double) : this(
        swerveDrive = try {
            SwerveParser(config).createSwerveDrive(maxSpeedMPS)
        } catch (e: Exception) {
            throw RuntimeException(e)
        }
    )

    init {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.NONE

        swerveDrive.setHeadingCorrection(false)
        swerveDrive.setMotorIdleMode(false)
        swerveDrive.pushOffsetsToControllers()
        setupPathPlanner()
        autoConstraints = PathConstraints(
            swerveDrive.maximumVelocity, 4.0,
            swerveDrive.maximumAngularVelocity, Units.degreesToRadians(720.0)
        )
    }

    fun setPos(pos: Pose2d) {
           val odometry_pos =
                swerveDrive.swerveDrivePoseEstimator
                        .getEstimatedPosition()
        swerveDrive.swerveDrivePoseEstimator.resetPosition(odometry_pos.rotation, RobotContainer.swerveSystem.swerveDrive.getModulePositions(), pos)
    }

    fun setPos(pos: Translation2d) {
           val odometry_pos =
                swerveDrive.swerveDrivePoseEstimator
                        .getEstimatedPosition()
        swerveDrive.swerveDrivePoseEstimator.resetPosition(odometry_pos.rotation, RobotContainer.swerveSystem.swerveDrive.getModulePositions(), Pose2d(pos, odometry_pos.rotation))
    }


    fun setupPathPlanner() {
        AutoBuilder.configureHolonomic(
            swerveDrive::getPose,
            swerveDrive::resetOdometry,
            swerveDrive::getRobotVelocity,
            this::autoDrive,
            PathPlannerLibConstants.pathPlannerConfig,
            this::isRed,
            this,
        )
    }

    fun drive(translation: Translation2d, rotation: Double, fieldRelative: Boolean) {
        KalmanVisionEstimator.updateOdometry("limelight-right", "limelight-left", translation.x, translation.y)
        swerveDrive.drive(translation, rotation, fieldRelative, false)
    }

    fun autoDrive(velocity: ChassisSpeeds) {
        swerveDrive.drive(velocity)
    }

    fun isRed(): Boolean {
        val alliance = DriverStation.getAlliance()
        if (alliance.isPresent)
            return alliance.get() == DriverStation.Alliance.Red
        return false
    }

    private fun setupAuto() {
        AutoBuilder.configureHolonomic(
            swerveDrive::getPose,
            swerveDrive::resetOdometry,
            swerveDrive::getRobotVelocity,
            this::autoDrive,
            PathPlannerLibConstants.pathPlannerConfig,
            this::isRed,
            this,
        )
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