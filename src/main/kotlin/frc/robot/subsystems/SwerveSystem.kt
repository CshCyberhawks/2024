package frc.robot.subsystems

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.constants.DriveConstants
import frc.robot.constants.PathPlannerLibConstants
import swervelib.SwerveDrive
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import com.pathplanner.lib.auto.AutoBuilder
import java.io.File

class SwerveSystem(directory: File) : SubsystemBase() {
    val swerveDrive: SwerveDrive

    init {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH
        try {
            swerveDrive = SwerveParser(directory).createSwerveDrive(DriveConstants.maxSpeed)
        } catch (e: Exception) {
            throw RuntimeException(e)
        }
        swerveDrive.setHeadingCorrection(false)
            AutoBuilder.configureHolonomic(
            swerveDrive::getPose,
            swerveDrive::resetOdometry, 
            swerveDrive::getRobotVelocity,
            this::autoDrive,
            PathPlannerLibConstants.pathPlannerConfig,
            this::getAlliance,
            this,
            )
    }

    fun drive(translation: Translation2d, rotation: Double, fieldRelative: Boolean) {
        swerveDrive.drive(translation, rotation, fieldRelative, false)
    }

    fun autoDrive(velocity: ChassisSpeeds) {
        swerveDrive.drive(velocity)
    }

    fun getAlliance() : Boolean {
        var alliance = DriverStation.getAlliance()
        if (alliance.isPresent())
            return alliance.get() == DriverStation.Alliance.Red
        return false
    }
}