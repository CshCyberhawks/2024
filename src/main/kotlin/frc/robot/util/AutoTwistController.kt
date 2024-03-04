package frc.robot.util

import MiscCalculations
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.robot.RobotContainer
import frc.robot.constants.DriveConstants
import frc.robot.constants.PathPlannerLibConstants
import kotlin.math.PI


class AutoTwistController {
    val rotationController: ProfiledPIDController

    var desiredRotation: Rotation2d = Rotation2d(0.0)

    init {
        this.rotationController =
            ProfiledPIDController(
                PathPlannerLibConstants.rotationPID.kP,
                PathPlannerLibConstants.rotationPID.kI,
                PathPlannerLibConstants.rotationPID.kD,
                TrapezoidProfile.Constraints(0.0, 0.0),
                .02
            )
        rotationController.setIntegratorRange(
            -PathPlannerLibConstants.rotationPID.iZone,
            PathPlannerLibConstants.rotationPID.iZone
        )
        rotationController.enableContinuousInput(-PI, PI)
    }

    public fun reset(currentRot: Rotation2d, currentSpeeds: ChassisSpeeds) {
        rotationController.reset(currentRot.radians, currentSpeeds.omegaRadiansPerSecond)
    }


    public fun calculateRotation(targetRotation: Rotation2d): Rotation2d {
        desiredRotation = targetRotation

        val angVelConstraint: Double = DriveConstants.MAX_ANGLE_SPEED
        var maxAngVel = angVelConstraint

        if (java.lang.Double.isFinite(maxAngVel)) {
            // Approximation of available module speed to do rotation with
            maxAngVel = angVelConstraint
        }

        val rotationConstraints =
            TrapezoidProfile.Constraints(
                maxAngVel, DriveConstants.MAX_ANGLE_ACCEL
            )

        val rotationFeedback: Double =
            rotationController.calculate(
                RobotContainer.swerveSystem.getSwervePose().rotation.radians,
                TrapezoidProfile.State(targetRotation.radians, 0.0),
                rotationConstraints
            )
        val rotationFF: Double =
            rotationController.getSetpoint().velocity

        return Rotation2d(rotationFF + rotationFeedback)
    }

    public fun isAtDesired(): Boolean {
        val currentRot = RobotContainer.swerveSystem.getSwervePose().rotation.degrees

        if (MiscCalculations.appxEqual(
                currentRot,
                desiredRotation.degrees,
                DriveConstants.TELEOP_TRANSLATION_AUTOTWIST_DEADZONE
            )
        ) {
            return true;
        }
        return false;
    }
}