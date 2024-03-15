package frc.robot.util

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Robot
import frc.robot.RobotContainer
import frc.robot.constants.CannonConstants
import frc.robot.constants.FieldConstants
import frc.robot.constants.TargetingConstants
import frc.robot.constants.TrunkConstants
import kotlin.math.*

data class ShotSetup(var robotAngle: Double, var shooterAngle: Double) {
    init {
        shooterAngle = -shooterAngle + 90
    }
}

class TargetingVariables(robotPose: Pose2d = RobotContainer.swerveSystem.getSwervePose(), robotVelocity: ChassisSpeeds = RobotContainer.swerveSystem.driveTrain.currentRobotChassisSpeeds) {
    val x: Double = robotPose.x - TargetingConstants.speakerX - TargetingConstants.endpointX
    val y: Double = robotPose.y - TargetingConstants.speakerY - TargetingConstants.endpointY
    val vx: Double = robotVelocity.vxMetersPerSecond
    val vy: Double = robotVelocity.vyMetersPerSecond
    val r: Double = sqrt(x * x + y * y)
}

class TargetingSystem {
//    private val g = 9.81
    // overaccount for gravity
    private val g = 11.0

    private val z = TargetingConstants.endpointZ - TargetingConstants.shooterZ

    private val rad2deg = 180.0 / PI

    private val shootingVelocity = TargetingConstants.velocityMultiplier * CannonConstants.LEFT_SHOOTER_SHOOT_VELOCITY * TargetingConstants.rpm2ups(
            Units.inchesToMeters(1.5))

    // dumb scaling dont think about it
    private val shootingVelocityScaling = 1.3

    fun calculateShot(robotPose: Pose2d = RobotContainer.swerveSystem.getSwervePose(), robotVelocity: ChassisSpeeds = RobotContainer.swerveSystem.driveTrain.currentRobotChassisSpeeds): ShotSetup {
        val shooterVars = TargetingVariables(robotPose, robotVelocity)

        val targetRobotAngle = velocityRobotAngleFunction(shooterVars)
        val targetShooterAngle = velocityShooterAngleFunction(shooterVars)

        return ShotSetup(targetRobotAngle, targetShooterAngle)
    }

    fun velocityShooterAngle() = velocityShooterAngleFunction(TargetingVariables())

    fun velocityRobotAngle() = velocityRobotAngleFunction(TargetingVariables())

    private fun velocityRobotAngleFunction(vars: TargetingVariables): Double {
        return acos(
                vars.x * ((vars.x * vars.x - vars.y * vars.y) * vars.vx + 2 * vars.x * vars.y * vars.vy) /
                        (vars.r.pow(1.5) * sqrt(vars.vx * vars.vx + vars.vy * vars.vy))
        ) * rad2deg
    }

    private fun velocityShooterAngleFunction(vars: TargetingVariables): Double {
        val inverseR = 1.0 / vars.r
        val rDot = inverseR * (vars.x * vars.vx + vars.y * vars.vy)
        val h = FieldConstants.Speaker.centerSpeakerOpening.z - TrunkConstants.SHOOTING_HEIGHT

        return atan(
                h * inverseR + g * vars.r /
                        ((shootingVelocityScaling + .15 * rDot) * shootingVelocity * vars.r / (sqrt(vars.r.pow(2) + h.pow(2))) + rDot).pow(
                                2
                        )
        ) * rad2deg +
                (40.0 * rDot) / (shootingVelocityScaling * shootingVelocity)
    }

    fun getShotNoVelocity(
        robotPose: Pose2d = RobotContainer.swerveSystem.getSwervePose(),
        robotVelocity: ChassisSpeeds = RobotContainer.swerveSystem.driveTrain.currentRobotChassisSpeeds): ShotSetup {

        val vars = TargetingVariables(robotPose, robotVelocity)

        Telemetry.putNumber("robot speaker rel pos x", vars.x, RobotContainer.telemetry.trunkTelemetry)
        Telemetry.putNumber("robot speaker rel pos y", vars.y, RobotContainer.telemetry.trunkTelemetry)
        Telemetry.putNumber("robot distance to speaker", vars.r, RobotContainer.telemetry.trunkTelemetry)

        return ShotSetup(noVelocityRobotAngle(vars), noVelocityShooterAngle(vars))
    }

    fun noVelocityRobotAngle(vars: TargetingVariables) = atan2(-vars.y, -vars.x) * rad2deg

    fun noVelocityShooterAngle(vars: TargetingVariables) =
       atan((z + (.5 * g * (vars.r.pow(2) + z.pow(2)) / shootingVelocity.pow(2))) / vars.r) * rad2deg

    fun test(robotPose: Pose2d, robotVelocity: ChassisSpeeds) {
        val vars = TargetingVariables(robotPose, robotVelocity)
        val noVelShot = getShotNoVelocity(robotPose, robotVelocity)
        val velShot = calculateShot(robotPose, robotVelocity)
        println("vars:")
        println(vars)
        println("no velocity:")
        println(noVelShot)
        println("velocity:")
        println(velShot)
    }
}
