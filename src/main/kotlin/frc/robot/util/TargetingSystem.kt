package frc.robot.util

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.util.Units
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

private class TargetingVariables(val robotPose: Pose2d = RobotContainer.swerveSystem.getSwervePose()) {
    val x: Double
    val y: Double
    val vx: Double
    val vy: Double
    val r: Double

    init {
        val robotVelocity = RobotContainer.swerveSystem.driveTrain.currentRobotChassisSpeeds
//        val robotAngle = robotPose.rotation.angle


//        val shootingOffset = TrunkConstants.SHOOTING_OFFSET
//        val xOffset =
//        + cos(robotAngle) * shootingOffset
//        val yOffset = FieldConstants.Speaker.centerSpeakerOpening.y
//        + sin(robotAngle) * shootingOffset

        x = robotPose.x - TargetingConstants.speakerX - TargetingConstants.endpointX
        y = robotPose.y - TargetingConstants.speakerY - TargetingConstants.endpointY
        vx = robotVelocity.vxMetersPerSecond
        vy = robotVelocity.vyMetersPerSecond
        r = sqrt(x * x + y * y)
    }
}

class TargetingSystem {
//    private val g = 9.81

    // overaccount for gravity
    private val g = 11.0

    private val rad2deg = 180.0 / PI

    private val shootingVelocity =
        TargetingConstants.velocityMultiplier * CannonConstants.LEFT_SHOOTER_SHOOT_VELOCITY * TargetingConstants.rpm2ups(
            Units.inchesToMeters(1.5)
        )

//    init {
//        if (CannonConstants.LEFT_SHOOTER_SHOOT_VELOCITY != CannonConstants.RIGHT_SHOOTER_SHOOT_VELOCITY) {
//            println("Shooting with spin, shooter velocity needs to be recalculated")
//        }
//    }

    // dumb scaling dont think about it
    private val shootingVelocityScaling = 1.3

    fun calculateShot(): ShotSetup {
        val shooterVars = TargetingVariables()

        val targetRobotAngle = robotAngleFunction(shooterVars)
        val targetShooterAngle = shooterAngleFunction(shooterVars)

        return ShotSetup(targetRobotAngle, targetShooterAngle)
    }

    fun calculateShootingAngle() = shooterAngleFunction(TargetingVariables())

    fun calculateRobotAngle() = robotAngleFunction(TargetingVariables())

    private fun robotAngleFunction(vars: TargetingVariables): Double {
        return acos(
            vars.x * ((vars.x * vars.x - vars.y * vars.y) * vars.vx + 2 * vars.x * vars.y * vars.vy) /
                    (vars.r.pow(1.5) * sqrt(vars.vx * vars.vx + vars.vy * vars.vy))
        ) * rad2deg
    }

    private fun shooterAngleFunction(vars: TargetingVariables): Double {
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

    fun getShotNoVelocity(): ShotSetup {
        val vars = TargetingVariables()
        Telemetry.putNumber("robot speaker rel pos x", vars.x, RobotContainer.telemetry.trunkTelemetry)
        Telemetry.putNumber("robot speaker rel pos y", vars.y, RobotContainer.telemetry.trunkTelemetry)
        Telemetry.putNumber("robot distance to speaker", vars.r, RobotContainer.telemetry.trunkTelemetry)

        val z = TargetingConstants.endpointZ - TargetingConstants.shooterZ

        val targetRobotAngle = acos(vars.x / vars.r) * rad2deg
        val targetShooterAngle = TargetingConstants.constantStupidConstant + atan(
            (z + (.5 * g * (vars.r.pow(2) + z.pow(2)) / shootingVelocity.pow(2))) / vars.r - TargetingConstants.stupidConstant / shootingVelocity
        ) * rad2deg

        return ShotSetup(targetRobotAngle, targetShooterAngle)
    }

    fun getShotNoVelocityFromPosition(position: Pose2d): ShotSetup {
        val vars = TargetingVariables(position)
        Telemetry.putNumber("robot speaker rel pos x", vars.x, RobotContainer.telemetry.trunkTelemetry)
        Telemetry.putNumber("robot speaker rel pos y", vars.y, RobotContainer.telemetry.trunkTelemetry)
        Telemetry.putNumber("robot distance to speaker", vars.r, RobotContainer.telemetry.trunkTelemetry)

        val z = TargetingConstants.endpointZ - TargetingConstants.shooterZ

        val targetRobotAngle = acos(vars.x / vars.r) * rad2deg
//        val targetShooterAngle = TargetingConstants.constantStupidConstant + atan((z + .5 * g * (vars.r.pow(2) + z.pow(2)) / shootingVelocity.pow(2)) / vars.r - TargetingConstants.stupidConstant/shootingVelocity) * rad2deg
        val targetShooterAngle = TargetingConstants.constantStupidConstant + atan(
            (z + (.5 * g * (vars.r.pow(2) + z.pow(2))) / shootingVelocity.pow(2)) / vars.r
        ) * rad2deg

        return ShotSetup(targetRobotAngle, targetShooterAngle)
    }

    fun test() {
//        println("2.9: " + RobotContainer.targetingSystem.getShotNoVelocityFromPosition(Pose2d(2.9, 5.55, Rotation2d(180.0))).shooterAngle)
//        println("5.37: " + RobotContainer.targetingSystem.getShotNoVelocityFromPosition(Pose2d(5.37, 6.41, Rotation2d(180.0))).shooterAngle)
    }
}
