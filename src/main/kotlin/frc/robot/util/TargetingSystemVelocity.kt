package frc.robot.util;

import frc.robot.RobotContainer
import frc.robot.constants.FieldConstants
import frc.robot.constants.TrunkConstants
import java.lang.Math
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.sqrt

data class ShotSetupVelocity(var shooterAngle: Double, var velocityRPM: Double)
private data class TargetingVariablesVelocity(val underStage: Boolean) {
    val x: Double
    val y: Double
    val vx: Double
    val vy: Double
    val r: Double
    val rSquared: Double

    init {
        val robotPose = RobotContainer.swerveSystem.getSwervePose()
        val robotVelocity = RobotContainer.swerveSystem.driveTrain.currentRobotChassisSpeeds
        val robotAngle = robotPose.rotation.radians

        val shootingOffset =
            if (underStage) TrunkConstants.UNDER_STAGE_SHOOTING_OFFSET else TrunkConstants.SHOOTING_OFFSET
        val xOffset = FieldConstants.Speaker.centerSpeakerOpening.x
//        + cos(robotAngle) * shootingOffset
        val yOffset = FieldConstants.Speaker.centerSpeakerOpening.y
//        + sin(robotAngle) * shootingOffset

        x = robotPose.x - xOffset
        y = robotPose.y - yOffset
        vx = robotVelocity.vxMetersPerSecond
        vy = robotVelocity.vyMetersPerSecond
        rSquared = x * x + y * y
        r = sqrt(rSquared)
    }
}

class TargetingSystemVelocity {
    private val g = 9.81

    // Add 3 m/s of extra "oomf" speed when entering the shooter
    private val oomfSpeed = 3.0

//    private fun robotAngleFunction(vars: TargetingVariablesVelocity): Double {
//        return acos(
//                vars.x * ((vars.x * vars.x - vars.y * vars.y) * vars.vx + 2 * vars.x * vars.y * vars.vy) /
//                        (vars.r.pow(1.5) * sqrt(vars.vx * vars.vx + vars.vy * vars.vy))
//        ) * rad2deg
//    }

    fun getShotNoVelocity(underStage: Boolean): ShotSetupVelocity {
        val vars = TargetingVariablesVelocity(underStage)
        Telemetry.putNumber("robot speaker rel pos x", vars.x, RobotContainer.telemetry.trunkTelemetry)
        Telemetry.putNumber("robot speaker rel pos y", vars.y, RobotContainer.telemetry.trunkTelemetry)
        Telemetry.putNumber("robot distance to speaker", vars.r, RobotContainer.telemetry.trunkTelemetry)

        val h = FieldConstants.Speaker.centerSpeakerOpening.z - if (vars.underStage)
            TrunkConstants.UNDER_STAGE_SHOOTING_HEIGHT else TrunkConstants.SHOOTING_HEIGHT

        val vy = sqrt(oomfSpeed * oomfSpeed + h * g * 2)
        val t = (vy - oomfSpeed) / g
        val vx = vars.x / t

        // https://www.desmos.com/calculator/cbzt5ry8cx
        val targetShooterAngle = Math.toDegrees(atan2(vy, vx))
        val targetShooterVelocity = sqrt(vy * vy + vx * vx) * ((60) / (0.0762 * PI)) * (27 / 40)

        return ShotSetupVelocity(targetShooterAngle, targetShooterVelocity)
    }
}
