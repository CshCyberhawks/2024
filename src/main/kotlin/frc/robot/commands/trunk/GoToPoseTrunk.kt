package frc.robot.commands.trunk

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose
import frc.robot.constants.TrunkConstants

class GoToPoseTrunk(val desiredPose: TrunkPose) : Command() {
    var currentTargetAngle: Double = TrunkConstants.SAFE_TRAVEL_ANGLE
    var currentTargetPosition: Double = RobotContainer.trunkSystem.getPosition()

    val isAngleSafe: Boolean
        get() = RobotContainer.trunkSystem.getThroughboreRotation() >= TrunkConstants.SAFE_TRAVEL_ANGLE

    val isPositionAlwaysSafe: Boolean
        get() = RobotContainer.trunkSystem.getPosition() >= TrunkConstants.SAFE_PIVOT_POSITION && desiredPose.position >= TrunkConstants.SAFE_PIVOT_POSITION

    override fun initialize() {
        RobotContainer.trunkSystem.brakeMotors()

        RobotContainer.trunkSystem.isAtPose = false
        RobotContainer.trunkSystem.setDesiredRotation(currentTargetAngle)

        currentTargetPosition = RobotContainer.trunkSystem.getPosition()
        currentTargetAngle = TrunkConstants.SAFE_TRAVEL_ANGLE
    }

    override fun execute() {
        SmartDashboard.putNumber("Current Target Angle", currentTargetAngle)


        if (isAngleSafe || isPositionAlwaysSafe) {
            currentTargetAngle = desiredPose.angle
            currentTargetPosition = desiredPose.position
            RobotContainer.trunkSystem.setDesiredRotation(currentTargetAngle)
        }

//        print("is angle safe? $isAngleSafe")
//        print("is position always safe? $isPositionAlwaysSafe")

        val elevatorPercent = RobotContainer.trunkSystem.calculatePositionOut(currentTargetPosition)
        RobotContainer.trunkSystem.io.setElevatorSpeed(elevatorPercent)
//        RobotContainer.trunkSystem.io.setElevatorSpeed(0.0)

        val rotationVolts = RobotContainer.trunkSystem.calculateRotationOut(currentTargetAngle)
        RobotContainer.trunkSystem.io.setRotationVoltage(rotationVolts)
//        RobotContainer.trunkSystem.io.setRotationVoltage(0.0)

    }

    override fun isFinished(): Boolean {
        return RobotContainer.trunkSystem.checkAtPose(
            RobotContainer.trunkSystem.trunkDesiredRotation,
            currentTargetPosition
        )

    }

    override fun end(interrupted: Boolean) {
        if (!interrupted) {
            RobotContainer.trunkSystem.isAtPose = true
        }
        RobotContainer.trunkSystem.io.setRotationVoltage(0.0)
        RobotContainer.trunkSystem.io.setElevatorSpeed(0.0)

    }
}
