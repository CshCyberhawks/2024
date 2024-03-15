package frc.robot.commands.trunk

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose

class HoldPoseTrunk(val pose: TrunkPose) : Command() {

    override fun execute() {
        val rotationVolts = RobotContainer.trunkSystem.calculateRotationOut(pose.angle)

        RobotContainer.trunkSystem.io.setRotationVoltage(rotationVolts)

        val elevatorPercent = RobotContainer.trunkSystem.calculatePositionOut(pose.position)

        RobotContainer.trunkSystem.io.setElevatorSpeed(elevatorPercent)
    }

    override fun isFinished(): Boolean {
        return false
    }
}