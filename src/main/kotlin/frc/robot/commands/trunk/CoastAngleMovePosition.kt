package frc.robot.commands.trunk

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose

class CoastAngleMovePosition(val pose: TrunkPose) : Command() {
    override fun execute() {
        RobotContainer.trunkSystem.io.rotationBrake = false

//        SmartDashboard.putNumber("Coast Angle Move Position", pose.position)
        val elevatorPercent = RobotContainer.trunkSystem.calculatePositionOut(pose.position)
        RobotContainer.trunkSystem.io.setElevatorSpeed(elevatorPercent)
        RobotContainer.trunkSystem.io.setRotationVoltage(0.0)
    }

    override fun isFinished(): Boolean {
        return RobotContainer.trunkSystem.checkAtPosition(pose.position)
    }

    override fun end(interrupted: Boolean) {
        if (interrupted == false) {
            RobotContainer.trunkSystem.isAtPose = true
        }
    }
}