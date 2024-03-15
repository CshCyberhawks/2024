package frc.robot.commands.trunk

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose

class ServoLockElevator : Command() {
    override fun initialize() {
        cancel()
    }

    override fun execute() {
    }

    override fun isFinished(): Boolean {
        return true
    }

    override fun end(interrupted: Boolean) {
    }
}