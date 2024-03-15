package frc.robot.commands.trunk

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer

class CoastTrunk: Command() {
    override fun initialize() {
        RobotContainer.trunkSystem.freeMotors()
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.trunkSystem.brakeMotors()
    }
}