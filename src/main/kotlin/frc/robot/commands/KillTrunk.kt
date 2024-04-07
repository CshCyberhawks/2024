package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.RobotContainer
import frc.robot.commands.trunk.CalibrateTrunk

class KillTrunk : Command() {
    override fun initialize() {
        RobotContainer.trunkSystem.io.positionBrake = false
    }

    override fun execute() {
        RobotContainer.stateMachine.currentTrunkCommand = Commands.runOnce({})
        RobotContainer.trunkSystem.io.setElevatorSpeed(0.0)
        RobotContainer.trunkSystem.io.setRotationVoltage(0.0)
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.trunkSystem.io.positionBrake = true
        val cal = CalibrateTrunk()
        cal.schedule()
    }
}
