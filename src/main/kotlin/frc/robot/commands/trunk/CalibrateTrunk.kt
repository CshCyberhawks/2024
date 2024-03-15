package frc.robot.commands.trunk

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose

class CalibrateTrunk : Command() {
    override fun initialize() {
        RobotContainer.trunkSystem.io.setElevatorSpeed(.2)
    }

    override fun execute() {
    }

    override fun isFinished(): Boolean {
        return RobotContainer.trunkSystem.io.atTopLimit()
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.trunkSystem.io.setZeroPosition()
        RobotContainer.trunkSystem.io.setElevatorSpeed(0.0)
        RobotContainer.trunkSystem.rotationPIDController.reset(RobotContainer.trunkSystem.getRotation(), 0.0)
//        RobotContainer.trunkSystem.rotationPIDController.reset()
        RobotContainer.stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.STOW)
    }
}