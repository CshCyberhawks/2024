package frc.robot.commands.trunk

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose

class CalibrateTrunk : Command() {
    override fun initialize() {
        RobotContainer.trunkSystem.io.setElevatorSpeed(0.3)
        RobotContainer.trunkSystem.io.setRotationVoltage(0.0)
    }

    override fun execute() {
    }

    override fun isFinished(): Boolean {
        return RobotContainer.trunkSystem.io.atStowLimit() || RobotContainer.trunkSystem.io.atTopLimit()
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.trunkSystem.io.setZeroPosition(RobotContainer.trunkSystem.io.atTopLimit())
        RobotContainer.trunkSystem.io.setElevatorSpeed(0.0)
        RobotContainer.trunkSystem.lowRotationPIDController.reset(
            RobotContainer.trunkSystem.getThroughboreRotation(),
            0.0
        )
        RobotContainer.trunkSystem.highRotationPIDController.reset(RobotContainer.trunkSystem.getFalconRotation(), 0.0)
        RobotContainer.trunkSystem.climbRotationPIDController.reset(RobotContainer.trunkSystem.getFalconRotation(), 0.0)
        RobotContainer.trunkSystem.elevatorPIDController.reset(RobotContainer.trunkSystem.getPosition(), 0.0)
        //        RobotContainer.trunkSystem.rotationPIDController.reset()

        if (RobotContainer.trunkSystem.io.atTopLimit()) {
            RobotContainer.stateMachine.currentTrunkCommand =
                GoToPoseTrunk(TrunkPose.TOP_STOW).andThen(GoToPoseAndHoldTrunk(TrunkPose.STOW))

        } else {
            RobotContainer.stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.STOW)

        }
    }
}