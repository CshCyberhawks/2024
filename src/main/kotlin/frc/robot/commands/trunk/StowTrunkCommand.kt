package frc.robot.commands.trunk

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose

class StowTrunkCommand: Command() {

    var cmd = GoToPoseAndHoldTrunk(TrunkPose.STOW)
    override fun initialize() {
        cmd = GoToPoseAndHoldTrunk(RobotContainer.stateMachine.stowType.pose)
        RobotContainer.stateMachine.currentTrunkCommand = cmd
    }

    override fun isFinished(): Boolean {
        return cmd.isFinished
    }
}